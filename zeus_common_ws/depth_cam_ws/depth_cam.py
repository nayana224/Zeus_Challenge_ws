# 튜닝 확인 순서
# 1. HSV -> 2. Red vs Pink 경계 -> 3. 면적계수(min_area 등) -> 4. 일반분리, 강분리 파라미터
import cv2, numpy as np, pyrealsense2 as rs, math, time, argparse, sys, select, json, socket, threading, queue
from time import monotonic as now

# ───────── 0) 네트워크 송신 기본 설정 ─────────
DEST_IP   = "192.168.1.23"
DEST_PORT = 10000
NET_TIMEOUT_S = 1.0
RETRY_COOLDOWN_S = 0.2
PERSISTENT_CONN = False
SEND_MIN_INTERVAL = 0.05

# ───────── 1) HSV 색 범위(기본) ─────────
# 튜닝 필요. 색상별로 H,S,V 하한을 낮춤(조명에 가장 민감한건 s와 v하한값)
COLOR_RANGES = {
    "Red":    [ (np.array([  0, 80, 50], np.uint8), np.array([  3,255,255], np.uint8)),
                (np.array([170,80, 50], np.uint8), np.array([180,255,255], np.uint8)) ],
    "Pink":   [ (np.array([  4, 60, 50], np.uint8), np.array([ 19,255,255], np.uint8)) ],
    "Yellow": [ (np.array([ 20,120, 50], np.uint8), np.array([ 35,255,255], np.uint8)) ],
    "Green":  [ (np.array([ 40, 80, 50], np.uint8), np.array([ 85,255,255], np.uint8)) ],
    "Blue":   [ (np.array([ 88, 60, 50], np.uint8), np.array([118,255,255], np.uint8)) ],
    "Purple": [ (np.array([120, 50, 50], np.uint8), np.array([144,255,255], np.uint8)) ],
}

# ───────── 색상별 분리 파라미터 ─────────
# 튜닝 필요
# dist: 올리면 더 잘게 찢. 내리면 덜 찢
# neck: 크면 하나의 블럭 둘로 나눔. 작으면 약하게 분리
# rm: 크면 얇은 조각 버림(노이즈에 강함), 작으면 얇은 조각 살림(과분할 위험)
PER_COLOR_SPLIT = {
    "Green":  {"dist": 0.50, "neck": 10, "rm": 0.12},
    "Pink":   {"dist": 0.60, "neck": 15, "rm": 0.18},
    "Yellow": {"dist": 0.58, "neck": 13, "rm": 0.14},
    "Blue":   {"dist": 0.55, "neck":  6, "rm": 0.20},
    "Purple": {"dist": 0.51, "neck": 18, "rm": 0.17},
    "Red":    {"dist": 0.75, "neck": 16, "rm": 0.28},
}

AREA_STRONG = 2200
PER_COLOR_SPLIT_STRONG = {
    "Green":  {"dist": 0.50, "neck": 10, "rm": 0.12},  # Green/Yellow은 강분리 사용 안 함
    "Pink":   {"dist": 0.70, "neck": 23, "rm": 0.31},
    "Yellow": {"dist": 0.58, "neck": 13, "rm": 0.14},
    "Blue":   {"dist": 0.95, "neck": 14, "rm": 0.28},
    "Purple": {"dist": 0.95, "neck": 14, "rm": 0.28},
    "Red":    {"dist": 0.75, "neck": 16, "rm": 0.28},
}

# ───────── 1-A) Pink 동적 스위칭용 HSV 프리셋 ─────────
PINK_HSV_INIT = [ (np.array([  4, 60, 50], np.uint8), np.array([ 19,255,255], np.uint8)) ]   # 시작값(기존)
# 튜닝필요. pink가 red로 오검출되면 범위를 더 좁게 수정. pink 못잡으면 pink범위 더 넓게
PINK_HSV_AFTER_RED = [ (np.array([  1, 60, 50], np.uint8), np.array([ 19,255,255], np.uint8)) ]  # Red 사라진 뒤 활성값

def get_active_color_ranges(pink_enabled: bool):
    active = dict(COLOR_RANGES)
    if pink_enabled:
        active["Pink"] = PINK_HSV_AFTER_RED  # Red 소실 후 Pink 켬 -> [1..19]
    else:
        active["Pink"] = []  # Pink 후보 자체를 안 만듦
    return active

# ───────── 2) 바운딩 박스 드로잉 색상 ─────────
DRAW_BGR = {
    "Red":(0,0,255), "Pink":(203,192,255), "Yellow":(0,255,255),
    "Green":(0,255,0), "Blue":(255,0,0), "Purple":(255,0,255),
}

# ───────── 3) 파라미터 ─────────
MIN_AREA           = 400
KERNEL             = np.ones((5,5), np.uint8)
TIMEOUT_MS         = 5000

# 자세(면적) 판정 범위(배타적 비교)
POSTURE_TRUE_MIN = 1300
POSTURE_TRUE_MAX = 2800
def is_posture_true(area: int) -> bool:
    return (area > POSTURE_TRUE_MIN) and (area < POSTURE_TRUE_MAX)

# LOCK 파라미터
LOCK_DIST_PX_MAX   = 60
LOCK_DZ_MM_MAX     = 10.0
LOCK_MISS_MAX      = 5
LOCK_EMA_ALPHA     = 0.35
LABEL_COOLDOWN_FRAMES = 8   # 락 진입 후 8프레임동안 라벨 전환 금지

# 면적 우선 범위
PREF_AREA_MIN = 1650
PREF_AREA_MAX = 2150
def _is_pref_area(a):
    return (PREF_AREA_MIN <= int(a) <= PREF_AREA_MAX)

# ---- Red/Pink 보수적 판별 파라미터 ----
# 튜닝 필요. red 블럭이 자꾸 pink로 바꾸면 CBCR값 올리거나 BR값 올림. 반대로 pink를 red로 바꾸면 값 내림.
CBCR_THRESH = 0.52   # Cb/Cr 임계값(↑할수록 Pink로 덜 뒤집힘)
BR_THRESH   = 0.48   # B/R   임계값(↑할수록 Pink로 덜 뒤집힘)

# LOCK 중 라벨 히스테리시스
VOTES_TO_FLIP_R2P = 4   # Red → Pink
VOTES_TO_FLIP_P2R = 3   # Pink → Red

# ───────── Util: 색 이름 정규화(참고용) ─────────
CANON = {
    "red":"Red","빨강":"Red","빨간색":"Red",
    "pink":"Pink","분홍":"Pink","분홍색":"Pink",
    "yellow":"Yellow","노랑":"Yellow","노란색":"Yellow",
    "green":"Green","초록":"Green","녹색":"Green",
    "blue":"Blue","파랑":"Blue","파란색":"Blue","블루":"Blue",
    "purple":"Purple","보라":"Purple","보라색":"Purple","list":"LIST"
}
def normalize_color_input(s: str):
    if not s: return None
    s = s.strip().lower()
    return CANON.get(s, None)

def parse_initial_args():
    parser = argparse.ArgumentParser(description="RealSense 다색 인식 + '가장 가까운 Z' 자동 선택 + LOCK 추적 + TCP 송신")
    parser.add_argument("--area-min", type=int, default=POSTURE_TRUE_MIN, help="자세 True 하한(배타) 표시용")
    parser.add_argument("--area-max", type=int, default=POSTURE_TRUE_MAX, help="자세 True 상한(배타) 표시용")
    parser.add_argument("--dest-ip", type=str, default=DEST_IP)
    parser.add_argument("--dest-port", type=int, default=DEST_PORT)
    parser.add_argument("--persist", action="store_true")
    return parser.parse_args()

# ───────── 네트워크 송신 클래스 ─────────
class TcpSender:
    def __init__(self, ip, port, timeout=NET_TIMEOUT_S, persistent=False):
        self.ip = ip; self.port = port; self.timeout = timeout
        self.persistent = persistent; self.sock = None
        if self.persistent: self._connect()
    def _connect(self):
        self.close()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.timeout); s.connect((self.ip, self.port)); self.sock = s
    def send_line(self, line_bytes: bytes):
        if not self.persistent:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(self.timeout); s.connect((self.ip, self.port))
                s.sendall(line_bytes)
            except Exception as e:
                print(f"[NET] send error(oneshot): {e}"); time.sleep(RETRY_COOLDOWN_S)
            finally:
                try: s.shutdown(socket.SHUT_RDWR)
                except: pass
                try: s.close()
                except: pass
        else:
            try:
                if self.sock is None: self._connect()
                self.sock.sendall(line_bytes)
            except Exception as e:
                print(f"[NET] send error(persistent): {e}, reconnect한다")
                time.sleep(RETRY_COOLDOWN_S)
                try:
                    self._connect(); self.sock.sendall(line_bytes)
                except Exception as e2:
                    print(f"[NET] resend failed: {e2}")
    def close(self):
        if self.sock is not None:
            try: self.sock.shutdown(socket.SHUT_RDWR)
            except: pass
            try: self.sock.close()
            except: pass
            self.sock = None

#최신큐
class LatestQueue:
    def __init__(self):
        self.q = queue.Queue(maxsize=1); self.lock = threading.Lock()
    def put_latest(self, item):
        with self.lock:
            if self.q.full():
                try: self.q.get_nowait()
                except queue.Empty: pass
            self.q.put_nowait(item)
    def get_or_none(self, timeout=None):
        try: return self.q.get(timeout=timeout)
        except queue.Empty: return None
#송신클래스
class TxWorker(threading.Thread):
    def __init__(self, ip, port, persistent, stop_event: threading.Event, payload_queue: LatestQueue, min_interval=SEND_MIN_INTERVAL):
        super().__init__(daemon=True)
        self.sender = TcpSender(ip, port, timeout=NET_TIMEOUT_S, persistent=persistent)
        self.stop_event = stop_event; self.payload_queue = payload_queue
        self.min_interval = min_interval; self.last_sent_ts = 0.0
    def run(self):
        try:
            while not self.stop_event.is_set():
                item = self.payload_queue.get_or_none(timeout=0.1)
                if item is None: continue
                remaining = self.min_interval - (time.time() - self.last_sent_ts)
                if remaining > 0: self.stop_event.wait(remaining)
                latest = item
                while True and not self.stop_event.is_set():
                    nxt = self.payload_queue.get_or_none(timeout=0.0)
                    if nxt is None: break
                    latest = nxt
                try:
                    line = (json.dumps(latest, ensure_ascii=False) + "\n").encode("utf-8")
                    self.sender.send_line(line); self.last_sent_ts = time.time()
                except Exception as e:
                    print(f"[NET] thread send error: {e}"); time.sleep(RETRY_COOLDOWN_S)
        finally:
            self.sender.close()

# ───────── Util: 마스크 결합 ─────────
def color_mask(hsv, ranges):
    m = None
    for low, high in ranges:
        part = cv2.inRange(hsv, low, high)
        m = part if m is None else (m | part)
    return m

# ───────── 2차 기준: Red vs Pink (보수적) ─────────
def disambiguate_red_vs_pink(hsv_img, bgr_img, contour,
                             cbcr_thresh=CBCR_THRESH, br_thresh=BR_THRESH,
                             min_pixels=40, initial_hint=None, allow_pink=True):
    """
    HSV 겹침(H≈0~20 또는 170~179)에서만 2차 기준으로 Red/Pink 확정한다.
    allow_pink=False이면 Pink 전환을 강제로 금지한다.
    """
    mask = np.zeros(hsv_img.shape[:2], np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)

    if int(cv2.countNonZero(mask)) < min_pixels:
        return (None, 0.0)

    H = hsv_img[...,0][mask>0]
    h_med = float(np.median(H))

    if initial_hint == "Pink" and 7.0 <= h_med <= 22.0:
        return ("Pink", 1.0)
    if initial_hint == "Red" and (h_med <= 6.0 or h_med >= 174.0):
        return ("Red", 1.0)

    ambiguous = (h_med <= 20.0) or (h_med >= 170.0)
    if not ambiguous:
        return (None, 0.0)

    ycrcb = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2YCrCb)
    Cr = ycrcb[...,1][mask>0].astype(np.float32)
    Cb = ycrcb[...,2][mask>0].astype(np.float32)
    cr_med = float(np.median(Cr)) + 1e-6
    cb_med = float(np.median(Cb))
    cbcr   = cb_med / cr_med

    B = bgr_img[...,0][mask>0].astype(np.float32)
    R = bgr_img[...,2][mask>0].astype(np.float32)
    br = float(np.median(B) / (np.median(R) + 1e-6))

    is_pink = (cbcr >= cbcr_thresh) and (br >= br_thresh)

    if not allow_pink:
        return ("Red", 1.0)

    if is_pink:
        s1 = max(0.0, min(1.0, (cbcr - cbcr_thresh) / 0.15))
        s2 = max(0.0, min(1.0, (br   - br_thresh)   / 0.15))
        strength = 0.5 * (s1 + s2)
        return ("Pink", strength)
    else:
        s1 = max(0.0, min(1.0, (cbcr_thresh - cbcr) / 0.20))
        s2 = max(0.0, min(1.0, (br_thresh   - br)   / 0.20))
        strength = 0.5 * (s1 + s2)
        return ("Red", strength)

# ───────── Util: 3D/2D 주성분 각도 ─────────
def angle_3d_deg(contour, depth, deproject_fn, stride=3):
    pts = contour.reshape(-1, 2)
    h, w = depth.shape
    XY = []
    for i in range(0, len(pts), max(1, stride)):
        u, v = int(pts[i][0]), int(pts[i][1])
        if 0 <= v < h and 0 <= u < w:
            d = int(depth[v, u])  # mm
            if d > 0:
                Xmm, Ymm, Zmm = deproject_fn(u, v, d/1000.0)
                XY.append([Xmm, Ymm])
    if len(XY) >= 5:
        XY = np.asarray(XY, dtype=np.float32)
        XY -= XY.mean(axis=0, keepdims=True)
        cov = np.cov(XY.T)
        eigvals, eigvecs = np.linalg.eig(cov)
        major = eigvecs[:, np.argmax(eigvals)]
        ang = math.degrees(math.atan2(major[1], major[0]))
        if ang > 90:  ang -= 180
        if ang <= -90: ang += 180
        return float(ang), True
    ptsf = pts.astype(np.float32)
    if len(ptsf) >= 5:
        ptsf -= ptsf.mean(axis=0, keepdims=True)
        cov2  = np.cov(ptsf.T)
        e2, U = np.linalg.eig(cov2)
        major2 = U[:, np.argmax(e2)]
        ang2 = math.degrees(math.atan2(major2[1], major2[0]))
        if ang2 > 90:  ang2 -= 180
        if ang2 <= -90: ang2 += 180
        return float(ang2), False
    return None, False

# ───────── 일반 물체 분리 ─────────
def split_touching_basic(mask_bin, color_bgr, min_area=MIN_AREA,
                         dist_thresh_rel=0.45, neck_cut=True, neck_k=7, rm_thin_ratio=0.12):
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    opened = cv2.morphologyEx(mask_bin, cv2.MORPH_OPEN, k3, iterations=1)

    cut = opened.copy()
    if neck_cut:
        kv = cv2.getStructuringElement(cv2.MORPH_RECT, (1, neck_k))
        kh = cv2.getStructuringElement(cv2.MORPH_RECT, (neck_k, 1))
        cut_v = cv2.morphologyEx(cut, cv2.MORPH_OPEN, kv, iterations=1)
        cut_h = cv2.morphologyEx(cut, cv2.MORPH_OPEN, kh, iterations=1)
        cut   = cv2.bitwise_or(cut_v, cut_h)

    dist = cv2.distanceTransform(cut, cv2.DIST_L2, 5)
    if dist.max() > 0:
        _, peaks = cv2.threshold(dist, dist_thresh_rel * dist.max(), 255, cv2.THRESH_BINARY)
    else:
        peaks = np.zeros_like(dist, dtype=np.uint8)
    peaks = peaks.astype(np.uint8)

    d8 = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    local_max = cv2.dilate(dist, d8)
    peaks2 = np.where((dist == local_max) & (dist > 0), 255, 0).astype(np.uint8)
    sure_fg = cv2.bitwise_or(peaks, peaks2)

    sure_bg = cv2.dilate(cut, k3, iterations=2)
    unknown = cv2.subtract(sure_bg, sure_fg)

    num, markers = cv2.connectedComponents(sure_fg)
    markers = markers + 1
    markers[unknown == 255] = 0

    ws_input = color_bgr.copy()
    markers = cv2.watershed(ws_input, markers)

    contours = []
    max_label = markers.max()
    for label in range(2, max_label + 1):
        region = np.uint8(markers == label) * 255
        if cv2.countNonZero(region) < min_area:
            continue
        x, y, w, h = cv2.boundingRect(region)
        rect_area = w * h
        obj_area  = int(cv2.countNonZero(region))
        if rect_area <= 0:
            continue
        solidity_like = obj_area / float(rect_area)
        if solidity_like < rm_thin_ratio:
            continue
        cs, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours.extend(cs)
    return contours

# ───────── 붙은 물체 강제 분리 ─────────
def split_touching_strong(mask_bin, color_bgr, min_area=MIN_AREA,
                          dist_thresh_rel=0.45, neck_cut=True, neck_k=7, rm_thin_ratio=0.12):
    def _run(mask, dist_rel, base_neck_k, add_corner_seeds=True):
        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k3, iterations=1)

        # 적응형 neck
        if cv2.countNonZero(opened) > 0:
            x, y, w, h = cv2.boundingRect(opened)
            adapt = int(max(base_neck_k, math.ceil(min(w, h) * 0.18)))
            if adapt % 2 == 0:
                adapt += 1
        else:
            adapt = base_neck_k

        cut = opened.copy()
        if neck_cut:
            kv = cv2.getStructuringElement(cv2.MORPH_RECT, (1, adapt))
            kh = cv2.getStructuringElement(cv2.MORPH_RECT, (adapt, 1))
            cut_v = cv2.morphologyEx(cut, cv2.MORPH_OPEN, kv, iterations=1)
            cut_h = cv2.morphologyEx(cut, cv2.MORPH_OPEN, kh, iterations=1)
            cut   = cv2.bitwise_or(cut_v, cut_h)

        dist = cv2.distanceTransform(cut, cv2.DIST_L2, 5)
        peaks = np.zeros_like(dist, dtype=np.uint8)
        if dist.max() > 0:
            _, peaks = cv2.threshold(dist, dist_rel * dist.max(), 255, cv2.THRESH_BINARY)
        peaks = peaks.astype(np.uint8)

        d8 = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        local_max = cv2.dilate(dist, d8)
        peaks2 = np.where((dist == local_max) & (dist > 0), 255, 0).astype(np.uint8)
        sure_fg = cv2.bitwise_or(peaks, peaks2)

        # 코너 시드 추가
        if add_corner_seeds:
            try:
                corners = cv2.goodFeaturesToTrack((cut>0).astype(np.uint8)*255,
                                                  maxCorners=6, qualityLevel=0.01, minDistance=10)
                if corners is not None:
                    corner_seed = np.zeros_like(sure_fg)
                    for pt in corners:
                        cx, cy = int(pt[0][0]), int(pt[0][1])
                        cv2.circle(corner_seed, (cx, cy), 3, 255, -1)
                    corner_seed = cv2.dilate(corner_seed, k3, iterations=1)
                    sure_fg = cv2.bitwise_or(sure_fg, corner_seed)
            except Exception:
                pass

        sure_bg = cv2.dilate(cut, k3, iterations=2)
        unknown = cv2.subtract(sure_bg, sure_fg)
        _, markers = cv2.connectedComponents((sure_fg>0).astype(np.uint8))
        markers = markers + 1
        markers[unknown == 255] = 0

        ws_input = color_bgr.copy()
        markers = cv2.watershed(ws_input, markers)

        contours = []
        max_label = markers.max()
        for label in range(2, max_label + 1):
            region = np.uint8(markers == label) * 255
            if cv2.countNonZero(region) < min_area:
                continue
            rx, ry, rw, rh = cv2.boundingRect(region)
            rect_area = rw * rh
            obj_area  = int(cv2.countNonZero(region))
            if rect_area <= 0:
                continue
            solidity_like = obj_area / float(rect_area)
            if solidity_like < rm_thin_ratio:
                continue
            cs, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours.extend(cs)
        return contours

    contours = _run(mask_bin, dist_thresh_rel, neck_k, add_corner_seeds=True)
    if len(contours) <= 1:
        strong_dist = min(0.98, dist_thresh_rel + 0.20)
        strong_neck = max(neck_k + 10, int(neck_k * 1.5))
        contours = _run(mask_bin, strong_dist, strong_neck, add_corner_seeds=True)
    return contours

# ───────── 깊이 유틸리티 ─────────
def depth_at(cx, cy, depth, k=3):
    h, w = depth.shape
    x0, x1 = max(cx-k, 0), min(cx+k+1, w)
    y0, y1 = max(cy-k, 0), min(cy+k+1, h)
    roi = depth[y0:y1, x0:x1]
    valid = roi[roi > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid))

# ───────── 후보 수집(기본) ─────────
def collect_candidates_all_colors_basic(hsv, color_bgr, depth, min_area=MIN_AREA,
                                        color_ranges=None, allow_pink=True):
    if color_ranges is None:
        color_ranges = COLOR_RANGES
    out = []
    for cname, ranges in color_ranges.items():
        if len(ranges) == 0:
            continue
        mask = color_mask(hsv, ranges)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)

        p = PER_COLOR_SPLIT.get(cname, {"dist":0.50, "neck":9, "rm":0.12})
        dist_rel = float(p["dist"]); neck_k = int(p["neck"]); rm_ratio = float(p["rm"])

        cnts = split_touching_basic(mask, color_bgr=color_bgr, min_area=min_area,
                                    dist_thresh_rel=dist_rel, neck_cut=True,
                                    neck_k=neck_k, rm_thin_ratio=rm_ratio)
        for c in cnts:
            area = int(cv2.contourArea(c))
            if area < min_area: continue
            M = cv2.moments(c)
            if M["m00"] <= 1e-5: continue
            cx = int(M["m10"]/M["m00"]); cy = int(M["m01"]/M["m00"])
            z_mm = depth_at(cx, cy, depth, k=3)
            if z_mm is None: continue
            x, y, ww, hh = cv2.boundingRect(c)
            posture_true = is_posture_true(area)

            label = cname
            if label in ("Red", "Pink"):
                decided_label, _ = disambiguate_red_vs_pink(
                    hsv, color_bgr, c, initial_hint=label, allow_pink=allow_pink
                )
                if decided_label is not None:
                    label = decided_label

            out.append({
                "color": label,
                "c": c, "cx": cx, "cy": cy,
                "x": x, "y": y, "w": ww, "h": hh,
                "cnt_area": area, "posture_true": posture_true,
                "z_mm": z_mm,
            })
    return out

# ───────── 단일 색상 재수집(강/기본 선택 가능) ─────────
def collect_candidates_for_one_color(hsv, color_bgr, depth, cname, min_area=MIN_AREA,
                                     strong=False, color_ranges=None, allow_pink=True):
    if color_ranges is None:
        color_ranges = COLOR_RANGES

    if cname in ("Green", "Yellow"):
        strong = False

    ranges = color_ranges.get(cname, [])
    if len(ranges) == 0:
        return []

    mask = color_mask(hsv, ranges)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)

    if strong:
        p = PER_COLOR_SPLIT_STRONG.get(cname, {"dist":0.50, "neck":9, "rm":0.12})
        split_fn = split_touching_strong
    else:
        p = PER_COLOR_SPLIT.get(cname, {"dist":0.50, "neck":9, "rm":0.12})
        split_fn = split_touching_basic

    dist_rel = float(p["dist"]); neck_k = int(p["neck"]); rm_ratio = float(p["rm"])

    cnts = split_fn(mask, color_bgr=color_bgr, min_area=min_area,
                    dist_thresh_rel=dist_rel, neck_cut=True,
                    neck_k=neck_k, rm_thin_ratio=rm_ratio)

    out = []
    for c in cnts:
        area = int(cv2.contourArea(c))
        if area < min_area: continue
        M = cv2.moments(c)
        if M["m00"] <= 1e-5: continue
        cx = int(M["m10"]/M["m00"]); cy = int(M["m01"]/M["m00"])
        z_mm = depth_at(cx, cy, depth, k=3)
        if z_mm is None: continue
        x, y, ww, hh = cv2.boundingRect(c)
        posture_true = is_posture_true(area)

        label = cname
        if label in ("Red", "Pink"):
            decided_label, _ = disambiguate_red_vs_pink(
                hsv, color_bgr, c, initial_hint=label, allow_pink=allow_pink
            )
            if decided_label is not None:
                label = decided_label

        out.append({
            "color": label,
            "c": c, "cx": cx, "cy": cy,
            "x": x, "y": y, "w": ww, "h": hh,
            "cnt_area": area, "posture_true": posture_true,
            "z_mm": z_mm,
        })
    return out

# ───────── 미리 선택(LOCK 규칙) + 면적 우선 ─────────
def pick_candidate(candidates, lock):
    if not candidates:
        return None

    pref = [c for c in candidates if _is_pref_area(c.get("cnt_area", 0))]
    pool = pref if len(pref) > 0 else candidates

    if lock.active:
        best_d = None
        for info in pool:
            if info["color"] != lock.color:
                continue
            dx = info["cx"] - (lock.cx if lock.cx is not None else info["cx"])
            dy = info["cy"] - (lock.cy if lock.cy is not None else info["cy"])
            dpx = math.hypot(dx, dy)
            dz  = abs(info["z_mm"] - (lock.z_mm if lock.z_mm is not None else info["z_mm"]))
            if dpx <= LOCK_DIST_PX_MAX and dz <= LOCK_DZ_MM_MAX:
                key = (0 if _is_pref_area(info["cnt_area"]) else 1, dpx, dz)
                if best_d is None or key < best_d[0]:
                    best_d = (key, info)
        return best_d[1] if best_d is not None else None
    else:
        best = None
        for info in pool:
            key = (0 if _is_pref_area(info["cnt_area"]) else 1, info["z_mm"])
            if best is None or key < best[0]:
                best = (key, info)
        return best[1] if best is not None else None

# ───────── LOCK 상태 클래스 ─────────
class LockState:
    def __init__(self):
        self.active = False
        self.color = None
        self.cx = None
        self.cy = None
        self.z_mm = None
        self.miss = 0
        self.angle_deg = None
        self.center_mm = None

        # 안정/디버그용
        self.stable_frames = 0
        self.prev_cx = None
        self.prev_cy = None

        # 라벨 히스테리시스 투표
        self.vote_r2p = 0  # Red → Pink
        self.vote_p2r = 0  # Pink → Red

        # 시각화용 스무딩된 bbox/center
        self.sm_cx = None
        self.sm_cy = None
        self.sm_ang = None
        self.sm_bx = None
        self.sm_by = None
        self.sm_bw = None
        self.sm_bh = None

    def reset(self):
        self.__init__()

def ema(prev, cur, alpha):
    return cur if prev is None else (alpha*cur + (1.0-alpha)*prev)

# 안정 판정(표시용)
COUNT_STABLE_FRAMES = 10
STABLE_MOVE_PX      = 6

# ───────── 메인 ─────────
def main():
    args = parse_initial_args()
    dest_ip, dest_port, persist = args.dest_ip, int(args.dest_port), bool(args.persist)

    print(f"[INIT] 자세 True 범위(배타): ({POSTURE_TRUE_MIN}, {POSTURE_TRUE_MAX}) px^2")
    print(f"[NET ] DEST={dest_ip}:{dest_port}  PERSIST={persist}")
    print("[MODE] AUTO+LOCK: Z(min) 우선 + 면적우선, 화면 중심 무관 동작한다")
    print(f"[PREF] 면적 우선 범위={PREF_AREA_MIN}~{PREF_AREA_MAX} px^2")

    stop_event = threading.Event()
    payload_queue = LatestQueue()
    tx_thread = TxWorker(dest_ip, dest_port, persist, stop_event, payload_queue, min_interval=SEND_MIN_INTERVAL)
    tx_thread.start()

    pipeline = rs.pipeline(); config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile  = pipeline.start(config)
    align    = rs.align(rs.stream.color)
    color_intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    def deproject_mm(u, v, depth_m):
        X, Y, Z = rs.rs2_deproject_pixel_to_point(color_intr, [float(u), float(v)], float(depth_m))
        return X*1000.0, Y*1000.0, Z*1000.0

    lock = LockState()

    # ───── Pink 동적 상태 ─────
    pink_enabled = False
    red_absent_streak = 0
    RED_GONE_FRAMES = 12  # 연속 12프레임 동안 Red 0개면 Pink 활성화

    try:
        while True:
            frames = pipeline.wait_for_frames(TIMEOUT_MS)
            frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())  # mm
            hsv   = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

            # 활성 색 범위 구성(Pink on/off 반영)
            active_ranges = get_active_color_ranges(pink_enabled)

            # 1) 전 색상 '기본 분리'로 후보 수집
            candidates = collect_candidates_all_colors_basic(
                hsv, color, depth, min_area=MIN_AREA,
                color_ranges=active_ranges, allow_pink=pink_enabled
            )

            # Red 존재 여부로 상태 갱신
            red_present = any(c["color"] == "Red" for c in candidates)
            if red_present:
                red_absent_streak = 0
            else:
                red_absent_streak += 1
                if (not pink_enabled) and red_absent_streak >= RED_GONE_FRAMES:
                    pink_enabled = True
                    print("[MODE] Red 소실 감지 → Pink 인식 활성화(HSV=[1..19])")

            # 2) 현재 LOCK 규칙 + 면적우선으로 미리 선택
            pre_chosen = pick_candidate(candidates, lock)

            # 3) 큰 면적(>= AREA_STRONG)이고 Green/Yellow이 아니면 강분리 재수집
            if (pre_chosen is not None and
                pre_chosen["cnt_area"] >= AREA_STRONG and
                pre_chosen["color"] not in ("Green", "Yellow")):
                candidates = [c for c in candidates if c["color"] != pre_chosen["color"]]
                candidates += collect_candidates_for_one_color(
                    hsv, color, depth,
                    cname=pre_chosen["color"], min_area=MIN_AREA, strong=True,
                    color_ranges=active_ranges, allow_pink=pink_enabled
                )

            # 디버그 컨투어 캔버스
            contours_debug = np.zeros_like(color)
            for info in candidates:
                cname_dbg = info["color"]
                c = info["c"]
                x, y, w, h = info["x"], info["y"], info["w"], info["h"]
                cx, cy = info["cx"], info["cy"]
                cv2.drawContours(contours_debug, [c], -1, DRAW_BGR[cname_dbg], 2)
                cv2.rectangle(contours_debug, (x, y), (x+w, y+h), DRAW_BGR[cname_dbg], 1)
                cv2.circle(contours_debug, (cx, cy), 3, DRAW_BGR[cname_dbg], -1)
                tag = f"{cname_dbg}{' *' if _is_pref_area(info['cnt_area']) else ''}"
                cv2.putText(contours_debug, tag, (x, max(0, y-6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, DRAW_BGR[cname_dbg], 1)

            # 최종 선택 & LOCK 처리
            chosen = None
            if lock.active:
                gate_pool = []
                for info in candidates:
                    if info["color"] != lock.color:
                        continue
                    dx = info["cx"] - lock.cx
                    dy = info["cy"] - lock.cy
                    dpx = math.hypot(dx, dy)
                    dz  = abs(info["z_mm"] - (lock.z_mm if lock.z_mm is not None else info["z_mm"]))
                    if dpx <= LOCK_DIST_PX_MAX and dz <= LOCK_DZ_MM_MAX:
                        gate_pool.append((dpx, dz, info))
                if gate_pool:
                    pref_pool = [g for g in gate_pool if _is_pref_area(g[2]["cnt_area"])]
                    pool = pref_pool if pref_pool else gate_pool
                    pool.sort(key=lambda t: (0 if _is_pref_area(t[2]["cnt_area"]) else 1, t[0], t[1]))
                    chosen = pool[0][2]
                    lock.miss = 0
                else:
                    lock.miss += 1
                    if lock.miss > LOCK_MISS_MAX:
                        lock.reset()
            else:
                if candidates:
                    pref_pool = [c for c in candidates if _is_pref_area(c["cnt_area"])]
                    pool = pref_pool if pref_pool else candidates
                    pool.sort(key=lambda c: (0 if _is_pref_area(c["cnt_area"]) else 1, c["z_mm"]))
                    chosen = pool[0]
                    # 새 LOCK 진입
                    lock.active = True
                    lock.color  = chosen["color"]
                    lock.cx     = chosen["cx"]
                    lock.cy     = chosen["cy"]
                    lock.z_mm   = chosen["z_mm"]
                    lock.miss   = 0
                    lock.stable_frames = 0
                    lock.prev_cx, lock.prev_cy = lock.cx, lock.cy

                    # bbox/center 스무딩 초기화
                    lock.sm_cx = float(lock.cx)
                    lock.sm_cy = float(lock.cy)
                    lock.sm_ang = None
                    lock.sm_bx = float(chosen["x"])
                    lock.sm_by = float(chosen["y"])
                    lock.sm_bw = float(chosen["w"])
                    lock.sm_bh = float(chosen["h"])

            # 선택/시각화/송신
            if chosen is not None:
                cname = chosen["color"]
                cx, cy = chosen["cx"], chosen["cy"]
                x, y, ww, hh = chosen["x"], chosen["y"], chosen["w"], chosen["h"]
                cnt_area = chosen["cnt_area"]
                posture_true = chosen["posture_true"]
                z_mm = chosen["z_mm"]

                # 안정 프레임 누적(표시용)
                if lock.prev_cx is None or lock.prev_cy is None:
                    lock.stable_frames = 1
                else:
                    move = math.hypot(cx - lock.prev_cx, cy - lock.prev_cy)
                    if move <= STABLE_MOVE_PX:
                        lock.stable_frames += 1
                    else:
                        lock.stable_frames = 1
                lock.prev_cx, lock.prev_cy = cx, cy

                # ---- 최종 라벨 확정(보수적) + 히스테리시스 ----
                decided_label, strength = disambiguate_red_vs_pink(
                    hsv, color, chosen["c"], initial_hint=cname, allow_pink=pink_enabled
                )
                if lock.active and lock.stable_frames < LABEL_COOLDOWN_FRAMES:
                    decided_label = None
                if decided_label is not None:
                    if lock.active and lock.color in ("Red", "Pink"):
                        if lock.color == "Red":
                            if decided_label == "Pink":
                                lock.vote_r2p += 1; lock.vote_p2r = 0
                            else:
                                lock.vote_p2r += 1; lock.vote_r2p = 0
                            if lock.vote_r2p >= VOTES_TO_FLIP_R2P:
                                cname = "Pink"; lock.vote_r2p = 0
                            else:
                                cname = "Red"
                        else:  # lock.color == "Pink"
                            if decided_label == "Red":
                                lock.vote_p2r += 1; lock.vote_r2p = 0
                            else:
                                lock.vote_r2p += 1; lock.vote_p2r = 0
                            if lock.vote_p2r >= VOTES_TO_FLIP_P2R:
                                cname = "Red"; lock.vote_p2r = 0
                            else:
                                cname = "Pink"
                    else:
                        # lock 전: Red 우선 유지, Pink는 충분히 강할 때만
                        cname = decided_label if decided_label == "Red" else ("Pink" if strength >= 0.5 and pink_enabled else "Red")

                # ── EMA 기반 시각화 좌표/박스 안정화 ──
                lock.sm_cx = ema(lock.sm_cx, cx, LOCK_EMA_ALPHA)
                lock.sm_cy = ema(lock.sm_cy, cy, LOCK_EMA_ALPHA)

                # bbox도 부드럽게
                lock.sm_bx = ema(lock.sm_bx, x,  LOCK_EMA_ALPHA)
                lock.sm_by = ema(lock.sm_by, y,  LOCK_EMA_ALPHA)
                lock.sm_bw = ema(lock.sm_bw, ww, LOCK_EMA_ALPHA)
                lock.sm_bh = ema(lock.sm_bh, hh, LOCK_EMA_ALPHA)

                sm_cx_i = int(round(lock.sm_cx))
                sm_cy_i = int(round(lock.sm_cy))
                sm_bx_i = int(round(lock.sm_bx))
                sm_by_i = int(round(lock.sm_by))
                sm_bw_i = int(round(lock.sm_bw))
                sm_bh_i = int(round(lock.sm_bh))

                Xmm, Ymm, Zmm = deproject_mm(cx, cy, z_mm/1000.0)
                center_mm = (round(Xmm,1), round(Ymm,1), round(Zmm,1))
                ang_deg, ok3d = angle_3d_deg(chosen["c"], depth, deproject_mm, stride=3)
                if ang_deg is not None:
                    lock.sm_ang = ema(lock.sm_ang, ang_deg, LOCK_EMA_ALPHA)

                # 시각화
                cv2.rectangle(color, (sm_bx_i, sm_by_i), (sm_bx_i+sm_bw_i, sm_by_i+sm_bh_i), DRAW_BGR[cname], 2)
                cv2.circle(color, (sm_cx_i, sm_cy_i), 6, DRAW_BGR[cname], -1)
                if lock.sm_ang is not None:
                    L = max(ww, hh) // 2 + 20
                    rad = math.radians(lock.sm_ang)
                    ex = int(sm_cx_i + L * math.cos(rad))
                    ey = int(sm_cy_i + L * math.sin(rad))
                    cv2.arrowedLine(color, (sm_cx_i, sm_cy_i), (ex, ey), DRAW_BGR[cname], 2, tipLength=0.25)

                label_txt = (f"{cname}{' *' if _is_pref_area(cnt_area) else ''} "
                             f"[pose={str(posture_true)}] "
                             f"cntA={cnt_area}  3D(mm)=({Xmm:.1f},{Ymm:.1f},{Zmm:.1f})  "
                             f"{'LOCK' if lock.active else 'FREE'}  "
                             f"stable={lock.stable_frames}/{COUNT_STABLE_FRAMES}")
                cv2.putText(color, label_txt, (sm_bx_i, max(0, sm_by_i-8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, DRAW_BGR[cname], 2)

                # LOCK 상태 업데이트
                if lock.active:
                    lock.color = cname
                    lock.cx, lock.cy = cx, cy
                    lock.z_mm = z_mm
                    lock.center_mm = center_mm
                    lock.angle_deg = float(lock.sm_ang) if lock.sm_ang is not None else (float(ang_deg) if ang_deg is not None else None)

                # 송신
                payload = {
                    "color": cname,     # 색깔
                    "pose": bool(posture_true),     # True / False
                    "center_mm": {"x": float(center_mm[0]), "y": float(center_mm[1]), "z": float(center_mm[2])},    # 중심 좌표
                    "angle_deg": lock.angle_deg if lock.active else None,   # 각도
                }
                payload_queue.put_latest(payload)

            # HUD
            hud = (f"AUTO+LOCK  TRUE-AREA(excl)=({POSTURE_TRUE_MIN},{POSTURE_TRUE_MAX}) "
                   f"PREF={PREF_AREA_MIN}-{PREF_AREA_MAX}px^2")
            hud += f"  STATE={'LOCKED' if lock.active else 'UNLOCK'}"
            hud += f"  | Pink={'ON[1..19]' if pink_enabled else 'OFF'}"
            if lock.active:
                hud += f"  miss={lock.miss}/{LOCK_MISS_MAX}  color={lock.color}"
                hud += f"  votes(R2P={lock.vote_r2p},P2R={lock.vote_p2r})"
            cv2.putText(color, hud, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255,255,255), 2)

            # ───── 두 창 표시 ─────
            cv2.imshow("color", color)
            cv2.imshow("contours_debug", contours_debug)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break

    finally:
        try: pipeline.stop()
        except Exception: pass
        cv2.destroyAllWindows()
        stop_event.set(); tx_thread.join(timeout=1.0)

if __name__ == "__main__":
    main()
