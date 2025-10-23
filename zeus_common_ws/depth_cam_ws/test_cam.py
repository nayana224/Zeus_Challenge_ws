# d435_multi_color_center_3d_angleXYZ_single_live_once_tx_threaded_autominZ_lock_nocontourwin_hpV_TOP.py
import cv2, numpy as np, pyrealsense2 as rs, math, time, argparse, sys, select, json, socket, threading, queue
from time import monotonic as now

# ───────── 0) 네트워크 송신 기본 설정 ─────────
DEST_IP   = "192.168.1.23"
DEST_PORT = 10000
NET_TIMEOUT_S = 1.0
RETRY_COOLDOWN_S = 0.2
PERSISTENT_CONN = False
SEND_MIN_INTERVAL = 0.05

# ───────── 과분할(끊김) 완화 전역 스위치 ─────────
USE_STRONG_SPLIT = False   # 강분리(watershed 가혹 모드) 재수집 비활성화

# ───────── Top-Layer(최상층) 강제 옵션 ─────────
TOP_ONLY   = True     # 항상 최상층만 허용
TOP_EPS_MM = 6.0      # 최상층 밴드 허용 오차(±mm)

# ───────── 1) HSV 색 범위 ─────────
COLOR_RANGES = {
    "Red":    [ (np.array([  0,120, 50], np.uint8), np.array([  4,255,255], np.uint8)),
                (np.array([170,120, 50], np.uint8), np.array([180,255,255], np.uint8)) ],
    "Pink":   [ (np.array([  4, 60, 50], np.uint8), np.array([ 20,255,255], np.uint8)) ],
    "Yellow": [ (np.array([ 20,120, 50], np.uint8), np.array([ 35,255,255], np.uint8)) ],
    "Green":  [ (np.array([ 40, 80, 50], np.uint8), np.array([ 85,255,255], np.uint8)) ],
    "Blue":   [ (np.array([ 88, 60, 50], np.uint8), np.array([125,255,255], np.uint8)) ],
    "Purple": [ (np.array([120, 50, 50], np.uint8), np.array([160,255,255], np.uint8)) ],
}

# ───────── 색상별 분리 파라미터 ─────────
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
    "Green":  {"dist": 0.50, "neck": 10, "rm": 0.12},
    "Pink":   {"dist": 0.70, "neck": 23, "rm": 0.31},
    "Yellow": {"dist": 0.58, "neck": 13, "rm": 0.14},
    "Blue":   {"dist": 0.95, "neck": 14, "rm": 0.28},
    "Purple": {"dist": 0.95, "neck": 14, "rm": 0.28},
    "Red":    {"dist": 0.75, "neck": 16, "rm": 0.28},
}

# ───────── 2) 바운딩 박스 드로잉 색상 ─────────
DRAW_BGR = {
    "Red":(0,0,255), "Pink":(203,192,255), "Yellow":(0,255,255),
    "Green":(0,255,0), "Blue":(255,0,0), "Purple":(255,0,255),
}

# ───────── 3) 파라미터 ─────────
MIN_AREA           = 400
KERNEL             = np.ones((5,5), np.uint8)
TIMEOUT_MS         = 5000

# ───────── 자세(면적) 판정 범위(배타적 비교) ─────────
POSTURE_TRUE_MIN = 1300   # area > 1300
POSTURE_TRUE_MAX = 3300   # area < 3300
def is_posture_true(area: int) -> bool:
    return (area > POSTURE_TRUE_MIN) and (area < POSTURE_TRUE_MAX)

# ───────── LOCK 파라미터 ─────────
LOCK_DIST_PX_MAX   = 60
LOCK_DZ_MM_MAX     = 30.0
LOCK_MISS_MAX      = 5
LOCK_EMA_ALPHA     = 0.35

# ───────── 면적 우선순위 ─────────
PREF_AREA_MIN = 1650
PREF_AREA_MAX = 2150
def _is_pref_area(a):
    return (PREF_AREA_MIN <= int(a) <= PREF_AREA_MAX)

# ───────── Util: 색 이름 정규화(미사용, 남겨둠) ─────────
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

# ───────── 네트워크 송신 보일러플레이트 ─────────
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

# ───────── 일반 물체 분리 (끊김 완화 버전) ─────────
def split_touching_basic(mask_bin, color_bgr, min_area=MIN_AREA,
                         dist_thresh_rel=0.60,
                         neck_cut=False,
                         neck_k=7, rm_thin_ratio=0.12):
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    closed = cv2.morphologyEx(mask_bin, cv2.MORPH_CLOSE, k3, iterations=1)
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, k3, iterations=1)
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

    sure_fg = peaks
    sure_bg = cv2.dilate(cut, k3, iterations=2)
    unknown = cv2.subtract(sure_bg, sure_fg)

    _, markers = cv2.connectedComponents(sure_fg)
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

# ───────── 붙은 물체 강제 분리 (코너 시드 OFF) ─────────
def split_touching_strong(mask_bin, color_bgr, min_area=MIN_AREA,
                          dist_thresh_rel=0.45, neck_cut=True, neck_k=7, rm_thin_ratio=0.12):
    def _run(mask, dist_rel, base_neck_k, add_corner_seeds=False):
        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k3, iterations=1)
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

    contours = _run(mask_bin, dist_thresh_rel, neck_k, add_corner_seeds=False)
    if len(contours) <= 1:
        strong_dist = min(0.98, dist_thresh_rel + 0.20)
        strong_neck = max(neck_k + 10, int(neck_k * 1.5))
        contours = _run(mask_bin, strong_dist, strong_neck, add_corner_seeds=False)
    return contours

# ───────── 깊이 유틸 ─────────
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
def collect_candidates_all_colors_basic(hsv, color_bgr, depth, min_area=MIN_AREA):
    out = []
    for cname, ranges in COLOR_RANGES.items():
        mask = color_mask(hsv, ranges)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)

        p = PER_COLOR_SPLIT.get(cname, {"dist":0.50, "neck":9, "rm":0.12})
        dist_rel = float(p["dist"]); neck_k = int(p["neck"]); rm_ratio = float(p["rm"])

        cnts = split_touching_basic(mask, color_bgr=color_bgr, min_area=min_area,
                                    dist_thresh_rel=max(0.60, dist_rel),
                                    neck_cut=False,
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
            out.append({
                "color": cname, "c": c, "cx": cx, "cy": cy,
                "x": x, "y": y, "w": ww, "h": hh,
                "cnt_area": area, "posture_true": posture_true,
                "z_mm": z_mm,
            })
    return out

# ───────── 단일 색상 재수집(강/기본 선택 가능) ─────────
def collect_candidates_for_one_color(hsv, color_bgr, depth, cname, min_area=MIN_AREA, strong=False):
    if cname in ("Green", "Yellow"):
        strong = False

    ranges = COLOR_RANGES[cname]
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
                    dist_thresh_rel=(dist_rel if not strong else dist_rel),
                    neck_cut=(True if strong else False),
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
        out.append({
            "color": cname, "c": c, "cx": cx, "cy": cy,
            "x": x, "y": y, "w": ww, "h": hh,
            "cnt_area": area, "posture_true": posture_true,
            "z_mm": z_mm,
        })
    return out

# ───────── Top-Layer 필터 ─────────
def top_band_filter(cands, eps_mm=TOP_EPS_MM):
    if (not TOP_ONLY) or (not cands):
        return cands, None
    z_min = min(c["z_mm"] for c in cands)
    band = [c for c in cands if c["z_mm"] <= z_min + eps_mm]
    return band, z_min

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
        self.stable_frames = 0
        self.prev_cx = None
        self.prev_cy = None
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
    print("[MODE] AUTO+LOCK(+TOP): Z(min) 우선 + 면적우선, 화면 중심 무관 동작한다")
    print(f"[PREF] 면적 우선 범위={PREF_AREA_MIN}~{PREF_AREA_MAX} px^2")
    if TOP_ONLY:
        print(f"[TOP ] Top-Layer only: eps={TOP_EPS_MM}mm")

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
    sm_cx = sm_cy = None
    sm_ang = None

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

            # ----- (1) V 채널 언샤프 + CLAHE로 밝기(경계) 강화 -----
            hsv0 = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
            h0, s0, v0 = cv2.split(hsv0)
            v_blur  = cv2.GaussianBlur(v0, (0, 0), 1.2)
            v_sharp = cv2.addWeighted(v0, 1.8, v_blur, -0.8, 0)
            clahe   = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            v_boost = clahe.apply(v_sharp)
            hsv     = cv2.merge([h0, s0, v_boost])               # 마스크 계산용 HSV(강화본)
            color_for_ws = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)  # 워터셰드 입력용 BGR(강화본)

            # 1) 전 색상 후보 수집
            candidates = collect_candidates_all_colors_basic(
                hsv,               # 강화된 HSV
                color_for_ws,      # 강화된 BGR (워터셰드 입력)
                depth,
                min_area=MIN_AREA
            )

            # 1.5) Top-Layer 필터 (전역 적용)
            candidates, z_top = top_band_filter(candidates, eps_mm=TOP_EPS_MM)

            # 2) 현재 LOCK 규칙 + 면적우선으로 '미리' 선택
            pre_chosen = pick_candidate(candidates, lock)

            # 3) (옵션) 큰 면적 강분리 재수집 → 재수집 결과에도 Top-Layer 필터 재적용
            if (USE_STRONG_SPLIT and pre_chosen is not None and
                pre_chosen["cnt_area"] >= AREA_STRONG and
                pre_chosen["color"] not in ("Green", "Yellow")):
                candidates = [c for c in candidates if c["color"] != pre_chosen["color"]]
                rec = collect_candidates_for_one_color(
                    hsv, color_for_ws, depth,
                    cname=pre_chosen["color"], min_area=MIN_AREA, strong=True
                )
                rec, _ = top_band_filter(rec, eps_mm=TOP_EPS_MM)   # 재수집 결과도 Top-Band 필터
                candidates += rec

            # 3.5) LOCK이 Top-Band 밖으로 벗어나면 즉시 해제
            if lock.active and z_top is not None and (lock.z_mm is not None):
                if lock.z_mm > z_top + TOP_EPS_MM:
                    lock.reset()

            # 최종 선택 & LOCK 처리 (Top-Band 필터 이후 후보군만 사용)
            chosen = None
            if lock.active:
                gate_pool = []
                for info in candidates:
                    if info["color"] != lock.color:
                        continue
                    # Top-Band 강제: 후보 자체가 이미 Top-Band로 필터링되어 있음
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

                # 시각화
                sm_cx = int(round(ema(sm_cx, cx, LOCK_EMA_ALPHA)))
                sm_cy = int(round(ema(sm_cy, cy, LOCK_EMA_ALPHA)))

                Xmm, Ymm, Zmm = deproject_mm(cx, cy, z_mm/1000.0)
                center_mm = (round(Xmm,1), round(Ymm,1), round(Zmm,1))
                ang_deg, ok3d = angle_3d_deg(chosen["c"], depth, deproject_mm, stride=3)
            
                cv2.rectangle(color, (x, y), (x+ww, y+hh), DRAW_BGR[cname], 2)
                cv2.circle(color, (sm_cx, sm_cy), 6, DRAW_BGR[cname], -1)
                if sm_ang is not None:
                    L = max(ww, hh) // 2 + 20
                    rad = math.radians(sm_ang)
                    ex = int(sm_cx + L * math.cos(rad))
                    ey = int(sm_cy + L * math.sin(rad))
                    cv2.arrowedLine(color, (sm_cx, sm_cy), (ex, ey), DRAW_BGR[cname], 2, tipLength=0.25)

                label = (f"{cname}{' *' if _is_pref_area(cnt_area) else ''} "
                         f"[pose={str(posture_true)}] "
                         f"cntA={cnt_area}  3D(mm)=({Xmm:.1f},{Ymm:.1f},{Zmm:.1f})  "
                         f"{'LOCK' if lock.active else 'FREE'}  "
                         f"stable={lock.stable_frames}/{COUNT_STABLE_FRAMES}")
                cv2.putText(color, label, (x, max(0, y-8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, DRAW_BGR[cname], 2)

                # LOCK 상태 업데이트
                if lock.active:
                    lock.color = cname
                    lock.cx, lock.cy = cx, cy
                    lock.z_mm = z_mm
                    lock.center_mm = center_mm
                    lock.angle_deg = float(sm_ang) if sm_ang is not None else (float(ang_deg) if ang_deg is not None else None)

                # 송신
                payload = {
                    "color": cname,
                    "pose": bool(posture_true),
                    "center_mm": {"x": float(center_mm[0]), "y": float(center_mm[1]), "z": float(center_mm[2])},
                    "angle_deg": lock.angle_deg if lock.active else None,
                }
                payload_queue.put_latest(payload)

            # HUD (Top-Layer 정보 포함)
            hud = (f"AUTO+LOCK+TOP  TRUE-AREA(excl)=({POSTURE_TRUE_MIN},{POSTURE_TRUE_MAX}) "
                   f"PREF={PREF_AREA_MIN}-{PREF_AREA_MAX}px^2")
            hud += f"  STATE={'LOCKED' if lock.active else 'UNLOCK'}"
            if lock.active:
                hud += f"  miss={lock.miss}/{LOCK_MISS_MAX}  color={lock.color}"
            if TOP_ONLY and (z_top is not None):
                hud += f"  z_top={z_top:.1f}mm ±{TOP_EPS_MM:.1f}"
            cv2.putText(color, hud, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255,255,255), 2)

            # ───── 단일 창 표시 ─────
            cv2.imshow("color", color)

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
