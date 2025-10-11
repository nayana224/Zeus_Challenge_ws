# d435_multi_color_center_3d_angleXYZ_single_live_once_tx_threaded.py
import cv2, numpy as np, pyrealsense2 as rs, math, time, argparse, sys, select, json, socket, threading, queue

# ───────── 0) 네트워크 송신 기본 설정 ─────────
DEST_IP   = "192.168.1.23"
DEST_PORT = 10000
NET_TIMEOUT_S = 1.0
RETRY_COOLDOWN_S = 0.2
PERSISTENT_CONN = False      # 기본값, --persist로 켬
SEND_MIN_INTERVAL = 0.05     # 스레드가 송신 주기를 관리한다(초)

# ───────── 1) HSV 색 범위 ─────────
COLOR_RANGES = {
    "Red":    [ (np.array([  0,120, 50], np.uint8), np.array([ 4,255,255], np.uint8)),
                (np.array([170,120, 50], np.uint8), np.array([180,255,255], np.uint8)) ],
    "Pink":   [ (np.array([3.8, 60, 50], np.uint8), np.array([20,255,255], np.uint8)),],
    "Yellow": [ (np.array([ 20,120, 50], np.uint8), np.array([ 35,255,255], np.uint8)) ],
    "Green":  [ (np.array([ 40, 80, 50], np.uint8), np.array([ 85,255,255], np.uint8)) ],
    "Blue":   [ (np.array([ 88, 80, 50], np.uint8), np.array([125,255,255], np.uint8)) ],
    "Purple": [ (np.array([120, 50, 50], np.uint8), np.array([160,255,255], np.uint8)) ],
}

# ───────── 2) 바운딩 박스 드로잉 색상 ─────────
DRAW_BGR = {
    "Red":(0,0,255), "Pink":(203,192,255), "Yellow":(0,255,255),
    "Green":(0,255,0), "Blue":(255,0,0), "Purple":(255,0,255),
}

# ───────── 3) 파라미터 ─────────
MIN_AREA         = 400
KERNEL           = np.ones((5,5), np.uint8)
TIMEOUT_MS       = 5000
POSTURE_AREA_MIN = 1300 # True / False 정하는 컨투어박스 크기 임계값

# ───────── Util: 색 이름 정규화 ─────────
CANON = {
    "red":"Red", "빨강":"Red", "빨간색":"Red",
    "pink":"Pink", "분홍":"Pink", "분홍색":"Pink",
    "yellow":"Yellow", "노랑":"Yellow", "노란색":"Yellow",
    "green":"Green", "초록":"Green", "녹색":"Green",
    "blue":"Blue", "파랑":"Blue", "파란색":"Blue", "블루":"Blue",
    "purple":"Purple", "보라":"Purple", "보라색":"Purple",
    "list":"LIST"
}
def normalize_color_input(s: str):  # 색상 입력 함수
    if not s: return None
    s = s.strip().lower()
    return CANON.get(s, None)

def parse_initial_args():   # 초기 타깃색상, 자세 판정용 면적 임계값 반환
    import argparse, sys
    parser = argparse.ArgumentParser(description="RealSense 단일 색상 인식 + TCP JSON 송신(스레드)다")
    parser.add_argument("--color","-c", type=str,
                        help="초기 인식 색상 (red/빨강, pink/분홍, yellow/노랑, green/초록, blue/파랑, purple/보라)다")
    parser.add_argument("--area-min", type=int, default=POSTURE_AREA_MIN,
                        help="자세(True/False) 판정용 박스면적 임계값(px^2)이다")
    parser.add_argument("--dest-ip", type=str, default=DEST_IP, help="송신 대상 IP다")
    parser.add_argument("--dest-port", type=int, default=DEST_PORT, help="송신 대상 포트다")
    parser.add_argument("--persist", action="store_true", help="지속 연결 모드로 송신한다")
    args = parser.parse_args()

    # 초기 색상
    if args.color:
        key = normalize_color_input(args.color)
        if key is None or key=="LIST":
            valid = "red/빨강, pink/분홍, yellow/노랑, green/초록, blue/파랑, purple/보라"
            print(f"[오류] 알 수 없는 색 이름: {args.color}\n사용 가능: {valid}")
            sys.exit(1)
        init_color = key
    else:
        init_color = "Yellow"

    return init_color, int(args.area_min), args.dest_ip, int(args.dest_port), bool(args.persist)

def try_readline_nonblock():    # 키보드 입력 읽어오는 함수(입력 감지됐을 때만)
    import select, sys
    try:
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if rlist:   # 입력이 감지됐을때만 줄 읽어옴
            line = sys.stdin.readline()
            if not line: return None
            return line.strip()
    except Exception:
        pass
    return None

# ───────── Util: 마스크 결합 ─────────
def color_mask(hsv, ranges):    # HSV색상공간에서 특정 색 범위에 해당하는 픽셀만 마스킹
    m = None
    for low, high in ranges:
        part = cv2.inRange(hsv, low, high)
        m = part if m is None else (m | part)
    return m

# ───────── Util: 3D/2D 주성분 각도 ─────────
def angle_3d_deg(contour, depth, deproject_fn, stride=3):   # 블럭 각도 구하는 함수
    pts = contour.reshape(-1, 2)    # 컨투어 포인트 추출
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

# ───────── 붙은 물체 분리 ─────────
def split_touching(mask_bin,               # 이진화된 마스크 이미지
                   color_bgr,              # 컬러 프레임 원본
                   min_area=MIN_AREA,      # 잡음 무시
                   dist_thresh_rel=0.45,   # 0.35→0.40~0.55로 키우면 더 빡세게 분리
                   neck_cut=True,          # 붙은 블럭의 연결부 끊음
                   neck_k=7,               # 목 절단용 커널 길이(홀수 권장, 5~15)
                   rm_thin_ratio=0.12):    # 너무 가늘면 제거(면적/외접사각형 면적 비)
    """
    같은 색으로 붙은 사각 블럭 분리 전용 파이프라인이다.
    1) 얇은 연결부를 개구로 끊고
    2) distanceTransform 로컬 최대값을 씨앗으로 만들고
    3) 워터셰드로 분리한다
    """
    # 0) 기본 노이즈 정리
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    opened = cv2.morphologyEx(mask_bin, cv2.MORPH_OPEN, k3, iterations=1)

    # 1) 얇은 목 절단(가로/세로 얇은 커널 개구)
    cut = opened.copy()
    if neck_cut:
        kv = cv2.getStructuringElement(cv2.MORPH_RECT, (1, neck_k))  # 세로선 커널
        kh = cv2.getStructuringElement(cv2.MORPH_RECT, (neck_k, 1))  # 가로선 커널
        cut_v = cv2.morphologyEx(cut, cv2.MORPH_OPEN, kv, iterations=1)
        cut_h = cv2.morphologyEx(cut, cv2.MORPH_OPEN, kh, iterations=1)
        cut   = cv2.bitwise_or(cut_v, cut_h)

    # 2) 거리변환 + 로컬 최대 피크(마커)
    dist = cv2.distanceTransform(cut, cv2.DIST_L2, 5)
    # 상대 임계치: 전체 최대값의 비율
    if dist.max() > 0:
        _, peaks = cv2.threshold(dist, dist_thresh_rel * dist.max(), 255, cv2.THRESH_BINARY)
    else:
        peaks = np.zeros_like(dist, dtype=np.uint8)
    peaks = peaks.astype(np.uint8)

    # 로컬 최대 강화: 팽창 결과와 같은 지점만 남기기
    d8 = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    local_max = cv2.dilate(dist, d8)
    peaks2 = np.where((dist == local_max) & (dist > 0), 255, 0).astype(np.uint8)
    # 둘 중 더 강한 마커 사용
    sure_fg = cv2.bitwise_or(peaks, peaks2)

    # 3) 확실한 배경(sure_bg)과 unknown 계산
    sure_bg = cv2.dilate(cut, k3, iterations=2)
    unknown = cv2.subtract(sure_bg, sure_fg)

    # 4) 레이블링 → 워터셰드
    num, markers = cv2.connectedComponents(sure_fg)
    markers = markers + 1
    markers[unknown == 255] = 0

    # 워터셰드는 3채널 입력 필요
    ws_input = color_bgr.copy()
    markers = cv2.watershed(ws_input, markers)

    # 5) 레이블별 바이너리 → 컨투어 추출
    contours = []
    max_label = markers.max()
    for label in range(2, max_label + 1):
        region = np.uint8(markers == label) * 255
        if cv2.countNonZero(region) < min_area:
            continue

        # 너무 얇은(사각 블럭 같지 않은) 후보 제거
        x, y, w, h = cv2.boundingRect(region)
        rect_area = w * h
        obj_area  = int(cv2.countNonZero(region))
        if rect_area <= 0:
            continue
        solidity_like = obj_area / float(rect_area)  # 사각형에 얼마나 꽉 찼나
        if solidity_like < rm_thin_ratio:
            continue

        cs, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours.extend(cs)

    return contours


# ───────── 네트워크 송신기(스레드 내부에서 사용) ─────────
class TcpSender:
    def __init__(self, ip, port, timeout=NET_TIMEOUT_S, persistent=False):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.persistent = persistent
        self.sock = None
        if self.persistent:
            self._connect()

    def _connect(self):
        self.close()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        s.connect((self.ip, self.port))
        self.sock = s

    def send_line(self, line_bytes: bytes):
        if not self.persistent:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(self.timeout)
                s.connect((self.ip, self.port))
                s.sendall(line_bytes)
            except Exception as e:
                print(f"[NET] send error(oneshot): {e}")
                time.sleep(RETRY_COOLDOWN_S)
            finally:
                try:
                    s.shutdown(socket.SHUT_RDWR)
                except:
                    pass
                try:
                    s.close()
                except:
                    pass
        else:
            try:
                if self.sock is None:
                    self._connect()
                self.sock.sendall(line_bytes)
            except Exception as e:
                print(f"[NET] send error(persistent): {e}, reconnect한다")
                time.sleep(RETRY_COOLDOWN_S)
                try:
                    self._connect()
                    self.sock.sendall(line_bytes)
                except Exception as e2:
                    print(f"[NET] resend failed: {e2}")

    def close(self):
        if self.sock is not None:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except:
                pass
            try:
                self.sock.close()
            except:
                pass
            self.sock = None

# ───────── 가장 최신 페이로드만 유지하는 큐 ─────────
class LatestQueue:
    def __init__(self):
        self.q = queue.Queue(maxsize=1)
        self.lock = threading.Lock()

    def put_latest(self, item):
        with self.lock:
            if self.q.full():
                try:
                    self.q.get_nowait()
                except queue.Empty:
                    pass
            self.q.put_nowait(item)

    def get_or_none(self, timeout=None):
        try:
            return self.q.get(timeout=timeout)
        except queue.Empty:
            return None

# ───────── 송신 스레드 ─────────
class TxWorker(threading.Thread):
    # 초기화 함수
    """ 파라미터
        ip, port : 로봇 주소
        persistent : True면 지속 연결 유지, False면 매 송신마다 1회성 접속
        stop_event : 외부에서 스레드 종료를 지시하는 이벤트
        payload_queue : 메인 쓰레드가 넣어주는 '최신 페이로드' 큐 (최신 1개만 유지)
        min_interval : 송신 최소 간격(초). 과도한 송신으로 인한 과부하/네트워크 지연 방지 """
    def __init__(self, ip, port, persistent, stop_event: threading.Event, payload_queue: LatestQueue, min_interval=SEND_MIN_INTERVAL):
        super().__init__(daemon=True)
        self.sender = TcpSender(ip, port, timeout=NET_TIMEOUT_S, persistent=persistent)
        self.stop_event = stop_event    # 종료 제어용 이벤트
        self.payload_queue = payload_queue  # 최신 페이로드 큐(최대 1개)
        self.min_interval = min_interval    # 송신 최소 간격
        self.last_sent_ts = 0.0     # 마지막 송신 시각

    # 백그라운드 스레드 메인 루프
        """ 1) 큐에서 최신 페이로드를 꺼낸다(없으면 잠깐 기다림)
        2) 송신 최소 간격을 만족할 때까지 대기(대기 중 더 최신 항목이 들어오면 교체)
        3) 가장 최신 페이로드를 JSON 1줄로 인코딩 후 TCP 송신
        4) 에러 시 로그 출력 + 짧은 대기 후 다음 루프 진행 """
    def run(self):
        try:
            while not self.stop_event.is_set():
                # 최신 페이로드를 기다리되, 없으면 잠깐 딜레이 후 루프 재시도
                item = self.payload_queue.get_or_none(timeout=0.1)
                if item is None:
                    continue

                # 송신 최소 간격 유지
                now = time.time()
                remain = self.min_interval - (now - self.last_sent_ts)
                if remain > 0:
                    # 대기하는 동안 더 최신 항목이 들어오면 교체됨
                    self.stop_event.wait(remain)

                # 직전 대기를 마친 뒤에도 가장 최신으로 갱신
                # 큐가 갱신되어도 마지막 것만 가져옴
                more = True
                latest = item
                while more and not self.stop_event.is_set():
                    nxt = self.payload_queue.get_or_none(timeout=0.0)
                    if nxt is None:
                        more = False
                    else:
                        latest = nxt    # 큐에 들어오는 즉시 교체(항상 최신만 송신하도록)

                try:
                    # JASON 1줄 인코딩하여 송신
                    line = (json.dumps(latest, ensure_ascii=False) + "\n").encode("utf-8")
                    self.sender.send_line(line)
                    self.last_sent_ts = time.time() # 마지막 송신 시간 갱신
                    # print(f"[TX] {latest}")
                except Exception as e:
                    # 네트워크 오류 및 연결 실패인 경우 짧은 딜레이 후 송신 재시도
                    print(f"[NET] thread send error: {e}")
                    time.sleep(RETRY_COOLDOWN_S)
        finally:
            self.sender.close()

# ───────── 메인 ─────────
def main():
    target_color, posture_area_min, dest_ip, dest_port, persist = parse_initial_args()
    if target_color not in COLOR_RANGES:
        print(f"[오류] 내부 색상 테이블에 없음: {target_color}")
        sys.exit(1)
    print(f"[INIT] 초기 타깃 색상: {target_color}")
    print(f"[INIT] 자세판정 임계값(area-min)={posture_area_min} px^2")
    print(f"[NET ] DEST={dest_ip}:{dest_port}  PERSIST={persist}")

    # 색 변경 시 1회만 콘솔 출력
    print_once_requested = False

    # ── 송신 스레드 준비
    stop_event = threading.Event()
    payload_queue = LatestQueue()
    tx_thread = TxWorker(dest_ip, dest_port, persist, stop_event, payload_queue, min_interval=SEND_MIN_INTERVAL)
    tx_thread.start()

    # ───────── RealSense 파이프라인 ─────────
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile  = pipeline.start(config)
    align    = rs.align(rs.stream.color)

    color_intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    def deproject_mm(u, v, depth_m):
        X, Y, Z = rs.rs2_deproject_pixel_to_point(color_intr, [float(u), float(v)], float(depth_m))
        return X*1000.0, Y*1000.0, Z*1000.0

    stable_flag = False
    last_color  = target_color

    try:
        while True:
            # 실행 도중 단일 색상 변경 입력 처리
            s = try_readline_nonblock()
            if s:
                key = normalize_color_input(s)
                if key in COLOR_RANGES:
                    target_color = key
                    print_once_requested = True
                    print(f"[INPUT] 타깃 색상 변경: {target_color}")
                elif key is not None:
                    print(f"[WARN] '{s}' 은(는) 유효한 색 이름이 아니다")
                elif key is None and s.strip():
                    print(f"[WARN] '{s}' 은(는) 유효한 입력이 아니다")

            frames = pipeline.wait_for_frames(TIMEOUT_MS)
            frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())  # mm
            hsv   = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

            ranges = COLOR_RANGES[target_color]
            mask = color_mask(hsv, ranges)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)

            cnts = split_touching(mask, color_bgr=color,
                                  min_area=MIN_AREA, dist_thresh_rel=0.50, neck_cut=True, neck_k=9, rm_thin_ratio=0.12)

            h, w = color.shape[:2]
            cx0, cy0 = (w // 2, h // 2)

            best = None  # (dist2, info_dict)
            for c in cnts:
                area = cv2.contourArea(c)
                if area < MIN_AREA:
                    continue

                M = cv2.moments(c)
                if M["m00"] <= 1e-5:
                    continue
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                x, y, ww, hh = cv2.boundingRect(c)
                cnt_area = int(cv2.contourArea(c))
                posture_true = (cnt_area >= posture_area_min)

                roi = depth[max(cy-3,0):cy+4, max(cx-3,0):cx+4]
                valid = roi[roi > 0]
                if valid.size > 0:
                    d_mm = float(np.median(valid))
                    Xmm, Ymm, Zmm = deproject_mm(cx, cy, d_mm/1000.0)
                    center_mm = (round(Xmm,1), round(Ymm,1), round(Zmm,1))
                    depth_txt = f"3D(mm)=({Xmm:.1f},{Ymm:.1f},{Zmm:.1f})"
                else:
                    center_mm = None
                    depth_txt = "3D N/A"

                ang_deg, ok3d = angle_3d_deg(c, depth, deproject_mm, stride=3)
                if ang_deg is not None:
                    angle_txt = f"{ang_deg:+.1f} deg ({'3D' if ok3d else '2D'})"
                else:
                    angle_txt = "angle N/A"

                dist2 = (cx - cx0) * (cx - cx0) + (cy - cy0) * (cy - cy0)

                info = {
                    "c": c, "cx": cx, "cy": cy,
                    "x": x, "y": y, "w": ww, "h": hh,
                    "cnt_area": cnt_area,
                    "posture_true": posture_true,
                    "center_mm": center_mm,
                    "depth_txt": depth_txt,
                    "ang_deg": ang_deg, "ok3d": ok3d, "angle_txt": angle_txt
                }

                if (best is None) or (dist2 < best[0]):
                    best = (dist2, info)

            if best is not None:
                info = best[1]
                cx, cy = info["cx"], info["cy"]
                x, y, ww, hh = info["x"], info["y"], info["w"], info["h"]
                center_mm = info["center_mm"]
                depth_txt = info["depth_txt"]
                ang_deg   = info["ang_deg"]
                ok3d      = info["ok3d"]
                angle_txt = info["angle_txt"]
                cnt_area  = info["cnt_area"]
                posture_true = info["posture_true"]
                c         = info["c"]

                # 안정성 플래그(단순 샘플)
                stable_flag = (target_color == last_color)
                last_color = target_color

                # 시각화
                cv2.rectangle(color, (x, y), (x+ww, y+hh), DRAW_BGR[target_color], 2)
                cv2.circle(color, (cx, cy), 5, DRAW_BGR[target_color], -1)

                if ang_deg is not None:
                    L = max(ww, hh) // 2 + 20
                    rad = math.radians(ang_deg)
                    ex = int(cx + L * math.cos(rad))
                    ey = int(cy + L * math.sin(rad))
                    cv2.arrowedLine(color, (cx, cy), (ex, ey), DRAW_BGR[target_color], 2, tipLength=0.25)

                label = (f"{target_color} [pose={str(posture_true)}] "
                         f"cntA={cnt_area} {depth_txt}  angle={angle_txt}")
                cv2.putText(color, label, (x, max(0, y-8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, DRAW_BGR[target_color], 2)

                # ───────── JSON 페이로드 생성 → 스레드 큐에 최신으로 전달 ─────────
                payload = {
                    "color": target_color,
                    "pose": bool(posture_true)
                }
                if center_mm is not None:
                    Xr, Yr, Zr = center_mm
                    payload["center_mm"] = {"x": float(Xr), "y": float(Yr), "z": float(Zr)}
                if ang_deg is not None:
                    payload["angle_deg"] = float(ang_deg)

                payload_queue.put_latest(payload)

                # 색 변경 후 1회 콘솔 로그
                if print_once_requested:
                    if center_mm is None:
                        print(f"[ONCE] {target_color:6s} px=({cx:3d},{cy:3d}) "
                              f"pose={posture_true} boxA={cnt_area} "
                              f"angle={ang_deg if ang_deg is not None else 'N/A'} "
                              f"src={'3D' if ok3d else ('2D' if ang_deg is not None else 'N/A')}")
                    else:
                        Xr, Yr, Zr = center_mm
                        print(f"[ONCE] {target_color:6s} px=({cx:3d},{cy:3d}) "
                              f"mm=({Xr:.1f},{Yr:.1f},{Zr:.1f}) "
                              f"pose={posture_true} boxA={cnt_area} "
                              f"angle={ang_deg if ang_deg is not None else 'N/A'} "
                              f"src={'3D' if ok3d else ('2D' if ang_deg is not None else 'N/A')}")
                    print_once_requested = False

            # HUD
            cv2.putText(color, f"TARGET(SINGLE)={target_color}  AREA_MIN={posture_area_min}", (10, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)

            cv2.imshow("color", color)
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break

    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        # 스레드 종료
        stop_event.set()
        tx_thread.join(timeout=1.0)

if __name__ == "__main__":
    main()