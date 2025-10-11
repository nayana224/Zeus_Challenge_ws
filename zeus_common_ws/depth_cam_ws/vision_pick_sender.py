'''
<전체 흐름>
1) 카메라 초기화: RGB(640x480@30), Depth(640x480@30) 켠 뒤, depth→color 정렬(rs.align)
2) 프레임 루프:
    - BRG -> HSV 변환 -> 색 범위별 마스크 -> 열고/닫기(노이즈 제거) -> 컨투어 추출
    - 첫 번째로 잡힌 색(하나만) 선택 -> 무게중심(cx, cy) -> 주변 7x7 depth의 중앙값으로 Z 추정
    - rs2_deproject_pixel_to_point로 X, Y, Z(mm) 복원
    - 컨투어 점들을 3D로 샘플링해 PCA(고유벡터)로 주축 계산 -> 각도 (-90 ~ 90)
    - 화면에 박스/중심/각도 그려 보여주고, 콘솔 로그는 0.1초 간격으로 출력
    - 네트워크 스레드에 가장 최신 payoad 하나만 넘김(전송은 스레드가 담당)
3) 송신 스레드: 
    - TCP 연결 유지/재시도
    - 최신 것만 20Hz이내로 전송(드롭 허용), 에러 시 자동 재연결 시도
4) 종료
'''



# d435_multi_color_center_3d_angleXYZ_send_threaded.py
import cv2, numpy as np, pyrealsense2 as rs, math, time, socket, json, errno, threading

# ───────── 0) 네트워크 설정 ─────────
SERVER_IP   = "192.168.1.23"
SERVER_PORT = 10000
RETRY_SEC   = 2.0
SEND_MIN_INTERVAL = 0.05   # 송신 스레드에서만 사용 -> 최대 약 20Hz로 보냄

# ───────── 1) HSV 색 범위 ─────────
COLOR_RANGES = {
    "Red":    [ (np.array([  0,120, 50], np.uint8), np.array([  2,255,255], np.uint8)),
                (np.array([175,120, 50], np.uint8), np.array([180,255,255], np.uint8)) ],
    "Pink":   [ (np.array([  3,110, 30], np.uint8), np.array([  6,255,255], np.uint8)) ],
    "Yellow": [ (np.array([ 20,120, 50], np.uint8), np.array([ 35,255,255], np.uint8)) ],
    "Green":  [ (np.array([ 40, 80, 50], np.uint8), np.array([ 85,255,255], np.uint8)) ],
    "Blue":   [ (np.array([ 88, 80, 50], np.uint8), np.array([105,255,255], np.uint8)) ],
    "Purple": [ (np.array([120, 50, 50], np.uint8), np.array([160,255,255], np.uint8)) ],
}

# ───────── 2) 드로잉(GUI) 색상 ─────────
DRAW_BGR = {
    "Red":(0,0,255), "Pink":(203,192,255), "Yellow":(0,255,255),
    "Green":(0,255,0), "Blue":(255,0,0), "Purple":(255,0,255),
}

MIN_AREA = 1500
KERNEL   = np.ones((5,5), np.uint8)
PRINT_MIN_INTERVAL = 0.10

# ───────── 3) RealSense 파이프라인 ─────────
pipeline = rs.pipeline()
config   = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile  = pipeline.start(config)
align    = rs.align(rs.stream.color)

# 컬러 카메라 내부 파라미터
color_intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

def deproject_mm(u, v, depth_m):
    X, Y, Z = rs.rs2_deproject_pixel_to_point(color_intr, [float(u), float(v)], float(depth_m))
    return X*1000.0, Y*1000.0, Z*1000.0

def color_mask(hsv, ranges):
    m = None
    for low, high in ranges:
        part = cv2.inRange(hsv, low, high)
        m = part if m is None else (m | part)
    return m

def angle_3d_deg(contour, depth, stride=3):
    pts = contour.reshape(-1, 2)
    h, w = depth.shape
    XY = []
    for i in range(0, len(pts), max(1, stride)):
        u, v = int(pts[i][0]), int(pts[i][1])
        if 0 <= v < h and 0 <= u < w:
            d = int(depth[v, u])  # mm
            if d > 0:
                Xmm, Ymm, Zmm = deproject_mm(u, v, d/1000.0)
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

    # 2D 폴백
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

# ───────── 4) 송신 스레드 ─────────
class Sender(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.sock = None
        self.last_retry = 0.0
        self.last_send  = 0.0
        self.lock = threading.Lock()
        self.latest_payload = None   # 항상 “마지막” 1개만 보관
        self.running = True

    def set_latest(self, payload: dict):
        # 메인 루프에서 호출, 덮어쓰기만 함(오버헤드 매우 적음)
        with self.lock:
            self.latest_payload = payload

    def _connect_with_retry(self):
        now = time.time()
        if self.sock is not None:
            return
        if (now - self.last_retry) < RETRY_SEC:
            return
        self.last_retry = now
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)
            s.connect((SERVER_IP, SERVER_PORT))
            s.settimeout(5.0)  # 블로킹 최소화
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # keepalive(옵션)
            try:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            except Exception:
                pass
            self.sock = s
            print(f"[NET] Connected to {SERVER_IP}:{SERVER_PORT}")
        except Exception as e:
            print(f"[NET] Connect failed: {e}")

    def _send_one(self, payload):
        if self.sock is None or payload is None:
            return
        now = time.time()
        if (now - self.last_send) < SEND_MIN_INTERVAL:
            return
        self.last_send = now
        try:
            line = json.dumps(payload, ensure_ascii=False).encode('utf-8') + b'\n'
            self.sock.sendall(line)   # 실패 시 except로 빠짐
        except (socket.timeout, OSError) as e:
            print(f"[NET] send err: {e} → reconnect")
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None  # 다음 루프에서 재접속

    def run(self):
        while self.running:
            self._connect_with_retry()
            # 최신 payload만 꺼내서 보냄(오래된 건 버림)
            with self.lock:
                payload = self.latest_payload
                self.latest_payload = None  # 한번 보낼 기회만 주고 비움(드롭 허용)
            self._send_one(payload)
            time.sleep(0.001)  # 아주 짧은 휴식으로 CPU 100% 방지

    def stop(self):
        self.running = False
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass

sender = Sender()
sender.start()

last_print = 0.0

try:
    # 한 개의 프레임 받아 처리하는 Sequence
    while True: 
        frames = pipeline.wait_for_frames() # pipeline에서 새 프레임 세트를 기다렸다가 받음
        frames = align.process(frames) # 정렬: depth 프레임을 color 프레임과 같은 픽셀 좌표계로 맞춤
        # 프레임 세트에서 컬러와 뎁스 각각 추출
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        # 둘 중 하나라도 없으면 다음 프레임으로
        if not color_frame or not depth_frame:
            continue
        
        # 컬러/뎁스 프레임을 Numpy 배열로 변환
        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())  # [mm]
        # 색 검출을 위해 BGR->HSV 변환 (H=색상, S=채도, V=명도 / 색 분류 쉬움)
        hsv   = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        # 이번 프레임에서 가장 먼저 발견한 물체 하나만 처리 및 전송하기 위한 Flag
        first_found = False

        # 사전에 정의한 색상들(ex: Red, Green, ...)을 순서대로 검사 
        for cname, ranges in COLOR_RANGES.items():
            # 이미 하나를 찾았다면 더이상 찾지 않고 루프 탈출
            if first_found:
                break
            
            # 현재 색(cname)의 HSV 하한/상한으로 이진 마스크 생성 (해당 색이면 255, 아니면 0)
            mask = color_mask(hsv, ranges)
            # 모폴로지 연산: 이진화된 영상에 대해 모양 및 구조를 다듬는 연산
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1) # 색 검출 후 작은 노이즈 점 제거
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=2) # 객체 내부 작은 구멍 메우기, 외곽 다듬기
            # 마스크에서 바깥 외곽선들을 찾음
            cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            # 너무 작은 외곽선은 거르고, 최소 면적보다 큰 것만 남김
            cnts = [c for c in cnts if cv2.contourArea(c) > MIN_AREA]
            # 해당 색으로 유효한 물체가 없으면 다음 색으로 넘어감
            if not cnts:
                continue
            
            # 첫 번째 컨투어 선택
            c = cnts[0]
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(color, (x, y), (x+w, y+h), DRAW_BGR[cname], 2)

            # 모멘트(면적, 무게중심 계산용 값) 계산
            M = cv2.moments(c)
            if M["m00"] <= 1e-5:
                continue
            # 무게중심 픽셀 좌표 계산
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(color, (cx, cy), 5, DRAW_BGR[cname], -1)
            # 중심(cx, cy) 주변 7x7 깊이 ROI 추출 -> 유효값의 중앙값으로 깊이[mm] 결정
            roi = depth[max(cy-3,0):cy+4, max(cx-3,0):cx+4]
            valid = roi[roi > 0]
            if valid.size > 0:
                d_mm = float(np.median(valid))
                Xmm, Ymm, Zmm = deproject_mm(cx, cy, d_mm/1000.0) # 픽셀 to 카메라 3D 좌표 변환
                center_mm = (round(Xmm,1), round(Ymm,1), round(Zmm,1))
                depth_txt = f"3D(mm)=({Xmm:.1f},{Ymm:.1f},{Zmm:.1f})"
            else:
                center_mm = None
                depth_txt = "3D N/A"

            # 컨투어 점들을 샘플링하여 각도 산출
            ang_deg, ok3d = angle_3d_deg(c, depth, stride=3)
            if ang_deg is not None:
                L = max(w, h) // 2 + 20
                rad = math.radians(ang_deg)
                ex = int(cx + L * math.cos(rad))
                ey = int(cy + L * math.sin(rad))
                cv2.arrowedLine(color, (cx, cy), (ex, ey), DRAW_BGR[cname], 2, tipLength=0.25)
                angle_txt = f"{ang_deg:+.1f} deg ({'3D' if ok3d else '2D'})"
            else:
                angle_txt = "angle N/A"

            # 콘솔 출력(비전 루프는 I/O도 최소화)
            now = time.time()
            if (now - last_print) >= PRINT_MIN_INTERVAL:
                if center_mm is None:
                    print(f"[{now:.3f}] {cname:6s} px=({cx:3d},{cy:3d}) angle={ang_deg if ang_deg is not None else 'N/A'} src={'3D' if ok3d else ('2D' if ang_deg is not None else 'N/A')}")
                else:
                    Xr, Yr, Zr = center_mm
                    print(f"[{now:.3f}] {cname:6s} px=({cx:3d},{cy:3d}) mm=({Xr:.1f},{Yr:.1f},{Zr:.1f}) angle={ang_deg if ang_deg is not None else 'N/A'} src={'3D' if ok3d else ('2D' if ang_deg is not None else 'N/A')}")
                last_print = now

            # ── 최신 payload만 넘김(전송은 송신 스레드가 수행) ──
            payload = {
                "ts": now, "color": cname,
                "center_px": {"x": cx, "y": cy},
                "center_mm": (None if center_mm is None else
                            {"x": center_mm[0], "y": center_mm[1], "z": center_mm[2]}),
                "angle_deg": (None if ang_deg is None else float(ang_deg)),
                "angle_src": ("3D" if ok3d else ("2D" if ang_deg is not None else "N/A")),
                "bbox": {"x": int(x), "y": int(y), "w": int(w), "h": int(h)},
                "camera": {"w": int(color.shape[1]), "h": int(color.shape[0])},
                "stable": True   
            }
            sender.set_latest(payload)

            label = f"{cname} {depth_txt}  angle={angle_txt}"
            cv2.putText(color, label, (x, max(0, y-8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, DRAW_BGR[cname], 2)

            first_found = True

        cv2.imshow("color", color)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            break

finally:
    try: pipeline.stop()
    except Exception: pass
    try: sender.stop()
    except Exception: pass
    cv2.destroyAllWindows()
