import numpy as np
import cv2
import pyrealsense2 as rs
import socket, json, time
import threading
import queue
from collections import Counter

# JSON TCP 통신 설정
HOST_RECEIVER = "192.168.1.23"  # 수신 측 IP 주소
PORT_JSON_OUT = 10000            # 포트 번호

# ----------------------------------------------------
# 전역 변수 정의 (시간 제어)
last_queue_time = 0.0
comm_queue = queue.Queue(maxsize=1)

is_debounce_active = False
debounce_start_time = 0.0
DEBOUNCE_TIME = 1.0
COOL_DOWN_PERIOD = 5.0
# ----------------------------------------------------


def _send_tcp_message(connector_type):
    """
    [함수] 실제 TCP 통신을 수행하는 내부 함수: JSON 메시지 전송
    """
    data = {"connector": connector_type}
    msg = (json.dumps(data) + '\n').encode('utf-8')

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.settimeout(0.5)
        s.connect((HOST_RECEIVER, PORT_JSON_OUT))
        s.send(msg)
        print(f"[JSON-TX] Sent: {msg.decode('utf-8').strip()}")
        return True
    except Exception:
        return False
    finally:
        try:
            s.shutdown(socket.SHUT_RDWR)
        except:
            pass
        s.close()


def comm_worker():
    """
    백그라운드 통신 워커 스레드 함수: 큐의 데이터를 꺼내 TCP 통신 수행
    """
    print("[COMM_WORKER] TCP Worker thread started and waiting for data...")
    while True:
        try:
            connector_type = comm_queue.get(timeout=0.1)
            if _send_tcp_message(connector_type):
                print("[COMM_WORKER] Communication attempt finished.")
            comm_queue.task_done()
        except queue.Empty:
            continue
        except Exception as e:
            print(f"[COMM_WORKER] Unexpected error: {e}")
            comm_queue.task_done()
            continue


# 1. RealSense 초기 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
try:
    profile = pipeline.start(config)
except Exception as e:
    print(f"RealSense Error: Failed to start pipeline. Error: {e}")
    exit()
align = rs.align(rs.stream.color)

# 2. HSV 색상 범위 및 필터 조건 설정
lower_yellow = np.array([5, 50, 50])    # 노란색 (XT)
upper_yellow = np.array([45, 255, 255])
lower_blue = np.array([95, 70, 40])     # 밝은 환경 대응: 파란색 (EC3)
upper_blue = np.array([145, 255, 255])

ROI_X, ROI_Y, ROI_W, ROI_H = 160, 120, 320, 240
MIN_W_H_YELLOW = 20
MIN_AREA_YELLOW = 1000
AREA_THRESHOLD = 6000
MIN_W_H_BLUE = 15
MIN_AREA_BLUE = 1000
kernel = np.ones((5, 5), np.uint8)

# 백그라운드 통신 스레드 시작
t_comm = threading.Thread(target=comm_worker)
t_comm.daemon = True
t_comm.start()

try:
    while True:
        current_time = time.time()

        try:
            frames = pipeline.wait_for_frames(100)
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
        except RuntimeError:
            continue

        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        roi_hsv = hsv[ROI_Y:ROI_Y + ROI_H, ROI_X:ROI_X + ROI_W]
        connector_type = None

        # 1. XT 커넥터 (노란색)
        mask_yellow = cv2.inRange(roi_hsv, lower_yellow, upper_yellow)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel, iterations=1)
        contours_y, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_y = sorted(contours_y, key=cv2.contourArea, reverse=True)

        if contours_y:
            largest_yellow_cnt = contours_y[0]
            x_roi, y_roi, w, h = cv2.boundingRect(largest_yellow_cnt)
            area = cv2.contourArea(largest_yellow_cnt)

            if w > MIN_W_H_YELLOW and h > MIN_W_H_YELLOW and area > MIN_AREA_YELLOW:
                if area >= AREA_THRESHOLD:
                    connector_type = "xt90"
                    color = (0, 255, 255)
                else:
                    connector_type = "xt60"
                    color = (0, 200, 255)

                x_global, y_global = ROI_X + x_roi, ROI_Y + y_roi
                cv2.rectangle(color_image, (x_global, y_global),
                              (x_global + w, y_global + h), color, 2)

        # 2. EC3 (파란색)
        if not connector_type:
            mask_blue = cv2.inRange(roi_hsv, lower_blue, upper_blue)
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel, iterations=1)
            contours_b, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours_b = sorted(contours_b, key=cv2.contourArea, reverse=True)

            if contours_b:
                cnt = contours_b[0]
                x_roi, y_roi, w, h = cv2.boundingRect(cnt)
                area = cv2.contourArea(cnt)

                if w > MIN_W_H_BLUE and h > MIN_W_H_BLUE and area > MIN_AREA_BLUE:
                    connector_type = "ec3"
                    x_global, y_global = ROI_X + x_roi, ROI_Y + y_roi
                    cv2.rectangle(color_image, (x_global, y_global),
                                  (x_global + w, y_global + h), (255, 0, 0), 2)

        # 3. 전송 로직 (EC3 즉시 전송, XT류 디바운스 유지)
        if connector_type == "ec3":
            if comm_queue.empty():
                comm_queue.put(connector_type)
                last_queue_time = current_time
                print(f"[MAIN_THREAD] EC3 detected. Immediate send executed.")
            continue  # EC3는 바로 전송하고 루프 재시작

        # XT 계열 (디바운스 + 쿨다운)
        if current_time - last_queue_time > COOL_DOWN_PERIOD:
            if connector_type:
                if not is_debounce_active:
                    is_debounce_active = True
                    debounce_start_time = current_time
                    print(f"[DEBOUNCE] Detection started: '{connector_type}'. Beginning {DEBOUNCE_TIME}s discard period.")
                elif current_time - debounce_start_time >= DEBOUNCE_TIME:
                    is_debounce_active = False
                    if comm_queue.empty():
                        comm_queue.put(connector_type)
                        last_queue_time = current_time
                        print(f"[MAIN_THREAD] {DEBOUNCE_TIME}s Debounce passed. Queued '{connector_type}'. 5s cooldown activated.")
            else:
                if is_debounce_active:
                    is_debounce_active = False
                    print("[DEBOUNCE] Connector lost during debounce. Resetting.")
        else:
            if is_debounce_active:
                is_debounce_active = False
                print("[DEBOUNCE] Cooldown active. Debounce state reset.")

        # 결과 표시
        cv2.imshow("Result", color_image)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
