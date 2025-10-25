import numpy as np
import cv2
import pyrealsense2 as rs
import socket,json,time
import threading
import queue 
from collections import Counter 


# JSON TCP 통신 설정
HOST_RECEIVER = "192.168.1.23" 
PORT_JSON_OUT = 10000          

# ----------------------------------------------------
# ⭐️ 전역 변수 정의
last_queue_time = 0.0          # 마지막으로 데이터를 큐에 넣은 시간 (5초 쿨다운 타이머)
comm_queue = queue.Queue(maxsize=1) 

# ⭐️ 1초 디바운스(Debounce) 관리 변수
is_debounce_active = False     # 현재 1초 디바운스 기간이 진행 중인지 여부
debounce_start_time = 0.0      # 디바운스 시작 시간
DEBOUNCE_TIME = 1.0            # ⭐️ 데이터 버리는 시간 (1초)

COOL_DOWN_PERIOD = 5.0         # 전송 후 쿨다운 주기 (5초)
# ----------------------------------------------------


def _send_tcp_message(connector_type):
    """
    실제 TCP 통신을 수행하는 내부 함수
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
    except Exception as e:
        return False
    finally:
        try: s.shutdown(socket.SHUT_RDWR)
        except: pass
        s.close()


def comm_worker():
    """
    백그라운드 통신 워커 스레드 함수
    """
    print("[COMM_WORKER] TCP Worker thread started and waiting for data...")
    while True:
        try:
            connector_type = comm_queue.get(timeout=0.1) 
            
            # 실제 TCP 통신 시도
            if _send_tcp_message(connector_type):
                print("[COMM_WORKER] Communication attempt finished.")
            
            comm_queue.task_done()
            
        except queue.Empty:
            continue
        except Exception as e:
            print(f"[COMM_WORKER] Unexpected error: {e}")
            comm_queue.task_done()
            continue


# 1. RealSense 초기 설정 (15fps 유지)
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
try:
    profile = pipeline.start(config)
except Exception as e:
    print(f"RealSense Error: Failed to start pipeline. Is the device connected? Error: {e}")
    exit()
align = rs.align(rs.stream.color)


# 2. HSV 색상 범위 및 필터 조건 설정 (유지)
lower_yellow = np.array([5, 50, 50])  
upper_yellow = np.array([45, 255, 255]) 
lower_blue = np.array([100, 100, 20])
upper_blue = np.array([140, 255, 180])
ROI_X, ROI_Y, ROI_W, ROI_H = 160, 120, 320, 240
MIN_W_H_YELLOW = 20           
MIN_AREA_YELLOW = 1000        
AREA_THRESHOLD = 6000          
MIN_W_H_BLUE = 15              
MIN_AREA_BLUE = 1000           
kernel = np.ones((5,5),np.uint8)


# ⭐️ 백그라운드 통신 스레드 시작
t_comm = threading.Thread(target=comm_worker)
t_comm.daemon = True 
t_comm.start()


try:
    while True:
        current_time = time.time()
        
        # 프레임 대기 및 정렬 (100ms 타임아웃 예외 처리 포함)
        try:
            frames = pipeline.wait_for_frames(100) 
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
        except RuntimeError:
            continue 
            
        if not color_frame or not depth_frame:
            continue

        # 이미지 처리 및 커넥터 타입 결정
        color_image = np.asanyarray(color_frame.get_data())
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        roi_hsv = hsv[ROI_Y:ROI_Y+ROI_H, ROI_X:ROI_X+ROI_W]
        connector_type = None

        
        # 1. XT 커넥터 (노란색) 객체 검출 (이하 로직 동일)
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
                    text_label = "XT90"
                    connector_type = "xt90" 
                else:
                    text_label = "XT60"
                    connector_type = "xt60" 
                
                # 바운딩 박스 그리기 및 텍스트 표시
                x_global, y_global = ROI_X + x_roi, ROI_Y + y_roi
                cv2.rectangle(color_image, (x_global, y_global), (x_global+w, y_global+h), (0, 255, 255), 2)
                cv2.putText(color_image, text_label, (x_global + w + 5, y_global + h // 2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(color_image, f"Area: {int(area)}", (x_global + w + 5, y_global + h // 2 + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)


        
        # 2. EC3 (남색) 객체 검출 부분 (이하 로직 동일)
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
                    text_label = "EC3"
                    connector_type = "ec3" 

                    # 바운딩 박스 그리기 및 텍스트 표시
                    x_global, y_global = ROI_X + x_roi, ROI_Y + y_roi
                    cv2.rectangle(color_image, (x_global, y_global), (x_global+w, y_global+h), (255, 0, 0), 2)
                    cv2.putText(color_image, text_label, (x_global, y_global - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    cv2.putText(color_image, f"Area: {int(area)}", (x_global, y_global - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        
        # 3. ⭐️ 1초 디바운스 및 전송 로직
        
        if current_time - last_queue_time > COOL_DOWN_PERIOD:
            # 쿨다운이 끝났을 때만 전송/디바운스 시도
            
            if connector_type:
                # A. 커넥터가 인식됨
                if not is_debounce_active:
                    # 1. 디바운스 시작: 커넥터 인식 시작
                    is_debounce_active = True
                    debounce_start_time = current_time
                    print(f"[DEBOUNCE] Detection started: '{connector_type}'. Beginning {DEBOUNCE_TIME}s discard period.")
                    # 1초가 지나기 전까지는 전송하지 않음
                    
                elif current_time - debounce_start_time >= DEBOUNCE_TIME:
                    # 2. 디바운스 종료: 1초 이상 안정적으로 인식됨.
                    is_debounce_active = False # 디바운스 비활성화

                    if comm_queue.empty():
                        comm_queue.put(connector_type)
                        last_queue_time = current_time # 5초 쿨다운 시작
                        print(f"[MAIN_THREAD] {DEBOUNCE_TIME}s Debounce passed. Queued '{connector_type}'. 5s cooldown activated.")
                    
            else:
                # B. 커넥터가 인식되지 않음 (None)
                if is_debounce_active:
                    # 디바운스 중이었는데 커넥터가 사라진 경우, 디바운스 상태를 리셋
                    is_debounce_active = False
                    print("[DEBOUNCE] Connector lost during debounce. Resetting.")
        
        else:
            # 쿨다운 중일 때는 디바운스 상태도 리셋하여 다음 쿨다운 후 새로운 인식을 기다림
            if is_debounce_active:
                is_debounce_active = False
                print("[DEBOUNCE] Cooldown active. Debounce state reset.")

        # 결과 출력 및 종료 조건
        cv2.imshow("Result", color_image)

        key = cv2.waitKey(1) & 0xFF
        if key == 27: 
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()