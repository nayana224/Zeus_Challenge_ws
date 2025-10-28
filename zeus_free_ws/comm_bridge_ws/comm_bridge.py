#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
<PC에서 실행하는 코드>
- [SEND] Robot으로부터 받은 신호('0'~'8')를 Serial 통신으로 MCU에게 보냄
- [SEND] MCU로부터 받은 신호('9','a')를 TCP 통신으로 Robot Controller에게 보냄
- [RECEIVE] TCP 통신으로 Robot Controller 신호 수신
- [RECEIVE] Serial 통신으로 MCU 신호 수신 (thread)
"""

import socket
import serial
import threading
import time

# ---------------- 설정 ----------------
ROBOT_HOST = "0.0.0.0"
PC_PORT = 5005
ROBOT_IP = "192.168.1.23"
ROBOT_PORT = 5006

SERIAL_PORT_0 = "/dev/ttyACM0"
SERIAL_PORT_1 = "/dev/ttyACM1"
BAUD = 115200

controller_socket = None
controller_lock = threading.Lock()
# -------------------------------------


def send_to_controller(msg):
    """로봇 컨트롤러로 TCP 송신"""
    global controller_socket
    try:
        with controller_lock:
            if controller_socket:
                controller_socket.send(msg.encode('utf-8'))
                print(f"[TCP→Controller] '{msg}'")
    except Exception as e:
        print("[ERROR][TCP Send]:", e)
        controller_socket = None


def serial_reader(ser):
    """MCU → PC 방향 수신 스레드 ('9','a' 문자만 수신)"""
    while True:
        try:
            if ser.in_waiting:
                data = ser.read(1).decode('utf-8', errors='ignore')
                if not data:
                    continue

                print(f"[MCU RX] {ser.port}: {repr(data)}")

                # 🔹 MCU가 보낸 문자가 '9' 또는 'a'일 때만 TCP로 전송
                if data in ('9', 'a'):
                    send_to_controller(data)
                else:
                    print(f"[WARN] Ignored MCU char: {repr(data)}")
            time.sleep(0.01)
        except Exception as e:
            print("[ERROR][Serial Read]:", e)
            break


def receive_command_from_robot():
    """로봇 컨트롤러로부터 TCP 명령 수신('0'~'8') → 시리얼로 문자 1개 전송"""
    global controller_socket
    ser_0 = serial.Serial(SERIAL_PORT_0, BAUD, timeout=1)
    ser_1 = serial.Serial(SERIAL_PORT_1, BAUD, timeout=1)

    # 시리얼 수신 스레드 시작
    threading.Thread(target=serial_reader, args=(ser_0,), daemon=True).start()
    threading.Thread(target=serial_reader, args=(ser_1,), daemon=True).start()

    # TCP 서버 설정 (Robot → PC)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((ROBOT_HOST, PC_PORT))
    s.listen(1)
    s.settimeout(1.0)
    print("[OK] Listening:", ROBOT_HOST, PC_PORT)

    try:
        while True:
            try:
                c, a = s.accept()
                with controller_lock:
                    controller_socket = c
                print("[CONNECTED] Robot Controller:", a)
            except socket.timeout:
                continue

            try:
                c.settimeout(1.0)
                while True:
                    data = c.recv(1)  # 한 문자씩만 수신
                    if not data:
                        break

                    msg = data.decode('utf-8', errors='ignore')
                    print(f"[RX] {a[0]} -> '{msg}'")

                    # '0'~'8' 문자만 MCU로 전송
                    if msg in [str(i) for i in range(9)]:
                        ser_0.write(msg.encode('utf-8'))
                        ser_1.write(msg.encode('utf-8'))
                        print(f"[TX→MCU] '{msg}' sent to both serial ports")
                    else:
                        print(f"[WARN] Ignored Robot char: '{msg}'")

            finally:
                with controller_lock:
                    controller_socket = None
                c.close()
                print("[DISCONNECTED] Controller closed.")

    except KeyboardInterrupt:
        print("\n[CLOSED] KeyboardInterrupt")
    finally:
        s.close()
        ser_0.close()
        ser_1.close()
        print("[CLOSED] sockets & serial closed")


# ------------------ 메인 실행부 ------------------
if __name__ == "__main__":
    print("=== [START] PC Bridge: Robot Controller <-> MCU ===")
    try:
        receive_command_from_robot()
    except Exception as e:
        print("[FATAL ERROR]:", e)
    finally:
        print("=== [EXITED] Program Terminated ===")
