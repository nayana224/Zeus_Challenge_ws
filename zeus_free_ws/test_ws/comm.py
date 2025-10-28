#!/usr/bin/python
# -*- coding: utf-8 -*-
import socket, json
'''
< Robot Controller에서 실행하는 코드 >
- [SEND] TCP 통신을 통해 PC에게 MCU 제어 신호를 보낸다.
- [RECEIVE] 뎁스카메라 PC로부터 CAM 인식 정보를 받는다.
- [RECEIVE] MCU->PC로부터 동작 완료 신호를 받는다.
'''

TARGET_PC_HOST = "192.168.1.1" # PC의 Host 번호
TARGET_PC_PORT = 5005 # 수신하는 PC의 Port 번호

CAM_HOST = ""   # 모든 인터페이스에서 수신 받기
CAM_PORT = 10000 # 뎁스카메라 PC로부터 받는 Port 번호

def get_connector_type(timeout=10.0):
    '''커넥터 타입 수신 받기'''
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((CAM_HOST, CAM_PORT))
    server_socket.listen(1)
    server_socket.settimeout(timeout)

    print("[JSON-RX] Waiting for connection on port %d..." % CAM_PORT)

    conn = None
    try:
        conn, addr = server_socket.accept()
        print("[JSON-RX] Connected by %s" % str(addr))

        data = conn.recv(1024).strip()
        print("[JSON-RX] Raw data: %s" % data)

        parsed = json.loads(data)
        connector_type = parsed.get("connector")
        print("[JSON-RX] Received connector type: %s" % connector_type)
        return connector_type

    except socket.timeout:
        print("[TIMEOUT] No connection within %ds." % timeout)
        return None
    except Exception as e:
        print("[ERROR] Unexpected error:", e)
        return None
    finally:
        try:
            if conn:
                conn.close()
        except:
            pass
        server_socket.close()


def receive_from_pc(timeout=None):
    """
    PC(브릿지)로부터 TCP 수신
    - MCU에서 전송된 피드백('9', 'a' 등)을 로봇 컨트롤러에서 받는다.
    - timeout=None이면 무한 대기, 숫자면 초 단위 타임아웃
    """
    HOST = ""            # 모든 인터페이스에서 수신
    PORT = 5006          # 로봇 컨트롤러 수신용 포트 (브릿지 코드의 ROBOT_PORT와 일치)
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    if timeout:
        server_socket.settimeout(timeout)

    print("[TCP-RX] Waiting for data on port %d..." % PORT)

    conn = None
    try:
        conn, addr = server_socket.accept()
        print("[TCP-RX] Connected by", addr)

        data = conn.recv(1024)
        if not data:
            print("[TCP-RX] No data received.")
            return None

        msg = data.decode('utf-8', errors='ignore').strip()
        print("[TCP-RX] Received: '%s'" % msg)
        return msg

    except socket.timeout:
        print("[TIMEOUT] No data within %ds." % timeout)
        return None
    except Exception as e:
        print("[ERROR][TCP-RX]:", e)
        return None
    finally:
        try:
            if conn:
                conn.close()
        except:
            pass
        server_socket.close()




def send_to_pc(msg):
    """공통 TCP 문자열 송신 함수"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.settimeout(1.0)
        s.connect((TARGET_PC_HOST, TARGET_PC_PORT))
        s.send(msg.encode('utf-8'))   # 문자열 → bytes로 전송
        print("[MCU] Sent:", msg)
        return True
    except Exception as e:
        print("[MCU][ERROR]:", e)
        return False
    finally:
        try: s.shutdown(socket.SHUT_RDWR)
        except: pass
        s.close()


def servo_on(v):
    """그리퍼 서보 ON(0)/OFF(1)"""
    # on:'0', off:'1'
    msg = str(v) if v in (0, 1) else None
    if msg is None:
        print("[ERROR] 그리퍼 서보모터 동작 오류:", v)
        return False
    return send_to_pc(msg)


def magnet_on(v):
    """그리퍼 전자석 ON(2)/OFF(3)"""
    # on:'2', off:'3'
    msg = str(v) if v in (2, 3) else None
    if msg is None:
        print("[ERROR] 그리퍼 전자석 동작 오류:", v)
        return False
    return send_to_pc(msg)


def convey_on(v):
    """컨베이어벨트 ON(4)/OFF(5)"""
    # on:'4', off:'5'(추가)
    msg = str(v) if v in (4, 5) else None
    if msg is None:
        print("[ERROR] 컨베이어벨트 동작 오류:", v)
        return False
    elif msg == '4':
        print("[CONVEY] 컨베이어 벨트 동작 시작.")
    elif msg == '5':
        print("[CONVEY] 컨베이어 벨트 동작 멈춤.")
    return send_to_pc(msg)


def door_servo_on(v):
    """도어 서보 ON(6)/OFF(7)"""
    # on:'6', off:'7'
    msg = str(v) if v in (6, 7) else None
    if msg == None:
        print("[ERROR] 도어용 서보모터 동작 오류 :", v)
        return False
    elif msg == '6':
        print("[SERVO_MOTOR] 도어용 서보모터 동작 시작.")
    elif msg == '7':
        print("[SERVO_MOTOR] 도어용 서보모터 동작 멈춤.")
    return send_to_pc(msg)

def volt_measurement(v):
    """전압측정 ON(8)/OFF(9)"""
    # on:'8', off:'9'
    msg = str(v) if v in (8, 9) else None
    if msg == None:
        print("[ERROR] 전압 측정 동작 오류 :", v)
        return False
    elif msg == '8':
        print("[VOLT_MEAS] 전압 측정 시작.")
    elif msg == '9':
        print("[VOLT_MEAS] 전압 측정 멈춤.")
    return send_to_pc(msg)


def ultra_start(v):
    """초음파 센서 측정 ON(a)/OFF(b)"""
    # on:'a', off:'b'
    if v not in ('a', 'b'):
        print("[ERROR] 초음파 센서 제어 파라미터 오류:", v)
        return False
    elif v == 'a':
        print("[ULTRA_SENSOR] 초음파 센서 측정 시작.")
    elif v == 'b':
        print("[ULTRA_SENSOR] 초음파 센서 측정 멈춤.")
    return send_to_pc(v)