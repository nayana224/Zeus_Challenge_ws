#!/usr/bin/python
# -*- coding: utf-8 -*-
import socket, json

HOST_MCU = "192.168.1.1" # 이건 윈도우 cmd창에서 "ipconfig" 명령어를 통해 확인 
PORT_MCU = 5005

HOST_CAM = ""   # 모든 인터페이스에서 수신
PORT_CAM = 10000

def get_connector_type(timeout=10.0):
    '''커넥터 타입 수신 받기'''
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST_CAM, PORT_CAM))
    server_socket.listen(1)
    server_socket.settimeout(timeout)

    print("[JSON-RX] Waiting for connection on port %d..." % PORT_CAM)

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




def _send_mcu(msg):
    """공통 TCP 송신 함수"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.settimeout(1.0)
        s.connect((HOST_MCU, PORT_MCU))
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
    """그리퍼 서보 ON/OFF"""
    # on:'0', off:'1'
    msg = str(v) if v in (0, 1) else None
    if msg is None:
        print("[ERROR] 그리퍼 서보모터 동작 오류:", v)
        return False
    return _send_mcu(msg)


def magnet_on(v):
    """그리퍼 전자석 ON/OFF"""
    # on:'2', off:'3'
    msg = str(v) if v in (2, 3) else None
    if msg is None:
        print("[ERROR] 그리퍼 전자석 동작 오류:", v)
        return False
    return _send_mcu(msg)


def convey_on(v):
    """컨베이어벨트 ON/OFF"""
    # on:'4', off:'5'(추가)
    msg = str(v) if v in (4, 5) else None
    if msg is None:
        print("[ERROR] 컨베이어벨트 동작 오류:", v)
        return False
    return _send_mcu(msg)


def door_servo_on(v):
    """도어 서보 ON/OFF"""
    # on:'6', off:'7'
    msg = str(v) if v in (6, 7) else None
    if msg is None:
        print("[ERROR] 도어용 서보모터 동작 오류 :", v)
        return False
    return _send_mcu(msg)

def volt_measurement(v):
    """전압측정 ON/OFF"""
    # on:'8', off:'9'
    msg = str(v) if v in (8, 9) else None
    if msg is None:
        print("[ERROR] 전압 측정 동작 오류 :", v)
        return False
    return _send_mcu(msg)


def ultra_start(v):
    """초음파 센서 측정 ON/OFF"""
    # on:'a', off:'b'
    if v not in ('a', 'b'):
        print("[ERROR] 초음파 센서 제어 파라미터 오류:", v)
        return False
    return _send_mcu(v)