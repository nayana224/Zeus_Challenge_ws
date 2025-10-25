#!/usr/bin/python
# -*- coding: utf-8 -*-
import socket
import json

HOST = ''               # 모든 인터페이스에서 수신
PORT_JSON_OUT = 10000   # Vision 프로그램의 송신 포트와 동일해야 함

def get_connector_type(timeout=5.0):
    """
    Vision 측(클라이언트)에서 전송한 JSON 메시지
    {"connector": "xt60"} 형태를 수신하여 connector 문자열을 반환한다.

    Returns:
        str: 'ec3', 'xt60', 'xt90' 중 하나
        None: 수신 실패 또는 잘못된 데이터
    """
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT_JSON_OUT))
    server_socket.listen(1)
    server_socket.settimeout(timeout)

    print(f"[JSON-RX] Waiting for connection on port {PORT_JSON_OUT}...")

    try:
        conn, addr = server_socket.accept()
        print(f"[JSON-RX] Connected by {addr}")

        data = conn.recv(1024).decode('utf-8').strip()
        print(f"[JSON-RX] Raw data: {data}")

        # JSON 파싱
        try:
            parsed = json.loads(data)
            connector_type = parsed.get("connector")
            print(f"[JSON-RX] Received connector type: {connector_type}")
            return connector_type

        except json.JSONDecodeError:
            print("[ERROR] Invalid JSON format.")
            return None

    except socket.timeout:
        print("[TIMEOUT] No connection received within timeout.")
        return None

    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        return None

    finally:
        try:
            conn.close()
        except:
            pass
        server_socket.close()
