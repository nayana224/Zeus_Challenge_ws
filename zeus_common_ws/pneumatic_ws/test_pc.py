#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import serial

HOST = "0.0.0.0"
PORT = 5005
# SERIAL_PORT = "COM14"
SERIAL_PORT = "/dev/ttyACM0"
BAUD = 115200

def receive_vacuum_on():
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(1)
    s.settimeout(1.0)  
    print("[OK] Serial:", SERIAL_PORT, BAUD)
    print("[OK] Listening:", HOST, PORT)

    try:
        while True:
            try:
                c, a = s.accept()  # 최대 1초 대기
            except socket.timeout:
                continue           # 다시 루프 → 여기서 Ctrl+C가 처리됨
            try:
                c.settimeout(1.0)  
                b = c.recv(1)
                if b in (b'0', b'1'):
                    ser.write(b)
                    print("[RX]", a[0], "->", repr(b))
            except socket.timeout:
                pass
            finally:
                c.close()
    except KeyboardInterrupt:
        print("\n[CLOSED] KeyboardInterrupt")
    finally:
        try: s.close()
        except: pass
        try: ser.close()
        except: pass
        print("[CLOSED] sockets & serial closed")



if __name__ == "__main__":
    receive_vacuum_on()
    