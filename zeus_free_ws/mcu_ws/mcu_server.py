#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import serial

HOST = "0.0.0.0"
PORT = 5005
SERIAL_PORT = "/dev/ttyACM0"
BAUD = 115200

def receive_command():
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
                c, a = s.accept()
            except socket.timeout:
                continue

            try:
                c.settimeout(1.0)
                b = c.recv(1)
                if not b:
                    continue
                if b.isdigit():   # '0'~'9' → 허용
                    ser.write(b)
                    print("[RX]", a[0], "->", repr(b))
                else:
                    print("[WARN] Ignored non-digit:", repr(b))
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
    receive_command()
