#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import time
from comm import handle_mcu_signal   # comm.py에 새로 추가할 함수 호출

SERIAL_PORT = "/dev/ttyACM0"
BAUD = 115200

def listen_serial():
    """MCU로부터 'a' 또는 '9' 수신 후 comm.py에 전달"""
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
    print("[OK] Listening on", SERIAL_PORT)

    try:
        while True:
            if ser.in_waiting:
                data = ser.read(1)
                if not data:
                    continue
                ch = data.decode('utf-8', errors='ignore')
                if ch in ('a', '9'):
                    print("[RX][MCU]", repr(ch))
                    handle_mcu_signal(ch)   # comm.py 쪽 함수로 넘김
                else:
                    print("[WARN] Ignored:", repr(ch))
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[STOP] Serial listening stopped.")
    finally:
        ser.close()

if __name__ == "__main__":
    listen_serial()
