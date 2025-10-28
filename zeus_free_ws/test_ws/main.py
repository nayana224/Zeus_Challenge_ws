#!/usr/bin/python
# -*- coding: utf-8 -*-
# main.py
from i611_MCS import Robot_emo
import threading
import time

# ---- 내가 만든 파일 import ---- #
from utils import *
from comm import get_connector_type, servo_on, door_servo_on, convey_on, magnet_on, receive_from_pc, volt_measurement
import shared

def camera_listener():
    # global connector_type
    while shared.running:
        c = get_connector_type(timeout=5.0)
        if c is not None:
            print("[CAM] Detected connector:", c)
            shared.connector_type = c  # 전역 변수에 저장
        time.sleep(0.1)

def mcu_listener():
    # global mcu_feedback
    while shared.running:
        f = receive_from_pc(timeout=2.0)
        if f is not None:
            print("[MCU] Feedback:", f)
            shared.mcu_feedback = f  # 전역 변수에 저장
        time.sleep(0.1)



# ---- main routine ---- #
def main():
    # global running
    rb = None
    
    try:
        # ---- 로봇 초기화 ---- #
        rb = setup_robot()


        
        while shared.running:
            try:               
                magnet_on(2)
                      
                
            except Robot_emo:
                print("[EMO] 비상정지 감지.")
                shared.running = False
                break

            except KeyboardInterrupt:
                print("[CTRL+C] 사용자 중단.")
                shared.running = False
                break

            except Exception as e:
                print("[ERROR]", e)
                shared.running = False
                break

    finally:
        print("[FINAL] 안전 종료 절차 시작.")
        try:
            shared.running = False
            # grip_open()
            # convey_on(5)
            # magnet_on(3)
            if rb:
                # move_to_home(rb)
                rb.close()
        except Exception as e:
            print("[FINAL][ERROR]", e)
        print("[FINAL] 모든 리소스 해제 완료.")


if __name__ == "__main__":
    main()
 