#!/usr/bin/python
# -*- coding: utf-8 -*-
# main.py
from i611_MCS import Robot_emo

# ---- 내가 만든 파일 import ---- #
from utils import *
from comm import get_connector_type, servo_on, door_servo_on, convey_on, magnet_on





# ---- main routine ---- #
def main():
    try:
        rb = setup_robot()

        while True:
            try:
                # ---- 하드웨어 초기화 --- #
                convey_on(4)
                print("[시작] 컨베이어 시작")
                time.sleep(3)
                door_servo_on(7)
                print("[시작] 도어 서보모터 OFF")
                grip_open()
                move_to_home(rb)
                
                
                # ---- 커넥터 인식 루프 ---- #
                connector = None
                while connector is None:
                    connector = get_connector_type()
                    if connector is None:
                        print("[MAIN] 커넥터 미인식 → 재시도 중...")
                        time.sleep(1.0)

                print("[MAIN] Received connector:", connector)

                # ---- 커넥터별 동작 분기 --- #
                if connector == "xt60":
                    print("[SEQUENCE_XT60] XT60 루틴 시작")
                    xt60_routine(rb)

                elif connector == "xt90":
                    print("[SEQUENCE_XT90] XT90 루틴 시작")
                    xt90_routine(rb)

                elif connector == "ec3":
                    print("[SEQUENCE_EC3] EC3 루틴 시작")
                    ec3_routine(rb)

                else:
                    print("[WARNING] Unknown connector type:", connector)

                rb.sleep(0.1)  # 다음 루프로 천천히 넘어감
                
                
            
                
                
                
            except Robot_emo:
                print("EMO(비상정지) 입력")
                break

            except KeyboardInterrupt:
                print("KeyboardInterrupt")
                break

            except Exception as e:
                print("[ERROR]", e)
                break   # 또는 상황에 따라 continue

    finally:
        try:
            print("[FINAL] Returning to home and releasing gripper.")
            grip_open()
            move_to_home(rb)
            servo_on(1)
            magnet_on(3)
            convey_on(5)
            door_servo_on(7)
            
        except:
            pass

        try:
            rb.close()
        except:
            pass


if __name__ == "__main__":
    main()
 