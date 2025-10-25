#!/usr/bin/python
# -*- coding: utf-8 -*-
# main.py
from i611_MCS import Robot_emo

# ---- 내가 만든 파일 import ---- #
from utils import *
from comm import get_connector_type, servo_on, door_on, convey_on, magnet_on




def main():
    try:
        rb = setup_robot()

        while True:
            try:
                grip_close()
                move_to_home(rb)
                
                
                p1 = Position(500.68, 229.88,   5.74,-176.63,  -0.13, 179.87)
                p2 = Position(601.28, 129.91, 195.28,-176.38,  -0.28, 179.98)
                p3 = Position(700.71,  30.49, 196.44,-176.31,  -0.29, 179.93)
                
                
                rb.motionparam(MotionParam(jnt_speed=5, lin_speed=20, acctime=0.3, dacctime=0.3))
                rb.line(p1.offset(dz=50))
                rb.line(p1)
                
                
                rb.line(p1.offset(dy=-200, dz=100))
                rb.line(p2.offset(dz=50))
                rb.line(p2)
                
                rb.line(p2.offset(dy=-200, dz=100))
                rb.line(p3.offset(dz=50))
                rb.line(p3)
                
                
                rb.line(p3.offset(dz=50))
                move_to_home(rb)
                
                
                
                '''
                # Vision 측에서 커넥터 타입 수신
                connector = get_connector_type()

                if connector:
                    print("[MAIN] Received connector:", connector)

                    # 커넥터 타입에 따라 동작 분기
                    if connector == "xt60":
                        print("[ACTION] XT60 detected → grip open")
                        grip_open()

                    elif connector == "xt90":
                        print("[ACTION] XT90 detected → grip close")
                        grip_close()
                    

                    elif connector == "ec3":
                        print("[ACTION] EC3 detected → move home")
                        move_to_home(rb)

                    else:
                        print("[WARNING] Unknown connector type:", connector)
                else:
                    print("[MAIN] No connector received. Waiting again...")

                rb.sleep(0.1)  # 루프 간 대기
                '''
                
                
                
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
            grip_close()
            move_to_home(rb)
        except:
            pass

        try:
            rb.close()
        except:
            pass


if __name__ == "__main__":
    main()
 