#!/usr/bin/python
# -*- coding: utf-8 -*-
# main.py
from i611_MCS import Robot_emo

# ---- 내가 만든 파일 import ---- #
from utils import *
from comm import get_connector_type




def main():
    try:
        rb = setup_robot()

        while True:
            try:
                
                get_connector_type()
                # # 1) 그리퍼/진공 초기화
                # grip_open()
                
                # # 2) 홈 이동
                # move_to_home(rb)
                # print_current_pose(rb, "Now is home pose.") 
                
                # x, y, z, rz, ry, rx = get_current_pose(rb)
                # p1 = Position(x, y, z, rz, ry, rx)
                # rb.line(p1)
                # print_current_pose(rb, "test position") # -> flange 좌표
                
                # p2 = Position(421.72,  -5.26, 469.85,  90.00,   0.00, 180.00) 
                # rb.line(p2)
                # print_current_pose(rb, "tcp position test") # ->
                
                # p3 = p2.offset(dx=+100)
                # rb.line(p3)
                # print_current_pose(rb, "position3")
                
                # rb.line(p2)
                # print_current_pose(rb, "test position home")

                
                
                
            except Robot_emo:
                print("EMO(비상정지) 입력")
                break
            except KeyboardInterrupt:
                print("KeyboardInterrupt")
                break
            except Exception as e:
                print("[ERROR] %s" % {e})
                break   # 또는 상황에 따라 continue 
    finally:
        try:
            grip_open()
            move_to_home(rb)
        except:
            pass
        try: 
            rb.close()
        except: 
            pass
            

if __name__ == "__main__":
    main()
 