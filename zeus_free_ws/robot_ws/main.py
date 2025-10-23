#!/usr/bin/python
# -*- coding: utf-8 -*-
# main.py
import traceback
from i611_MCS import Robot_emo
from utils import *
from comm import *
from pose_table import *


def main():
    try:
        rb = setup_robot()

        while True:
            try:
                # 1) 그리퍼/진공 초기화
                grip_open()
                # send_vacuum_on(0)
                
                # 2) 홈 이동
                move_to_home(rb)
                print_current_pose(rb, "Now is home pose.") # 어 쓰니야 버겁다...
                
            
                
                rb.relline(drx=30)
                rb.relline(drx=-30)
                rb.relline(dry=30)
                rb.relline(dry=-30)
                rb.relline(drz=30)
                rb.relline(drz=-30)
                
                move_to_home(rb)
                
            except Robot_emo:
                print("EMO(비상정지) 입력")
                break
            except KeyboardInterrupt:
                print("KeyboardInterrupt")
                break
            except Exception as e:
                print("[ERROR] %s" % {e})
                traceback.print_exc()
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
 