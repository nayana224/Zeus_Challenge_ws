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
                send_vacuum_on(0)
                
                # 2) 홈 이동
                move_to_home(rb)
                                
                time.sleep(1)

                # 3) Pick 위치까지 간 후, 공압으로 블록 집기
                P_cam, angle_deg, color = recv_cam_info(expect_stable=True, z_min=130.0, z_max=665.0)
                P_tcp = cam_to_tcp(P_cam)
                pick_sequence(rb, P_tcp)

                # 4) 집은 블록을 Place 상공 위치로 이동
                rotate_and_home(rb, delta_deg=angle_deg)
                
                # 5) 블록 놓는 위치까지 이동
                # - 집은 블록의 color를 통해 목표 위치, 조정할 angle 정보 추출
                # - 따 놓은 TCP포즈를 그대로 쓰며, 회전(rz, ry, rx)은 현재 로봇의 회전값으로 덮어 사용
                eval_tcp_pose, delta_angle = get_target_pose_by_color(color)
                place_target = tcp_pose_with_current_rot(rb, eval_tcp_pose)
                place_sequence(rb, place_target, delta_angle, lift=150.0, approach=30.0)
                
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
            send_vacuum_on(0)
        except:
            pass
        try: 
            rb.close()
        except: 
            pass
            

if __name__ == "__main__":
    main()
 