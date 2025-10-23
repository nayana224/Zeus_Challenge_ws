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
        srv, conn = init_cam_server()

        while True:
            try:
                # 1) 그리퍼/진공 초기화
                grip_open()
                send_vacuum_on(0)
                
                # 2) 홈 이동
                move_to_home(rb)
                time.sleep(2)

                
                P_cam, angle_deg, color = recv_cam_info(conn, expect_stable=True)
                P_tcp = cam_to_tcp(P_cam)
                pick_sequence(rb, P_tcp)

                # 3) 홈 복귀
                rotate_and_home(rb, delta_deg=angle_deg)
                print_current_pose(rb, "After block picked")
                
                
                # 8) 목표 도안 home
                rb.motionparam(MotionParam(jnt_speed=40, lin_speed=20, acctime=0.3, dacctime=0.3))
                print_current_pose(rb, "Picking Sequence")
                
                # 9) 집은 블록의 color을 통해 목표 위치, 조정할 angle 정보 추출
                eval_tcp_pose, delta_angle = get_target_pose_by_color(color)
                
                # 10) 따 놓은 TCP 포즈를 그대로 쓰되, 회전(rz, ry, rx)은 현재 로봇의 회전값으로 덮어씀
                place_target = tcp_pose_with_current_rot(rb, eval_tcp_pose)
                
                # 현장 보정으로 z를 100 낮춰서 플레이스
                place_sequence(rb, place_target, delta_angle, lift=150.0, approach=30.0)
            
            
            except Robot_emo:
                print("EMO(비상정지) 입력")
                break
            except socket.error:
                print("[WARN] Camera disconnected, waiting for reconnect...")
                srv.close()
                time.sleep(1)
                srv, conn = init_cam_server()
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
 