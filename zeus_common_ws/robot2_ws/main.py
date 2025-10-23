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
                print_current_pose(rb, "AFTER_HOME")
                
                time.sleep(1)

                # -----------------------------
                # [픽] 카메라로부터 블록 위치 수신 → TCP로 변환 → 픽
                # -----------------------------
                P_cam, angle_deg, color, pose = recv_cam_info(expect_stable=True, z_min=130.0, z_max=665.0)
                P_tcp = cam_to_tcp(P_cam)
                pick_sequence(rb, P_tcp)
                print_current_pose(rb, "After_block_approach")

                # 3) 회전 및 홈 복귀
                result = rotate_and_home(rb, delta_deg=angle_deg, pose=pose)

                # 회전 중단(즉, pose=False)이면 루프 처음으로
                if not result:
                    print("[MAIN] rotate_and_home 중단 → 루프 재시작")
                    continue

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
 