#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
from i611_MCS import i611Robot, MotionParam, Joint, Position
from i611_common import *
from i611_io import *

# ---- 내가 만든 파일 import ---- #
from comm import send_vacuum_on  # 진공 ON/OFF 재사용
from pose_table import *


HOME_JOINT = Joint(75.04,   6.73,  58.85,   0.00, 114.42,  73.04)
HOME_POSE_1 = Position(406.79,  -5.16, 620.01+100,  92.00,   0.00, 180.00) # -> 이걸로 하면 안 됨
HOME_POSE_2 = Position(406.79,  -5.16, 720.01,  92.00,   0.00, 180.00)
TOOL_DEF   = (1, 0, 0, 100, 0, 0, 0)   # (툴번호, x,y,z,rz,ry,rx)


def setup_robot():
    """로봇 연결 + 기본 모션 파라미터/툴셋팅"""
    rb = i611Robot()
    rb.open()
    IOinit()
    rb.settool(*TOOL_DEF)
    rb.changetool(TOOL_DEF[0]) # tool 1번
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=20, acctime=0.3, dacctime=0.3, posture=7))
    return rb

def print_current_pose(rb, label="NOW"):
    '''현재 위치 Log'''
    try:
        cur_pose = rb.getpos().pos2list()
        x, y, z, rz, ry, rx = cur_pose[:6]
        print("[%s] TCP: x=%.2f mm, y=%.2f mm, z=%.2f mm, rz=%.2f°, ry=%.2f°, rx=%.2f°" %
      (label, x, y, z, rz, ry, rx))
    except Exception as e:
        print("[%s] pose print error: %s" % (label, e))

def grip_open():
    '''그리퍼 열기'''
    print("[GRIPPER] 그리퍼 열기")
    dout(50, '1')
    time.sleep(1)
    dout(50, '0')

def grip_close():
    '''그리퍼 닫기'''
    print("[GRIPPER] 그리퍼 닫기")
    dout(48, '1')
    time.sleep(1)
    dout(48, '0')

def move_to_home(rb):
    '''홈 위치로 복귀'''
    rb.motionparam(MotionParam(jnt_speed=60, lin_speed=100, acctime=0.3, dacctime=0.3, posture=7))
    # rb.move(HOME_JOINT)
    rb.move(HOME_JOINT)
    print("조인트 기반 완료")
 
    print("[HOME] Home 위치로 복귀 완료")
    print_current_pose(rb,"Home")





# -----------------------------
# 포즈 정렬/변환
# -----------------------------

def cam_to_tcp(P_cam):
    """
    카메라[mm] -> TCP[mm] 좌표계 변환
    R = diag(-1,-1,1), t = [+32.0, +40.0, -28.0]
    """
    R = [[-1,0,0],
         [0,-1,0],
         [0,0,1]]
    t = [35.0-3, 30.0+19.0, -28.0]  # 현장 보정값
    x = R[0][0]*P_cam[0] + R[0][1]*P_cam[1] + R[0][2]*P_cam[2] + t[0]
    y = R[1][0]*P_cam[0] + R[1][1]*P_cam[1] + R[1][2]*P_cam[2] + t[1]
    z = R[2][0]*P_cam[0] + R[2][1]*P_cam[1] + R[2][2]*P_cam[2] + t[2]
    return [x, y, z]


# def cam_to_tcp(P_cam):
#     """
#     카메라[mm] -> TCP[mm] 좌표계 변환
#     R = diag(-1,-1,1), t = [+32.0, +40.0, -28.0]
#     """
#     R = [[-1,0,0],
#          [0,-1,0],
#          [0,0,1]]
#     # t = [35.0 - 4 + 5, 40 + 8 -7, -28.0]  # 현장 보정값 -> x에 +0
#     t = [35.0 - 10, 40 + 8 -12, -28.0]
#     x = R[0][0]*P_cam[0] + R[0][1]*P_cam[1] + R[0][2]*P_cam[2] + t[0]
#     y = R[1][0]*P_cam[0] + R[1][1]*P_cam[1] + R[1][2]*P_cam[2] + t[1]
#     z = R[2][0]*P_cam[0] + R[2][1]*P_cam[1] + R[2][2]*P_cam[2] + t[2]
#     return [x, y, z]


def wrap_deg(a):
    '''이상한 범위의 각도로 가는 것 방지'''
    return ((a + 180.0) % 360.0) - 180.0



def tcp_pose_with_current_rot(rb, pose6):
    """
    eval_position.py의 TCP 포즈를 그대로 쓰되,
    회전(rz, ry, rx)은 현재 로봇의 회전값으로 덮어씀.
    (z 보정 없음)
    """
    x, y, z, _, _, _ = pose6
    rz, ry, rx = rb.getpos().pos2list()[3:6]
    return (float(x), float(y), float(z), float(rz), float(ry), float(rx))


# -----------------------------
# 픽 & 플레이스 동작
# -----------------------------

        

def pick_sequence(rb, P_tcp):
    """Pick 위치까지 도달 & 공압으로 집기"""
    # [픽] 1) XY 평면 접근 → 2) Z로 내려감 → 3) 진공 ON → 4) Z로 복귀
    # P_tcp : 상대 toolmove가 아니라, '픽까지 필요한 Δx,Δy,Δz'를 의미(프로젝트 정의 유지)
    rb.asyncm(1)
    
    # rb.motionparam(MotionParam(jnt_speed=40, lin_speed=350, acctime=0.3, dacctime=0.3))
    rb.motionparam(MotionParam(jnt_speed=70, lin_speed=600, acctime=0.3, dacctime=0.3, posture=7))
    rb.toolmove(dx=P_tcp[0], dy=P_tcp[1], dz=P_tcp[2] - 40)
    
    rb.join()
    rb.asyncm(2)
    
    send_vacuum_on(True)
    time.sleep(0.15)  # 흡착 압착/실링 안정화

    # rb.motionparam(MotionParam(jnt_speed=5, lin_speed=30, acctime=0.3, dacctime=0.3))
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=30, acctime=0.3, dacctime=0.3, posture=7))
    rb.toolmove(dx=0, dy=0, dz=35+1)  
    time.sleep(0.2) 
    print_current_pose(rb, "공압 On & Pick 위치 도달")


def rotate_and_home(rb, delta_deg, lift=150.0):
    """Place 상공 위치로 도달"""
    # 툴 Z축 기준 회전을 하기 전에 월드 Z로 lift만큼 올린 뒤,
    # J1=-180, J6=J6+delta_deg 를 '한 방'에 이동.
    # Exception: pose = False인 경우, lift만큼 올린 후 블록 떨어뜨림

    # 각도 보정
    delta_deg = -float(delta_deg)

    # 1) 현재 TCP 포즈에서 위로 lift(mm)
    cur = rb.getpos().pos2list() # [x,y,z,rz,ry,rx, ...]
    x, y, z, rz, ry, rx = cur[:6]
    p_up = Position(x, y, z + float(lift), rz, ry, rx)

    # 2) 목표 조인트(예: 홈 기준에서 J1/J6만 보정)
    j1, j2, j3, j4, j5, j6 = 72.04, 0.09, 75.16, 0.00, 104.75, 72.04
    j1 = wrap_deg(j1 - 180.0)
    j6 = wrap_deg(j6 + delta_deg)

    rb.asyncm(1)
    try:
        rb.motionparam(MotionParam(jnt_speed=70, lin_speed=500, acctime=0.3, dacctime=0.3, posture=7))
        
        # (a) 먼저 위로: Position에는 line()을 써야 함
        rb.line(p_up)
        rb.join()  # 조인트 모드로 전환하기 전에 동기화

        # (b) 조인트 목표로 '동시에' 이동 (J1/J6 적용)
        rb.move(Joint(j1, j2, j3, j4, j5, j6))
        rb.join()

        print_current_pose(rb, "Place 상공에 도달")


    except Exception as e:
        print("[Error] rotate_and_home 실패:", e)
        raise
    finally:
        rb.asyncm(2)  # 구간 끝나면 해제


def place_sequence(rb, target_tcp, delta_angle, lift=200.0, approach=30.0):
    '''블록을 목표 위치로 Place 후, Home 위치로 복귀'''
    
    # [플레이스] 절대 TCP 타깃으로 안전하게 내려놓는 동작:
    # 1) 타깃 상공(lift)까지 절대 이동(line)
    # 2) 접근 거리만큼 천천히 하강(line)
    # 3) (옵션) 진공 OFF
    # 4) 상공으로 복귀(line)
    # target_tcp : (x,y,z,rz,ry,rx) → 절대 TCP 좌표


    # 오프셋 제거??????????????????????????????????? -> 좌표테이블의 z축 그대로 -100해주기??????????????
    x, y, z, rz, ry, rx = target_tcp
    x = float(x); y = float(y); z = float(z)  # 필요 시 실수형 변환
    z = float(z) - 90.0

    # 목표한 블록 위치의 각도 보정
    rz = rz + float(delta_angle)

    rb.asyncm(1)
    
    # 접근
    p_down  = Position(x, y, z + float(approach), rz, ry, rx)
    rb.motionparam(MotionParam(jnt_speed=60, lin_speed=700, acctime=0.3, dacctime=0.3, posture=7))
    rb.move(p_down)
    # rb.line(p_down)
    
    # 실제로 놓기
    p_place = Position(x, y, z, rz, ry, rx)
    rb.motionparam(MotionParam(jnt_speed=20, lin_speed=30, acctime=0.2, dacctime=0.3, posture=7))
    # rb.move(p_place)
    rb.line(p_place)
    
    
    rb.join()
    
    # 흡착 해제
    if send_vacuum_on(1):
        send_vacuum_on(0)
        time.sleep(0.6) # vacuum off delay

    # 3) 현재 자세에서 '월드 Z로' 위로 먼저
    cur = rb.getpos().pos2list()
    x, y, z, rz, ry, rx = cur[:6]
    rb.motionparam(MotionParam(jnt_speed=60, lin_speed=600, acctime=0.3, dacctime=0.3, posture=7))
    rb.line(Position(x, y, z + 50.0, rz, ry, rx))  # 위로 120mm
   
    rb.join()
    rb.move(HOME_JOINT)
    
    rb.asyncm(2)
    rb.join()
    

