#!/usr/bin/python
# -*- coding: utf-8 -*-
# util.py
import time
from i611_MCS import i611Robot, MotionParam, Joint, Position
from i611_common import *
from i611_io import *
from comm import send_vacuum_on  # 진공 ON/OFF 재사용
from pose_table import *


# --- 로봇 기본 파라미터 ---
HOME_JOINT = (75.57, 6.02, 69.57, 0.00, 104.41, 75.57)
HOME_TCP = (371.99, -15.29, 664.85,  90.0,  0.0, 180.0)



def setup_robot():
    '''로봇 연결, TCP 좌표계 설정'''
    rb = i611Robot()
    rb.open()
    IOinit()
    rb.settool(1, 0, 0, 100, 0, 0, 0) # Tool 좌표계 오프셋 설정
    rb.changetool(1) # Tool 좌표계 설정
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=20, acctime=0.3, dacctime=0.3))
    return rb

def grip_open():
    '''그리퍼 열기'''
    dout(50, '1')
    time.sleep(1)
    dout(50, '0')
    print("[GRIPPER] Opened.")

def grip_close():
    '''그리퍼 닫기'''
    dout(48, '1')
    time.sleep(1)
    dout(48, '0')
    print("[GRIPPER] Closed.")

def move_to_home(rb):
    '''홈 위치로 이동'''
    rb.motionparam(MotionParam(jnt_speed=35, lin_speed=300, acctime=0.3, dacctime=0.3))
    rb.move(Joint(*HOME_JOINT))
    rb.join()
    print_current_pose(rb, label="HOME POSE")

def print_current_pose(rb, label="NOW"):
    '''현재 Pose print'''
    try:
        rb.settool(1, 0, 0, 100, 0, 0, 0) # Tool 좌표계 오프셋 설정
        rb.changetool(1) # Tool 좌표계 설정
        
        cur_pose = rb.getpos().pos2list()
        x, y, z, rz, ry, rx = cur_pose[:6]
        print("[%s] TCP: x=%.2f mm, y=%.2f mm, z=%.2f mm, rz=%.2f°, ry=%.2f°, rx=%.2f°" %
      (label, x, y, z, rz, ry, rx))
    except Exception as e:
        print("[%s] 현재 TCP Pose 로깅 에러: %s" % (label, e))

    

# -----------------------------
# 포즈 정렬/변환
# -----------------------------



def make_tcp_vertical(rb):
    """현재 위치에서 TCP 자세를 바닥 기준 수직으로 맞춤"""
    cur_pose = rb.getpos().pos2list()
    x,y,z = cur_pose[0], cur_pose[1], cur_pose[2]
    tgt = Position(x, y, z, 90.0, 0.0, 180.0)
    print("[TCP_Vertical] TCP 자세 바닥 기준 수직: (rz,ry,rx)=(90,0,180)")
    rb.line(tgt)


def cam_to_tcp(P_cam):
    """
    카메라(mm) -> TCP(mm) 좌표계 변환
    R = diag(-1,-1,1), t = [+32.0, +40.0, -28.0]
    """
    R = [[-1,0,0],
         [0,-1,0],
         [0,0,1]]
    t = [35.0-3.0, 30.0+10.0, -28.0]  # 추가 보정값
    x = R[0][0]*P_cam[0] + R[0][1]*P_cam[1] + R[0][2]*P_cam[2] + t[0]
    y = R[1][0]*P_cam[0] + R[1][1]*P_cam[1] + R[1][2]*P_cam[2] + t[1]
    z = R[2][0]*P_cam[0] + R[2][1]*P_cam[1] + R[2][2]*P_cam[2] + t[2]
    return [x, y, z]






# -----------------------------
# eval_position.py 좌표 보정
# -----------------------------
def tcp_pose_with_current_rot(rb, pose6):
    """
    eval_position.py의 TCP 포즈를 그대로 쓰되,
    회전(rz, ry, rx)은 현재 로봇의 회전값으로 덮어쓴다.
    (z 보정 없음)
    """
    x, y, z, _, _, _ = pose6
    rz, ry, rx = rb.getpos().pos2list()[3:6]
    return (float(x), float(y), float(z), float(rz), float(ry), float(rx))


# -----------------------------
# 픽 & 플레이스 동작
# -----------------------------
'''
def pick_sequence(rb, P_tcp):
    """
    [픽] 1) XY 평면 접근 → 2) Z로 내려감 → 3) 진공 ON → 4) Z로 복귀
    P_tcp : 상대 toolmove가 아니라, '픽까지 필요한 Δx,Δy,Δz'를 의미(프로젝트 정의 유지)
    """
    # 예측 동작 ON
    rb.asyncm(1)
    
    
    rb.motionparam(MotionParam(jnt_speed=40, lin_speed=350, acctime=0.3, dacctime=0.3))
    rb.toolmove(dx=P_tcp[0], dy=P_tcp[1], dz=P_tcp[2] - 40)
    rb.join()
    print_current_pose(rb, "블록 집기 전 상공 위치")
    
    # 예측 동작 OFF
    rb.join()
    rb.asyncm(2)
    
    send_vacuum_on(True)
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=30, acctime=0.3, dacctime=0.3))
    rb.toolmove(dx=0, dy=0, dz=30 + 5) # dz값은 계속 보정해야 집기가 원활함
    time.sleep(0.3) # 공압으로 집기까지 딜레이 
    rb.join()
    print_current_pose(rb, "블록 집기 완료")
'''
def pick_sequence(rb, P_tcp):
    """
    [픽] 카메라 절대 좌표로 직접 이동
    """
    # 예측 동작 ON
    rb.asyncm(1)

    # 현재 자세 유지한 채 절대 포즈 이동
    cur_pose = rb.getpos().pos2list()
    x0, y0, z0, rz, ry, rx = cur_pose[:6]

    # 목표 절대 위치 (Home 기준 좌표)
    x, y, z = P_tcp
    p_up = Position(x, y, z + 40, rz, ry, rx)
    p_dn = Position(x, y, z, rz, ry, rx)

    rb.motionparam(MotionParam(jnt_speed=40, lin_speed=350, acctime=0.3, dacctime=0.3))
    rb.line(p_up)
    rb.join()

    print_current_pose(rb, "블록 집기 전 상공 위치")

    send_vacuum_on(True)
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=30, acctime=0.3, dacctime=0.3))
    rb.line(p_dn)
    rb.join()

    time.sleep(0.3)
    print_current_pose(rb, "블록 집기 완료")

    # 다시 위로 복귀
    rb.line(p_up)
    rb.join()

    rb.asyncm(2)

    
def wrap_deg(a):
    return ((a + 180.0) % 360.0) - 180.0   # 필요하면 범위 정리(−180~+180)
    
def rotate_and_home(rb, delta_deg, lift=150.0):
    """툴 Z축 기준 회전을 하기 전에 월드 Z로 lift만큼 올린 뒤,
       J1=-180, J6=J6+delta_deg 를 '한 방'에 이동."""
    if delta_deg is None:
        print("[ROT] angle None → 회전 생략")
        return

    # 좌표계 보정(필요 시)
    delta = -float(delta_deg)
    
    
    '''
    # 1) 현재 TCP 포즈에서 위로 lift(mm)
    cur_pose = rb.getpos().pos2list()       # [x,y,z,rz,ry,rx,...]
    x, y, z, rz, ry, rx = cur_pose[:6]
    p_up = Position(x, y, z + float(lift), rz, ry, rx)
    '''

    # 2) 목표 조인트(예: 홈 기준에서 J1/J6만 보정)
    j1, j2, j3, j4, j5, j6 = 72.04, 0.09, 75.16, 0.00, 104.75, 72.04
    j1 = wrap_deg(j1 - 180.0)
    j6 = wrap_deg(j6 + delta)

    # 예측 동작 ON
    rb.asyncm(1)
    
    try:
        print("블록 집고 Place 위치로 이동 시작")
        rb.motionparam(MotionParam(jnt_speed=35, lin_speed=350, acctime=0.3, dacctime=0.3))

        # (a) 먼저 위로: Position에는 line()을 써야 함
        # rb.line(p_up)
        
        # 1) 현재 TCP 위치에서 위로 lift[mm]
        rb.relline(dz=lift)
        rb.join()  # 조인트 모드로 전환하기 전에 동기화

        # (b) 조인트 목표로 '동시에' 이동 (J1/J6 적용)
        rb.move(Joint(j1, j2, j3, j4, j5, j6))
        rb.join()

        # 상태 로그
        print_current_pose(rb, "Place 위치까지 이동")

    except Exception as e:
        print("[rotate_and_home] 실패:", e)
        raise
    finally:
        rb.asyncm(2)  # 구간 끝나면 해제
        


def place_sequence(rb, target_tcp, delta_angle, lift=200.0, approach=30.0):
    # 여기에 파라미터 delta_angle을 넣어야 하나?
    """
    [플레이스] 절대 TCP 타깃으로 안전하게 내려놓는 동작:
      1) 타깃 상공(lift)까지 절대 이동(line)
      2) 접근 거리만큼 천천히 하강(line)
      3) (옵션) 진공 OFF
      4) 상공으로 복귀(line)

    target_tcp : (x,y,z,rz,ry,rx) → 절대 TCP 좌표
    """
    x, y, z, rz, ry, rx = target_tcp
    z = float(z) - 100.0
    
    # 목표한 블록 위치의 각도 보정
    rz = rz + float(delta_angle)

    rb.asyncm(1)
    
    # 접근
    p_down  = Position(x, y, z + float(approach), rz, ry, rx)
    rb.motionparam(MotionParam(jnt_speed=35, lin_speed=350, acctime=0.3, dacctime=0.3))
    rb.move(p_down)
    
    # 실제로 놓기
    p_place = Position(x, y, z, rz, ry, rx)
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=40, acctime=0.2, dacctime=0.3))
    rb.move(p_place)
    
    
    rb.join()
    
    # 흡착 해제
    if send_vacuum_on(1):
        send_vacuum_on(0)
        time.sleep(0.3) # vacuum off delay

    # 3) 현재 자세에서 '월드 Z로' 위로 먼저
    cur = rb.getpos().pos2list()
    x, y, z, rz, ry, rx = cur[:6]
    rb.motionparam(MotionParam(jnt_speed=35, lin_speed=350, acctime=0.3, dacctime=0.3))
    rb.line(Position(x, y, z + 120.0, rz, ry, rx))  # ↑ 위로 120mm
   
    rb.move(Joint(*HOME_JOINT))
    
    rb.asyncm(2)
    rb.join()
    

