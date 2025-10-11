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
HOME_JOINT = (72.04, 0.09, 75.16, 0.00, 104.75, 72.04)
HOME_TCP = (371.99, -15.29, 664.85,  90.0,  0.0, 180.0)
TOOL_DEF   = (1, 0, 0, 100, 0, 0, 0)   # (툴번호, x,y,z,rz,ry,rx)



# -----------------------------
# 기본 세팅 & 유틸
# -----------------------------
def setup_robot():
    """로봇 연결 + 기본 모션 파라미터/툴셋팅"""
    rb = i611Robot()
    rb.open()
    IOinit()
    rb.settool(*TOOL_DEF)
    rb.changetool(TOOL_DEF[0]) # tool 1번
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=20, acctime=0.3, dacctime=0.3))
    return rb

def grip_open():
    print("gripper open!")
    dout(50, '1'); time.sleep(1); dout(50, '0')

def grip_close():
    print("gripper close!")
    dout(48, '1'); time.sleep(1); dout(48, '0')

def move_to_home(rb):
    rb.motionparam(MotionParam(jnt_speed=35, lin_speed=300, acctime=0.3, dacctime=0.3))
    rb.move(Joint(*HOME_JOINT))
    print("홈 위치 이동 완료")

def print_current_pose(rb, label="NOW"):
    try:
        lst = rb.getpos().pos2list()
        x, y, z, rz, ry, rx = lst[:6]
        print("[%s] TCP: x=%.2f mm, y=%.2f mm, z=%.2f mm, rz=%.2f°, ry=%.2f°, rx=%.2f°" %
      (label, x, y, z, rz, ry, rx))
    except Exception as e:
        print("[%s] pose print error: %s" % (label, e))


def get_current_tcp_rot(rb):
    try:
        lst = rb.getpos().pos2list()
        rz, ry, rx = lst[3:6]
        print("TCP: rz=%.2f°, ry=%.2f°, rx=%.2f°" % (rz, ry, rx))
        return rz, ry, rx
    except Exception as e:
        print("pose print error: %s" % e)
    

# -----------------------------
# 포즈 정렬/변환
# -----------------------------
def make_tcp_vertical(rb):
    """현재 위치에서 TCP 자세를 바닥 기준 수직으로 맞춤"""
    cur = rb.getpos().pos2list()
    x,y,z = cur[0], cur[1], cur[2]
    tgt = Position(x, y, z, 90.0, 0.0, 180.0)
    print("[ABS] set posture → (rz,ry,rx)=(90,0,180)")
    rb.line(tgt)
    rb.settool(*TOOL_DEF); rb.changetool(TOOL_DEF[0])

def cam_to_tcp(P_cam):
    """
    카메라(mm) → TCP(mm) 단순 변환(현장 보정 포함)
    R = diag(-1,-1,1), t = [+32.0, +40.0, -28.0]
    """
    R = [[-1,0,0],[0,-1,0],[0,0,1]]
    t = [35.0-3.0, 30.0+10.0, -28.0]  # 현장 보정값
    x = R[0][0]*P_cam[0] + R[0][1]*P_cam[1] + R[0][2]*P_cam[2] + t[0]
    y = R[1][0]*P_cam[0] + R[1][1]*P_cam[1] + R[1][2]*P_cam[2] + t[1]
    z = R[2][0]*P_cam[0] + R[2][1]*P_cam[1] + R[2][2]*P_cam[2] + t[2]
    return [x, y, z]


def rotate_and_home(rb, delta_deg):
    """툴 Z축 기준으로 현재 자세에서 delta_deg 만큼 '상대 회전'."""
    delta_deg = -delta_deg # 좌표계 고려
    if delta_deg is None:
        print("[ROT] angle None → 회전 생략")
        return
    '''
    rb.move(Joint(72.04, 0.09, 75.16, 0.00, 104.75, 72.04 + delta_deg))
    rb.reljntmove(dj1=-180)
    '''
    try:
        # 1) 증분 회전이 지원되면 이 한 줄이면 끝
        rb.motionparam(MotionParam(jnt_speed=35, lin_speed=350, acctime=0.3, dacctime=0.3))
        rb.move(Joint(72.04, 0.09, 75.16, 0.00, 104.75, 72.04 + delta_deg))
        
        # 2) 현재 TCP 포즈 확인 (특히 RZ)
        lst = rb.getpos().pos2list()
        rz, ry, rx = lst[3:6]
        print("TCP: rz=%.2f°, ry=%.2f°, rx=%.2f°" % (rz, ry, rx))
        
        return
    except TypeError:
        pass
    except Exception as e:
        print("[Rot] toolmove(drz=...) 실패", e)


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
def pick_sequence(rb, P_tcp):
    """
    [픽] 1) XY 평면 접근 → 2) Z로 내려감 → 3) 진공 ON → 4) Z로 복귀
    P_tcp : 상대 toolmove가 아니라, '픽까지 필요한 Δx,Δy,Δz'를 의미(프로젝트 정의 유지)
    """
    rb.asyncm(1)
    
    rb.motionparam(MotionParam(jnt_speed=40, lin_speed=350, acctime=0.3, dacctime=0.3))
    rb.toolmove(dx=P_tcp[0], dy=P_tcp[1], dz=P_tcp[2] - 40)
    
    rb.join()
    rb.asyncm(2)
    
    send_vacuum_on(True)
    
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=30, acctime=0.3, dacctime=0.3))
    rb.toolmove(dx=0, dy=0, dz=30 + 5) # set dz's margine
    time.sleep(0.2) # time delay for picking by vacuum 
        


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
    
    rb.asyncm(2)
    rb.join()
    
    # 흡착 해제
    if send_vacuum_on(1):
        send_vacuum_on(0)
        time.sleep(0.3) # vacuum off delay

    # 상공 복귀
    up_place = Position(-371.85, 15.41, 564.68, -90.0, 0.00, 180.00)
    rb.motionparam(MotionParam(jnt_speed=35, lin_speed=350, acctime=0.3, dacctime=0.3))
    rb.move(up_place)

    # 다시 home으로 복귀
    rb.reljntmove(dj1=+180)
    move_to_home(rb)
    

