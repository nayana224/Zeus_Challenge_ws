#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
from i611_MCS import i611Robot, MotionParam, Joint, Position
from i611_common import *
from i611_io import *
from comm import get_connector_type, servo_on, door_servo_on, convey_on, magnet_on



HOME_JOINT = (75.57, 6.02, 69.57, 0.00, 104.41, 75.57)
TOOL_DEF   = (1, 0, 0, 190, 0, 0, 0)   # (툴번호, x,y,z,rz,ry,rx)

# ---- 좌표 Sequence ---- #
p_xt60 = [
    (0, 0, 0, 0, 0, 0),
    (0, 0, 0, 0, 0, 0)
]

p_xt90 = [
    (0, 0, 0, 0, 0, 0),
]

p_ec3 = [
    (0, 0, 0, 0, 0, 0)
]


def setup_robot():
    """로봇 연결 + 기본 모션 파라미터/툴셋팅"""
    rb = i611Robot()
    rb.open()
    IOinit()
    rb.settool(*TOOL_DEF)
    rb.changetool(TOOL_DEF[0]) # tool 1번
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=20, acctime=0.3, dacctime=0.3))
    return rb

def get_current_pose(rb):
    cur_pose = rb.getpos().pos2list()
    x, y, z, rz, ry, rx = cur_pose[:6]
    
    return x, y, z, rz, ry, rx

def print_current_pose(rb, label="NOW"):
    '''현재 위치 Log'''
    try:
        cur_pose = rb.getpos().pos2list()
        x, y, z, rz, ry, rx = cur_pose[:6]
        print("[%s] TCP: x=%.2f mm, y=%.2f mm, z=%.2f mm, rz=%.2f°, ry=%.2f°, rx=%.2f°" %
      (label, x, y, z, rz, ry, rx))
    except Exception as e:
        print("[%s] pose print error: %s" % (label, e))


def move_to_home(rb):
    '''홈 위치로 복귀'''
    rb.motionparam(MotionParam(jnt_speed=10, lin_speed=100, acctime=0.3, dacctime=0.3))
    rb.move(Joint(*HOME_JOINT))
    print("[HOME] Home 위치로 복귀 완료")
    print_current_pose(rb,"Home")



# 자유미션의 그리퍼는 반대로 하게 함
def grip_close():
    '''그리퍼 열기'''
    print("[GRIPPER] 그리퍼 닫기")
    dout(50, '1')
    time.sleep(1)
    dout(50, '0')

def grip_open():
    '''그리퍼 닫기'''
    print("[GRIPPER] 그리퍼 열기")
    dout(48, '1')
    time.sleep(1)
    dout(48, '0')





def xt60_routine(rb):
    print("[XT60_ROUTINE] Started.")
    
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=100, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[0]) # xt60 커넥터 상공으로 위치
    
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=20, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[1]) # xt60 커넥터로 천천히 접근
    magnet_on(2) # 전자석 ON -> 커넥터 change
    
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=100, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[2]) # 상공으로 위치
    
    # 이제 초음파에게 'a'를 수신받을 때까지 계쏙 대기
    # 받으면 전압 측정 단계 시작
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=100, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[3]) # 전압 측정하는 곳 상공으로 위치
    
    
def xt90_routine(rb):
    print("[XT90_ROUTINE] Started.")

def ec3_routine(rb):
    print("[EC3_ROUTINE] Started.")
