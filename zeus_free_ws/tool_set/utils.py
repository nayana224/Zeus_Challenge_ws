#!/usr/bin/python
# -*- coding: utf-8 -*-
# util.py
import time
from i611_MCS import i611Robot, MotionParam, Joint, Position
from i611_common import *
from i611_io import *


# --- 로봇 기본 파라미터 ---
HOME_JOINT = (72.04, 0.09, 75.16, 0.00, 104.75, 72.04)
HOME_TCP = (371.99, -15.29, 664.85,  90.0,  0.0, 180.0)
GRIPPER_DEF   = (1, 0, 0, 190, 0, 0, 0)   # (툴번호, x,y,z,rz,ry,rx)
ELEC_MAGNET_DEF = (1, 0, 0, 100, 0, 0, 0) # (툴번호, x,y,z,rz,ry,rx)



# -----------------------------
# 기본 세팅 & 유틸
# -----------------------------
def setup_tool(rb, TOOL_DEF):
    rb.settool(*TOOL_DEF)
    rb.changetool(TOOL_DEF[0])

def setup_robot():
    """로봇 연결 + 기본 모션 파라미터/툴셋팅"""
    rb = i611Robot()
    rb.open()
    IOinit()
    setup_tool(rb, GRIPPER_DEF)
    '''
    rb.settool(*TOOL_DEF)
    rb.changetool(TOOL_DEF[0]) # tool 1번
    '''
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
    
