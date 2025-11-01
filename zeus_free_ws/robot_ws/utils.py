#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
from i611_MCS import i611Robot, MotionParam, Joint, Position
from i611_common import *
from i611_io import *
from comm import get_connector_type, servo_on, door_servo_on, convey_on, magnet_on, receive_from_pc, volt_measurement
import shared

HOME_JOINT = (75.57, 6.02, 69.57, 0.00, 104.41, 75.57)
# TOOL_DEF   = (1, 0, 30, 200, 0, 0, 0)   # (툴번호, x,y,z,rz,ry,rx)
TOOL_DEF   = (1, 0, 0, 0, 0, 0, 0)   # (툴번호, x,y,z,rz,ry,rx)


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
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=20, acctime=0.3, dacctime=0.3, posture=7))
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
    rb.motionparam(MotionParam(jnt_speed=20, lin_speed=100, acctime=0.3, dacctime=0.3))
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
    '''
    < xt60[0] 좌표 >
         X/J1,   Y/J2,   Z/J3,  Rz/J4,  Ry/J5,  Rx/J6, P, Mt
    Pos,   706.22,  36.80, 266.37,-175.44,   0.07, 179.99, 7, 00000000   
    TCP,   706.22,  36.80, 266.37,-175.44,   0.07, 179.99, 0, 00000000   
    Jnt,    84.85,  47.25,  66.09,   0.02,  66.60,  -9.71   
    Limit, Normal, Normal, Normal, Normal, Normal, Normal

    Singular Status:
    [0] Right/Left
    [0] Upper/Lower elbow
    [0] Wrist flip/non flip
    [0] soft limit
    [0] unreachable point

    '''
    
    '''
    < xt60[1] 좌표 >
         X/J1,   Y/J2,   Z/J3,  Rz/J4,  Ry/J5,  Rx/J6, P, Mt
Pos,   112.17, 645.12, 324.22, -92.12,  -0.10, 179.95, 7, 00000000   
TCP,   112.17, 645.12, 324.22, -92.12,  -0.10, 179.95, 0, 00000000   
Jnt,   161.35,  38.41,  73.84,   0.02,  67.85, -16.54   
Limit, Normal, Normal, Normal, Normal, Normal, Normal

Singular Status:
  [0] Right/Left
  [0] Upper/Lower elbow
  [0] Wrist flip/non flip
  [0] soft limit
  [0] unreachable point


    '''
    
    '''
    < xt60[2] 좌표 >
             X/J1,   Y/J2,   Z/J3,  Rz/J4,  Ry/J5,  Rx/J6, P, Mt
Pos,   711.40,  36.59, 267.70,-171.26,   0.23,-179.96, 7, 00000000   
TCP,   711.40,  36.59, 267.70,-171.26,   0.23,-179.96, 0, 00000000   
Jnt,    84.87,  47.78,  64.77,   0.02,  67.22, -13.88   
Limit, Normal, Normal, Normal, Normal, Normal, Normal

Singular Status:
  [0] Right/Left
  [0] Upper/Lower elbow
  [0] Wrist flip/non flip
  [0] soft limit
  [0] unreachable point

    '''
    
    # ---- 초기화 루틴 ---- #
    p_xt60 = [
        Position(706.22,  36.80, 266.37,-175.44,   0.07, 179.99), # 툴 체인지 좌표
        Position(112.17, 645.12, 324.22, -92.12,  -0.10, 179.95), # 커넥터 체결 위치
        Position(711.40,  36.59, 267.70,-171.26,   0.23,-179.96)
    ]
    print("[XT60_ROUTINE] Started.")
    convey_on(4)

    # ---- 툴 체인지 루틴 ---- #
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=400, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[0].offset(dz=50)) # xt60 커넥터 상공으로 위치
    print("[툴 체인지 루틴] 커넥터 전자석 상공으로 위치")
    
    print("[툴 체인지 루틴] xt60 커넥터 전자석으로 천천히 접근")
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=15, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[0]) # xt60 커넥터로 천천히 접근
    
    print("[툴 체인지 루틴] 전자석 ON")
    magnet_on(2) # 전자석 ON -> 커넥터 change
    print("[툴 체인지 루틴] 툴체인지를 위한 딜레이 2초")
    time.sleep(2) # 툴체인지를 위한 시간 여유
    
    print("[툴 체인지 루틴] 안전한 위치로 복귀")
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=100, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[0].offset(dz=100)) # 상공으로 위치 후,
    rb.line(p_xt60[1].offset(dz=50)) # xt60 배터리 상공으로 위치 후 대기
    
    # ---- MCU 피드백 대기 루프 ---- #
    print("[MCU로부터 수신 루프] Waiting for MCU feedback...")
    while True:
        if shared.mcu_feedback == 'a':  # 예: 초음파 감지 신호
            print("[MCU로부터 수신 루프] MCU로부터 'a' 문자 수신 받음")
            door_servo_on(6) # 초음파 감지 후, 바로 도어용 서보모터 닫기
            shared.mcu_feedback = None   # 처리 후 초기화
            print("[MCU로부터 수신 루프] mcu_feedback 플래그 초기화함")
            break
        time.sleep(0.1)  # CPU 점유 방지 (10Hz)
    time.sleep(3)
    convey_on(5)
    
    # ---- 커넥터 체결 루틴 ---- #
    print("커넥터 체결 루틴 시작.")
    
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=100, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[1].offset(dz=50)) # xt60 커넥터로 천천히 접근
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=10, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[1]) # xt60 커넥터로 천천히 접근
    print("[커넥터 체결 루틴] 커넥터 체결을 위해 천천히 접근")
    
    volt_measurement(8)
    print("[커넥터 체결 루틴] 전압 측정을 위한 딜레이 1초")
    time.sleep(1)
    
    
    # ---- MCU 피드백 대기 루프 ---- #
    print("[MCU로부터 수신 루프] Waiting for MCU feedback...")
    # while True:
    #     if shared.mcu_feedback == 'p':  # 예: 초음파 감지 신호
    #         print("[MCU로부터 수신 루프] MCU로부터 'p' 문자 수신 받음")
    #         door_servo_on(7) # 초음파 감지 후, 바로 도어용 서보모터 닫기
            
    #         shared.mcu_feedback = None   # 처리 후 초기화
    #         print("[MCU로부터 수신 루프] mcu_feedback 플래그 초기화함")
    #         break
    #     # elif
    #     time.sleep(0.1)  # CPU 점유 방지 (10Hz)
    
    door_servo_on(7)
    
        
    
    # ---- 툴 원상복귀 루틴 ---- #
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=10, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[1].offset(dz=100)) # xt60 배터리 상공으로 위치
    time.sleep(1)
    convey_on(4)
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=400, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[1].offset(dz=100, dy=-100)) # 안전한 궤적으로 돌리기
    
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=400, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[2].offset(dz=100)) # xt60 커넥터 상공으로 위치
    print("[툴 체인지 루틴] 커넥터 전자석 상공으로 위치")
    
    
    print("[툴 체인지 루틴] xt60 커넥터 전자석으로 천천히 접근")
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=15, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[2]) # xt60 커넥터로 천천히 접근
    
    print("[툴 체인지 루틴] 전자석 OFF")
    magnet_on(3) # 전자석 ON -> 커넥터 change
    print("[툴 체인지 루틴] 툴체인지를 위한 딜레이 2초")
    time.sleep(1.5) # 툴체인지를 위한 시간 여유
    
    rb.line(p_xt60[2].offset(dz=50)) # xt60 커넥터 상공으로 위치
    move_to_home(rb)

    
    
    
    
def xt90_routine(rb):
    '''
    <xt90[1]>
         X/J1,   Y/J2,   Z/J3,  Rz/J4,  Ry/J5,  Rx/J6, P, Mt
Pos,   124.31, 634.50, 321.14,-100.84,   0.89, 179.16, 7, 00000000   
TCP,   124.31, 634.50, 321.14,-100.84,   0.89, 179.16, 0, 00000000   
Jnt,   159.92,  37.50,  76.37,   1.07,  65.39,  -9.68   
Limit, Normal, Normal, Normal, Normal, Normal, Normal

Singular Status:
  [0] Right/Left
  [0] Upper/Lower elbow
  [0] Wrist flip/non flip
  [0] soft limit
  [0] unreachable point




    '''
    print("[XT90_ROUTINE] Started.")
    # ---- 초기화 루틴 ---- #
    p_xt90 = [
        Position(496.32, 245.10, 265.14, 173.14,   0.87,-178.36), # 툴 체인지 좌표
        Position(124.31, 634.50, 321.14,-100.84,   0.89, 179.16) # 커넥터 체결 위치
    ]
    print("[XT60_ROUTINE] Started.")
    convey_on(4)

    # ---- 툴 체인지 루틴 ---- #
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=400, acctime=0.3, dacctime=0.3))
    rb.line(p_xt90[0].offset(dz=50)) # xt60 커넥터 상공으로 위치
    print("[툴 체인지 루틴] 커넥터 전자석 상공으로 위치")
    
    print("[툴 체인지 루틴] xt60 커넥터 전자석으로 천천히 접근")
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=15, acctime=0.3, dacctime=0.3))
    rb.line(p_xt90[0]) # xt60 커넥터로 천천히 접근
    
    print("[툴 체인지 루틴] 전자석 ON")
    magnet_on(2) # 전자석 ON -> 커넥터 change
    print("[툴 체인지 루틴] 툴체인지를 위한 딜레이 2초")
    time.sleep(2) # 툴체인지를 위한 시간 여유
    
    print("[툴 체인지 루틴] 안전한 위치로 복귀")
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=100, acctime=0.3, dacctime=0.3))
    rb.line(p_xt90[0].offset(dz=100)) # 상공으로 위치 후,
    rb.line(p_xt90[1].offset(dz=50)) # xt60 배터리 상공으로 위치 후 대기
    
    # ---- MCU 피드백 대기 루프 ---- #
    print("[MCU로부터 수신 루프] Waiting for MCU feedback...")
    while True:
        if shared.mcu_feedback == 'a':  # 예: 초음파 감지 신호
            print("[MCU로부터 수신 루프] MCU로부터 'a' 문자 수신 받음")
            door_servo_on(6) # 초음파 감지 후, 바로 도어용 서보모터 닫기
            shared.mcu_feedback = None   # 처리 후 초기화
            print("[MCU로부터 수신 루프] mcu_feedback 플래그 초기화함")
            break
        time.sleep(0.1)  # CPU 점유 방지 (10Hz)
    time.sleep(3)
    convey_on(5)
    
    # ---- 커넥터 체결 루틴 ---- #
    print("커넥터 체결 루틴 시작.")
    
    
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=10, acctime=0.3, dacctime=0.3))
    rb.line(p_xt90[1]) # xt60 커넥터로 천천히 접근
    print("[커넥터 체결 루틴] 커넥터 체결을 위해 천천히 접근")
    
    volt_measurement(8)
    print("[커넥터 체결 루틴] 전압 측정을 위한 딜레이 1초")
    time.sleep(1)
    
    
    # ---- MCU 피드백 대기 루프 ---- #
    print("[MCU로부터 수신 루프] Waiting for MCU feedback...")
    # while True:
    #     if shared.mcu_feedback == 'p':  # 예: 초음파 감지 신호
    #         print("[MCU로부터 수신 루프] MCU로부터 'p' 문자 수신 받음")
    #         door_servo_on(7) # 초음파 감지 후, 바로 도어용 서보모터 닫기
            
    #         shared.mcu_feedback = None   # 처리 후 초기화
    #         print("[MCU로부터 수신 루프] mcu_feedback 플래그 초기화함")
    #         break
    #     # elif
    #     time.sleep(0.1)  # CPU 점유 방지 (10Hz)
    
    door_servo_on(7)
    
        
    
    # ---- 툴 원상복귀 루틴 ---- #
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=10, acctime=0.3, dacctime=0.3))
    rb.line(p_xt90[1].offset(dz=100)) # xt60 배터리 상공으로 위치
    time.sleep(1)
    convey_on(4)
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=200, acctime=0.3, dacctime=0.3))
    rb.line(p_xt90[1].offset(dz=100, dy=-100)) # 안전한 궤적으로 돌리기
    
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=200, acctime=0.3, dacctime=0.3))
    rb.line(p_xt90[0].offset(dz=50)) # xt60 커넥터 상공으로 위치
    print("[툴 체인지 루틴] 커넥터 전자석 상공으로 위치")
    
    
    print("[툴 체인지 루틴] xt60 커넥터 전자석으로 천천히 접근")
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=15, acctime=0.3, dacctime=0.3))
    rb.line(p_xt90[0]) # xt60 커넥터로 천천히 접근
    
    print("[툴 체인지 루틴] 전자석 OFF")
    magnet_on(3) # 전자석 ON -> 커넥터 change
    print("[툴 체인지 루틴] 툴체인지를 위한 딜레이 2초")
    time.sleep(1.5) # 툴체인지를 위한 시간 여유
    
    rb.line(p_xt90[0].offset(dz=50)) # xt60 커넥터 상공으로 위치
    move_to_home(rb)
    
    
    

def ec3_routine(rb):
    '''
    <p_ec3[0] 좌표>
             X/J1,   Y/J2,   Z/J3,  Rz/J4,  Ry/J5,  Rx/J6, P, Mt
    Pos,   603.96, 136.98, 264.47, 177.15,  -0.05, 179.99, 7, 00000000   
    TCP,   603.96, 136.98, 264.47, 177.15,  -0.05, 179.99, 0, 00000000   
    Jnt,    93.48,  37.93,  85.25,   0.02,  56.87,   6.32   
    Limit, Normal, Normal, Normal, Normal, Normal, Normal

    Singular Status:
    [0] Right/Left
    [0] Upper/Lower elbow
    [0] Wrist flip/non flip
    [0] soft limit
    [0] unreachable point

    '''
    
    '''
    <p_ec3[1] 좌표>
         X/J1,   Y/J2,   Z/J3,  Rz/J4,  Ry/J5,  Rx/J6, P, Mt
Pos,   106.99, 639.03, 310.91, -95.43,   0.30, 179.98, 7, 00000000   
TCP,   106.99, 639.03, 310.91, -95.43,   0.30, 179.98, 0, 00000000   
Jnt,   161.61,  38.23,  76.46,   0.10,  65.02, -13.00   
Limit, Normal, Normal, Normal, Normal, Normal, Normal

Singular Status:
  [0] Right/Left
  [0] Upper/Lower elbow
  [0] Wrist flip/non flip
  [0] soft limit
  [0] unreachable point



    '''
    
    '''
    
    
             X/J1,   Y/J2,   Z/J3,  Rz/J4,  Ry/J5,  Rx/J6, P, Mt
Pos,   602.07, 135.78, 267.40, 174.66,  -0.13,-180.00, 7, 00000000   
TCP,   602.07, 135.78, 267.40, 174.66,  -0.13,-180.00, 0, 00000000   
Jnt,    93.38,  37.54,  85.48,   0.02,  57.12,   8.71   
Limit, Normal, Normal, Normal, Normal, Normal, Normal

Singular Status:
  [0] Right/Left
  [0] Upper/Lower elbow
  [0] Wrist flip/non flip
  [0] soft limit
  [0] unreachable point

'''
    print("[EC3_ROUTINE] Started.")
    # ---- 초기화 루틴 ---- #
    p_ec3 = [
        Position(603.96, 136.98, 264.47, 177.15,  -0.05, 179.99), # 툴 체인지 좌표
        Position(106.99, 639.03, 310.91, -95.43,   0.30, 179.98), # 커넥터 체결 위치
        Position(602.07, 135.78, 267.40, 174.66,  -0.13,-180.00)
    ]
    convey_on(4)

    # ---- 툴 체인지 루틴 ---- #
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=400, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[0].offset(dz=50)) # xt60 커넥터 상공으로 위치
    print("[툴 체인지 루틴] 커넥터 전자석 상공으로 위치")
    
    print("[툴 체인지 루틴] xt60 커넥터 전자석으로 천천히 체결")
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=15, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[0]) # xt60 커넥터로 천천히 접근
    
    print("[툴 체인지 루틴] 전자석 ON")
    magnet_on(2) # 전자석 ON -> 커넥터 change
    print("[툴 체인지 루틴] 툴체인지를 위한 딜레이 2초")
    time.sleep(1.5) # 툴체인지를 위한 시간 여유
    
    print("[툴 체인지 루틴] 안전한 위치로 복귀")
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=100, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[0].offset(dz=100)) # 상공으로 위치 후,
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=300, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[0].offset(dx=-100, dz=100))
    rb.line(p_ec3[1].offset(dz=100)) # xt60 배터리 상공으로 위치 후 대기
    
    # ---- MCU 피드백 대기 루프 ---- #
    print("[MCU로부터 수신 루프] Waiting for MCU feedback...")
    while True:
        if shared.mcu_feedback == 'a':  # 예: 초음파 감지 신호
            print("[MCU로부터 수신 루프] MCU로부터 'a' 문자 수신 받음")
            door_servo_on(6) # 초음파 감지 후, 바로 도어용 서보모터 닫기
            shared.mcu_feedback = None   # 처리 후 초기화
            print("[MCU로부터 수신 루프] mcu_feedback 플래그 초기화함")
            break
        time.sleep(0.1)  # CPU 점유 방지 (10Hz)
    time.sleep(3)
    convey_on(5)
    
    # ---- 커넥터 체결 루틴 ---- #
    print("커넥터 체결 루틴 시작.")
    
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=100, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[1].offset(dz=50)) # xt60 커넥터로 천천히 접근
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=10, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[1]) # xt60 커넥터로 천천히 접근
    print("[커넥터 체결 루틴] 커넥터 체결을 위해 천천히 접근")
    
    volt_measurement(8)
    print("[커넥터 체결 루틴] 전압 측정을 위한 딜레이 1초")
    time.sleep(1)
    
    
    # ---- MCU 피드백 대기 루프 ---- #
    print("[MCU로부터 수신 루프] Waiting for MCU feedback...")
    # while True:
    #     if shared.mcu_feedback == 'p':  # 예: 초음파 감지 신호
    #         print("[MCU로부터 수신 루프] MCU로부터 'p' 문자 수신 받음")
    #         door_servo_on(7) # 초음파 감지 후, 바로 도어용 서보모터 닫기
            
    #         shared.mcu_feedback = None   # 처리 후 초기화
    #         print("[MCU로부터 수신 루프] mcu_feedback 플래그 초기화함")
    #         break
    #     # elif
    #     time.sleep(0.1)  # CPU 점유 방지 (10Hz)
    
    door_servo_on(7)
    
    
    
    # ---- 툴 원상복귀 루틴 ---- #
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=10, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[1].offset(dz=50)) # xt60 배터리 상공으로 위치
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=200, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[1].offset(dz=100)) # xt60 배터리 상공으로 위치
    time.sleep(1)
    convey_on(4)
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=200, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[2].offset(dz=100, dy=-100)) # 안전한 궤적으로 돌리기
    
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=200, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[2].offset(dz=50)) # xt60 커넥터 상공으로 위치
    print("[툴 체인지 루틴] 커넥터 전자석 상공으로 위치")
    
    
    print("[툴 체인지 루틴] xt60 커넥터 전자석으로 천천히 접근")
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=15, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[2]) # xt60 커넥터로 천천히 접근
    
    print("[툴 체인지 루틴] 전자석 OFF")
    magnet_on(3) # 전자석 ON -> 커넥터 change
    print("[툴 체인지 루틴] 툴체인지를 위한 딜레이 2초")
    time.sleep(1.5) # 툴체인지를 위한 시간 여유
    
    rb.line(p_ec3[0].offset(dz=50)) # xt60 커넥터 상공으로 위치
    move_to_home(rb)
