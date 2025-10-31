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
    '''
    < xt60[0] 좌표 >
             X/J1,   Y/J2,   Z/J3,  Rz/J4,  Ry/J5,  Rx/J6, P, Mt
    Pos,   702.52,  35.72, 266.61,-179.72,  -0.07, 179.97, 7, 00000000   
    TCP,   702.52,  35.72, 266.61,-179.72,  -0.07, 179.97, 0, 00000000   
    Jnt,    84.74,  46.82,  66.94,   0.02,  66.32,  -5.55   
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
    Pos,   109.48, 649.62, 324.24, -84.34,  -0.21, 179.89, 7, 00000000   
    TCP,   109.48, 649.62, 324.24, -84.34,  -0.21, 179.89, 0, 00000000   
    Jnt,   161.70,  38.88,  72.95,   0.02,  68.40, -23.97   
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
        Position(702.52,  35.72, 266.61,-179.72,  -0.07, 179.97), # 툴 체인지 좌표
        Position(109.48, 649.62, 324.24, -84.34,  -0.21, 179.89) # 커넥터 체결 위치
    ]
    print("[XT60_ROUTINE] Started.")
    convey_on(4)

    # ---- 툴 체인지 루틴 ---- #
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=200, acctime=0.3, dacctime=0.3))
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
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=200, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[1].offset(dz=100, dy=-100)) # 안전한 궤적으로 돌리기
    
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=200, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[0].offset(dz=50)) # xt60 커넥터 상공으로 위치
    print("[툴 체인지 루틴] 커넥터 전자석 상공으로 위치")
    
    
    print("[툴 체인지 루틴] xt60 커넥터 전자석으로 천천히 접근")
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=15, acctime=0.3, dacctime=0.3))
    rb.line(p_xt60[0]) # xt60 커넥터로 천천히 접근
    
    print("[툴 체인지 루틴] 전자석 OFF")
    magnet_on(3) # 전자석 ON -> 커넥터 change
    print("[툴 체인지 루틴] 툴체인지를 위한 딜레이 2초")
    time.sleep(1.5) # 툴체인지를 위한 시간 여유
    
    rb.line(p_xt60[0].offset(dz=50)) # xt60 커넥터 상공으로 위치
    move_to_home(rb)

    
    
    
    
def xt90_routine(rb):
    '''
    <xt90[1]>
             X/J1,   Y/J2,   Z/J3,  Rz/J4,  Ry/J5,  Rx/J6, P, Mt
    Pos,   122.34, 633.53, 320.77,-100.94,   0.70, 179.18, 7, 00000000   
    TCP,   122.34, 633.53, 320.77,-100.94,   0.70, 179.18, 0, 00000000   
    Jnt,   160.06,  37.39,  76.61,   1.02,  65.43,  -9.43   
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
        Position(122.34, 633.53, 320.77,-100.94,   0.70, 179.18) # 커넥터 체결 위치
    ]
    print("[XT60_ROUTINE] Started.")
    convey_on(4)

    # ---- 툴 체인지 루틴 ---- #
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=200, acctime=0.3, dacctime=0.3))
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
    Pos,   104.73, 643.38, 309.83, -90.95,  -0.08, 179.96, 7, 00000000   
    TCP,   104.73, 643.38, 309.83, -90.95,  -0.08, 179.96, 0, 00000000   
    Jnt,   161.93,  38.77,  75.60,   0.02,  65.73, -17.14   
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
    p_ec3 = [
        Position(603.96, 136.98, 264.47, 177.15,  -0.05, 179.99), # 툴 체인지 좌표
        Position(104.73, 643.38, 309.83, -90.95,  -0.08, 179.96) # 커넥터 체결 위치
    ]
    print("[XT60_ROUTINE] Started.")
    convey_on(4)

    # ---- 툴 체인지 루틴 ---- #
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=200, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[0].offset(dz=50)) # xt60 커넥터 상공으로 위치
    print("[툴 체인지 루틴] 커넥터 전자석 상공으로 위치")
    
    print("[툴 체인지 루틴] xt60 커넥터 전자석으로 천천히 접근")
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=15, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[0]) # xt60 커넥터로 천천히 접근
    
    print("[툴 체인지 루틴] 전자석 ON")
    magnet_on(2) # 전자석 ON -> 커넥터 change
    print("[툴 체인지 루틴] 툴체인지를 위한 딜레이 2초")
    time.sleep(2) # 툴체인지를 위한 시간 여유
    
    print("[툴 체인지 루틴] 안전한 위치로 복귀")
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=100, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[0].offset(dz=100)) # 상공으로 위치 후,
    rb.line(p_ec3[1].offset(dz=50)) # xt60 배터리 상공으로 위치 후 대기
    
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
    rb.line(p_ec3[1].offset(dz=100)) # xt60 배터리 상공으로 위치
    time.sleep(1)
    convey_on(4)
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=200, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[1].offset(dz=100, dy=-100)) # 안전한 궤적으로 돌리기
    
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=200, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[0].offset(dz=50)) # xt60 커넥터 상공으로 위치
    print("[툴 체인지 루틴] 커넥터 전자석 상공으로 위치")
    
    
    print("[툴 체인지 루틴] xt60 커넥터 전자석으로 천천히 접근")
    rb.motionparam(MotionParam(jnt_speed=5, lin_speed=15, acctime=0.3, dacctime=0.3))
    rb.line(p_ec3[0]) # xt60 커넥터로 천천히 접근
    
    print("[툴 체인지 루틴] 전자석 OFF")
    magnet_on(3) # 전자석 ON -> 커넥터 change
    print("[툴 체인지 루틴] 툴체인지를 위한 딜레이 2초")
    time.sleep(1.5) # 툴체인지를 위한 시간 여유
    
    rb.line(p_ec3[0].offset(dz=50)) # xt60 커넥터 상공으로 위치
    move_to_home(rb)
