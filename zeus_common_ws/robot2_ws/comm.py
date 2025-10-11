#!/usr/bin/python
# -*- coding: utf-8 -*-

# comm.py
import socket, json, time

# --- 환경 설정(필요하면 여기만 바꾸면 됨) ---
HOST_VACUUM = "192.168.1.1" # 이건 윈도우 cmd창에서 "ipconfig" 명령어를 통해 확인 
PORT_VACUUM = 5005

LISTEN_IP   = "192.168.1.23"   # 로봇 Controller IP
LISTEN_PORT = 10000
RECV_TIMEOUT_S = 0.5


def send_vacuum_on(v):
    """진공 ON/OFF: True→'1', False→'0' 를 TCP로 전송"""
    msg = b'1' if v else b'0'
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.settimeout(1.0)
        s.connect((HOST_VACUUM, PORT_VACUUM))
        s.send(msg)
        print("[Vacuum] send msg: %s" % msg)
        return True
    except Exception as e:
        print("[Vacuum] send error: %s" % e)
        return False
    finally:
        try: s.shutdown(socket.SHUT_RDWR)
        except: pass
        s.close()


def recv_cam_info(expect_stable=False, z_min=130.0, z_max=665.0):
    """
    카메라 JSON 한 줄 수신해서 (x,y,z) mm 리턴.
    - expect_stable: (이 송신코드에는 stable 필드가 없으므로) 항상 통과. 호환성만 유지.
    - z_min, z_max: None 주면 해당 필터 비활성화 (예: z_min=None, z_max=None)
    반환: ([x,y,z], angle_deg or None, color or None)
    """
    # NOTE: 이 함수는 송신 스크립트가 기본값(PERSISTENT_CONN=False)인
    # '원샷 연결' 모드에 맞춰, 1개의 JSON 라인만 받고 종료한다.
    # (지속연결 모드여도 첫 유효 라인을 받으면 반환하도록 동작)

    while True:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((LISTEN_IP, LISTEN_PORT))
            srv.listen(1)
            print("[CAM_Point] Listening on %s:%s ..." % (LISTEN_IP, LISTEN_PORT))

            conn, addr = srv.accept()
            print("[CAM_Point] Connected from %s" % (addr,))
            try:
                conn.settimeout(1.0)  # 수신 대기 타임아웃(유휴 시 재시도)
                buf = b""
                while True:
                    try:
                        chunk = conn.recv(4096)
                        if not chunk:
                            break
                        buf += chunk
                        # 여러 줄 들어오면 한 줄씩 처리
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            line = line.strip()
                            if not line:
                                continue

                            # JSON 파싱
                            try:
                                data = json.loads(line.decode("utf-8", errors="replace"))
                            except Exception as e:
                                print("[CAM] JSON parse err: %s" % e)
                                continue
                            
                            # # (1) posture_true
                            # posture_true = data.get("posture_true")
                            # if not posture_true:
                            #     continue

                            # (2) 좌표 추출: center_mm만 사용(송신코드에는 center_mm_filtered 없음)
                            cmm = data.get("center_mm")
                            if not cmm:
                                # 좌표가 없으면 다음 라인 대기
                                continue

                            try:
                                x = float(cmm["x"]); y = float(cmm["y"]); z = float(cmm["z"])
                            except Exception as e:
                                print("[CAM] center_mm parse err: %s" % e)
                                continue

                            # (3) 유효 Z 범위 필터 (원치 않으면 None으로 비활성화)
                            if (z_min is not None and z < z_min) or (z_max is not None and z > z_max):
                                # 범위 밖이면 다음 라인 대기
                                continue

                            # (4) 각도
                            ang = data.get("angle_deg", None)
                            if ang is not None:
                                try:
                                    ang = float(ang)
                                except Exception:
                                    ang = None

                            # (5) 색상
                            color = data.get("color", None)

                            print("[CAM_Point] got camera[mm] = (%.1f, %.1f, %.1f)" % (x, y, z))
                            print("[CAM_Color] got color: %s" % color)

                            return [x, y, z], ang, color

                    except socket.timeout:
                        continue
            finally:
                try: conn.close()
                except: pass
        finally:
            try: srv.close()
            except: pass

        time.sleep(0.1)






'''

def recv_cam_info(expect_stable=False, z_min=130.0, z_max=665.0):
    """
    카메라 JSON 한 줄 수신해서 (x,y,z) mm 리턴.
    - expect_stable=True면 stable 프레임만 받음
    - z 범위 필터링
    """
    while True:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((LISTEN_IP, LISTEN_PORT))
            srv.listen(1)
            print("[CAM_Point] Listening on %s:%s ..." % (LISTEN_IP, LISTEN_PORT))

            conn, addr = srv.accept()
            print("[CAM_Point] Connected from %s" % (addr,))   # addr 전체를 하나의 값으로 취급
            try:
                conn.settimeout(1.0)  # 없으면 재시도
                buf = b""
                while True:
                    try:
                        chunk = conn.recv(4096)
                        if not chunk:
                            break
                        buf += chunk
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            if not line:
                                continue
                            # JSON 파싱
                            try:
                                data = json.loads(line.decode("utf-8"))
                            except Exception as e:
                                print("[CAM] JSON parse err: %s" % e)
                                continue
                            
                            # 1) 안정 프레임 필터
                            if expect_stable and not data.get("stable", False):
                                continue
                            
                            # 2) 좌표[mm] 추출 
                            cmm = data.get("center_mm_filtered") or data.get("center_mm")
                            if not cmm:
                                continue
                            x = float(cmm["x"]); y = float(cmm["y"]); z = float(cmm["z"])
                            
                            # 3) 유효 Z 범위 필터 -> 이거 없어도 되지 않나?
                            if not (z_min <= z <= z_max):
                                continue
                            
                            # 4) 각도
                            ang = data.get("angle_deg", None)
                            print("[CAM_Point] got camera[mm] = (%.1f, %.1f, %.1f)" % (x, y, z)) 
                            
                            # 5) 색상
                            color = data.get("color", None)
                            print("[CAM_Color] got color: %s" % color)
                            
                            
                            return [x, y, z], ang, color

                    except socket.timeout:
                        continue
            finally:
                try: conn.close()
                except: pass
        finally:
            try: srv.close()
            except: pass

        time.sleep(0.1)

'''