#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
# 송신
      payload = {
          "color": cname,     # 색깔
          "pose": bool(posture_true),     # True / False
          "center_mm": {"x": float(center_mm[0]), "y": float(center_mm[1]), "z": float(center_mm[2])},    # 중심 좌표
          "angle_deg": lock.angle_deg if lock.active else None,   # 각도
      }
'''


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
    RealSense 카메라로부터 블록 중심 좌표, 각도, 색상 정보를 지속적으로 수신
    연결이 끊기면 자동 재연결함
    """
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((LISTEN_IP, LISTEN_PORT))
    srv.listen(1)
    print("[CAM_Point] Server started, waiting for camera connection at %s:%d ..." % (LISTEN_IP, LISTEN_PORT))

    while True:
        conn, addr = srv.accept()
        print("[CAM_Point] Connected from %s" % (addr,))
        conn.settimeout(2.0)

        buffer = b""
        while True:
            try:
                chunk = conn.recv(4096)
                if not chunk:
                    print("[CAM_Point] Camera disconnected. Waiting again...")
                    break  # 연결이 끊기면 다음 accept로

                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    if not line:
                        continue
                    try:
                        data = json.loads(line.decode('utf-8'))
                    except Exception as e:
                        print("[CAM_Point] JSON parse error:", e)
                        continue

                    if expect_stable and not data.get("stable", False):
                        continue

                    cmm = data.get("center_mm_filtered") or data.get("center_mm")
                    if not cmm:
                        continue

                    x, y, z = float(cmm["x"]), float(cmm["y"]), float(cmm["z"])
                    if not (z_min <= z <= z_max):
                        continue

                    ang = data.get("angle_deg")
                    color = data.get("color")
                    pose = data.get("pose")

                    print("[CAM_Point] got (%.1f, %.1f, %.1f), color=%s" % (x, y, z, color))
                    # 필요한 경우 값을 반환하지 않고 지속적으로 감시하도록 변경 가능
                    return [x, y, z], ang, color, pose

            except socket.timeout:
                # 데이터가 일시적으로 없을 뿐이면 계속 유지
                continue
            except Exception as e:
                print("[CAM_Point] Error:", e)
                break  # 연결 오류 시 다음 클라이언트 대기

        try:
            conn.close()
        except:
            pass
        time.sleep(0.5)  # 재연결 대기








'''
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

def init_cam_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((LISTEN_IP, LISTEN_PORT))
    srv.listen(1)
    print("[CAM_Point] Waiting for camera connection...")
    conn, addr = srv.accept()
    print("[CAM_Point] Connected from %s" % (addr,))
    return srv, conn


def recv_cam_info(conn, expect_stable=False, z_min=130.0, z_max=665.0):
    import socket, json, time

    # Python2 호환: ConnectionError 대체
    try:
        ConnectionError
    except NameError:
        ConnectionError = socket.error

    buf = b""
    while True:
        try:
            chunk = conn.recv(4096)
            if not chunk:
                print("[CAM] Disconnected.")
                raise ConnectionError("Camera disconnected")
            buf += chunk

            # 여러 JSON이 한 번에 붙어서 올 경우 처리
            while b"\n" in buf or b"}{" in buf:
                # ① 먼저 \n 기준으로 자르기
                if b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                else:
                    # ② 개행이 없지만 }{ 으로 이어진 JSON 분리
                    parts = buf.split(b"}{", 1)
                    line = parts[0] + b"}"
                    buf = b"{" + parts[1] if len(parts) > 1 else b""
                
                if not line.strip():
                    continue

                try:
                    data = json.loads(line.decode("utf-8").strip())
                except Exception as e:
                    print("[CAM] JSON parse error:", e)
                    continue

                # 안정 프레임만 필터링
                if expect_stable and not data.get("stable", False):
                    continue

                cmm = data.get("center_mm_filtered") or data.get("center_mm")
                if not cmm:
                    continue

                try:
                    x, y, z = float(cmm["x"]), float(cmm["y"]), float(cmm["z"])
                except Exception as e:
                    print("[CAM] Invalid center_mm data:", e)
                    continue

                if not (z_min <= z <= z_max):
                    continue

                ang = data.get("angle_deg", None)
                color = data.get("color", None)
                print("[CAM_Point] got (%.1f, %.1f, %.1f), color=%s" % (x, y, z, color))

                return [x, y, z], ang, color

        except socket.timeout:
            # 타임아웃 시 재시도
            continue
        except ConnectionError:
            raise
        except Exception as e:
            print("[CAM] recv_cam_info unknown error:", e)
            time.sleep(0.1)
            continue
'''

