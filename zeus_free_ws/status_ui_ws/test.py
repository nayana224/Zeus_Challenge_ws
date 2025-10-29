import sys
import socket
import threading

from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtGui import QFont


# ─────────────────────────
# 1) 네트워크 수신 -> 시그널로 GUI에 전달할 헬퍼
# ─────────────────────────
class StatusSignal(QObject):
    status_received = pyqtSignal(str)  # "p", "f", etc.


# ─────────────────────────
# 2) PyQt 메인 윈도우
# ─────────────────────────
class StatusWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("상태 모니터")
        self.setGeometry(600, 300, 320, 180)

        self.label = QLabel("대기 중", self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFont(QFont("맑은 고딕", 28, QFont.Bold))
        self.label.setStyleSheet("color: gray;")

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

    def update_status_label(self, value: str):
        """
        value == 'p'  -> 정상(파랑)
        value == 'f'  -> 불량(빨강)
        기타           -> 알 수 없음(회색)
        """
        if value == 'p':
            self.label.setText("정상")
            self.label.setStyleSheet("color: blue;")
        elif value == 'f':
            self.label.setText("불량")
            self.label.setStyleSheet("color: red;")
        else:
            self.label.setText("알 수 없음")
            self.label.setStyleSheet("color: gray;")


# ─────────────────────────
# 3) TCP 서버 스레드
# ─────────────────────────
class TCPServerThread(threading.Thread):
    """
    간단한 TCP 서버.
    - 지정한 IP/PORT로 bind해서 listen
    - 클라이언트가 연결되면 루프 돌면서 데이터 수신
    - 받은 데이터(문자열)에서 'p' 또는 'f'를 뽑아서 시그널로 전송
    """
    def __init__(self, host_ip: str, port: int, sig: StatusSignal):
        super().__init__(daemon=True)
        self.host_ip = host_ip
        self.port = port
        self.sig = sig
        self.stop_flag = False

    def run(self):
        # 서버 소켓 생성
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 재사용 옵션 (프로그램 재시작 시 "Address already in use" 줄이기 위해)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            server_sock.bind((self.host_ip, self.port))
            server_sock.listen(1)
            print(f"[TCP] Listening on {self.host_ip}:{self.port}")

            while not self.stop_flag:
                # 클라이언트 접속 대기
                client_sock, addr = server_sock.accept()
                print(f"[TCP] Connected from {addr}")

                # 클라이언트 소켓 처리
                with client_sock:
                    while not self.stop_flag:
                        data = client_sock.recv(1024)
                        if not data:
                            # 클라이언트 쪽에서 연결 끊음
                            print("[TCP] Client disconnected")
                            break

                        # 받은 raw 바이트 -> 문자열로
                        try:
                            msg = data.decode(errors='ignore').strip()
                        except UnicodeDecodeError:
                            msg = ""

                        print(f"[TCP] RX raw: {repr(msg)}")

                        # "p"나 "f"만 관심
                        # 만약 여러 글자 들어올 수도 있으니 한 글자씩 체크
                        for ch in msg:
                            if ch in ('p', 'f'):
                                # PyQt GUI thread에게 전달
                                self.sig.status_received.emit(ch)

        except OSError as e:
            print(f"[TCP] Socket error: {e}")
        finally:
            server_sock.close()
            print("[TCP] Server socket closed.")

    def stop(self):
        self.stop_flag = True
        # 실제로 즉시 unblock시키려면 더 세련된 종료처리가 필요할 수 있으나
        # 여기서는 간단하게 stop_flag만 둡니다.


# ─────────────────────────
# 4) 메인 코드
# ─────────────────────────
if __name__ == "__main__":
    HOST_IP = "192.168.1.1"   # 요청하신 IP
    HOST_PORT = 5007         # 임의 포트 (필요시 바꾸시면 됩니다)

    app = QApplication(sys.argv)

    # PyQt 윈도우 생성
    window = StatusWindow()
    window.show()

    # 시그널 객체
    sig = StatusSignal()
    sig.status_received.connect(window.update_status_label)

    # TCP 서버 스레드 시작
    server_thread = TCPServerThread(HOST_IP, HOST_PORT, sig)
    server_thread.start()

    # Qt 앱 실행
    exit_code = app.exec_()

    # 종료 시 스레드 정리 시도
    server_thread.stop()
    sys.exit(exit_code)
