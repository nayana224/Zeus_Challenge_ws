# -*- coding: utf-8 -*-
"""
tool_set.py  (for Python 2)
---------------------------------
ZEUS ZERO 매뉴얼 기반 Tool 활성화 코드
---------------------------------
"""

import sys
import time

try:
    import robot  # 제우스 컨트롤러 기본 로봇 라이브러리
except ImportError:
    print("[ERROR] 'robot' 모듈을 찾을 수 없습니다. 컨트롤러 내부(i611usr)에서 실행하세요.")
    sys.exit(1)

def main():
    print("=== ZEUS ROBOT TOOL SETUP (MENU-BASED) ===")

    # 1️⃣ 로봇 연결
    rb = robot.Robot()
    rb.open()
    time.sleep(0.3)
    print("[INFO] Connected to robot.")

    # 2️⃣ 활성화할 Tool 번호 선택
    TOOL_ID = 1  # Web Teaching에서 설정한 Tool 번호
    rb.select_tool(TOOL_ID)
    print("[INFO] Tool {} selected.".format(TOOL_ID))

    # 3️⃣ 현재 Tool 정보 확인
    try:
        tool_data = rb.get_tooldata(TOOL_ID)
        print("[INFO] Tool {} offset data: {}".format(TOOL_ID, tool_data))
    except:
        print("[WARN] get_tooldata() 함수가 없습니다. SDK 버전에 따라 다를 수 있습니다.")

    # 4️⃣ 확인 안내
    print("\n[GUIDE] 다음을 실행하여 TCP 보정 확인:")
    print("  $ python eval_position.py")
    print("현재 활성화된 Tool의 Offset이 TCP에 반영됩니다.")

    rb.close()
    print("[INFO] Robot connection closed.")

if __name__ == "__main__":
    main()
