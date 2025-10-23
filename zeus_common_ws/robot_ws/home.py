#!/usr/bin/python
# -*- coding: utf-8 -*-

from i611_MCS import *
from i611_common import *
import time

def main():
    rb = i611Robot()
    rb.open()
    try:
        target_joint = Joint(75.57, 6.02, 69.57, 0.00, 104.41, 75.57)
        rb.move(target_joint)

        tgt_rz, tgt_ry, tgt_rx = 90.0, 0.0, 180.0
        cur = rb.getpos().pos2list()
        x, y, z = cur[0], cur[1], cur[2]

        tgt = Position(x, y, z, tgt_rz, tgt_ry, tgt_rx).offset(0, 0, 0, 0, 0, 0, 1)
        rb.line(tgt)

        print("홈 위치 → 자세 세팅 → 툴 기준 이동 완료")

    except Robot_emo:
        print("EMO(비상정지) 입력")
    except Exception as e:
        print("Error:", e)
    finally:
        rb.close()

if __name__ == "__main__":
    main()
