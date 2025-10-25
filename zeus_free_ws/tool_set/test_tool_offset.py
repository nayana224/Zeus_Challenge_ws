#!/usr/bin/python
# -*- coding: utf-8 -*-
# ZEUS Robot - TCP 오프셋(Z축만) 적용 예제
# Python2 / i611 SDK 호환

from i611shm import *
import math

def read_flange_position():
    """현재 플랜지 좌표 (베이스 기준) 읽기"""
    pos = shm_read(0x3000, 6).split(',')
    x = float(pos[0]) * 1000
    y = float(pos[1]) * 1000
    z = float(pos[2]) * 1000
    rz = math.degrees(float(pos[3]))
    ry = math.degrees(float(pos[4]))
    rx = math.degrees(float(pos[5]))
    return x, y, z, rz, ry, rx

def apply_z_offset(z, dz):
    """Z축 방향으로 오프셋 적용"""
    return z + dz

def main():
    print("[INFO] 현재 플랜지 좌표 읽는 중...")
    x, y, z, rz, ry, rx = read_flange_position()
    print("Flange Pos [mm, deg]:")
    print("  X={:.3f}, Y={:.3f}, Z={:.3f}, Rz={:.3f}, Ry={:.3f}, Rx={:.3f}".format(
        x, y, z, rz, ry, rx))

    # 🔧 내가 적용하고 싶은 Tool Offset (Z 방향만)
    dz = 100  # +100mm 오프셋

    z_tcp = apply_z_offset(z, dz)
    print("\n[UPDATED TCP with Z Offset]")
    print("  X={:.3f}, Y={:.3f}, Z={:.3f}".format(x, y, z_tcp))
    print("  (Z offset applied: +{} mm)".format(dz))

if __name__ == "__main__":
    main()
