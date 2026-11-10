#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
静摩擦/前馈自动测定：读取固件在测试模式输出的 $F 帧和 [FRICTION] 摘要。

固件流程（src/main.cpp，kFrictionTestEnabled=true）：
  1. 关闭航向保持；
  2. 正/反方向各慢速斜坡 3 次；
  3. 用 IMU 角度变化 + 陀螺角速度判断“刚要动”的 omega 指令；
  4. 输出 POS_MEDIAN / NEG_MEDIAN / FF_SYMMETRIC。

用法：
  python3 test/friction_test.py [串口] [--timeout 180]
"""

import glob
import os
import sys
import time
import threading
import termios
import collections


def candidate_ports():
    out = []
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        if "VEX" in p and "if02" in p:
            real = os.path.realpath(p)
            if os.path.exists(real) and real not in out:
                out.append(real)
    for p in ("/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"):
        if os.path.exists(p) and p not in out:
            out.append(p)
    for p in sorted(glob.glob("/dev/ttyACM*")):
        if p.endswith("ACM0"):
            continue
        if p not in out:
            out.append(p)
    return out


def pick_port():
    p = candidate_ports()
    return p[0] if p else "/dev/ttyACM1"


def open_serial(port):
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY)
    speed = getattr(termios, "B115200", 4098)
    _, _, _, _, _, _, cc = termios.tcgetattr(fd)
    cc = list(cc)
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 1
    termios.tcsetattr(
        fd,
        termios.TCSANOW,
        [0, 0, termios.CREAD | termios.CLOCAL | termios.CS8, 0, speed, speed, cc],
    )
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def reader(fd, out, stop):
    buf = bytearray()
    while not stop.is_set():
        try:
            chunk = os.read(fd, 4096)
        except OSError:
            break
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            s = line.decode("ascii", "ignore").strip()
            if s:
                out.append(s)


def main():
    args = sys.argv[1:]
    port = None
    timeout_s = 180.0
    pos = []
    i = 0
    while i < len(args):
        if args[i] == "--timeout":
            i += 1
            timeout_s = float(args[i])
        else:
            pos.append(args[i])
        i += 1
    if pos:
        port = pos[0]
    if port is None:
        port = pick_port()

    print(f"[friction] 串口={port}  timeout={timeout_s:.0f}s")
    fd = open_serial(port)
    lines = collections.deque()
    stop = threading.Event()
    threading.Thread(target=reader, args=(fd, lines, stop), daemon=True).start()

    t0 = time.time()
    last_ramp = 0.0
    breaks = []
    try:
        while time.time() - t0 < timeout_s:
            while lines:
                s = lines.popleft()
                if "[FRICTION]" in s:
                    print(s, flush=True)
                    if "[FRICTION] DONE" in s:
                        return 0
                elif "$F,break" in s or "$F,timeout" in s:
                    print(s, flush=True)
                    breaks.append(s)
                elif "$F,ramp" in s:
                    if time.time() - last_ramp >= 1.0:
                        last_ramp = time.time()
                        print(s, flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        try:
            os.close(fd)
        except OSError:
            pass
    print("[friction] 超时/中断；已收到的 break 行：")
    for s in breaks:
        print(" ", s)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
