#!/usr/bin/env python3
"""显示 $H 航向零点诊断帧；CSV 保留整数单位，不向机器人发送命令。"""
import argparse
import csv
import math
import os
import select
import sys
import time

FIELDS = [
    "ms", "imu_valid", "calibrating", "bias_ready", "reference_ready",
    "relative_valid", "rotation_mrad", "initial_mrad", "relative_mrad",
    "gyro_mrad_s", "still_ms", "latched_ms",
]


def parse(line):
    start = line.find("$H,")
    if start < 0:
        return None
    values = line[start + 3:].strip().split(",")
    if len(values) != len(FIELDS):
        return None
    try:
        return dict(zip(FIELDS, map(int, values)))
    except ValueError:
        return None


def main():
    from distance_monitor import open_serial, pick_port
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", help="默认自动选择 V5 User 口")
    parser.add_argument("--csv", help="保存文件（已有文件不会被覆盖）")
    parser.add_argument("--seconds", type=float, default=0, help="采集秒数；0 表示直到 Ctrl+C")
    args = parser.parse_args()
    fd = None
    output = None
    count = 0
    try:
        fd = open_serial(args.port or pick_port())
        output = open(args.csv, "x", newline="") if args.csv else None
        writer = csv.DictWriter(output, fieldnames=FIELDS) if output else None
        if writer:
            writer.writeheader()
        start = time.monotonic()
        last_print = start - 1
        last_ready = None
        buf = bytearray()
        warned = False
        while args.seconds <= 0 or time.monotonic() - start < args.seconds:
            if not count and not warned and time.monotonic() - start > 5:
                print("未收到 $H：请确认已烧录新固件、程序已启动、User 串口选择正确。", file=sys.stderr)
                warned = True
            if not select.select([fd], [], [], 0.2)[0]:
                continue
            chunk = os.read(fd, 4096)
            if not chunk:
                raise OSError("串口连接已断开")
            buf.extend(chunk)
            while b"\n" in buf:
                line, _, rest = buf.partition(b"\n")
                buf = bytearray(rest)
                row = parse(line.decode("ascii", "ignore"))
                if row is None:
                    continue
                count += 1
                if writer:
                    writer.writerow(row)
                    if count % 10 == 0:
                        output.flush()
                now = time.monotonic()
                if now - last_print >= 1 or row["reference_ready"] != last_ready:
                    deg = 180.0 / math.pi / 1000.0
                    print("t=%.2fs imu=%d cal=%d bias=%d ready=%d valid=%d "
                          "initial=%.2f° relative=%.2f° gyro=%.2f°/s still=%dms lock=%.2fs" % (
                              row["ms"] / 1000.0, row["imu_valid"], row["calibrating"],
                              row["bias_ready"], row["reference_ready"], row["relative_valid"],
                              row["initial_mrad"] * deg, row["relative_mrad"] * deg,
                              row["gyro_mrad_s"] * deg, row["still_ms"], row["latched_ms"] / 1000.0),
                          flush=True)
                    last_print = now
                    last_ready = row["reference_ready"]
        return 0 if count else 1
    except KeyboardInterrupt:
        return 0 if count else 1
    except (OSError, ValueError) as exc:
        print("[heading] %s" % exc, file=sys.stderr)
        return 1
    finally:
        if output:
            output.close()
        if fd is not None:
            os.close(fd)
        print("[heading] 已收到 %d 帧" % count, file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
