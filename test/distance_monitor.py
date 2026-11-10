#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
V5 四路距离传感器实时监视器。

固件通过 User 口以 20 Hz 输出：
  $D,ms,instMask,validMask,
     dF,cF,sF,vF,tF,  dR,cR,sR,vR,tR,  dL,cL,sL,vL,tL,  dRt,cRt,sRt,vRt,tRt,
     laneMode,laneGroup,wallMask,usableMask,
     leftMm,rightMm,errMm,pMms,iMms,dMms,corrMms,outVxMms,outVyMms,outOmegaMradS,
     filterS0..S3,filterValidMask,targetWallMm,corridorWidthMm,laneReason,
     speedScaleMilli,forwardScaleMilli,motionObstacleMm,laneFlags,
     sideLeftRawMm,sideRightRawMm,pidTargetMmS,tooCloseMask

方向顺序：前(7)、后(8)、左(9)、右(10)。
  d = 距离 [mm]（-1=读取失败，9999=无目标）
  c = 置信度 0..63（-1=读取失败）
  s = 物体大小 0..400（-1=无效）
  v = 物体速度 [mm/s]
  t = VEXos 识别的设备类型（7=Distance，0=无设备，2=Motor，6=IMU…）
  instMask/validMask：bit0=前、bit1=后、bit2=左、bit3=右

车道字段（旧固件可能没有，解析器会自动兼容）：
  laneMode：0=关 1=识别 2=双墙 3=左墙 4=右墙 5=无墙 6=转弯 7=传感器故障 8=过近
  laneGroup：-1=未选 0=Group A(前后) 1=Group B(左右)
  wallMask/usableMask/filtValidMask：逻辑 S0..S3，bit0=S0
  leftMm/rightMm：运动左/右传感器的滤波距离 [mm]，-1=不可用
  errMm：墙距 PID 误差，向右为正
  pMms/iMms/dMms/corrMms：PID 各项与最终纠偏 [mm/s]
  filterS0..S3：四路滤波距离，逻辑 S0..S3 = 前/左/后/右；targetWallMm：单墙目标
  corridorWidthMm：双墙参考垂距之和；laneReason：状态机原因码
  speedScaleMilli/forwardScaleMilli：状态限速/前向安全比例 ×1000
  motionObstacleMm：运动方向最近障碍；laneFlags：active/turning/degraded/fault 等
  sideLeftRawMm/sideRightRawMm：物理左/右原始距离；pidTargetMmS：控制目标横向速度
  tooCloseMask：bit0=左过近，bit1=右过近
  outVxMms/outVyMms/outOmegaMradS：LaneKeeper 输出命令（omega 为透传值；
  最终航向修正后的 omega 看 $P 的 tw）

用法：
  python3 test/distance_monitor.py                 # 自动找 /dev/ttyACM*
  python3 test/distance_monitor.py /dev/ttyACM1
  python3 test/distance_monitor.py /dev/ttyACM1 --csv distance.csv
  python3 test/distance_monitor.py /dev/ttyACM1 --headless 10
"""

import collections
import glob
import os
import sys
import time
import threading
import termios
import unicodedata

# (显示名, 端口, CSV 名)
SIDES = [
    ("前", 7, "front"),
    ("后", 8, "rear"),
    ("左", 9, "left"),
    ("右", 10, "right"),
]
RANGE_MIN_MM = 20
RANGE_MAX_MM = 2000
NO_TARGET_MM = 9999

# $D 末尾追加的车道保持字段（顺序与 src/diag/telemetry.cpp 一致）
LANE_FIELDS = [
    "lane_mode", "lane_group", "wall_mask", "usable_mask",
    "left_mm", "right_mm", "lane_err_mm",
    "lane_p_mm_s", "lane_i_mm_s", "lane_d_mm_s", "lane_corr_mm_s",
    "out_vx_mm_s", "out_vy_mm_s", "out_omega_mrad_s",
    "filter_s0_mm", "filter_s1_mm", "filter_s2_mm", "filter_s3_mm",
    "filter_valid_mask", "target_wall_mm", "corridor_width_mm",
    "lane_reason", "speed_scale_milli", "forward_scale_milli",
    "motion_obstacle_mm", "lane_flags",
    "side_left_raw_mm", "side_right_raw_mm", "pid_target_mm_s",
    "too_close_mask",
]
LANE_MODE_NAMES = {
    0: "关",
    1: "识别",
    2: "双墙",
    3: "左墙",
    4: "右墙",
    5: "无墙",
    6: "转弯",
    7: "传感器故障",
    8: "过近",
}
LANE_REASON_NAMES = {
    0: "none",
    1: "disabled",
    2: "config",
    3: "input",
    4: "startup",
    5: "stopped",
    6: "no-group",
    7: "group-changed",
    8: "after-turn",
    9: "wall-acquiring",
    10: "corridor-inconsistent",
    11: "sensor-timeout",
    12: "dual",
    13: "left-only",
    14: "right-only",
    15: "no-walls",
    16: "turning",
    17: "too-close",
    18: "corridor-inactive",
}

# ANSI
CLR = "\x1b[2J\x1b[H"
HIDE = "\x1b[?25l"
SHOW = "\x1b[?25h"
RESET = "\x1b[0m"
BOLD = "\x1b[1m"
DIM = "\x1b[2m"
GREEN = "\x1b[32m"
YELLOW = "\x1b[33m"
RED = "\x1b[31m"

COLS = [
    ("方向", 4, "left"),
    ("端口", 4, "center"),
    ("距离", 10, "left"),
    ("距离条", 12, "left"),
    ("置信", 4, "center"),
    ("大小", 4, "center"),
    ("速度", 6, "right"),
    ("状态", 10, "left"),
]


# ---------------------------------------------------------------- 串口
def candidate_ports():
    out = []

    # V5 有两个 CDC 接口：if00=system，if02=user（printf 数据）。
    # 优先用 by-id 精确选 user 口，避免设备重新枚举后 ACM 编号变化。
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        if "VEX" in p and "if02" in p:
            real = os.path.realpath(p)
            if os.path.exists(real) and real not in out:
                out.append(real)

    # 没有 by-id 时按常见编号兜底；不要优先选 ACM0（通常是 system 口）。
    for p in ("/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"):
        if os.path.exists(p) and p not in out:
            out.append(p)

    for p in sorted(glob.glob("/dev/ttyACM*")):
        if p.endswith("ACM0"):
            continue
        if p not in out:
            out.append(p)

    for p in sorted(glob.glob("/dev/ttyUSB*")):
        if p not in out:
            out.append(p)
    return out


def pick_port():
    ports = candidate_ports()
    return ports[0] if ports else "/dev/ttyACM1"


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


def reader_thread(fd, out, stop, stats):
    buf = bytearray()
    while not stop.is_set():
        try:
            chunk = os.read(fd, 4096)
        except OSError:
            break
        if not chunk:
            continue
        buf += chunk
        while True:
            i = buf.find(b"\n")
            if i < 0:
                break
            line = bytes(buf[:i]).decode("ascii", "ignore").strip()
            del buf[: i + 1]
            if not line:
                continue
            if "$D," in line:
                out.append(line)
                stats["d"] = stats.get("d", 0) + 1
            elif "$S," in line:
                out.append(line)
                stats["s"] = stats.get("s", 0) + 1
            elif "$P," in line:
                stats["p"] = stats.get("p", 0) + 1
            else:
                stats["other"] = stats.get("other", 0) + 1


# ---------------------------------------------------------------- 解析
def parse(line):
    i = line.find("$D,")
    if i < 0:
        return None
    parts = line[i:].split(",")[1:]
    base_fields = 3 + 4 * 5
    if len(parts) < base_fields:
        return None
    try:
        nums = [int(x) for x in parts]
    except ValueError:
        return None

    frame = {
        "ms": nums[0],
        "inst_mask": nums[1],
        "valid_mask": nums[2],
        "readings": [],
        "lane": None,
    }
    k = 3
    for side, port, csv_name in SIDES:
        frame["readings"].append(
            {
                "side": side,
                "port": port,
                "csv": csv_name,
                "distance": nums[k],
                "confidence": nums[k + 1],
                "size": nums[k + 2],
                "velocity": nums[k + 3],
                "plugged_type": nums[k + 4],
            }
        )
        k += 5

    # 车道字段是后加的可选扩展：旧固件只有前 23 个字段。
    if len(nums) >= base_fields + len(LANE_FIELDS):
        frame["lane"] = dict(
            zip(LANE_FIELDS, nums[base_fields: base_fields + len(LANE_FIELDS)])
        )
    return frame


def parse_scan(line):
    """解析端口扫描帧：$S,ms,t1,t2,...,t21（tN 为 N 号端口类型）。"""
    i = line.find("$S,")
    if i < 0:
        return None
    parts = line[i:].split(",")[1:]
    if len(parts) < 22:
        return None
    try:
        nums = [int(x) for x in parts[:22]]
    except ValueError:
        return None
    return {
        "ms": nums[0],
        "types": {port: nums[port] for port in range(1, 22)},
    }


def reading_status(r, installed, valid):
    d = r["distance"]
    t = r["plugged_type"]
    if not installed:
        if t == 0:
            return "未连接", RED
        if t == 7:
            return "识别异常", RED
        return f"类型{t}", RED
    if valid:
        return "有效", GREEN
    if d == -1:
        return "读取失败", RED
    if d == NO_TARGET_MM:
        return "无目标", YELLOW
    if d < RANGE_MIN_MM:
        return "过近", YELLOW
    if d > RANGE_MAX_MM:
        return "超量程", YELLOW
    return "无效", YELLOW


def distance_bar(d, valid, width=12):
    if not valid:
        return "—" * width
    d = max(0, min(d, RANGE_MAX_MM))
    n = int(round(width * d / float(RANGE_MAX_MM)))
    return "█" * n + "░" * (width - n)


# ---------------------------------------------------------------- 对齐（中文按 2 列宽）
def disp_width(s):
    return sum(2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1 for ch in s)


def pad(s, width, align="left"):
    s = str(s)
    space = max(0, width - disp_width(s))
    if align == "right":
        return " " * space + s
    if align == "center":
        left = space // 2
        return " " * left + s + " " * (space - left)
    return s + " " * space


def border(left, mid, right):
    return left + mid.join("─" * (w + 2) for _, w, _ in COLS) + right


def table_row(cells, ansi=True):
    padded = []
    for (title, width, align), cell in zip(COLS, cells):
        # 先按显示宽度补齐，再上色；ANSI 不参与宽度计算
        if isinstance(cell, tuple):
            text, color = cell
            text = pad(text, width, align)
            padded.append(f"{color}{text}{RESET}" if ansi else text)
        else:
            padded.append(pad(cell, width, align))
    return "│ " + " │ ".join(padded) + " │"


# ---------------------------------------------------------------- 显示
def render(frame, port, frames, age_ms, csv_path, ansi=True, diag=None, scan=None):
    def c(code, text):
        return f"{code}{text}{RESET}" if ansi else text

    diag = diag or {}
    lines = []
    lines.append(c(BOLD, "V5 四路距离传感器") + f"   串口 {port}")
    if frame is None:
        if diag.get("p", 0) > 0:
            lines.append(c(YELLOW, "收到 $P 帧，但没有 $D：固件可能不是最新版（未包含 DistanceArray/$D）。"))
        elif diag.get("s", 0) > 0:
            lines.append(c(YELLOW, "收到 $S 端口扫描，但没有 $D：检查 kDistanceTelemetryEnabled 或测距读取。"))
        elif diag.get("other", 0) > 0:
            lines.append(c(YELLOW, "收到串口数据，但没有识别到 $D/$P：检查固件与波特率。"))
        else:
            lines.append(c(YELLOW, "没有收到任何串口帧：检查程序是否在 opcontrol 运行、User 口和数据线。"))
    else:
        types = " ".join(
            f"{r['side']}:{r['plugged_type']}" for r in frame["readings"]
        )
        lines.append(
            f"帧 {frames}   age {age_ms:.0f} ms   "
            f"inst=0x{frame['inst_mask']:X} valid=0x{frame['valid_mask']:X}   "
            f"类型 {types}"
        )

    if scan is None:
        lines.append(c(DIM, "端口扫描：等待 $S 帧…"))
    else:
        dist_ports = sorted(p for p, t in scan["types"].items() if t == 7)
        if dist_ports == [7, 8, 9, 10]:
            lines.append(c(GREEN, "端口扫描：Distance 在 7/8/9/10，接线正确"))
        elif dist_ports:
            lines.append(c(YELLOW, "端口扫描：Distance 在 " +
                           "/".join(str(p) for p in dist_ports) +
                           "（期望 7/8/9/10，请检查接线）"))
        else:
            lines.append(c(RED, "端口扫描：VEXos 未发现任何 Distance 设备（检查供电/智能线/端口）"))
    lines.append("")
    lines.append(border("┌", "┬", "┐"))
    lines.append(table_row([(title, BOLD) for title, _, _ in COLS], ansi))
    lines.append(border("├", "┼", "┤"))

    if frame is None:
        for side, port_no, _ in SIDES:
            lines.append(table_row([side, port_no, "—", "—", "—", "—", "—", "—"], ansi))
    else:
        for i, r in enumerate(frame["readings"]):
            # 主机端按 VEXos 类型 + 量程重新判定，避免固件 mask 异常时误显示
            installed = (r["plugged_type"] == 7)
            valid = installed and RANGE_MIN_MM <= r["distance"] <= RANGE_MAX_MM
            status, color = reading_status(r, installed, valid)
            d = r["distance"]
            d_txt = "—" if d == -1 else f"{d} mm"
            conf = "—" if r["confidence"] == -1 else str(r["confidence"])
            size = "—" if r["size"] == -1 else str(r["size"])
            vel = "—" if not installed else f"{r['velocity']:+d}"
            bar = distance_bar(d, valid)
            bar_cell = (bar, GREEN) if valid else (bar, DIM)
            lines.append(
                table_row([r["side"], r["port"], d_txt, bar_cell, conf, size, vel,
                           (status, color)], ansi)
            )

    lines.append(border("└", "┴", "┘"))
    lines.append("")

    if frame is not None:
        lane = frame.get("lane")
        if not lane:
            lines.append(c(DIM, "车道字段：无（旧固件或未绑定 LaneKeeper）"))
        else:
            mode = lane["lane_mode"]
            reason = lane.get("lane_reason", -1)
            mode_name = LANE_MODE_NAMES.get(mode, str(mode))
            reason_name = LANE_REASON_NAMES.get(reason, str(reason))
            group = { -1: "—", 0: "A", 1: "B" }.get(lane["lane_group"], "?")
            wall = lane["wall_mask"] & 0xF
            usable = lane["usable_mask"] & 0xF
            filt = lane.get("filter_valid_mask", -1)
            left = "—" if lane["left_mm"] < 0 else f"{lane['left_mm']} mm"
            right = "—" if lane["right_mm"] < 0 else f"{lane['right_mm']} mm"
            width = lane.get("corridor_width_mm", -1)
            width_txt = "—" if width < 0 else f"{width} mm"
            if mode in (2, 3, 4):
                color = GREEN
            elif mode in (1, 6, 8):
                color = YELLOW
            elif mode == 7:
                color = RED
            else:
                color = DIM
            lines.append(c(
                color,
                f"车道：{mode_name}({reason_name})  组={group}  "
                f"墙=0b{wall:04b} 可用=0b{usable:04b} "
                f"滤波=0b{filt & 0xF:04b}"
            ))
            side_l = lane.get("side_left_raw_mm", -1)
            side_r = lane.get("side_right_raw_mm", -1)
            too_close = lane.get("too_close_mask", 0)
            pid_target = lane.get("pid_target_mm_s", 0)
            lines.append(c(
                DIM,
                f"      控制左={left} 右={right} 宽={width_txt} "
                f"目标={lane.get('target_wall_mm', -1)} mm  "
                f"误差={lane['lane_err_mm']} mm"
            ))
            lines.append(c(
                DIM,
                f"      物理侧原始 左={side_l} mm 右={side_r} mm  "
                f"过近=0b{too_close & 0x3:02b}"
            ))
            lines.append(c(
                DIM,
                f"      P/I/D={lane['lane_p_mm_s']}/{lane['lane_i_mm_s']}/"
                f"{lane['lane_d_mm_s']} mm/s  "
                f"PID目标={pid_target} mm/s 修正={lane['lane_corr_mm_s']} mm/s"
            ))
            lines.append(c(
                DIM,
                f"      速度比例={lane.get('speed_scale_milli', -1) / 1000.0:.2f} "
                f"前向安全={lane.get('forward_scale_milli', -1) / 1000.0:.2f} "
                f"障碍={lane.get('motion_obstacle_mm', -1)} mm "
                f"flags=0x{lane.get('lane_flags', 0):X}"
            ))
            lines.append(c(
                DIM,
                f"      输出 vx={lane['out_vx_mm_s']} vy={lane['out_vy_mm_s']} "
                f"ω={lane['out_omega_mrad_s'] / 1000.0:.2f} rad/s"
            ))

    lines.append(c(DIM, "有效范围 20–2000 mm；9999=无目标，<20=过近。"))
    lines.append(c(DIM, "类型：7=Distance，0=无设备；其他值=端口插错设备。"))
    if csv_path:
        lines.append(c(DIM, f"CSV 记录中：{csv_path}"))
    lines.append(c(DIM, "Ctrl-C 退出"))
    print("\n".join(lines), flush=True)


def headless_line(frame, age_ms):
    if frame is None:
        return "等待 $D 帧…"
    vals = []
    for i, r in enumerate(frame["readings"]):
        installed = (r["plugged_type"] == 7)
        valid = installed and RANGE_MIN_MM <= r["distance"] <= RANGE_MAX_MM
        status, _ = reading_status(r, installed, valid)
        vals.append(f"{r['side']}={r['distance']}mm/{status}/t{r['plugged_type']}")
    text = f"age={age_ms:4.0f}ms  " + "  ".join(vals)
    lane = frame.get("lane")
    if lane:
        mode_name = LANE_MODE_NAMES.get(lane["lane_mode"], lane["lane_mode"])
        text += (f"  lane={mode_name}/g{lane['lane_group']}"
                 f"/L{lane.get('side_left_raw_mm', -1)}"
                 f"/R{lane.get('side_right_raw_mm', -1)}"
                 f"/err{lane['lane_err_mm']}"
                 f"/out{lane.get('pid_target_mm_s', 0)}"
                 f"/corr{lane['lane_corr_mm_s']}")
    return text


# ---------------------------------------------------------------- 主程序
def main():
    args = sys.argv[1:]
    port = None
    csv_path = None
    headless_s = None
    no_color = False
    pos = []
    i = 0
    while i < len(args):
        a = args[i]
        if a in ("-h", "--help"):
            print(__doc__)
            return 0
        if a == "--list":
            cands = candidate_ports()
            if cands:
                print("可用串口：")
                for p in cands:
                    print(f"  {p}")
            else:
                print("未发现 /dev/ttyACM*、/dev/ttyUSB* 或 /dev/serial/by-id/*")
            return 0
        if a == "--csv":
            i += 1
            if i >= len(args):
                print("--csv 需要一个文件名", file=sys.stderr)
                return 2
            csv_path = args[i]
        elif a == "--headless":
            i += 1
            if i >= len(args):
                print("--headless 需要秒数", file=sys.stderr)
                return 2
            try:
                headless_s = float(args[i])
            except ValueError:
                print("--headless 的秒数不是数字", file=sys.stderr)
                return 2
        elif a == "--no-color":
            no_color = True
        else:
            pos.append(a)
        i += 1

    if pos:
        port = pos[0]
    if port is None:
        port = pick_port()

    ansi = sys.stdout.isatty() and not no_color

    print(f"[distance] 串口={port}  (Ctrl-C 退出)", file=sys.stderr)
    try:
        fd = open_serial(port)
    except OSError as e:
        print(f"[distance] 无法打开 {port}: {e}", file=sys.stderr)
        cands = candidate_ports()
        if cands:
            print("[distance] 可用串口：" + " ".join(cands), file=sys.stderr)
        else:
            print("[distance] 未发现 /dev/ttyACM*、/dev/ttyUSB* 或 /dev/serial/by-id/*", file=sys.stderr)
        print("[distance] 连接排查：", file=sys.stderr)
        print("  1) 用 USB 数据线连接 V5 Brain 的 User 口（不是充电线）", file=sys.stderr)
        print("  2) V5 已开机，程序运行在 opcontrol/driver 模式", file=sys.stderr)
        print("  3) Linux 权限：sudo usermod -aG dialout $USER，然后重新登录", file=sys.stderr)
        print("  4) lsusb 应能看到 VEX Robotics 设备；也可运行 `pros v5 ls` 检查", file=sys.stderr)
        return 1

    raw = collections.deque(maxlen=4000)
    stop = threading.Event()
    stats = {"d": 0, "p": 0, "other": 0}
    threading.Thread(target=reader_thread, args=(fd, raw, stop, stats), daemon=True).start()

    csv_f = open(csv_path, "w", newline="") if csv_path else None
    if csv_f:
        header = ["ms", "inst_mask", "valid_mask"]
        for _, _, csv_name in SIDES:
            header += [f"{csv_name}_mm", f"{csv_name}_conf",
                       f"{csv_name}_size", f"{csv_name}_vel_mm_s",
                       f"{csv_name}_type"]
        header += LANE_FIELDS
        csv_f.write(",".join(header) + "\n")

    frames = 0
    last_frame = None
    last_scan = None
    last_frame_wall = None
    last_render = 0.0
    last_headless = 0.0
    start = time.time()

    if ansi:
        sys.stdout.write(HIDE)

    try:
        while True:
            got = False
            while raw:
                line = raw.popleft()
                if "$S," in line:
                    sc = parse_scan(line)
                    if sc is not None:
                        last_scan = sc
                    continue
                f = parse(line)
                if f is None:
                    continue
                frames += 1
                last_frame = f
                last_frame_wall = time.time()
                got = True
                if csv_f:
                    row = [f["ms"], f["inst_mask"], f["valid_mask"]]
                    for r in f["readings"]:
                        row += [r["distance"], r["confidence"], r["size"],
                                r["velocity"], r["plugged_type"]]
                    lane = f.get("lane")
                    row += [lane[name] if lane else -1 for name in LANE_FIELDS]
                    csv_f.write(",".join(str(x) for x in row) + "\n")
                    if frames % 20 == 0:  # 约 1 s 刷一次盘
                        csv_f.flush()

            now = time.time()
            age_ms = (now - last_frame_wall) * 1000.0 if last_frame_wall else 0.0

            if headless_s is not None:
                if now - last_headless >= 1.0:
                    last_headless = now
                    print(headless_line(last_frame if age_ms < 1000 else None, age_ms))
                if now - start >= headless_s:
                    break
            else:
                # 有新帧就刷新；没有新帧时每秒刷新一次，显示等待/超时状态
                if got or now - last_render >= 1.0:
                    last_render = now
                    if ansi:
                        sys.stdout.write(CLR)
                    render(
                        last_frame if age_ms < 1000 else None,
                        port,
                        frames,
                        age_ms,
                        csv_path,
                        ansi=ansi,
                        diag=stats,
                        scan=last_scan,
                    )
                    if not ansi:
                        print("-" * 80)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        try:
            os.close(fd)
        except OSError:
            pass
        if csv_f:
            csv_f.close()
        if ansi:
            sys.stdout.write(SHOW)
            sys.stdout.flush()

    print(f"[distance] 共收到 {frames} 帧" + (f" -> {csv_path}" if csv_path else ""),
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
