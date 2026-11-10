#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V5 LaneKeeper 实时分析 UI：两侧距离 + 墙距/过近 PID 输出。

用途：
  - 连续显示物理左/右原始距离、控制侧滤波距离、误差、P/I/D、PID 目标/实际；
  - 三个可缩放曲线窗口（Tkinter Canvas，不依赖 matplotlib）；
  - 同时写 CSV，便于回放/离线分析。

用法：
  python3 test/lane_plot.py                    # 自动找串口，10s 窗口
  python3 test/lane_plot.py /dev/ttyACM1 10
  python3 test/lane_plot.py /dev/ttyACM1 10 --csv lane.csv
  python3 test/lane_plot.py /dev/ttyACM1 --headless 10 --csv lane.csv

曲线窗口操作与 test/pid_tune.py 一致：
  滚轮缩放、左键拖动平移、单击游标、双击跟随最新。

说明：
  - 固件仍是 20 ms 控制周期；UI 约 30 Hz 刷新，不参与控制；
  - 两侧原始距离来自 $D 的 sideLeftRawMm/sideRightRawMm；
  - 控制左/右是当前运动组选中的左右传感器滤波值（前进=B，横移=A）；
  - “控制目标”是限幅后、速度比例前的横向目标；“实际”是最终施加值；
  - TOO_CLOSE 时墙距 PID 被旁路，P/I/D 为 0，横向输出是过近排斥量。
"""

import collections
import csv
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import distance_monitor as dm
import pid_tune
from pid_tune import BG, TXT, TXT_DIM, GraphWindow


CSV_FIELDS = [
    "ms", "left_raw", "right_raw", "left_filter", "right_filter",
    "left_ctrl", "right_ctrl", "err_mm", "p_mm_s", "i_mm_s", "d_mm_s",
    "pid_target_mm_s", "corr_mm_s", "out_vx_mm_s", "out_vy_mm_s",
    "mode", "group", "too_close_mask", "corridor", "reason",
]


def _num(d, key, default=-1):
    try:
        return int(d.get(key, default))
    except (TypeError, ValueError):
        return default


def sample_from_frame(frame):
    """把一帧 $D 变成 UI/CSV 需要的扁平静态结构。"""
    if not frame or not frame.get("lane"):
        return None
    lane = frame["lane"]
    readings = frame.get("readings") or []

    def raw_side(index, fallback_key):
        v = _num(lane, fallback_key, -1)
        if v >= 0:
            return v
        if index < len(readings):
            d = readings[index].get("distance", -1)
            if 0 < d <= 2000:
                return int(d)
        return -1

    # readings 顺序 = 前、后、左、右；物理左=S1=index2，物理右=S3=index3
    left_raw = raw_side(2, "side_left_raw_mm")
    right_raw = raw_side(3, "side_right_raw_mm")
    return {
        "ms": frame["ms"],
        "left_raw": left_raw,
        "right_raw": right_raw,
        "left_filter": _num(lane, "filter_s1_mm", -1),
        "right_filter": _num(lane, "filter_s3_mm", -1),
        "left_ctrl": _num(lane, "left_mm", -1),
        "right_ctrl": _num(lane, "right_mm", -1),
        "err_mm": _num(lane, "lane_err_mm", 0),
        "p_mm_s": _num(lane, "lane_p_mm_s", 0),
        "i_mm_s": _num(lane, "lane_i_mm_s", 0),
        "d_mm_s": _num(lane, "lane_d_mm_s", 0),
        "pid_target_mm_s": _num(lane, "pid_target_mm_s", 0),
        "corr_mm_s": _num(lane, "lane_corr_mm_s", 0),
        "out_vx_mm_s": _num(lane, "out_vx_mm_s", 0),
        "out_vy_mm_s": _num(lane, "out_vy_mm_s", 0),
        "mode": _num(lane, "lane_mode", -1),
        "group": _num(lane, "lane_group", -1),
        "too_close_mask": _num(lane, "too_close_mask", 0),
        "corridor": 1 if (_num(lane, "lane_flags", 0) & (1 << 9)) else 0,
        "reason": _num(lane, "lane_reason", -1),
    }


def reader_thread(fd, out, stop):
    buf = b""
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
            text = line.decode("ascii", "ignore").strip()
            i = text.find("$D,")
            if i >= 0:
                out.append(text[i:])


def drain(raw, samples, writer):
    while raw:
        frame = dm.parse(raw.popleft())
        s = sample_from_frame(frame)
        if s is None:
            continue
        samples.append(s)
        if writer is not None:
            writer.writerow([s[k] for k in CSV_FIELDS])


def make_csv(path):
    if not path:
        return None, None
    f = open(path, "w", newline="", encoding="utf-8")
    w = csv.writer(f)
    w.writerow(CSV_FIELDS)
    return f, w


def open_serial(port):
    try:
        return dm.open_serial(port)
    except OSError as e:
        print(f"[lane_plot] 无法打开 {port}: {e}", file=sys.stderr)
        return None


def mode_name(mode):
    return dm.LANE_MODE_NAMES.get(mode, str(mode))


def group_name(g):
    return {-1: "—", 0: "A(前后)", 1: "B(左右)"}.get(g, "?")


def headless(port, seconds, csv_path):
    fd = open_serial(port)
    if fd is None:
        return 1
    raw = collections.deque(maxlen=8000)
    stop = threading.Event()
    threading.Thread(target=reader_thread, args=(fd, raw, stop), daemon=True).start()
    samples = collections.deque(maxlen=12000)
    f, w = make_csv(csv_path)
    t0, last = time.time(), 0.0
    try:
        while time.time() - t0 < seconds:
            drain(raw, samples, w)
            if f and time.time() - last >= 1.0:
                f.flush()
            if samples and time.time() - last >= 1.0:
                last = time.time()
                s = samples[-1]
                print(f"t={s['ms']}  Lraw={s['left_raw']:4d} Rraw={s['right_raw']:4d}  "
                      f"Lf={s['left_filter']:4d} Rf={s['right_filter']:4d}  "
                      f"mode={mode_name(s['mode'])} g={s['group']} "
                      f"too=0b{s['too_close_mask'] & 3:02b}  "
                      f"err={s['err_mm']:+4d} P/I/D={s['p_mm_s']:+4d}/"
                      f"{s['i_mm_s']:+4d}/{s['d_mm_s']:+4d}  "
                      f"target={s['pid_target_mm_s']:+4d} corr={s['corr_mm_s']:+4d} mm/s")
            time.sleep(0.02)
    finally:
        stop.set()
        try:
            os.close(fd)
        except OSError:
            pass
        if f:
            f.close()
    print(f"[lane_plot] 采集 {len(samples)} 帧", file=sys.stderr)
    return 0


def run_ui(port, span_s, csv_path):
    import tkinter as tk

    fd = open_serial(port)
    if fd is None:
        return 1
    raw = collections.deque(maxlen=8000)
    stop = threading.Event()
    threading.Thread(target=reader_thread, args=(fd, raw, stop), daemon=True).start()
    samples = collections.deque(maxlen=12000)
    f, w = make_csv(csv_path)

    root = tk.Tk()
    root.title("V5 LaneKeeper 实时分析 — 两侧距离 / PID")
    root.configure(bg=BG)
    root.geometry("980x220+40+40")

    tk.Label(root, text="LaneKeeper 实时分析", fg="#9cf", bg=BG,
             font=("monospace", 14, "bold")).pack(fill="x", padx=8, pady=(6, 2))
    tk.Label(root, text="曲线窗口：滚轮缩放 / 左键拖动 / 单击游标 / 双击跟随最新",
             fg="#89c", bg=BG, font=("monospace", 9)).pack(fill="x", padx=8)

    state_lbl = tk.Label(root, text="等待 $D 数据…", fg="#ffd27f", bg=BG,
                         anchor="w", font=("monospace", 12, "bold"))
    state_lbl.pack(fill="x", padx=8, pady=(6, 0))
    dist_lbl = tk.Label(root, text="", fg=TXT, bg=BG, anchor="w",
                        justify="left", font=("monospace", 11))
    dist_lbl.pack(fill="x", padx=8)
    pid_lbl = tk.Label(root, text="", fg=TXT, bg=BG, anchor="w",
                       justify="left", font=("monospace", 11))
    pid_lbl.pack(fill="x", padx=8)
    warn_lbl = tk.Label(root, text="", fg="#ff8b4d", bg=BG, anchor="w",
                        font=("monospace", 11, "bold"))
    warn_lbl.pack(fill="x", padx=8, pady=(2, 4))

    span_ms = max(1.0, span_s) * 1000.0
    graphs = [
        GraphWindow(root, "两侧原始距离（mm）", "mm", "#4da3ff",
                    lambda s: (s["left_raw"], s["right_raw"]), 50, span_ms,
                    target_label="左", current_label="右"),
        GraphWindow(root, "墙距/过近误差（mm，向右为正）", "mm", "#ffb84d",
                    lambda s: (0, s["err_mm"]), 10, span_ms,
                    target_label="0", current_label="误差"),
        GraphWindow(root, "横向控制输出（PID/过近排斥，mm/s）", "mm/s", "#7fe08a",
                    lambda s: (s["pid_target_mm_s"], s["corr_mm_s"]), 50, span_ms,
                    target_label="目标", current_label="实际"),
    ]
    for i, g in enumerate(graphs):
        g.win.geometry(f"760x330+{40 + i * 30}+{330 + i * 30}")

    def tick():
        drain(raw, samples, w)
        if f:
            f.flush()
        if samples:
            s = samples[-1]
            state_lbl.configure(
                text=(f"{mode_name(s['mode'])}  /  group={group_name(s['group'])}  "
                      f"corridor={s['corridor']}  "
                      f"tooClose=0b{s['too_close_mask'] & 3:02b}  "
                      f"reason={dm.LANE_REASON_NAMES.get(s['reason'], s['reason'])}"))
            dist_lbl.configure(
                text=(f"物理侧原始：左 {s['left_raw']} mm   右 {s['right_raw']} mm"
                      f"     滤波：左 {s['left_filter']}   右 {s['right_filter']} mm\n"
                      f"控制侧：左 {s['left_ctrl']}   右 {s['right_ctrl']} mm"
                      f"     误差 {s['err_mm']:+d} mm"))
            pid_lbl.configure(
                text=(f"P/I/D = {s['p_mm_s']:+d} / {s['i_mm_s']:+d} / "
                      f"{s['d_mm_s']:+d} mm/s\n"
                      f"控制目标 = {s['pid_target_mm_s']:+d} mm/s   "
                      f"实际修正 = {s['corr_mm_s']:+d} mm/s   "
                      f"输出 vx={s['out_vx_mm_s']} vy={s['out_vy_mm_s']} mm/s"))
            if s["too_close_mask"]:
                warn_lbl.configure(text="⚠ 过近：自动居中/远离；不是故障停车")
            elif s["mode"] == 7:
                warn_lbl.configure(text="⚠ SENSOR_FAULT：按配置减速/停车")
            else:
                warn_lbl.configure(text="")
            data = list(samples)
            for g in graphs:
                g.update(s["ms"], [(x["ms"],) + g.pick(x) for x in data])
        root.after(33, tick)

    def on_close():
        stop.set()
        try:
            os.close(fd)
        except OSError:
            pass
        if f:
            f.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    for g in graphs:
        g.win.protocol("WM_DELETE_WINDOW", on_close)
    root.after(33, tick)
    root.mainloop()
    return 0


def main():
    args = sys.argv[1:]
    port, span_s, csv_path, headless_s = None, 10.0, None, None
    pos, i = [], 0
    while i < len(args):
        if args[i] == "--csv":
            i += 1
            if i >= len(args):
                print("--csv 需要文件名", file=sys.stderr)
                return 2
            csv_path = args[i]
        elif args[i] == "--headless":
            i += 1
            if i >= len(args):
                print("--headless 需要秒数", file=sys.stderr)
                return 2
            headless_s = float(args[i])
        else:
            pos.append(args[i])
        i += 1
    if pos:
        port = pos[0]
    if len(pos) > 1:
        span_s = float(pos[1])
    if port is None:
        port = dm.pick_port()

    print(f"[lane_plot] 串口={port} 窗口={span_s}s csv={csv_path}")
    if headless_s is not None:
        return headless(port, headless_s, csv_path)
    try:
        return run_ui(port, span_s, csv_path)
    except Exception as e:
        print(f"[lane_plot] UI 启动失败：{e}", file=sys.stderr)
        print("无图形环境时可用：python3 test/lane_plot.py <port> --headless 10 --csv lane.csv",
              file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
