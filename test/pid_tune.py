#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PID 调参：固定中文诊断表 + 每条曲线一个独立窗口（类 Ozone 变量图像）。

曲线窗口操作：
  - 滚轮：以鼠标位置为中心缩放时间轴（放大看细节）
  - 左键单击：放置竖线游标，显示该时刻的“目标/读取”值
  - 左键拖动：平移（查看历史）
  - 双击：回到“跟随最新”并清除游标
  - Y 轴自动按当前可见区间缩放

单位：
  - 平移 vx/vy：mm/s
  - 航向角：度（°）
  - 角速度/PID 修正：度/秒（°/s）
  - 固件原始 $P 帧仍是 mrad / mrad/s，本脚本在解析后统一换算。

字段顺序（与 src/diag/telemetry.cpp 的 DIAG_TUNE_FIELDS 一致）：
  ms, tvx,tvy, tw,wraw, rot,gz, ht,herr, hP,hI,hD,hcorr, hext,hrot,
  aFL,aFR,aRL,aRR, cFL,cFR,cRL,cRR, bias,drift, st,bm,imu,hdg, rc,
  kp,ki,kd, intLim, pLim,iLim,dLim,outLim,ff

用法：
  python3 test/pid_tune.py [串口] [初始窗口秒数] [--csv 文件] [--headless 秒数]
"""

import math
import os
import sys
import time
import threading
import collections
import glob
import termios

FIELDS = [
    "ms",
    "tvx", "tvy", "tw", "wraw", "rot", "gz",
    "ht", "herr",
    "hP", "hI", "hD", "hcorr", "hext", "hrot",
    "aFL", "aFR", "aRL", "aRR",
    "cFL", "cFR", "cRL", "cRR",
    "bias", "drift",
    "st", "bm", "imu", "hdg", "rc",
    "kp", "ki", "kd", "intLim", "pLim", "iLim", "dLim", "outLim", "ff",
]

BG = "#1e1e1e"
PANEL = "#252526"
GRID = "#333333"
TXT = "#d4d4d4"
TXT_DIM = "#8a8a8a"
TARGET_COL = "#9a9a9a"

HISTORY_MS = 120000.0   # 保留 120s 历史供放大/回看

# 固件 $P 的旋转量是 mrad / mrad/s；显示/CSV 转成度、度/秒。
MIL_TO_DEG = 180.0 / (math.pi * 1000.0)
# 陀螺零偏/漂移是 mdps；显示/CSV 转成 deg/s。
MDPS_TO_DPS = 0.001

RPM_TO_MPS = (2.0 * math.pi / 60.0) * 0.05
INV_2SQRT2 = 1.0 / (2.0 * math.sqrt(2.0))


def wrap_deg(x):
    """把角度折叠到 (-180, 180]，用于显示，不改变控制用的最短角差。"""
    return (x + 180.0) % 360.0 - 180.0


def forward_kinematics(a_fl, a_fr, a_rl, a_rr):
    # 输入为 milli-rpm（固件 ×1000 回传）
    s_fl, s_fr, s_rl, s_rr = (a / 1000.0 * RPM_TO_MPS for a in (a_fl, a_fr, a_rl, a_rr))
    return ((s_fr - s_fl + s_rr - s_rl) * INV_2SQRT2 * 1000.0,
            (s_fl + s_fr - s_rl - s_rr) * INV_2SQRT2 * 1000.0)


# ---------------------------------------------------------------- 串口
def candidate_ports():
    """V5 有两个 CDC 接口：if00=system，if02=user（printf 数据）。"""
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
    termios.tcsetattr(fd, termios.TCSANOW,
                      [0, 0, termios.CREAD | termios.CLOCAL | termios.CS8,
                       0, speed, speed, cc])
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def reader_thread(fd, out, stop):
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
            if "$P," in line:
                out.append(line)


def parse(line):
    i = line.find("$P,")
    if i < 0:
        return None
    parts = line[i:].split(",")[1:]
    if len(parts) < len(FIELDS):
        return None
    try:
        d = dict(zip(FIELDS, (int(x) for x in parts[: len(FIELDS)])))
    except ValueError:
        return None
    # mrad / mrad/s / mrad·s -> deg / deg/s / deg·s
    for k in ("rot", "ht", "herr", "tw", "wraw", "gz",
              "hP", "hI", "hD", "hcorr", "hext",
              "pLim", "iLim", "dLim", "outLim", "intLim", "ff"):
        d[k] *= MIL_TO_DEG
    # mdps -> deg/s
    for k in ("bias", "drift"):
        d[k] *= MDPS_TO_DPS
    return d


# ---------------------------------------------------------------- 曲线窗口
class GraphWindow:
    """一条曲线一个独立窗口；支持滚轮缩放、拖动平移、双击跟随。"""

    def __init__(self, root, title, unit, color, pick, yfloor, span_ms,
                 target_label="目标", current_label="当前"):
        import tkinter as tk
        self.tk = tk
        self.unit = unit
        self.target_label = target_label
        self.current_label = current_label
        self.color = color
        self.pick = pick
        self.yfloor = yfloor
        self.follow = True
        self.span = span_ms
        self.left = None           # 暂停时的视图左边界 [ms]
        self.t_now = 0.0
        self._drag_x = None
        self._drag_left = 0.0
        self._press_x = 0
        self._dragged = False
        self.marker_t = None       # 单击竖线游标的时间 [ms]

        self.win = tk.Toplevel(root)
        self.win.title(title)
        self.win.geometry("760x360")
        self.win.configure(bg=BG)

        head = tk.Frame(self.win, bg=BG)
        head.pack(fill="x", padx=8, pady=(6, 0))
        tk.Label(head, text=title, fg=TXT, bg=BG,
                 font=("monospace", 13, "bold")).pack(side="left")
        lg = tk.Frame(head, bg=BG)
        lg.pack(side="right")
        c1 = tk.Canvas(lg, width=28, height=12, bg=BG, highlightthickness=0)
        c1.pack(side="left")
        c1.create_line(2, 6, 26, 6, fill=TARGET_COL, dash=(4, 3), width=2)
        tk.Label(lg, text=self.target_label, fg=TXT_DIM, bg=BG,
                 font=("monospace", 10)).pack(side="left", padx=(0, 8))
        c2 = tk.Canvas(lg, width=28, height=12, bg=BG, highlightthickness=0)
        c2.pack(side="left")
        c2.create_line(2, 6, 26, 6, fill=color, width=2)
        tk.Label(lg, text=self.current_label, fg=color, bg=BG,
                 font=("monospace", 10)).pack(side="left")

        self.readout = tk.Label(self.win, text="等待数据…", fg="#7fd4ff", bg=BG,
                                anchor="w", font=("monospace", 12, "bold"))
        self.readout.pack(fill="x", padx=8, pady=(2, 2))

        self.marker_readout = tk.Label(
            self.win, text="游标：单击曲线放置", fg="#ffd27f", bg=BG,
            anchor="w", font=("monospace", 10))
        self.marker_readout.pack(fill="x", padx=8, pady=(0, 4))

        self.canvas = tk.Canvas(self.win, bg=PANEL, highlightthickness=1,
                                highlightbackground="#3c3c3c", cursor="crosshair")
        self.canvas.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self._last = None
        self.canvas.bind("<Configure>", lambda e: self._redraw())
        self.canvas.bind("<MouseWheel>", lambda e: self._zoom(e, e.delta))
        self.canvas.bind("<Button-4>", lambda e: self._zoom(e, 1))    # X11 滚轮上=放大
        self.canvas.bind("<Button-5>", lambda e: self._zoom(e, -1))   # X11 滚轮下=缩小
        self.canvas.bind("<ButtonPress-1>", self._press)
        self.canvas.bind("<B1-Motion>", self._motion)
        self.canvas.bind("<ButtonRelease-1>", self._release)
        self.canvas.bind("<Double-Button-1>", lambda e: self._follow())

    # ---- 视图 ----
    def _view(self):
        if self.follow or self.left is None:
            right = self.t_now
            return right - self.span, right
        return self.left, self.left + self.span

    def _zoom(self, e, delta):
        left, right = self._view()
        span = right - left
        ml, pw = getattr(self, "_ml", 68), getattr(self, "_pw", 600)
        frac = min(max((e.x - ml) / max(pw, 1), 0.0), 1.0)
        tcur = left + frac * span
        factor = 0.8 if delta > 0 else 1.25
        new_span = min(max(span * factor, 100.0), HISTORY_MS)
        self.follow = False
        self.span = new_span
        self.left = tcur - frac * new_span
        self._redraw()

    def _press(self, e):
        self._drag_x = e.x
        self._drag_left = self._view()[0]
        self._press_x = e.x
        self._dragged = False

    def _motion(self, e):
        if self._drag_x is None:
            return
        if abs(e.x - self._press_x) > 2:
            self._dragged = True
        pw = getattr(self, "_pw", 600)
        self.follow = False
        self.left = self._drag_left - (e.x - self._drag_x) / max(pw, 1) * self.span
        self._redraw()

    def _release(self, e):
        if self._drag_x is not None and not self._dragged:
            self._set_marker(e.x)
        self._drag_x = None

    def _follow(self):
        self.follow = True
        self.left = None
        self.marker_t = None
        self._update_marker_readout()
        self._redraw()

    def _set_marker(self, x):
        """把画布 x 坐标换算成时间，放置竖线游标。"""
        left, right = self._view()
        ml, pw = getattr(self, "_ml", 68), getattr(self, "_pw", 600)
        frac = min(max((x - ml) / max(pw, 1), 0.0), 1.0)
        self.marker_t = left + frac * (right - left)
        if self._last is not None and self._last[1]:
            times = self._last[1]
            self.marker_t = min(max(self.marker_t, times[0][0]), times[-1][0])
        self._update_marker_readout()
        self._redraw()

    def _nearest_sample(self):
        if self.marker_t is None or self._last is None:
            return None
        samples = self._last[1]
        if not samples:
            return None
        return min(samples, key=lambda s: abs(s[0] - self.marker_t))

    def _update_marker_readout(self):
        if self.marker_t is None:
            self.marker_readout.configure(text="游标：单击曲线放置")
            return
        s = self._nearest_sample()
        if s is None:
            self.marker_readout.configure(
                text=f"游标 t={self.marker_t / 1000.0:.2f}s  无数据")
            return
        self.marker_readout.configure(
            text=(f"游标 t={self.marker_t / 1000.0:.2f}s  "
                  f"{self.target_label} {s[1]:+.2f}  "
                  f"{self.current_label} {s[2]:+.2f}  "
                  f"差 {s[2] - s[1]:+.2f}  {self.unit}"))

    def _redraw(self):
        if self._last is not None:
            self.draw(*self._last)

    # ---- 绘制 ----
    def draw(self, t_now, samples):
        self.t_now = t_now
        cv = self.canvas
        cv.delete("all")
        w, h = cv.winfo_width(), cv.winfo_height()
        if w < 40 or h < 40:
            return
        t_lo, t_hi = self._view()
        ml, mr, mt, mb = 68, 14, 14, 26
        pw, ph = w - ml - mr, h - mt - mb
        self._ml, self._pw = ml, pw
        mid = mt + ph / 2.0

        vis = [s for s in samples if t_lo <= s[0] <= t_hi]
        vals = []
        for s in vis:
            vals += [abs(s[1]), abs(s[2])]
        ymax = max(vals) if vals else 0.0
        ymax = max(ymax, self.yfloor) * 1.15

        def y2px(v):
            return mid - (v / ymax) * (ph / 2.0)

        def x2px(t):
            return ml + (t - t_lo) / max(t_hi - t_lo, 1e-6) * pw

        # 横向网格 + Y 刻度
        for frac in (1.0, 0.5, 0.0, -0.5, -1.0):
            yv = ymax * frac
            y = y2px(yv)
            cv.create_line(ml, y, w - mr, y, fill=(GRID if frac else "#555"))
            cv.create_text(ml - 6, y, text=f"{yv:+.0f}", fill=TXT_DIM,
                           anchor="e", font=("monospace", 9))
        # 纵向网格 + 时间刻度
        span = t_hi - t_lo
        for step in (100, 200, 500, 1000, 2000, 5000, 10000, 20000, 30000):
            if span / step <= 12:
                break
        k = int(t_lo // step)
        while k * step <= t_hi:
            t = k * step
            x = x2px(t)
            cv.create_line(x, mt, x, h - mb, fill=GRID)
            rel = (t - t_hi) / 1000.0
            lbl = "now" if abs(rel) < step / 2000.0 else f"{rel:+.0f}s"
            cv.create_text(x, h - mb + 4, text=lbl, fill=TXT_DIM,
                           anchor="n", font=("monospace", 9))
            k += 1
        cv.create_text(6, mt, text=self.unit, fill=TXT_DIM, anchor="nw",
                       font=("monospace", 9))
        cv.create_text(w - mr, mt, text=("跟随最新 (双击)" if self.follow else "已暂停 (双击跟随)"),
                       fill=("#6c6" if self.follow else "#c96"), anchor="ne",
                       font=("monospace", 9))

        if len(vis) > 1:
            pt, pm = [], []
            for s in vis:
                x = x2px(s[0])
                pt += [x, y2px(s[1])]
                pm += [x, y2px(s[2])]
            cv.create_line(*pt, fill=TARGET_COL, dash=(4, 3), width=2)
            cv.create_line(*pm, fill=self.color, width=2)

        # ---- 单击游标：平行于 Y 轴的竖线 + 该时刻的目标/读取值 ----
        if self.marker_t is not None:
            x = x2px(self.marker_t)
            if ml <= x <= w - mr:
                cv.create_line(x, mt, x, h - mb, fill="#ffd27f",
                               dash=(3, 3), width=2)
                near = (min(samples, key=lambda s: abs(s[0] - self.marker_t))
                        if samples else None)
                if near is not None:
                    anchor = "nw" if x < w - 230 else "ne"
                    tx = x + 6 if anchor == "nw" else x - 6
                    txt = (f"t={self.marker_t / 1000.0:.2f}s\n"
                           f"{self.target_label} {near[1]:+.2f}\n"
                           f"{self.current_label} {near[2]:+.2f}\n"
                           f"差 {near[2] - near[1]:+.2f} {self.unit}")
                    cv.create_text(tx, mt + 20, text=txt, anchor=anchor,
                                   fill="#ffd27f", font=("monospace", 9))

    def update(self, t_now, samples):
        self._last = (t_now, samples)
        self.draw(t_now, samples)
        if samples:
            tgt, cur = samples[-1][1], samples[-1][2]
            self.readout.configure(
                text=f"{self.target_label} {tgt:+.1f}    "
                     f"{self.current_label} {cur:+.1f}    "
                     f"差 {cur - tgt:+.1f}  {self.unit}")
        self._update_marker_readout()


# ---------------------------------------------------------------- 中文诊断表
class Diagnostics:
    def __init__(self, parent):
        import tkinter as tk
        self.vars = {}
        box = tk.Frame(parent, bg=BG, bd=1, relief="ridge")
        box.pack(fill="x", padx=6, pady=4)
        rows = [
            [("遥控", "rc"), ("IMU有效", "imu"), ("IMU状态", "st"),
             ("零偏已测", "bm"), ("航向使能", "hdg"), ("主动旋转", "hrot")],
            [("目标航向°", "ht"), ("实测偏航°", "rot"), ("误差°", "herr"), ("陀螺°/s", "gz")],
            [("Kp", "kp"), ("Ki", "ki"), ("Kd", "kd"), (None, None)],
            [("P项°/s", "hP"), ("I项°/s", "hI"), ("D项°/s", "hD"), ("输出°/s", "hcorr")],
            [("P限幅°/s", "pLim"), ("I限幅°/s", "iLim"), ("D限幅°/s", "dLim"),
             ("输出限幅°/s", "outLim"), ("积分限幅°·s", "intLim")],
            [("施加ω°/s", "tw"), ("斜坡前ω°/s", "wraw"), ("外部ω°/s", "hext"),
             ("前馈°/s", "ff"), ("零偏°/s", "bias"), ("漂移°/s", "drift")],
            [("轮命令 FL FR RL RR", "cmd"), ("轮实测 FL FR RL RR", "act")],
        ]
        for r, row in enumerate(rows):
            col = 0
            for label, key in row:
                if key is None:
                    col += 2
                    continue
                tk.Label(box, text=label, fg="#7fb", bg=BG,
                         font=("monospace", 9), anchor="e").grid(
                    row=r, column=col, sticky="e", padx=(8, 3), pady=1)
                v = tk.StringVar(value="—")
                tk.Label(box, textvariable=v, fg="#fff", bg=BG,
                         font=("monospace", 10, "bold"), anchor="w",
                         width=(22 if key in ("cmd", "act") else 8)).grid(
                    row=r, column=col + 1, sticky="w", padx=(0, 8), pady=1)
                self.vars[key] = v
                col += 2

    def update(self, d):
        f = self.vars
        f["rc"].set("有" if d["rc"] else "无")
        f["imu"].set("有效" if d["imu"] else "无效")
        f["st"].set(str(d["st"]))
        f["bm"].set("是" if d["bm"] else "否")
        f["hdg"].set("开" if d["hdg"] else "关")
        f["hrot"].set("是" if d["hrot"] else "否")
        f["ht"].set(f"{d['ht']:+.2f}")
        f["rot"].set(f"{wrap_deg(d['rot']):+.2f}")
        f["herr"].set(f"{d['herr']:+.2f}")
        f["gz"].set(f"{d['gz']:+.1f}")
        f["kp"].set(f"{d['kp']/1000:.2f}")
        f["ki"].set(f"{d['ki']/1000:.2f}")
        f["kd"].set(f"{d['kd']/1000:.2f}")
        f["hP"].set(f"{d['hP']:+.2f}")
        f["hI"].set(f"{d['hI']:+.2f}")
        f["hD"].set(f"{d['hD']:+.2f}")
        f["hcorr"].set(f"{d['hcorr']:+.2f}")
        f["pLim"].set(f"±{d['pLim']:.1f}")
        f["iLim"].set(f"±{d['iLim']:.1f}")
        f["dLim"].set(f"±{d['dLim']:.1f}")
        f["outLim"].set(f"±{d['outLim']:.1f}")
        f["intLim"].set(f"{d['intLim']:.1f}")
        f["tw"].set(f"{d['tw']:+.1f}")
        f["wraw"].set(f"{d['wraw']:+.1f}")
        f["hext"].set(f"{d['hext']:+.1f}")
        f["ff"].set(f"{d['ff']:+.1f}")
        f["bias"].set(f"{d['bias']:+.3f}")
        f["drift"].set(f"{d['drift']:+.3f}")
        f["cmd"].set(f"{d['cFL']/1000:.1f},{d['cFR']/1000:.1f},{d['cRL']/1000:.1f},{d['cRR']/1000:.1f}")
        f["act"].set(f"{d['aFL']/1000:.1f},{d['aFR']/1000:.1f},{d['aRL']/1000:.1f},{d['aRR']/1000:.1f}")


# ---------------------------------------------------------------- 主程序
CURVES = [
    ("航向角度环：目标 vs 实测", "°", "#ff6b6b",
     lambda d: (wrap_deg(d["rot"] + d["herr"]), wrap_deg(d["rot"])), 5),
    ("ω 指令（PID 输出）vs 陀螺实测", "°/s", "#ffb84d",
     lambda d: (d["tw"], d["gz"]), 30),
    ("前后 vx", "mm/s", "#4da3ff", lambda d: (d["tvx"], d["mvx"]), 50),
    ("平移 vy", "mm/s", "#7fe08a", lambda d: (d["tvy"], d["mvy"]), 50),
]


def main():
    args = sys.argv[1:]
    port, window_s, csv_path, headless_s = None, 10.0, None, None
    pos, i = [], 0
    while i < len(args):
        if args[i] == "--csv":
            i += 1
            csv_path = args[i]
        elif args[i] == "--headless":
            i += 1
            headless_s = float(args[i])
        else:
            pos.append(args[i])
        i += 1
    if pos:
        port = pos[0]
    if port is None:
        port = pick_port()
    if len(pos) > 1:
        window_s = float(pos[1])
    span_ms = max(window_s, 0.1) * 1000.0

    print(f"[pid_tune] 串口={port} 初始窗口={window_s}s csv={csv_path}")
    fd = open_serial(port)
    raw = collections.deque(maxlen=8000)
    stop = threading.Event()
    threading.Thread(target=reader_thread, args=(fd, raw, stop), daemon=True).start()

    samples = collections.deque()
    csv_f = open(csv_path, "w") if csv_path else None
    if csv_f:
        csv_f.write(",".join(FIELDS) + ",mvx,mvy\n")

    def drain():
        while raw:
            d = parse(raw.popleft())
            if d is None:
                continue
            d["mvx"], d["mvy"] = forward_kinematics(d["aFL"], d["aFR"], d["aRL"], d["aRR"])
            samples.append(d)
            if csv_f:
                csv_f.write(",".join(str(x) for x in
                                     ([d[k] for k in FIELDS] +
                                      [f"{d['mvx']:.1f}", f"{d['mvy']:.1f}"])) + "\n")

    if headless_s is not None:
        t0, last = time.time(), 0.0
        while time.time() - t0 < headless_s:
            drain()
            if time.time() - last > 1.0 and samples:
                last = time.time()
                d = samples[-1]
                print(f"  n={len(samples):5d} 目标={d['ht']:+.2f}° 偏航={d['rot']:+.2f}° "
                      f"误差={d['herr']:+.2f}° P={d['hP']:+.1f} I={d['hI']:+.1f} "
                      f"D={d['hD']:+.1f} 输出={d['hcorr']:+.1f}°/s")
            time.sleep(0.02)
    else:
        import tkinter as tk
        root = tk.Tk()
        root.title("V5 航向 PID 调参 — 诊断表")
        root.configure(bg=BG)
        tk.Label(root, text="固定位置诊断表", fg="#9cf", bg=BG,
                 font=("monospace", 11, "bold")).pack(fill="x", padx=6, pady=4)
        tk.Label(root, text="航向保持控制的是角度；ω 只是 PID 输出的角速度指令，不是独立速度环",
                 fg="#89c", bg=BG, font=("monospace", 9)).pack(fill="x", padx=6)
        tk.Label(root, text="曲线窗口：单击放置游标 / 滚轮缩放 / 左键拖动平移 / 双击跟随最新",
                 fg="#89c", bg=BG, font=("monospace", 9)).pack(fill="x", padx=6)
        tk.Label(root, text="航向角按 ±180° 折叠显示；误差取最短角差，不是 359° 那种长路径",
                 fg="#89c", bg=BG, font=("monospace", 9)).pack(fill="x", padx=6)

        angle_readout = tk.Label(
            root, text="航向角（角度环）：等待数据…", fg="#ffd27f", bg=BG,
            anchor="w", font=("monospace", 13, "bold"))
        angle_readout.pack(fill="x", padx=6, pady=(4, 2))

        diag = Diagnostics(root)

        graphs = []
        for k, (title, unit, color, pick, yfloor) in enumerate(CURVES):
            gw = GraphWindow(root, title, unit, color, pick, yfloor, span_ms)
            gw.win.geometry(f"760x360+{40 + k*32}+{40 + k*32}")
            graphs.append(gw)

        def tick():
            drain()
            t_now = samples[-1]["ms"] if samples else 0
            while samples and samples[0]["ms"] < t_now - HISTORY_MS:
                samples.popleft()
            if samples:
                d_last = samples[-1]
                diag.update(d_last)
                angle_readout.configure(
                    text=(f"航向角（角度环）  目标 {d_last['ht']:+.2f}°   "
                          f"实测 {wrap_deg(d_last['rot']):+.2f}°   "
                          f"误差 {d_last['herr']:+.2f}°  （最短角差）"))
            else:
                angle_readout.configure(text="航向角（角度环）：等待数据…")
            data = list(samples)
            for gw in graphs:
                gw.update(t_now, [(x["ms"],) + gw.pick(x) for x in data])
            root.after(33, tick)

        def on_close():
            stop.set()
            if csv_f:
                csv_f.close()
            root.destroy()

        root.protocol("WM_DELETE_WINDOW", on_close)
        for gw in graphs:
            gw.win.protocol("WM_DELETE_WINDOW", on_close)
        root.after(33, tick)
        root.mainloop()

    stop.set()
    if csv_f:
        csv_f.close()
    print(f"[pid_tune] 采集 {len(samples)} 帧 -> {csv_path or '(未存)'}")


if __name__ == "__main__":
    main()
