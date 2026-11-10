#!/usr/bin/env bash
#
# 一键实时显示四路 V5 Distance Sensor 读数。
#
# 做的事：
#   1. 启动 V5 的 slot 1（当前固件的 opcontrol）
#   2. 运行 test/distance_monitor.py 实时显示
#   3. 脚本退出 / Ctrl-C 时停止 V5 程序
#
# 用法：
#   bash test/distance_realtime.sh
#   bash test/distance_realtime.sh --csv distance.csv
#   bash test/distance_realtime.sh /dev/ttyACM1
#   bash test/distance_realtime.sh --headless 5
#
# 说明：
#   - V5 User 口通常为 /dev/ttyACM1；不传端口时由 Python 自动查找。
#   - 如果程序已经在运行，可以直接用 test/distance_monitor.py，不必用本脚本。
set -euo pipefail

cd "$(dirname "$0")/.."

cleanup() {
    pros v5 stop >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "[distance] 启动 V5 slot 1 ..."
pros v5 run 1 >/dev/null
sleep 0.5

echo "[distance] 开始实时读取（Ctrl-C 退出）..."
python3 test/distance_monitor.py "$@"
