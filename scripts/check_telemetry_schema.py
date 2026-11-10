#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""校验固件遥测字段表与上位机解析脚本一致。

1) $P：src/diag/telemetry.cpp 的 DIAG_TUNE_FIELDS ↔ test/pid_tune.py 的 FIELDS；
2) $D 车道追加字段数：telemetry.cpp 的 kLaneFieldCount ↔
   test/distance_monitor.py 的 LANE_FIELDS 长度。

用法：python3 scripts/check_telemetry_schema.py（scripts/test_native.sh 会调用）
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
CPP = ROOT / "src" / "diag" / "telemetry.cpp"
TUNE_PY = ROOT / "test" / "pid_tune.py"
MONITOR_PY = ROOT / "test" / "distance_monitor.py"


def firmware_fields():
    text = CPP.read_text(encoding="utf-8")
    try:
        block = text.split("#define DIAG_TUNE_FIELDS(X)", 1)[1]
        block = block.split("#define X", 1)[0]
    except IndexError:
        sys.exit("error: DIAG_TUNE_FIELDS(X) not found in %s" % CPP)
    names = re.findall(r"^\s*X\(([A-Za-z_]\w*),", block, re.M)
    if not names:
        sys.exit("error: no X(name, ...) entries found in DIAG_TUNE_FIELDS")
    return names


def host_fields():
    text = TUNE_PY.read_text(encoding="utf-8")
    match = re.search(r"FIELDS\s*=\s*\[(.*?)\]", text, re.S)
    if not match:
        sys.exit("error: FIELDS not found in %s" % TUNE_PY)
    return re.findall(r'"([A-Za-z_]\w*)"', match.group(1))


def lane_field_counts():
    cpp = CPP.read_text(encoding="utf-8")
    match = re.search(r"kLaneFieldCount\s*=\s*(\d+)", cpp)
    if not match:
        sys.exit("error: kLaneFieldCount not found in %s" % CPP)
    fw = int(match.group(1))

    py = MONITOR_PY.read_text(encoding="utf-8")
    match = re.search(r"LANE_FIELDS\s*=\s*\[(.*?)\]", py, re.S)
    if not match:
        sys.exit("error: LANE_FIELDS not found in %s" % MONITOR_PY)
    names = re.findall(r'"([A-Za-z_]\w*)"', match.group(1))
    return fw, names


def main():
    fw = firmware_fields()
    host = host_fields()
    if not host or host[0] != "ms":
        sys.exit("error: %s FIELDS must start with 'ms'" % TUNE_PY)
    if host[1:] != fw:
        print("error: $P 字段顺序不一致")
        print("  固件 DIAG_TUNE_FIELDS:", len(fw), "个载荷字段")
        print("  脚本 FIELDS[1:]:      ", len(host) - 1, "个字段")
        for i, (a, b) in enumerate(zip(host[1:], fw)):
            if a != b:
                print("  第 %d 个字段不同：脚本=%s 固件=%s" % (i, a, b))
                break
        if len(host) - 1 != len(fw):
            print("  字段数不同：脚本 %d，固件 %d" % (len(host) - 1, len(fw)))
        return 1

    lane_count, lane_names = lane_field_counts()
    if len(lane_names) != lane_count:
        print("error: $D 车道字段数不一致")
        print("  固件 kLaneFieldCount:", lane_count)
        print("  LANE_FIELDS:         ", len(lane_names))
        return 1

    print("telemetry schema tests: ALL PASS (%d $P fields, %d $D lane fields)"
          % (len(fw), lane_count))
    return 0


if __name__ == "__main__":
    sys.exit(main())
