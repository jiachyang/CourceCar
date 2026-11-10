#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从已确认的 maze_model.json 生成主机端测试用的 C++17 地图 fixture。

为什么需要这个脚本：
  - 车道保持是局部纠偏，不在固件里跑全局地图，避免把“已知全局地图”错误地
    耦合进实时控制；
  - 但模拟测试必须使用**同一份**墙体数据，不能靠示意图重新描点；
  - 因此以 JSON 为唯一真值，生成 `tests/native/fixtures/maze_model.hpp`，
    并在 `scripts/test_native.sh` 里用 --check 阻止 JSON 与 fixture 漂移。

用法：
  python3 scripts/gen_maze_fixture.py            # 生成
  python3 scripts/gen_maze_fixture.py --check    # 只校验是否最新
  python3 scripts/gen_maze_fixture.py --json test/data/maze_model.json
"""

import argparse
import hashlib
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_JSON = ROOT / "test" / "data" / "maze_model.json"
DEFAULT_OUT = ROOT / "tests" / "native" / "fixtures" / "maze_model.hpp"


def die(msg):
    print(f"[maze-fixture] ERROR: {msg}", file=sys.stderr)
    raise SystemExit(1)


def load_model(path):
    raw = path.read_bytes()
    try:
        model = json.loads(raw.decode("utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        die(f"无法读取 {path}: {exc}")
    sha = hashlib.sha256(raw).hexdigest()

    grid = model.get("grid") or {}
    if int(grid.get("cols", 0)) != 8 or int(grid.get("rows", 0)) != 8:
        die("grid 必须为 8x8")
    cell = float(grid.get("cell_size_m", 0.0))
    if abs(cell - 0.45) > 1e-12:
        die(f"cell_size_m 期望 0.45，实际 {cell}")

    vertical = model.get("fixed_vertical_segments") or []
    horizontal = model.get("fixed_horizontal_segments") or []
    movable = model.get("movable_segments") or {}
    sites = model.get("sites") or {}
    if not vertical or not horizontal:
        die("fixed 墙体为空")
    if len(movable) != 3:
        die(f"movable 隔板期望 3 条，实际 {len(movable)}")

    return {
        "model": model,
        "sha": sha,
        "cell": cell,
        "vertical": vertical,
        "horizontal": horizontal,
        "movable": movable,
        "sites": sites,
    }


def cxx_number(value):
    """尽量输出紧凑、可精确回读的 C++ double。"""
    f = float(value)
    if f == int(f):
        return f"{int(f)}.0"
    # repr() 在 Python 3 中保证 double 往返精度，且 0.45 不会写成 0.45000000000000001。
    return repr(f)


def generate(data, json_rel):
    v = data["vertical"]
    h = data["horizontal"]
    m = data["movable"]
    sites = data["sites"]

    lines = []
    lines.append("// 本文件由 scripts/gen_maze_fixture.py 生成，请勿手工编辑。")
    lines.append(f"// 源 JSON：{json_rel}")
    lines.append(f"// 源文件 SHA256：{data['sha']}")
    lines.append("// 用途：主机端车道保持/地图射线模拟测试；不参与固件实时控制。")
    lines.append("#pragma once")
    lines.append("")
    lines.append("#include <cstdint>")
    lines.append("")
    lines.append("namespace maze_fixture {")
    lines.append("")
    lines.append("struct VerticalSegment {")
    lines.append("    std::int32_t x = 0;")
    lines.append("    std::int32_t yStart = 0;")
    lines.append("    std::int32_t yEnd = 0;")
    lines.append("};")
    lines.append("")
    lines.append("struct HorizontalSegment {")
    lines.append("    std::int32_t y = 0;")
    lines.append("    std::int32_t xStart = 0;")
    lines.append("    std::int32_t xEnd = 0;")
    lines.append("};")
    lines.append("")
    lines.append("struct MovableSegment {")
    lines.append('    const char* name = "";')
    lines.append("    char orientation = 'V';")
    lines.append("    std::int32_t position = 0;")
    lines.append("    std::int32_t start = 0;")
    lines.append("    std::int32_t end = 0;")
    lines.append("};")
    lines.append("")
    lines.append("struct Site {")
    lines.append('    const char* id = "";')
    lines.append("    std::int32_t cellX = 0;")
    lines.append("    std::int32_t cellY = 0;")
    lines.append("    double xM = 0.0;")
    lines.append("    double yM = 0.0;")
    lines.append("};")
    lines.append("")
    lines.append("inline constexpr std::int32_t kCols = 8;")
    lines.append("inline constexpr std::int32_t kRows = 8;")
    lines.append(f"inline constexpr double kCellSizeM = {cxx_number(data['cell'])};")
    lines.append("inline constexpr double kFieldSizeM = kCols * kCellSizeM;")
    lines.append(f'inline constexpr const char* kSourceSha256 = "{data["sha"]}";')
    lines.append("")

    lines.append("inline constexpr VerticalSegment kFixedVertical[] = {")
    for s in v:
        lines.append(
            f"    VerticalSegment{{{int(s['x'])}, {int(s['y_start'])}, "
            f"{int(s['y_end'])}}},"
        )
    lines.append("};")
    lines.append("inline constexpr int kFixedVerticalCount =")
    lines.append("    static_cast<int>(sizeof(kFixedVertical) / sizeof(kFixedVertical[0]));")
    lines.append("")

    lines.append("inline constexpr HorizontalSegment kFixedHorizontal[] = {")
    for s in h:
        lines.append(
            f"    HorizontalSegment{{{int(s['y'])}, {int(s['x_start'])}, "
            f"{int(s['x_end'])}}},"
        )
    lines.append("};")
    lines.append("inline constexpr int kFixedHorizontalCount =")
    lines.append(
        "    static_cast<int>(sizeof(kFixedHorizontal) / sizeof(kFixedHorizontal[0]));"
    )
    lines.append("")

    lines.append("inline constexpr MovableSegment kMovable[] = {")
    for name in ("M1", "M2", "M3"):
        if name not in m:
            die(f"缺少活动隔板 {name}")
        s = m[name]
        orient = str(s.get("orientation", "")).upper()
        if orient not in ("V", "H"):
            die(f"{name} orientation 非法：{orient}")
        lines.append(
            f'    MovableSegment{{"{name}", \'{orient}\', '
            f"{int(s['position'])}, {int(s['start'])}, {int(s['end'])}}},"
        )
    lines.append("};")
    lines.append("inline constexpr int kMovableCount =")
    lines.append("    static_cast<int>(sizeof(kMovable) / sizeof(kMovable[0]));")
    lines.append("")

    lines.append("inline constexpr Site kSites[] = {")
    for key in ("S", "F", "A", "B", "C"):
        if key not in sites:
            die(f"sites 缺少 {key}")
        s = sites[key]
        cell_xy = s.get("cell") or []
        center = s.get("center_m_from_top_left") or []
        if len(cell_xy) != 2 or len(center) != 2:
            die(f"site {key} 缺少 cell/center")
        lines.append(
            f'    Site{{"{key}", {int(cell_xy[0])}, {int(cell_xy[1])}, '
            f"{cxx_number(center[0])}, {cxx_number(center[1])}}},"
        )
    lines.append("};")
    lines.append("inline constexpr int kSiteCount =")
    lines.append("    static_cast<int>(sizeof(kSites) / sizeof(kSites[0]));")
    lines.append("")
    lines.append("}  // namespace maze_fixture")
    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=pathlib.Path, default=DEFAULT_JSON)
    parser.add_argument("--out", type=pathlib.Path, default=DEFAULT_OUT)
    parser.add_argument("--check", action="store_true",
                        help="只检查 out 是否与 JSON 生成结果一致")
    args = parser.parse_args()

    json_path = args.json.resolve()
    out_path = args.out.resolve()
    data = load_model(json_path)

    try:
        json_rel = json_path.relative_to(ROOT).as_posix()
    except ValueError:
        json_rel = str(json_path)

    text = generate(data, json_rel)

    if args.check:
        try:
            current = out_path.read_text(encoding="utf-8")
        except OSError as exc:
            die(f"无法读取 {out_path}: {exc}")
        if current != text:
            print(
                "[maze-fixture] fixture 与 JSON 不一致。"
                f"请运行：python3 scripts/gen_maze_fixture.py --json {json_rel}",
                file=sys.stderr,
            )
            return 1
        print(f"[maze-fixture] OK：{out_path.relative_to(ROOT)} 与 JSON 一致")
        return 0

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(text, encoding="utf-8")
    print(f"[maze-fixture] 已生成 {out_path.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
