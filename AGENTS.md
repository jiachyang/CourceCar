# AGENTS.md

本仓库的唯一开发规范：**`docs/development_rules.md`**。
任何代码/文档改动前必须先读它；交付前必须通过该文件第 9 节的检查清单。

## 硬约束速查（细则见规范，冲突时以规范为准）

1. 分层：core → modules → main；**模块之间不得互相 include**；一个硬件只有一个管理者。
2. main 只做调度：禁止 IK / 电机写值 / 传感器读取 / 遥测格式 / 标定流程；规模 ≤ 200 行。
3. 遥测字段只在 `src/diag/telemetry.cpp` 定义，改字段必须同步 `test/pid_tune.py`。
4. 构建 `-Wall -Wextra` 零警告；纯逻辑改动必须有主机单测。
5. 不扩大改动范围；架构变更同步 `docs/architecture.md`。

## 交付前命令（必须全绿）

```bash
make -j4
bash scripts/check_rules.sh
bash scripts/test_native.sh
```
