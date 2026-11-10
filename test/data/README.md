# test/data

## `maze_model.json`

- 来源：`/home/jc/下载/maze_model.json`（用户提供的已确认迷宫数据）
- 复制方式：逐字节复制，未重新描点
- SHA256：`5bed05050cc36a2813fd2360d4b2c390e2eb847d7f7b316e62c63cf0fe10b601`
- 作用：`scripts/gen_maze_fixture.py` 的唯一输入；生成
  `tests/native/fixtures/maze_model.hpp`，供主机端射线/车道保持模拟测试使用。

固件实时控制**不加载**这张全局地图；LaneKeeper 是局部纠偏模块。
更新 JSON 后重新运行：

```bash
python3 scripts/gen_maze_fixture.py
bash scripts/test_native.sh
```
