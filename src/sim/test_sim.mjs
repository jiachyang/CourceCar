#!/usr/bin/env node
/**
 * 主机端仿真自检：从 maze_nav_sim.html 抽取 DOM-free 核心，跑 8 种挡板配置。
 *
 * 用法：node src/sim/test_sim.mjs
 * 不依赖浏览器；不参与 PROS 构建。
 */
import fs from 'node:fs';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';

const root = fileURLToPath(new URL('../..', import.meta.url));
const html = fs.readFileSync(new URL('./maze_nav_sim.html', import.meta.url), 'utf8');
const match = html.match(/\/\/ ==== SIM CORE START ====([\s\S]*?)\/\/ ==== SIM CORE END ====/);
if (!match) throw new Error('找不到 SIM CORE 标记');
const core = new Function(match[1] + `
  return { createSim, selfCheck, MAP, F_CELL, CELL, policyValue, S_CELL, encodeBelief };
`)();

const json = JSON.parse(fs.readFileSync(root + 'test/data/maze_model.json', 'utf8'));

function checkMap() {
  assert.deepEqual(core.MAP.grid, json.grid, 'grid');
  assert.deepEqual(core.MAP.sites, json.sites, 'sites');
  assert.deepEqual(
    core.MAP.fixed_vertical_segments,
    json.fixed_vertical_segments.map(s => [s.x, s.y_start, s.y_end]),
    'fixed vertical'
  );
  assert.deepEqual(
    core.MAP.fixed_horizontal_segments,
    json.fixed_horizontal_segments.map(s => [s.y, s.x_start, s.x_end]),
    'fixed horizontal'
  );
  assert.deepEqual(core.MAP.movable_segments, json.movable_segments, 'movable');
  assert.equal(core.MAP.source_sha256, '5bed05050cc36a2813fd2360d4b2c390e2eb847d7f7b316e62c63cf0fe10b601');
}

function runCase(bits, opts) {
  const sim = core.createSim(Object.assign({ barriers: bits }, opts));
  const s = sim.runUntil(300);
  const err = Math.hypot(s.est.x - s.truePose.x, s.est.y - s.truePose.y);
  const wrongObs = s.obsLog.filter(o => !o.timeout && o.value !== bits[o.barrier]);
  return { sim, s, err, wrongObs };
}

const cases = [];
for (let bits = 0; bits < 8; ++bits) {
  cases.push([(bits >> 2) & 1, (bits >> 1) & 1, bits & 1]);
}

console.log('== 地图数据 ==');
checkMap();
console.log('PASS: 16 竖墙 + 11 横墙 + 3 活动挡板与 maze_model.json 一致');

console.log('== 规划自检 ==');
const chk = core.selfCheck();
assert.ok(chk.ok, chk.report.join('; '));
for (const bits of cases) {
  assert.ok(Number.isFinite(core.policyValue(core.S_CELL, 0, 0, bits)), 'config ' + bits);
}
console.log('PASS: 8 种已知配置与全未知信念均有有限代价');

console.log('== 标称工况：ToF σ=8 mm，编码器打滑 2%，IMU 漂移 0 ==');
for (const bits of cases) {
  const { s, err, wrongObs } = runCase(bits, { seed: 1, noiseMm: 8, slipPct: 2, driftDps: 0, cruiseMps: 0.55 });
  assert.ok(s.done, '未完成 ' + bits + ' phase=' + s.phase);
  assert.ok(!s.stuck, '卡死 ' + bits);
  assert.equal(s.visited, 7, '未访问 A/B/C ' + bits);
  assert.equal(s.cell, core.F_CELL, '未到 F ' + bits);
  assert.ok(s.collisions < 200, '碰撞过多 ' + bits + ' = ' + s.collisions);
  assert.ok(err < 0.15, '估计误差过大 ' + bits + ' = ' + err.toFixed(3));
  assert.equal(wrongObs.length, 0, '挡板误判 ' + bits);
  console.log(`PASS: ${bits.join('')}  t=${s.time.toFixed(1)}s  coll=${s.collisions}  err=${(err * 100).toFixed(1)}cm  known=${s.known.join('')}`);
}

console.log('== 压力工况：ToF σ=20 mm，编码器打滑 8%，IMU 漂移 0.02°/s ==');
for (const bits of cases) {
  const { s, err, wrongObs } = runCase(bits, { seed: 4, noiseMm: 20, slipPct: 8, driftDps: 0.02, cruiseMps: 0.55 });
  assert.ok(s.done, '压力工况未完成 ' + bits + ' phase=' + s.phase);
  assert.ok(!s.stuck, '压力工况卡死 ' + bits);
  assert.equal(s.visited, 7, '压力工况未访问 A/B/C ' + bits);
  assert.equal(s.cell, core.F_CELL, '压力工况未到 F ' + bits);
  assert.ok(s.collisions < 300, '压力工况碰撞过多 ' + bits + ' = ' + s.collisions);
  assert.ok(err < 0.15, '压力工况估计误差过大 ' + bits + ' = ' + err.toFixed(3));
  assert.equal(wrongObs.length, 0, '压力工况挡板误判 ' + bits);
  console.log(`PASS: ${bits.join('')}  t=${s.time.toFixed(1)}s  coll=${s.collisions}  err=${(err * 100).toFixed(1)}cm  known=${s.known.join('')}`);
}

console.log('== 全封闭挡板：三个都必须被识别 ==');
{
  const { s } = runCase([1, 1, 1], { seed: 2, noiseMm: 8, slipPct: 2, driftDps: 0, cruiseMps: 0.55 });
  assert.deepEqual(s.known, [1, 1, 1], '全封闭挡板未全部识别');
  console.log('PASS: M1/M2/M3 全部识别为封闭，t=' + s.time.toFixed(1) + 's');
}

console.log('\nmaze navigation simulation tests: ALL PASS');
