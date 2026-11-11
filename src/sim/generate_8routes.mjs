#!/usr/bin/env node
/**
 * 生成 8 种挡板配置（M1/M2/M3 = 000..111）的路线测试数据。
 *
 * 用法：node src/sim/generate_8routes.mjs [seed] [noiseMm] [slipPct] [driftDps]
 * 默认：seed=1, noise=8 mm, slip=2%, drift=0（标称）。
 *
 * 输出（src/sim/）：
 *   route_test_summary.csv   8 行汇总
 *   route_test_summary.md    可读汇总
 *   route_test_trace.csv     每个 case 的到达/观测/完成时间线
 *   route_test_data.json     含完整 route / obsLog / events
 */
import fs from 'node:fs';
import { fileURLToPath } from 'node:url';

const here = fileURLToPath(new URL('.', import.meta.url));
const html = fs.readFileSync(here + 'maze_nav_sim.html', 'utf8');
const match = html.match(/\/\/ ==== SIM CORE START ====([\s\S]*?)\/\/ ==== SIM CORE END ====/);
if (!match) throw new Error('找不到 SIM CORE 标记');
const core = new Function(match[1] + `
  return { createSim, cellXY, F_CELL, CFG };
`)();

const args = process.argv.slice(2).map(Number);
const seed = Number.isFinite(args[0]) && args[0] > 0 ? args[0] : 1;
const noiseMm = Number.isFinite(args[1]) ? args[1] : 8;
const slipPct = Number.isFinite(args[2]) ? args[2] : 2;
const driftDps = Number.isFinite(args[3]) ? args[3] : 0;
const cruiseMps = 0.55;

const cases = [];
for (let bits = 0; bits < 8; ++bits) {
  const bars = [(bits >> 2) & 1, (bits >> 1) & 1, bits & 1];
  const sim = core.createSim({
    barriers: bars, seed, noiseMm, slipPct, driftDps, cruiseMps
  });
  const s = sim.runUntil(300);
  const errCm = Math.hypot(s.est.x - s.truePose.x, s.est.y - s.truePose.y) * 100;
  const yawErrDeg = Math.abs(Math.atan2(Math.sin(s.est.yaw - s.truePose.yaw),
    Math.cos(s.est.yaw - s.truePose.yaw))) * 180 / Math.PI;
  const route = s.route.map(p => {
    const q = core.cellXY(p);
    return q.x + ',' + q.y;
  });
  const obs = s.obsLog.map(o => `M${o.barrier + 1}=${o.value}${o.timeout ? '!' : ''}`);
  cases.push({
    case: bits, m1: bars[0], m2: bars[1], m3: bars[2],
    done: s.done, stuck: s.stuck, time_s: +s.time.toFixed(2),
    collisions: s.collisions, visited: s.visited,
    known: s.known.slice(), obs, obsLog: s.obsLog.slice(),
    pose_error_cm: +errCm.toFixed(2), yaw_error_deg: +yawErrDeg.toFixed(2),
    moves: s.route.length - 1, route, routeLog: s.routeLog.slice(),
    events: s.events.slice()
  });
}

// summary csv
const sumHead = 'case,m1,m2,m3,done,stuck,time_s,collisions,visited,known,observations,pose_error_cm,yaw_error_deg,moves,route';
const sumRows = cases.map(c => [
  c.case, c.m1, c.m2, c.m3, c.done, c.stuck, c.time_s, c.collisions,
  c.visited, '"' + c.known.join(' ') + '"', '"' + c.obs.join(' ') + '"',
  c.pose_error_cm, c.yaw_error_deg, c.moves, '"' + c.route.join(' ') + '"'
].join(','));
fs.writeFileSync(here + 'route_test_summary.csv', sumHead + '\n' + sumRows.join('\n') + '\n');

// trace csv
const traceHead = 'case,m1,m2,m3,seq,time_s,type,cell_x,cell_y,site,barrier,value,visited,known';
const traceRows = [];
for (const c of cases) {
  c.routeLog.forEach((e, i) => {
    const q = e.cell !== undefined ? core.cellXY(e.cell) : { x: '', y: '' };
    traceRows.push([
      c.case, c.m1, c.m2, c.m3, i, e.t.toFixed(2), e.type,
      q.x, q.y, e.site || '', e.barrier !== undefined ? 'M' + (e.barrier + 1) : '',
      e.value !== undefined ? e.value : '',
      e.visited !== undefined ? e.visited : '',
      '"' + (e.known ? e.known.join(' ') : '') + '"'
    ].join(','));
  });
}
fs.writeFileSync(here + 'route_test_trace.csv', traceHead + '\n' + traceRows.join('\n') + '\n');

// markdown
let md = '# 8 种挡板配置的路线测试数据\n\n';
md += `参数：seed=${seed}，ToF σ=${noiseMm} mm，编码器打滑=${slipPct}%，IMU 漂移=${driftDps}°/s，巡航=${cruiseMps} m/s。\n\n`;
md += '| case | M1 M2 M3 | done | time (s) | collisions | known | observations | moves | pose err (cm) |\n';
md += '| --- | --- | --- | --- | --- | --- | --- | --- | --- |\n';
for (const c of cases) {
  md += `| ${c.case} | ${c.m1} ${c.m2} ${c.m3} | ${c.done} | ${c.time_s.toFixed(1)} | ${c.collisions} | ${c.known.join(' ')} | ${c.obs.join(' ') || '—'} | ${c.moves} | ${c.pose_error_cm.toFixed(1)} |\n`;
}
md += '\n## 每个 case 的格路径\n\n';
for (const c of cases) {
  md += `### case ${c.case}（M1=${c.m1}, M2=${c.m2}, M3=${c.m3}）\n\n`;
  md += '```text\n' + c.route.join('  ') + '\n```\n\n';
}
fs.writeFileSync(here + 'route_test_summary.md', md);

// json
fs.writeFileSync(here + 'route_test_data.json', JSON.stringify({
  config: { seed, noiseMm, slipPct, driftDps, cruiseMps },
  cases
}, null, 2) + '\n');

console.log('case  bars  done  time_s  coll  known   obs                 moves');
for (const c of cases) {
  console.log(
    String(c.case).padEnd(5),
    `${c.m1}${c.m2}${c.m3}`.padEnd(5),
    String(c.done).padEnd(5),
    c.time_s.toFixed(1).padStart(6),
    String(c.collisions).padStart(5),
    c.known.join('').padEnd(7),
    (c.obs.join(' ') || '—').padEnd(19),
    String(c.moves).padStart(5)
  );
}
console.log('\nwrote route_test_summary.csv / .md, route_test_trace.csv, route_test_data.json');
