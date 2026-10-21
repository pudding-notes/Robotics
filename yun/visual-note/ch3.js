/* =========================================================
   Chapter 3 — Manipulator Inverse Kinematics (Lecture 3)
   ========================================================= */
(function () {
  'use strict';
  const R = String.raw;
  const { I, mul, rx, ry, rz, tr, inv, app, R3, p3, fromRp, RT, col, unit, cross, norm, scl, add, sub, atan2d, f, tm, tv, D } = M;
  const K = window.KIN;
  const { chainT, robot, dragSpec, act, hlIf, fr, pt, vec, ln, arc, body, drop, watch } = K;
  const X = [1, 0, 0], Yv = [0, 1, 0], Z = [0, 0, 1], O = [0, 0, 0];
  const deg = x => f(x, 2) + '^\\circ';
  const wrap = a => ((a + 540) % 360) - 180;
  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
  const base = rx(90); // arm plane = world XZ, Z0 toward viewer

  /* ---------- planar IK helpers ---------- */
  // 2-link: returns both (θ1, θ2) with θ2 ≥ 0 first ("elbow +") and θ2 ≤ 0 second
  function ik2(L1, L2, x, y) {
    const c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    const ok = c2 >= -1 - 1e-9 && c2 <= 1 + 1e-9;
    const cc = clamp(c2, -1, 1), s = Math.sqrt(Math.max(0, 1 - cc * cc));
    const sols = [s, -s].map(s2 => {
      const t2 = atan2d(s2, cc), k1 = L1 + L2 * cc, k2 = L2 * s2;
      return { t1: wrap(atan2d(y, x) - atan2d(k2, k1)), t2, k1, k2, s2 };
    });
    return { c2, ok, sols };
  }
  function ik3(L, x, y, phi) {
    const wx = x - L[2] * Math.cos(phi * D), wy = y - L[2] * Math.sin(phi * D);
    const r = ik2(L[0], L[1], wx, wy);
    r.sols.forEach(s => { s.t3 = wrap(phi - s.t1 - s.t2); });
    r.w = [wx, wy];
    return r;
  }
  const row = a => ({ al: 0, a, d: 0, th: 0, type: 'R' });
  const planar = L => [row(0), row(L[0]), row(L[1])];
  const planar2 = L => [row(0), row(L[0])];
  const plane = () => ({ frame: 'P0', type: 'plane', get: p => [p.x, p.y], set: (p, v) => { p.x = M.round(clamp(v[0], -3, 3), 3); p.y = M.round(clamp(v[1], -3, 3), 3); } });
  const ring = (S, id, r, label, o) => { if (r > 1e-3) arc(S, id, Object.assign({ f: 'aF0', c: [0, 0, 0], n: Z, u: X, ang: 359.5, r, noHead: true, col: 'mute', label }, o || {})); };
  const target = (S, at, label, c) => { fr(S, 'P0', base, { hidden: true }); body(S, 'tgt', 'W', 'sph', [0.07], app(base, [at[0], at[1], 0.14]), { c: c || 'k', drag: 'tgt' }); pt(S, 'tgtL', app(base, [at[0], at[1], 0.0]), label || '목표', { c: c || 'k', r: 0.01 }); };

  const sections = [];
  const sec = o => sections.push(o);

  /* ================= 3.1 IK & existence ================= */
  {
    const v0 = { L1: 1.2, L2: 0.8, x: 1.3, y: 0.9 };
    const draw = (S, p, lv) => {
      const L = [p.L1, p.L2], r = ik2(p.L1, p.L2, p.x, p.y);
      fr(S, 'W', I(), { hidden: true });
      let qa, qb = null;
      if (r.ok) { qa = [r.sols[0].t1, r.sols[0].t2]; qb = [r.sols[1].t1, r.sols[1].t2]; }
      else qa = [atan2d(p.y, p.x), r.c2 > 1 ? 0 : 180];
      robot(S, planar2(L), qa, { base, tool: p.L2, pre: 'a', fs: 0.3, w: 0.07, jr: 0.09, jh: 0.16, noDrag: true, hide: [1, 2] });
      if (qb && Math.abs(r.sols[0].t2) > 0.5) robot(S, planar2(L), qb, { base, tool: p.L2, pre: 'b', noFrames: true, op: 0.28, w: 0.07, jr: 0.09, jh: 0.16, noDrag: true });
      target(S, [p.x, p.y], r.ok ? '목표' : '목표 (도달 불가)', r.ok ? 'k' : 'warn');
      if (lv >= 2) {
        ring(S, 'ro', p.L1 + p.L2, 'r<sub>out</sub> = L₁+L₂');
        ring(S, 'ri', Math.abs(p.L1 - p.L2), 'r<sub>in</sub> = |L₁−L₂|');
      }
      return r;
    };
    sec({
      id: 'c3-intro', group: 'lecture', ref: '슬라이드 3–4', title: '3.1 역기구학과 해의 존재',
      view: { pos: [0.5, -7.4, 0.7], target: [0.5, 0, 0.5] }, grid: [3, 0.5], scale: 0.7, fsize: 1,
      build() {
        return {
          init(S) { draw(S, v0, 1); },
          steps: [
            {
              t: '순기구학은 하나, 역기구학은 여러 개',
              tex: R`\begin{bmatrix}\theta_1\\ \vdots\\ \theta_n\end{bmatrix}\ \underset{\text{inverse (depends)}}{\overset{\text{forward (unique)}}{\rightleftarrows}}\ \begin{bmatrix}P\\ O\end{bmatrix}`,
              html: R`<p><b>역기구학(inverse kinematics)</b>은 끝점의 목표 위치·방향이 주어졌을 때 그것을 만드는 관절값을 찾는 문제입니다. 관절값 → 끝점은 하나로 정해지지만, 거꾸로는 해가 <b>하나, 여러 개, 또는 없을</b> 수 있습니다.</p>
              <p>화면: 관절 2개짜리 팔이 같은 목표점에 두 가지 자세(진한 팔, 흐린 팔)로 닿습니다.</p>`
            },
            {
              t: '작업공간과 해의 개수 — 직접 바꿔 보기',
              live: {
                assume: p => { const r = Math.hypot(p.x, p.y); return [R`관절 각도 제한 없음 (360° 회전 가능)`, { t: R`\(|L_1-L_2| \le r \le L_1+L_2\) (목표가 고리 안)`, ok: r <= p.L1 + p.L2 + 1e-3 && r >= Math.abs(p.L1 - p.L2) - 1e-3 }]; },
                params: [
                  { id: 'L1', label: R`\(L_1\)`, min: 0.3, max: 1.6, step: 0.05, val: v0.L1, dp: 2 },
                  { id: 'L2', label: R`\(L_2\)`, min: 0.3, max: 1.6, step: 0.05, val: v0.L2, dp: 2 },
                  { id: 'x', label: R`\(x\)`, min: -3, max: 3, step: 0.01, val: v0.x, dp: 2 },
                  { id: 'y', label: R`\(y\)`, min: -3, max: 3, step: 0.01, val: v0.y, dp: 2 }
                ],
                drag: { tgt: plane() },
                apply(S, p) { drop(S, 'bodies'); drop(S, 'frames'); drop(S, 'arcs'); drop(S, 'pts'); draw(S, p, 2); },
                tex(p) {
                  const r = Math.hypot(p.x, p.y), ro = p.L1 + p.L2, ri = Math.abs(p.L1 - p.L2);
                  const n = r > ro + 1e-3 || r < ri - 1e-3 ? 0 : (Math.abs(r - ro) < 0.01 || Math.abs(r - ri) < 0.01 ? 1 : 2);
                  const dex = Math.abs(p.L1 - p.L2) < 0.026 ? R`L_1 = L_2:\ \text{dextrous = 원점 한 점, reachable = 원}` : R`L_1 \ne L_2:\ \text{dextrous 없음, reachable = 고리(ring)}`;
                  return R`\begin{aligned}&r_{in} = |L_1-L_2| = ${f(ri, 2)}\ \le\ r = \sqrt{x^2+y^2} = ${f(r, 3)}\ \le\ r_{out} = L_1+L_2 = ${f(ro, 2)}\ ?\\ &\Rightarrow\ \text{해의 개수} = ${n}${n === 1 ? R`\ (\text{경계: 팔이 다 펴지거나 접힘})` : n === 0 ? R`\ (\text{작업공간 밖})` : ''}\qquad ${dex}\end{aligned}`;
                }
              },
              html: R`<p><b>작업공간(workspace)</b>: 끝점이 닿을 수 있는 영역. 회색 원 두 개 사이의 고리가 이 팔의 reachable workspace입니다. 보라 점(목표)을 마우스로 끌어 보세요.</p>
              <p>고리 안쪽에서는 해가 2개(흐린 팔 = 두 번째 해), 경계에서는 팔이 완전히 펴지거나 접혀 1개, 밖에서는 0개입니다. \(L_1 = L_2\)로 맞추면 안쪽 원이 사라져 원점까지 닿고, 원점에서는 모든 방향이 가능합니다(dextrous).</p>`
            }
          ]
        };
      }
    });
  }

  /* ================= 3.2 multiple solutions ================= */
  {
    const L = [1.2, 1.0, 0.5];
    const v0 = { x: 1.4, y: 1.2, phi: 60, c1: 100, c2: -60, c3: 0 };
    const draw = (S, p) => {
      fr(S, 'W', I(), { hidden: true });
      const r = ik3(L, p.x, p.y, p.phi);
      const qc = [p.c1, p.c2, p.c3];
      robot(S, planar(L), qc, { base, tool: L[2], pre: 'c', noFrames: true, op: 0.16, noDrag: true, w: 0.06, jr: 0.08, jh: 0.12 });
      if (!r.ok) { target(S, [p.x, p.y], '도달 불가', 'warn'); return { r, best: -1 }; }
      const dist = r.sols.map(s => Math.abs(wrap(s.t1 - qc[0])) + Math.abs(wrap(s.t2 - qc[1])) + Math.abs(wrap(s.t3 - qc[2])));
      const best = dist[0] <= dist[1] ? 0 : 1;
      r.sols.forEach((s, i) => robot(S, planar(L), [s.t1, s.t2, s.t3], { base, tool: L[2], pre: 's' + i, noFrames: i !== best, hide: [0, 1, 2], op: i === best ? null : 0.35, noDrag: true, glow: i === best ? [1, 2, 3] : [], w: 0.08, jr: 0.1, jh: 0.16 }));
      target(S, [p.x, p.y], '목표');
      vec(S, 'phi', app(base, [p.x, p.y, 0]), app(base, [p.x + 0.45 * Math.cos(p.phi * D), p.y + 0.45 * Math.sin(p.phi * D), 0]), { c: 'k', w: 0.8, label: 'φ' });
      return { r, best, dist };
    };
    sec({
      id: 'c3-multi', group: 'lecture', ref: '슬라이드 5–6', title: '3.2 여러 해와 해 고르기',
      view: { pos: [0.7, -6.2, 0.9], target: [0.7, 0, 0.8] }, grid: [3, 0.5], scale: 0.7, fsize: 1,
      build() {
        return {
          init(S) { },
          steps: [
            {
              t: '같은 위치·방향, 두 가지 자세',
              live: {
                assume: p => [R`관절 제한·장애물 없음`, R`선택 기준: 현재 자세에서 \(\sum|\Delta\theta_i|\)가 가장 작은 해`],
                params: [
                  { id: 'x', label: R`\(x\)`, min: -2.5, max: 2.5, step: 0.01, val: v0.x, dp: 2 },
                  { id: 'y', label: R`\(y\)`, min: -2.5, max: 2.5, step: 0.01, val: v0.y, dp: 2 },
                  { id: 'phi', label: R`\(\phi\)`, min: -180, max: 180, step: 1, val: v0.phi, unit: '°' },
                  { id: 'c1', label: R`현재 \(\theta_1\)`, min: -180, max: 180, step: 1, val: v0.c1, unit: '°' },
                  { id: 'c2', label: R`현재 \(\theta_2\)`, min: -180, max: 180, step: 1, val: v0.c2, unit: '°' },
                  { id: 'c3', label: R`현재 \(\theta_3\)`, min: -180, max: 180, step: 1, val: v0.c3, unit: '°' }
                ],
                drag: { tgt: plane() },
                apply(S, p) { drop(S, 'bodies'); drop(S, 'frames'); drop(S, 'vecs'); drop(S, 'pts'); const o = draw(S, p); p._best = o.best; },
                tex(p) {
                  const r = ik3(L, p.x, p.y, p.phi), qc = [p.c1, p.c2, p.c3];
                  if (!r.ok) return R`\text{목표가 작업공간 밖: 해 없음}`;
                  const dist = r.sols.map(s => Math.abs(wrap(s.t1 - qc[0])) + Math.abs(wrap(s.t2 - qc[1])) + Math.abs(wrap(s.t3 - qc[2])));
                  const b = dist[0] <= dist[1] ? 0 : 1;
                  const line = (s, i) => hlIf(i === b, R`\text{해 ${i + 1}: } (${f(s.t1, 1)}^\circ,\ ${f(s.t2, 1)}^\circ,\ ${f(s.t3, 1)}^\circ),\ \ \textstyle\sum|\Delta\theta| = ${f(dist[i], 1)}^\circ`);
                  return R`\begin{aligned}&${line(r.sols[0], 0)}\\ &${line(r.sols[1], 1)}\end{aligned}`;
                }
              },
              html: R`<p>손끝 위치 (x, y)와 방향 φ를 정해도 3관절 평면 팔은 보통 <b>두 가지 자세</b>로 닿습니다(팔꿈치를 위로/아래로). 아주 흐린 팔은 로봇의 <b>현재 자세</b>입니다.</p>
              <p>어느 해를 고를까? 슬라이드 5의 기준 중 하나인 <b>"가장 가까운 해"</b>(관절 이동량 합이 작은 것)를 강조 표시했습니다. 현재 자세 슬라이더를 바꾸면 선택이 바뀝니다. 실제로는 장애물을 피하려고 먼 해를 고르기도 하고, 관절 수와 0이 아닌 링크 파라미터가 많을수록 해가 늘어납니다(PUMA 560은 최대 8개).</p>`
            }
          ]
        };
      }
    });
  }

  /* ================= 3.3 algebraic ================= */
  const L3 = [1.2, 1.0];
  const ALG0 = { x: 1.0, y: 1.6, phi: 75 };
  function algScene(S, p, o) {
    o = o || {};
    fr(S, 'W', I(), { hidden: true });
    const r = ik2(L3[0], L3[1], p.x, p.y);
    const si = p.sol || 0, s = r.sols[si], other = r.sols[1 - si];
    const t3 = wrap(p.phi - s.t1 - s.t2);
    const rows = planar(L3);
    robot(S, rows, [s.t1, s.t2, t3], { base, tool: 0.35, pre: 'a', fs: 0.35, w: 0.08, jr: 0.1, jh: 0.16, noDrag: true, hide: o.hide || [], known: o.known, joint: o.joint, limits: o.limits, limZ: 0.1 });
    if (o.both && r.ok) robot(S, rows, [other.t1, other.t2, wrap(p.phi - other.t1 - other.t2)], { base, tool: 0.35, pre: 'b', noFrames: true, op: 0.3, c: o.bothWarn ? 'warn' : null, noDrag: true, w: 0.08, jr: 0.1, jh: 0.16 });
    if (o.ring) { ring(S, 'ro', L3[0] + L3[1], 'l₁+l₂'); ring(S, 'ri', Math.abs(L3[0] - L3[1]), '|l₁−l₂|'); }
    fr(S, 'G', mul(base, tr(p.x, p.y, 0), rz(p.phi)), { label: '', name: '목표 {3}', size: 0.4, ghost: true, op: 0.5 });
    fr(S, 'P0', base, { hidden: true });
    body(S, 'tgt', 'W', 'sph', [0.065], app(base, [p.x, p.y, 0.16]), { c: 'k', drag: 'tgt' });
    return { r, s, t3, other };
  }
  const kVecs = (S, s, w) => {
    vec(S, 'k1', { f: 'aF1' }, { f: 'aF1', p: [s.k1, 0, 0] }, { c: 'x', w: w || 0.8, label: 'k₁ = l₁+l₂c₂', lo: [0, 0, -0.18] });
    vec(S, 'k2', { f: 'aF1', p: [s.k1, 0, 0] }, { f: 'aF1', p: [s.k1, s.k2, 0] }, { c: 'y', w: w || 0.8, label: 'k₂ = l₂s₂', lo: [0.15, 0, 0] });
  };
  sec({
    id: 'c3-alg', group: 'lecture', ref: '슬라이드 8–11', title: '3.3 대수적 풀이 (평면 3R)',
    view: { pos: [0.7, -5.6, 0.8], target: [0.5, 0, 0.8] }, grid: [3, 0.5], scale: 0.7, fsize: 1,
    build() {
      const p = Object.assign({ sol: 0 }, ALG0);
      const r = ik2(L3[0], L3[1], p.x, p.y), s = r.sols[0], o2 = r.sols[1], t3 = wrap(p.phi - s.t1 - s.t2);
      const rr = p.x * p.x + p.y * p.y, rk = Math.hypot(s.k1, s.k2), gam = atan2d(s.k2, s.k1);
      const lim2 = { 2: [0, 180, '가정: s₂ > 0 (0° < θ₂ < 180°)'] };
      return {
        init(S) { algScene(S, p, { hide: [0, 1, 2, 3], known: [] }); },
        steps: [
          {
            t: '목표 프레임과 기구학 식',
            do(S) { algScene(S, p, { hide: [1, 2, 3], known: [] }); },
            assume: [R`목표 = 손목 프레임 {3}의 \((x, y, \phi)\) (손끝 길이는 이미 빼 둠)`, R`관절 범위 제한은 아직 두지 않음`],
            tex: R`{}^0_3T = \begin{bmatrix}c_{123} & -s_{123} & 0 & l_1c_1 + l_2c_{12}\\ s_{123} & c_{123} & 0 & l_1s_1 + l_2s_{12}\\ 0&0&1&0\\0&0&0&1\end{bmatrix} = \begin{bmatrix}c_\phi & -s_\phi & 0 & x\\ s_\phi & c_\phi & 0 & y\\ 0&0&1&0\\0&0&0&1\end{bmatrix}\ \Rightarrow\ \begin{aligned}c_\phi &= c_{123},\ s_\phi = s_{123}\\ x &= l_1c_1 + l_2c_{12}\\ y &= l_1s_1 + l_2s_{12}\end{aligned}`,
            html: R`<p>2장 예제의 \({}^0_3T\)와 목표 행렬의 원소를 하나씩 같다고 놓으면 <b>비선형 방정식 4개</b>가 나옵니다. 숫자: \(l_1 = 1.2,\ l_2 = 1.0,\ (x, y, \phi) = (${p.x}, ${p.y}, ${p.phi}^\circ)\).</p>
            <p>3D에서는 아직 값을 모르는 관절을 모두 흐리게 그렸습니다. 관절각을 하나 구할 때마다 그 관절과 그것이 돌리는 링크가 진해지고 빛납니다.</p>`
          },
          {
            t: '① 제곱해 더하면 θ₁이 사라진다',
            do(S) { algScene(S, p, { hide: [1, 2, 3], known: [2], joint: [2], both: true, ring: true }); ln(S, 'r', app(base, [0, 0, 0]), app(base, [p.x, p.y, 0]), { c: 'k', label: '√(x²+y²)', lt: 0.5, lo: [0, 0, 0.12] }); },
            assume: [{ t: R`\(-1 \le c_2 \le 1\) ⇔ \(|l_1-l_2| \le \sqrt{x^2+y^2} \le l_1+l_2\) (목표가 회색 고리 안): \(c_2 = ${f(r.c2, 4)}\)`, ok: r.ok }],
            tex: R`\begin{aligned}x^2 &= l_1^2c_1^2 + 2l_1l_2\,c_1c_{12} + l_2^2c_{12}^2,\qquad y^2 = l_1^2s_1^2 + 2l_1l_2\,s_1s_{12} + l_2^2s_{12}^2\\ x^2+y^2 &= l_1^2\underbrace{(c_1^2+s_1^2)}_{1} + l_2^2\underbrace{(c_{12}^2+s_{12}^2)}_{1} + 2l_1l_2\underbrace{(c_1c_{12}+s_1s_{12})}_{\cos(\theta_{12}-\theta_1) = c_2}\\ \Rightarrow\ c_2 &= \frac{x^2+y^2-l_1^2-l_2^2}{2l_1l_2} = \frac{${f(rr, 3)} - ${f(L3[0] ** 2, 2)} - ${f(L3[1] ** 2, 2)}}{${f(2 * L3[0] * L3[1], 2)}} = ${f(r.c2, 4)}\end{aligned}`,
            html: R`<p>가운데 항은 코사인 차 공식 \(c_1c_{12}+s_1s_{12} = \cos(\theta_{12}-\theta_1) = \cos\theta_2\)로 정리됩니다. θ₁이 완전히 사라지고 θ₂의 코사인만 남습니다. 기하학적으로는 원점–목표 거리(보라 점선)가 팔꿈치 각도만으로 정해진다는 뜻입니다.</p>
            <p>3D: 아직 θ₂의 <b>부호</b>를 모르므로 두 후보 팔(진한 것, 흐린 것)이 모두 보이고, <b>관절 2와 링크 2</b>가 빛납니다.</p>`
          },
          {
            t: '② s₂의 부호를 정해 θ₂ — 관절 2',
            do(S) { algScene(S, p, { hide: [1, 2, 3], known: [2], joint: [2], both: true, bothWarn: true, limits: lim2 }); },
            assume: [{ t: R`\(s_2 > 0\) 인 해를 택함 (관절 2 둘레의 초록 호 = 허용 범위)`, ok: true }],
            tex: R`s_2 = \pm\sqrt{1-c_2^2} = \pm ${f(Math.abs(s.s2), 4)}\ \xrightarrow{\ s_2>0\ }\ \theta_2 = \text{atan2}(s_2, c_2) = \text{atan2}(${f(s.s2, 4)},\ ${f(r.c2, 4)}) = ${deg(s.t2)}\qquad (\text{다른 해: } ${deg(o2.t2)})`,
            html: R`<p>코사인만으로는 부호를 알 수 없어 θ₂가 두 개입니다(elbow-up / elbow-down). 여기서는 \(s_2>0\)을 <b>가정</b>해 하나를 고릅니다. 붉은 반투명 팔이 버린 해(\(s_2<0\))이고, 초록 호가 가정한 θ₂ 범위입니다. 이후 계산은 모두 이 선택을 따라갑니다.</p>
            <p class="note">\(\theta_2 = \cos^{-1}c_2\)가 아니라 atan2를 쓰는 이유: 부호(사분면)를 \(s_2\)로 정확히 지정할 수 있기 때문입니다.</p>`
          },
          {
            t: '③ c₁₂, s₁₂를 풀어 k₁, k₂로 묶기',
            do(S) { algScene(S, p, { hide: [2, 3], known: [2], joint: [2] }); kVecs(S, s); },
            tex: R`\begin{aligned}x &= l_1c_1 + l_2(c_1c_2 - s_1s_2) = \underbrace{(l_1 + l_2c_2)}_{k_1}c_1 - \underbrace{(l_2s_2)}_{k_2}s_1\\ y &= l_1s_1 + l_2(s_1c_2 + c_1s_2) = k_1s_1 + k_2c_1,\qquad k_1 = ${f(s.k1, 4)},\ k_2 = ${f(s.k2, 4)}\end{aligned}`,
            html: R`<p>덧셈정리로 \(c_{12}, s_{12}\)를 풀면 x, y가 \(c_1, s_1\)에 대한 <b>일차식</b>이 됩니다. θ₂를 알았으므로 \(k_1, k_2\)는 숫자입니다.</p>
            <p>3D: {1}의 X축을 따라 \(k_1\)(빨강), Y축을 따라 \(k_2\)(초록)를 이으면 정확히 손목에 닿습니다. 즉 "{1}에서 본 손목 위치"가 \((k_1, k_2)\)입니다. 아직 θ₁은 모르므로 관절 1은 흐립니다.</p>`
          },
          {
            t: '④ γ로 바꿔 쓰면 θ₁ — 관절 1',
            do(S) {
              algScene(S, p, { hide: [2, 3], known: [1, 2], joint: [1] }); kVecs(S, s);
              arc(S, 'g', { f: 'aF1', c: [0, 0, 0], n: Z, u: X, ang: gam, r: 0.7, label: 'γ' });
              arc(S, 't1', { f: 'aF0', c: [0, 0, 0], n: Z, u: X, ang: s.t1, r: 0.45, label: 'θ₁', col: 'x' });
            },
            assume: [{ t: R`\((x, y) \ne (0, 0)\) — 원점이면 atan2(0, 0)이 정의되지 않음 (θ₁ 임의)`, ok: Math.hypot(p.x, p.y) > 1e-6 }],
            tex: R`\begin{aligned}&r = \sqrt{k_1^2+k_2^2} = ${f(rk, 4)},\quad k_1 = r\cos\gamma,\ k_2 = r\sin\gamma,\ \ \gamma = \text{atan2}(k_2, k_1) = ${deg(gam)}\\ &\frac{x}{r} = \cos\gamma\,c_1 - \sin\gamma\,s_1 = \cos(\gamma+\theta_1),\qquad \frac{y}{r} = \cos\gamma\,s_1 + \sin\gamma\,c_1 = \sin(\gamma+\theta_1)\\ &\Rightarrow\ \gamma + \theta_1 = \text{atan2}(y, x)\ \Rightarrow\ \theta_1 = ${deg(atan2d(p.y, p.x))} - ${deg(gam)} = ${deg(s.t1)}\end{aligned}`,
            html: R`<p>\((k_1, k_2)\)를 극좌표 \((r, \gamma)\)로 바꾸면 x, y가 각각 \(\cos(\gamma+\theta_1)\), \(\sin(\gamma+\theta_1)\)의 꼴이 되어 각도 하나로 묶입니다. 그림으로는: 목표의 방위각(atan2(y,x)) = {1}의 회전(θ₁, 빨간 호) + {1} 안에서 본 손목 방위각(γ, 보라 호).</p>
            <p>3D: <b>관절 1과 링크 1</b>이 정해져 진해지고 빛납니다.</p>`
          },
          {
            t: '⑤ θ₃ = φ − θ₁ − θ₂ — 관절 3',
            do(S) { algScene(S, p, { known: [1, 2, 3], joint: [3] }); },
            tex: R`c_\phi = c_{123},\ s_\phi = s_{123}\ \Rightarrow\ \theta_1+\theta_2+\theta_3 = \phi\ \Rightarrow\ \theta_3 = ${p.phi}^\circ - (${f(s.t1, 2)}^\circ) - (${f(s.t2, 2)}^\circ) = ${deg(t3)}`,
            html: R`<p>방향 식의 코사인과 사인이 모두 같으므로 각도 자체가 같습니다(360° 배수 차이 제외). <b>관절 3과 손끝</b>이 정해지며 손목 프레임이 목표 프레임과 겹칩니다.</p>`
          },
          {
            t: '직접 바꿔 보기: 목표를 끌면 계산이 따라온다',
            live: {
              params: [
                { id: 'x', label: R`\(x\)`, min: -2.3, max: 2.3, step: 0.01, val: ALG0.x, dp: 2 },
                { id: 'y', label: R`\(y\)`, min: -2.3, max: 2.3, step: 0.01, val: ALG0.y, dp: 2 },
                { id: 'phi', label: R`\(\phi\)`, min: -180, max: 180, step: 1, val: ALG0.phi, unit: '°' },
                { id: 'sol', label: '해', min: 0, max: 1, step: 1, val: 0 }
              ],
              drag: { tgt: plane() },
              apply(S, p) {
                drop(S, 'bodies'); drop(S, 'frames'); drop(S, 'vecs'); drop(S, 'arcs'); drop(S, 'lines');
                const lim = { 2: p.sol ? [-180, 0, '가정: s₂ < 0'] : [0, 180, '가정: s₂ > 0'] };
                const o = algScene(S, p, { both: true, bothWarn: true, ring: true, limits: lim });
                if (o.r.ok) kVecs(S, o.s, 0.6);
              },
              assume: p => {
                const r = ik2(L3[0], L3[1], p.x, p.y);
                return [{ t: R`\(-1 \le c_2 = ${f(r.c2, 3)} \le 1\)`, ok: r.ok }, { t: p.sol ? R`\(s_2 < 0\) 해 선택` : R`\(s_2 > 0\) 해 선택`, ok: true }, { t: R`\((x,y) \ne (0,0)\)`, ok: Math.hypot(p.x, p.y) > 0.005 }];
              },
              tex(p) {
                const r = ik2(L3[0], L3[1], p.x, p.y), s = r.sols[p.sol || 0], t3 = wrap(p.phi - s.t1 - s.t2), rr = p.x * p.x + p.y * p.y;
                if (!r.ok) return R`c_2 = \frac{${f(rr, 3)} - 1.44 - 1}{2.4} = ${f(r.c2, 3)}\ \Rightarrow\ |c_2| > 1:\ \text{해 없음 (작업공간 밖)}`;
                return R`\begin{aligned}c_2 &= \frac{${f(rr, 3)} - 1.44 - 1}{2.4} = ${f(r.c2, 4)},\ \ s_2 = ${p.sol ? '-' : '+'}${f(Math.abs(s.s2), 4)} &\Rightarrow\ \theta_2 &= \hz{${deg(s.t2)}}\\ k_1 &= ${f(s.k1, 3)},\ k_2 = ${f(s.k2, 3)},\ \gamma = ${deg(atan2d(s.k2, s.k1))} &\Rightarrow\ \theta_1 &= ${deg(atan2d(p.y, p.x))} - \gamma = \hx{${deg(s.t1)}}\\ & &\theta_3 &= ${p.phi}^\circ - \theta_1 - \theta_2 = \hy{${deg(t3)}}\end{aligned}`;
              }
            },
            html: R`<p>보라 점(목표)을 끌거나 슬라이더로 x, y, φ를 바꿔 보세요. 팔이 매 순간 위 세 줄의 계산대로 움직이고, 위의 가정 박스가 실시간으로 ✓/✗를 표시합니다. "해" 슬라이더로 \(s_2\)의 부호(가정)를 바꾸면 초록 호의 범위와 진한 팔/붉은 팔이 서로 바뀝니다. 목표를 고리 밖으로 끌면 \(|c_2| > 1\)이 되어 해가 사라집니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= 3.4 geometric ================= */
  sec({
    id: 'c3-geo', group: 'lecture', ref: '슬라이드 12–13', title: '3.4 기하학적 풀이 (삼각형으로)',
    view: { pos: [0.7, -5.6, 0.8], target: [0.5, 0, 0.8] }, grid: [3, 0.5], scale: 0.7, fsize: 1,
    build() {
      const lim2 = { 2: [-180, 0, '가정: −180° < θ₂ < 0°'] };
      const geo = (S, p, lv, joint, known) => {
        const pp = Object.assign({}, p, { sol: 1 }); // θ2 < 0 branch (slide 12)
        const o = algScene(S, pp, { hide: lv >= 4 ? [] : [1, 2, 3], known: known, joint: joint || [], limits: lv >= 2 ? lim2 : null, both: lv >= 2 && lv < 4, bothWarn: true });
        if (!o.r.ok) return o;
        const s = o.s, Wt = app(base, [p.x, p.y, 0]), Ob = app(base, O), E = app(base, [L3[0] * Math.cos(s.t1 * D), L3[0] * Math.sin(s.t1 * D), 0]);
        ln(S, 'ow', Ob, Wt, { c: 'k', dash: false, label: '√(x²+y²)', lt: 0.55, lo: [0, 0, -0.15] });
        ln(S, 'oe', Ob, E, { c: 'mute', dash: false, label: 'l₁', lt: 0.5, lo: [0, 0, 0.12] });
        ln(S, 'ew', E, Wt, { c: 'mute', dash: false, label: 'l₂', lt: 0.5, lo: [0, 0, 0.12] });
        const beta = atan2d(p.y, p.x), rr = Math.hypot(p.x, p.y);
        const psi = Math.acos(clamp((rr * rr + L3[0] ** 2 - L3[1] ** 2) / (2 * L3[0] * rr), -1, 1)) / D;
        if (lv >= 2) arc(S, 'el', { f: 'aF2', c: [0, 0, 0.12], n: Z, u: [-Math.cos(s.t2 * D), Math.sin(s.t2 * D), 0], ang: 180 + s.t2, r: 0.3, label: '180°+θ₂', col: 'z' });
        if (lv >= 3) {
          arc(S, 'be', { f: 'aF0', c: [0, 0, 0], n: Z, u: X, ang: beta, r: 0.55, label: 'β', col: 'k' });
          arc(S, 'ps', { f: 'aF0', c: [0, 0, 0], n: Z, u: [Math.cos(beta * D), Math.sin(beta * D), 0], ang: psi, r: 0.8, label: 'ψ', col: 'x' });
        }
        return Object.assign(o, { beta, psi, rr });
      };
      const p = Object.assign({}, ALG0);
      const o0 = geo(ENG.emptyState(), p, 4);
      return {
        init(S) { geo(S, p, 1, [], []); },
        steps: [
          {
            t: '공간 문제를 평면 삼각형으로',
            do(S) { drop(S, 'lines'); drop(S, 'arcs'); geo(S, p, 1, [], []); },
            assume: [R`팔꿈치가 \(\theta_2 < 0\) 쪽으로 꺾인 자세 (슬라이드 12의 그림)`],
            tex: R`\triangle(\text{원점, 팔꿈치, 손목}):\quad \text{세 변} = l_1,\ l_2,\ \sqrt{x^2+y^2}`,
            html: R`<p>기하학적 방법은 팔의 기하를 <b>평면 도형 문제</b>로 쪼갭니다. 원점, 팔꿈치, 손목이 만드는 삼각형의 세 변 \(l_1,\ l_2,\ \sqrt{x^2+y^2}\)은 모두 알고 있습니다. 관절은 아직 모두 흐립니다.</p>`
          },
          {
            t: '① 코사인 법칙 → θ₂ — 관절 2',
            do(S) { drop(S, 'lines'); drop(S, 'arcs'); geo(S, p, 2, [2], [2]); },
            assume: [{ t: R`\(-180^\circ < \theta_2 < 0^\circ\) — 이때 팔꿈치 내각이 \(180^\circ + \theta_2\) (초록 호)`, ok: o0.s.t2 < 0 }, { t: R`삼각형이 존재: \(|l_1 - l_2| \le \sqrt{x^2+y^2} \le l_1 + l_2\)`, ok: o0.r.ok }],
            tex: R`\begin{aligned}x^2+y^2 &= l_1^2 + l_2^2 - 2l_1l_2\cos(180^\circ+\theta_2) = l_1^2 + l_2^2 + 2l_1l_2\cos\theta_2\\ \cos\theta_2 &= \frac{x^2+y^2-l_1^2-l_2^2}{2l_1l_2} = ${f(o0.r.c2, 4)}\ \xrightarrow{\ -180^\circ<\theta_2<0\ }\ \theta_2 = -\cos^{-1}(${f(o0.r.c2, 4)}) = ${deg(o0.s.t2)}\end{aligned}`,
            html: R`<p>팔꿈치의 삼각형 내각(파란 호)은 \(180^\circ+\theta_2\)입니다. \(\cos(180^\circ+\theta_2) = -\cos\theta_2\)이므로 대수적 방법과 같은 식이 나옵니다. \(\cos^{-1}\)은 0°~180°만 돌려주므로, <b>가정</b> θ₂ < 0에 맞게 부호를 붙입니다. 붉은 팔이 가정에 어긋나는 다른 해입니다. <b>관절 2와 링크 2</b>가 빛납니다.</p>`
          },
          {
            t: '② β와 ψ → θ₁ — 관절 1',
            do(S) { drop(S, 'lines'); drop(S, 'arcs'); geo(S, p, 3, [1], [1, 2]); },
            assume: [{ t: R`\(0^\circ < \psi < 180^\circ\) (삼각형의 내각)`, ok: o0.psi > 0 && o0.psi < 180 }, R`\(\theta_2 < 0\)이면 \(\theta_1 = \beta + \psi\), \(\theta_2 > 0\)이면 \(\beta - \psi\)`],
            tex: R`\begin{aligned}\beta &= \text{atan2}(y, x) = ${deg(o0.beta)}\\ l_2^2 &= l_1^2 + (x^2+y^2) - 2l_1\sqrt{x^2+y^2}\cos\psi\ \Rightarrow\ \cos\psi = \frac{x^2+y^2+l_1^2-l_2^2}{2l_1\sqrt{x^2+y^2}} = ${f(Math.cos(o0.psi * D), 4)},\ \psi = ${deg(o0.psi)}\\ \theta_1 &= \beta + \psi = ${deg(o0.beta)} + ${deg(o0.psi)} = ${deg(o0.s.t1)}\end{aligned}`,
            html: R`<p>β(보라 호)는 목표점의 방위각, ψ(빨간 호)는 원점 쪽 내각으로, 변 \(l_2\)의 맞은편 각에 코사인 법칙을 한 번 더 쓰면 나옵니다. θ₂ < 0이면 링크 1이 목표 방향보다 반시계 쪽으로 꺾이므로 \(\theta_1 = \beta + \psi\)입니다. <b>관절 1과 링크 1</b>이 빛납니다.</p>`
          },
          {
            t: '③ θ₃ = φ − θ₁ − θ₂ — 관절 3',
            do(S) { drop(S, 'lines'); drop(S, 'arcs'); geo(S, p, 4, [3], [1, 2, 3]); },
            tex: R`\theta_3 = \phi - \theta_1 - \theta_2 = ${p.phi}^\circ - (${f(o0.s.t1, 2)}^\circ) - (${f(o0.s.t2, 2)}^\circ) = ${deg(wrap(p.phi - o0.s.t1 - o0.s.t2))}`,
            html: R`<p>마지막은 대수적 방법과 같습니다. 두 방법은 같은 답을 주지만, 기하학적 방법은 "왜 해가 두 개인지"(삼각형을 위/아래로 접을 수 있음)를 눈으로 보여 줍니다. <b>관절 3</b>이 빛납니다.</p>`
          },
          {
            t: '직접 바꿔 보기',
            live: {
              params: [
                { id: 'x', label: R`\(x\)`, min: -2.3, max: 2.3, step: 0.01, val: ALG0.x, dp: 2 },
                { id: 'y', label: R`\(y\)`, min: -2.3, max: 2.3, step: 0.01, val: ALG0.y, dp: 2 },
                { id: 'phi', label: R`\(\phi\)`, min: -180, max: 180, step: 1, val: ALG0.phi, unit: '°' }
              ],
              drag: { tgt: plane() },
              apply(S, p) { drop(S, 'bodies'); drop(S, 'frames'); drop(S, 'lines'); drop(S, 'arcs'); drop(S, 'vecs'); geo(S, p, 4); },
              assume: p => { const o = geo(ENG.emptyState(), p, 4); return [{ t: R`삼각형 존재 (목표가 고리 안)`, ok: o.r.ok }, { t: R`\(-180^\circ<\theta_2<0^\circ\) 해 사용`, ok: true }]; },
              tex(p) {
                const o = geo(ENG.emptyState(), p, 4);
                if (!o.r.ok) return R`\text{삼각형이 만들어지지 않음}\ (\sqrt{x^2+y^2} > l_1+l_2\ \text{또는}\ < |l_1-l_2|):\ \text{해 없음}`;
                return R`\theta_2 = \hz{${deg(o.s.t2)}},\quad \theta_1 = \beta+\psi = ${deg(o.beta)} + ${deg(o.psi)} = \hx{${deg(o.s.t1)}},\quad \theta_3 = \hy{${deg(wrap(p.phi - o.s.t1 - o.s.t2))}}`;
              }
            },
            html: R`<p>목표를 끌면 삼각형이 따라 변하고 β, ψ, 팔꿈치 각이 다시 계산됩니다. 삼각형이 일직선이 되는 순간이 작업공간의 경계입니다(ψ = 0 또는 θ₂ = 0, −180°).</p>`
          }
        ]
      };
    }
  });

  /* ================= Pieper robot (6R, last three axes intersect) ================= */
  const PI = { a2: 0.45, a3: 0.25, d4: 0.12, H: 0.35 };
  const pRows = [{ al: 0, a: 0, d: 0, th: 0, type: 'R' }, { al: -90, a: 0, d: 0, th: 0, type: 'R' }, { al: 0, a: PI.a2, d: 0, th: 0, type: 'R' }];
  const pBase = tr(0, 0, PI.H);
  function pFK(q) { // in base frame {0}
    const T3 = chainT(pRows, q.slice(0, 3)).Ts[3];
    const Tw = mul(T3, tr(PI.a3, -PI.d4, 0));
    const T4 = mul(Tw, rz(q[3])), T5 = mul(T4, ry(q[4])), T6 = mul(T5, rz(q[5]));
    return { T3, Tw, T4, T5, T6 };
  }
  function pieperDraw(S, q, o) {
    o = o || {};
    const P = o.pre || '';
    fr(S, 'W', I(), { hidden: true });
    const up = o.upto, gl = o.glow || [], jn = o.joint || [];
    const ex = (m, x) => {
      const r = Object.assign({}, x);
      if (up != null && m > up) { r.op = 0.1; r.drag = null; }
      if (gl.indexOf(m) >= 0 || jn.indexOf(m) >= 0) r.glow = true;
      if (o.known && o.known.indexOf(m) < 0) { r.op = 0.13; r.drag = null; r.glow = false; }
      if (o.c) r.c = o.c;
      if (o.op != null) r.op = o.op;
      if (o.noDrag) r.drag = null;
      return r;
    };
    const lim = o.limits || {};
    robot(S, pRows, q.slice(0, 3), { base: pBase, pre: P, pedestal: o.noPed ? null : [0.06, PI.H], fs: 0.12, w: 0.035, jr: 0.045, jh: 0.1, upto: up != null ? Math.min(up, 3) : null, glow: gl, joint: jn, known: o.known, c: o.c, op: o.op, noDrag: o.noDrag, noFrames: o.noFrames, hide: o.hide, frameOpt: o.frameOpt, limits: { 1: lim[1], 2: lim[2], 3: lim[3] }, limR: 0.13 });
    const k = pFK(q), B = T => mul(pBase, T);
    fr(S, P + 'F4', B(k.T4), { label: '4', size: 0.1, hidden: o.noFrames || (up != null && up < 4) || (o.hide && o.hide.indexOf(4) >= 0) });
    fr(S, P + 'A5', B(mul(k.T4, rx(-90))), { hidden: true });
    fr(S, P + 'F5', B(k.T5), { label: '5', size: 0.1, hidden: true });
    fr(S, P + 'F6', B(k.T6), { label: '6', size: 0.14, hidden: o.noFrames || (up != null && up < 6) || (o.hide && o.hide.indexOf(6) >= 0) });
    body(S, P + 'l3a', P + 'F3', 'box', [PI.a3, 0.035, 0.035], [PI.a3 / 2, 0, 0], ex(3, { c: 'j3', drag: 'q3' }));
    body(S, P + 'l3b', P + 'F3', 'box', [0.035, PI.d4, 0.035], [PI.a3, -PI.d4 / 2, 0], ex(3, { c: 'j3', drag: 'q3' }));
    body(S, P + 'w4', P + 'F4', 'cyl', [0.04, 0.09], [0, 0, -0.07], ex(4, { c: 'j4', drag: 'q4' }));
    body(S, P + 'w5', P + 'A5', 'cyl', [0.035, 0.1], [0, 0, 0], ex(5, { c: 'j5', drag: 'q5' }));
    body(S, P + 'w6', P + 'F6', 'cyl', [0.04, 0.025], [0, 0, 0.06], ex(6, { c: 'j6', drag: 'q6' }));
    if (o.op == null) {
      pt(S, P + 'jl4', { f: P + 'F4', p: [0, 0, -0.14] }, 'J4', { c: 'j4', r: 0.001, op: ex(4, {}).op });
      pt(S, P + 'jl5', { f: P + 'A5', p: [0, 0, 0.09] }, 'J5', { c: 'j5', r: 0.001, op: ex(5, {}).op });
      pt(S, P + 'jl6', { f: P + 'F6', p: [0, 0, 0.12] }, 'J6', { c: 'j6', r: 0.001, op: ex(6, {}).op });
    }
    body(S, P + 'tl', P + 'F6', 'box', [0.06, 0.012, 0.012], [0.03, 0, 0.075], ex(6, { c: 'ink' }));
    if (lim[5]) {
      const L5 = lim[5], th = q[4], inR = th > L5[0] && th < L5[1];
      arc(S, P + 'lim5', { f: P + 'A5', c: [0, 0, 0.07], n: [0, 0, 1], u: [Math.cos(L5[0] * D), Math.sin(L5[0] * D), 0], ang: L5[1] - L5[0], r: 0.11, noHead: true, col: inR ? 'ok' : 'warn', label: L5[2] || '' });
    }
    return k;
  }
  const pDrag = Object.assign(dragSpec(pRows), { q4: { frame: 'F4', type: 'rot' }, q5: { frame: 'A5', type: 'rot' }, q6: { frame: 'F6', type: 'rot' } });
  function pIK(Pt, Rt) {
    const [x, y, z] = Pt, rr = x * x + y * y + z * z, { a2, a3, d4 } = PI;
    const A = 2 * a2 * a3, Bq = 2 * a2 * d4, C = rr - a2 * a2 - a3 * a3 - d4 * d4;
    const disc = A * A + Bq * Bq - C * C;
    const t1 = atan2d(y, x), g1 = Math.hypot(x, y);
    const cands = [1, -1].map(sg => {
      const t3 = wrap(2 * atan2d(Bq + sg * Math.sqrt(Math.max(0, disc)), A + C));
      const f1 = a2 + a3 * Math.cos(t3 * D) + d4 * Math.sin(t3 * D), f2 = a3 * Math.sin(t3 * D) - d4 * Math.cos(t3 * D);
      const t2 = wrap(atan2d(-z, g1) - atan2d(f2, f1));
      const R03 = R3(mul(rz(t1), rx(-90), rz(t2 + t3)));
      const R36 = mulR(RT(R03), Rt);
      const b = atan2d(Math.hypot(R36[2][0], R36[2][1]), R36[2][2]), al = atan2d(R36[1][2], R36[0][2]), ga = atan2d(R36[2][1], -R36[2][0]);
      return { t1, t2, t3, t4: al, t5: b, t6: ga, f1, f2, R03, R36, sg };
    });
    return { rr, A, B: Bq, C, disc, g1, cands };
  }
  function mulR(a, b) { return [0, 1, 2].map(i => [0, 1, 2].map(j => a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j])); }
  const pView = { pos: [1.15, -1.25, 1.0], target: [0.2, 0.05, 0.38] };

  /* ================= 3.5 Pieper (detailed, view-guided) ================= */
  sec({
    id: 'c3-pieper', group: 'lecture', ref: '슬라이드 14–21', title: '3.5 Pieper 해법: 손목 축 3개가 한 점에서',
    view: pView, grid: [0.8, 0.1], scale: 0.2, fsize: 1,
    build() {
      const q = [25, 40, -55, 30, 60, -20];
      const { a2, a3, d4 } = PI;
      const k = pFK(q), Pw = p3(k.Tw), r2 = Pw[0] ** 2 + Pw[1] ** 2 + Pw[2] ** 2;
      const ik0 = pIK(Pw, R3(k.T6)), alt = ik0.cands.find(c => Math.abs(c.t3 - q[2]) > 1) || ik0.cands[0];
      const good = ik0.cands.find(c => Math.abs(c.t3 - q[2]) < 1) || ik0.cands[0];
      const qAlt = [alt.t1, alt.t2, alt.t3, alt.t4, alt.t5, alt.t6];
      const Wc = app(pBase, Pw);
      const fOf = t3 => [a2 + a3 * Math.cos(t3 * D) + d4 * Math.sin(t3 * D), a3 * Math.sin(t3 * D) - d4 * Math.cos(t3 * D)];
      const gOf = (t2, ff) => [Math.cos(t2 * D) * ff[0] - Math.sin(t2 * D) * ff[1], -(Math.sin(t2 * D) * ff[0] + Math.cos(t2 * D) * ff[1])];
      const [f1, f2] = fOf(q[2]), [g1, g3] = gOf(q[1], [f1, f2]);
      const phiF = atan2d(f2, f1), phiG = atan2d(-g3, g1);
      /* camera presets (world) */
      const Ts = chainT(pRows, q.slice(0, 3), pBase).Ts;
      const z2 = col(Ts[2], 2), o1 = p3(Ts[1]);
      const mid = scl(add(o1, Wc), 0.5);
      const vPlane = { pos: add(add(mid, scl(z2, 1.55)), [0, 0, 0.12]), target: mid };
      const topT = [Wc[0] / 2, Wc[1] / 2, PI.H];
      const vTop = { pos: add(topT, [-0.02, -0.2, 1.5]), target: topT };
      const vWrist = { pos: add(add(Wc, scl(z2, 0.62)), add(scl([Math.cos(q[0] * D), Math.sin(q[0] * D), 0], 0.3), [0, 0, 0.32])), target: Wc };
      /* drawing helpers */
      const clr = S => { ['vecs', 'arcs', 'lines', 'pts', 'bodies'].forEach(kd => drop(S, kd)); delete S.frames.Wc; };
      const wc = (S, lab) => pt(S, 'wc', Wc, lab == null ? '손목 중심' : lab, { c: 'k', r: 0.35 });
      const axes = S => {
        ln(S, 'x4', { f: 'F4', p: [0, 0, -0.22] }, { f: 'F4', p: [0, 0, 0.18] }, { c: 'k', dash: true });
        ln(S, 'x5', { f: 'A5', p: [0, 0, -0.16] }, { f: 'A5', p: [0, 0, 0.16] }, { c: 'k', dash: true });
        ln(S, 'x6', { f: 'F6', p: [0, 0, -0.16] }, { f: 'F6', p: [0, 0, 0.22] }, { c: 'k', dash: true });
      };
      const th1 = (S, t) => arc(S, 'a1', { f: 'K1', c: [0, 0, 0.02], n: Z, u: X, ang: t, r: 0.16, label: 'θ₁ = ' + f(t, 1) + '°', col: 'j1', grow: true });
      const th2 = (S, t) => arc(S, 'a2', { f: 'K2', n: Z, u: X, ang: t, r: 0.12, label: 'θ₂ = ' + f(t, 1) + '°', col: 'j2', grow: true });
      const th3 = (S, t, id, lab) => arc(S, id || 'a3', { f: 'K3', n: Z, u: X, ang: t, r: 0.09, label: lab != null ? lab : 'θ₃ = ' + f(t, 1) + '°', col: 'j3', grow: true });
      const x1ref = S => ln(S, 'x1r', { f: 'K2' }, { f: 'K2', p: [0.62, 0, 0] }, { c: 'mute', dash: true, label: 'X̂₁ (수평)', lt: 1 });
      const x2ref = (S, pre) => ln(S, (pre || '') + 'x2r', { f: (pre || '') + 'F2' }, { f: (pre || '') + 'F2', p: [0.62, 0, 0] }, { c: 'mute', dash: true, label: pre ? '' : 'X̂₂ 연장', lt: 1 });
      const fVecs = (S, ff, o) => {
        o = o || {};
        vec(S, 'f1', { f: 'F2' }, { f: 'F2', p: [ff[0], 0, 0] }, { c: 'x', w: 1.1, label: o.noLab ? '' : 'f₁ = ' + f(ff[0], 3), lt: 0.55, lo: [0, 0, 0.05] });
        vec(S, 'f2', { f: 'F2', p: [ff[0], 0, 0] }, { f: 'F2', p: [ff[0], ff[1], 0] }, { c: 'y', w: 1.1, label: o.noLab ? '' : 'f₂ = ' + f(ff[1], 3), lt: 0.5, lo: [0.05, 0, 0] });
      };
      const gVecs = (S, gg) => {
        vec(S, 'g1', { f: 'F1' }, { f: 'F1', p: [gg[0], 0, 0] }, { c: 'x', w: 1.1, label: 'g₁ = ' + f(gg[0], 3), lt: 0.55, lo: [0, 0, 0.05] });
        vec(S, 'g3', { f: 'F1', p: [gg[0], 0, 0] }, { f: 'F1', p: [gg[0], 0, gg[1]] }, { c: 'z', w: 1.1, label: 'g₃ = ' + f(gg[1], 3), lt: 0.5, lo: [0.06, 0, 0] });
      };
      const whole = (S, lab) => vec(S, 'fw', { f: 'F2' }, Wc, { c: 'ink', w: 0.8, label: lab || '', lt: 0.45 });
      const GH = 0.32;
      const armDim = (S, o) => pieperDraw(S, q, Object.assign({ hide: [0, 1, 4, 6], op: GH, frameOpt: FC }, o || {}));
      const UL = (...it) => '<ul class="ls">' + it.map(x => '<li>' + x + '</li>').join('') + '</ul>';
      const OL = (...it) => '<ol class="ls">' + it.map(x => '<li>' + x + '</li>').join('') + '</ol>';
      const P3s = `(${a3}, −${d4}, 0)`;
      /* viewport card: the same point read in four frames */
      const card = (on, o) => {
        o = o || {};
        const ff = o.f || [f1, f2], gg = o.g || [g1, g3], w = o.w || Pw;
        const r = (key, sym, val, note) => `<div class="vr${on === key ? ' on' : ''}"><b>${sym}</b> = ${val} <span class="vn">${note}</span></div>`;
        return `<div class="vh">손목 중심 = 같은 점, 다른 축으로 읽은 값</div>`
          + r('3', '³P', P3s, '· {3}의 축 · 상수')
          + r('f', 'f', `(${f(ff[0], 3)}, ${f(ff[1], 3)}, 0)`, '· {2}의 축 · θ₃만')
          + r('g', 'g', `(${f(gg[0], 3)}, 0, ${f(gg[1], 3)})`, '· {1}의 축 · θ₂, θ₃')
          + r('0', '⁰P', `(${f(w[0], 3)}, ${f(w[1], 3)}, ${f(w[2], 3)})`, '· {0}의 축 · θ₁, θ₂, θ₃')
          + (o.extra || '');
      };
      /* {1} and {2} share the shoulder origin → colour them differently (all axes drawn in + direction) */
      const FC = { 1: { cols: ['j1', 'j1', 'j1'], size: 0.2 }, 2: { cols: ['j2', 'j2', 'j2'], size: 0.2 } };
      const both12 = (S, o) => pieperDraw(S, q, Object.assign({ hide: [0, 4, 6], op: GH, frameOpt: FC }, o || {}));
      const readLab = S => {
        const m6 = add(scl(o1, 0.45), scl(Wc, 0.55));
        pt(S, 'rf', add(m6, [0, 0, 0.11]), `{2}(분홍)로 읽으면 (f₁, f₂) = (${f(f1, 3)}, ${f(f2, 3)})`, { c: 'j2', r: 0.001 });
        pt(S, 'rg', add(m6, [0, 0, 0.07]), `{1}(청록)로 읽으면 (g₁, g₃) = (${f(g1, 3)}, ${f(g3, 3)})`, { c: 'j1', r: 0.001 });
      };
      /* angle arcs about joint-2 axis Ẑ₂ (+ = the way θ₂ turns: here downward) */
      const angArcs = (S, t2, full) => {
        ln(S, 'x1r', { f: 'K2' }, { f: 'K2', p: [0.6, 0, 0] }, { c: 'j1', dash: true, label: '', lt: 1 });
        ln(S, 'x2r', { f: 'F2' }, { f: 'F2', p: [0.6, 0, 0] }, { c: 'j2', dash: true, label: '', lt: 1 });
        arc(S, 'a2', { f: 'K2', n: Z, u: X, ang: t2, r: 0.17, label: 'θ₂' + (full ? ' = ' + f(t2, 1) + '°' : ''), col: 'j2', grow: true });
        arc(S, 'pf', { f: 'F2', n: Z, u: X, ang: phiF, r: 0.27, label: 'φ_f' + (full ? ' = ' + f(phiF, 1) + '°' : ''), col: 'pt', grow: true });
        arc(S, 'pg', { f: 'K2', n: Z, u: X, ang: phiG, r: 0.38, label: 'θ₂ + φ_f' + (full ? ' = ' + f(phiG, 1) + '°' : ''), col: 'ink', grow: true });
      };
      const Wframe = (S, T) => fr(S, 'Wc', mul(pBase, T || k.Tw), { label: 'w', sub: '3', name: '{3} 방향', size: 0.12 });
      return {
        init(S) { pieperDraw(S, q, { hide: [1, 2, 3, 4] }); },
        steps: [
          {
            t: '손목 중심: 축 4, 5, 6이 한 점에서 만난다',
            do(S) { wc(S); axes(S); },
            assume: [R`Pieper의 조건: 연속한 세 관절 축(여기서는 4, 5, 6)이 한 점에서 만난다 — 보라 점선 세 개가 보라 점에서 교차`, R`손목 방향은 과제 정의대로 \({}^3_6R = R_Z(\theta_4)R_Y(\theta_5)R_Z(\theta_6)\) (축 4 ∥ \(\Zh_3\))로 모델링. 엄밀한 D-H(\(\alpha_3 = 90^\circ\))라면 축 4는 팔뚝 방향이지만, 위치 식(f, g, r)과 θ₁~θ₃ 풀이는 똑같습니다.`],
            tex: R`{}^0P_{4ORG} = {}^0P_{5ORG} = {}^0P_{6ORG}\quad(\text{wrist center})`,
            html: R`<p>6자유도 일반 로봇은 닫힌 해(closed-form solution)가 없습니다.</p>
              <p>하지만 <b>연속한 세 축이 한 점에서 만나면</b> 풀 수 있습니다(Pieper).</p>
              ${UL('보라 점선 세 개: 축 4, 5, 6', '보라 점: 세 축이 만나는 손목 중심(wrist center)', '대부분의 산업용 로봇 손목이 이 구조입니다')}
              <p class="note">이 로봇: \(a_2 = 0.45,\ a_3 = 0.25,\ d_4 = 0.12\) m (과제 2 문제 3과 같은 구조, 숫자만 다름).</p>`
          },
          {
            t: '팔(arm)과 손목(wrist)을 분리',
            do(S) { clr(S); pieperDraw(S, q, { glow: [1, 2, 3], hide: [1, 2, 4] }); wc(S); axes(S); },
            tex: R`{}^0_6T = \begin{bmatrix}\underbrace{{}^0_3R(\theta_1,\theta_2,\theta_3)}_{\text{arm}}\ \underbrace{{}^3_6R(\theta_4,\theta_5,\theta_6)}_{\text{wrist}} & {}^0P_{w}(\theta_1,\theta_2,\theta_3)\\ 0 & 1\end{bmatrix}`,
            html: R`<p>손목 관절 4, 5, 6을 아무리 돌려도 손목 중심은 움직이지 않습니다.<br>세 축이 모두 그 점을 지나기 때문입니다.</p>
              <p>그래서 문제를 둘로 나눕니다.</p>
              ${OL('<b>위치</b>: 앞 세 관절(θ₁, θ₂, θ₃, 빛나는 부분)만으로 손목 중심을 목표에 놓기', '<b>방향</b>: 남은 회전을 손목 세 관절(θ₄, θ₅, θ₆)로 맞추기')}
              <p>6변수 문제가 3변수 문제 두 개가 됩니다.</p>`
          },
          {
            t: '① 같은 점을 네 프레임에서 읽는다: ³P → f → g → ⁰P',
            view: vPlane,
            do(S) { clr(S); both12(S); wc(S); vec(S, 'p3', { f: 'F3' }, Wc, { c: 'j3', w: 0.8, label: '³P', lt: 0.5 }); whole(S, 'f와 g가 가리키는 화살표'); },
            vpInfo: card(''),
            tex: R`{}^0P_w = {}^0_1T\ \underbrace{{}^1_2T\ \underbrace{{}^2_3T\ \underbrace{{}^3P_w}_{\{3\}\text{에서}}}_{f:\ \{2\}\text{에서}}}_{g:\ \{1\}\text{에서}}`,
            html: R`<p>손목 중심이라는 <b>한 점</b>의 좌표를 네 프레임의 축으로 차례로 읽습니다.</p>
              ${UL('\\({}^3P\\): {3}(팔뚝)의 축으로 읽은 값', '\\(f\\): {2}(위팔)의 축으로 읽은 값', '\\(g\\): {1}(θ₁만 돈 프레임)의 축으로 읽은 값', '\\((x, y, z)\\): {0}(바닥)의 축으로 읽은 값')}
              <p>변환행렬을 하나 곱할 때마다 "어느 프레임의 축으로 읽느냐"가 한 단계씩 바뀝니다. 3D 왼쪽 아래 카드에 네 값을 함께 표시했습니다.</p>
              <p class="note">시점이 팔이 움직이는 평면(관절 2·3 축 방향)으로 바뀌었습니다.<br>이 로봇은 \(a_1 = d_2 = 0\)이라 {1}과 {2}의 원점이 어깨의 같은 점입니다.<br>그래서 <b>f와 g는 같은 검은 화살표</b>를 다른 축으로 읽은 값입니다.</p>`
          },
          {
            t: '② ³P: {3}에서 본 손목 중심',
            view: vPlane,
            do(S) { clr(S); armDim(S, { glow: [3] }); wc(S); vec(S, 'pa3', { f: 'F3' }, { f: 'F3', p: [a3, 0, 0] }, { c: 'x', w: 1.1, label: 'a₃ = ' + a3, lt: 0.5, lo: [0, 0, 0.05] }); vec(S, 'pd4', { f: 'F3', p: [a3, 0, 0] }, { f: 'F3', p: [a3, -d4, 0] }, { c: 'y', w: 1.1, label: '−d₄ = −' + d4, lt: 0.5, lo: [0.05, 0, 0] }); },
            vpInfo: card('3'),
            tex: R`{}^3P_w = \begin{bmatrix}a_3\\ -d_4\\ 0\end{bmatrix} = \begin{bmatrix}${a3}\\ -${d4}\\ 0\end{bmatrix}\quad(\text{관절각과 무관한 상수})`,
            html: R`<p>{3}은 팔꿈치(관절 3)에 붙어 팔뚝과 함께 도는 프레임입니다. 그 축으로 읽으면 손목 중심은</p>
              ${UL('\\(\\Xh_3\\) 방향으로 \\(a_3\\) (빨강)', '\\(\\Yh_3\\) 방향으로 \\(-d_4\\) (초록)')}
              <p>에 있습니다. 팔뚝에 고정된 점이므로 관절각이 바뀌어도 이 좌표는 그대로입니다.</p>`
          },
          {
            t: '③ f = ²₃T ³P: {2}에서 본 손목 중심 — θ₃만 들어감',
            view: vPlane,
            do(S) {
              clr(S); armDim(S, { glow: [3] }); wc(S, '');
              vec(S, 'pa2', { f: 'F2' }, { f: 'F2', p: [a2, 0, 0] }, { c: 'j2', w: 0.7, label: 'a₂', lt: 0.5, lo: [0, 0, -0.05] });
              vec(S, 'pa3', { f: 'F3' }, { f: 'F3', p: [a3, 0, 0] }, { c: 'j3', w: 0.7, label: '', lt: 0.5 }); vec(S, 'pd4', { f: 'F3', p: [a3, 0, 0] }, { f: 'F3', p: [a3, -d4, 0] }, { c: 'j3', w: 0.7, label: '', lt: 0.5 });
              x2ref(S); th3(S, q[2]); fVecs(S, [f1, f2]);
            },
            vpInfo: card('f'),
            tex: R`\begin{aligned}\begin{bmatrix}f_1\\f_2\\f_3\end{bmatrix} &= \underbrace{\begin{bmatrix}a_2\\0\\0\end{bmatrix}}_{\{3\}\text{ 원점}} + R_Z(\theta_3)\begin{bmatrix}a_3\\-d_4\\0\end{bmatrix} = \begin{bmatrix}a_2 + a_3c_3 + d_4s_3\\ a_3s_3 - d_4c_3\\ 0\end{bmatrix}\\ &= \begin{bmatrix}${a2} + ${a3}c_3 + ${d4}s_3\\ ${a3}s_3 - ${d4}c_3\\ 0\end{bmatrix} \overset{\theta_3 = ${q[2]}^\circ}{=} \begin{bmatrix}${f(f1, 4)}\\ ${f(f2, 4)}\\ 0\end{bmatrix}\end{aligned}`,
            html: R`<p>{2}는 위팔에 붙은 프레임입니다. ³P를 {2}의 축으로 옮겨 읽는 과정은 두 단계입니다.</p>
              ${OL('{3}의 원점(팔꿈치)은 {2}의 \\(\\Xh_2\\) 방향으로 \\(a_2\\) 떨어져 있음 → \\((a_2, 0, 0)\\)', '{3}은 {2}에 대해 θ₃만큼 돌아가 있음 → ³P를 \\(R_Z(\\theta_3)\\)로 돌려서 더함')}
              <p>결과를 {2}의 축으로 나눈 것이 f입니다.</p>
              ${UL('<b>f₁</b> (빨강): \\(\\Xh_2\\) 방향, 위팔을 따라', '<b>f₂</b> (초록): \\(\\Yh_2\\) 방향, 위팔에 수직')}
              <p>노란 호가 θ₃(점선 X̂₂ 연장선 → 팔뚝 X̂₃)입니다. f 식에 들어 있는 관절각은 <b>θ₃뿐</b>입니다.</p>`
          },
          {
            t: '③-2 확인: θ₂를 돌려도 f는 그대로',
            view: vPlane,
            live: {
              params: [{ id: 'q2', label: R`\(\theta_2\)`, min: -60, max: 120, step: 1, val: q[1], unit: '°' }, { id: 'q3', label: R`\(\theta_3\)`, min: -150, max: 60, step: 1, val: q[2], unit: '°' }],
              drag: { q2: pDrag.q2, q3: pDrag.q3 },
              apply(S, p) {
                clr(S);
                const qq = q.slice(); qq[1] = p.q2; qq[2] = p.q3;
                pieperDraw(S, qq, { hide: [0, 1, 4, 6], op: GH, frameOpt: FC });
                const ff = fOf(p.q3); fVecs(S, ff); th3(S, p.q3); x2ref(S);
                pt(S, 'wc', app(pBase, p3(pFK(qq).Tw)), '', { c: 'k', r: 0.35 });
              },
              vpInfo(p) { const qq = q.slice(); qq[1] = p.q2; qq[2] = p.q3; const ff = fOf(p.q3); return card('f', { f: ff, g: gOf(p.q2, ff), w: p3(pFK(qq).Tw) }); },
              tex(p) {
                const qq = q.slice(); qq[1] = p.q2; qq[2] = p.q3; const ff = fOf(p.q3), w = p3(pFK(qq).Tw), A = act(p);
                return R`\begin{aligned}f &= (${hlIf(A === 3, f(ff[0], 4))},\ ${hlIf(A === 3, f(ff[1], 4))})\quad(\theta_3\text{만 바꾸면 변함})\\ {}^0P_w &= (${hlIf(A === 2 || A === 3, f(w[0], 3))},\ ${hlIf(A === 2 || A === 3, f(w[1], 3))},\ ${hlIf(A === 2 || A === 3, f(w[2], 3))})\quad(\text{둘 다에 따라 변함})\end{aligned}`;
              }
            },
            html: R`<p>슬라이더를 움직이거나 3D에서 관절을 끌어 보세요. 왼쪽 아래 카드의 숫자를 보면 됩니다.</p>
              ${UL('<b>θ₂를 바꾸면</b>: 팔 전체가 돌지만 f₁, f₂는 그대로 ({2}도 같이 돌기 때문). g와 ⁰P는 바뀝니다.', '<b>θ₃을 바꾸면</b>: f가 바뀝니다.')}
              <p>그래서 f를 "θ₃만의 함수"라고 합니다.</p>`
          },
          {
            t: '④ g = ¹₂T f: {1}에서 본 손목 중심 — θ₂ 추가',
            view: vPlane,
            do(S) { clr(S); both12(S, { glow: [2] }); wc(S, ''); x1ref(S); th2(S, q[1]); gVecs(S, [g1, g3]); whole(S, ''); },
            vpInfo: card('g'),
            assume: [R`\(\alpha_1 = -90^\circ,\ a_1 = 0,\ d_2 = 0,\ f_3 = 0\) (이 로봇의 값)`],
            tex: R`\begin{aligned}\begin{bmatrix}g_1\\g_2\\g_3\end{bmatrix} &= \begin{bmatrix}c_2 & -s_2 & 0\\ s_2c\alpha_1 & c_2c\alpha_1 & -s\alpha_1\\ s_2s\alpha_1 & c_2s\alpha_1 & c\alpha_1\end{bmatrix}\begin{bmatrix}f_1\\f_2\\f_3\end{bmatrix} + \begin{bmatrix}a_1\\ -s\alpha_1d_2\\ c\alpha_1d_2\end{bmatrix}\\ &\overset{c\alpha_1 = 0,\ s\alpha_1 = -1}{=} \begin{bmatrix}c_2f_1 - s_2f_2\\ f_3\\ -(s_2f_1 + c_2f_2)\end{bmatrix} = \begin{bmatrix}${f(g1, 4)}\\ 0\\ ${f(g3, 4)}\end{bmatrix}\end{aligned}`,
            html: R`<p>같은 검은 화살표를 이번에는 {1}의 축으로 읽습니다.</p>
              <p>{1}은 θ₁만큼만 돌아간 프레임이라 \(\Xh_1\)은 <b>수평</b>(점선), \(\Zh_1\)은 <b>수직</b>입니다. 그래서</p>
              ${UL('<b>g₁</b> (빨강) = 손목 중심까지의 수평 거리', '<b>g₂</b> = f₃ = 0: 팔 전체가 이 수직 평면 안에 있어 평면 밖 성분이 없음', '<b>g₃</b> (파랑) = 손목 중심의 높이 (= z)')}
              <p>분홍 호가 θ₂(수평 X̂₁ → 위팔 X̂₂)입니다. g에는 θ₂와 θ₃이 들어 있습니다.</p>`
          },
          {
            t: '④-2 핵심: 화살표의 각도 = θ₂ + φ_f',
            view: vPlane,
            do(S) { clr(S); both12(S, { glow: [2] }); wc(S, ''); whole(S, ''); angArcs(S, q[1], true); readLab(S); },
            vpInfo: card('g', { extra: `<div class="vr" style="margin-top:4px"><b>θ₂</b> = ${f(q[1], 2)}° <span class="vn">· {1}의 X̂₁ → 위팔 X̂₂</span></div><div class="vr"><b>φ_f</b> = atan2(f₂, f₁) = ${f(phiF, 2)}° <span class="vn">· 위팔 → 화살표</span></div><div class="vr on"><b>θ₂ + φ_f</b> = atan2(−g₃, g₁) = ${f(phiG, 2)}° <span class="vn">· 수평 → 화살표</span></div>` }),
            tex: R`\begin{aligned}&\underbrace{\theta_2}_{\text{수평}\to\text{위팔}} + \underbrace{\varphi_f}_{\text{위팔}\to\text{화살표}} = \underbrace{\text{atan2}(-g_3,\ g_1)}_{\text{수평}\to\text{화살표}}\qquad (\varphi_f = \text{atan2}(f_2, f_1))\\ &\Rightarrow\ \theta_2 = \text{atan2}(-z,\ g_1) - \text{atan2}(f_2,\ f_1) = ${f(phiG, 2)}^\circ - (${f(phiF, 2)}^\circ) = ${deg(phiG - phiF)}\\[4pt] &\text{같은 내용을 식으로: } g_1 = c_2f_1 - s_2f_2,\quad -g_3 = s_2f_1 + c_2f_2\ \Leftrightarrow\ \begin{bmatrix}g_1\\ -g_3\end{bmatrix} = \begin{bmatrix}c_2 & -s_2\\ s_2 & c_2\end{bmatrix}\begin{bmatrix}f_1\\ f_2\end{bmatrix}\end{aligned}`,
            html: R`<p><b>한 줄 요약</b>: 검은 화살표가 수평선에서 돌아간 각 = 위팔이 돌아간 각 θ₂ + 위팔에서 화살표까지의 각 φ_f.</p>
              ${OL(
                '<b>프레임 두 개</b> (원점이 같아서 색으로 구분, 축은 모두 + 방향)<br>· {1} 청록: X̂₁ 수평, Ẑ₁ 위쪽 — θ₂와 무관<br>· {2} 분홍: X̂₂ 위팔 방향, Ŷ₂ 위팔에 수직 — θ₂만큼 돌아감',
                '<b>같은 화살표, 다른 숫자</b> (화살표 위 글자)<br>· {2}로 읽으면 (f₁, f₂) = (' + f(f1, 3) + ', ' + f(f2, 3) + ')<br>· {1}로 읽으면 (g₁, g₃) = (' + f(g1, 3) + ', ' + f(g3, 3) + '), g₃ = 높이 z',
                '<b>각도 세 개</b> (모두 관절 2 축 Ẑ₂ 기준, 오른손 법칙)<br>· 분홍 호 θ₂ = ' + f(q[1], 1) + '°: 수평 점선 → 위팔 점선<br>· 주황 호 φ_f = atan2(f₂, f₁) = ' + f(phiF, 1) + '°: 위팔 → 화살표<br>· 검정 호 θ₂ + φ_f = ' + f(phiG, 1) + '°: 수평 → 화살표',
                '<b>검정 호를 g로 쓰기</b><br>· 화살표의 수평 성분 = g₁, 위쪽 성분 = g₃<br>· 이 로봇의 Ẑ₂ 방향 때문에 + 각도는 <b>아래로</b> 도는 쪽 (θ₂ = 40°이면 위팔이 수평보다 40° 아래)<br>· 그래서 아래로 잰 각 = atan2(아래 성분, 수평 성분) = atan2(−g₃, g₁)',
                '<b>정리</b><br>θ₂ = atan2(−z, g₁) − atan2(f₂, f₁) = ' + f(phiG, 2) + '° − (' + f(phiF, 2) + '°) = ' + f(phiG - phiF, 2) + '°'
              )}
              <p class="note"><b>−z가 붙는 이유</b><br>축을 뒤집은 것이 아닙니다. Ẑ₁은 위쪽 그대로입니다.<br>관절 2의 + 회전이 위팔을 아래로 내리는 방향이라서, 각도를 "아래쪽이 +"로 재기 때문입니다.<br>φ_f = ${f(phiF, 1)}°가 음수인 것도 같은 이유: 화살표가 위팔보다 <b>위쪽</b>에 있다는 뜻입니다.</p>`
          },
          {
            t: '⑤ ⁰P = ⁰₁T g: 위에서 본 θ₁',
            view: vTop,
            do(S) {
              clr(S); pieperDraw(S, q, { glow: [1], hide: [2, 3, 4], op: 0.45 }); wc(S, '');
              th1(S, q[0]);
              ln(S, 'gh', { f: 'K1' }, [Wc[0], Wc[1], PI.H], { c: 'x', dash: false, label: 'g₁ = ' + f(g1, 3), lt: 0.5 });
              ln(S, 'gx', [Wc[0], 0, PI.H], [Wc[0], Wc[1], PI.H], { c: 'mute', dash: true, label: 'y = s₁g₁', lt: 0.5 });
              ln(S, 'gy', { f: 'K1' }, [Wc[0], 0, PI.H], { c: 'mute', dash: true, label: 'x = c₁g₁', lt: 0.5 });
              ln(S, 'gv', [Wc[0], Wc[1], PI.H], Wc, { c: 'z', dash: true, label: 'z = g₃', lt: 0.5 });
            },
            vpInfo: card('0'),
            tex: R`{}^0P_w = R_Z(\theta_1)\,g = \begin{bmatrix}c_1g_1 - s_1g_2\\ s_1g_1 + c_1g_2\\ g_3\end{bmatrix} = \begin{bmatrix}c_1g_1\\ s_1g_1\\ g_3\end{bmatrix} = \begin{bmatrix}${f(Pw[0], 4)}\\ ${f(Pw[1], 4)}\\ ${f(Pw[2], 4)}\end{bmatrix}`,
            html: R`<p>시점이 위에서 내려다보도록 바뀌었습니다.</p>
              <p>{0} → {1}은 수직축 둘레 θ₁ 회전뿐입니다. 그래서</p>
              ${UL('수평 거리 g₁(빨강)이 θ₁ 방향(청록 호)으로 놓임', 'x = c₁g₁, y = s₁g₁ 로 나뉨 (점선)', '높이 z = g₃ 은 그대로')}`
          },
          {
            t: '⑥ r = x² + y² + z²: θ₁, θ₂가 사라진다',
            live: {
              params: [{ id: 'q1', label: R`\(\theta_1\)`, min: -180, max: 180, step: 1, val: q[0], unit: '°' }, { id: 'q2', label: R`\(\theta_2\)`, min: -60, max: 120, step: 1, val: q[1], unit: '°' }],
              drag: { q1: pDrag.q1, q2: pDrag.q2 },
              apply(S, p) {
                clr(S);
                const qq = q.slice(); qq[0] = p.q1; qq[1] = p.q2;
                pieperDraw(S, qq, { joint: act(p) ? [act(p)] : [], hide: [2, 3, 4] });
                const kk = pFK(qq); pt(S, 'wc', app(pBase, p3(kk.Tw)), '손목 중심', { c: 'k', r: 0.35 });
                body(S, 'sph', 'F0', 'sph', [Math.sqrt(r2)], [0, 0, 0], { c: 'acc', op: 0.12 });
              },
              vpInfo(p) { const qq = q.slice(); qq[0] = p.q1; qq[1] = p.q2; const w = p3(pFK(qq).Tw); return card('', { g: gOf(p.q2, [f1, f2]), w, extra: `<div class="vr on" style="margin-top:4px"><b>r</b> = x² + y² + z² = ${f(w[0] ** 2 + w[1] ** 2 + w[2] ** 2, 4)} <span class="vn">· 그대로!</span></div>` }); },
              tex(p) {
                const qq = q.slice(); qq[0] = p.q1; qq[1] = p.q2; const w = p3(pFK(qq).Tw);
                return R`\begin{aligned}(x,y,z) &= (${f(w[0])},\ ${f(w[1])},\ ${f(w[2])})\\ r = x^2+y^2+z^2 &= \ha{${f(w[0] ** 2 + w[1] ** 2 + w[2] ** 2, 4)}}\\ &= g_1^2 + g_3^2\ (\theta_1\text{ 회전은 길이 보존})\\ &= f_1^2 + f_2^2\ (\theta_2\text{ 회전도 길이 보존})\end{aligned}`;
              }
            },
            html: R`<p>θ₁, θ₂를 돌려 보세요. 손목 중심은 반투명 구 위에서만 움직이고 <b>r은 변하지 않습니다</b>.</p>
              ${UL('θ₁, θ₂는 둘 다 어깨를 지나는 축 둘레의 회전', '회전은 길이를 바꾸지 않음 → \\(r = |{}^0P|^2 = |g|^2 = |f|^2\\)', '어깨~손목 중심 거리를 바꿀 수 있는 건 팔꿈치 θ₃뿐')}
              <p>θ₁, θ₂를 한 번에 없애는 것이 Pieper 방법의 핵심입니다.</p>`
          },
          {
            t: '⑦ 세 가지 경우 → 이 로봇은 a₁ = 0',
            do(S) {
              clr(S); pieperDraw(S, q, { upto: 3, glow: [3], hide: [1, 2, 4] }); wc(S); body(S, 'sph', 'F0', 'sph', [Math.sqrt(r2)], [0, 0, 0], { c: 'acc', op: 0.12 });
              ln(S, 'ax1', { f: 'J1', p: [0, 0, -0.25] }, { f: 'J1', p: [0, 0, 0.3] }, { c: 'k', dash: false, label: '축 1', lt: 1, lo: [0, 0, 0.04] });
              ln(S, 'ax2', { f: 'J2', p: [0, 0, -0.3] }, { f: 'J2', p: [0, 0, 0.3] }, { c: 'k', dash: false, label: '축 2', lt: 1, lo: [0, 0, 0.04] });
              pt(S, 'x12', { f: 'J2' }, 'a₁ = 0: 축 1, 2가 만남', { c: 'ok', r: 0.3 });
            },
            assume: [{ t: R`\(a_1 = 0\) (축 1과 축 2가 한 점에서 만남 — 초록 점) → 경우 1`, ok: true }],
            vpInfo: `<div class="vh">축 1·2의 관계로 경우가 나뉨</div><div class="vr on"><b>경우 1</b> a₁ = 0 (축이 만남) → r 하나로 θ₃ <span class="vn">· 이 로봇</span></div><div class="vr"><b>경우 2</b> sα₁ = 0 (축이 평행) → z 하나로 θ₃</div><div class="vr"><b>경우 3</b> 둘 다 아님 → 4차 방정식</div>`,
            tex: R`\begin{aligned}r &= \underbrace{(k_1c_2 + k_2s_2)\,2a_1}_{\theta_2\text{가 들어 있는 항}} + k_3(\theta_3)\\ z &= \underbrace{(k_1s_2 - k_2c_2)\,s\alpha_1}_{\theta_2\text{가 들어 있는 항}} + k_4(\theta_3)\\[4pt] \text{경우 1. } &a_1 = 0\ \Rightarrow\ r = k_3(\theta_3)\\ \text{경우 2. } &s\alpha_1 = 0\ \Rightarrow\ z = k_4(\theta_3)\\ \text{경우 3. } &\text{둘 다 아님}\ \Rightarrow\ \theta_2\text{ 소거, 4차 방정식}\end{aligned}`,
            html: R`<p><b>질문</b>: r을 쓰면 항상 θ₂가 사라질까?<br>→ 아닙니다. 축 1과 축 2의 관계(a₁, α₁)에 따라 다릅니다.</p>
              <p>위 식은 슬라이드 17의 일반식입니다. k₁~k₄는 θ₃만의 함수입니다.</p>
              ${UL('k₁ = f₁, k₂ = −f₂', 'k₃ = f₁² + f₂² + f₃² + a₁² + d₂² + 2d₂f₃', 'k₄ = f₃cα₁ + d₂cα₁')}
              <p>θ₂가 들어 있는 항을 0으로 만드는 조건에 따라 세 경우로 나뉩니다.</p>
              ${OL('<b>a₁ = 0</b> (축 1, 2가 한 점에서 만남)<br>r의 θ₂ 항이 0 → r = k₃(θ₃) → <b>r 하나로 θ₃</b>', '<b>sα₁ = 0</b> (축 1, 2가 평행)<br>z의 θ₂ 항이 0 → z = k₄(θ₃) → <b>z 하나로 θ₃</b>', '<b>둘 다 아님</b><br>r, z 두 식에서 θ₂를 소거 → θ₃의 4차 방정식 (손계산 어려움)')}
              <p><b>그림으로 보면</b></p>
              ${UL('a₁ = 0이면 θ₂의 회전 중심(어깨, 초록 점)이 축 1 위에 있음', '그래서 θ₂로 팔을 돌려도 손목 중심은 원점 중심의 구(반투명) 위에서만 움직임', 'a₁ ≠ 0이면 회전 중심이 원점에서 벗어나 θ₂에 따라 거리 r이 바뀜')}
              <p><b>이 로봇</b>: a₁ = 0 → 경우 1. ⑥에서 본 대로 \(r = k_3 = f_1^2 + f_2^2\)입니다.</p>`
          },
          {
            t: '⑧ r 식 전개 → a cos θ₃ + b sin θ₃ = c',
            view: vPlane,
            do(S) { clr(S); armDim(S, { glow: [3] }); wc(S, ''); fVecs(S, [f1, f2], { noLab: true }); whole(S, '√r = ' + f(Math.sqrt(r2), 4)); th3(S, q[2]); },
            vpInfo: card('f', { extra: `<div class="vr on" style="margin-top:4px"><b>r</b> = f₁² + f₂² = ${f(r2, 4)}</div>` }),
            tex: R`\begin{aligned} r &= f_1^2+f_2^2 = (a_2 + a_3c_3 + d_4s_3)^2 + (a_3s_3 - d_4c_3)^2\\ &= a_2^2 + a_3^2(c_3^2+s_3^2) + d_4^2(s_3^2+c_3^2) + 2a_2(a_3c_3+d_4s_3) + \underbrace{2a_3d_4(c_3s_3 - s_3c_3)}_{0}\\ &\Rightarrow\ \underbrace{2a_2a_3}_{a = ${f(ik0.A, 4)}}\cos\theta_3 + \underbrace{2a_2d_4}_{b = ${f(ik0.B, 4)}}\sin\theta_3 = \underbrace{r - a_2^2 - a_3^2 - d_4^2}_{c = ${f(ik0.C, 5)}}\end{aligned}`,
            html: R`<p>검은 화살표의 길이 \(\sqrt r\)은 직각삼각형(빨강 f₁, 초록 f₂)의 빗변입니다.</p>
              ${UL('피타고라스로 전개하면 교차항이 사라짐', 'cos θ₃, sin θ₃의 1차식만 남음', '\\(r = ' + f(r2, 5) + '\\)은 목표 위치에서 바로 계산되는 숫자')}`
          },
          {
            t: '⑨ 반각 치환 → θ₃ 두 개',
            view: vPlane,
            do(S) { clr(S); pieperDraw(S, q, { known: [3], joint: [3], hide: [1, 2, 4] }); pieperDraw(S, qAlt, { pre: 'x', noFrames: true, noDrag: true, noPed: true, op: 0.28 }); wc(S, ''); th3(S, good.t3, 'a3', 'θ₃ = ' + f(good.t3, 1) + '°'); },
            assume: [{ t: R`\(a^2 + b^2 - c^2 = ${f(ik0.disc, 6)} \ge 0\) (실수해 존재)`, ok: ik0.disc >= 0 }, R`\(a + c \ne 0\) (반각 공식의 분모)`],
            tex: R`\begin{aligned}&u = \tan\tfrac{\theta_3}{2},\ \ \cos\theta_3 = \tfrac{1-u^2}{1+u^2},\ \sin\theta_3 = \tfrac{2u}{1+u^2}\ \xrightarrow{\times(1+u^2)}\ a(1-u^2) + 2bu = c(1+u^2)\\ &\Rightarrow\ (a+c)u^2 - 2bu + (c-a) = 0\ \Rightarrow\ u = \frac{b \pm\sqrt{a^2+b^2-c^2}}{a+c} = ${f(good.sg > 0 ? (ik0.B + Math.sqrt(ik0.disc)) / (ik0.A + ik0.C) : (ik0.B - Math.sqrt(ik0.disc)) / (ik0.A + ik0.C), 4)}\ \text{or}\ ${f(alt.sg > 0 ? (ik0.B + Math.sqrt(ik0.disc)) / (ik0.A + ik0.C) : (ik0.B - Math.sqrt(ik0.disc)) / (ik0.A + ik0.C), 4)}\\ &\theta_3 = 2\tan^{-1}u = \ha{${deg(good.t3)}}\ \text{or}\ ${deg(alt.t3)}\end{aligned}`,
            html: R`<p>같은 거리 √r을 팔꿈치를 <b>굽혀서</b>도, <b>반대로 꺾어서</b>도 만들 수 있어 해가 두 개입니다.</p>
              ${UL('진한 팔: θ₃ = ' + f(good.t3, 1) + '°', '흐린 팔: θ₃ = ' + f(alt.t3, 1) + '°')}
              <p>어느 쪽인지는 관절 범위 조건으로 고릅니다(다음 단계).</p>`
          },
          {
            t: '⑩ θ₂: 선택한 θ₃로 f를 구해 각도 차',
            view: vPlane,
            do(S) {
              clr(S); both12(S, { glow: [2] }); wc(S, ''); whole(S, '');
              angArcs(S, good.t2, true);
            },
            vpInfo: card('g', { extra: `<div class="vr" style="margin-top:4px">θ₂ = (θ₂ + φ_f) − φ_f = ${f(phiG, 2)}° − (${f(phiF, 2)}°) = <b>${f(good.t2, 2)}°</b></div>` }),
            tex: R`\begin{aligned}&\theta_3 = ${deg(good.t3)}\ \Rightarrow\ f_1 = ${f(good.f1, 4)},\ f_2 = ${f(good.f2, 4)}\\ &\theta_2 = \text{atan2}(-z, g_1) - \text{atan2}(f_2, f_1) = \text{atan2}(${f(-Pw[2], 4)},\ ${f(ik0.g1, 4)}) - \text{atan2}(${f(good.f2, 4)},\ ${f(good.f1, 4)})\\ &\phantom{\theta_2} = ${deg(atan2d(-Pw[2], ik0.g1))} - (${deg(atan2d(good.f2, good.f1))}) = \ha{${deg(good.t2)}}\quad(\text{다른 후보: } ${deg(alt.t2)})\end{aligned}`,
            html: R`<p>④-2의 관계를 거꾸로 씁니다: 검정 호(수평 → 화살표)에서 주황 호(위팔 → 화살표)를 빼면 분홍 호 θ₂. 필요한 값의 출처:</p>
              ${UL('g₁ = √(x² + y²), z → 목표 위치에서', 'f₁, f₂ → 방금 구한 θ₃에서')}
              <p>두 θ₃ 후보 각각 θ₂를 계산하고, 관절 범위에 맞는 쪽을 택합니다.</p>`
          },
          {
            t: '⑪ θ₁: 위에서 본 방향',
            view: vTop,
            do(S) { clr(S); pieperDraw(S, q, { hide: [2, 3, 4], op: 0.45 }); wc(S, ''); th1(S, good.t1); ln(S, 'gh', { f: 'K1' }, [Wc[0], Wc[1], PI.H], { c: 'x', dash: false, label: 'g₁ = √(x²+y²)', lt: 0.5 }); },
            tex: R`x = c_1g_1,\ y = s_1g_1\ \Rightarrow\ \theta_1 = \text{atan2}(y, x) = \text{atan2}(${f(Pw[1], 4)},\ ${f(Pw[0], 4)}) = \ha{${deg(good.t1)}}\quad(g_1 > 0\text{인 해})`,
            html: R`<p>⑤를 거꾸로 씁니다. 위에서 본 손목 중심의 방향이 곧 θ₁입니다.</p>
              <p class="note">다른 해: \(\theta_1 \pm 180^\circ\) (g₁ < 0, 팔이 뒤로 넘어간 자세)<br>이 경우 θ₂, θ₃도 다시 계산되며, 관절 범위 조건으로 하나를 고릅니다.</p>`
          },
          {
            t: '⑫ ³₆R = (⁰₃R)ᵀ ⁰₆R: 손목이 담당할 회전',
            view: vWrist,
            do(S) { clr(S); pieperDraw(S, q, { hide: [1, 2, 3, 4], op: GH }); Wframe(S); axes(S); },
            tex: R`{}^0_3R = R_z(\theta_1)R_x(-90^\circ)R_z(\theta_2+\theta_3) = \begin{bmatrix} c_1c_{23} & -c_1s_{23} & -s_1\\ s_1c_{23} & -s_1s_{23} & c_1\\ -s_{23} & -c_{23} & 0\end{bmatrix},\qquad {}^3_6R = ({}^0_3R)^T\,{}^0_6R_d = ${tm(good.R36, 4)}`,
            html: R`<p>시점이 손목으로 확대되었습니다.</p>
              ${UL('<b>{3} 방향</b> 좌표계: 팔이 만들어 놓은 방향', '<b>{6}</b> 좌표계: 목표 방향', '둘 사이의 회전 \\({}^3_6R\\)을 손목 세 관절이 만들어야 함')}
              <p>\({}^0_6R = {}^0_3R\,{}^3_6R\)의 양변 왼쪽에 \(({}^0_3R)^{-1} = ({}^0_3R)^T\)를 곱해 구합니다.</p>`
          },
          {
            t: '⑬ θ₄: Ẑ₃(=축 4) 둘레 회전',
            view: vWrist,
            do(S) { clr(S); pieperDraw(S, [q[0], q[1], q[2], q[3], 0, 0], { hide: [1, 2, 3, 6], op: GH }); Wframe(S); arc(S, 'w4', { f: 'Wc', n: Z, u: X, ang: q[3], r: 0.09, label: 'θ₄ = ' + f(good.t4, 1) + '°', col: 'j4', grow: true }); },
            tex: R`\theta_4 = \text{atan2}(r_{23},\ r_{13}) = \text{atan2}(${f(good.R36[1][2], 4)},\ ${f(good.R36[0][2], 4)}) = \ha{${deg(good.t4)}}`,
            html: R`<p>Z-Y-Z의 첫 회전: {3}의 Ẑ축(축 4) 둘레로 θ₄.</p>
              ${UL('\\(R_ZR_YR_Z\\)의 3열 = \\((c_4s_5,\\ s_4s_5,\\ c_5)\\)', 'r₁₃, r₂₃의 비가 θ₄를 줌', 's₅ > 0이라 s₅로 나눠도 부호가 그대로 → atan2(r₂₃, r₁₃)')}`
          },
          {
            t: '⑭ θ₅: 새 Ŷ₄(=축 5) 둘레 회전',
            view: vWrist,
            do(S) { clr(S); pieperDraw(S, [q[0], q[1], q[2], q[3], q[4], 0], { hide: [1, 2, 3, 4], op: GH }); Wframe(S); arc(S, 'w5', { f: 'F4', n: [0, 1, 0], u: [0, 0, 1], ang: q[4], r: 0.1, label: 'θ₅ = ' + f(good.t5, 1) + '°', col: 'j5', grow: true }); },
            assume: [{ t: R`\(0^\circ < \theta_5 < 180^\circ\) → \(s_5 = +\sqrt{r_{31}^2+r_{32}^2}\)`, ok: q[4] > 0 && q[4] < 180 }],
            tex: R`\theta_5 = \text{atan2}\big(\sqrt{r_{31}^2+r_{32}^2},\ r_{33}\big) = \text{atan2}\big(${f(Math.hypot(good.R36[2][0], good.R36[2][1]), 4)},\ ${f(good.R36[2][2], 4)}\big) = \ha{${deg(good.t5)}}`,
            html: R`<p>둘째 회전: θ₄만큼 돌아간 프레임 {4}의 Ŷ축 둘레로 θ₅. Ẑ축이 θ₅만큼 기울어집니다(호). 끝 프레임은 아직 θ₆ = 0인 상태입니다.</p>
              ${UL('cos: r₃₃ = c₅', 'sin: \\(r_{31}^2 + r_{32}^2 = s_5^2\\) → s₅ = +√(…)', '둘 다 알므로 atan2로 하나로 결정')}
              <p class="note">음의 제곱근을 택하면 (θ₄ + 180°, −θ₅, θ₆ + 180°)인 쌍둥이 해가 나옵니다.<br>0° < θ₅ < 180° 조건이 그 해를 버립니다.</p>`
          },
          {
            t: '⑮ θ₆: 새 Ẑ₅(=축 6) 둘레 회전 → 목표 도달',
            view: vWrist,
            do(S) { clr(S); pieperDraw(S, q, { hide: [1, 2, 3, 4], op: GH }); Wframe(S); arc(S, 'w6', { f: 'F5', c: [0, 0, 0.05], n: Z, u: X, ang: q[5], r: 0.08, label: 'θ₆ = ' + f(good.t6, 1) + '°', col: 'j6', grow: true }); },
            tex: R`\theta_6 = \text{atan2}(r_{32},\ -r_{31}) = \text{atan2}(${f(good.R36[2][1], 4)},\ ${f(-good.R36[2][0], 4)}) = \ha{${deg(good.t6)}}\qquad (\theta_1,\dots,\theta_6) = (${[good.t1, good.t2, good.t3, good.t4, good.t5, good.t6].map(v => f(v, 1)).join(',\\ ')})^\circ`,
            html: R`<p>마지막 회전: 기울어진 Ẑ축(축 6) 둘레로 θ₆.</p>
              ${UL('3행 \\((-s_5c_6,\\ s_5s_6)\\) → c₆ = −r₃₁/s₅, s₆ = r₃₂/s₅', 'θ₆ = atan2(r₃₂, −r₃₁)')}
              <p>이제 끝의 프레임이 목표 방향 {6}과 일치합니다.</p>`
          },
          {
            t: '직접 움직여 보기: 6개 관절',
            live: {
              params: q.map((v, i) => ({ id: 'q' + (i + 1), label: R`\(\theta_${i + 1}\)`, min: -180, max: 180, step: 1, val: v, unit: '°' })),
              drag: pDrag,
              apply(S, p) {
                clr(S);
                const qq = [p.q1, p.q2, p.q3, p.q4, p.q5, p.q6];
                pieperDraw(S, qq, { joint: act(p) ? [act(p)] : [], hide: [1, 2] });
                const kk = pFK(qq); pt(S, 'wc', app(pBase, p3(kk.Tw)), '손목 중심', { c: 'k', r: 0.35 });
                watch(S, 'F6', 'F0', '실시간 \\({}^0_6T\\)');
              },
              tex(p) {
                const qq = [p.q1, p.q2, p.q3, p.q4, p.q5, p.q6], kk = pFK(qq), w = p3(kk.Tw), A = act(p);
                const arm = A && A <= 3, wr = A && A >= 4;
                return R`{}^0P_w = ${hlIf(arm, tv(w))}\ (\theta_1,\theta_2,\theta_3\text{만}),\qquad {}^3_6R = ${hlIf(wr, tm(R3(mul(rz(p.q4), ry(p.q5), rz(p.q6)))))}\ (\theta_4,\theta_5,\theta_6\text{만})`;
              }
            },
            html: R`<p>두 부분이 독립이라는 것을 직접 확인해 보세요.</p>
              ${UL('손목 관절(4, 5, 6)을 돌리면: 손목 중심 위치는 그대로, \\({}^3_6R\\)만 바뀜', '팔 관절(1, 2, 3)을 돌리면: 손목 중심이 움직임')}`
          }
        ]
      };
    }
  });

  /* ================= Practice: Pieper (HW2 P3 type, numbers changed) ================= */
  sec({
    id: 'c3-hw-pieper', group: 'practice', ref: '과제 2 · 문제 3 유형 (숫자 변경)', title: 'Pieper 방법으로 6개 관절각 구하기',
    view: pView, grid: [0.8, 0.1], scale: 0.2, fsize: 1,
    params: {
      orig: { q: [25, 40, -55, 30, 60, -20] }, origLabel: '기본 문제',
      rand() {
        const pick = (a, b) => a + Math.floor(Math.random() * (b - a + 1));
        for (let n = 0; n < 200; n++) {
          const q = [pick(-80, 80), pick(5, 85), pick(-120, 60), pick(-150, 150), pick(15, 165), pick(-150, 150)];
          const k = pFK(q), sol = pIK(p3(k.Tw), R3(k.T6));
          const okC = sol.cands.filter(c => c.t2 >= 0 && c.t2 <= 90 && c.t1 >= -90 && c.t1 <= 90);
          if (sol.disc > 1e-6 && okC.length === 1 && Math.abs(okC[0].t3 - q[2]) < 0.5) return { q };
        }
        return { q: [25, 40, -55, 30, 60, -20] };
      }
    },
    build(P) {
      const k = pFK(P.q);
      const Pt = p3(k.Tw).map(v => M.round(v, 4)), Rt = R3(k.T6).map(r => r.map(v => M.round(v, 4)));
      const sol = pIK(Pt, Rt);
      const good = sol.cands.findIndex(c => c.t2 >= -0.5 && c.t2 <= 90.5 && c.t1 >= -90 && c.t1 <= 90);
      const g = sol.cands[good < 0 ? 0 : good], bad = sol.cands[good === 0 ? 1 : 0];
      const qs = [g.t1, g.t2, g.t3, g.t4, g.t5, g.t6];
      const Tg = mul(pBase, fromRp(Rt, Pt));
      const tgt = S => { fr(S, 'TG', Tg, { label: '', name: '목표 {6}', size: 0.14, ghost: true, op: 0.5 }); pt(S, 'tg', p3(Tg), '목표', { c: 'k', r: 0.3 }); };
      const home = [0, 30, -30, 0, 60, 0];
      const lim = { 1: [-90, 90, 'θ₁ ∈ [−90°, 90°]'], 2: [0, 90, 'θ₂ ∈ [0°, 90°]'], 5: [0, 180, 'θ₅ ∈ (0°, 180°)'] };
      const qb = [bad.t1, bad.t2, bad.t3, bad.t4, bad.t5, bad.t6];
      const badDraw = (S, warn) => pieperDraw(S, qb, { pre: 'x', noFrames: true, noDrag: true, noPed: true, op: 0.28, c: warn ? 'warn' : null });
      return {
        calcData: { Pt, Rt, sol, g, bad },
        answers: ['θ₁', 'θ₂', 'θ₃', 'θ₄', 'θ₅', 'θ₆'].map((l, i) => ({ label: l, val: qs[i], unit: '°', tol: 0.5, dp: 2, mod: 360 })),
        init(S) { pieperDraw(S, home, { hide: [1, 2, 4] }); tgt(S); },
        steps: [
          {
            t: '문제',
            tex: R`a_1 = 0,\ d_2 = 0,\ \alpha_1 = -90^\circ,\ f_1 = 0.45 + 0.25c_3 + 0.12s_3,\ f_2 = 0.25s_3 - 0.12c_3,\ f_3 = 0\qquad {}^0_6T_d = ${tm(fromRp(Rt, Pt), 4)}`,
            html: R`<p>6회전 관절 로봇이고 마지막 세 축이 끝점 원점에서 만납니다. \({}^0_3R = R_Z(\theta_1)R_X(-90^\circ)R_Z(\theta_2+\theta_3)\), \({}^3_6R = R_Z(\theta_4)R_Y(\theta_5)R_Z(\theta_6)\). 위 목표 자세를 만드는 여섯 관절각을 Pieper 방법으로 구하시오. 조건: \(-90^\circ\le\theta_1\le 90^\circ,\ 0^\circ\le\theta_2\le 90^\circ,\ 0^\circ<\theta_5<180^\circ\).</p>
              <p class="note">과제 2 문제 3과 같은 풀이 흐름이고 숫자만 다릅니다. 손목은 문제에 주어진 \({}^3_6R\) 정의(축 4 ∥ \(\Zh_3\))대로 그렸습니다. 먼저 손으로 풀고 위에서 채점해 보세요.</p>`
          },
          {
            t: '① r로 θ₃ — 관절 3',
            do(S) { pieperDraw(S, qs, { hide: [1, 2, 4], known: [3], joint: [3] }); badDraw(S, false); body(S, 'sph', 'F0', 'sph', [Math.sqrt(sol.rr)], [0, 0, 0], { c: 'acc', op: 0.1 }); },
            assume: [R`축 4, 5, 6이 끝점 원점에서 만남 → 목표 위치 = 손목 중심`, R`\(a_1 = 0\) → 경우 1: r 식에 θ₃만 남음`, { t: R`실수해 존재: \(a^2+b^2-c^2 = ${f(sol.disc, 6)} \ge 0\)`, ok: sol.disc >= 0 }],
            tex: R`\begin{aligned}r &= x^2+y^2+z^2 = ${f(sol.rr, 5)} = f_1^2+f_2^2 = (0.45 + 0.25c_3 + 0.12s_3)^2 + (0.25s_3 - 0.12c_3)^2\\ &= 0.45^2+0.25^2+0.12^2 + 2(0.45)(0.25c_3 + 0.12s_3)\quad(\because\ c_3^2+s_3^2 = 1)\\ &\Rightarrow\ \underbrace{${f(sol.A, 4)}}_{a}\cos\theta_3 + \underbrace{${f(sol.B, 4)}}_{b}\sin\theta_3 = \underbrace{${f(sol.C, 5)}}_{c}\\ &u = \tan\tfrac{\theta_3}{2}:\ (a+c)u^2 - 2bu + (c-a) = 0\ \Rightarrow\ u = \frac{b\pm\sqrt{a^2+b^2-c^2}}{a+c}\ \Rightarrow\ \theta_3 = ${deg(sol.cands[0].t3)}\ \text{or}\ ${deg(sol.cands[1].t3)}\end{aligned}`,
            html: R`<p>f를 제곱해 더하면 교차항 \(0.25\cdot0.12(c_3s_3 - s_3c_3)\)이 사라지고 \(c_3^2+s_3^2 = 1\)로 정리되어 \(a\cos\theta_3 + b\sin\theta_3 = c\) 꼴이 됩니다. 반각 치환 \(\cos\theta = \frac{1-u^2}{1+u^2},\ \sin\theta = \frac{2u}{1+u^2}\)을 넣으면 u의 2차방정식 → 해 두 개.</p>
            <p>3D: 진한 팔과 흐린 팔이 두 θ₃ 후보로, 둘 다 손목 중심이 같은 구(반지름 \(\sqrt r\)) 위에 있습니다. 지금 정해진 <b>관절 3과 링크 3</b>만 진하게 빛납니다.</p>`
          },
          {
            t: '② θ₁ — 관절 1',
            do(S) { delete S.bodies.sph; pieperDraw(S, qs, { hide: [1, 2, 4], known: [1, 3], joint: [1], limits: lim }); badDraw(S, false); },
            assume: [{ t: R`\(-90^\circ \le \theta_1 = ${f(g.t1, 2)}^\circ \le 90^\circ\) — 반대편 해 \(\theta_1 \pm 180^\circ\) (\(g_1<0\)) 제외`, ok: g.t1 >= -90 && g.t1 <= 90 }],
            tex: R`\alpha_1 = -90^\circ,\ a_1 = d_2 = f_3 = 0\ \Rightarrow\ g_2 = 0\ \Rightarrow\ \begin{bmatrix}x\\y\end{bmatrix} = \begin{bmatrix}c_1g_1 - s_1\cdot 0\\ s_1g_1 + c_1\cdot 0\end{bmatrix} = g_1\begin{bmatrix}c_1\\s_1\end{bmatrix}\ \Rightarrow\ \theta_1 = \text{atan2}(${f(Pt[1], 4)},\ ${f(Pt[0], 4)}) = ${deg(g.t1)},\ \ g_1 = +${f(sol.g1, 4)}`,
            html: R`<p>손목 중심은 항상 {1}의 XZ 평면 위에 있으므로 수평 방위각이 곧 θ₁입니다. 관절 1 둘레의 초록 호가 가정한 범위 \([-90^\circ, 90^\circ]\)이고, 짧은 초록 화살표(현재 각)가 그 안에 있습니다. <b>관절 1</b>이 정해져 진해지고 빛납니다.</p>`
          },
          {
            t: '③ θ₂ — 관절 2, 그리고 해 고르기',
            do(S) { pieperDraw(S, qs, { hide: [1, 2, 4], known: [1, 2, 3], joint: [2], limits: lim }); badDraw(S, true); },
            assume: [{ t: R`\(0^\circ \le \theta_2 \le 90^\circ\): 선택한 해 \(\theta_2 = ${f(g.t2, 2)}^\circ\)`, ok: true }, { t: R`다른 후보 \(\theta_3 = ${f(bad.t3, 1)}^\circ\)이면 \(\theta_2 = ${f(bad.t2, 1)}^\circ\) → 조건 위반 (붉은 팔)`, ok: false }],
            tex: R`\begin{aligned}g_1 &= c_2f_1 - s_2f_2,\quad g_3 = -(s_2f_1 + c_2f_2) = z\ \Rightarrow\ \begin{bmatrix}g_1\\ -z\end{bmatrix} = \begin{bmatrix}c_2 & -s_2\\ s_2 & c_2\end{bmatrix}\begin{bmatrix}f_1\\ f_2\end{bmatrix}\ \Rightarrow\ \theta_2 = \text{atan2}(-z, g_1) - \text{atan2}(f_2, f_1)\\[4pt] &\theta_3 = ${deg(g.t3)}:\ (f_1,f_2) = (${f(g.f1, 4)},\ ${f(g.f2, 4)})\\ &\qquad\theta_2 = ${deg(atan2d(-Pt[2], sol.g1))} - (${deg(atan2d(g.f2, g.f1))}) = \ha{${deg(g.t2)}}\ \ \checkmark\\ &\theta_3 = ${deg(bad.t3)}:\ (f_1,f_2) = (${f(bad.f1, 4)},\ ${f(bad.f2, 4)})\\ &\qquad\theta_2 = ${deg(bad.t2)}\ \ \times\ (\text{조건 위반})\end{aligned}`,
            html: R`<p>\((g_1, -z)\)는 벡터 \((f_1, f_2)\)를 θ₂만큼 회전한 것이므로 각도 차로 θ₂를 얻습니다. 두 θ₃ 후보 각각에 대해 θ₂를 계산하고, 가정 \(0^\circ\le\theta_2\le 90^\circ\)를 만족하는 <span class="ta">강조된 해</span>를 택합니다. 붉은 반투명 팔이 버린 해입니다(관절 2의 호가 붉게 보이는 쪽).</p>`
          },
          {
            t: '④ ⁰₃R과 ³₆R — 팔이 만든 회전 빼기',
            do(S) { pieperDraw(S, qs, { hide: [1, 2, 4], known: [1, 2, 3], joint: [1, 2, 3], limits: lim }); },
            tex: R`{}^0_3R = R_Z(${f(g.t1, 2)}^\circ)R_X(-90^\circ)R_Z(${f(g.t2 + g.t3, 2)}^\circ) = ${tm(g.R03, 4)},\quad {}^3_6R = ({}^0_3R)^T\,{}^0_6R_d = ${tm(g.R36, 4)}`,
            html: R`<p>\({}^0_6R = {}^0_3R\,{}^3_6R\)에서 \({}^0_3R\)은 직교행렬이므로 역행렬 = 전치(1.4절). 팔 세 관절(빛나는 부분)이 만든 회전을 목표에서 빼면 손목이 담당할 회전이 남습니다.</p>`
          },
          {
            t: '⑤ θ₄, θ₅, θ₆ — 손목',
            do(S) { pieperDraw(S, qs, { hide: [1, 2, 4], joint: [4, 5, 6], limits: lim }); },
            assume: [{ t: R`\(0^\circ < \theta_5 = ${f(g.t5, 2)}^\circ < 180^\circ\) (즉 \(s_5 > 0\)) — 손목 관절 5의 초록 호`, ok: g.t5 > 0 && g.t5 < 180 }],
            tex: R`\begin{aligned}R_ZR_YR_Z &= \begin{bmatrix} c_4c_5c_6 - s_4s_6 & -c_4c_5s_6 - s_4c_6 & c_4s_5\\ s_4c_5c_6 + c_4s_6 & -s_4c_5s_6 + c_4c_6 & s_4s_5\\ -s_5c_6 & s_5s_6 & c_5\end{bmatrix}\\ \theta_5 &= \text{atan2}\big(\sqrt{(${f(g.R36[2][0], 4)})^2 + (${f(g.R36[2][1], 4)})^2},\ ${f(g.R36[2][2], 4)}\big) = ${deg(g.t5)},\ \ \theta_4 = \text{atan2}(${f(g.R36[1][2], 4)},\ ${f(g.R36[0][2], 4)}) = ${deg(g.t4)},\ \ \theta_6 = \text{atan2}(${f(g.R36[2][1], 4)},\ ${f(-g.R36[2][0], 4)}) = ${deg(g.t6)}\end{aligned}`,
            html: R`<p>3열 \((c_4s_5,\ s_4s_5,\ c_5)\)과 3행 \((-s_5c_6,\ s_5s_6,\ c_5)\)만 쓰면 됩니다. \(r_{31}^2+r_{32}^2 = s_5^2\)이고, 가정 \(s_5 > 0\)이므로 양의 제곱근을 씁니다. 그 덕분에 \(r_{13}/s_5,\ r_{23}/s_5\)의 부호가 그대로 \(c_4, s_4\)의 부호가 됩니다. <b>손목 관절 4, 5, 6</b>이 빛납니다.</p>`
          },
          {
            t: '⑥ 확인: 구한 각도로 움직이기',
            live: {
              params: qs.map((v, i) => ({ id: 'q' + (i + 1), label: R`\(\theta_${i + 1}\)`, min: -180, max: 180, step: 1, val: M.round(v, 1), unit: '°' })),
              from: Object.fromEntries(home.map((v, i) => ['q' + (i + 1), v])),
              drag: pDrag,
              apply(S, p) {
                drop(S, 'bodies');
                const qq = [p.q1, p.q2, p.q3, p.q4, p.q5, p.q6];
                pieperDraw(S, qq, { joint: act(p) ? [act(p)] : [], hide: [1, 2, 4], limits: lim });
                watch(S, 'F6', 'F0', '실시간 \\({}^0_6T\\)');
              },
              assume: p => [{ t: R`\(-90^\circ\le\theta_1 = ${f(p.q1, 1)}^\circ\le 90^\circ\)`, ok: p.q1 >= -90 && p.q1 <= 90 }, { t: R`\(0^\circ\le\theta_2 = ${f(p.q2, 1)}^\circ\le 90^\circ\)`, ok: p.q2 >= 0 && p.q2 <= 90 }, { t: R`\(0^\circ<\theta_5 = ${f(p.q5, 1)}^\circ<180^\circ\)`, ok: p.q5 > 0 && p.q5 < 180 }],
              tex(p) {
                const qq = [p.q1, p.q2, p.q3, p.q4, p.q5, p.q6], T = pFK(qq).T6, e = norm(sub(p3(T), Pt));
                return R`{}^0_6T = ${tm(T, 4)},\qquad |\Delta P| = ${f(e * 1000, 1)}\ \text{mm}`;
              }
            },
            html: R`<p>처음 자세에서 구한 관절각까지 관절 공간에서 움직여 목표 프레임(흐린 좌표계)에 도착합니다. 관절을 끌어 다른 자세를 만들고 \({}^0_6T\)가 어떻게 달라지는지도 확인해 보세요.</p>`
          }
        ]
      };
    }
  });

  /* ================= Practice: planar 3R numeric ================= */
  sec({
    id: 'c3-hw-3r', group: 'practice', ref: '강의 3 · 평면 3R 연습', title: '평면 3R 팔: 목표 자세에서 관절각',
    view: { pos: [0.7, -5.4, 0.8], target: [0.6, 0, 0.9] }, grid: [3, 0.5], scale: 0.7, fsize: 1,
    params: {
      orig: { x: 1.3, y: 1.1, phi: 30 }, origLabel: '기본 문제',
      rand() {
        const pick = (a, b) => a + Math.floor(Math.random() * (b - a + 1));
        for (let n = 0; n < 100; n++) { const x = pick(-150, 200) / 100, y = pick(30, 200) / 100; const rr = Math.hypot(x, y); if (rr > 0.45 && rr < 2.1) return { x, y, phi: pick(-90, 150) }; }
        return { x: 1.3, y: 1.1, phi: 30 };
      }
    },
    build(P) {
      const p = Object.assign({ sol: 0 }, P);
      const r = ik2(L3[0], L3[1], p.x, p.y), s = r.sols[0], t3 = wrap(p.phi - s.t1 - s.t2);
      return {
        answers: [
          { label: R`\(\theta_1\)`, val: s.t1, unit: '°', tol: 0.3, dp: 2, mod: 360 },
          { label: R`\(\theta_2\)`, val: s.t2, unit: '°', tol: 0.3, dp: 2 },
          { label: R`\(\theta_3\)`, val: t3, unit: '°', tol: 0.3, dp: 2, mod: 360 }
        ],
        init(S) { algScene(S, p, { hide: [1, 2], known: [] }); },
        steps: [
          {
            t: '문제',
            tex: R`l_1 = 1.2,\ l_2 = 1.0,\qquad (x, y, \phi) = (${p.x},\ ${p.y},\ ${p.phi}^\circ),\qquad \theta_2 > 0\ \text{인 해}`,
            html: R`<p>3.3절과 같은 평면 3R 팔입니다. 손목 프레임 {3}이 위 목표와 일치하도록 하는 \(\theta_1, \theta_2, \theta_3\)를 구하시오(\(\theta_2>0\)인 해). 대수적·기하학적 방법 중 편한 쪽으로 풀어 보세요.</p>`
          },
          {
            t: '① θ₂ — 관절 2',
            do(S) { algScene(S, p, { hide: [1, 2], known: [2], joint: [2], both: true, bothWarn: true, limits: { 2: [0, 180, '가정: θ₂ > 0'] } }); },
            assume: [{ t: R`\(\theta_2 > 0\) (\(s_2>0\)) 해 — 초록 호. 붉은 팔은 \(\theta_2<0\) 해`, ok: true }, { t: R`\(|c_2| \le 1\): \(c_2 = ${f(r.c2, 4)}\)`, ok: r.ok }],
            tex: R`c_2 = \frac{${f(p.x * p.x + p.y * p.y, 4)} - 1.44 - 1}{2.4} = ${f(r.c2, 4)},\quad s_2 = +\sqrt{1-c_2^2} = ${f(s.s2, 4)},\quad \theta_2 = ${deg(s.t2)}`,
            html: R`<p>흐린 팔이 \(\theta_2<0\)인 다른 해입니다.</p>`
          },
          {
            t: '② θ₁ — 관절 1',
            do(S) { algScene(S, p, { hide: [2], known: [1, 2], joint: [1], limits: { 2: [0, 180, '가정: θ₂ > 0'] } }); kVecs(S, s); },
            tex: R`k_1 = 1.2 + 1.0c_2 = ${f(s.k1, 4)},\ k_2 = 1.0s_2 = ${f(s.k2, 4)},\quad \theta_1 = \text{atan2}(${p.y}, ${p.x}) - \text{atan2}(k_2, k_1) = ${deg(atan2d(p.y, p.x))} - ${deg(atan2d(s.k2, s.k1))} = ${deg(s.t1)}`,
            html: R`<p>기하학적으로는 \(\theta_1 = \beta - \psi\) (θ₂ > 0이므로)와 같은 값입니다.</p>`
          },
          {
            t: '③ θ₃ — 관절 3',
            do(S) { algScene(S, p, { known: [1, 2, 3], joint: [3] }); },
            tex: R`\theta_3 = ${p.phi}^\circ - (${f(s.t1, 2)}^\circ) - (${f(s.t2, 2)}^\circ) = ${deg(t3)}`,
            html: R`<p>손목 프레임이 목표 프레임과 겹쳤습니다.</p>`
          }
        ]
      };
    }
  });

  window.COURSE.chapters.push({ id: 'ch3', title: 'Lecture 3 · Manipulator Inverse Kinematics', short: 'Ch3 역기구학', sections });
  window.COURSE.upcoming = [];
})();
