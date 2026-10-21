/* =========================================================
   Chapter 2 — Manipulator Forward Kinematics (Lecture 2)
   Craig (modified) D-H:  i-1_i T = Rx(α_{i-1}) Dx(a_{i-1}) Rz(θ_i) Dz(d_i)
   ========================================================= */
(function () {
  'use strict';
  const R = String.raw;
  const { I, mul, rx, ry, rz, tr, inv, app, R3, p3, fromRp, col, unit, cross, norm, scl, add, sub, atan2d, f, tm, tv, D } = M;
  const X = [1, 0, 0], Yv = [0, 1, 0], Z = [0, 0, 1], O = [0, 0, 0];

  const fr = (S, id, T, o) => { S.frames[id] = Object.assign({ T, label: id }, o || {}); };
  const pt = (S, id, at, label, o) => { S.pts[id] = Object.assign({ at, label, c: 'pt' }, o || {}); };
  const vec = (S, id, a, b, o) => { S.vecs[id] = Object.assign({ a, b, c: 'pt' }, o || {}); };
  const ln = (S, id, a, b, o) => { S.lines[id] = Object.assign({ a, b, c: 'mute' }, o || {}); };
  const arc = (S, id, o) => { S.arcs[id] = Object.assign({ c: [0, 0, 0], r: 0.6 }, o); };
  const body = (S, id, fId, shape, size, pos, o) => { S.bodies[id] = Object.assign({ f: fId, shape, size, pos: pos || [0, 0, 0] }, o || {}); };
  const drop = (S, kind, pre) => { Object.keys(S[kind]).forEach(k => { if (!pre || k.indexOf(pre) === 0) delete S[kind][k]; }); };
  const watch = (S, fId, rel, title) => { S.watch = { f: fId, rel, title }; };
  const deg = x => f(x, 2) + '^\\circ';
  const cls = j => ['cx', 'cy', 'cz', 'ca'][j];

  /* ---------- D-H machinery ---------- */
  const dhT = (al, a, th, d) => mul(rx(al), tr(a, 0, 0), rz(th), tr(0, 0, d));
  const jv = (r, qi) => ({ th: r.type === 'R' ? (r.th || 0) + qi : (r.th || 0), d: r.type === 'P' ? (r.d || 0) + qi : (r.d || 0) });
  function chainT(rows, q, base) {
    const Ts = [base || I()], Js = [null];
    rows.forEach((r, i) => {
      const v = jv(r, q[i] || 0);
      const J = mul(Ts[i], rx(r.al), tr(r.a, 0, 0), rz(v.th));
      Js.push(J); Ts.push(mul(J, tr(0, 0, v.d)));
    });
    return { Ts, Js };
  }
  /* draws a D-H robot: frames F0..Fn, hidden joint frames J1..Jn (after θ) and K1..Kn (before θ),
     link/joint bodies (drag tags q1..qn).
     Two highlighting semantics:
       o.upto / o.glow  — "transform i" = link a_{i-1} + joint i (+ offset d_i)   (forward-kinematics derivations)
       o.known / o.joint — "joint i"   = joint i housing + the link it carries (link i) (IK, dragging)
     o.limits = {i:[min,max]} draws the allowed range of joint i as an arc (assumption), green if satisfied, red if not.
     o.c overrides body colour (e.g. 'warn' for a rejected solution). */
  function robot(S, rows, q, o) {
    o = o || {};
    const P = o.pre || '';
    const { Ts, Js } = chainT(rows, q, o.base);
    const n = rows.length, w = o.w || 0.07, jr = o.jr || 0.09, jh = o.jh || 0.24;
    const ex = (m, jm, extra) => {
      const r = Object.assign({}, extra);
      if (o.upto != null && m > o.upto) { r.op = 0.1; r.drag = null; }
      if (o.glow && o.glow.indexOf(m) >= 0) r.glow = true;
      if (o.joint && o.joint.indexOf(jm) >= 0) r.glow = true;
      if (o.known && jm > 0 && o.known.indexOf(jm) < 0) { r.op = 0.13; r.drag = null; r.glow = false; }
      if (o.c) r.c = o.c;
      if (o.op != null) r.op = o.op;
      if (o.noDrag) r.drag = null;
      return r;
    };
    for (let i = 0; i <= n; i++) {
      const hide = o.noFrames || (o.hide && o.hide.indexOf(i) >= 0) || (o.upto != null && i > o.upto);
      fr(S, P + 'F' + i, Ts[i], Object.assign({ label: String(i), size: o.fs || 0.42, hidden: hide }, (o.frameOpt && o.frameOpt[i]) || {}));
    }
    for (let i = 1; i <= n; i++) {
      const r = rows[i - 1], v = jv(r, q[i - 1] || 0);
      fr(S, P + 'J' + i, Js[i], { hidden: true });
      fr(S, P + 'K' + i, mul(Ts[i - 1], rx(r.al), tr(r.a, 0, 0)), { hidden: true });
      const jc = 'j' + Math.min(i, 6);
      if (r.type === 'R') body(S, P + 'jh' + i, P + 'J' + i, 'cyl', [jr, jh], [0, 0, v.d], ex(i, i, { c: jc, drag: 'q' + i }));
      else body(S, P + 'jh' + i, P + 'J' + i, 'box', [jr * 2, jr * 2, jh * 1.2], [0, 0, 0], ex(i, i, { c: jc, drag: 'q' + i }));
      if (o.op == null && !o.noLabels) { const xo = ex(i, i, {}); pt(S, P + 'jl' + i, { f: P + 'J' + i, p: [0, 0, v.d + (r.type === 'R' ? jh * 0.62 : jh * 0.75)] }, 'J' + i, { c: jc, r: 0.001, op: xo.op }); }
      if (Math.abs(v.d) > 1e-6) {
        if (r.type === 'P') body(S, P + 'ds' + i, P + 'F' + i, 'box', [w * 0.8, w * 0.8, Math.abs(v.d)], [0, 0, -v.d / 2], ex(i, i, { c: jc, drag: 'q' + i }));
        else body(S, P + 'ds' + i, P + 'J' + i, 'box', [w, w, Math.abs(v.d)], [0, 0, v.d / 2], ex(i, i, { c: jc, drag: 'q' + i }));
      }
      const lim = o.limits && o.limits[i];
      if (lim && r.type === 'R') {
        const th = v.th, inR = th >= lim[0] - 0.05 && th <= lim[1] + 0.05;
        arc(S, P + 'lim' + i, { f: P + 'K' + i, c: [0, 0, v.d + (o.limZ || 0)], n: [0, 0, 1], u: [Math.cos(lim[0] * D), Math.sin(lim[0] * D), 0], ang: lim[1] - lim[0], r: o.limR || jr * 3.4, noHead: true, col: inR ? 'ok' : 'warn', label: (lim[2] || `가정: ${lim[0]}° ≤ θ${i} ≤ ${lim[1]}°`) });
        vec(S, P + 'cur' + i, { f: P + 'J' + i, p: [0, 0, v.d + (o.limZ || 0)] }, { f: P + 'J' + i, p: [(o.limR || jr * 3.4) * 1.08, 0, v.d + (o.limZ || 0)] }, { c: inR ? 'ok' : 'warn', w: 0.45, label: '' });
      }
    }
    for (let i = 0; i <= n; i++) {
      const a = i < n ? rows[i].a : (o.tool || 0);
      if (Math.abs(a) > 1e-6) body(S, P + 'lk' + i, P + 'F' + i, 'box', [Math.abs(a), w, w], [a / 2, 0, 0], ex(i < n ? i + 1 : n, i, { c: i === 0 ? 'body2' : 'j' + Math.min(i, 6), drag: i > 0 && o.drag !== false ? 'q' + i : null }));
    }
    if (o.toolZ) body(S, P + 'tz', P + 'F' + n, 'box', [0.06, 0.06, o.toolZ], [0, 0, o.toolZ / 2], ex(n, n, { c: 'ink' }));
    if (o.pedestal) body(S, P + 'ped', 'W', 'cyl', [o.pedestal[0], o.pedestal[1]], [0, 0, o.pedestal[1] / 2], { c: o.c || 'body2', op: o.op });
    return { Ts, Js };
  }
  const act = p => (p && p._active && /^q\d$/.test(p._active)) ? +p._active.slice(1) : null;
  const hlIf = (on, t) => on ? R`\ha{${t}}` : t;
  const dragSpec = (rows, P) => Object.fromEntries(rows.map((r, i) => ['q' + (i + 1), { frame: (P || '') + 'F' + (i + 1), type: r.type === 'P' ? 'lin' : 'rot' }]));
  const sym = (v, fallback) => v;
  function dhTable(rows, opt) {
    opt = opt || {};
    const cell = (x, i, key) => (opt.sym && opt.sym[i] && opt.sym[i][key] != null) ? opt.sym[i][key] : (key === 'al' || key === 'th' ? (Math.abs(x) < 1e-9 ? '0' : f(x, 2) + '^\\circ') : f(x, 3));
    const body = rows.map((r, i) => {
      const v = opt.q ? jv(r, opt.q[i]) : null;
      let th = r.type === 'R' ? (opt.q ? R`\ha{${f(v.th, 1)}^\circ}` : R`\ha{\theta_${i + 1}}`) : cell(r.th || 0, i, 'th');
      let d = r.type === 'P' ? (opt.q ? R`\ha{${f(v.d, 3)}}` : R`\ha{d_${i + 1}}`) : cell(r.d || 0, i, 'd');
      const cells = [R`\class{cj${Math.min(i + 1, 6)}}{${i + 1}}`, cell(r.al, i, 'al'), cell(r.a, i, 'a'), d, th];
      return (opt.active === i + 1 ? cells.map(c => R`\class{ca}{${c}}`) : cells).join(' & ');
    }).join(R` \\ `);
    return R`\begin{array}{c|cccc} i & \alpha_{i-1} & a_{i-1} & d_i & \theta_i \\ \hline ${body}\end{array}`;
  }
  const dhSymMat = (i, j) => R`\begin{bmatrix} c\theta_${i} & -s\theta_${i} & 0 & a_{${j}}\\ s\theta_${i}c\alpha_{${j}} & c\theta_${i}c\alpha_{${j}} & -s\alpha_{${j}} & -s\alpha_{${j}}d_${i}\\ s\theta_${i}s\alpha_{${j}} & c\theta_${i}s\alpha_{${j}} & c\alpha_{${j}} & c\alpha_{${j}}d_${i}\\ 0&0&0&1\end{bmatrix}`;
  const Tsym = (a, b) => R`{}^{${a}}_{${b}}T`;

  const sections = [];
  const sec = o => sections.push(o);

  /* ================= 2.1 FK & joint variables ================= */
  {
    const rows = [{ al: 0, a: 0, d: 0.6, th: 0, type: 'R' }, { al: 90, a: 0, d: 0, th: 0, type: 'P' }];
    const show = (S, q) => {
      robot(S, rows, q, { fs: 0.35, w: 0.06, jh: 0.2, frameOpt: { 0: { size: 0.6 } } });
      pt(S, 'E', { f: 'F2' }, '끝점');
    };
    sec({
      id: 'c2-fk', group: 'lecture', ref: '슬라이드 3, 8', title: '2.1 순기구학과 관절 변수',
      view: { pos: [2.6, -2.8, 2.3], target: [0.1, 0, 0.45] }, grid: [2, 0.25], scale: 0.55, fsize: 1,
      build() {
        return {
          init(S) { show(S, [0, 0.4]); },
          steps: [
            {
              t: '순기구학이 푸는 문제',
              tex: R`\underbrace{q = \begin{bmatrix}q_1\\ \vdots\\ q_n\end{bmatrix}}_{\text{joint space}}\ \xrightarrow{\ \text{forward kinematics}\ }\ \underbrace{{}^0_NT = \begin{bmatrix}{}^0_NR & {}^0P\\ 0 & 1\end{bmatrix}}_{\text{Cartesian space}}`,
              html: R`<p><b>순기구학(forward kinematics)</b>은 각 관절의 값(관절 변수)을 알 때 끝점(end-effector)의 위치와 방향을 구하는 문제입니다. 힘이나 토크는 고려하지 않고 기하학만 다룹니다.</p>
              <p>화면의 로봇은 관절 2개짜리 예입니다: 1번은 수직축 회전, 2번은 팔이 길이 방향으로 늘어나는 관절입니다.</p>`
            },
            {
              t: '관절은 두 종류: 회전과 직동',
              do(S) { show(S, [60, 0.9]); },
              motion: [{ f: ['J1', 'F1', 'J2', 'F2'], rot: [Z, 60], about: 'world' }, { f: 'F2', tr: [0, 0, 0.5], about: 'local' }],
              tex: R`q_i = \begin{cases}\theta_i & \text{if joint } i \text{ is revolute (회전)}\\ d_i & \text{if joint } i \text{ is prismatic (직동)}\end{cases}`,
              html: R`<p>먼저 관절 1이 <b>회전(revolute)</b>해 각도 \(\theta_1\)이 바뀌고, 그다음 관절 2가 <b>직동(prismatic)</b>으로 길이 \(d_2\)만큼 늘어납니다.</p>
              <p>각 관절은 자유도가 1이라 변수 하나(\(q_i\))로 상태가 정해집니다. 자유도 n인 로봇은 관절이 n개이고 관절 벡터 \(q\)는 n×1입니다.</p>`
            },
            {
              t: '관절값을 바꾸면 끝점이 바뀐다',
              live: {
                params: [
                  { id: 'q1', label: R`\(\theta_1\)`, min: -180, max: 180, step: 1, val: 60, unit: '°' },
                  { id: 'q2', label: R`\(d_2\)`, min: 0.1, max: 1.4, step: 0.01, val: 0.9, unit: 'm', dp: 2 }
                ],
                drag: dragSpec(rows),
                apply(S, p) { show(S, [p.q1, p.q2]); watch(S, 'F2', 'F0', '실시간 \\({}^0_2T\\)'); },
                tex(p) {
                  const c = Math.cos(p.q1 * D), s = Math.sin(p.q1 * D);
                  return R`{}^0P = \begin{bmatrix} d_2\sin\theta_1\\ -d_2\cos\theta_1\\ 0.6\end{bmatrix} = \begin{bmatrix} (${f(p.q2, 2)})(${f(s)})\\ -(${f(p.q2, 2)})(${f(c)})\\ 0.6\end{bmatrix} = ${tv([p.q2 * s, -p.q2 * c, 0.6])}`;
                }
              },
              html: R`<p>\(\theta_1\)은 끝점을 수평 원 위에서 돌리고, \(d_2\)는 그 원의 반지름을 바꿉니다. 어떤 관절값을 넣든 끝점 위치는 위 식 하나로 계산됩니다. 이것이 순기구학 식입니다.</p>
              <p class="note">3D 화면에서 J1 기둥(청록)을 잡고 돌리면 \(\theta_1\), J2의 막대(분홍)를 앞뒤로 끌면 \(d_2\)가 바뀝니다. 같은 색 = 같은 관절이 움직이는 부분입니다.</p>`
            }
          ]
        };
      }
    });
  }

  /* ================= 2.2 a, alpha ================= */
  {
    const scene = (S, a, al, lv) => {
      const Tb = mul(tr(a, 0, 0), rx(al));
      fr(S, 'Fm', I(), { label: 'i−1', name: '{i−1}', size: 0.55, hl: lv >= 3 ? 'xz' : 'z' });
      fr(S, 'Ab', Tb, { hidden: true });
      ln(S, 'axA', [0, 0, -1.2], [0, 0, 1.5], { c: 'z', dash: false, label: '축 i−1', lt: 1, lo: [0, 0, 0.1] });
      ln(S, 'axB', { f: 'Ab', p: [0, 0, -1.2] }, { f: 'Ab', p: [0, 0, 1.5] }, { c: 'k', dash: false, label: '축 i', lt: 1, lo: [0, 0, 0.1] });
      body(S, 'hA', 'Fm', 'cyl', [0.12, 0.5], [0, 0, 0], { c: 'body2' });
      body(S, 'hB', 'Ab', 'cyl', [0.12, 0.5], [0, 0, 0], { c: 'body2' });
      if (a > 1e-3) body(S, 'link', 'Fm', 'box', [a, 0.1, 0.1], [a / 2, 0, 0], { c: 'j1' });
      if (lv >= 2) vec(S, 'a', [0, 0, 0], [a, 0, 0], { c: 'acc', w: 0.9, label: 'a<sub>i−1</sub> = ' + f(a, 2), lo: [0, 0, 0.22] });
      if (lv >= 3) {
        ln(S, 'zc', [a, 0, 0], [a, 0, 1.0], { c: 'z', label: 'Ẑ<sub>i−1</sub> 평행이동', lt: 1, lo: [0, 0, 0.1] });
        if (Math.abs(al) > 0.5) arc(S, 'al', { c: [a, 0, 0], n: X, u: Z, ang: al, r: 0.65, label: 'α<sub>i−1</sub> = ' + f(al, 0) + '°' });
      }
    };
    const v0 = { a: 1.4, al: 50 };
    sec({
      id: 'c2-aal', group: 'lecture', ref: '슬라이드 4–6, 9', title: '2.2 링크 길이 a와 비틀림 α',
      view: { pos: [2.4, -3.6, 2.2], target: [0.7, 0, 0.3] }, grid: [2.5, 0.5], scale: 0.6, fsize: 1,
      build() {
        return {
          init(S) { scene(S, v0.a, v0.al, 1); },
          steps: [
            {
              t: '링크 = 두 관절 축을 잇는 강체',
              tex: R`\text{link } i-1:\ \text{axis } i-1 \;\longrightarrow\; \text{axis } i`,
              html: R`<p>매니퓰레이터는 관절로 이어진 링크들의 사슬입니다. 링크 \(i-1\)의 양 끝에는 관절 축 \(i-1\)(파랑)과 관절 축 \(i\)(보라)가 붙어 있습니다. 링크 0은 바닥(base), 링크 1이 처음 움직이는 링크입니다.</p>
              <p>링크의 실제 모양(곡선, 두께)은 중요하지 않습니다. <b>두 축이 공간에서 어떻게 놓였는지</b>만 기구학에 영향을 줍니다. 이를 숫자 2개로 나타냅니다.</p>`
            },
            {
              t: '링크 길이 a: 두 축의 공통 수선',
              assume: [R`\(a_{i-1} \ge 0\) (거리이므로 항상 양수)`],
              do(S) { scene(S, v0.a, v0.al, 2); },
              tex: R`a_{i-1} = \text{distance from } \Zh_{i-1} \text{ to } \Zh_i \text{ measured along } \Xh_{i-1}`,
              html: R`<p>꼬인 위치에 있는 두 직선 사이에는 <b>두 직선에 모두 수직인 선분(공통 수선, mutual perpendicular)</b>이 딱 하나 있습니다. 그 길이가 링크 길이 \(a_{i-1}\)입니다(주황 화살표).</p>
              <p>이 공통 수선 방향이 곧 \(\Xh_{i-1}\)이 됩니다. 그래서 a는 "X축을 따라 잰 거리"입니다.</p>`
            },
            {
              t: '링크 비틀림 α: X축으로 잰 두 축의 각도',
              do(S) { scene(S, v0.a, v0.al, 3); },
              tex: R`\alpha_{i-1} = \text{angle from } \Zh_{i-1} \text{ to } \Zh_i \text{ measured about } \Xh_{i-1}\ (\text{right-hand})`,
              html: R`<p>축 \(i-1\)을 공통 수선 끝까지 평행이동한 선(파랑 점선)에서 축 \(i\)(보라)까지, \(\Xh_{i-1}\)을 엄지로 하는 오른손 방향으로 잰 각이 \(\alpha_{i-1}\)입니다.</p>`
            },
            {
              t: '직접 바꿔 보기: 특수한 경우',
              assume: [R`\(a_{i-1} \ge 0\)`, R`α는 \(\Xh_{i-1}\)을 엄지로 한 오른손 방향이 +`],
              live: {
                params: [
                  { id: 'a', label: R`\(a_{i-1}\)`, min: 0, max: 2.4, step: 0.05, val: v0.a, dp: 2 },
                  { id: 'al', label: R`\(\alpha_{i-1}\)`, min: -180, max: 180, step: 1, val: v0.al, unit: '°' }
                ],
                apply(S, p) { drop(S, 'arcs'); drop(S, 'vecs'); drop(S, 'lines'); drop(S, 'bodies'); scene(S, p.a, p.al, 3); },
                tex(p) {
                  const kind = p.a < 0.025 && Math.abs(p.al) < 1 ? R`\text{같은 직선 (coincident)}` : p.a < 0.025 ? R`a=0:\ \text{두 축이 만난다 (intersect)}` : (Math.abs(p.al) < 1 || Math.abs(Math.abs(p.al) - 180) < 1) ? R`\alpha=0^\circ/180^\circ:\ \text{두 축이 평행 (parallel)}` : R`\text{꼬인 위치 (skew)}`;
                  return R`a_{i-1} = ${f(p.a, 2)},\quad \alpha_{i-1} = ${f(p.al, 0)}^\circ\qquad\Rightarrow\quad ${kind}`;
                }
              },
              html: R`<p>a를 0으로 내리면 두 축이 한 점에서 만나고(PUMA 손목처럼), α를 0°로 두면 두 축이 평행해집니다(평면 팔처럼). 실제 로봇은 대부분 이런 특수한 값(0, ±90°)을 씁니다.</p>
              <p class="note">a = 0일 때는 공통 수선이 길이 0이라 방향이 정해지지 않습니다. 이때는 \(\Xh\)를 두 Z축이 이루는 평면에 수직으로 잡습니다(슬라이드 10).</p>`
            }
          ]
        };
      }
    });
  }

  /* ================= 2.3 d, theta ================= */
  {
    const v0 = { d: 0.7, th: 55 };
    const scene = (S, d, th, lv) => {
      const Ti = mul(tr(0, 0, d), rz(th));
      fr(S, 'Fi', Ti, { label: 'i', size: 0.55 });
      ln(S, 'ax', [0, 0, -0.8], [0, 0, 1.8], { c: 'k', dash: false, label: '축 i (Ẑ<sub>i</sub>)', lt: 1, lo: [0, 0, 0.1] });
      body(S, 'hub', 'W', 'cyl', [0.12, 0.35], [0, 0, 0], { c: 'body2' });
      body(S, 'lkm', 'W', 'box', [1.4, 0.1, 0.1], [-0.7, 0, 0], { c: 'j1' }); pt(S, 'lkmL', [-1.1, 0, 0.12], '링크 i−1', { c: 'j1', r: 0.001 });
      vec(S, 'xm', [-1.4, 0, 0], [0, 0, 0], { c: 'x', w: 0.7, label: 'a<sub>i−1</sub> 방향 (X̂<sub>i−1</sub>)', lt: 0.3, lo: [0, 0, 0.15] });
      body(S, 'lk', 'Fi', 'box', [1.3, 0.1, 0.1], [0.65, 0, 0], { c: 'j2', drag: 'th' }); pt(S, 'lkL', { f: 'Fi', p: [1.15, 0, 0.12] }, '링크 i', { c: 'j2', r: 0.001 });
      if (Math.abs(d) > 1e-3) body(S, 'col', 'Fi', 'box', [0.08, 0.08, Math.abs(d)], [0, 0, -d / 2], { c: 'j2', drag: 'd' });
      if (lv >= 2) vec(S, 'dv', [0, 0, 0], [0, 0, d], { c: 'acc', w: 0.9, label: 'd<sub>i</sub> = ' + f(d, 2), lt: 0.5, lo: [0.25, 0, 0] });
      if (lv >= 3) {
        ln(S, 'xe', [0, 0, d], [1.1, 0, d], { c: 'x', label: 'X̂<sub>i−1</sub> 연장', lt: 1, lo: [0.1, 0, 0] });
        if (Math.abs(th) > 0.5) arc(S, 'th', { c: [0, 0, d], n: Z, u: X, ang: th, r: 0.7, label: 'θ<sub>i</sub> = ' + f(th, 0) + '°' });
      }
    };
    sec({
      id: 'c2-dth', group: 'lecture', ref: '슬라이드 7–9', title: '2.3 링크 오프셋 d와 관절각 θ',
      view: { pos: [2.6, -3.2, 2.6], target: [0, 0, 0.4] }, grid: [2.5, 0.5], scale: 0.6, fsize: 1,
      build() {
        return {
          init(S) { fr(S, 'W', I(), { hidden: true }); scene(S, v0.d, v0.th, 1); },
          steps: [
            {
              t: '이웃 링크와의 연결: 축 i 위에서 잰다',
              tex: R`\text{link } i-1 \;\xrightarrow{\ \text{joint } i\ }\; \text{link } i`,
              html: R`<p>이번에는 관절 축 \(i\) 하나(보라)를 봅니다. 링크 \(i-1\)의 공통 수선 \(a_{i-1}\)(빨강)이 이 축에 들어오고, 링크 \(i\)의 공통 수선 \(a_i\)가 이 축에서 나갑니다. 두 수선이 축 위에서 <b>얼마나 떨어져 있는지</b>와 <b>얼마나 돌아가 있는지</b>가 나머지 두 파라미터입니다.</p>`
            },
            {
              t: '링크 오프셋 d: 축을 따라 잰 거리',
              assume: [R`d는 부호가 있는 거리 (\(\Zh_i\) 방향이 +)`],
              do(S) { scene(S, v0.d, v0.th, 2); },
              tex: R`d_i = \text{distance from } \Xh_{i-1} \text{ to } \Xh_i \text{ measured along } \Zh_i`,
              html: R`<p>\(a_{i-1}\)이 축 \(i\)와 만나는 점에서 \(a_i\)가 만나는 점까지, 축 \(i\)를 따라 잰 부호 있는 거리입니다.</p>`
            },
            {
              t: '관절각 θ: 축을 중심으로 잰 각도',
              do(S) { scene(S, v0.d, v0.th, 3); },
              tex: R`\theta_i = \text{angle from } \Xh_{i-1} \text{ to } \Xh_i \text{ measured about } \Zh_i`,
              html: R`<p>\(\Xh_{i-1}\)(=\(a_{i-1}\) 방향)을 축 위로 옮겨 연장한 선에서 \(\Xh_i\)까지 \(\Zh_i\) 둘레로 잰 각입니다. d와 θ는 축 \(i\)에서 재므로 첨자가 \(i\)입니다.</p>
              <p>정리: 링크 \(i-1\)은 \(a_{i-1}, \alpha_{i-1}\)(링크 자체 모양), \(d_i, \theta_i\)(다음 링크와의 연결)를 가집니다.</p>`
            },
            {
              t: '직접 바꿔 보기: 관절 변수는 둘 중 하나',
              live: {
                params: [
                  { id: 'd', label: R`\(d_i\)`, min: -0.8, max: 1.6, step: 0.02, val: v0.d, dp: 2 },
                  { id: 'th', label: R`\(\theta_i\)`, min: -180, max: 180, step: 1, val: v0.th, unit: '°' }
                ],
                drag: { th: { frame: 'Fi', type: 'rot' }, d: { frame: 'Fi', type: 'lin' } },
                apply(S, p) { drop(S, 'arcs'); drop(S, 'vecs'); drop(S, 'lines'); drop(S, 'bodies'); scene(S, p.d, p.th, 3); },
                tex(p) { return R`d_i = ${f(p.d, 2)},\quad \theta_i = ${f(p.th, 0)}^\circ\qquad \begin{cases}\text{revolute: } \theta_i\ \text{변수},\ d_i\ \text{고정}\\ \text{prismatic: } d_i\ \text{변수},\ \theta_i\ \text{고정}\end{cases}`; }
              },
              html: R`<p>링크 \(i\)(분홍 막대)를 잡고 돌리면 θ, 세로 기둥을 잡고 위아래로 끌면 d가 바뀝니다. 회전 관절이면 θ만, 직동 관절이면 d만 움직이고 나머지 세 값은 상수입니다(슬라이드 9 표).</p>`
            }
          ]
        };
      }
    });
  }

  /* ================= shared spatial 3R arm (2.4, 2.6) ================= */
  const arm3 = [
    { al: 0, a: 0, d: 0, th: 0, type: 'R' },
    { al: -90, a: 0.35, d: 0.25, th: 0, type: 'R' },
    { al: 0, a: 1.0, d: 0, th: 0, type: 'R' }
  ];
  const arm3Base = tr(0, 0, 0.8), arm3Tool = 0.75;
  const arm3Draw = (S, q, o) => robot(S, arm3, q, Object.assign({ base: arm3Base, tool: arm3Tool, pedestal: [0.14, 0.8], fs: 0.38 }, o || {}));
  const arm3View = { pos: [3.1, -4.4, 3.3], target: [0.8, 0, 0.9] };

  /* ================= 2.4 frame rules ================= */
  sec({
    id: 'c2-rules', group: 'lecture', ref: '슬라이드 10–11', title: '2.4 링크 좌표계 붙이는 규칙',
    view: arm3View, grid: [2.5, 0.5], scale: 0.55, fsize: 1,
    build() {
      const q = [25, -35, 55];
      const axes = S => [1, 2, 3].forEach(i => ln(S, 'ax' + i, { f: 'J' + i, p: [0, 0, -0.55] }, { f: 'J' + i, p: [0, 0, 0.55 + (arm3[i - 1].d || 0)] }, { c: 'k', dash: false, label: '축 ' + i, lt: 1, lo: [0, 0, 0.08] }));
      const perps = S => {
        vec(S, 'a1', { f: 'F1' }, { f: 'F1', p: [arm3[1].a, 0, 0] }, { c: 'acc', w: 0.8, label: 'a<sub>1</sub>', lo: [0, 0, 0.12] });
        vec(S, 'a2', { f: 'F2' }, { f: 'F2', p: [arm3[2].a, 0, 0] }, { c: 'acc', w: 0.8, label: 'a<sub>2</sub>', lo: [0, 0, 0.12] });
        vec(S, 'd2', { f: 'J2' }, { f: 'F2' }, { c: 'acc', w: 0.8, label: 'd<sub>2</sub>', lo: [0.12, 0, 0] });
      };
      const base = (S, hl, hide) => { arm3Draw(S, q, { hide: hide || [0, 1, 2, 3], frameOpt: { 1: { hl }, 2: { hl }, 3: { hl } } }); fr(S, 'W', I(), { hidden: true }); };
      return {
        init(S) { base(S, null); },
        steps: [
          {
            t: '① 관절 축을 직선으로 본다',
            do(S) { axes(S); },
            tex: R`\text{joint axis } i \;=\; \text{a line in space}`,
            html: R`<p>예시로 회전 관절 3개짜리 팔을 씁니다. 먼저 각 관절의 회전축을 무한히 긴 직선으로 생각합니다(보라 선). 축 1은 수직, 축 2와 3은 서로 평행한 수평축입니다.</p>`
          },
          {
            t: '② 이웃한 축 사이의 공통 수선',
            do(S) { perps(S); },
            tex: R`a_1:\ \text{axis }1 \perp \text{axis }2,\qquad a_2:\ \text{axis }2 \parallel \text{axis }3`,
            html: R`<p>축 1–2 사이, 축 2–3 사이의 공통 수선을 찾습니다(주황). 이 수선들이 프레임의 X축과 원점을 정해 줍니다. 축 2 위에서 두 수선 \(a_1\)과 \(a_2\)가 만나는 위치가 \(d_2\)만큼 떨어져 있습니다.</p>`
          },
          {
            t: '③ Ẑᵢ = 관절 축 i, 원점 = aᵢ 수선이 축 i와 만나는 점',
            do(S) { base(S, 'z', [0]); axes(S); perps(S); },
            tex: R`\Zh_i \parallel \text{axis } i,\qquad O_i = \text{axis } i\ \cap\ a_i`,
            html: R`<p>각 프레임 \(\{i\}\)의 Z축(파랑)을 관절 축 \(i\)에 맞추고, 원점은 <b>다음</b> 공통 수선 \(a_i\)가 축 \(i\)와 만나는 점에 둡니다. 마지막 링크는 다음 축이 없으므로 아래 ⑥의 규칙을 씁니다.</p>`
          },
          {
            t: '④ X̂ᵢ = aᵢ 방향 (aᵢ = 0이면 Zᵢ와 Zᵢ₊₁에 수직)',
            do(S) { base(S, 'xz', [0]); axes(S); perps(S); },
            tex: R`\Xh_i \parallel a_i\ (\text{from axis } i \text{ toward axis } i+1),\qquad a_i = 0 \Rightarrow \Xh_i \perp (\Zh_i,\Zh_{i+1}) \text{ plane}`,
            html: R`<p>X축(빨강)은 공통 수선을 따라 축 \(i\)에서 축 \(i+1\) 쪽을 향합니다. 두 축이 만나서 \(a_i = 0\)이면 두 Z축이 만드는 평면의 법선 방향으로 잡습니다.</p>`
          },
          {
            t: '⑤ Ŷᵢ는 오른손 법칙으로 완성',
            do(S) { base(S, null, [0]); axes(S); perps(S); },
            tex: R`\Yh_i = \Zh_i \times \Xh_i`,
            html: R`<p>Z와 X가 정해지면 Y는 자동으로 정해집니다. 이것으로 중간 링크들의 프레임이 모두 붙었습니다.</p>`
          },
          {
            t: '⑥ 첫 프레임 {0}과 마지막 프레임 {N}',
            do(S) { base(S, null, []); axes(S); perps(S); arm3Draw(S, [0, q[1], q[2]], { pre: 'g', op: 0.22, noDrag: true, hide: [0, 2, 3], frameOpt: { 1: { ghost: true, op: 0.6, name: '{1} (θ₁=0)' } } }); },
            assume: [R`\(\theta_1 = 0\)이면 {1}이 {0}과 겹친다 (흐린 팔과 프레임)`, R`그래서 \(a_0 = 0,\ \alpha_0 = 0,\ d_1 = 0\)`],
            tex: R`\{0\} \equiv \{1\}\ \text{when } q_1 = 0\ \Rightarrow\ a_0 = 0,\ \alpha_0 = 0,\ (d_1 = 0)\qquad \{N\}:\ \Xh_N \parallel \Xh_{N-1}\ \text{when } \theta_N = 0,\ d_N = 0`,
            html: R`<p><b>{0}</b>은 바닥에 고정되며 위치는 자유입니다. 계산이 쉬워지도록 \(\theta_1 = 0\)일 때 {1}과 겹치게 잡으면 \(a_0 = \alpha_0 = 0\), \(d_1 = 0\)이 됩니다(지금 {0}과 {1}이 같은 원점에 있고 θ₁만큼 돌아가 있음).</p>
            <p><b>{N}</b>(마지막): 회전 관절이면 \(\theta_N = 0\)일 때 \(\Xh_N\)이 \(\Xh_{N-1}\)과 나란하고 \(d_N = 0\)이 되게, 직동 관절이면 \(\theta_N = 0\)이 되게 잡습니다. 그래서 {3}은 축 3 위에 있고, 그 뒤의 손끝(tool)까지는 별도 변환으로 처리합니다.</p>`
          },
          {
            t: '⑦ 프레임에서 D-H 표 읽기',
            live: {
              params: [1, 2, 3].map(i => ({ id: 'q' + i, label: R`\(\theta_${i}\)`, min: -180, max: 180, step: 1, val: q[i - 1], unit: '°' })),
              drag: dragSpec(arm3),
              apply(S, p) { drop(S, 'vecs'); drop(S, 'lines'); arm3Draw(S, [p.q1, p.q2, p.q3], { hide: [], joint: act(p) ? [act(p)] : [] }); axes(S); perps(S); },
              tex(p) { return dhTable(arm3, { q: [p.q1, p.q2, p.q3], active: act(p) }) + R`\qquad \text{(}a,\,d\text{ in m)}`; }
            },
            html: R`<p>프레임이 붙으면 표는 그대로 읽힙니다. 예: 2행은 \(\Xh_1\) 둘레로 \(\Zh_1\)에서 \(\Zh_2\)까지 −90°(\(\alpha_1\)), \(\Xh_1\)을 따라 0.35(\(a_1\)), \(\Zh_2\)를 따라 0.25(\(d_2\)).</p>
            <p>링크를 끌어 관절을 돌려 보세요. 표에서 <span class="ta">θ 열만</span> 바뀌고 나머지는 상수입니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= 2.5 neighbouring transform ================= */
  {
    const v0 = { al: 40, a: 1.3, th: 50, d: 0.7 };
    const frames = (S, p, lv, ghost) => {
      const T1 = rx(p.al), T2 = mul(T1, tr(p.a, 0, 0)), T3 = mul(T2, rz(p.th)), T4 = mul(T3, tr(0, 0, p.d));
      fr(S, 'Fm', I(), { label: 'i−1', name: '{i−1}', size: 0.6 });
      ln(S, 'axm', [0, 0, -0.6], [0, 0, 1.2], { c: 'z', dash: false });
      const g = ghost ? { ghost: true, op: 0.4 } : {};
      if (lv >= 1) fr(S, 'R', T1, Object.assign({ label: 'R', size: 0.45 }, g));
      if (lv >= 2) { fr(S, 'Q', T2, Object.assign({ label: 'Q', size: 0.45 }, g)); vec(S, 'av', [0, 0, 0], [p.a, 0, 0], { c: 'acc', w: 0.7, label: 'a<sub>i−1</sub>', lo: [0, 0, -0.15] }); }
      if (lv >= 3) fr(S, 'P', T3, Object.assign({ label: 'P', size: 0.45 }, g));
      if (lv >= 4) {
        fr(S, 'Fi', T4, { label: 'i', size: 0.6 });
        ln(S, 'axi', { f: 'Fi', p: [0, 0, -1.1 - p.d] }, { f: 'Fi', p: [0, 0, 0.7] }, { c: 'k', dash: false });
        if (Math.abs(p.d) > 1e-3) vec(S, 'dv', { f: 'P' }, { f: 'Fi' }, { c: 'acc', w: 0.7, label: 'd<sub>i</sub>', lo: [0.12, 0, 0] });
      }
      if (lv >= 1 && Math.abs(p.al) > 0.5) arc(S, 'al', { c: [0.25, 0, 0], n: X, u: Z, ang: p.al, r: 0.5, label: 'α' });
      if (lv >= 3 && Math.abs(p.th) > 0.5) arc(S, 'th', { f: 'Q', c: [0, 0, 0], n: Z, u: X, ang: p.th, r: 0.45, label: 'θ' });
      return T4;
    };
    const Tn = p => dhT(p.al, p.a, p.th, p.d);
    sec({
      id: 'c2-link', group: 'lecture', ref: '슬라이드 13', title: '2.5 이웃 링크 변환 ⁱ⁻¹ᵢT',
      view: { pos: [3.0, -3.4, 2.6], target: [0.8, 0.2, 0.5] }, grid: [2.5, 0.5], scale: 0.6, fsize: 1,
      build() {
        const p = v0;
        return {
          init(S) { frames(S, p, 0); },
          steps: [
            {
              t: '목표: {i−1}에서 {i}까지',
              tex: R`{}^{i-1}P = {}^{i-1}_{R}T\ {}^{R}_{Q}T\ {}^{Q}_{P}T\ {}^{P}_{i}T\ {}^{i}P = {}^{i-1}_{\ \ i}T\ {}^{i}P`,
              html: R`<p>D-H 파라미터 4개를 하나씩 적용하는 중간 프레임 {R}, {Q}, {P}를 거쳐 {i}에 도착합니다. 각 단계는 한 축에 대한 회전 또는 이동 하나뿐입니다. 이번 예의 값: \(\alpha_{i-1}=40^\circ,\ a_{i-1}=1.3,\ \theta_i=50^\circ,\ d_i=0.7\).</p>`
            },
            {
              t: '① {R}: X̂ᵢ₋₁ 둘레로 α 회전',
              pre(S) { fr(S, 'R', I(), { label: 'R', size: 0.45 }); },
              do(S) { frames(S, p, 1); },
              motion: [{ f: 'R', rot: [X, p.al], about: 'local' }],
              tex: R`{}^{i-1}_{R}T = R_X(\alpha_{i-1})`,
              html: R`<p>Z축이 다음 관절 축과 같은 방향을 향하도록 비틉니다.</p>`
            },
            {
              t: '② {Q}: X̂_R 방향으로 a 이동',
              pre(S) { fr(S, 'Q', rx(p.al), { label: 'Q', size: 0.45 }); },
              do(S) { frames(S, p, 2); },
              motion: [{ f: 'Q', tr: [p.a, 0, 0], about: 'local' }],
              tex: R`{}^{i-1}_{Q}T = R_X(\alpha_{i-1})\,D_X(a_{i-1})`,
              html: R`<p>공통 수선을 따라 다음 축까지 갑니다. 이제 {Q}의 Z축은 관절 축 \(i\) 위에 있습니다. 이동 프레임 기준이므로 오른쪽에 곱합니다(1.8절).</p>`
            },
            {
              t: '③ {P}: Ẑ_Q 둘레로 θ 회전',
              pre(S) { fr(S, 'P', mul(rx(p.al), tr(p.a, 0, 0)), { label: 'P', size: 0.45 }); },
              do(S) { frames(S, p, 3); },
              motion: [{ f: 'P', rot: [Z, p.th], about: 'local' }],
              tex: R`{}^{i-1}_{P}T = R_X(\alpha_{i-1})\,D_X(a_{i-1})\,R_Z(\theta_i)`,
              html: R`<p>X축을 다음 공통 수선 방향 \(\Xh_i\)로 돌립니다. 회전 관절이면 이 θ가 관절 변수입니다.</p>`
            },
            {
              t: '④ {i}: Ẑ_P 방향으로 d 이동',
              pre(S) { fr(S, 'Fi', mul(rx(p.al), tr(p.a, 0, 0), rz(p.th)), { label: 'i', size: 0.6 }); },
              do(S) { frames(S, p, 4); watch(S, 'Fi', 'Fm', '실시간 \\({}^{i-1}_{\\ \\ i}T\\)'); },
              motion: [{ f: 'Fi', tr: [0, 0, p.d], about: 'local' }],
              tex: R`{}^{i-1}_{\ \ i}T = R_X(\alpha_{i-1})\,D_X(a_{i-1})\,R_Z(\theta_i)\,D_Z(d_i)`,
              html: R`<p>축을 따라 \(d_i\)만큼 올라가면 {i}입니다. 네 행렬의 순서는 파라미터를 적용한 순서 그대로입니다.</p>`
            },
            {
              t: '⑤-1 R_X(α) D_X(a) 곱하기',
              do(S) { frames(S, p, 4, true); S.frames.Q.ghost = false; S.frames.Q.op = 1; },
              tex: R`R_X(\alpha)D_X(a) = \begin{bmatrix}1&0&0&0\\0&c\alpha&-s\alpha&0\\0&s\alpha&c\alpha&0\\0&0&0&1\end{bmatrix}\begin{bmatrix}1&0&0&a\\0&1&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix} = \begin{bmatrix}1&0&0&\hx{a}\\0&c\alpha&-s\alpha&0\\0&s\alpha&c\alpha&0\\0&0&0&1\end{bmatrix} = {}^{i-1}_{\ \ Q}T`,
              html: R`<p>먼저 앞의 두 개. 이동 \(a\)가 X 방향이고 \(R_X\)는 X축을 바꾸지 않으므로 4열이 그대로 \((a, 0, 0)\)입니다. 결과가 {Q}(진하게 표시)의 위치·자세입니다.</p>`
            },
            {
              t: '⑤-2 R_Z(θ) 곱하기',
              do(S) { frames(S, p, 4, true); S.frames.P.ghost = false; S.frames.P.op = 1; },
              tex: R`{}^{i-1}_{\ \ Q}T\,R_Z(\theta) = \begin{bmatrix}1&0&0&a\\0&c\alpha&-s\alpha&0\\0&s\alpha&c\alpha&0\\0&0&0&1\end{bmatrix}\begin{bmatrix}c\theta&-s\theta&0&0\\s\theta&c\theta&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix} = \begin{bmatrix}c\theta & -s\theta & 0 & a\\ s\theta c\alpha & c\theta c\alpha & -s\alpha & 0\\ s\theta s\alpha & c\theta s\alpha & c\alpha & 0\\ 0&0&0&1\end{bmatrix}`,
              html: R`<p>2행 1열 예: \((0)(c\theta) + (c\alpha)(s\theta) + (-s\alpha)(0) + 0 = s\theta c\alpha\). 회전만 곱했으므로 4열(위치)은 변하지 않습니다. 결과가 {P}입니다.</p>`
            },
            {
              t: '⑤-3 D_Z(d) 곱하기 → 슬라이드 13의 식',
              do(S) { frames(S, p, 4, true); },
              tex: R`\begin{aligned}&\begin{bmatrix}c\theta & -s\theta & 0 & a\\ s\theta c\alpha & c\theta c\alpha & -s\alpha & 0\\ s\theta s\alpha & c\theta s\alpha & c\alpha & 0\\ 0&0&0&1\end{bmatrix}\begin{bmatrix}1&0&0&0\\0&1&0&0\\0&0&1&d\\0&0&0&1\end{bmatrix}\\ &\quad = ${dhSymMat('i', 'i-1')}\\ &\quad = ${tm(Tn(p))}\end{aligned}`,
              html: R`<p>\(D_Z(d)\)를 곱하면 회전부는 그대로이고, 4열에 <b>3열 × d</b>가 더해집니다: \((0, -s\alpha, c\alpha)\,d\). 즉 α만큼 기울어진 Z축을 따라 d만큼 이동한 효과입니다. 오른쪽은 이번 예의 숫자(α = 40°, a = 1.3, θ = 50°, d = 0.7)입니다.</p>`
            },
            {
              t: '직접 바꿔 보기: 네 파라미터 → 행렬',
              live: {
                params: [
                  { id: 'al', label: R`\(\alpha_{i-1}\)`, min: -180, max: 180, step: 1, val: v0.al, unit: '°' },
                  { id: 'a', label: R`\(a_{i-1}\)`, min: 0, max: 2, step: 0.05, val: v0.a, dp: 2 },
                  { id: 'th', label: R`\(\theta_i\)`, min: -180, max: 180, step: 1, val: v0.th, unit: '°' },
                  { id: 'd', label: R`\(d_i\)`, min: -1, max: 1.5, step: 0.05, val: v0.d, dp: 2 }
                ],
                apply(S, p) { drop(S, 'vecs'); drop(S, 'arcs'); drop(S, 'lines'); frames(S, p, 4, true); watch(S, 'Fi', 'Fm', '실시간 \\({}^{i-1}_{\\ \\ i}T\\)'); },
                tex(p) {
                  const T = Tn(p), c = x => Math.cos(x * D), s = x => Math.sin(x * D);
                  return R`{}^{i-1}_{\ \ i}T = \begin{bmatrix} c${f(p.th, 0)}^\circ & -s${f(p.th, 0)}^\circ & 0 & ${f(p.a, 2)}\\ s${f(p.th, 0)}^\circ c${f(p.al, 0)}^\circ & c${f(p.th, 0)}^\circ c${f(p.al, 0)}^\circ & -s${f(p.al, 0)}^\circ & -s${f(p.al, 0)}^\circ(${f(p.d, 2)})\\ s${f(p.th, 0)}^\circ s${f(p.al, 0)}^\circ & c${f(p.th, 0)}^\circ s${f(p.al, 0)}^\circ & c${f(p.al, 0)}^\circ & c${f(p.al, 0)}^\circ(${f(p.d, 2)})\\0&0&0&1\end{bmatrix} = ${tm(T, 3, (i, j) => i < 3 ? cls(j) : null)}`;
                }
              },
              html: R`<p>슬라이더 하나를 움직일 때 행렬의 어느 칸이 바뀌는지 보세요. α만 바꾸면 2·3행만, a는 (1,4) 한 칸만, θ는 1·2열만, d는 4열의 아래 두 칸만 바뀝니다. 흐린 프레임이 중간 단계 {R}, {Q}, {P}입니다.</p>`
            }
          ]
        };
      }
    });
  }

  /* ================= 2.6 chaining ================= */
  sec({
    id: 'c2-chain', group: 'lecture', ref: '슬라이드 14', title: '2.6 변환 연결 ⁰ₙT',
    view: arm3View, grid: [2.5, 0.5], scale: 0.55, fsize: 1,
    build() {
      const q0 = [25, -35, 55];
      const Ti = (q, i) => { const r = arm3[i - 1], v = jv(r, q[i - 1]); return dhT(r.al, r.a, v.th, v.d); };
      const acc = (S, upto) => { for (let i = 1; i <= upto; i++) vec(S, 'c' + i, { f: 'F0' }, { f: 'F' + i }, { c: 'k', w: 0.6, label: '<sup>0</sup><sub>' + i + '</sub>T', lt: 0.6, lo: [0, 0, 0.1] }); };
      const draw = (S, q, upto, jsel) => { arm3Draw(S, q, { hide: [], upto: upto < 3 ? upto : null, glow: jsel ? [] : (upto >= 1 ? [upto] : []), joint: jsel || [] }); fr(S, 'W', I(), { hidden: true }); drop(S, 'vecs'); acc(S, upto); };
      return {
        init(S) { draw(S, q0, 0); },
        steps: [
          {
            t: '⁰₁T',
            do(S) { draw(S, q0, 1); watch(S, 'F1', 'F0', '실시간 \\({}^0_1T\\)'); },
            tex: R`{}^0_1T = ${tm(Ti(q0, 1))}`,
            html: R`<p>각 행의 D-H 값을 2.5절 행렬에 넣으면 이웃 프레임 사이 변환이 하나씩 나옵니다. 1행 (0, 0, 0, θ₁) → Z축 회전만 남습니다. 빛나는 부분: <b>관절 1</b>.</p><p class="note">3D 화면에서는 지금 곱한 행렬이 설명하는 부분(관절·링크)만 진하게, 그 부분은 강조색으로 빛나고 아직 곱하지 않은 부분은 흐리게 보입니다.</p>`
          },
          {
            t: '⁰₂T = ⁰₁T ¹₂T',
            do(S) { draw(S, q0, 2); watch(S, 'F2', 'F0', '실시간 \\({}^0_2T\\)'); },
            tex: R`{}^0_2T = {}^0_1T\,{}^1_2T = ${tm(Ti(q0, 1))}${tm(Ti(q0, 2))} = ${tm(mul(Ti(q0, 1), Ti(q0, 2)))}`,
            html: R`<p>{2}는 {1} 기준으로 알려져 있으므로 오른쪽에 곱합니다(1.8절). 보라 화살표가 {0}에서 각 프레임까지의 누적 변환입니다.</p><p>지금 빛나는 부분이 \({}^1_2T\)가 설명하는 것: <b>링크 1(\(a_1 = 0.35\)), 축 2 방향 오프셋(\(d_2 = 0.25\)), 관절 2</b>입니다.</p>`
          },
          {
            t: '⁰₃T = ⁰₁T ¹₂T ²₃T',
            do(S) { draw(S, q0, 3); watch(S, 'F3', 'F0', '실시간 \\({}^0_3T\\)'); },
            tex: R`{}^0_NT = {}^0_1T\ {}^1_2T\ {}^2_3T \cdots {}^{N-1}_{\ \ \ N}T,\qquad {}^0_3T = ${tm(chainT(arm3, q0).Ts[3])}`,
            html: R`<p>새로 빛나는 부분은 \({}^2_3T\)가 설명하는 <b>링크 2(\(a_2 = 1.0\))와 관절 3</b>입니다. 손끝 막대(0.75)는 {3} 뒤에 붙은 별도 변환이라 표에 없습니다.</p><p>이렇게 이어 붙인 \({}^0_NT\)는 모든 관절 변수의 함수입니다. 관절 공간(joint space)의 값 \(q\)를 넣으면 직교 공간(Cartesian space)의 위치·방향이 나옵니다.</p>`
          },
          {
            t: '직접 움직이며 곱 확인',
            live: {
              params: [1, 2, 3].map(i => ({ id: 'q' + i, label: R`\(\theta_${i}\)`, min: -180, max: 180, step: 1, val: q0[i - 1], unit: '°' })),
              drag: dragSpec(arm3),
              apply(S, p) { draw(S, [p.q1, p.q2, p.q3], 3, act(p) ? [act(p)] : []); watch(S, 'F3', 'F0', '실시간 \\({}^0_3T\\)'); },
              tex(p) {
                const q = [p.q1, p.q2, p.q3], A = act(p);
                return R`\underbrace{${hlIf(A === 1, tm(Ti(q, 1), 2))}}_{\class{cj1}{{}^0_1T}}\underbrace{${hlIf(A === 2, tm(Ti(q, 2), 2))}}_{\class{cj2}{{}^1_2T}}\underbrace{${hlIf(A === 3, tm(Ti(q, 3), 2))}}_{\class{cj3}{{}^2_3T}} = \underbrace{${tm(chainT(arm3, q).Ts[3], 3)}}_{{}^0_3T}`;
              }
            },
            html: R`<p>링크를 잡고 돌리면 해당 관절의 행렬 하나만 바뀌고(<span class="ta">강조색</span>, 3D에서는 그 관절과 그 관절이 돌리는 링크가 빛남), 그 변화가 곱을 통해 \({}^0_3T\)에 전달됩니다. 관절 1을 돌리면 그 뒤의 모든 프레임이 같이 도는 이유입니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= Example: planar 3R (slides 12a, 15) ================= */
  {
    const L1 = 1.2, L2 = 1.0, L3 = 0.5;
    const rows = [{ al: 0, a: 0, d: 0, th: 0, type: 'R' }, { al: 0, a: L1, d: 0, th: 0, type: 'R' }, { al: 0, a: L2, d: 0, th: 0, type: 'R' }];
    const base = rx(90);
    const draw = (S, q, o) => { robot(S, rows, q, Object.assign({ base, tool: L3, fs: 0.42, w: 0.08, jr: 0.1, jh: 0.18 }, o || {})); };
    const q0 = [30, 40, -50];
    const Ai = (q, i) => dhT(0, rows[i - 1].a, q[i - 1], 0);
    const Asym = i => R`\begin{bmatrix}c_${i}&-s_${i}&0&${i === 1 ? '0' : 'L_' + (i - 1)}\\ s_${i}&c_${i}&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix}`;
    window.KIN_EX3R = { rows, base, L3 };
    sec({
      id: 'c2-ex3r', group: 'example', ref: '슬라이드 12(a), 15', title: '예제: 평면 3R 팔 (elbow manipulator)',
      view: { pos: [1.2, -5.2, 1.6], target: [1.0, 0, 1.0] }, grid: [3, 0.5], scale: 0.7, fsize: 1,
      build() {
        const sym3 = R`\begin{bmatrix} c_{123} & -s_{123} & 0 & L_1c_1 + L_2c_{12}\\ s_{123} & c_{123} & 0 & L_1s_1 + L_2s_{12}\\ 0&0&1&0\\0&0&0&1\end{bmatrix}`;
        return {
          init(S) { draw(S, q0, { upto: 0 }); },
          steps: [
            {
              t: 'D-H 표',
              do(S) { draw(S, q0); },
              tex: dhTable(rows, { sym: [{}, { a: 'L_1' }, { a: 'L_2' }] }),
              html: R`<p>세 관절 축이 모두 화면 밖을 향해 평행합니다 → 모든 \(\alpha = 0\). 축들이 한 평면 위 같은 높이에 있으니 \(d = 0\). 공통 수선은 링크 자체이므로 \(a_1 = L_1\), \(a_2 = L_2\). {0}은 θ₁ = 0일 때 {1}과 겹치게 잡아 1행은 θ₁만 남습니다.</p>
              <p class="note">\(L_3\)(손끝 길이)는 표에 없습니다. {3}은 축 3 위에 있고, 손끝은 \({}^3P = [L_3\ 0\ 0]^T\)로 따로 붙입니다.</p>`
            },
            {
              t: 'A₁ = ⁰₁T — 관절 1',
              do(S) { draw(S, q0, { upto: 1, glow: [1] }); watch(S, 'F1', 'F0', '실시간 \\({}^0_1T\\)'); },
              tex: R`A_1 = {}^0_1T = ${Asym(1)} = ${tm(Ai(q0, 1))}`,
              html: R`<p>1행 (0, 0, 0, θ₁)에서는 \(R_Z(\theta_1)\)만 남습니다. 3D 화면에서 <b>관절 1</b>이 빛나고, 아직 곱하지 않은 뒤쪽 관절들은 흐리게 보입니다.</p>`
            },
            {
              t: 'A₂ = ¹₂T — 링크 1과 관절 2',
              do(S) { draw(S, q0, { upto: 2, glow: [2] }); watch(S, 'F2', 'F1', '실시간 \\({}^1_2T\\)'); },
              tex: R`A_2 = {}^1_2T = ${Asym(2)} = ${tm(Ai(q0, 2))}`,
              html: R`<p>2행 (0, L₁, 0, θ₂): 링크 1을 따라 \(L_1\) 이동(1행 4열) 후 관절 2에서 θ₂ 회전. 지금 빛나는 것이 이 행렬이 설명하는 <b>링크 1 + 관절 2</b>입니다. 이동은 회전 <b>이전</b> 프레임의 X축을 따라 일어납니다(\(D_X(a)\)가 \(R_Z(\theta)\)보다 앞).</p>`
            },
            {
              t: 'A₃ = ²₃T — 링크 2와 관절 3',
              do(S) { draw(S, q0, { upto: 3, glow: [3] }); watch(S, 'F3', 'F2', '실시간 \\({}^2_3T\\)'); },
              tex: R`A_3 = {}^2_3T = ${Asym(3)} = ${tm(Ai(q0, 3))}`,
              html: R`<p>3행 (0, L₂, 0, θ₃): 링크 2를 따라 \(L_2\), 관절 3에서 θ₃. 이것으로 {3}까지 모두 이어졌습니다.</p>`
            },
            {
              t: 'A₁A₂ = ⁰₂T (덧셈정리)',
              do(S) { draw(S, q0, { upto: 2, glow: [1, 2] }); watch(S, 'F2', 'F0', '실시간 \\({}^0_2T\\)'); },
              tex: R`A_1A_2 = \begin{bmatrix} c_1c_2 - s_1s_2 & -c_1s_2 - s_1c_2 & 0 & L_1c_1\\ s_1c_2 + c_1s_2 & -s_1s_2 + c_1c_2 & 0 & L_1s_1\\ 0&0&1&0\\0&0&0&1\end{bmatrix} = \begin{bmatrix} c_{12} & -s_{12} & 0 & L_1c_1\\ s_{12} & c_{12} & 0 & L_1s_1\\ 0&0&1&0\\0&0&0&1\end{bmatrix}`,
              html: R`<p>\(c_1c_2 - s_1s_2 = \cos(\theta_1+\theta_2)\), \(s_1c_2 + c_1s_2 = \sin(\theta_1+\theta_2)\)(덧셈정리). 4열은 \(A_1\)의 회전 \(\times\ [L_1\ 0\ 0]^T\) = 링크 1 끝(관절 2)의 위치. 빛나는 부분: 관절 1, 링크 1, 관절 2.</p>`
            },
            {
              t: '⁰₃T = (A₁A₂)A₃',
              do(S) { draw(S, q0, { glow: [3] }); watch(S, 'F3', 'F0', '실시간 \\({}^0_3T\\)'); },
              tex: R`{}^0_3T = A_1A_2A_3 = ${sym3},\quad c_{12} = \cos(\theta_1+\theta_2),\ c_{123} = \cos(\theta_1+\theta_2+\theta_3)`,
              html: R`<p>같은 방법으로 \(c_{12}c_3 - s_{12}s_3 = c_{123}\), 4열은 \(L_1c_1 + c_{12}L_2\). 새로 빛나는 것: 링크 2와 관절 3.</p><p>축이 모두 평행하므로 회전각이 더해집니다. 손끝 방향은 \(\theta_1+\theta_2+\theta_3\), 위치는 링크 벡터 두 개의 합입니다. 손끝까지: \({}^0P_{tool} = {}^0_3T\,[L_3\ 0\ 0\ 1]^T\).</p>`
            },
            {
              t: '직접 움직여 보기',
              live: {
                params: [1, 2, 3].map(i => ({ id: 'q' + i, label: R`\(\theta_${i}\)`, min: -180, max: 180, step: 1, val: q0[i - 1], unit: '°' })),
                drag: dragSpec(rows),
                apply(S, p) {
                  draw(S, [p.q1, p.q2, p.q3], { joint: act(p) ? [act(p)] : [] }); drop(S, 'pts');
                  pt(S, 'tip', { f: 'F3', p: [L3, 0, 0] }, '손끝'); watch(S, 'F3', 'F0', '실시간 \\({}^0_3T\\)');
                },
                tex(p) {
                  const q = [p.q1, p.q2, p.q3], A = act(p), T = mul(Ai(q, 1), Ai(q, 2), Ai(q, 3)), tip = app(T, [L3, 0, 0]);
                  return R`\begin{aligned}{}^0_3T &= \underbrace{${hlIf(A === 1, tm(Ai(q, 1), 2))}}_{\class{cj1}{A_1}}\underbrace{${hlIf(A === 2, tm(Ai(q, 2), 2))}}_{\class{cj2}{A_2}}\underbrace{${hlIf(A === 3, tm(Ai(q, 3), 2))}}_{\class{cj3}{A_3}} = ${tm(T)}\\ {}^0P_{tool} &= {}^0_3T\begin{bmatrix}0.5\\0\\0\\1\end{bmatrix} = ${tv(tip)}\end{aligned}`;
                }
              },
              html: R`<p>링크를 마우스로 잡고 돌려 보세요(\(L_1 = 1.2,\ L_2 = 1.0,\ L_3 = 0.5\)). 움직이는 관절과 그 행렬(\(A_i\))이 함께 강조됩니다. 관절 3만 돌리면 \({}^0_3T\)의 4열(위치)은 그대로이고 회전 부분만 바뀝니다. {3}의 원점이 축 3 위에 있기 때문입니다.</p>`
            }
          ]
        };
      }
    });
  }

  /* ================= Practice 1: 3-DOF arm variant (HW2 P2 type) ================= */
  {
    const L1 = 0.35, L2 = 0.25, Dd = 0.05, H = 0.3;
    const rows = [{ al: 0, a: 0, d: 0, th: 0, type: 'R' }, { al: 90, a: L1, d: Dd, th: 0, type: 'R' }, { al: 0, a: L2, d: 0, th: 0, type: 'R' }];
    const base = tr(0, 0, H);
    const draw = (S, q, o) => robot(S, rows, q, Object.assign({ base, tool: 0.08, pedestal: [0.05, H], fs: 0.12, w: 0.025, jr: 0.035, jh: 0.08 }, o || {}));
    sec({
      id: 'c2-hw-3dof', group: 'practice', ref: '과제 2 · 문제 2 유형 (구조·숫자 변경)', title: '3자유도 팔: 프레임·변환·관절각',
      params: {
        orig: { t1: 40, t2: 35, t3: -50 }, origLabel: '기본 문제',
        rand() { const pick = (a, b) => a + Math.floor(Math.random() * (b - a + 1)); return { t1: pick(-150, 150), t2: pick(10, 70), t3: pick(-90, 60) }; }
      },
      grid: [0.8, 0.1], scale: 0.18, fsize: 1,
      build(P) {
        const qT = [P.t1, P.t2, P.t3];
        const Tt = chainT(rows, qT, base).Ts[3];
        const T03 = chainT(rows, qT).Ts[3];
        const Pg = p3(T03).map(x => M.round(x, 3)), Rg = R3(T03).map(r => r.map(x => M.round(x, 4)));
        const [x, y, z] = Pg;
        const s2 = z / L2, c2 = Math.sqrt(Math.max(0, 1 - s2 * s2)), t2 = atan2d(s2, c2);
        const rho = L1 + L2 * c2, t1 = atan2d(y, x) - atan2d(-Dd, rho);
        const t1n = ((t1 + 540) % 360) - 180;
        const phi = atan2d(Rg[2][0], Rg[2][1]), t3 = phi - t2;
        const tgtWorld = mul(base, fromRp(Rg, Pg));
        const t2b = 180 - t2, rhob = L1 + L2 * Math.cos(t2b * D), t1b = (((atan2d(y, x) - atan2d(-Dd, rhob)) + 540) % 360) - 180, t3b = (((phi - t2b) + 540) % 360) - 180;
        const qs = [t1n, t2, t3], qb = [t1b, t2b, t3b];
        const lim2 = { 2: [-90, 90, '가정: −90° < θ₂ < 90°'] };
        const solve = (S, known, joint, alt) => {
          draw(S, qs, { known, joint, limits: lim2 });
          if (alt) robot(S, rows, qb, { base, tool: 0.08, pre: 'x', c: 'warn', op: 0.3, noFrames: true, noDrag: true, w: 0.025, jr: 0.035, jh: 0.08, known });
        };
        return {
          calcData: { Pg, Rg, t2, rho, t1n, t3, phi },
          view: { pos: [1.25, -1.55, 1.15], target: [0.2, 0, 0.3] },
          answers: [
            { label: R`\(\theta_1\)`, val: t1n, unit: '°', tol: 0.3, dp: 2, mod: 360 },
            { label: R`\(\theta_2\)`, val: t2, unit: '°', tol: 0.3, dp: 2 },
            { label: R`\(\theta_3\)`, val: t3, unit: '°', tol: 0.3, dp: 2 }
          ],
          init(S) { fr(S, 'W', I(), { hidden: true }); draw(S, [0, 0, 0]); fr(S, 'TG', tgtWorld, { label: '', name: '목표 {3}', size: 0.12, ghost: true, op: 0.45 }); pt(S, 'tg', p3(tgtWorld), '목표 P', { c: 'k', r: 0.35 }); },
          steps: [
            {
              t: '문제',
              tex: R`L_1 = ${L1}\,\text{m},\ L_2 = ${L2}\,\text{m},\ \text{offset } D = ${Dd}\,\text{m},\qquad {}^0P_3 = ${tv(Pg)}\,\text{m},\qquad {}^0_3R = ${tm(Rg, 4)}`,
              html: R`<p>과제 2 문제 2와 같은 계열의 팔이지만, 관절 2가 링크 1 끝에서 <b>축 방향으로 D만큼 어긋나 있는</b> 변형입니다(어깨 오프셋). 관절 1 ⊥ 관절 2, 관절 2 ∥ 관절 3.</p>
              <p>1) 프레임 {0}~{3}을 붙이고 2) \({}^0_1T, {}^1_2T, {}^2_3T, {}^0_3T\)를 구하고 3) 위치 P를 만드는 \(\theta_1, \theta_2\)(단, \(-90^\circ<\theta_2<90^\circ\)) 4) 방향 R을 만드는 \(\theta_3\)을 구하시오. 흐린 좌표계와 보라 점이 목표입니다.</p>`
            },
            {
              t: '① 프레임 붙이기',
              do(S) { S.frames.F0.hl = null; },
              assume: [R`{0}은 \(\theta_1 = 0\)일 때 {1}과 겹치게 잡는다 → \(a_0 = \alpha_0 = d_1 = 0\)`, R`{3}은 축 3 위(관절 3 위치), 손끝은 별도`],
              tex: R`\Zh_0 = \Zh_1 \parallel \text{axis 1 (vertical)},\quad \Zh_2 \parallel \Zh_3 \parallel \text{axes 2, 3 (horizontal)}`,
              html: R`<p>2.4절 규칙대로: 축 1은 수직이고 {0}은 θ₁ = 0일 때 {1}과 겹치게. 축 1과 2 사이 공통 수선은 링크 1(길이 L₁)이고 \(\Xh_1\)이 그 방향입니다. 축 2는 \(\Xh_1\) 둘레로 +90° 기울어져 있고, 공통 수선 끝에서 축 2를 따라 D만큼 가서 {2}가 있습니다. 축 2와 3은 평행하고 거리 L₂.</p>
              <p class="note">화면을 돌려 작은 프레임들을 확인해 보세요. 과제 원문에서는 D = 0인 경우입니다.</p>`
            },
            {
              t: '② D-H 표',
              tex: dhTable(rows, { sym: [{}, { al: '90^\\circ', a: 'L_1', d: 'D' }, { a: 'L_2' }] }),
              html: R`<p>2행의 \(d_2 = D\)가 원래 문제와의 차이입니다.</p>`
            },
            {
              t: '③-1 ⁰₁T — 관절 1',
              do(S) { draw(S, [0, 0, 0], { upto: 1, glow: [1] }); },
              tex: R`{}^0_1T = \begin{bmatrix}c_1&-s_1&0&0\\s_1&c_1&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix}`,
              html: R`<p>1행 (0, 0, 0, θ₁) → 수직축 회전만. 빛나는 관절 1이 이 행렬이 설명하는 부분입니다.</p>`
            },
            {
              t: '③-2 ¹₂T — 링크 1, 오프셋 D, 관절 2',
              do(S) { draw(S, [0, 0, 0], { upto: 2, glow: [2] }); },
              tex: R`{}^1_2T = R_X(90^\circ)D_X(L_1)R_Z(\theta_2)D_Z(D) = \begin{bmatrix}c_2&-s_2&0&L_1\\0&0&-1&-D\\s_2&c_2&0&0\\0&0&0&1\end{bmatrix}`,
              html: R`<p>2.5절 일반식에 \(\alpha_1 = 90^\circ\)(\(c\alpha = 0,\ s\alpha = 1\))를 넣으면 2·3행이 바뀌고, 4열에 \(-s\alpha\,d = -D\)가 생깁니다. 빛나는 부분: 링크 1(L₁), 축 2 방향 오프셋(D), 관절 2.</p>`
            },
            {
              t: '③-3 ²₃T — 링크 2와 관절 3',
              do(S) { draw(S, [0, 0, 0], { upto: 3, glow: [3] }); },
              tex: R`{}^2_3T = \begin{bmatrix}c_3&-s_3&0&L_2\\s_3&c_3&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix}`,
              html: R`<p>축 2와 3이 평행(α₂ = 0)하므로 평면 팔과 같은 모양입니다.</p>`
            },
            {
              t: '④-1 ⁰₂T = ⁰₁T ¹₂T',
              do(S) { draw(S, [0, 0, 0], { upto: 2, glow: [1, 2] }); },
              tex: R`{}^0_2T = \begin{bmatrix}c_1&-s_1&0&0\\s_1&c_1&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix}\begin{bmatrix}c_2&-s_2&0&L_1\\0&0&-1&-D\\s_2&c_2&0&0\\0&0&0&1\end{bmatrix} = \begin{bmatrix}c_1c_2 & -c_1s_2 & s_1 & c_1L_1 + s_1D\\ s_1c_2 & -s_1s_2 & -c_1 & s_1L_1 - c_1D\\ s_2 & c_2 & 0 & 0\\ 0&0&0&1\end{bmatrix}`,
              html: R`<p>행 × 열로 곱합니다. 예: (1,4) = \(c_1L_1 + (-s_1)(-D) = c_1L_1 + s_1D\). 3열 \((s_1, -c_1, 0)\)은 {2}의 Z축 = 관절 2의 축 방향으로, 수평이고 θ₁에 따라 돈다는 것을 보여 줍니다. 지금 빛나는 부분이 이 두 행렬이 설명하는 관절 1, 링크 1, 오프셋 D, 관절 2입니다.</p>`
            },
            {
              t: '④-2 ⁰₃T = ⁰₂T ²₃T',
              do(S) { draw(S, [0, 0, 0], { glow: [3] }); },
              tex: R`\begin{aligned}{}^0_3T &= {}^0_2T\begin{bmatrix}c_3&-s_3&0&L_2\\s_3&c_3&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix}:\quad (1,1) = c_1c_2c_3 - c_1s_2s_3 = c_1c_{23},\quad (1,4) = c_1c_2L_2 + c_1L_1 + s_1D\\ {}^0_3T &= \begin{bmatrix} c_1c_{23} & -c_1s_{23} & s_1 & c_1(L_1 + L_2c_2) + s_1D\\ s_1c_{23} & -s_1s_{23} & -c_1 & s_1(L_1 + L_2c_2) - c_1D\\ s_{23} & c_{23} & 0 & L_2s_2\\ 0&0&0&1\end{bmatrix}\end{aligned}`,
              html: R`<p>\(c_2c_3 - s_2s_3 = \cos(\theta_2+\theta_3) = c_{23}\), \(s_2c_3 + c_2s_3 = s_{23}\)(덧셈정리). 4열은 \({}^0_2R\,[L_2\ 0\ 0]^T + {}^0P_2\)입니다. 새로 빛나는 부분: <b>링크 2와 관절 3</b>.</p>`
            },
            {
              t: '⑤ θ₂: 높이(z)에서 — 관절 2',
              do(S) { solve(S, [2], [2], true); },
              assume: [{ t: R`\(-90^\circ < \theta_2 < 90^\circ\) (즉 \(c_2 > 0\)) — 관절 2 둘레의 초록 호` }, { t: R`\(|z| \le L_2\) (그래야 \(|s_2| \le 1\))` }],
              tex: R`z = L_2s_2\ \Rightarrow\ s_2 = \frac{${f(z)}}{${L2}} = ${f(s2, 4)},\quad c_2 = \pm\sqrt{1-s_2^2} = \pm${f(c2, 4)}\ \xrightarrow{\ c_2>0\ }\ \theta_2 = \text{atan2}(${f(s2, 4)},\ ${f(c2, 4)}) = ${deg(t2)}`,
              html: R`<p>④의 (3,4) 성분 \(z = L_2s_2\)에는 θ₂만 들어 있습니다. \(s_2\)만으로는 θ₂와 180°−θ₂ 두 값이 나오고, 가정 \(c_2>0\)으로 하나를 고릅니다.</p>
              <p>3D: 진하게 보이는 것은 값이 정해진 <b>관절 2와 링크 2</b>(초록 호 = 허용 범위 안), 흐린 것은 아직 모르는 관절. 붉은 반투명 팔은 버린 해(θ₂ = ${f(t2b, 1)}°)로, 같은 높이에 닿지만 가정을 어깁니다.</p>`
            },
            {
              t: '⑥ θ₁: 오프셋 D까지 고려 — 관절 1',
              do(S) { solve(S, [1, 2], [1], false); },
              assume: [R`\(\rho = L_1 + L_2c_2 \neq 0\) 또는 \(D \neq 0\) (둘 다 0이면 θ₁ 결정 불가)`],
              tex: R`\begin{aligned}x &= c_1\rho + s_1D,\ \ y = s_1\rho - c_1D\ \Rightarrow\ \begin{bmatrix}x\\y\end{bmatrix} = \begin{bmatrix}c_1 & -s_1\\ s_1 & c_1\end{bmatrix}\begin{bmatrix}\rho\\ -D\end{bmatrix},\quad \rho = ${f(rho, 4)}\\ \theta_1 &= \text{atan2}(y,x) - \text{atan2}(-D,\rho) = ${deg(atan2d(y, x))} - (${deg(atan2d(-Dd, rho))}) = ${deg(t1n)}\end{aligned}`,
              html: R`<p>④의 x, y 식을 \(\rho = L_1 + L_2c_2\)로 묶으면, 평면 벡터 \((\rho, -D)\)를 θ₁만큼 회전한 것이 목표의 (x, y)입니다. 회전하면 방위각이 θ₁만큼 더해지므로 θ₁ = (목표 방위각) − (팔 자체 방위각). D = 0이면 바로 \(\theta_1 = \text{atan2}(y,x)\). 빛나는 것: <b>관절 1과 링크 1</b>.</p>`
            },
            {
              t: '⑦ θ₃: 방향 행렬에서 — 관절 3',
              do(S) { solve(S, [1, 2, 3], [3], false); },
              tex: R`r_{31} = s_{23},\ r_{32} = c_{23}\ \Rightarrow\ \theta_2+\theta_3 = \text{atan2}(${f(Rg[2][0], 4)},\ ${f(Rg[2][1], 4)}) = ${deg(phi)}\ \Rightarrow\ \theta_3 = ${deg(phi)} - ${deg(t2)} = ${deg(t3)}`,
              html: R`<p>④의 3행 \((s_{23}, c_{23}, 0)\)에는 \(\theta_2+\theta_3\)만 들어 있어 바로 꺼낼 수 있습니다. 검산: 3열 \((s_1, -c_1, 0)\)에서 \(\theta_1 = \text{atan2}(r_{13}, -r_{23}) = ${deg(atan2d(Rg[0][2], -Rg[1][2]))}\)도 ⑥과 같아야 합니다. 빛나는 것: <b>관절 3과 그 뒤 손끝</b>. 세 관절이 모두 정해져 팔이 목표 프레임과 겹칩니다.</p>`
            },
            {
              t: '⑧ 확인하고 직접 움직여 보기',
              live: {
                params: [
                  { id: 'q1', label: R`\(\theta_1\)`, min: -180, max: 180, step: 1, val: M.round(t1n, 1), unit: '°' },
                  { id: 'q2', label: R`\(\theta_2\)`, min: -180, max: 180, step: 1, val: M.round(t2, 1), unit: '°' },
                  { id: 'q3', label: R`\(\theta_3\)`, min: -180, max: 180, step: 1, val: M.round(t3, 1), unit: '°' }
                ],
                from: { q1: 0, q2: 0, q3: 0 },
                drag: dragSpec(rows),
                apply(S, p) { draw(S, [p.q1, p.q2, p.q3], { joint: act(p) ? [act(p)] : [], limits: lim2 }); watch(S, 'F3', 'F0', '실시간 \\({}^0_3T\\)'); },
                assume: p => [{ t: R`\(-90^\circ < \theta_2 = ${f(p.q2, 1)}^\circ < 90^\circ\)`, ok: p.q2 > -90 && p.q2 < 90 }],
                tex(p) {
                  const T = chainT(rows, [p.q1, p.q2, p.q3]).Ts[3], e = norm(sub(p3(T), Pg));
                  return R`{}^0_3T(${f(p.q1, 1)}^\circ, ${f(p.q2, 1)}^\circ, ${f(p.q3, 1)}^\circ) = ${tm(T, 4)},\qquad |{}^0P_3 - P_{goal}| = ${f(e * 1000, 1)}\,\text{mm}`;
                }
              },
              html: R`<p>구한 관절각으로 팔이 목표 프레임에 도착합니다(오차는 문제의 반올림 때문). 링크를 끌어 다른 자세를 만들어 보면, 같은 위치라도 θ₂를 180°−θ₂로 바꾼 다른 해가 있다는 것도 확인할 수 있습니다. 이 "여러 해" 문제는 Ch3 역기구학에서 다룹니다.</p>`
            }
          ]
        };
      }
    });
  }

  /* ================= Practice 2: SCARA (multi-joint D-H practice) ================= */
  {
    const L1 = 0.6, L2 = 0.5, H = 0.9;
    const rows = [{ al: 0, a: 0, d: 0, th: 0, type: 'R' }, { al: 0, a: L1, d: 0, th: 0, type: 'R' }, { al: 180, a: L2, d: 0, th: 0, type: 'P' }, { al: 0, a: 0, d: 0, th: 0, type: 'R' }];
    const base = tr(0, 0, H);
    const draw = (S, q, o) => robot(S, rows, q, Object.assign({ base, pedestal: [0.12, H], fs: 0.25, w: 0.07, jr: 0.08, jh: 0.16, toolZ: 0.12 }, o || {}));
    sec({
      id: 'c2-hw-scara', group: 'practice', ref: '과제 2 · 문제 1 유형 연습', title: 'SCARA 로봇: 4관절 D-H와 ⁰₄T',
      params: {
        orig: { t1: 30, t2: 45, d3: 0.3, t4: 20 }, origLabel: '기본 문제',
        rand() { const pick = (a, b) => a + Math.floor(Math.random() * (b - a + 1)); return { t1: pick(-120, 120), t2: pick(-130, 130), d3: pick(5, 55) / 100, t4: pick(-90, 90) }; }
      },
      view: { pos: [1.9, -2.2, 2.0], target: [0.4, 0.2, 0.6] }, grid: [1.5, 0.25], scale: 0.4, fsize: 1,
      build(P) {
        const q = [P.t1, P.t2, P.d3, P.t4];
        const T = chainT(rows, q).Ts[4];
        const phi = P.t1 + P.t2 - P.t4;
        return {
          answers: [
            { label: R`\(x\)`, val: T[0][3], tol: 0.005, dp: 4, unit: 'm' },
            { label: R`\(y\)`, val: T[1][3], tol: 0.005, dp: 4, unit: 'm' },
            { label: R`\(z\)`, val: T[2][3], tol: 0.005, dp: 4, unit: 'm' },
            { label: R`\(r_{11}\)`, val: T[0][0], tol: 0.005, dp: 4 }
          ],
          init(S) { fr(S, 'W', I(), { hidden: true }); draw(S, [0, 0, 0.1, 0]); },
          steps: [
            {
              t: '문제',
              tex: R`L_1 = ${L1},\ L_2 = ${L2}\ \text{(m)},\qquad q = (\theta_1, \theta_2, d_3, \theta_4) = (${P.t1}^\circ,\ ${P.t2}^\circ,\ ${P.d3}\,\text{m},\ ${P.t4}^\circ)`,
              html: R`<p>SCARA는 수직 회전축 2개(어깨·팔꿈치), 위아래로 움직이는 직동 관절 1개, 손목 회전 1개로 된 조립용 로봇입니다. PUMA처럼 관절 종류와 축 방향이 섞인 로봇의 D-H 표를 만드는 연습입니다.</p>
              <p>1) D-H 표 2) 네 변환행렬 3) 주어진 q에서 \({}^0_4T\)를 구하시오. {0}은 기둥 위, θ₁ = 0일 때 {1}과 겹치게 둡니다.</p>`
            },
            {
              t: '① D-H 표',
              tex: dhTable(rows, { sym: [{}, { a: 'L_1' }, { al: '180^\\circ', a: 'L_2' }, {}] }),
              html: R`<p>축 1, 2는 위를 향하는 평행축 → \(\alpha_1 = 0\), \(a_1 = L_1\). 축 3(직동)은 축 2와 평행하지만 <b>아래</b>를 향하게 잡으면 \(d_3\)가 양수일 때 손이 내려갑니다 → \(\alpha_2 = 180^\circ\). 축 4는 축 3과 같은 직선 위 → \(a_3 = 0,\ \alpha_3 = 0\). 3행은 θ가 아니라 \(d_3\)가 변수입니다.</p>`
            },
            {
              t: '②-1 ⁰₁T — 어깨 관절',
              do(S) { draw(S, [0, 0, 0.1, 0], { upto: 1, glow: [1] }); },
              tex: R`{}^0_1T = \begin{bmatrix}c_1&-s_1&0&0\\s_1&c_1&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix}`,
              html: R`<p>어깨(관절 1): 수직축 회전.</p>`
            },
            {
              t: '②-2 ¹₂T — 링크 1과 팔꿈치 관절',
              do(S) { draw(S, [0, 0, 0.1, 0], { upto: 2, glow: [2] }); },
              tex: R`{}^1_2T = \begin{bmatrix}c_2&-s_2&0&L_1\\s_2&c_2&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix}`,
              html: R`<p>링크 1(L₁) 끝의 팔꿈치(관절 2)도 수직축 회전 → 평면 팔과 같은 꼴.</p>`
            },
            {
              t: '②-3 ²₃T — 링크 2와 직동 관절',
              do(S) { draw(S, [0, 0, 0.1, 0], { upto: 3, glow: [3] }); },
              tex: R`{}^2_3T = R_X(180^\circ)D_X(L_2)D_Z(d_3) = \begin{bmatrix}1&0&0&L_2\\0&-1&0&0\\0&0&-1&-d_3\\0&0&0&1\end{bmatrix}`,
              html: R`<p>θ₃ = 0, α₂ = 180°(\(c\alpha=-1,\ s\alpha=0\))를 일반식에 넣으면 Y, Z가 뒤집히고 4열에 \(c\alpha\,d_3 = -d_3\)가 생깁니다. 빛나는 것: 링크 2와 위아래로 움직이는 축.</p>`
            },
            {
              t: '②-4 ³₄T — 손목 회전',
              do(S) { draw(S, [0, 0, 0.1, 0], { upto: 4, glow: [4] }); },
              tex: R`{}^3_4T = \begin{bmatrix}c_4&-s_4&0&0\\s_4&c_4&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix}`,
              html: R`<p>축 4는 축 3과 같은 직선 위라 \(a_3 = 0,\ \alpha_3 = 0\): Z축 회전만 남습니다.</p>`
            },
            {
              t: '③-1 ⁰₂T, ⁰₃T 차례로',
              do(S) { draw(S, [0, 0, 0.1, 0], { upto: 3, glow: [1, 2, 3] }); },
              tex: R`{}^0_2T = \begin{bmatrix}c_{12}&-s_{12}&0&L_1c_1\\s_{12}&c_{12}&0&L_1s_1\\0&0&1&0\\0&0&0&1\end{bmatrix},\quad {}^0_3T = {}^0_2T\,{}^2_3T = \begin{bmatrix}c_{12}&s_{12}&0&L_1c_1+L_2c_{12}\\s_{12}&-c_{12}&0&L_1s_1+L_2s_{12}\\0&0&-1&-d_3\\0&0&0&1\end{bmatrix}`,
              html: R`<p>처음 두 관절은 평면 팔과 같아서 \(c_{12}, s_{12}\)가 나옵니다(평면 3R 예제와 같은 덧셈정리). \({}^2_3T\)의 회전부 diag(1, −1, −1)을 곱하면 2·3열의 부호가 바뀌고, 4열에는 \({}^0_2R\,[L_2\ 0\ -d_3]^T\)가 더해집니다.</p>`
            },
            {
              t: '③-2 ⁰₄T = ⁰₃T ³₄T',
              do(S) { draw(S, [0, 0, 0.1, 0], { glow: [4] }); },
              tex: R`{}^0_4T = \begin{bmatrix} c_\phi & s_\phi & 0 & L_1c_1 + L_2c_{12}\\ s_\phi & -c_\phi & 0 & L_1s_1 + L_2s_{12}\\ 0&0&-1&-d_3\\0&0&0&1\end{bmatrix},\quad \phi = \theta_1+\theta_2-\theta_4 = ${f(phi, 0)}^\circ\quad\Rightarrow\quad ${tm(T, 4)}`,
              assume: [R`\(\alpha_2 = 180^\circ\)로 잡음 → \(d_3>0\)이면 손이 <b>아래</b>로 이동`, R`\(a_3 = 0\): 축 3과 축 4가 같은 직선`],
              html: R`<p>1행 1열: \(c_{12}c_4 + s_{12}s_4 = \cos(\theta_1+\theta_2-\theta_4) = c_\phi\). 손의 Z축은 항상 아래(−Z₀)를 향하고, 손의 수평 방향은 \(\theta_1+\theta_2-\theta_4\)로 정해집니다. θ₄의 부호가 반대인 이유는 축 4가 아래를 향하기 때문입니다.</p>`
            },
            {
              t: '직접 움직여 보기',
              live: {
                params: [
                  { id: 'q1', label: R`\(\theta_1\)`, min: -150, max: 150, step: 1, val: P.t1, unit: '°' },
                  { id: 'q2', label: R`\(\theta_2\)`, min: -150, max: 150, step: 1, val: P.t2, unit: '°' },
                  { id: 'q3', label: R`\(d_3\)`, min: 0.02, max: 0.7, step: 0.01, val: P.d3, unit: 'm', dp: 2 },
                  { id: 'q4', label: R`\(\theta_4\)`, min: -180, max: 180, step: 1, val: P.t4, unit: '°' }
                ],
                from: { q1: 0, q2: 0, q3: 0.1, q4: 0 },
                drag: dragSpec(rows),
                apply(S, p) { draw(S, [p.q1, p.q2, p.q3, p.q4], { joint: act(p) ? [act(p)] : [] }); watch(S, 'F4', 'F0', '실시간 \\({}^0_4T\\)'); },
                tex(p) {
                  const q = [p.q1, p.q2, p.q3, p.q4];
                  return dhTable(rows, { q, active: act(p), sym: [{}, { a: '0.6' }, { al: '180^\\circ', a: '0.5' }, {}] }) + R`\qquad {}^0_4T = ${tm(chainT(rows, q).Ts[4], 3)}`;
                }
              },
              html: R`<p>표의 <span class="ta">강조된 칸</span>이 관절 변수입니다. 직동 관절(손목 위 막대)을 위아래로 끌면 \(d_3\)와 4열의 z만 바뀝니다.</p>`
            }
          ]
        };
      }
    });
  }

  /* ================= Practice 3: D-H builder ================= */
  {
    const presets = {
      '평면 3R (슬라이드 15)': [{ type: 'R', al: 0, a: 0, d: 0, th: 0 }, { type: 'R', al: 0, a: 1.2, d: 0, th: 0 }, { type: 'R', al: 0, a: 1.0, d: 0, th: 0 }],
      'SCARA': [{ type: 'R', al: 0, a: 0, d: 0, th: 0 }, { type: 'R', al: 0, a: 0.6, d: 0, th: 0 }, { type: 'P', al: 180, a: 0.5, d: 0, th: 0 }, { type: 'R', al: 0, a: 0, d: 0, th: 0 }],
      '2.4절 공간 3R': arm3.map(r => Object.assign({}, r)),
      '빈 6축 표': Array.from({ length: 6 }, () => ({ type: 'R', al: 0, a: 0, d: 0, th: 0 }))
    };
    const clone = rows => rows.map(r => Object.assign({}, r));
    const initP = () => ({ rows: clone(presets['평면 3R (슬라이드 15)']), q: [30, 40, -50, 0, 0, 0], unit: 'm' });
    sec({
      id: 'c2-builder', group: 'practice', ref: '과제 2 · 문제 1 검산 도구', title: 'D-H 빌더: 표를 넣으면 로봇이 된다',
      view: { pos: [3.2, -4.2, 3.2], target: [0.6, 0, 0.5] }, grid: [3, 0.5], scale: 0.6, fsize: 1,
      build() {
        const st = {
          t: '표를 입력하고 관절을 움직여 보기',
          live: {
            params: [],
            init(p) { if (!p.rows) Object.assign(p, initP()); },
            reset(p) { Object.assign(p, initP()); },
            drag: Object.fromEntries([1, 2, 3, 4, 5, 6].map(i => ['q' + i, {
              frame: 'F' + i, type: 'rot',
              get: p => p.q[i - 1], set: (p, v) => { p.q[i - 1] = v; }
            }])),
            apply(S, p) {
              drop(S, 'bodies'); drop(S, 'frames'); drop(S, 'pts');
              const rows = p.rows.map(r => ({ type: r.type, al: +r.al || 0, a: +r.a || 0, d: +r.d || 0, th: +r.th || 0 }));
              const q = rows.map((r, i) => r.type === 'P' ? (p.q[i] = Math.max(-2, Math.min(2, p.q[i])), p.q[i]) : p.q[i]);
              robot(S, rows, q, { fs: 0.3, w: 0.05, jr: 0.07, jh: 0.16 });
              watch(S, 'F' + rows.length, 'F0', '실시간 \\({}^0_' + rows.length + 'T\\)');
              // drag type for prismatic rows
              rows.forEach((r, i) => { st.live.drag['q' + (i + 1)].type = r.type === 'P' ? 'lin' : 'rot'; });
            },
            tex(p) {
              const rows = p.rows.map(r => ({ type: r.type, al: +r.al || 0, a: +r.a || 0, d: +r.d || 0, th: +r.th || 0 }));
              const n = rows.length, T = chainT(rows, p.q.slice(0, n)).Ts[n];
              return R`{}^0_{${n}}T = ${tm(T, 3)}`;
            },
            custom(el, p, refresh) {
              const draw = () => {
                const n = p.rows.length;
                el.innerHTML = `<div class="dh-acts"><select id="dhPreset" aria-label="예시 불러오기"><option value="">예시 불러오기…</option>${Object.keys(presets).map(k => `<option>${k}</option>`).join('')}</select><button class="btn sm" id="dhAdd">행 추가</button><button class="btn sm" id="dhDel">행 삭제</button></div>
                <div style="overflow-x:auto"><table class="dh"><thead><tr><th>i</th><th>종류</th><th>α<sub>i−1</sub> °</th><th>a<sub>i−1</sub></th><th>d<sub>i</sub></th><th>θ<sub>i</sub> °</th></tr></thead><tbody>
                ${p.rows.map((r, i) => `<tr><td>${i + 1}</td><td><select data-i="${i}" data-k="type"><option ${r.type === 'R' ? 'selected' : ''}>R</option><option ${r.type === 'P' ? 'selected' : ''}>P</option></select></td>
                ${['al', 'a', 'd', 'th'].map(k => `<td><input data-i="${i}" data-k="${k}" type="number" step="${k === 'al' || k === 'th' ? 1 : 0.01}" value="${r[k]}" aria-label="${k} ${i + 1}"></td>`).join('')}</tr>`).join('')}
                </tbody></table></div>
                <p style="font-size:12px;color:var(--mute);margin:2px 0 6px">R 관절은 θ 칸이 오프셋, P 관절은 d 칸이 오프셋이고 아래 관절값이 더해집니다.</p>
                ${p.rows.map((r, i) => `<div class="lv-row"><label for="dq${i}">q<sub>${i + 1}</sub> ${r.type === 'R' ? 'θ' : 'd'}</label><input type="range" id="dq${i}" data-q="${i}" min="${r.type === 'R' ? -180 : -1}" max="${r.type === 'R' ? 180 : 1.5}" step="${r.type === 'R' ? 1 : 0.01}" value="${p.q[i]}"><input class="num" type="number" id="dqn${i}" data-qn="${i}" step="${r.type === 'R' ? 1 : 0.01}" value="${+(+p.q[i]).toFixed(2)}"><span class="u">${r.type === 'R' ? '°' : ''}</span></div>`).join('')}`;
                el.querySelector('#dhPreset').onchange = ev => { const v = ev.target.value; if (!presets[v]) return; p.rows = clone(presets[v]); p.q = [0, 0, 0, 0, 0, 0].map((x, i) => p.rows[i] && p.rows[i].type === 'P' ? 0.2 : 0); draw(); refresh(); };
                el.querySelector('#dhAdd').onclick = () => { if (p.rows.length < 6) { p.rows.push({ type: 'R', al: 0, a: 0, d: 0, th: 0 }); draw(); refresh(); } };
                el.querySelector('#dhDel').onclick = () => { if (p.rows.length > 1) { p.rows.pop(); draw(); refresh(); } };
                el.querySelectorAll('tbody select,tbody input').forEach(inp => inp.addEventListener(inp.tagName === 'SELECT' ? 'change' : 'input', ev => {
                  const i = +ev.target.dataset.i, k = ev.target.dataset.k;
                  p.rows[i][k] = k === 'type' ? ev.target.value : ev.target.value;
                  if (k === 'type') { p.q[i] = ev.target.value === 'P' ? 0.2 : 0; draw(); }
                  refresh();
                }));
                el.querySelectorAll('input[data-q]').forEach(inp => inp.addEventListener('input', ev => { p.q[+ev.target.dataset.q] = +ev.target.value; refresh(); }));
                el.querySelectorAll('input[data-qn]').forEach(inp => inp.addEventListener('change', ev => { p.q[+ev.target.dataset.qn] = +ev.target.value; refresh(); }));
              };
              draw();
            },
            onChange(p, el) {
              if (!el) return;
              p.q.forEach((v, i) => { const r = el.querySelector('#dq' + i), n = el.querySelector('#dqn' + i); if (r && document.activeElement !== r) r.value = v; if (n && document.activeElement !== n) n.value = +(+v).toFixed(2); });
            }
          },
          html: R`<p>D-H 표를 직접 입력하면 Craig 규약(\(R_X(\alpha)D_X(a)R_Z(\theta)D_Z(d)\))대로 프레임과 링크가 그려집니다. 과제 2 문제 1(PUMA 560)처럼 손으로 만든 표를 넣고, 그림 속 프레임 배치·축 방향과 일치하는지 확인하는 데 쓰세요.</p>
          <p class="note">링크(밝은 막대)는 \(a\), 세로 막대는 \(d\)를 나타냅니다. 3D 화면에서 링크를 끌어 관절을 움직일 수 있습니다. 길이 단위는 자유이지만 1~2 정도 크기가 보기 좋습니다.</p>`
        };
        return {
          init(S) { },
          steps: [st]
        };
      }
    });
  }

  window.KIN = { dhT, jv, chainT, robot, dragSpec, dhTable, act, hlIf, fr, pt, vec, ln, arc, body, drop, watch };
  window.COURSE.chapters.push({ id: 'ch2', title: 'Lecture 2 · Manipulator Forward Kinematics', short: 'Ch2 순기구학', sections });
  window.COURSE.upcoming = [{ id: 'ch3', title: 'Lecture 3 · Inverse Kinematics', short: 'Ch3 역기구학' }];
})();
