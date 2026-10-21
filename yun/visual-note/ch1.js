/* =========================================================
   Chapter 1 — Coordinate Transformation (Lecture 1, slides 3–30)
   ========================================================= */
(function () {
  'use strict';
  const R = String.raw;
  const { I, mul, rx, ry, rz, tr, rotK, inv, app, R3, p3, fromRp, RT, mv3, col, unit, dot, cross, norm, scl, add, sub, atan2d, f, tm, tv, tvr, D } = M;
  const X = [1, 0, 0], Yv = [0, 1, 0], Z = [0, 0, 1], O = [0, 0, 0];
  const R4 = T => fromRp(R3(T), [0, 0, 0]);

  /* state helpers */
  const fr = (S, id, T, o) => { S.frames[id] = Object.assign({ T, label: id }, o || {}); };
  const pt = (S, id, at, label, o) => { S.pts[id] = Object.assign({ at, label, c: 'pt' }, o || {}); };
  const vec = (S, id, a, b, o) => { S.vecs[id] = Object.assign({ a, b, c: 'pt' }, o || {}); };
  const ln = (S, id, a, b, o) => { S.lines[id] = Object.assign({ a, b, c: 'mute' }, o || {}); };
  const arc = (S, id, o) => { S.arcs[id] = Object.assign({ c: [0, 0, 0], r: 0.6 }, o); };
  const body = (S, id, fId, shape, size, pos, o) => { S.bodies[id] = Object.assign({ f: fId, shape, size, pos: pos || [0, 0, 0] }, o || {}); };
  const drop = (S, kind, pre) => { Object.keys(S[kind]).forEach(k => { if (!pre || k.indexOf(pre) === 0) delete S[kind][k]; }); };
  const watch = (S, fId, rel, title) => { S.watch = { f: fId, rel, title }; };
  const nowatch = S => { S.watch = null; };
  const deg = x => f(x, 3) + '^\\circ';
  /* stacked component arrows along three directions starting at `o` (world) */
  function comps(S, pre, o, dirs, vals, labels, w) {
    let p = o.slice();
    ['x', 'y', 'z'].forEach((c, i) => {
      const q = add(p, scl(dirs[i], vals[i]));
      if (Math.abs(vals[i]) > 1e-6) vec(S, pre + c, p, q, { c, w: w || 0.75, label: labels ? labels[i] : '', lo: [0, 0, 0.1] });
      p = q;
    });
  }
  /* dashed drop-lines from p to the three axes of the world frame */
  function feet(S, pre, p) {
    ln(S, pre + 'x', p, [p[0], 0, 0], { c: 'x' });
    ln(S, pre + 'y', p, [0, p[1], 0], { c: 'y' });
    ln(S, pre + 'z', p, [0, 0, p[2]], { c: 'z' });
  }
  const colCls = j => ['cx', 'cy', 'cz', 'ca'][j];
  /* assumption visual: allowed β range (−90°, 90°) about Y, drawn at point c */
  function betaRange(S, c, beta, r) {
    r = r || 0.75;
    const ok = beta > -90 && beta < 90;
    arc(S, 'brng', { c, n: [0, 1, 0], u: [0, 0, 1], ang: 180, r, noHead: true, col: ok ? 'ok' : 'warn', label: '가정: −90° < β < 90°' });
    vec(S, 'bcur', c, add(c, scl([Math.cos(beta * D), 0, -Math.sin(beta * D)], r * 1.08)), { c: ok ? 'ok' : 'warn', w: 0.5, label: 'β' });
  }

  const sections = [];
  const sec = o => sections.push(o);

  /* ================= 1.1 ================= */
  sec({
    id: 'c1-pos', group: 'lecture', ref: '슬라이드 3–4', title: '1.1 위치 벡터와 좌표계',
    view: { pos: [6.2, -5.6, 4.4], target: [1.1, 0.8, 0.6] }, grid: [4, 0.5], scale: 1, fsize: 1.2,
    build() {
      const p = [2.2, 1.6, 1.4];
      return {
        init(S) { fr(S, 'A', I()); },
        steps: [
          {
            t: '기준 좌표계 {A}',
            tex: R`\{A\}:\quad \hx{\Xh_A},\ \hy{\Yh_A},\ \hz{\Zh_A}\qquad (=\hat i,\ \hat j,\ \hat k)`,
            html: R`<p>위치를 숫자로 말하려면 먼저 <b>기준 좌표계(reference frame)</b>를 정해야 합니다. 좌표계 {A}는 원점 하나와 서로 수직인 <b>단위벡터(unit vector)</b> 세 개로 이루어집니다.</p>
            <p><span class="tx">빨강</span>·<span class="ty">초록</span>·<span class="tz">파랑</span> 화살표가 각각 \(\Xh_A,\Yh_A,\Zh_A\)이고 길이는 모두 1입니다. 오른손 법칙(right-hand rule): X를 Y 쪽으로 감아쥐면 엄지가 Z를 가리킵니다.</p>`
          },
          {
            t: '점 P의 위치 벡터',
            tex: R`{}^A P = \begin{bmatrix}P_x\\P_y\\P_z\end{bmatrix} = ${tv(p)}`,
            do(S) { pt(S, 'P', p, 'P'); vec(S, 'AP', O, p, { label: '<sup>A</sup>P', lt: 0.55 }); },
            html: R`<p>점 P는 {A}의 원점에서 P까지 가는 3×1 <b>위치 벡터(position vector)</b>로 표현합니다.</p>
            <p>왼쪽 위첨자(leading superscript) A는 <b>"어느 좌표계에서 잰 값인가"</b>를 뜻합니다. 같은 점이라도 기준 좌표계가 바뀌면 숫자가 바뀝니다. 이 사실이 이 장 전체의 출발점입니다.</p>`
          },
          {
            t: '성분 = 각 축 방향으로 간 거리',
            tex: R`{}^A P = \hx{P_x}\,\Xh_A + \hy{P_y}\,\Yh_A + \hz{P_z}\,\Zh_A = \hx{2.2}\,\Xh_A+\hy{1.6}\,\Yh_A+\hz{1.4}\,\Zh_A`,
            do(S) {
              comps(S, 'c', O, [X, Yv, Z], p, ['P<sub>x</sub>', 'P<sub>y</sub>', 'P<sub>z</sub>']);
              ln(S, 'g1', [p[0], p[1], 0], [0, p[1], 0]); ln(S, 'g2', [p[0], p[1], 0], [p[0], 0, 0]);
            },
            html: R`<p>원점에서 <span class="tx">X 방향 2.2</span> → <span class="ty">Y 방향 1.6</span> → <span class="tz">Z 방향 1.4</span>만큼 차례로 가면 P에 도착합니다. 세 화살표를 이어 붙인 것이 바로 벡터의 성분 분해입니다.</p>
            <p class="note">성분(component) = 그 축의 단위벡터와의 내적. 예: \(P_x = {}^AP\cdot\Xh_A\). 다음 절의 "방향 코사인"이 이 생각을 축 전체로 확장한 것입니다.</p>`
          },
          {
            t: '위치만으로는 부족하다 → 방향(orientation)',
            tex: R`\{B\}:\ \text{origin at }{}^AP,\qquad \text{orientation} = \Rm{A}{B}\ ?`,
            pre(S) { fr(S, 'B', tr(p)); },
            do(S) { drop(S, 'vecs', 'c'); drop(S, 'lines'); fr(S, 'B', mul(tr(p), rz(70), rx(-40)), { size: 0.8 }); },
            motion: [{ f: 'B', rot: [Z, 70], about: 'local' }, { f: 'B', rot: [X, -40], about: 'local' }],
            html: R`<p>로봇 손가락 끝점의 위치가 같아도, 손이 어느 쪽을 향하는지는 마음대로 바뀔 수 있습니다(지금 {B}가 제자리에서 도는 모습).</p>
            <p>그래서 물체에 좌표계 {B}를 붙이고, {B}가 {A}에 대해 어떻게 놓였는지를 <b>회전행렬(rotation matrix)</b> \(\Rm{A}{B}\)로 기술합니다. 다음 절에서 이 행렬을 만듭니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= 1.2 ================= */
  sec({
    id: 'c1-dcm', group: 'lecture', ref: '슬라이드 5', title: '1.2 회전행렬 = 방향 코사인',
    view: { pos: [2.9, -2.6, 2.1], target: [0.15, 0.15, 0.3] }, grid: [1.5, 0.25], scale: 0.55, fsize: 1,
    build() {
      const Rb = mul(rz(40), ry(-30));
      const XB = col(Rb, 0), YB = col(Rb, 1), ZB = col(Rb, 2);
      const proj = (S, v, c) => {
        drop(S, 'vecs'); drop(S, 'lines'); drop(S, 'arcs');
        comps(S, 'k', O, [X, Yv, Z], v, null, 0.6);
        feet(S, 'f', v);
        S.frames.B.hl = c;
      };
      const ang = v => Math.acos(Math.max(-1, Math.min(1, v))) / D;
      const colTex = (nm, v, c, H = '\\' + nm + 'h') => R`{}^A\h${c}{${H}_B} = \begin{bmatrix}${H}_B\cdot\hx{\Xh_A}\\ ${H}_B\cdot\hy{\Yh_A}\\ ${H}_B\cdot\hz{\Zh_A}\end{bmatrix} = \begin{bmatrix}\cos(${f(ang(v[0]), 1)}^\circ)\\ \cos(${f(ang(v[1]), 1)}^\circ)\\ \cos(${f(ang(v[2]), 1)}^\circ)\end{bmatrix} = ${tv(v)}`;
      return {
        init(S) { fr(S, 'A', I()); },
        steps: [
          {
            t: '{B}: 원점은 같고 방향만 다르다',
            assume: [R`두 좌표계의 원점이 같다 (순수 회전만)`, R`각 좌표계의 축은 길이 1이고 서로 수직, 오른손 규칙`],
            pre(S) { fr(S, 'B', I()); },
            do(S) { fr(S, 'B', R4(Rb)); watch(S, 'B', 'A', '실시간 \\({}^A_B T\\) (왼쪽 위 3×3이 R)'); },
            motion: [{ f: 'B', rot: [Z, 40], about: 'local' }, { f: 'B', rot: [Yv, -30], about: 'local' }],
            tex: R`\Rm{A}{B} = \begin{bmatrix} {}^A\hx{\Xh_B} & {}^A\hy{\Yh_B} & {}^A\hz{\Zh_B}\end{bmatrix}`,
            html: R`<p>{B}가 {A}와 원점을 공유한 채 방향만 바뀌었습니다. 이 방향을 숫자로 적는 방법이 <b>회전행렬</b>입니다.</p>
            <p>핵심 한 줄: <b>회전행렬의 각 열(column) = {B}의 축 단위벡터를 {A}에서 본 성분</b>. 아래 실시간 행렬의 왼쪽 위 3×3이 애니메이션과 함께 바뀌는 것을 보세요.</p>`
          },
          {
            t: '1열: X̂_B를 {A}의 축에 투영',
            do(S) { proj(S, XB, 'x'); },
            tex: colTex('X', XB, 'x'),
            html: R`<p>\(\Xh_B\)(진한 빨강 화살표) 끝에서 {A}의 각 축으로 수선(점선)을 내리면, 축 위에 생기는 길이가 성분입니다. 작은 <span class="tx">빨강</span>·<span class="ty">초록</span>·<span class="tz">파랑</span> 화살표 세 개를 이으면 \(\Xh_B\) 끝에 도착합니다.</p>
            <p>그 길이 = 내적(dot product) \(\Xh_B\cdot\Xh_A\) 등. 이 세 숫자가 R의 <b>첫 번째 열</b>이 됩니다.</p>`
          },
          {
            t: '내적이 왜 코사인인가',
            do(S) {
              const n = unit(cross(X, XB));
              arc(S, 'th', { n, u: X, ang: ang(XB[0]), r: 0.45, label: 'θ<sub>11</sub>=' + f(ang(XB[0]), 1) + '°', grow: true });
            },
            tex: R`\Xh_B\cdot\Xh_A = |\Xh_B|\,|\Xh_A|\cos\theta_{11} = 1\cdot 1\cdot\cos(${f(ang(XB[0]), 1)}^\circ) = ${f(XB[0])} = r_{11}`,
            html: R`<p>두 벡터 모두 길이가 1이므로 내적은 <b>두 축 사이 각도의 코사인</b>입니다. 그래서 R의 원소들을 <b>방향 코사인(direction cosines)</b>이라 부릅니다.</p>
            <p>보라색 호가 \(\Xh_A\)와 \(\Xh_B\) 사이의 각 \(\theta_{11}\)입니다. 각이 0°면 1, 90°면 0, 180°면 −1.</p>`
          },
          {
            t: '2열: Ŷ_B',
            do(S) { proj(S, YB, 'y'); },
            tex: colTex('Y', YB, 'y'),
            html: R`<p>같은 방법으로 \(\Yh_B\)를 {A}의 세 축에 투영합니다. \(\Yh_B\)는 Z 방향 성분이 0이라 수평면 위에 누워 있습니다(첫 회전이 Z축 회전이었고 두 번째 회전이 \(\Yh_B\) 자신을 축으로 했기 때문).</p>`
          },
          {
            t: '3열: Ẑ_B',
            do(S) { proj(S, ZB, 'z'); },
            tex: colTex('Z', ZB, 'z'),
            html: R`<p>\(\Zh_B\)의 성분이 세 번째 열입니다. Z 성분 \(r_{33}=${f(ZB[2])}\)은 \(\Zh_B\)가 위쪽(\(\Zh_A\))과 이루는 각 \(${f(ang(ZB[2]), 1)}^\circ\)의 코사인입니다.</p>`
          },
          {
            t: '완성된 회전행렬',
            do(S) { drop(S, 'vecs'); drop(S, 'lines'); drop(S, 'arcs'); S.frames.B.hl = null; },
            tex: R`\Rm{A}{B} = \begin{bmatrix}\hx{\Xh_B\cdot\Xh_A} & \hy{\Yh_B\cdot\Xh_A} & \hz{\Zh_B\cdot\Xh_A}\\ \hx{\Xh_B\cdot\Yh_A} & \hy{\Yh_B\cdot\Yh_A} & \hz{\Zh_B\cdot\Yh_A}\\ \hx{\Xh_B\cdot\Zh_A} & \hy{\Yh_B\cdot\Zh_A} & \hz{\Zh_B\cdot\Zh_A}\end{bmatrix} = ${tm(R3(Rb), 3, (i, j) => colCls(j))}`,
            html: R`<p>세 열을 나란히 놓으면 회전행렬이 완성됩니다. 열 방향으로 읽으면 "{B}의 축을 {A}에서 본 것", <b>행 방향으로 읽으면 "{A}의 축을 {B}에서 본 것"</b>입니다(1.4절에서 이 사실로 역행렬을 얻습니다).</p>
            <p class="note">화면을 돌려 보며 각 색 화살표의 기울기와 열의 숫자를 대응시켜 보세요.</p>`
          }
        ]
      };
    }
  });

  /* ================= 1.3 ================= */
  const R3b = mul(rz(35), rx(20));
  const BP3 = [1.4, 0.9, 0.8];
  sec({
    id: 'c1-map', group: 'lecture', ref: '슬라이드 6', title: '1.3 회전 매핑: 같은 점, 다른 좌표계',
    view: { pos: [4.6, -3.6, 3.0], target: [0.5, 0.7, 0.6] }, grid: [2.5, 0.5], scale: 0.7, fsize: 1,
    build() {
      const Rm = R3(R3b), AP = mv3(Rm, BP3);
      const RowTex = (i, nm, H = '\\' + nm + 'h') => R`P_${nm.toLowerCase()} &= ${H}\cdot{}^BP = (${H}\cdot\hat U)P_u + (${H}\cdot\hat V)P_v + (${H}\cdot\hat W)P_w`;
      return {
        init(S) {
          fr(S, 'A', I(), { sub: false, name: '{A} XYZ' });
          fr(S, 'B', R3b, { axes: ['U', 'V', 'W'], sub: false, name: '{B} UVW' });
        },
        steps: [
          {
            t: '{B}에서 알고 있는 점 P',
            assume: [R`{A}와 {B}의 원점이 같다 (평행이동 없음, 1.5절에서 일반화)`],
            do(S) {
              pt(S, 'P', { f: 'B', p: BP3 }, 'P');
              const b = [col(R3b, 0), col(R3b, 1), col(R3b, 2)];
              comps(S, 'u', O, b, BP3, ['P<sub>u</sub>', 'P<sub>v</sub>', 'P<sub>w</sub>']);
            },
            tex: R`{}^BP = \hx{P_u}\,\hat U + \hy{P_v}\,\hat V + \hz{P_w}\,\hat W = ${tv(BP3)}`,
            html: R`<p>슬라이드 6처럼 두 좌표계가 원점을 공유합니다. {A}의 축은 \(\Xh,\Yh,\Zh\), {B}의 축은 \(\hat U,\hat V,\hat W\)입니다.</p>
            <p>점 P는 {B}에서 \({}^BP=(${BP3.join(', ')})\)로 알려져 있습니다. 화면의 세 화살표는 \(\hat U\) 방향 ${BP3[0]}, \(\hat V\) 방향 ${BP3[1]}, \(\hat W\) 방향 ${BP3[2]}입니다. <b>목표: 같은 점을 {A}로 표현하기.</b></p>`
          },
          {
            t: 'X̂에 투영해 P_x 구하기',
            do(S) {
              ln(S, 'fx', AP, [AP[0], 0, 0], { c: 'x' });
              vec(S, 'ax', O, [AP[0], 0, 0], { c: 'x', w: 1.1, label: 'P<sub>x</sub>=' + f(AP[0]), lo: [0, -0.1, -0.12] });
            },
            tex: R`\begin{aligned}${RowTex(0, 'X')}\\ &= (${f(Rm[0][0])})(${BP3[0]}) + (${f(Rm[0][1])})(${BP3[1]}) + (${f(Rm[0][2])})(${BP3[2]}) = \hx{${f(AP[0])}}\end{aligned}`,
            html: R`<p>\(P_x\)는 벡터 P를 \(\Xh\) 축에 투영한 길이, 즉 \(\Xh\cdot{}^BP\)입니다. \({}^BP\)를 \(\hat U,\hat V,\hat W\) 성분으로 풀어 쓰면 내적이 세 항으로 나뉩니다.</p>
            <p>괄호 안 \(\Xh\cdot\hat U\) 같은 값은 축과 축 사이의 방향 코사인입니다. <b>P와는 상관없이 두 좌표계의 관계만으로 정해지는 숫자</b>라는 점이 중요합니다.</p>`
          },
          {
            t: 'P_y, P_z도 같은 방법',
            do(S) {
              drop(S, 'vecs', 'u');
              ln(S, 'fy', AP, [0, AP[1], 0], { c: 'y' }); ln(S, 'fz', AP, [0, 0, AP[2]], { c: 'z' });
              delete S.vecs.ax;
              comps(S, 'a', O, [X, Yv, Z], AP, ['P<sub>x</sub>', 'P<sub>y</sub>', 'P<sub>z</sub>']);
            },
            tex: R`\begin{aligned}P_x &= (\Xh\cdot\hat U)P_u + (\Xh\cdot\hat V)P_v + (\Xh\cdot\hat W)P_w = ${f(AP[0])}\\ P_y &= (\Yh\cdot\hat U)P_u + (\Yh\cdot\hat V)P_v + (\Yh\cdot\hat W)P_w = ${f(AP[1])}\\ P_z &= (\Zh\cdot\hat U)P_u + (\Zh\cdot\hat V)P_v + (\Zh\cdot\hat W)P_w = ${f(AP[2])}\end{aligned}`,
            html: R`<p>세 식이 모두 같은 모양입니다. 이번에는 같은 점 P가 {A}의 축 방향 화살표 세 개로 다시 분해되었습니다. <b>점은 그대로이고, 설명(description)만 바뀐 것</b>입니다.</p>`
          },
          {
            t: '행렬로 묶기',
            do(S) { watch(S, 'B', 'A', '실시간 \\({}^A_B T\\)'); },
            tex: R`{}^AP = \underbrace{\begin{bmatrix}\Xh\cdot\hat U & \Xh\cdot\hat V & \Xh\cdot\hat W\\ \Yh\cdot\hat U & \Yh\cdot\hat V & \Yh\cdot\hat W\\ \Zh\cdot\hat U & \Zh\cdot\hat V & \Zh\cdot\hat W\end{bmatrix}}_{\Rm{A}{B}}{}^BP = ${tm(Rm)}${tv(BP3)} = ${tv(AP)}`,
            html: R`<p>세 식의 계수를 모으면 1.2절의 방향 코사인 행렬 \(\Rm{A}{B}\)가 그대로 나옵니다. 따라서</p>
            <p style="text-align:center">\(\boxed{{}^AP = \Rm{A}{B}\,{}^BP}\)</p>
            <p class="note">아래첨자 B와 다음 위첨자 B가 "약분"되는 모양으로 기억하면 순서를 틀리지 않습니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= 1.4 ================= */
  sec({
    id: 'c1-inv', group: 'lecture', ref: '슬라이드 7', title: '1.4 회전행렬의 역 = 전치',
    view: { pos: [3.6, -3.2, 2.6], target: [0.2, 0.3, 0.3] }, grid: [2, 0.5], scale: 0.6, fsize: 1,
    build() {
      const Rm = R3(R3b), Rt = RT(Rm);
      const b = [col(R3b, 0), col(R3b, 1), col(R3b, 2)];
      return {
        init(S) { fr(S, 'A', I()); fr(S, 'B', R3b); },
        steps: [
          {
            t: '행(row) = {A}의 축을 {B}에서 본 것',
            assume: [R`R의 열(=축 단위벡터)들은 서로 수직이고 길이 1 (직교정규)`],
            do(S) {
              S.frames.A.hl = 'x';
              const xa = Rm[0]; // components of X_A along U,V,W
              comps(S, 'r', O, b, xa, null, 0.6);
              ['x', 'y', 'z'].forEach((c, i) => ln(S, 'f' + c, X, scl(b[i], xa[i]), { c }));
            },
            tex: R`\Rm{A}{B} = \begin{bmatrix} {}^B\hx{\Xh_A}^T\\ {}^B\Yh_A^T\\ {}^B\Zh_A^T\end{bmatrix},\qquad {}^B\Xh_A = \begin{bmatrix}${f(Rm[0][0])} & ${f(Rm[0][1])} & ${f(Rm[0][2])}\end{bmatrix}^T`,
            html: R`<p>방향 코사인 행렬의 1행은 \((\Xh_A\cdot\Xh_B,\ \Xh_A\cdot\Yh_B,\ \Xh_A\cdot\Zh_B)\)입니다. 이것은 <b>\(\Xh_A\)를 {B}의 축에 투영한 성분</b>, 즉 {B}에서 본 \(\Xh_A\)입니다.</p>
            <p>화면에서 \(\Xh_A\) 끝을 {B}의 세 축 방향 화살표로 분해했습니다. 이 세 길이가 R의 1행과 같습니다.</p>`
          },
          {
            t: '기준을 {B}로 바꿔서 보기',
            do(S) {
              drop(S, 'vecs'); drop(S, 'lines'); S.frames.A.hl = null;
              S.frames.A.T = fromRp(Rt, O); S.frames.B.T = I();
              watch(S, 'A', 'B', '실시간 \\({}^B_A T\\) (B 기준으로 본 A)');
            },
            motion: [{ f: ['A', 'B'], rotM: fromRp(Rt, O), about: 'world' }],
            tex: R`\Rm{B}{A} = \Rm{A}{B}^{T} = ${tm(Rt)}`,
            html: R`<p>화면 전체를 {B}가 똑바로 서도록 돌렸습니다. 이제 {B}가 기준이고 {A}가 기울어져 보입니다.</p>
            <p>{B}에서 본 {A}의 축은 R의 <b>행</b>이었으므로, 그것을 열로 세우면(=전치) \(\Rm{B}{A}\)가 됩니다. 아래 실시간 행렬이 왼쪽 식과 같은지 확인해 보세요.</p>`
          },
          {
            t: '직교정규 행렬: 역행렬 = 전치',
            tex: R`\Rm{A}{B}^{T}\,\Rm{A}{B} = \begin{bmatrix} {}^A\Xh_B^T\\ {}^A\Yh_B^T\\ {}^A\Zh_B^T\end{bmatrix}\begin{bmatrix}{}^A\Xh_B & {}^A\Yh_B & {}^A\Zh_B\end{bmatrix} = I_3\ \Rightarrow\ \Rm{A}{B}^{-1} = \Rm{A}{B}^{T},\quad {}^BP = \Rm{A}{B}^{T}\,{}^AP`,
            html: R`<p>R의 열들은 길이 1이고 서로 수직이라(직교정규, orthonormal) \(R^TR\)의 대각 원소는 1, 나머지는 0입니다. 그래서 <b>역행렬 대신 전치만 하면 됩니다</b>.</p>
            <p class="note">손 계산 팁: 3×3 역행렬 공식을 쓸 필요가 없습니다. 또한 \(\det R=+1\)이고 각 행·열 제곱합이 1이므로 계산 검산에 활용하세요.</p>`
          }
        ]
      };
    }
  });

  /* ================= Example 1.A (slide 8) ================= */
  sec({
    id: 'c1-ex8', group: 'example', ref: '슬라이드 8', title: '예제: Z축 30° 회전된 좌표계',
    view: { pos: [2.2, -3.6, 3.4], target: [-0.2, 0.9, 0] }, grid: [2.5, 0.5], scale: 0.7, fsize: 1,
    build() {
      const T = rz(30), BP = [0, 2, 0], AP = app(T, BP);
      return {
        init(S) { fr(S, 'A', I()); },
        steps: [
          {
            t: '문제',
            pre(S) { fr(S, 'B', I()); },
            do(S) { fr(S, 'B', T); },
            motion: [{ f: 'B', rot: [Z, 30], about: 'local' }],
            tex: R`\{B\} = \text{Rot}(\Zh,\,30^\circ),\qquad {}^BP = \begin{bmatrix}0 & 2 & 0\end{bmatrix}^T`,
            html: R`<p>{B}는 {A}에 대해 \(\Zh\)축으로 30° 회전한 좌표계입니다. \({}^BP=[0\ 2\ 0]^T\)일 때</p><p>1) \(\Rm{A}{B}\)를 구하고 2) \({}^AP\)를 구하시오.</p>`
          },
          {
            t: '1) 회전행렬',
            do(S) { arc(S, 'a', { n: Z, u: X, ang: 30, r: 0.7, label: '30°', grow: true }); watch(S, 'B', 'A', '실시간 \\({}^A_B T\\)'); },
            tex: R`\Rm{A}{B} = R_Z(30^\circ) = \begin{bmatrix}\hx{\cos30^\circ} & \hy{-\sin30^\circ} & \hz{0}\\ \hx{\sin30^\circ} & \hy{\cos30^\circ} & \hz{0}\\ \hx{0} & \hy{0} & \hz{1}\end{bmatrix} = ${tm(R3(T), 3, (i, j) => colCls(j))}`,
            html: R`<p>열 = {B}의 축을 {A}에서 본 것이므로 그림에서 바로 읽을 수 있습니다. \(\Xh_B\)는 X에서 30° 돌아가 \((\cos30^\circ, \sin30^\circ, 0)\), \(\Yh_B\)는 \((-\sin30^\circ, \cos30^\circ, 0)\), \(\Zh_B\)는 그대로 \((0,0,1)\).</p>`
          },
          {
            t: '2) {B}에서 본 점 P',
            do(S) { pt(S, 'P', { f: 'B', p: BP }, 'P'); vec(S, 'BP', O, { f: 'B', p: BP }, { label: '<sup>B</sup>P', lt: 0.5, lo: [0.15, 0, 0] }); },
            tex: R`{}^BP = 0\cdot\Xh_B + 2\cdot\hy{\Yh_B} + 0\cdot\Zh_B`,
            html: R`<p>\({}^BP=[0\ 2\ 0]^T\)는 "\(\Yh_B\) 방향으로 2"입니다. 화면의 주황 화살표가 \(\Yh_B\)를 따라 놓여 있습니다.</p>`
          },
          {
            t: '2) A 기준 좌표 계산',
            do(S) {
              feet(S, 'f', AP); delete S.lines.fz;
              comps(S, 'c', O, [X, Yv, Z], AP, ['−1', '1.732', '']);
            },
            tex: R`{}^AP = \Rm{A}{B}\,{}^BP = ${tm(R3(T))}\begin{bmatrix}0\\2\\0\end{bmatrix} = \begin{bmatrix}-2\sin30^\circ\\ 2\cos30^\circ\\ 0\end{bmatrix} = ${tv(AP)}`,
            html: R`<p>점 P에서 {A}의 축으로 수선을 내리면 X 방향 <b>−1</b>, Y 방향 <b>1.732</b>입니다. 행렬 곱의 결과와 그림이 일치합니다.</p>
            <p class="note">R의 2열만 2배 한 것과 같습니다: \({}^BP\)가 \(\Yh_B\) 방향뿐이기 때문입니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= 1.5 general mapping ================= */
  const R5 = mul(rz(40), rx(20)), p5 = [2.6, 1.4, 0.6], T5 = mul(tr(p5), R5), BP5 = [1.2, 0.7, 0.6];
  const RBP5 = mv3(R3(R5), BP5), AP5 = app(T5, BP5);
  const v5 = { pos: [6.0, -4.6, 4.2], target: [1.8, 1.2, 0.7] };
  function baseFrames5(S) {
    fr(S, 'A', I()); fr(S, 'B', T5);
    vec(S, 'AB', O, { f: 'B' }, { c: 'acc', label: '<sup>A</sup>P<sub>BORG</sub>', lo: [0, -0.1, -0.15] });
  }
  sec({
    id: 'c1-frame', group: 'lecture', ref: '슬라이드 9–12', title: '1.5 프레임과 일반 매핑',
    view: v5, grid: [4, 0.5], scale: 0.8, fsize: 0.9,
    build() {
      return {
        init(S) { fr(S, 'A', I()); },
        steps: [
          {
            t: '프레임 = {회전, 원점 위치}',
            pre(S) { fr(S, 'B', I()); },
            do(S) { baseFrames5(S); watch(S, 'B', 'A', '실시간 \\({}^A_B T\\)'); },
            motion: [{ f: 'B', tr: p5, about: 'world' }, { f: 'B', rotM: R5, about: 'local' }],
            tex: R`\{B\} = \{\,\Rm{A}{B},\ {}^AP_{BORG}\,\},\quad \Rm{A}{B} = ${tm(R3(R5))},\ \ {}^AP_{BORG} = ${tv(p5)}`,
            html: R`<p><b>프레임(frame)</b>은 위치와 방향을 한꺼번에 담은 것입니다. 축 방향 3개(회전행렬)와 원점 위치 1개, 합해서 벡터 4개입니다.</p>
            <p>{B}가 원점에서 출발해 \({}^AP_{BORG}\)만큼 이동한 뒤 제자리에서 회전했습니다. 보라색 화살표가 \({}^AP_{BORG}\)입니다.</p>`
          },
          {
            t: '{B}에 붙은 점 P',
            do(S) {
              pt(S, 'P', { f: 'B', p: BP5 }, 'P');
              vec(S, 'BP', { f: 'B' }, { f: 'B', p: BP5 }, { label: '<sup>B</sup>P', lo: [0.1, 0.05, 0.08] });
              const b = [col(R5, 0), col(R5, 1), col(R5, 2)];
              comps(S, 'u', p5, b, BP5, null, 0.55);
            },
            tex: R`{}^BP = ${tv(BP5)}`,
            html: R`<p>우리가 아는 것은 P를 {B}에서 잰 값 \({}^BP\)뿐입니다(작은 색 화살표 = {B}의 축 방향 성분).</p><p>원하는 것: {A}에서 본 \({}^AP\). 이번에는 원점도 다르고 방향도 달라서 1.3절의 식만으로는 안 됩니다. 두 단계로 나눕니다.</p>`
          },
          {
            t: '① 중간 프레임 {B′}: 회전만 해결',
            pre(S) { fr(S, 'Bp', T5, { name: "{B′}", sub: "B′", ghost: true }); },
            do(S) {
              fr(S, 'Bp', tr(p5), { name: "{B′}", sub: "B′", ghost: true });
              drop(S, 'vecs', 'u');
              comps(S, 'q', p5, [X, Yv, Z], RBP5, null, 0.55);
            },
            motion: [{ f: 'Bp', rotM: fromRp(RT(R3(R5)), O), about: 'local' }],
            tex: R`{}^{B'}P = \Rm{A}{B}\,{}^BP = ${tm(R3(R5))}${tv(BP5)} = ${tv(RBP5)}`,
            html: R`<p>{B}의 복사본을 제자리에서 돌려 <b>원점은 {B}, 방향은 {A}와 같은</b> 중간 프레임 {B′}를 만들었습니다(흐린 좌표계).</p>
            <p>{B}와 {B′}는 원점이 같고 방향만 다르므로 1.3절 식이 그대로 적용됩니다. 작은 화살표가 이제 {A}의 축 방향 성분입니다.</p>`
          },
          {
            t: '② 평행이동을 더한다',
            do(S) { vec(S, 'AP', O, { f: 'B', p: BP5 }, { c: 'ink', w: 0.9, label: '<sup>A</sup>P', lt: 0.6, lo: [0, 0.1, 0.12] }); drop(S, 'vecs', 'q'); },
            tex: R`{}^AP = \Rm{A}{B}\,{}^BP + {}^AP_{BORG} = ${tv(RBP5)} + ${tv(p5)} = ${tv(AP5)}`,
            html: R`<p>{B′}와 {A}는 방향이 같고 원점만 다르므로 벡터를 더하기만 하면 됩니다. 화면의 삼각형: <span class="ta">\({}^AP_{BORG}\)</span> + <span class="tp">\({}^BP\)(회전 후)</span> = \({}^AP\).</p>
            <p style="text-align:center">\(\boxed{{}^AP = \Rm{A}{B}\,{}^BP + {}^AP_{BORG}}\)</p>`
          }
        ]
      };
    }
  });

  /* ================= 1.6 homogeneous ================= */
  sec({
    id: 'c1-homo', group: 'lecture', ref: '슬라이드 13', title: '1.6 동차변환 행렬 T',
    view: v5, grid: [4, 0.5], scale: 0.8, fsize: 0.9,
    build() {
      return {
        init(S) {
          baseFrames5(S); pt(S, 'P', { f: 'B', p: BP5 }, 'P');
          vec(S, 'BP', { f: 'B' }, { f: 'B', p: BP5 }, { label: '<sup>B</sup>P', lo: [0.1, 0.05, 0.08] });
          vec(S, 'AP', O, { f: 'B', p: BP5 }, { c: 'ink', w: 0.9, label: '<sup>A</sup>P', lt: 0.6, lo: [0, 0.1, 0.12] });
        },
        steps: [
          {
            t: '덧셈까지 곱셈 하나로',
            tex: R`\begin{bmatrix}{}^AP\\ 1\end{bmatrix} = \left[\begin{array}{c|c}\Rm{A}{B} & {}^AP_{BORG}\\ \hline 0\ \ 0\ \ 0 & 1\end{array}\right]\begin{bmatrix}{}^BP\\ 1\end{bmatrix}\quad\Longleftrightarrow\quad {}^AP = \Tm{A}{B}\,{}^BP`,
            html: R`<p>1.5절 식 \(R\,{}^BP + P_{BORG}\)은 곱셈과 덧셈이 섞여 있어 여러 번 이어 쓰기 불편합니다. 벡터 끝에 <b>1</b>을 붙인 4×1 벡터를 쓰면 4×4 행렬 곱 하나로 됩니다.</p>
            <p>위 3행을 전개하면 \(R\,{}^BP + P_{BORG}\cdot 1\), 마지막 행은 \(0+1=1\). 덧붙인 1이 평행이동 열을 "켜는" 스위치 역할을 합니다.</p>`
          },
          {
            t: '숫자로 확인',
            do(S) { watch(S, 'B', 'A', '실시간 \\({}^A_B T\\)'); },
            tex: R`\Tm{A}{B} = ${tm(T5)},\qquad \Tm{A}{B}\begin{bmatrix}${BP5.join('\\\\')}\\1\end{bmatrix} = \begin{bmatrix}${AP5.map(x => f(x)).join('\\\\')}\\1\end{bmatrix}`,
            html: R`<p>1.5절에서 두 단계로 얻은 \({}^AP\)와 같은 값이 한 번의 곱으로 나옵니다.</p>`
          },
          {
            t: 'T의 열이 뜻하는 것',
            pre(S) { S.frames.B.T = I(); },
            motion: [{ f: 'B', tr: p5, about: 'world' }, { f: 'B', rotM: R5, about: 'local' }],
            tex: R`\Tm{A}{B} = ${tm(T5, 3, (i, j) => i < 3 ? colCls(j) : null)} = \begin{bmatrix}\hx{{}^A\Xh_B} & \hy{{}^A\Yh_B} & \hz{{}^A\Zh_B} & \ha{{}^AP_{BORG}}\\ 0&0&0&1\end{bmatrix}`,
            html: R`<p>T의 1~3열은 {B}의 축 방향, 4열은 {B}의 원점 위치입니다. 즉 <b>\(\Tm{A}{B}\) 하나가 곧 "{A}에서 본 프레임 {B}"의 설명</b>입니다.</p>
            <p>그래서 같은 T를 세 가지로 읽을 수 있습니다: ① 프레임의 기술(description), ② 좌표 변환(mapping, \({}^BP\to{}^AP\)), ③ 점을 옮기는 연산자(operator, 1.7절).</p>`
          }
        ]
      };
    }
  });

  /* ================= Example slide 14 ================= */
  sec({
    id: 'c1-ex14', group: 'example', ref: '슬라이드 14', title: '예제: 회전 30° + 이동 (10, 5)',
    view: { pos: [9, -11, 15], target: [6, 6, 0] }, grid: [14, 2], scale: 2.4, fsize: 2.6,
    build() {
      const T = mul(tr(10, 5, 0), rz(30)), BP = [3, 7, 0], AP = app(T, BP);
      return {
        init(S) { fr(S, 'A', I()); },
        steps: [
          {
            t: '문제',
            pre(S) { fr(S, 'B', I()); },
            do(S) { fr(S, 'B', T); vec(S, 'AB', O, { f: 'B' }, { c: 'acc', label: '<sup>A</sup>P<sub>BORG</sub>=(10,5,0)', lo: [0, -0.4, -0.3] }); },
            motion: [{ f: 'B', tr: [10, 5, 0], about: 'world' }, { f: 'B', rot: [Z, 30], about: 'local' }],
            tex: R`\{B\}:\ \text{Rot}(\Zh,30^\circ),\ \text{Trans}(10\,\Xh_A + 5\,\Yh_A),\qquad {}^BP = \begin{bmatrix}3&7&0\end{bmatrix}^T,\quad {}^AP = ?`,
            html: R`<p>{B}는 {A}에 대해 \(\Zh\)축으로 30° 회전하고, \(\Xh_A\)로 10, \(\Yh_A\)로 5만큼 이동한 프레임입니다. \({}^BP=[3\ 7\ 0]^T\)일 때 \({}^AP\)를 구하시오.</p>`
          },
          {
            t: '① 변환행렬 T 만들기',
            do(S) { watch(S, 'B', 'A', '실시간 \\({}^A_B T\\)'); arc(S, 'a', { f: 'B', n: Z, u: rz(-30).slice(0, 3).map(r => r[0]), ang: 30, r: 1.5, label: '30°', grow: true }); },
            tex: R`\Tm{A}{B} = \begin{bmatrix}\cos30^\circ & -\sin30^\circ & 0 & 10\\ \sin30^\circ & \cos30^\circ & 0 & 5\\ 0&0&1&0\\0&0&0&1\end{bmatrix} = ${tm(T)}`,
            html: R`<p>왼쪽 위 3×3은 \(R_Z(30^\circ)\), 4열은 원점 위치 (10, 5, 0)입니다.</p>`
          },
          {
            t: '② B 기준의 점 P',
            do(S) {
              drop(S, 'arcs');
              pt(S, 'P', { f: 'B', p: BP }, 'P');
              const b = [col(T, 0), col(T, 1), col(T, 2)];
              comps(S, 'u', p3(T), b, BP, ['3 X̂<sub>B</sub>', '7 Ŷ<sub>B</sub>', ''], 1.2);
            },
            tex: R`{}^BP = 3\,\hx{\Xh_B} + 7\,\hy{\Yh_B}`,
            html: R`<p>{B}의 원점에서 \(\Xh_B\) 방향 3, \(\Yh_B\) 방향 7만큼 가면 P입니다.</p>`
          },
          {
            t: '③ A 기준 좌표 계산',
            do(S) {
              drop(S, 'vecs', 'u');
              vec(S, 'BP', { f: 'B' }, { f: 'B', p: BP }, { label: '<sup>B</sup>P', w: 1.2 });
              vec(S, 'AP', O, { f: 'B', p: BP }, { c: 'ink', w: 1.2, label: '<sup>A</sup>P', lt: 0.45, lo: [-0.5, 0.3, 0.3] });
              ln(S, 'fx', AP, [AP[0], 0, 0], { c: 'x', label: f(AP[0]), lt: 1, lo: [0, -0.5, 0] });
              ln(S, 'fy', AP, [0, AP[1], 0], { c: 'y', label: f(AP[1]), lt: 1, lo: [-0.6, 0, 0] });
            },
            tex: R`{}^AP = \Tm{A}{B}\begin{bmatrix}3\\7\\0\\1\end{bmatrix} = \begin{bmatrix}3(0.866) - 7(0.5) + 10\\ 3(0.5) + 7(0.866) + 5\\ 0\\ 1\end{bmatrix} = \begin{bmatrix}${f(AP[0])}\\${f(AP[1])}\\0\\1\end{bmatrix}`,
            html: R`<p>삼각형: <span class="ta">원점 이동</span> + <span class="tp">회전된 \({}^BP\)</span> = \({}^AP\). 결과 \({}^AP = [${f(AP[0])}\ \ ${f(AP[1])}\ \ 0]^T\).</p>`
          }
        ]
      };
    }
  });

  /* ================= 1.7 operators ================= */
  sec({
    id: 'c1-op', group: 'lecture', ref: '슬라이드 15–17', title: '1.7 연산자: 점을 움직이는 행렬',
    view: { pos: [6.2, -5, 4.4], target: [1.4, 1.2, 0.8] }, grid: [4, 0.5], scale: 0.8, fsize: 1,
    build() {
      const P1 = [1.8, 0.6, 0.6], q = [1, 1.2, 0.8];
      const Pn = [1.0, 1.3, 0.5];
      const P2t = app(tr(q), P1), P2r = app(rz(60), P1), P2x = app(rx(50), Pn), Tg = mul(tr(q), rz(60)), P2g = app(Tg, P1);
      const ghost1 = S => pt(S, 'P1', P1, 'P<sub>1</sub>', { c: 'mute' });
      return {
        init(S) { fr(S, 'A', I()); fr(S, 'pf', I(), { hidden: true }); },
        steps: [
          {
            t: '매핑 vs 연산자',
            do(S) { pt(S, 'P', { f: 'pf', p: P1 }, 'P<sub>1</sub>'); vec(S, 'OP', O, { f: 'pf', p: P1 }, { w: 0.8 }); },
            tex: R`\text{mapping: } {}^AP = \Tm{A}{B}\,{}^BP\qquad\text{operator: } {}^AP_2 = T\,{}^AP_1`,
            html: R`<p>지금까지는 <b>같은 점</b>을 다른 좌표계로 다시 쓰는 매핑(mapping)이었습니다.</p>
            <p>연산자(operator)는 좌표계는 {A} 하나로 고정하고 <b>점을 실제로 옮깁니다</b>. 놀랍게도 같은 모양의 행렬을 씁니다. 해석만 다릅니다.</p>`
          },
          {
            t: '평행이동 연산자 D_Q',
            pre(S) { S.frames.pf.T = I(); },
            do(S) {
              S.frames.pf.T = tr(q); ghost1(S); S.pts.P.label = 'P<sub>2</sub>';
              vec(S, 'Q', P1, P2t, { c: 'acc', w: 0.7, label: 'Q', lo: [0, 0, 0.12] });
            },
            motion: [{ f: 'pf', tr: q, about: 'world' }],
            tex: R`D_Q(q) = \begin{bmatrix}1&0&0&q_x\\0&1&0&q_y\\0&0&1&q_z\\0&0&0&1\end{bmatrix},\quad {}^AP_2 = D_Q\,{}^AP_1 = ${tv(P1)} + ${tv(q)} = ${tv(P2t)}`,
            html: R`<p>회전 부분이 단위행렬이고 4열만 있는 T입니다. 점 \(P_1\)이 벡터 Q만큼 이동해 \(P_2\)가 됩니다.</p>`
          },
          {
            t: '회전 연산자 R_Z(θ)',
            pre(S) { S.frames.pf.T = I(); delete S.vecs.Q; },
            do(S) {
              S.frames.pf.T = rz(60); delete S.vecs.Q;
              arc(S, 'a', { c: [0, 0, P1[2]], n: Z, u: unit([P1[0], P1[1], 0]), r: Math.hypot(P1[0], P1[1]), ang: 60, label: 'θ = 60°', grow: true });
              ln(S, 'ax', [0, 0, -0.2], [0, 0, 1.6], { c: 'z', dash: false });
            },
            motion: [{ f: 'pf', rot: [Z, 60], about: 'world' }],
            tex: R`R_Z(\theta)=\begin{bmatrix}\cos\theta & -\sin\theta & 0 & 0\\ \sin\theta & \cos\theta & 0 & 0\\ 0&0&1&0\\0&0&0&1\end{bmatrix},\quad R_Z(60^\circ)${tv(P1)} = ${tv(P2r)}`,
            html: R`<p>점이 \(\Zh\)축 둘레로 60° 돌았습니다. 높이(z)는 그대로이고 xy 평면에서 원을 그립니다. 4열이 0인 것은 "이동 없음"을 뜻합니다.</p>`
          },
          {
            t: 'R_X(θ) 유도 (슬라이드 16)',
            pre(S) { S.frames.pf.T = I(); drop(S, 'arcs'); drop(S, 'lines'); delete S.pts.P; delete S.pts.P1; delete S.vecs.OP; },
            do(S) {
              S.frames.pf.T = rx(50); drop(S, 'arcs'); drop(S, 'lines'); delete S.pts.P1; delete S.pts.P; delete S.vecs.OP;
              pt(S, 'Pn0', Pn, 'p (회전 전)', { c: 'mute' });
              pt(S, 'Pn', { f: 'pf', p: Pn }, 'p (회전 후)');
              arc(S, 'a', { c: [Pn[0], 0, 0], n: X, u: unit([0, Pn[1], Pn[2]]), r: Math.hypot(Pn[1], Pn[2]), ang: 50, label: 'θ = 50°', grow: true });
              ln(S, 'ax', [-0.3, 0, 0], [2, 0, 0], { c: 'x', dash: false });
              ln(S, 'r1', [Pn[0], 0, 0], P2x, { c: 'mute' });
              ln(S, 'py', P2x, [P2x[0], 0, P2x[2]], { c: 'y' });
              ln(S, 'pz', P2x, [P2x[0], P2x[1], 0], { c: 'z' });
              arc(S, 'phi', { c: [Pn[0], 0, 0], n: X, u: Yv, ang: atan2d(Pn[2], Pn[1]), r: Math.hypot(Pn[1], Pn[2]) * 0.45, noHead: true, col: 'mute', label: 'φ' });
            },
            motion: [{ f: 'pf', rot: [X, 50], about: 'world' }],
            tex: R`\begin{aligned}&p_o = \rho\cos\phi,\ \ p_a = \rho\sin\phi\quad(\rho = \sqrt{p_o^2+p_a^2},\ \text{YZ 평면의 극좌표})\\ &p_y = \rho\cos(\phi+\theta) = \rho\cos\phi\cos\theta - \rho\sin\phi\sin\theta = p_o\cos\theta - p_a\sin\theta\\ &p_z = \rho\sin(\phi+\theta) = \rho\sin\phi\cos\theta + \rho\cos\phi\sin\theta = p_o\sin\theta + p_a\cos\theta,\qquad p_x = p_n\end{aligned}\ \Rightarrow\ \begin{bmatrix}p_x\\p_y\\p_z\end{bmatrix} = \begin{bmatrix}1&0&0\\0&\cos\theta&-\sin\theta\\0&\sin\theta&\cos\theta\end{bmatrix}\begin{bmatrix}p_n\\p_o\\p_a\end{bmatrix} = ${tv(P2x)}`,
            html: R`<p>X축 둘레로 돌리면 x 성분은 변하지 않고, (y, z)만 YZ 평면에서 반지름 ρ인 원을 따라 θ만큼 돕니다(파란 호). 회전 전 각도를 φ(회색 호)라 하면 회전 후 각도는 φ + θ이고, 덧셈정리로 풀면 위 식이 됩니다. 슬라이드 16의 \(p_n,p_o,p_a\)는 회전 전 성분 (${Pn.join(', ')})입니다.</p>
            <p>회전 후 y·z 성분(점선)을 삼각함수로 쓰면 왼쪽 식이 되고, 계수를 모은 것이 \(R_X(\theta)\)입니다. \(R_Y\)도 같은 방법으로 얻으며, 부호 위치가 다른 이유(\(Z\to X\) 순환)는 과제 문제 2와 연결됩니다.</p>`
          },
          {
            t: '회전 + 이동을 한 번에',
            pre(S) { S.frames.pf.T = I(); drop(S, 'arcs'); drop(S, 'lines'); drop(S, 'pts'); },
            do(S) {
              S.frames.pf.T = Tg; drop(S, 'arcs'); drop(S, 'lines'); drop(S, 'pts');
              ghost1(S); pt(S, 'P', { f: 'pf', p: P1 }, 'P<sub>2</sub>'); vec(S, 'OP', O, { f: 'pf', p: P1 }, { w: 0.8 });
            },
            motion: [{ f: 'pf', rot: [Z, 60], about: 'world' }, { f: 'pf', tr: q, about: 'world' }],
            tex: R`T = D_Q(q)\,R_Z(60^\circ) = ${tm(Tg)},\qquad {}^AP_2 = T\,{}^AP_1 = ${tv(P2g)}`,
            html: R`<p>먼저 Z축으로 60° 돌리고 그다음 Q만큼 옮겼습니다. 두 연산을 곱한 T 하나로 같은 결과가 나옵니다.</p>
            <p class="note">이 T를 프레임으로 해석하면 "{A}에 대해 60° 돌고 q만큼 떨어진 {B}"이기도 합니다. 행렬은 같고 해석만 다릅니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= 1.8 compound ================= */
  sec({
    id: 'c1-comp', group: 'lecture', ref: '슬라이드 18', title: '1.8 변환 합성 (이동 프레임 기준)',
    view: { pos: [6.8, -5.4, 5], target: [2.1, 1.1, 0.8] }, grid: [4, 0.5], scale: 0.8, fsize: 0.85,
    build() {
      const TAB = mul(tr(2.2, 0, 0.3), rz(50)), TBC = mul(tr(1.6, 0.4, 0.9), rx(90)), TAC = mul(TAB, TBC);
      const CP = [0.8, 0.6, 0], BPc = app(TBC, CP), APc = app(TAC, CP);
      return {
        init(S) { fr(S, 'A', I()); },
        steps: [
          {
            t: '{A}에서 본 {B}',
            pre(S) { fr(S, 'B', I()); },
            do(S) { fr(S, 'B', TAB); vec(S, 'AB', O, { f: 'B' }, { c: 'acc', label: '<sup>A</sup><sub>B</sub>T' }); watch(S, 'B', 'A', '실시간 \\({}^A_B T\\)'); },
            motion: [{ f: 'B', tr: p3(TAB), about: 'world' }, { f: 'B', rot: [Z, 50], about: 'local' }],
            tex: R`\Tm{A}{B} = ${tm(TAB)}`,
            html: R`<p>{B}는 {A}에 대해 알려져 있습니다.</p>`
          },
          {
            t: '{B}에서 본 {C} — B의 축을 따라 움직인다',
            pre(S) { fr(S, 'C', TAB); },
            do(S) { fr(S, 'C', TAC); vec(S, 'BC', { f: 'B' }, { f: 'C' }, { c: 'acc', label: '<sup>B</sup><sub>C</sub>T' }); watch(S, 'C', 'B', '실시간 \\({}^B_C T\\) (B 기준)'); },
            motion: [{ f: 'C', tr: p3(TBC), about: 'local' }, { f: 'C', rot: [X, 90], about: 'local' }],
            tex: R`\Tm{B}{C} = ${tm(TBC)}`,
            html: R`<p>{C}는 {B}에 대해서만 알려져 있습니다. {C}가 {B}의 위치에서 출발해 <b>{B}의 축 방향</b>으로 (1.6, 0.4, 0.9) 이동하고 자기 X축으로 90° 도는 것을 보세요. 이동 방향이 {A}의 축과 다릅니다.</p>`
          },
          {
            t: '점 P를 거슬러 올라가기',
            do(S) {
              pt(S, 'P', { f: 'C', p: CP }, 'P');
              vec(S, 'CP', { f: 'C' }, { f: 'C', p: CP }, { label: '<sup>C</sup>P' });
              vec(S, 'AP', O, { f: 'C', p: CP }, { c: 'ink', w: 0.8, label: '<sup>A</sup>P', lt: 0.5, lo: [0, 0.15, 0.15] });
            },
            tex: R`{}^BP = \Tm{B}{C}\,{}^CP = ${tv(BPc)},\qquad {}^AP = \Tm{A}{B}\,{}^BP = ${tv(APc)}`,
            html: R`<p>\({}^CP=(${CP.join(', ')})\)를 알 때, 먼저 {B}로 옮기고 다시 {A}로 옮깁니다. 두 번 곱한 셈입니다.</p>`
          },
          {
            t: '합성 변환',
            do(S) { watch(S, 'C', 'A', '실시간 \\({}^A_C T\\)'); vec(S, 'ACd', O, { f: 'C' }, { c: 'k', w: 0.7, label: '<sup>A</sup><sub>C</sub>T', lo: [0, -0.1, -0.15] }); },
            tex: R`\begin{aligned}{}^AP &= \Rm{A}{B}\,{}^BP + {}^AP_{BORG} = \Rm{A}{B}\big(\Rm{B}{C}\,{}^CP + {}^BP_{CORG}\big) + {}^AP_{BORG} = \underbrace{\Rm{A}{B}\Rm{B}{C}}_{\Rm{A}{C}}{}^CP + \underbrace{\Rm{A}{B}\,{}^BP_{CORG} + {}^AP_{BORG}}_{{}^AP_{CORG}}\\ \Tm{A}{C} &= \Tm{A}{B}\,\Tm{B}{C} = \begin{bmatrix}\Rm{A}{B}\Rm{B}{C} & \Rm{A}{B}\,{}^BP_{CORG} + {}^AP_{BORG}\\ 0\ 0\ 0 & 1\end{bmatrix} = ${tm(TAC)}\end{aligned}`,
            html: R`<p>\({}^AP = \Tm{A}{B}(\Tm{B}{C}\,{}^CP)\)이므로 \(\Tm{A}{C}=\Tm{A}{B}\Tm{B}{C}\)입니다.</p>
            <p><b>새 변환이 현재(움직인) 프레임 기준으로 주어지면 오른쪽에 곱합니다(post-multiply).</b> 다음 절에서 고정 프레임 기준과 비교합니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= 1.9 fixed vs moving ================= */
  sec({
    id: 'c1-order', group: 'lecture', ref: '슬라이드 19', title: '1.9 고정축 회전 vs 이동축 회전',
    view: { pos: [6.5, -2.6, 3.6], target: [0, 0, 0.4] }, grid: [3.5, 0.5], scale: 0.75, fsize: 1,
    build() {
      const pF = [0, -1.7, 0], pM = [0, 1.7, 0];
      const fixedR = mul(rz(90), ry(90)), movR = mul(ry(90), rz(90));
      const lab = (n, s) => ({ axes: ['x', 'y', 'z'], sub: s, name: n });
      return {
        init(S) {
          fr(S, 'F0', tr(pF), Object.assign(lab('', '0'), { ghost: true, label: '' }));
          fr(S, 'M0', tr(pM), Object.assign(lab('', '0'), { ghost: true, label: '' }));
          fr(S, 'F', tr(pF), lab('고정축(fixed) 기준', '1'));
          fr(S, 'M', tr(pM), lab('이동축(moving) 기준', '1'));
        },
        steps: [
          {
            t: '첫 회전: y축으로 φ = 90° (둘 다 동일)',
            assume: [R`두 좌표계 모두 기준 \(x_0y_0z_0\)과 겹친 상태에서 출발`, R`원점은 고정, 회전만 고려 (비교를 위해 옆으로 떨어뜨려 그림)`],
            do(S) { S.frames.F.T = mul(tr(pF), ry(90)); S.frames.M.T = mul(tr(pM), ry(90)); },
            motion: [{ f: 'F', rot: [Yv, 90], about: 'worldDir' }, { f: 'M', rot: [Yv, 90], about: 'local', with: true }],
            tex: R`{}^0_1R = R_{y,\phi}\qquad(\phi = 90^\circ)`,
            html: R`<p>흐린 좌표계가 기준 \(x_0y_0z_0\), 진한 좌표계가 움직이는 \(x_1y_1z_1\)입니다. 처음에는 두 축이 겹쳐 있으므로 첫 회전은 어느 쪽 해석이든 같습니다.</p>`
          },
          {
            t: '두 번째 회전: z₀축 vs 현재 z₁축',
            do(S) { S.frames.F.T = mul(tr(pF), fixedR); S.frames.M.T = mul(tr(pM), movR); watch(S, 'F', 'F0', '실시간 고정축 쪽 \\({}^0_1T\\)'); },
            motion: [{ f: 'F', rot: [Z, 90], about: 'worldDir' }, { f: 'M', rot: [Z, 90], about: 'local', with: true }],
            tex: R`\begin{aligned}\text{fixed: }&{}^0_1R = R_{z,\theta}\,R_{y,\phi} = ${tm(R3(fixedR))}\\ \text{moving: }&{}^0_1R = R_{y,\phi}\,R_{z,\theta} = ${tm(R3(movR))}\end{aligned}`,
            html: R`<p>앞쪽(고정축)은 기준 \(z_0\)축(위쪽)으로 돌았고, 뒤쪽(이동축)은 첫 회전 후 옆으로 누운 자기 \(z_1\)축으로 돌았습니다. <b>결과가 다릅니다.</b></p>
            <p>고정 프레임 기준 회전은 <b>왼쪽에 곱하기(pre-multiply)</b>, 즉 수행 순서의 <b>역순</b>으로 씁니다. 이동 프레임 기준은 오른쪽에 곱합니다(1.8절).</p>`
          },
          {
            t: '왜 역순인가 — 슬라이드 19의 증명',
            pre(S) { S.frames.M.T = mul(tr(pM), ry(90)); },
            do(S) { S.frames.M.T = mul(tr(pM), fixedR); S.frames.M.name = '되돌리고-돌리고-다시'; S.watch = null; },
            motion: [{ f: 'M', rot: [Yv, -90], about: 'local' }, { f: 'M', rot: [Z, 90], about: 'local' }, { f: 'M', rot: [Yv, 90], about: 'local' }],
            tex: R`{}^0_1R = R_{y_0,\phi}\,\big[R_{y,-\phi}\,R_{z,\theta}\,R_{y,\phi}\big] = R_{z,\theta}\,R_{y,\phi}`,
            html: R`<p>첫 회전 뒤에는 \(z_0\)축이 현재 프레임의 축이 아닙니다. 그래서 현재 축만 써서 흉내 냅니다: ① −φ로 되돌려 \(z\)를 \(z_0\)에 맞추고 ② θ만큼 돌리고 ③ 다시 φ.</p>
            <p>뒤쪽 좌표계가 이 세 동작을 거쳐 앞쪽(고정축 결과)과 같은 자세가 됩니다. 괄호 안을 정리하면 \(R_{y,\phi}R_{y,-\phi}=I\)가 소거되어 \(R_{z,\theta}R_{y,\phi}\)만 남습니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= 1.10 inverse transform ================= */
  sec({
    id: 'c1-tinv', group: 'lecture', ref: '슬라이드 20–21', title: '1.10 변환의 역행렬',
    view: { pos: [3.2, -6.2, 6.8], target: [1.6, 1.2, 0] }, grid: [6, 1], scale: 1.1, fsize: 1.2,
    build() {
      const T = mul(tr(4, 3, 0), rz(30)), Rm = R3(T), Rt = RT(Rm), p = p3(T), Ti = inv(T), q = p3(Ti);
      return {
        init(S) { fr(S, 'A', I()); },
        steps: [
          {
            t: '주어진 것과 구할 것',
            pre(S) { fr(S, 'B', I()); },
            do(S) { fr(S, 'B', T); vec(S, 'AB', O, { f: 'B' }, { c: 'acc', label: '<sup>A</sup>P<sub>BORG</sub>', lo: [0.2, -0.3, -0.2] }); },
            motion: [{ f: 'B', tr: p, about: 'world' }, { f: 'B', rot: [Z, 30], about: 'local' }],
            tex: R`\Tm{A}{B} = ${tm(T)}\quad\Longrightarrow\quad \Tm{B}{A} = \ ?`,
            html: R`<p>슬라이드 21 예제입니다. 4×4 역행렬을 가우스 소거로 구하는 대신 <b>구조</b>를 이용합니다: 회전 부분과 원점 부분을 따로 구합니다.</p>`
          },
          {
            t: '회전 부분: 전치',
            tex: R`\Rm{B}{A} = \Rm{A}{B}^{T} = ${tm(Rt)}`,
            html: R`<p>1.4절 그대로입니다. {A}의 축을 {B}에서 보면 R의 행이 됩니다.</p>`
          },
          {
            t: '원점 부분: −Rᵀ·P_BORG',
            do(S) {
              vec(S, 'BA', { f: 'B' }, O, { c: 'k', label: '<sup>B</sup>P<sub>AORG</sub>', lt: 0.45, lo: [0, 0.3, 0.2] });
              comps(S, 'c', p, [col(T, 0), col(T, 1), col(T, 2)], q, [f(q[0]), f(q[1]), ''], 1.0);
            },
            tex: R`{}^B\big({}^AP_{BORG}\big) = \Rm{B}{A}\,{}^AP_{BORG} + {}^BP_{AORG} = 0\ \Rightarrow\ {}^BP_{AORG} = -\Rm{A}{B}^{T}\,{}^AP_{BORG} = -${tm(Rt)}${tv(p)} = ${tv(q)}`,
            html: R`<p>{B}의 원점을 {B}에서 보면 당연히 0입니다. 이 사실을 1.5절 매핑 식에 넣으면 \({}^BP_{AORG}\)가 나옵니다.</p>
            <p>보라색 화살표가 {B} 원점에서 {A} 원점으로 가는 벡터이고, 작은 화살표가 그것의 <b>{B} 축 방향</b> 성분 (${f(q[0])}, ${f(q[1])}, 0)입니다. 단순히 \(-{}^AP_{BORG}=(-4,-3,0)\)이 아닌 이유: 축이 30° 돌아가 있기 때문입니다.</p>`
          },
          {
            t: '결과 (슬라이드 21과 비교)',
            tex: R`\Tm{B}{A} = \begin{bmatrix}\Rm{A}{B}^{T} & -\Rm{A}{B}^{T}\,{}^AP_{BORG}\\ 0\ 0\ 0 & 1\end{bmatrix} = ${tm(Ti)}`,
            html: R`<p>슬라이드의 답 −4.964, −0.598과 일치합니다. 회전 블록은 전치, 4열은 \(-R^Tp\). 이 공식은 과제(변환 체인)에서 계속 씁니다.</p>`
          },
          {
            t: '{B}를 기준으로 화면 다시 보기',
            do(S) { drop(S, 'vecs', 'c'); S.frames.A.T = Ti; S.frames.B.T = I(); watch(S, 'A', 'B', '실시간 \\({}^B_A T\\)'); },
            motion: [{ f: ['A', 'B'], rotM: fromRp(Rt, O), tr: q, about: 'world' }],
            tex: R`{}^BP_{AORG} = ${tv(q)}`,
            html: R`<p>전체를 \(\Tm{B}{A}\)로 옮겨 {B}가 기준이 되게 했습니다. 이제 {A}의 원점이 격자 위 (${f(q[0])}, ${f(q[1])})에, 축은 −30° 돌아간 상태로 보입니다. 실시간 행렬이 위 결과와 같습니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= 1.11 XYZ fixed angles / ZYX Euler ================= */
  sec({
    id: 'c1-rpy', group: 'lecture', ref: '슬라이드 22–26', title: '1.11 X-Y-Z 고정각과 Z-Y-X 오일러각',
    view: { pos: [7.6, -2.6, 4.2], target: [0, 0, 0.3] }, grid: [3.5, 0.5], scale: 0.75, fsize: 1,
    build() {
      const g = 30, b = 20, a = 40;
      const pL = [0, -1.6, 0], pR = [0, 1.6, 0];
      const Rr = mul(rz(a), ry(b), rx(g)), r = R3(Rr);
      const cb = Math.hypot(r[0][0], r[1][0]);
      const B2 = atan2d(-r[2][0], cb), A2 = atan2d(r[1][0] / cb, r[0][0] / cb), G2 = atan2d(r[2][1] / cb, r[2][2] / cb);
      const sym = R`\begin{bmatrix} c\alpha c\beta & c\alpha s\beta s\gamma - s\alpha c\gamma & c\alpha s\beta c\gamma + s\alpha s\gamma\\ s\alpha c\beta & s\alpha s\beta s\gamma + c\alpha c\gamma & s\alpha s\beta c\gamma - c\alpha s\gamma\\ -s\beta & c\beta s\gamma & c\beta c\gamma\end{bmatrix}`;
      return {
        init(S) {
          fr(S, 'A1', tr(pL), { ghost: true, label: 'A', name: '' });
          fr(S, 'B', tr(pL), { name: '{B} 고정각 X→Y→Z' });
        },
        steps: [
          {
            t: '① X_A축으로 γ = 30°',
            do(S) { S.frames.B.T = mul(tr(pL), rx(g)); },
            motion: [{ f: 'B', rot: [X, g], about: 'worldDir' }],
            tex: R`R = R_X(\gamma) = \begin{bmatrix}1&0&0\\0&c\gamma&-s\gamma\\0&s\gamma&c\gamma\end{bmatrix}`,
            html: R`<p>{B}가 {A}와 겹친 상태에서 출발해, <b>고정된 {A}의 축</b>을 기준으로 X → Y → Z 순서로 돌립니다(roll-pitch-yaw). 첫 번째는 \(\Xh_A\)축으로 γ.</p>`
          },
          {
            t: '② Y_A축으로 β = 20°',
            do(S) { S.frames.B.T = mul(tr(pL), ry(b), rx(g)); },
            motion: [{ f: 'B', rot: [Yv, b], about: 'worldDir' }],
            tex: R`R = R_Y(\beta)\,R_X(\gamma)`,
            html: R`<p>{B}의 Y축이 아니라 <b>흐린 {A}의 Y축</b>으로 돕니다. 고정축 기준이므로 새 회전은 <b>왼쪽</b>에 붙습니다(1.9절).</p>`
          },
          {
            t: '③ Z_A축으로 α = 40°',
            do(S) { S.frames.B.T = mul(tr(pL), Rr); watch(S, 'B', 'A1', '실시간 \\({}^A_B T\\)'); },
            motion: [{ f: 'B', rot: [Z, a], about: 'worldDir' }],
            tex: R`\begin{aligned}R_Y(\beta)R_X(\gamma) &= \begin{bmatrix}c\beta&0&s\beta\\0&1&0\\-s\beta&0&c\beta\end{bmatrix}\begin{bmatrix}1&0&0\\0&c\gamma&-s\gamma\\0&s\gamma&c\gamma\end{bmatrix} = \begin{bmatrix}c\beta & s\beta s\gamma & s\beta c\gamma\\ 0 & c\gamma & -s\gamma\\ -s\beta & c\beta s\gamma & c\beta c\gamma\end{bmatrix}\\ {}^A_BR_{XYZ} &= R_Z(\alpha)\big[R_Y(\beta)R_X(\gamma)\big] = ${sym} = ${tm(r)}\end{aligned}`,
            html: R`<p>먼저 \(R_YR_X\)를 곱하고(3행이 이미 최종 형태), 그 결과에 \(R_Z\)를 곱하면 1·2행만 섞입니다. 세 회전을 마치면 슬라이드 22의 행렬이 됩니다. 수행 순서는 X→Y→Z인데 식은 \(R_ZR_YR_X\)로 <b>역순</b>입니다.</p>
            <p class="note">특히 3행 \((-s\beta,\ c\beta s\gamma,\ c\beta c\gamma)\)과 1열 \((c\alpha c\beta,\ s\alpha c\beta,\ -s\beta)\)이 단순합니다. 역문제(⑤)에서 이 원소만 씁니다.</p>`
          },
          {
            t: '④ Z-Y-X 오일러각: 이동축으로 Z→Y′→X″',
            pre(S) { fr(S, 'A2', tr(pR), { ghost: true, label: 'A', name: '' }); fr(S, 'E', tr(pR), { name: '{B} 오일러 Z→Y′→X″' }); },
            do(S) { fr(S, 'A2', tr(pR), { ghost: true, label: 'A', name: '' }); fr(S, 'E', mul(tr(pR), Rr), { name: '{B} 오일러 Z→Y′→X″' }); },
            motion: [{ f: 'E', rot: [Z, a], about: 'local' }, { f: 'E', rot: [Yv, b], about: 'local' }, { f: 'E', rot: [X, g], about: 'local' }],
            tex: R`{}^A_BR_{Z'Y'X'}(\alpha,\beta,\gamma) = R_Z(\alpha)\,R_Y(\beta)\,R_X(\gamma) = {}^A_BR_{XYZ}(\gamma,\beta,\alpha)`,
            html: R`<p>오른쪽 좌표계는 이번에는 <b>자기 자신의 축</b>으로 Z(α) → Y′(β) → X″(γ) 순서로 돕니다. 이동축 기준이라 오른쪽에 차례로 곱해져 \(R_ZR_YR_X\)이 됩니다.</p>
            <p>두 좌표계의 최종 자세가 똑같습니다. <b>고정축 X-Y-Z = 이동축 Z-Y-X</b>(순서를 뒤집으면 같다). 그래서 24가지 각도 규약 중 고유한 것은 12가지입니다(슬라이드 26).</p>`
          },
          {
            t: '⑤ 역문제: R에서 (γ, β, α) 꺼내기',
            do(S) { betaRange(S, pL, B2); },
            assume: [{ t: R`\(c\beta > 0\), 즉 \(-90^\circ < \beta < 90^\circ\) — {B} 옆의 초록 반원 (현재 β = ${f(B2, 1)}°)`, ok: B2 > -90 && B2 < 90 }],
            tex: R`\begin{aligned}&r_{11} = c\alpha c\beta,\ r_{21} = s\alpha c\beta,\ r_{31} = -s\beta\ \Rightarrow\ r_{11}^2 + r_{21}^2 = c^2\beta(c^2\alpha + s^2\alpha) = c^2\beta\ \xrightarrow{\ c\beta>0\ }\ c\beta = +\sqrt{r_{11}^2+r_{21}^2}\\ \beta &= \text{atan2}\big(-r_{31},\ \sqrt{r_{11}^2+r_{21}^2}\big) = \text{atan2}(${f(-r[2][0])},\ ${f(cb)}) = ${deg(B2)}\\ \alpha &= \text{atan2}\big(r_{21}/c\beta,\ r_{11}/c\beta\big) = \text{atan2}(${f(r[1][0] / cb)},\ ${f(r[0][0] / cb)}) = ${deg(A2)}\\ \gamma &= \text{atan2}\big(r_{32}/c\beta,\ r_{33}/c\beta\big) = \text{atan2}(${f(r[2][1] / cb)},\ ${f(r[2][2] / cb)}) = ${deg(G2)}\end{aligned}`,
            html: R`<p>제곱해 더하면 α가 사라져 \(c^2\beta\)만 남습니다. 제곱근의 부호는 정할 수 없으므로 <b>가정</b> \(c\beta > 0\)으로 양의 근을 택합니다. 그러면 \(\beta = \text{atan2}(s\beta, c\beta)\)가 −90°~90° 안에서 결정됩니다(초록 반원 = 허용 범위, 초록 화살표 = 구한 β).</p>
            <p>그다음 1열의 \(r_{21}/r_{11}\)에서 α, 3행의 \(r_{32}/r_{33}\)에서 γ. 위 숫자가 처음 넣은 (30°, 20°, 40°)로 돌아왔습니다.</p>
            <p class="note">atan2(y, x)는 부호로 사분면까지 결정하므로 \(\tan^{-1}\)보다 안전합니다.</p>`
          },
          {
            t: '⑥ 특이점 β = ±90° (짐벌 락)',
            assume: [{ t: R`⑤의 가정 \(c\beta \ne 0\)이 깨짐 → 0으로 나누게 됨`, ok: false }, R`대신 \(\alpha = 0\)으로 둔다는 새 가정을 씀`],
            pre(S) {
              S.frames.B.T = mul(tr(pL), ry(90)); S.frames.E.T = mul(tr(pR), ry(90));
              S.frames.B.name = 'γ만 +30°'; S.frames.E.name = 'α만 −30°'; S.watch = null; delete S.arcs.brng; delete S.vecs.bcur;
            },
            do(S) {
              S.frames.B.T = mul(tr(pL), ry(90), rx(30)); S.frames.E.T = mul(tr(pR), rz(-30), ry(90));
              S.frames.B.name = 'γ만 +30°'; S.frames.E.name = 'α만 −30°'; S.watch = null; delete S.arcs.brng; delete S.vecs.bcur;
            },
            motion: [{ f: 'B', rot: [X, 30], about: 'local' }, { f: 'E', rot: [Z, -30], about: 'worldDir', with: true }],
            tex: R`\beta = 90^\circ:\ R = \begin{bmatrix}0 & s(\gamma-\alpha) & c(\gamma-\alpha)\\ 0 & c(\gamma-\alpha) & -s(\gamma-\alpha)\\ -1 & 0 & 0\end{bmatrix}\ \Rightarrow\ \alpha = 0,\ \ \gamma = \text{atan2}(r_{12},\, r_{22})`,
            html: R`<p>β = 90°이면 \(c\beta=0\)이라 ⑤의 식이 0으로 나누게 됩니다. 이때 왼쪽은 γ만 +30°, 오른쪽은 α만 −30° 바꿨는데 <b>똑같은 자세</b>가 되었습니다. γ와 α가 같은 축을 돌리게 되어 \(\gamma-\alpha\)만 정해집니다.</p>
            <p>그래서 관례적으로 α = 0으로 두고 γ를 구합니다(β = −90°이면 \(\gamma=-\text{atan2}(r_{12},r_{22})\)).</p>`
          }
        ]
      };
    }
  });

  /* ================= 1.12 angle-axis ================= */
  sec({
    id: 'c1-aa', group: 'lecture', ref: '슬라이드 27–28, 30', title: '1.12 등가 각-축 표현',
    view: { pos: [4.6, -3.8, 3.2], target: [0.2, 0.2, 0.4] }, grid: [2.5, 0.5], scale: 0.7, fsize: 1,
    build() {
      const k = unit([0.707, 0.707, 0]), th = 30, al = 45, be = 90;
      const RK = rotK(k, th), r = R3(RK);
      const back = mul(ry(-be), rz(-al));
      const t2 = Math.acos((r[0][0] + r[1][1] + r[2][2] - 1) / 2) / D;
      const kk = scl([r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]], 1 / (2 * Math.sin(t2 * D)));
      const arcK = S => arc(S, 'th', { f: 'kf', c: scl(k, 0.95), n: k, u: Z, ang: th, r: 0.55, label: 'θ = 30°', grow: true, col: 'k' });
      return {
        init(S) {
          fr(S, 'A', I(), { ghost: true });
          fr(S, 'kf', I(), { hidden: true });
          fr(S, 'B', I());
          ln(S, 'K', { f: 'kf', p: scl(k, -1.6) }, { f: 'kf', p: scl(k, 2.2) }, { c: 'k', dash: false, label: 'K̂', lt: 1, lo: [0, 0, 0.12] });
        },
        steps: [
          {
            t: '임의의 축 K로 θ만큼 회전',
            do(S) { S.frames.B.T = RK; arcK(S); watch(S, 'B', 'A', '실시간 \\({}^A_B T\\)'); },
            motion: [{ f: 'B', rot: [k, th], about: 'world' }],
            tex: R`\Rm{A}{B} = R_K(\theta),\qquad {}^A\Kh = \begin{bmatrix}0.707 & 0.707 & 0\end{bmatrix}^T,\ \ \theta = 30^\circ`,
            html: R`<p>{B}가 {A}와 겹친 상태에서 주축이 아닌 <b>임의 방향의 축 \(\Kh\)</b>(보라색) 둘레로 오른손 법칙에 따라 θ만큼 돌았습니다. 슬라이드 30 예제의 값입니다.</p>
            <p>이 행렬을 어떻게 구할까요? 아는 것은 주축 회전 \(R_X,R_Y,R_Z\)뿐입니다. → K를 Z축으로 옮겨 놓고 돌리면 됩니다.</p>`
          },
          {
            t: '① K를 Z축에 맞추기',
            pre(S) { S.frames.B.T = I(); drop(S, 'arcs'); fr(S, 'Bt', RK, { ghost: true, op: 0.25, label: '', name: '' }); },
            do(S) { S.frames.B.T = back; S.frames.kf.T = back; drop(S, 'arcs'); fr(S, 'Bt', RK, { ghost: true, op: 0.25, label: '', name: '' }); S.watch = null; },
            motion: [{ f: ['B', 'kf'], rot: [Z, -al], about: 'world' }, { f: ['B', 'kf'], rot: [Yv, -be], about: 'world' }],
            tex: R`R_{y,-\beta}\,R_{z,-\alpha}:\ \Kh\to\Zh,\qquad \alpha = \text{atan2}(k_y,k_x) = 45^\circ,\ \ \beta = \text{atan2}\big(\sqrt{k_x^2+k_y^2},\,k_z\big) = 90^\circ`,
            html: R`<p>{B}를 원래 자리로 되돌린 뒤, K와 {B}를 함께 Z축 둘레로 −α(K를 XZ 평면으로), Y축 둘레로 −β(K를 Z축으로) 돌렸습니다. 이제 K가 \(\Zh_A\)와 겹칩니다.</p>
            <p class="note">α는 K를 xy 평면에 투영했을 때의 방위각, β는 K와 Z축 사이의 각입니다.</p>`
          },
          {
            t: '② 이제 Z축으로 θ',
            do(S) { S.frames.B.T = mul(rz(th), back); arcK(S); },
            motion: [{ f: 'B', rot: [Z, th], about: 'world' }],
            tex: R`R_{z,\theta}\,R_{y,-\beta}\,R_{z,-\alpha}`,
            html: R`<p>K가 Z축이므로 \(R_Z(\theta)\)로 돌리면 됩니다.</p>`
          },
          {
            t: '③ 원래 방향으로 되돌리기',
            do(S) { S.frames.B.T = RK; S.frames.kf.T = I(); },
            motion: [{ f: ['B', 'kf'], rot: [Yv, be], about: 'world' }, { f: ['B', 'kf'], rot: [Z, al], about: 'world' }],
            tex: R`R_K(\theta) = R_{z,\alpha}\,R_{y,\beta}\,R_{z,\theta}\,R_{y,-\beta}\,R_{z,-\alpha}`,
            html: R`<p>①의 역순으로 되돌리면 K가 원래 방향으로 돌아가고, {B}는 처음 1단계에서 본 자세(흐린 목표)와 정확히 겹칩니다. 모든 회전이 고정축 기준이므로 왼쪽으로 쌓였습니다.</p>`
          },
          {
            t: '④ 닫힌 식과 슬라이드 30 답',
            do(S) { delete S.frames.Bt; watch(S, 'B', 'A', '실시간 \\({}^A_B T\\)'); },
            tex: R`R_K(\theta)=\begin{bmatrix}k_x^2v\theta + c\theta & k_xk_yv\theta - k_zs\theta & k_xk_zv\theta + k_ys\theta\\ k_xk_yv\theta + k_zs\theta & k_y^2v\theta + c\theta & k_yk_zv\theta - k_xs\theta\\ k_xk_zv\theta - k_ys\theta & k_yk_zv\theta + k_xs\theta & k_z^2v\theta + c\theta\end{bmatrix}= ${tm(r)},\quad v\theta = 1 - c\theta`,
            html: R`<p>③의 다섯 행렬을 곱하고 \(k_x = s\beta c\alpha,\ k_y = s\beta s\alpha,\ k_z = c\beta\)로 바꿔 쓰면 정리됩니다. 예: (3,3) 원소는 \(c^2\beta + s^2\beta\,c\theta = k_z^2 + (1-k_z^2)c\theta = k_z^2v\theta + c\theta\). 이렇게 하면 슬라이드 28의 식이 됩니다. \(k=(0.707, 0.707, 0)\), θ = 30°를 넣으면 슬라이드 30의 답(0.933, 0.067, 0.354, …)과 같습니다. 원점이 그대로이므로 \({}^AP_{BORG}=0\)입니다.</p>`
          },
          {
            t: '⑤ 역문제: R에서 θ와 K',
            assume: [{ t: R`\(\sin\theta \ne 0\) (θ ≠ 0°, 180°) — 0°이면 축이 정해지지 않고, 180°이면 다른 방법 필요`, ok: Math.abs(Math.sin(t2 * D)) > 1e-6 }],
            tex: R`\begin{aligned}&r_{11}+r_{22}+r_{33} = (k_x^2+k_y^2+k_z^2)v\theta + 3c\theta = (1 - c\theta) + 3c\theta = 1 + 2c\theta,\qquad r_{32}-r_{23} = (k_yk_zv\theta + k_xs\theta) - (k_yk_zv\theta - k_xs\theta) = 2k_xs\theta\\ &\theta = \cos^{-1}\!\Big(\frac{r_{11}+r_{22}+r_{33}-1}{2}\Big) = ${deg(t2)},\qquad \Kh = \frac{1}{2\sin\theta}\begin{bmatrix}r_{32}-r_{23}\\ r_{13}-r_{31}\\ r_{21}-r_{12}\end{bmatrix} = ${tv(kk)}\end{aligned}`,
            html: R`<p>대각합(trace)은 \(1+2\cos\theta\)이고, 비대칭 부분 \(R-R^T\)에 \(2\sin\theta\,\Kh\)가 들어 있습니다. 숫자가 처음 축과 각도로 돌아옵니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= Example slide 29 (noa) ================= */
  sec({
    id: 'c1-ex29', group: 'example', ref: '슬라이드 29', title: '예제: 고정축 순서 vs noa 이동축 순서',
    view: { pos: [16, -15, 13], target: [2.5, 1.5, 3] }, grid: [10, 2], scale: 2, fsize: 2.2,
    build() {
      const p = [7, 3, 1];
      const nm = { axes: ['n', 'o', 'a'], sub: false, name: '{noa}' };
      const T1 = rz(90), T2 = mul(ry(90), rz(90)), T3 = mul(tr(4, -3, 7), ry(90), rz(90));
      const U1 = rz(90), U2 = mul(rz(90), tr(4, -3, 7)), U3 = mul(rz(90), tr(4, -3, 7), ry(90));
      const at = T => tv(app(T, p), 3);
      return {
        init(S) {
          fr(S, 'A', I(), { axes: ['x', 'y', 'z'], sub: false, name: '{xyz}', ghost: true });
          fr(S, 'N', I(), nm);
          pt(S, 'p', { f: 'N', p }, 'p');
          vec(S, 'np', { f: 'N' }, { f: 'N', p }, { w: 1, label: '' });
        },
        steps: [
          {
            t: '문제',
            tex: R`p_{noa} = \begin{bmatrix}7&3&1\end{bmatrix}^T`,
            html: R`<p>점 p가 프레임 \(F_{noa}\)에 붙어 있고, 처음에는 noa와 xyz가 겹쳐 있습니다. 다음 두 경우의 \(p_{xyz}\)를 구하시오.</p>
            <p>1. Rot(z, 90°) → Rot(y, 90°) → Trans(4, −3, 7) <i>(기준 xyz축)</i><br>2. Rot(a, 90°) → noa축 방향 Trans(4, −3, 7) → Rot(o, 90°) <i>(현재 noa축)</i></p>`
          },
          {
            t: 'Case 1 ① Rot(z, 90°)',
            do(S) { S.frames.N.T = T1; },
            motion: [{ f: 'N', rot: [Z, 90], about: 'world' }],
            tex: R`Rot(z,90^\circ)\,p_{noa} = ${at(T1)}`,
            html: R`<p>기준 z축 둘레로 프레임과 점이 함께 돕니다.</p>`
          },
          {
            t: 'Case 1 ② Rot(y, 90°)',
            do(S) { S.frames.N.T = T2; },
            motion: [{ f: 'N', rot: [Yv, 90], about: 'world' }],
            tex: R`Rot(y,90^\circ)\,Rot(z,90^\circ)\,p_{noa} = ${at(T2)}`,
            html: R`<p>흐린 <b>기준 y축</b> 둘레입니다(noa의 o축이 아님). 기준축이므로 왼쪽에 곱합니다.</p>`
          },
          {
            t: 'Case 1 ③ Trans(4, −3, 7)',
            do(S) { S.frames.N.T = T3; pt(S, 'r1', app(T3, p), '(5, 4, 10)', { c: 'ink', r: 0.8 }); },
            motion: [{ f: 'N', tr: [4, -3, 7], about: 'world' }],
            tex: R`p_{xyz} = Trans(4,-3,7)\,Rot(y,90^\circ)\,Rot(z,90^\circ)\,p_{noa} = ${at(T3)}`,
            html: R`<p>기준 x, y, z 방향으로 (4, −3, 7) 이동합니다. 슬라이드 29 왼쪽 답 (5, 4, 10)과 같습니다.</p>`
          },
          {
            t: 'Case 2 ① Rot(a, 90°)',
            pre(S) { S.frames.N.T = I(); },
            do(S) { S.frames.N.T = U1; },
            motion: [{ f: 'N', rot: [Z, 90], about: 'local' }],
            tex: R`Rot(a,90^\circ)`,
            html: R`<p>다시 처음 자세에서 시작합니다. 이번에는 <b>현재 noa축</b> 기준입니다. 처음에는 a = z라 Case 1과 같아 보입니다.</p>`
          },
          {
            t: 'Case 2 ② noa 방향으로 Trans(4, −3, 7)',
            do(S) { S.frames.N.T = U2; },
            motion: [{ f: 'N', tr: [4, -3, 7], about: 'local' }],
            tex: R`Rot(a,90^\circ)\,Trans(4,-3,7)`,
            html: R`<p>n 방향 4, o 방향 −3, a 방향 7입니다. n축이 이미 y 방향을 향하고 있어서 실제 이동은 (3, 4, 7)이 됩니다. 이동축 기준이므로 오른쪽에 곱합니다.</p>`
          },
          {
            t: 'Case 2 ③ Rot(o, 90°)',
            do(S) { S.frames.N.T = U3; pt(S, 'r2', app(U3, p), '(0, 5, 0)', { c: 'ink', r: 0.8 }); },
            motion: [{ f: 'N', rot: [Yv, 90], about: 'local' }],
            tex: R`p_{xyz} = Rot(a,90^\circ)\,Trans(4,-3,7)\,Rot(o,90^\circ)\,p_{noa} = ${at(U3)}`,
            html: R`<p>현재 o축 둘레로 돌아 p가 (0, 5, 0)에 옵니다(슬라이드 29 오른쪽 답). 같은 "회전 2번 + 이동 1번"이라도 기준축이 다르면 곱하는 방향과 결과가 완전히 달라집니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= Practice 1: cylindrical robot (HW1 P1) ================= */
  sec({
    id: 'c1-hw-cyl', group: 'practice', ref: '과제 1 · 문제 1 유형', title: '원통형 로봇의 관절 변수',
    params: {
      orig: { x: 4, y: 5, z: 6 }, origLabel: '과제 원래 숫자 (4, 5, 6)',
      rand() {
        const pick = (a, b) => a + Math.floor(Math.random() * (b - a + 1));
        let x = 0, y = 0; while (!x) x = pick(-6, 6); while (!y) y = pick(-6, 6);
        return { x, y, z: pick(2, 7) };
      }
    },
    build(P) {
      const { x, y, z } = P;
      const al = atan2d(y, x), r = Math.hypot(x, y), l = z;
      const goal = [x, y, z];
      const car = mul(tr(0, 0, l), rz(al)), H = mul(car, tr(r, 0, 0)), Hf = mul(H, rz(-al));
      const top = Math.max(l + 1.2, 3);
      const sz = Math.max(r, l, 4);
      const ca = Math.cos(al * D), sa = Math.sin(al * D);
      return {
        view: { pos: [x / 2 + sz * 1.6, y / 2 - sz * 1.9, z / 2 + sz * 1.3], target: [x / 2, y / 2, z / 2] },
        grid: [Math.ceil(sz + 1), 1],
        answers: [
          { label: R`\(\alpha\)`, val: al, unit: '°', tol: 0.2, dp: 2 },
          { label: R`\(r\)`, val: r, tol: 0.02 },
          { label: R`\(l\)`, val: l, tol: 0.01 }
        ],
        init(S) {
          fr(S, 'A', I(), { size: 1.2 });
          fr(S, 'car', I(), { hidden: true });
          fr(S, 'H', I(), { size: 1, label: 'H' });
          body(S, 'base', 'A', 'cyl', [0.9, 0.12], [0, 0, -0.06], { c: 'body2' });
          body(S, 'col', 'A', 'cyl', [0.22, top], [0, 0, top / 2]);
          body(S, 'carr', 'car', 'box', [0.7, 0.7, 0.45], [0, 0, 0], { c: 'body2' });
          body(S, 'arm', 'car', 'box', [0.0001, 0.26, 0.26], [0, 0, 0]);
          pt(S, 'goal', goal, `목표 (${x}, ${y}, ${z})`, { c: 'k', r: 1.4 });
        },
        steps: [
          {
            t: '문제',
            tex: R`T_1 = Trans(\hat x, r),\quad T_2 = Rot(\hat z,\alpha),\quad T_3 = Trans(\hat z, l)\qquad \text{origin of hand} = ${tv(goal, 0)}`,
            html: R`<p>원통형 로봇은 이동 2개와 회전 1개로 손(hand) 프레임 {H}를 놓습니다. 고정 프레임 기준으로 ① x축 방향 r 이동 → ② z축 둘레 α 회전 → ③ z축 방향 l 이동.</p>
            <p>1) 세 변환행렬 2) 전체 변환행렬 3) 손 원점을 (${x}, ${y}, ${z})에 놓는 관절 변수와 T 4) 손의 방향을 원래 좌표계와 같게 만드는 추가 회전과 최종 T를 구하시오.</p>`
          },
          {
            t: '① T₁: x축으로 r만큼 뻗기',
            assume: [R`세 변환 모두 <b>고정 프레임</b> 기준 → 새 변환을 왼쪽에 곱함 (\(T = T_3T_2T_1\))`],
            do(S) { S.frames.H.T = tr(r, 0, 0); S.bodies.arm.size = [r, 0.26, 0.26]; S.bodies.arm.pos = [r / 2, 0, 0]; },
            motion: [{ f: 'H', tr: [r, 0, 0], about: 'world' }],
            tex: R`T_1 = \begin{bmatrix}1&0&0&r\\0&1&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix}`,
            html: R`<p>팔이 x축을 따라 r만큼 뻗어 나갑니다(아직 r의 값은 모르지만 화면에서는 답 값으로 보여 줍니다).</p>`
          },
          {
            t: '② T₂: z축 둘레로 α 회전',
            do(S) {
              S.frames.car.T = rz(al); S.frames.H.T = mul(rz(al), tr(r, 0, 0));
              arc(S, 'al', { n: Z, u: X, ang: al, r: Math.min(r * 0.45, 2), label: 'α', grow: true });
            },
            motion: [{ f: ['car', 'H'], rot: [Z, al], about: 'world' }],
            tex: R`T_2 = \begin{bmatrix}C\alpha&-S\alpha&0&0\\S\alpha&C\alpha&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix},\qquad T_2T_1 = \begin{bmatrix}C\alpha&-S\alpha&0&rC\alpha\\S\alpha&C\alpha&0&rS\alpha\\0&0&1&0\\0&0&0&1\end{bmatrix}`,
            html: R`<p>기둥이 돌면서 팔 끝이 원호를 그립니다. 고정 프레임 기준 변환이므로 \(T_2\)를 <b>왼쪽</b>에 곱합니다. 회전이 손의 위치 \((r,0,0)\)까지 돌려 \((rC\alpha, rS\alpha, 0)\)이 됩니다.</p>`
          },
          {
            t: '③ T₃: z축으로 l만큼 올리기',
            do(S) { S.frames.car.T = car; S.frames.H.T = H; watch(S, 'H', 'A', '실시간 \\({}^A_H T\\)'); },
            motion: [{ f: ['car', 'H'], tr: [0, 0, l], about: 'world' }],
            tex: R`T = T_3T_2T_1 = \begin{bmatrix}C\alpha&-S\alpha&0&rC\alpha\\S\alpha&C\alpha&0&rS\alpha\\0&0&1&l\\0&0&0&1\end{bmatrix}`,
            html: R`<p>캐리지가 기둥을 따라 올라갑니다. 2)의 답: \(T=T_3T_2T_1\). 4열이 손의 위치입니다.</p>`
          },
          {
            t: '④ 목표 위치 → 관절 변수',
            assume: [R`\(r \ge 0\) (팔 길이) → \(r = +\sqrt{x^2+y^2}\)`, R`α는 atan2로: x, y의 부호로 사분면 결정`],
            do(S) {
              ln(S, 'dn', goal, [x, y, 0], { c: 'z', label: 'l', lt: 0.5, lo: [0.2, 0, 0] });
              ln(S, 'rr', O, [x, y, 0], { c: 'x', label: 'r', lt: 0.55, lo: [0, 0, 0.2] });
            },
            tex: R`\begin{bmatrix}rC\alpha\\ rS\alpha\\ l\end{bmatrix} = ${tv(goal, 0)}\ \Rightarrow\ \alpha = \text{atan2}(${y},\,${x}) = ${f(al, 2)}^\circ,\quad r = \sqrt{${x < 0 ? '(' + x + ')' : x}^2+${y < 0 ? '(' + y + ')' : y}^2} = \sqrt{${x * x + y * y}} = ${f(r)},\quad l = ${l}`,
            html: R`<p>4열을 목표 위치와 같게 놓습니다. 바닥에 그린 선의 길이가 r, 수직선이 l, 원호가 α입니다. α는 반드시 <b>atan2</b>로 구해야 x, y의 부호에 맞는 사분면이 나옵니다.</p>`
          },
          {
            t: '⑤ 변환행렬 숫자로',
            tex: R`T = \begin{bmatrix}C\alpha&-S\alpha&0&${x}\\S\alpha&C\alpha&0&${y}\\0&0&1&${l}\\0&0&0&1\end{bmatrix} = ${tm(H)}`,
            html: R`<p>\(C\alpha = ${x}/\sqrt{${x * x + y * y}} = ${f(ca, 4)}\), \(S\alpha = ${y}/\sqrt{${x * x + y * y}} = ${f(sa, 4)}\). 실시간 행렬과 비교해 보세요.</p>`
          },
          {
            t: '⑥ 방향 되돌리기: 이동 z축으로 −α',
            do(S) { S.frames.H.T = Hf; drop(S, 'lines'); },
            motion: [{ f: 'H', rot: [Z, -al], about: 'local' }],
            tex: R`T_{final} = T\cdot Rot(z,-\alpha) = \begin{bmatrix}C\alpha&-S\alpha&0&${x}\\S\alpha&C\alpha&0&${y}\\0&0&1&${l}\\0&0&0&1\end{bmatrix}\begin{bmatrix}C\alpha&S\alpha&0&0\\-S\alpha&C\alpha&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix} = ${tm(Hf)}`,
            html: R`<p>손의 축이 α만큼 돌아가 있으므로 손 자신의 z축(이동 프레임) 둘레로 −α = ${f(-al, 2)}° 돌립니다. 이동 프레임 기준이므로 <b>오른쪽</b>에 곱합니다. 결과의 회전 블록이 단위행렬이 되어 {H}의 축이 {A}와 평행해졌습니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= Practice 2: XYZ fixed angles from R (HW1 P3) ================= */
  sec({
    id: 'c1-hw-rpy', group: 'practice', ref: '과제 1 · 문제 3 유형', title: 'R에서 X-Y-Z 고정각 구하기',
    view: { pos: [3.6, -3.2, 2.6], target: [0, 0, 0.3] }, grid: [2, 0.5], scale: 0.6, fsize: 1,
    params: {
      orig: { R: [[0.33, -0.770, 0.547], [0.908, 0.418, 0.0396], [-0.259, 0.483, 0.837]] }, origLabel: '과제 원래 행렬',
      rand() {
        const pick = (a, b) => a + Math.floor(Math.random() * (b - a + 1));
        const g = pick(-80, 80), b = pick(-70, 70), a = pick(-170, 170);
        const Rr = R3(mul(rz(a), ry(b), rx(g))).map(row => row.map(v => M.round(v, 3)));
        return { R: Rr };
      }
    },
    build(P) {
      const r = P.R;
      const cb = Math.hypot(r[0][0], r[1][0]);
      const b = atan2d(-r[2][0], cb), a = atan2d(r[1][0] / cb, r[0][0] / cb), g = atan2d(r[2][1] / cb, r[2][2] / cb);
      const Rt = mul(rz(a), ry(b), rx(g));
      return {
        answers: [
          { label: R`\(\gamma\)`, val: g, unit: '°', tol: 0.3, dp: 2 },
          { label: R`\(\beta\)`, val: b, unit: '°', tol: 0.3, dp: 2 },
          { label: R`\(\alpha\)`, val: a, unit: '°', tol: 0.3, dp: 2 }
        ],
        init(S) { fr(S, 'A', I(), { ghost: true }); fr(S, 'Bt', Rt, { label: 'B' }); },
        steps: [
          {
            t: '문제',
            tex: R`\Rm{A}{B} = ${tm(r, 4)}\quad\Rightarrow\quad (\gamma,\beta,\alpha) = ?`,
            html: R`<p>{A}와 {B}가 처음에 겹쳐 있고, {B}를 \(\Xh_A\)축으로 γ, \(\Yh_A\)축으로 β, \(\Zh_A\)축으로 α만큼 차례로 돌렸습니다. 위 회전행렬에 해당하는 X-Y-Z 고정각을 구하시오.</p>
            <p class="note">공식은 1.11절 ⑤. 먼저 손으로 풀고 위의 "직접 풀고 답 확인하기"에 넣어 보세요.</p>`
          },
          {
            t: '① β (3행 1열과 1열의 크기)',
            do(S) { betaRange(S, [0, 0, 0], b, 0.7); },
            assume: [{ t: R`\(c\beta > 0\) (\(-90^\circ<\beta<90^\circ\)) — 초록 반원`, ok: b > -90 && b < 90 }],
            tex: R`\beta = \text{atan2}\big(-r_{31},\ \sqrt{r_{11}^2+r_{21}^2}\big) = \text{atan2}\big(${f(-r[2][0], 4)},\ \sqrt{(${f(r[0][0], 4)})^2 + (${f(r[1][0], 4)})^2}\big) = \text{atan2}(${f(-r[2][0], 4)},\ ${f(cb, 4)}) = ${deg(b)}`,
            html: R`<p>\(r_{31} = -s\beta\)이고 \(r_{11}^2+r_{21}^2 = c^2\beta\)입니다. \(c\beta = ${f(cb, 4)}\)는 이후 두 식의 분모로 계속 씁니다.</p>`
          },
          {
            t: '② α (1열)',
            tex: R`\alpha = \text{atan2}\Big(\frac{r_{21}}{c\beta},\ \frac{r_{11}}{c\beta}\Big) = \text{atan2}\Big(\frac{${f(r[1][0], 4)}}{${f(cb, 4)}},\ \frac{${f(r[0][0], 4)}}{${f(cb, 4)}}\Big) = ${deg(a)}`,
            html: R`<p>1열은 \((c\alpha c\beta,\ s\alpha c\beta,\ -s\beta)\). \(c\beta>0\)이므로 분모를 지워도 사분면은 같습니다.</p>`
          },
          {
            t: '③ γ (3행)',
            tex: R`\gamma = \text{atan2}\Big(\frac{r_{32}}{c\beta},\ \frac{r_{33}}{c\beta}\Big) = \text{atan2}\Big(\frac{${f(r[2][1], 4)}}{${f(cb, 4)}},\ \frac{${f(r[2][2], 4)}}{${f(cb, 4)}}\Big) = ${deg(g)}`,
            html: R`<p>3행은 \((-s\beta,\ c\beta s\gamma,\ c\beta c\gamma)\)입니다.</p>`
          },
          {
            t: '④ 검증: 구한 각도로 실제로 돌려 보기',
            pre(S) { S.frames.Bt.ghost = true; S.frames.Bt.label = ''; S.frames.Bt.op = 0.3; fr(S, 'B', I()); },
            do(S) { S.frames.Bt.ghost = true; S.frames.Bt.label = ''; S.frames.Bt.op = 0.3; fr(S, 'B', Rt); watch(S, 'B', 'A', '실시간 \\({}^A_B T\\)'); },
            motion: [{ f: 'B', rot: [X, g], about: 'world' }, { f: 'B', rot: [Yv, b], about: 'world' }, { f: 'B', rot: [Z, a], about: 'world' }],
            tex: R`R_Z(${f(a, 2)}^\circ)\,R_Y(${f(b, 2)}^\circ)\,R_X(${f(g, 2)}^\circ) = ${tm(R3(Rt))}`,
            html: R`<p>{A}와 겹친 {B}를 고정축 X → Y → Z 순서로 돌리면 흐린 목표 자세에 정확히 도착합니다. 곱한 결과가 주어진 행렬과 (반올림 오차 안에서) 같습니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= Practice 3: transform chain (HW1 P5) ================= */
  const ROTS = (() => {
    const out = [], perms = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];
    perms.forEach(p => [1, -1].forEach(s0 => [1, -1].forEach(s1 => [1, -1].forEach(s2 => {
      const m = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]; const s = [s0, s1, s2];
      p.forEach((c, i) => { m[i][c] = s[i]; });
      const det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
      if (det > 0) out.push(m);
    }))));
    return out;
  })();
  const T4 = rows => rows.concat([[0, 0, 0, 1]]);
  sec({
    id: 'c1-hw-chain', group: 'practice', ref: '과제 1 · 문제 5 유형', title: '카메라–물체–엔드이펙터 변환 체인',
    params: {
      orig: {
        T5C: T4([[0, 0, -1, 3], [0, -1, 0, 0], [-1, 0, 0, 5]]), T5H: T4([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 4]]),
        TCO: T4([[0, 0, 1, 2], [1, 0, 0, 2], [0, 1, 0, 4]]), THE: T4([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 3]])
      }, origLabel: '과제 원래 행렬',
      rand() {
        const pick = (a, b) => a + Math.floor(Math.random() * (b - a + 1));
        const rr = () => ROTS[pick(0, ROTS.length - 1)];
        const mk = (R, p) => fromRp(R, p);
        return {
          T5C: mk(rr(), [pick(1, 5), pick(-2, 2), pick(3, 6)]), T5H: mk(rr(), [pick(-2, 2), pick(-2, 2), pick(2, 5)]),
          TCO: mk(rr(), [pick(0, 3), pick(0, 3), pick(2, 5)]), THE: mk(I().slice(0, 3).map(r => r.slice(0, 3)), [0, 0, pick(1, 3)])
        };
      }
    },
    build(P) {
      const { T5C, T5H, TCO, THE } = P;
      const TC = T5C, TO = mul(T5C, TCO), TH = T5H, TE = mul(T5H, THE);
      const TEH = inv(THE), TH5 = inv(T5H), TEO = mul(TEH, TH5, T5C, TCO);
      const pos = [p3(TC), p3(TO), p3(TH), p3(TE), [0, 0, 0]];
      const cx = pos.reduce((s, p) => add(s, p), [0, 0, 0]).map(v => v / pos.length);
      const ext = Math.max(...pos.map(p => norm(sub(p, cx)))) + 2;
      const lb = (a, b) => `<sup>${a}</sup><sub>${b}</sub>T`;
      const ti = (m) => tm(m, 0);
      return {
        view: { pos: add(cx, [ext * 1.7, -ext * 1.9, ext * 1.4]), target: cx },
        grid: [Math.ceil(ext + 2), 1], scale: 1, fsize: 0.9,
        answers: [
          { label: R`\({}^EP_{O}\ x\)`, val: TEO[0][3], tol: 0.01 },
          { label: R`\({}^EP_{O}\ y\)`, val: TEO[1][3], tol: 0.01 },
          { label: R`\({}^EP_{O}\ z\)`, val: TEO[2][3], tol: 0.01 }
        ],
        init(S) {
          fr(S, 'L5', I(), { label: '5', name: '{5} 링크5' });
          fr(S, 'C', TC, { name: '{C} 카메라' }); fr(S, 'O', TO, { name: '{O} 물체' });
          fr(S, 'H', TH, { name: '{H} 손' }); fr(S, 'E', TE, { name: '{E} 엔드이펙터' });
          vec(S, '5C', O, { f: 'C' }, { c: 'acc', w: 0.6, label: lb(5, 'C') });
          vec(S, 'CO', { f: 'C' }, { f: 'O' }, { c: 'acc', w: 0.6, label: lb('C', 'O') });
          vec(S, '5H', O, { f: 'H' }, { c: 'acc', w: 0.6, label: lb(5, 'H') });
          vec(S, 'HE', { f: 'H' }, { f: 'E' }, { c: 'acc', w: 0.6, label: lb('H', 'E') });
        },
        steps: [
          {
            t: '문제',
            tex: R`{}^5_CT = ${ti(T5C)},\ {}^5_HT = ${ti(T5H)},\ {}^C_OT = ${ti(TCO)},\ {}^H_ET = ${ti(THE)}`,
            html: R`<p>로봇 5번 링크에 카메라 {C}가 달려 물체 {O}를 봅니다. 엔드이펙터 {E}가 물체 {O}와 일치하려면 필요한 상대 변환 \(\Tm{E}{O}\)를 구하시오.</p>
            <p>화면의 보라 화살표가 주어진 네 관계입니다. 화살표 방향 = "앞 프레임에서 본 뒤 프레임".</p>`
          },
          {
            t: '① 경로 찾기: E → H → 5 → C → O',
            do(S) {
              vec(S, 'EH', { f: 'E' }, { f: 'H' }, { c: 'k', w: 0.6, label: lb('E', 'H') + ' (역)', lo: [0.3, 0, 0] });
              vec(S, 'H5', { f: 'H' }, O, { c: 'k', w: 0.6, label: lb('H', 5) + ' (역)', lo: [0.3, 0, 0] });
            },
            tex: R`\Tm{E}{O} = \Tm{E}{H}\ \Tm{H}{5}\ {}^5_CT\ \Tm{C}{O}`,
            html: R`<p>E에서 O까지 가는 길을 화살표로 이어 봅니다. E→H와 H→5는 주어진 방향과 반대이므로(<span class="tk">보라색</span>) <b>역변환</b>이 필요합니다. 위첨자–아래첨자가 사슬처럼 이어지는지 확인하세요.</p>`
          },
          {
            t: '② 역변환 두 개',
            tex: R`\Tm{E}{H} = ({}^H_ET)^{-1} = ${ti(TEH)},\qquad \Tm{H}{5} = ({}^5_HT)^{-1} = \begin{bmatrix}R^T & -R^Tp\\0&1\end{bmatrix} = ${ti(TH5)}`,
            html: R`<p>1.10절 공식: 회전 블록은 전치, 4열은 \(-R^Tp\). \({}^5_HT\)의 경우 \(R^T\)를 먼저 구하고 \(p=(${p3(T5H).join(', ')})\)에 곱한 뒤 부호를 바꿉니다.</p>`
          },
          {
            t: '③ 네 행렬 곱하기',
            do(S) { vec(S, 'EO', { f: 'E' }, { f: 'O' }, { c: 'pt', w: 0.9, label: lb('E', 'O'), lo: [0, 0, 0.3] }); watch(S, 'O', 'E', '실시간 \\({}^E_O T\\)'); },
            tex: R`\Tm{E}{O} = ${ti(TEH)}${ti(TH5)}${ti(T5C)}${ti(TCO)} = ${ti(TEO)}`,
            html: R`<p>왼쪽부터 차례로 곱합니다(행렬 곱은 결합법칙은 성립하지만 교환은 안 됩니다). 주황 화살표가 결과 관계입니다.</p>
            <p class="note">중간 검산: \({}^5_OT = {}^5_CT\,{}^C_OT\)의 4열 = (${p3(TO).join(', ')})이 화면의 {O} 위치와 같은지 보세요.</p>`
          },
          {
            t: '④ 확인: {E}를 움직여 {O}에 도착',
            pre(S) { fr(S, 'X', TE, { name: "{E}→", sub: 'E' }); },
            do(S) { fr(S, 'X', TO, { name: '', label: '', sub: 'E' }); },
            motion: [{ f: 'X', rotM: TEO, tr: p3(TEO), about: 'local' }],
            tex: R`{}^5_ET\ \Tm{E}{O} = ${ti(mul(TE, TEO))} = {}^5_OT`,
            html: R`<p>{E}의 복사본을 자기 축 기준으로 \(\Tm{E}{O}\)만큼(먼저 이동, 그다음 회전) 움직이면 {O}와 정확히 겹칩니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= Practice 4: screw angle-axis (HW1 P6) ================= */
  sec({
    id: 'c1-hw-screw', group: 'practice', ref: '과제 1 · 문제 6 유형', title: '나사 조이기: 각-축 회전행렬', grid: [2.5, 0.5], scale: 0.65, fsize: 0.9,
    params: {
      orig: { k: [2, 1, 2], lead: 1.5, adv: 0.5 }, origLabel: '과제 원래 숫자',
      rand() {
        const pick = (a, b) => a + Math.floor(Math.random() * (b - a + 1));
        let k = [0, 0, 0]; while (!k[0] && !k[1]) k = [pick(-3, 3), pick(-3, 3), pick(1, 3)];
        const lead = [1, 1.25, 1.5, 2][pick(0, 3)];
        const advs = [0.25, 0.5, 0.75, 1].filter(a => a < lead);
        return { k, lead, adv: advs[pick(0, advs.length - 1)] };
      }
    },
    build(P) {
      const { k, lead, adv } = P;
      const kn = norm(k), kh = unit(k);
      const al = atan2d(k[1], k[0]), be = atan2d(Math.hypot(k[0], k[1]), k[2]);
      const th = adv / lead * 360;
      const RK = rotK(kh, th), r = R3(RK);
      const S0 = mul(rz(al), ry(be));
      const v = 1 - Math.cos(th * D), c = Math.cos(th * D), s = Math.sin(th * D);
      const kxy = [kh[0], kh[1], 0];
      const side = unit(Math.hypot(kh[0], kh[1]) > 0.1 ? cross(kh, Z) : X);
      const tgt = scl(kh, 0.9);
      return {
        view: { pos: add(tgt, add(scl(side, 4.4), add(scl(kh, 0.8), [0, 0, 1.6]))), target: tgt },
        answers: [
          { label: R`\(\alpha\)`, val: al, unit: '°', tol: 0.2, dp: 2 },
          { label: R`\(\beta\)`, val: be, unit: '°', tol: 0.2, dp: 2 },
          { label: R`\(\theta\)`, val: th, unit: '°', tol: 0.2, dp: 2 },
          { label: R`\(r_{11}\)`, val: r[0][0], tol: 0.005, dp: 4 }
        ],
        init(S) {
          fr(S, 'A', I(), { ghost: true });
          fr(S, 'scr', S0, { hidden: true });
          fr(S, 'D', I(), { label: 'B', size: 0.8 });
          body(S, 'shank', 'scr', 'cyl', [0.045, 1.3], [0, 0, 0.65]);
          body(S, 'head', 'scr', 'cyl', [0.16, 0.1], [0, 0, -0.05], { c: 'body2' });
          body(S, 'slot', 'scr', 'box', [0.27, 0.05, 0.03], [0, 0, -0.11], { c: 'ink' });
          ln(S, 'K', scl(kh, -0.9), scl(kh, 2.4), { c: 'k', dash: false, label: 'k', lt: 1, lo: [0, 0, 0.12] });
        },
        steps: [
          {
            t: '문제',
            tex: R`{}^Ak = ${tvr(k)}^T,\quad \text{lead} = ${lead}\ \text{mm/rev},\quad \text{advance} = ${adv}\ \text{mm}`,
            html: R`<p>오른나사(right-handed, single-start)를 드라이버로 조입니다. 나사축 방향은 {A}에서 \(k=(${k.join(', ')})\)이고 나사 머리에서 공작물 쪽을 향합니다. 한 바퀴에 ${lead} mm 들어가고, ${adv} mm를 더 넣어야 합니다. 조이는 방향(오른손 법칙)을 +회전으로 합니다.</p>
            <p>1) z축을 k에 맞추는 보조각 α, β 2) 단위벡터 \(\hat k\) 3) 회전각 θ 4) \(R_{k,\theta}\)</p>`
          },
          {
            t: '① 보조각 α, β',
            assume: [R`\(\beta \in [0^\circ, 180^\circ]\): z축에서 k까지 기울인 각 (atan2의 첫 인자 \(\ge 0\))`],
            pre(S) { fr(S, 'Zf', I(), { label: '', name: '', size: 0.9, hl: 'z' }); },
            do(S) {
              fr(S, 'Zf', S0, { label: '', name: '', size: 0.9, hl: 'z' });
              if (Math.hypot(kxy[0], kxy[1]) > 1e-6) arc(S, 'al', { n: Z, u: X, ang: al, r: 0.5, label: 'α', grow: true });
              arc(S, 'be', { n: unit(cross(Z, kh)), u: Z, ang: be, r: 0.7, label: 'β', grow: true });
              ln(S, 'kp', O, scl(unit(kxy), 1.2), { c: 'mute' });
            },
            motion: [{ f: 'Zf', rot: [Yv, be], about: 'world' }, { f: 'Zf', rot: [Z, al], about: 'world' }],
            tex: R`\alpha = \text{atan2}(k_y,k_x) = \text{atan2}(${k[1]},${k[0]}) = ${deg(al)},\quad \beta = \text{atan2}\big(\sqrt{k_x^2+k_y^2},\,k_z\big) = \text{atan2}(\sqrt{${k[0] * k[0] + k[1] * k[1]}},\,${k[2]}) = ${deg(be)}`,
            html: R`<p>\(R_z(\alpha)R_y(\beta)\)를 하면 z축이 k와 겹칩니다(β: z축을 기울이는 각, α: 기울어진 방향의 방위각). 화면에서 Z축만 진하게 표시한 좌표계가 \(\Zh\) → k로 옮겨집니다.</p>
            <p>따라서 \(R_{k,\theta} = R_{z,\alpha}\,R_{y,\beta}\,R_{z,\theta}\,R_{y,-\beta}\,R_{z,-\alpha}\) (1.12절).</p>`
          },
          {
            t: '② 단위벡터 k̂',
            do(S) { delete S.frames.Zf; drop(S, 'arcs'); delete S.lines.kp; },
            tex: R`\hat k = \frac{1}{\sqrt{${k.map(x => (x < 0 ? '(' + x + ')' : x) + '^2').join('+')}}}${tvr(k)}^T = \frac{1}{${f(kn, 4)}}${tvr(k)}^T = ${tvr(kh, 4)}^T`,
            html: R`<p>회전 공식에는 크기 1인 축이 필요합니다.</p>`
          },
          {
            t: '③ 회전각 θ — 실제로 조여 보기',
            assume: [R`오른나사 + k가 공작물 쪽 → 오른손 법칙 방향 회전이 전진 → \(\theta > 0\)`, R`미끄러짐 없음: 회전량 ∝ 전진량 (한 바퀴 = lead)`],
            do(S) {
              S.frames.scr.T = mul(S0, tr(0, 0, adv), rz(th)); S.frames.D.T = RK;
              arc(S, 'th', { c: scl(kh, 1.1), n: kh, u: unit(cross(kh, Math.abs(kh[2]) < 0.9 ? Z : X)), ang: th, r: 0.5, label: 'θ', grow: true });
            },
            motion: [{ f: 'scr', rot: [Z, th], tr: [0, 0, adv], about: 'local', w: 2 }, { f: 'D', rot: [kh, th], about: 'world', with: true, w: 2 }],
            tex: R`\theta = \frac{\text{advance}}{\text{lead}}\times 360^\circ = \frac{${adv}}{${lead}}\times 360^\circ = +${f(th, 2)}^\circ`,
            html: R`<p>오른나사는 오른손 법칙 방향으로 돌리면 엄지 방향(= k, 공작물 쪽)으로 들어갑니다. 그래서 θ는 <b>양수</b>입니다. 화면의 나사가 k 둘레로 돌면서 ${adv} mm 전진하고, 좌표계 {B}가 같은 회전을 합니다.</p>`
          },
          {
            t: '④ 회전행렬 R_k,θ',
            do(S) { watch(S, 'D', 'A', '실시간 \\({}^A_B T\\)'); },
            tex: R`\begin{aligned}&c\theta = ${f(c, 4)},\ s\theta = ${f(s, 4)},\ v\theta = 1 - c\theta = ${f(v, 4)}\\ &R_{k,\theta} = \begin{bmatrix}k_x^2v\theta + c\theta & k_xk_yv\theta - k_zs\theta & k_xk_zv\theta + k_ys\theta\\ k_xk_yv\theta + k_zs\theta & k_y^2v\theta + c\theta & k_yk_zv\theta - k_xs\theta\\ k_xk_zv\theta - k_ys\theta & k_yk_zv\theta + k_xs\theta & k_z^2v\theta + c\theta\end{bmatrix} = ${tm(r, 4)}\end{aligned}`,
            html: R`<p>\(\hat k\)의 성분과 θ를 슬라이드 28 공식에 넣습니다. 예: \(r_{11} = k_x^2v\theta + c\theta = (${f(kh[0], 4)})^2(${f(v, 4)}) + (${f(c, 4)}) = ${f(r[0][0], 4)}\).</p>
            <p class="note">검산: 대각합 \(r_{11}+r_{22}+r_{33} = 1 + 2c\theta = ${f(1 + 2 * c, 4)}\).</p>`
          }
        ]
      };
    }
  });

  window.COURSE = window.COURSE || { chapters: [], upcoming: [] };
  window.COURSE.chapters.push({ id: 'ch1', title: 'Lecture 1 · Coordinate Transformation', short: 'Ch1 좌표 변환', sections });
  window.COURSE.upcoming = [
    { id: 'ch2', title: 'Lecture 2 · Forward Kinematics (D-H)', short: 'Ch2 순기구학' },
    { id: 'ch3', title: 'Lecture 3 · Inverse Kinematics', short: 'Ch3 역기구학' }
  ];
})();
