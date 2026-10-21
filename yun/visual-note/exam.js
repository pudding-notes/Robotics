/* =========================================================
   Mock exam — homework-style problems (English statements),
   new numbers on every draw, answer check + step-by-step solution
   ========================================================= */
(function () {
  'use strict';
  const R = String.raw;
  const { I, mul, rx, ry, rz, tr, rotK, inv, app, R3, p3, fromRp, RT, mv3, unit, norm, cross, add, sub, scl, atan2d, f, tm, tv, D } = M;
  const K = window.KIN;
  const { fr, pt, vec, ln, arc, body, drop, robot, chainT, dhTable } = K;
  const X = [1, 0, 0], Yv = [0, 1, 0], Z = [0, 0, 1], O = [0, 0, 0];

  const rnd = (a, b) => a + Math.floor(Math.random() * (b - a + 1));
  const pick = a => a[Math.floor(Math.random() * a.length)];
  const nz = (a, b) => { let v = 0; while (!v) v = rnd(a, b); return v; };
  const rd = (x, n) => M.round(x, n);
  const wrap = a => ((a + 540) % 360) - 180;
  const deg = x => f(x, 2) + '^\\circ';
  const pn = x => (x < 0 ? '(' + f(x, 4) + ')' : f(x, 4));          // parenthesise negatives inside formulas
  const AX = { x: rx, y: ry, z: rz };
  const AXV = { x: X, y: Yv, z: Z };
  const NOA = { x: 'n', y: 'o', z: 'a' };
  const head = (n, pts, body) => `<div class="exam-q"><div class="exam-h">Problem ${n} <span>(${pts} points)</span></div>${body}</div>`;
  const pmat = A => tm(A, 4);
  const sections = [];
  const sec = o => sections.push(o);

  /* ================= guide ================= */
  sec({
    id: 'x-guide', group: 'lecture', ref: '사용법', title: '모의고사 사용법',
    view: { pos: [3, -3.5, 2.6], target: [0, 0, 0.3] }, grid: [2, 0.5], scale: 0.6, fsize: 1,
    build() {
      return {
        init(S) { fr(S, 'A', I(), { size: 1 }); },
        steps: [{
          t: '이렇게 쓰세요',
          html: R`<p>과제 1·2와 같은 형식의 영어 문제 8개입니다. 각 문제는 열 때마다, 그리고 <b>🎲 새 문제</b>를 누를 때마다 숫자(때로는 회전축·로봇 구조)가 새로 뽑히므로 답을 외울 수 없습니다.</p>
          <ol style="margin:0 0 8px 18px;padding:0">
            <li>1단계(문제)만 보고 종이에 손으로 풉니다. 계산기는 fx-570ES PLUS 기준(Deg 모드).</li>
            <li>위의 <b>답안 입력 · 채점</b>에 값을 넣고 채점합니다.</li>
            <li>틀렸으면 2단계부터 풀이를 넘기며 어디서 갈라졌는지 찾습니다.</li>
          </ol>
          <p>범위: Problem 1–5 = 강의 1 (좌표 변환, 과제 1 유형), Problem 6–7 = 강의 2 (D-H, 순기구학), Problem 7–8 = 강의 3 (역기구학, 과제 2 유형).</p>
          <p class="note">각도는 도(°), 길이는 문제에 주어진 단위. 채점 허용 오차는 각도 ±0.3°~0.5°, 길이·원소 ±0.005 정도입니다(반올림된 문제 숫자 때문).</p>`
        }]
      };
    }
  });

  /* ================= P1: fixed vs moving axes ================= */
  sec({
    id: 'x-p1', group: 'practice', ref: 'Lecture 1 · slide 29 type', title: 'Problem 1 · Fixed vs. moving axes',
    view: { pos: [16, -15, 13], target: [2, 1, 2] }, grid: [10, 2], scale: 2, fsize: 2.2,
    params: {
      exam: true,
      rand() {
        const axes = ['x', 'y', 'z']; const a1 = pick(axes), a2 = pick(axes.filter(a => a !== a1));
        return { p: [rnd(1, 8), rnd(-4, 6), rnd(-3, 5)], a1, a2, t1: pick([90, -90]), t2: pick([90, -90]), d: [nz(-5, 6), nz(-5, 6), nz(-4, 7)] };
      }
    },
    build(P) {
      const { p, a1, a2, t1, t2, d } = P;
      const R1 = AX[a1](t1), R2 = AX[a2](t2), Tt = tr(...d);
      const Ta = [R1, mul(R2, R1), mul(Tt, R2, R1)];
      const Tb = [R1, mul(R1, Tt), mul(R1, Tt, R2)];
      const pa = app(Ta[2], p), pb = app(Tb[2], p);
      const nm = { axes: ['n', 'o', 'a'], sub: false, name: '{noa}' };
      const ds = `[${d.join(', ')}]^T`;
      const step = (S, T, tag, P2) => { S.frames.N.T = T; if (P2) pt(S, tag, app(T, p), `(${app(T, p).map(v => f(v, 0)).join(', ')})`, { c: 'ink', r: 0.8 }); };
      const rTex = (ax, t) => R`Rot(${ax}, ${t}^\circ)`;
      const ctr = scl(add(add(pa, pb), p), 1 / 3), ext = Math.max(8, ...[pa, pb, p].map(v => norm(sub(v, ctr)) * 2));
      return {
        view: { pos: add(ctr, [ext * 1.1, -ext * 1.25, ext * 1.0]), target: ctr },
        answers: [
          { label: R`(a) \(x\)`, val: pa[0], tol: 0.01, dp: 0 }, { label: R`(a) \(y\)`, val: pa[1], tol: 0.01, dp: 0 }, { label: R`(a) \(z\)`, val: pa[2], tol: 0.01, dp: 0 },
          { label: R`(b) \(x\)`, val: pb[0], tol: 0.01, dp: 0 }, { label: R`(b) \(y\)`, val: pb[1], tol: 0.01, dp: 0 }, { label: R`(b) \(z\)`, val: pb[2], tol: 0.01, dp: 0 }
        ],
        init(S) {
          fr(S, 'A', I(), { axes: ['x', 'y', 'z'], sub: false, name: '{xyz}', ghost: true });
          fr(S, 'N', I(), nm);
          pt(S, 'p', { f: 'N', p }, 'p');
          vec(S, 'np', { f: 'N' }, { f: 'N', p }, { w: 1, label: '' });
        },
        steps: [
          {
            t: 'Problem',
            html: head(1, 15, R`<p>A point \(p = [${p.join(', ')}]^T\) is attached to a frame \(F_{noa}\), which initially coincides with the reference frame \(F_{xyz}\). Find the coordinates \(p_{xyz}\) after each of the following sequences of transformations.</p>
              <p>(a) Rotation of \(${t1}^\circ\) about the ${a1}-axis \(\rightarrow\) rotation of \(${t2}^\circ\) about the ${a2}-axis \(\rightarrow\) translation of \(${ds}\). <i>All motions are relative to the fixed reference frame.</i></p>
              <p>(b) Rotation of \(${t1}^\circ\) about the ${NOA[a1]}-axis \(\rightarrow\) translation of \(${ds}\) along the noa axes \(\rightarrow\) rotation of \(${t2}^\circ\) about the ${NOA[a2]}-axis. <i>All motions are relative to the current (moving) frame.</i></p>`)
          },
          {
            t: '(a) 식 세우기: 고정축 → 왼쪽에 곱함',
            tex: R`p_{xyz} = Trans(${d.join(',')})\,${rTex(a2, t2)}\,${rTex(a1, t1)}\,p_{noa}`,
            html: R`<p>기준(고정) 축 기준 변환은 새 변환을 <b>왼쪽</b>에 곱하므로, 식에는 일어난 순서의 <b>역순</b>으로 적힙니다. 계산은 오른쪽부터 점에 하나씩 곱하는 것이 가장 빠릅니다.</p>`
          },
          {
            t: `(a) ① ${a1}축 ${t1}° 회전`,
            do(S) { step(S, Ta[0]); },
            motion: [{ f: 'N', rot: [AXV[a1], t1], about: 'world' }],
            tex: R`${rTex(a1, t1)}\,p = ${tmi(R3(R1))}${tv(p, 0)} = ${tv(app(R1, p), 0)}`
          },
          {
            t: `(a) ② ${a2}축 ${t2}° 회전`,
            do(S) { step(S, Ta[1]); },
            motion: [{ f: 'N', rot: [AXV[a2], t2], about: 'world' }],
            tex: R`${rTex(a2, t2)}\,${tv(app(R1, p), 0)} = ${tv(app(Ta[1], p), 0)}`,
            html: R`<p>흐린 <b>기준 ${a2}축</b> 둘레로 돕니다(이동한 noa의 축이 아님).</p>`
          },
          {
            t: '(a) ③ 이동',
            do(S) { step(S, Ta[2], 'ra', true); },
            motion: [{ f: 'N', tr: d, about: 'world' }],
            tex: R`p_{xyz} = ${tv(app(Ta[1], p), 0)} + ${tv(d, 0)} = \ha{${tv(pa, 0)}}`
          },
          {
            t: '(b) 식 세우기: 이동축 → 오른쪽에 곱함',
            pre(S) { S.frames.N.T = I(); delete S.pts.ra; },
            do(S) { S.frames.N.T = I(); delete S.pts.ra; },
            tex: R`p_{xyz} = ${rTex(NOA[a1], t1)}\,Trans(${d.join(',')})\,${rTex(NOA[a2], t2)}\,p_{noa}`,
            html: R`<p>현재(이동) 축 기준 변환은 <b>오른쪽</b>에 곱하므로 일어난 순서 <b>그대로</b> 적습니다. 처음에 n = x, o = y, a = z이므로 \(Rot(${NOA[a1]})\)은 \(R_${a1.toUpperCase()}\) 모양, \(Rot(${NOA[a2]})\)는 \(R_${a2.toUpperCase()}\) 모양의 행렬입니다.</p>`
          },
          {
            t: '(b) 오른쪽부터 점에 곱하기',
            do(S) { step(S, Tb[2], 'rb', true); },
            motion: [{ f: 'N', rot: [AXV[a1], t1], about: 'local' }, { f: 'N', tr: d, about: 'local' }, { f: 'N', rot: [AXV[a2], t2], about: 'local' }],
            dur: 3600,
            tex: R`\begin{aligned}${rTex(NOA[a2], t2)}\,p &= ${tv(app(R2, p), 0)}\\ Trans(\cdot)\,(\ldots) &= ${tv(app(R2, p), 0)} + ${tv(d, 0)} = ${tv(app(mul(Tt, R2), p), 0)}\\ ${rTex(NOA[a1], t1)}\,(\ldots) &= \ha{${tv(pb, 0)}}\end{aligned}`,
            html: R`<p>3D는 실제 일어난 순서(현재 축 기준 회전 → 현재 축 방향 이동 → 현재 축 기준 회전)대로 움직입니다. 행렬 계산은 오른쪽부터 해도 결과가 같습니다(곱셈의 결합법칙).</p>
            <p>(a)와 (b)의 답 \(${tv(pa, 0)}\) vs \(${tv(pb, 0)}\): 같은 동작 이름이라도 기준축이 다르면 결과가 다릅니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= P2: homogeneous transform + inverse ================= */
  sec({
    id: 'x-p2', group: 'practice', ref: 'Lecture 1 · slides 14, 21 type', title: 'Problem 2 · Homogeneous transform & inverse',
    view: { pos: [6, -14, 12], target: [3, 2, 1] }, grid: [14, 2], scale: 2.2, fsize: 2.4,
    params: {
      exam: true,
      rand() { return { th: pick([30, 45, 60, 90, 120, 135, 150, -30, -45, -60, -90, -120, -150]), p: [nz(-6, 9), nz(-6, 9), rnd(0, 4)], bp: [nz(-4, 6), nz(-4, 6), rnd(0, 3)] }; }
    },
    build(P) {
      const { th, p, bp } = P;
      const T = mul(tr(...p), rz(th)), AP = app(T, bp), Ti = inv(T), q = p3(Ti);
      return {
        answers: [
          { label: R`(b) \({}^AP_x\)`, val: AP[0], tol: 0.01, dp: 3 }, { label: R`(b) \({}^AP_y\)`, val: AP[1], tol: 0.01, dp: 3 }, { label: R`(b) \({}^AP_z\)`, val: AP[2], tol: 0.01, dp: 3 },
          { label: R`(c) \({}^BP_{AORG,x}\)`, val: q[0], tol: 0.01, dp: 3 }, { label: R`(c) \({}^BP_{AORG,y}\)`, val: q[1], tol: 0.01, dp: 3 }, { label: R`(c) \({}^BP_{AORG,z}\)`, val: q[2], tol: 0.01, dp: 3 }
        ],
        init(S) { fr(S, 'A', I()); fr(S, 'B', T); pt(S, 'P', { f: 'B', p: bp }, 'P'); vec(S, 'BP', { f: 'B' }, { f: 'B', p: bp }, { label: '<sup>B</sup>P', lt: 0.5 }); },
        steps: [
          {
            t: 'Problem',
            html: head(2, 10, R`<p>Frame {B} is obtained by rotating frame {A} by \(${th}^\circ\) about \(\hat Z_A\), and the origin of {B} is located at \({}^AP_{BORG} = [${p.join(', ')}]^T\).</p>
              <p>(a) Write the homogeneous transform \({}^A_BT\).<br>(b) A point is given in {B} as \({}^BP = [${bp.join(', ')}]^T\). Find \({}^AP\).<br>(c) Find \({}^B_AT\) without inverting a 4×4 matrix numerically, and report \({}^BP_{AORG}\).</p>`)
          },
          {
            t: '(a) ᴬ_BT',
            tex: R`{}^A_BT = \begin{bmatrix} R_Z(${th}^\circ) & {}^AP_{BORG}\\ 0\ 0\ 0 & 1\end{bmatrix} = ${pmat(T)}`,
            html: R`<p>회전부 3×3은 \(R_Z(${th}^\circ)\), 4열 위 세 칸은 {B} 원점 위치, 4행은 항상 \([0\ 0\ 0\ 1]\).</p>`
          },
          {
            t: '(b) ᴬP = R·ᴮP + p',
            do(S) { vec(S, 'AP', O, { f: 'B', p: bp }, { c: 'ink', w: 1.2, label: '<sup>A</sup>P' }); vec(S, 'AB', O, { f: 'B' }, { c: 'acc', label: 'P_BORG' }); },
            tex: R`{}^AP = R\,{}^BP + {}^AP_{BORG} = ${tv(mv3(R3(T), bp), 4)} + ${tv(p, 0)} = \ha{${tv(AP, 4)}}`,
            html: R`<p>4×4 대신 블록으로: 회전부 × 점 + 위치부. 계산기로는 MatA×MatB+MatC.</p>`
          },
          {
            t: '(c) ᴮ_AT = [Rᵀ, −Rᵀp]',
            do(S) { drop(S, 'vecs'); vec(S, 'BA', { f: 'B' }, O, { c: 'k', label: '<sup>B</sup>P_AORG' }); },
            tex: R`{}^B_AT = \begin{bmatrix}R^T & -R^T\,{}^AP_{BORG}\\ 0 & 1\end{bmatrix}:\quad R^T = ${tm(R3(Ti), 4)},\quad -R^T p = \ha{${tv(q, 4)}}`,
            html: R`<p>회전의 역은 전치, 위치부는 \(-R^Tp\) (1.4절). 이 벡터가 {B}에서 본 {A} 원점 위치 \({}^BP_{AORG}\)입니다. 검산: \({}^A_BT\,{}^B_AT = I\).</p>`
          }
        ]
      };
    }
  });

  /* ================= P3: X-Y-Z fixed angles ================= */
  sec({
    id: 'x-p3', group: 'practice', ref: 'HW1 type · X-Y-Z fixed angles', title: 'Problem 3 · Fixed-angle extraction',
    view: { pos: [3.2, -3.6, 2.6], target: [0, 0, 0.3] }, grid: [2, 0.5], scale: 0.6, fsize: 1,
    params: {
      exam: true,
      rand() { return { g: nz(-160, 160), b: nz(-65, 65), a: nz(-160, 160) }; }
    },
    build(P) {
      const { g, b, a } = P;
      const Rm = R3(mul(rz(a), ry(b), rx(g))).map(r => r.map(v => rd(v, 4)));
      const r = Rm;
      const cb = Math.hypot(r[0][0], r[1][0]);
      const B = atan2d(-r[2][0], cb), A = atan2d(r[1][0], r[0][0]), G = atan2d(r[2][1], r[2][2]);
      return {
        answers: [{ label: R`\(\gamma\)`, val: G, unit: '°', tol: 0.3, dp: 2, mod: 360 }, { label: R`\(\beta\)`, val: B, unit: '°', tol: 0.3, dp: 2 }, { label: R`\(\alpha\)`, val: A, unit: '°', tol: 0.3, dp: 2, mod: 360 }],
        init(S) { fr(S, 'A', I(), { ghost: true }); fr(S, 'B', I()); },
        steps: [
          {
            t: 'Problem',
            html: head(3, 10, R`<p>The orientation of frame {B} relative to frame {A} is described by the rotation matrix</p>
              <p style="text-align:center">\({}^A_BR = ${tm3(Rm)}\)</p>
              <p>Frame {B} was obtained from {A} by a rotation \(\gamma\) about \(\hat X_A\), then \(\beta\) about \(\hat Y_A\), then \(\alpha\) about \(\hat Z_A\) (X-Y-Z fixed angles). Find \(\gamma, \beta, \alpha\), assuming \(-90^\circ < \beta < 90^\circ\).</p>`)
          },
          {
            t: '① 식 세우기',
            tex: R`{}^A_BR = R_Z(\alpha)R_Y(\beta)R_X(\gamma) = \begin{bmatrix} c_\alpha c_\beta & \cdots & \cdots\\ s_\alpha c_\beta & \cdots & \cdots\\ -s_\beta & c_\beta s_\gamma & c_\beta c_\gamma\end{bmatrix}`,
            html: R`<p>고정축이므로 왼쪽에 곱해 \(R_ZR_YR_X\). 각도가 적게 섞인 <b>1열과 3행</b>만 쓰면 됩니다.</p>`
          },
          {
            t: '② β',
            do(S) { S.frames.B.T = fromRp(R3(mul(ry(B))), O); },
            motion: [{ f: 'B', rot: [Yv, B], about: 'world' }],
            assume: [R`\(-90^\circ<\beta<90^\circ\) → \(c_\beta>0\) → 양의 제곱근`],
            tex: R`\beta = \text{atan2}\big(-r_{31},\ \sqrt{r_{11}^2+r_{21}^2}\big) = \text{atan2}\big(${f(-r[2][0], 4)},\ \sqrt{${pn(r[0][0])}^2+${pn(r[1][0])}^2}\big) = \ha{${deg(B)}}`,
            html: R`<p>계산기: Pol(√(r₁₁²+r₂₁²), −r₃₁)의 θ. (3D는 각을 하나씩 확인용으로 보여 줍니다.)</p>`
          },
          {
            t: '③ α',
            do(S) { S.frames.B.T = fromRp(R3(mul(rz(A), ry(B))), O); },
            motion: [{ f: 'B', rot: [Z, A], about: 'world' }],
            tex: R`\alpha = \text{atan2}\Big(\frac{r_{21}}{c_\beta}, \frac{r_{11}}{c_\beta}\Big) = \text{atan2}(${f(r[1][0], 4)},\ ${f(r[0][0], 4)}) = \ha{${deg(A)}}`,
            html: R`<p>\(c_\beta > 0\)이므로 나누지 않아도 부호가 같아 같은 각입니다.</p>`
          },
          {
            t: '④ γ와 검증',
            do(S) { S.frames.B.T = fromRp(Rm, O); },
            tex: R`\gamma = \text{atan2}(r_{32},\ r_{33}) = \text{atan2}(${f(r[2][1], 4)},\ ${f(r[2][2], 4)}) = \ha{${deg(G)}}`,
            html: R`<p>검증: \(R_Z(${f(A, 1)}^\circ)R_Y(${f(B, 1)}^\circ)R_X(${f(G, 1)}^\circ)\)을 곱하면 주어진 행렬이 다시 나옵니다(3D의 {B}가 그 자세).</p>`
          }
        ]
      };
    }
  });
  function tmi(A) { return R`\begin{bmatrix}${A.map(r => r.map(v => f(v, 0)).join('&')).join('\\\\')}\end{bmatrix}`; }
  function tm3(A) { return R`\begin{bmatrix}${A.map(r => r.map(v => f(v, 4)).join('&')).join('\\\\')}\end{bmatrix}`; }

  /* ================= P4: angle-axis ================= */
  sec({
    id: 'x-p4', group: 'practice', ref: 'HW1 type · equivalent angle-axis', title: 'Problem 4 · Rotation about an arbitrary axis',
    view: { pos: [3.4, -3.4, 2.4], target: [0.2, 0.2, 0.4] }, grid: [2, 0.5], scale: 0.6, fsize: 1,
    params: {
      exam: true,
      rand() { let k; do { k = [rnd(-2, 3), rnd(-2, 3), rnd(-1, 3)]; } while (k.filter(v => v !== 0).length < 2); return { k, th: pick([30, 45, 60, 90, 120, 135, 150, -60, -90, -120]) }; }
    },
    build(P) {
      const { k, th } = P;
      const n = norm(k), kh = unit(k), c = Math.cos(th * D), s = Math.sin(th * D), v = 1 - c;
      const Rk = R3(rotK(kh, th));
      const [kx, ky, kz] = kh;
      return {
        answers: [{ label: R`\(r_{11}\)`, val: Rk[0][0], tol: 0.005, dp: 4 }, { label: R`\(r_{12}\)`, val: Rk[0][1], tol: 0.005, dp: 4 }, { label: R`\(r_{13}\)`, val: Rk[0][2], tol: 0.005, dp: 4 }],
        init(S) { fr(S, 'A', I(), { ghost: true }); fr(S, 'B', I()); ln(S, 'k', scl(kh, -0.6), scl(kh, 1.7), { c: 'k', dash: false, label: 'k', lt: 1 }); },
        steps: [
          {
            t: 'Problem',
            html: head(4, 10, R`<p>Frame {B} initially coincides with frame {A}. It is then rotated by \(\theta = ${th}^\circ\) about an axis through the origin whose direction in {A} is \(k = [${k.join(', ')}]^T\) (right-hand rule).</p>
              <p>(a) Find the unit vector \(\hat K\).<br>(b) Find the rotation matrix \({}^A_BR = R_K(\theta)\) and report the first row \((r_{11}, r_{12}, r_{13})\).</p>`)
          },
          {
            t: '(a) 단위벡터',
            tex: R`|k| = \sqrt{${k.map(pn).map(x => x + '^2').join('+')}} = ${f(n, 4)},\qquad \hat K = \frac{k}{|k|} = ${tv(kh, 4)}`,
            html: R`<p>공식에는 반드시 <b>단위벡터</b>를 넣어야 합니다.</p>`
          },
          {
            t: '(b) 공식과 c, s, v',
            tex: R`R_K(\theta) = \begin{bmatrix} k_xk_xv+c & k_xk_yv-k_zs & k_xk_zv+k_ys\\ k_xk_yv+k_zs & k_yk_yv+c & k_yk_zv-k_xs\\ k_xk_zv-k_ys & k_yk_zv+k_xs & k_zk_zv+c\end{bmatrix},\quad c = ${f(c, 4)},\ s = ${f(s, 4)},\ v = 1-c = ${f(v, 4)}`,
            html: R`<p>대각선은 \(k^2v + c\), 비대각선은 \(kk\,v \pm k\,s\). ± 부호는 외적 행렬 모양(위 오른쪽 \(-k_z\), \(+k_y\), …)을 따릅니다.</p>`
          },
          {
            t: '(b) 1행 계산',
            do(S) { S.frames.B.T = fromRp(Rk, O); },
            motion: [{ f: 'B', rot: [kh, th], about: 'world' }],
            tex: R`\begin{aligned} r_{11} &= ${pn(kx)}^2(${f(v, 4)}) + ${pn(c)} = \ha{${f(Rk[0][0], 4)}}\\ r_{12} &= ${pn(kx)}${pn(ky)}(${f(v, 4)}) - ${pn(kz)}(${f(s, 4)}) = \ha{${f(Rk[0][1], 4)}}\\ r_{13} &= ${pn(kx)}${pn(kz)}(${f(v, 4)}) + ${pn(ky)}(${f(s, 4)}) = \ha{${f(Rk[0][2], 4)}}\end{aligned}`,
            html: R`<p>전체 행렬: \(${tm3(Rk)}\). 검산: 각 행·열의 제곱합 = 1, 그리고 \(R\hat K = \hat K\)(회전축은 움직이지 않음).</p>`
          }
        ]
      };
    }
  });

  /* ================= P5: cylindrical robot ================= */
  sec({
    id: 'x-p5', group: 'practice', ref: 'HW1 Problem 1 type', title: 'Problem 5 · Cylindrical robot',
    view: { pos: [11, -12, 10], target: [1, 1, 3] }, grid: [10, 2], scale: 1.6, fsize: 1.8,
    params: {
      exam: true,
      rand() { return { x: nz(-6, 6), y: nz(-6, 6), z: rnd(2, 7) }; }
    },
    build(P) {
      const { x, y, z } = P;
      const al = atan2d(y, x), r = Math.hypot(x, y);
      const T = mul(tr(0, 0, z), rz(al), tr(r, 0, 0));
      const sz = Math.max(r, z, 4);
      return {
        view: { pos: [x / 2 + sz * 1.6, y / 2 - sz * 1.9, z / 2 + sz * 1.3], target: [x / 2, y / 2, z / 2] },
        answers: [{ label: R`\(\alpha\)`, val: al, unit: '°', tol: 0.3, dp: 2, mod: 360 }, { label: R`\(r\)`, val: r, tol: 0.01, dp: 3 }, { label: R`\(l\)`, val: z, tol: 0.01, dp: 3 }],
        init(S) {
          fr(S, 'A', I());
          body(S, 'col', 'A', 'cyl', [0.22, z + 1.5], [0, 0, (z + 1.5) / 2]);
          pt(S, 'goal', [x, y, z], `goal (${x}, ${y}, ${z})`, { c: 'k', r: 1.4 });
        },
        steps: [
          {
            t: 'Problem',
            html: head(5, 10, R`<p>A cylindrical robot places its hand frame {H} by the following sequence of transformations, all relative to the fixed reference frame: \(T_1 = Trans(\hat x, r)\) (extend along x), then \(T_2 = Rot(\hat z, \alpha)\), then \(T_3 = Trans(\hat z, l)\) (lift along z).</p>
              <p>(a) Find the total transform \({}^0_HT\) in terms of \(r, \alpha, l\).<br>(b) The origin of the hand must be placed at \([${x}, ${y}, ${z}]^T\). Find the joint variables \(\alpha\), \(r\,(>0)\) and \(l\).</p>`)
          },
          {
            t: '(a) T = T₃T₂T₁',
            tex: R`{}^0_HT = Trans(\hat z,l)\,Rot(\hat z,\alpha)\,Trans(\hat x,r) = \begin{bmatrix} c_\alpha & -s_\alpha & 0 & r\,c_\alpha\\ s_\alpha & c_\alpha & 0 & r\,s_\alpha\\ 0&0&1&l\\0&0&0&1\end{bmatrix}`,
            html: R`<p>고정 프레임 기준이므로 새 변환을 왼쪽에 곱합니다. 위치부는 극좌표 \((r, \alpha)\)에 높이 \(l\)을 붙인 원통 좌표입니다.</p>`
          },
          {
            t: '(b) 관절 변수',
            do(S) { fr(S, 'H', T, { label: 'H', size: 0.8 }); body(S, 'arm', 'H', 'box', [r, 0.22, 0.22], [-r / 2, 0, 0], { c: 'body2' }); arc(S, 'al', { c: [0, 0, z], n: Z, u: X, ang: al, r: 1.2, label: 'α = ' + f(al, 2) + '°', grow: true }); },
            tex: R`l = z = ${z},\qquad r = \sqrt{${pn(x)}^2+${pn(y)}^2} = \ha{${f(r, 4)}},\qquad \alpha = \text{atan2}(${y},\ ${x}) = \ha{${deg(al)}}`,
            html: R`<p>위치부 \((r c_\alpha, r s_\alpha, l) = (${x}, ${y}, ${z})\)에서 바로 나옵니다. \(r>0\)이므로 \(\alpha\)는 atan2(y, x) 하나로 정해집니다. 계산기: Pol(${x}, ${y}) → r과 θ를 한 번에.</p>`
          }
        ]
      };
    }
  });

  /* ================= P6: D-H + FK of a shown manipulator ================= */
  const TEMPL = {
    scara: {
      name: 'R–R–P–R (SCARA type)',
      mk: () => { const L1 = pick([0.4, 0.45, 0.5, 0.6]), L2 = pick([0.3, 0.35, 0.4]); return { L: [L1, L2], q: [nz(-120, 120), nz(-130, 130), rnd(5, 30) / 100, nz(-90, 90)] }; },
      rows: L => [{ al: 0, a: 0, d: 0, th: 0, type: 'R' }, { al: 0, a: L[0], d: 0, th: 0, type: 'R' }, { al: 180, a: L[1], d: 0, th: 0, type: 'P' }, { al: 0, a: 0, d: 0, th: 0, type: 'R' }],
      text: (L, q) => R`Joints 1, 2 and 4 are revolute about vertical axes and joint 3 is prismatic, sliding <b>downward</b>. The distance between axes 1 and 2 is \(L_1 = ${L[0]}\) m and between axes 2 and 3 is \(L_2 = ${L[1]}\) m. Frame {4} is attached to the tool flange (origin on the joint-4 axis, at the end of the slider).`,
      qtext: q => R`\theta_1 = ${q[0]}^\circ,\ \theta_2 = ${q[1]}^\circ,\ d_3 = ${q[2]}\ \text{m},\ \theta_4 = ${q[3]}^\circ`,
      sym: R`{}^0_4T = \begin{bmatrix} c_{12-4} & s_{12-4} & 0 & L_1c_1 + L_2c_{12}\\ s_{12-4} & -c_{12-4} & 0 & L_1s_1 + L_2s_{12}\\ 0&0&-1&-d_3\\0&0&0&1\end{bmatrix}`,
      draw: { base: tr(0, 0, 0.5), tool: 0.06, pedestal: [0.06, 0.5], fs: 0.14, w: 0.04, jr: 0.05, jh: 0.12 }
    },
    stanford: {
      name: 'R–R–P (spherical / Stanford type)',
      mk: () => ({ L: [], q: [nz(-150, 150), nz(-150, 150), rnd(20, 60) / 100] }),
      rows: () => [{ al: 0, a: 0, d: 0, th: 0, type: 'R' }, { al: -90, a: 0, d: 0, th: 0, type: 'R' }, { al: 90, a: 0, d: 0, th: 0, type: 'P' }],
      text: () => R`Joint 1 is revolute about the vertical axis, joint 2 is revolute about a horizontal axis that intersects axis 1, and joint 3 is prismatic along a direction perpendicular to axis 2 and passing through the intersection point. Frame {3} is attached at the tip of the prismatic link; \(d_3\) is the extension measured from the intersection point.`,
      qtext: q => R`\theta_1 = ${q[0]}^\circ,\ \theta_2 = ${q[1]}^\circ,\ d_3 = ${q[2]}\ \text{m}`,
      sym: R`{}^0_3T = \begin{bmatrix} c_1c_2 & -s_1 & c_1s_2 & d_3c_1s_2\\ s_1c_2 & c_1 & s_1s_2 & d_3s_1s_2\\ -s_2 & 0 & c_2 & d_3c_2\\ 0&0&0&1\end{bmatrix}`,
      draw: { base: tr(0, 0, 0.45), tool: 0.06, pedestal: [0.06, 0.45], fs: 0.14, w: 0.04, jr: 0.05, jh: 0.12 }
    },
    rrr: {
      name: 'R–R–R (planar 2-link arm on a rotating base)',
      mk: () => { const L2 = pick([0.3, 0.35, 0.4, 0.45]), L3 = pick([0.2, 0.25, 0.3]); return { L: [L2, L3], q: [nz(-150, 150), nz(-80, 80), nz(-120, 120)] }; },
      rows: L => [{ al: 0, a: 0, d: 0, th: 0, type: 'R' }, { al: -90, a: 0, d: 0, th: 0, type: 'R' }, { al: 0, a: L[0], d: 0, th: 0, type: 'R' }],
      tipX: L => L[1],
      text: L => R`Joint 1 is revolute about the vertical axis. Joints 2 and 3 are parallel to each other, perpendicular to joint 1, and axis 2 intersects axis 1. The upper arm (between axes 2 and 3) has length \(L_2 = ${L[0]}\) m and the forearm has length \(L_3 = ${L[1]}\) m from axis 3 to the tool point T, which lies on \(\hat X_3\).`,
      qtext: q => R`\theta_1 = ${q[0]}^\circ,\ \theta_2 = ${q[1]}^\circ,\ \theta_3 = ${q[2]}^\circ`,
      sym: R`{}^0_3T = \begin{bmatrix} c_1c_{23} & -c_1s_{23} & -s_1 & L_2c_1c_2\\ s_1c_{23} & -s_1s_{23} & c_1 & L_2s_1c_2\\ -s_{23} & -c_{23} & 0 & -L_2s_2\\ 0&0&0&1\end{bmatrix},\quad {}^0P_T = {}^0_3T\begin{bmatrix}L_3\\0\\0\\1\end{bmatrix}`,
      draw: { base: tr(0, 0, 0.45), tool: 0, pedestal: [0.06, 0.45], fs: 0.14, w: 0.04, jr: 0.05, jh: 0.12 }
    }
  };
  sec({
    id: 'x-p6', group: 'practice', ref: 'HW2 Problem 1–2 type · D-H', title: 'Problem 6 · D-H parameters & forward kinematics',
    view: { pos: [1.6, -1.9, 1.6], target: [0.15, 0.1, 0.4] }, grid: [1.5, 0.25], scale: 0.35, fsize: 1,
    params: {
      exam: true,
      rand() { const kind = pick(Object.keys(TEMPL)); return Object.assign({ kind }, TEMPL[kind].mk()); }
    },
    build(P) {
      const tp = TEMPL[P.kind], rows = tp.rows(P.L), q = P.q, n = rows.length;
      const T = chainT(rows, q).Ts[n];
      const tipL = tp.tipX ? tp.tipX(P.L) : 0;
      const pos = app(T, [tipL, 0, 0]);
      const qd = q.map((v, i) => rows[i].type === 'P' ? v : v);
      const draw = (S, qq, o) => robot(S, rows, qq, Object.assign({ noDrag: true }, tp.draw, o || {}));
      const home = rows.map(r => (r.type === 'P' ? 0.25 : 0));
      const what = tp.tipX ? 'the tool point T' : `the origin of frame {${n}}`;
      const tipPt = (S, qq) => { if (tp.tipX) { const Tq = mul(tp.draw.base, chainT(rows, qq).Ts[n]); body(S, 'tipl', 'F' + n, 'box', [tipL, 0.03, 0.03], [tipL / 2, 0, 0], { c: 'j3' }); pt(S, 'tip', app(Tq, [tipL, 0, 0]), 'T', { c: 'k', r: 0.4 }); } };
      const mats = rows.map((r, i) => {
        const th = r.type === 'R' ? `\\theta_${i + 1}` : (r.th ? f(r.th, 0) + '^\\circ' : '0');
        const d = r.type === 'P' ? `d_${i + 1}` : f(r.d, 3);
        return R`{}^{${i}}_{${i + 1}}T:\ \alpha_{${i}} = ${f(r.al, 0)}^\circ,\ a_{${i}} = ${f(r.a, 3)},\ d_{${i + 1}} = ${d},\ \theta_{${i + 1}} = ${th}`;
      });
      return {
        answers: [{ label: R`\(x\)`, val: pos[0], unit: 'm', tol: 0.005, dp: 4 }, { label: R`\(y\)`, val: pos[1], unit: 'm', tol: 0.005, dp: 4 }, { label: R`\(z\)`, val: pos[2], unit: 'm', tol: 0.005, dp: 4 }],
        init(S) { draw(S, home, { noFrames: true }); tipPt(S, home); },
        steps: [
          {
            t: 'Problem',
            html: head(6, 15, R`<p>The ${tp.name} manipulator shown in the 3D view (home pose) is described as follows. ${tp.text(P.L, q)}</p>
              <p>(a) Assign link frames {0} through {${n}} (frame {0} coincides with frame {1} when the first joint variable is zero) and determine the link parameters \(a_{i-1}, \alpha_{i-1}, d_i, \theta_i\) using the D-H convention.<br>
              (b) Determine all link transformations \({}^{i-1}_{\ \ i}T\) and \({}^0_${n}T\).<br>
              (c) For \(${tp.qtext(q)}\), compute the position of ${what} with respect to frame {0}.</p>`)
          },
          {
            t: '(a) 프레임과 D-H 표',
            do(S) { drop(S, 'bodies'); drop(S, 'pts'); draw(S, home); tipPt(S, home); },
            tex: dhTable(rows),
            html: R`<p>한 가지 표준 배치입니다. Ẑ 방향을 반대로 잡는 등 다른 배치도 규칙만 지키면 정답이지만, (c)의 위치 값은 배치와 상관없이 같아야 합니다.</p>
            <p class="note">${P.kind === 'scara' ? '직동 관절 3이 아래로 내려가므로 Ẑ₃를 아래로 잡아 α₂ = 180°.' : P.kind === 'stanford' ? '축 1과 2가 만나므로 a₁ = 0, α₁ = −90°. 축 3(직동 방향)은 축 2에 수직이고 같은 점을 지나므로 a₂ = 0, α₂ = +90°.' : '축 1과 2가 만나므로 a₁ = 0, α₁ = −90°. 축 2와 3은 평행하므로 α₂ = 0, a₂ = L₂. 공구점 T는 {3}의 X축 위 L₃.'}</p>`
          },
          {
            t: '(b) 링크 변환과 전체 변환',
            tex: R`\begin{gathered}${mats.join('\\\\')}\\[4pt] ${tp.sym}\end{gathered}`,
            html: R`<p>각 행을 \({}^{i-1}_{\ \ i}T = R_X(\alpha_{i-1})D_X(a_{i-1})R_Z(\theta_i)D_Z(d_i)\) 공식에 대입하고, 왼쪽 두 개부터 곱했습니다. α가 0°, ±90°, 180°라 cos α, sin α가 0, ±1이 되어 행 섞기만으로 곱할 수 있습니다.</p>`
          },
          {
            t: '(c) 숫자 대입',
            do(S) { drop(S, 'bodies'); drop(S, 'pts'); draw(S, q); tipPt(S, q); },
            tex: R`${tp.qtext(q)}\ \Rightarrow\ {}^0_${n}T = ${pmat(T)}${tp.tipX ? R`,\quad {}^0P_T = ${tv(pos, 4)}` : ''}\ \Rightarrow\ \ha{(x, y, z) = (${pos.map(v => f(v, 4)).join(',\\ ')})}`,
            html: R`<p>3D는 주어진 관절값으로 움직인 자세입니다(높이는 받침대 위 {0} 기준).</p>`
          }
        ]
      };
    }
  });

  /* ================= P7: 3-DOF arm IK (HW2 P2) ================= */
  sec({
    id: 'x-p7', group: 'practice', ref: 'HW2 Problem 2 type', title: 'Problem 7 · 3-DOF arm: frames, transforms, inverse kinematics',
    view: { pos: [1.5, -1.9, 1.4], target: [0.2, 0.1, 0.4] }, grid: [1.5, 0.25], scale: 0.3, fsize: 1,
    params: {
      exam: true,
      rand() { const L1 = pick([0.35, 0.4, 0.45, 0.5]), L2 = pick([0.2, 0.25, 0.3]); return { L1, L2, t: [nz(-80, 80), rnd(10, 80), nz(-90, 60)] }; }
    },
    build(P) {
      const { L1, L2, t } = P;
      const rows = [{ al: 0, a: 0, d: 0, th: 0, type: 'R' }, { al: 90, a: L1, d: 0, th: 0, type: 'R' }, { al: 0, a: L2, d: 0, th: 0, type: 'R' }];
      const T = chainT(rows, t).Ts[3];
      const Pg = p3(T).map(v => rd(v, 3)), Rg = R3(T).map(r => r.map(v => rd(v, 4)));
      const [x, y, z] = Pg, rho = Math.hypot(x, y), s2 = z / L2, c2 = (rho - L1) / L2;
      const t2 = atan2d(s2, c2), t1 = atan2d(y, x), t23 = atan2d(Rg[2][0], Rg[2][1]), t3 = wrap(t23 - t2);
      const base = tr(0, 0, 0.3);
      const draw = (S, q, o) => robot(S, rows, q, Object.assign({ base, tool: 0.08, pedestal: [0.05, 0.3], fs: 0.12, w: 0.025, jr: 0.035, jh: 0.08, noDrag: true }, o || {}));
      const tgt = S => { fr(S, 'TG', mul(base, fromRp(Rg, Pg)), { label: '', name: 'goal {3}', size: 0.1, ghost: true }); };
      return {
        answers: [{ label: R`\(\theta_1\)`, val: t1, unit: '°', tol: 0.4, dp: 2, mod: 360 }, { label: R`\(\theta_2\)`, val: t2, unit: '°', tol: 0.4, dp: 2 }, { label: R`\(\theta_3\)`, val: t3, unit: '°', tol: 0.4, dp: 2, mod: 360 }],
        init(S) { draw(S, [0, 0, 0], { noFrames: true }); tgt(S); },
        steps: [
          {
            t: 'Problem',
            html: head(7, 20, R`<p>The arm with three degrees of freedom shown in the 3D view has joints 1 and 2 perpendicular, and joints 2 and 3 parallel. The distance from axis 1 to axis 2 is \(L_1 = ${L1}\) m and from axis 2 to axis 3 is \(L_2 = ${L2}\) m. The joint angles \(\theta_2, \theta_3\) are measured counterclockwise when viewed from the front, as in the homework figure.</p>
              <p>(a) Assign link frames {0} through {3} and sketch how each frame is attached.<br>
              (b) Derive \({}^0_1T, {}^1_2T, {}^2_3T\) and \({}^0_3T\).<br>
              (c) The desired position of frame {3} is \(P = [${Pg.join(',\\ ')}]^T\) m. Find \(\theta_1\) and \(\theta_2\) (take the solution with \(L_1 + L_2\cos\theta_2 > 0\)).<br>
              (d) The desired orientation of {3} with respect to {0} is given by the rotation matrix below. Find \(\theta_3\).</p><p style="text-align:center">\({}^0_3R = ${tm3(Rg)}\)</p>`)
          },
          {
            t: '(a)(b) D-H 표와 ⁰₃T',
            do(S) { drop(S, 'bodies'); drop(S, 'pts'); draw(S, [0, 0, 0]); tgt(S); },
            tex: R`${dhTable(rows)}\qquad {}^0_3T = \begin{bmatrix} c_1c_{23} & -c_1s_{23} & s_1 & c_1(L_1+L_2c_2)\\ s_1c_{23} & -s_1s_{23} & -c_1 & s_1(L_1+L_2c_2)\\ s_{23} & c_{23} & 0 & L_2s_2\\ 0&0&0&1\end{bmatrix}`,
            html: R`<p>Ẑ₂, Ẑ₃를 정면에서 볼 때 종이에서 나오는 방향으로 잡으면 θ₂, θ₃의 + 방향이 반시계가 되고 α₁ = +90°입니다. \({}^0_2T = {}^0_1T\,{}^1_2T\)를 먼저 구하고 \({}^2_3T\)를 곱합니다.</p>`
          },
          {
            t: '(c) θ₂, θ₁',
            do(S) { drop(S, 'bodies'); drop(S, 'pts'); draw(S, [t1, t2, 0], { known: [1, 2], joint: [1, 2] }); tgt(S); },
            assume: [R`\(\rho = L_1 + L_2c_2 > 0\) → \(\rho = +\sqrt{x^2+y^2}\)`],
            tex: R`\begin{aligned} s_2 &= \frac{z}{L_2} = \frac{${z}}{${L2}} = ${f(s2, 4)},\qquad c_2 = \frac{\sqrt{x^2+y^2} - L_1}{L_2} = \frac{${f(rho, 4)} - ${L1}}{${L2}} = ${f(c2, 4)}\\ \theta_2 &= \text{atan2}(s_2, c_2) = \ha{${deg(t2)}},\qquad \theta_1 = \text{atan2}(y, x) = \text{atan2}(${y},\ ${x}) = \ha{${deg(t1)}}\end{aligned}`,
            html: R`<p>x² + y² = ρ²에서 θ₁을 지우고 c₂를, z에서 s₂를 얻은 뒤 atan2 → 후보 고르기가 필요 없습니다. 검산: \(c_2^2+s_2^2 = ${f(c2 * c2 + s2 * s2, 4)} \approx 1\).</p>`
          },
          {
            t: '(d) θ₃',
            do(S) { drop(S, 'bodies'); drop(S, 'pts'); draw(S, [t1, t2, t3], { joint: [3] }); tgt(S); },
            tex: R`\begin{aligned}s_{23} &= r_{31} = ${f(Rg[2][0], 4)},\ \ c_{23} = r_{32} = ${f(Rg[2][1], 4)}\ \Rightarrow\ \theta_2+\theta_3 = ${deg(t23)}\\ \theta_3 &= ${f(t23, 2)}^\circ - ${f(t2, 2)}^\circ = \ha{${deg(t3)}}\end{aligned}`,
            html: R`<p>검산: 3열 \((s_1, -c_1, 0) = (${f(Rg[0][2], 4)}, ${f(Rg[1][2], 4)}, 0)\)에서 θ₁ = atan2(r₁₃, −r₂₃) = ${f(atan2d(Rg[0][2], -Rg[1][2]), 2)}° — (c)와 일치해야 합니다. 3D의 {3}이 흐린 목표 프레임과 겹칩니다.</p>`
          }
        ]
      };
    }
  });

  /* ================= P8: Pieper ================= */
  const CONSTS = [[0.4, 0.3, 0.1], [0.45, 0.25, 0.12], [0.5, 0.3, 0.1], [0.4, 0.35, 0.05], [0.35, 0.3, 0.1]];
  function pFK(c, q) {
    const [a2, a3, d4] = c;
    const A = mul(rz(q[0]), rx(-90), rz(q[1])), R03 = R3(mul(rz(q[0]), rx(-90), rz(q[1] + q[2])));
    const E = app(A, [a2, 0, 0]);
    const W = add(E, mv3(R03, [a3, -d4, 0]));
    const R06 = R3(mul(fromRp(R03, O), rz(q[3]), ry(q[4]), rz(q[5])));
    return { E, W, R03, R06 };
  }
  const mulR = (a, b) => [0, 1, 2].map(i => [0, 1, 2].map(j => a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]));
  function pIK(c, Pt, Rt) {
    const [a2, a3, d4] = c, [x, y, z] = Pt, rr = x * x + y * y + z * z;
    const A = 2 * a2 * a3, B = 2 * a2 * d4, C = rr - a2 * a2 - a3 * a3 - d4 * d4, disc = A * A + B * B - C * C;
    const t1 = atan2d(y, x), g1 = Math.hypot(x, y);
    const cands = [1, -1].map(sg => {
      const u = (B + sg * Math.sqrt(Math.max(0, disc))) / (A + C);
      const t3 = wrap(2 * Math.atan(u) / D);
      const f1 = a2 + a3 * Math.cos(t3 * D) + d4 * Math.sin(t3 * D), f2 = a3 * Math.sin(t3 * D) - d4 * Math.cos(t3 * D);
      const t2 = wrap(atan2d(-z, g1) - atan2d(f2, f1));
      const R03 = R3(mul(rz(t1), rx(-90), rz(t2 + t3))), R36 = mulR(RT(R03), Rt);
      const t5 = atan2d(Math.hypot(R36[2][0], R36[2][1]), R36[2][2]), t4 = atan2d(R36[1][2], R36[0][2]), t6 = atan2d(R36[2][1], -R36[2][0]);
      return { u, t1, t2, t3, t4, t5, t6, f1, f2, R03, R36 };
    });
    return { rr, A, B, C, disc, g1, cands };
  }
  sec({
    id: 'x-p8', group: 'practice', ref: 'HW2 Problem 3 type', title: "Problem 8 · Pieper's method",
    view: { pos: [1.3, -1.6, 1.1], target: [0.2, 0.1, 0.1] }, grid: [1.5, 0.25], scale: 0.25, fsize: 1,
    params: {
      exam: true,
      rand() {
        for (let n = 0; n < 400; n++) {
          const c = pick(CONSTS);
          const q = [rnd(-16, 16) * 5, rnd(1, 17) * 5, rnd(-24, 10) * 5, rnd(-30, 30) * 5, rnd(4, 32) * 5, rnd(-30, 30) * 5];
          const k = pFK(c, q), Pt = k.W.map(v => rd(v, 4)), Rt = k.R06.map(r => r.map(v => rd(v, 4)));
          const s = pIK(c, Pt, Rt);
          const ok = s.cands.filter(z => z.t2 >= -0.3 && z.t2 <= 90.3 && z.t1 >= -90 && z.t1 <= 90);
          if (s.disc > 1e-5 && s.g1 > 0.12 && ok.length === 1 && [ok[0].t1, ok[0].t2, ok[0].t3, ok[0].t4, ok[0].t5, ok[0].t6].every((v, i) => Math.abs(wrap(v - q[i])) < 0.2)) return { c, q };
        }
        return { c: [0.4, 0.3, 0.1], q: [30, 0, 0, -90, 90, 60] };
      }
    },
    build(P) {
      const c = P.c, [a2, a3, d4] = c, k = pFK(c, P.q);
      const Pt = k.W.map(v => rd(v, 4)), Rt = k.R06.map(r => r.map(v => rd(v, 4)));
      const s = pIK(c, Pt, Rt);
      const gi = s.cands.findIndex(z => z.t2 >= -0.3 && z.t2 <= 90.3 && z.t1 >= -90 && z.t1 <= 90);
      const g = s.cands[gi < 0 ? 0 : gi], b = s.cands[gi === 0 ? 1 : 0];
      const qs = [g.t1, g.t2, g.t3, g.t4, g.t5, g.t6];
      const Td = fromRp(Rt, Pt);
      const link = (S, id, a, bb, col, w) => {
        const dv = sub(bb, a), L = norm(dv); if (L < 1e-6) return;
        const ax = cross(X, dv), an = Math.acos(Math.max(-1, Math.min(1, dv[0] / L))) / D;
        const Rf = norm(ax) < 1e-9 ? (dv[0] > 0 ? I() : rz(180)) : rotK(unit(ax), an);
        fr(S, 'L' + id, mul(tr(...a), Rf), { hidden: true });
        body(S, 'b' + id, 'L' + id, 'box', [L, w || 0.03, w || 0.03], [L / 2, 0, 0], { c: col });
      };
      const arm = (S, q, pre, o) => {
        o = o || {};
        const kk = pFK(c, q);
        const op = o.op;
        body(S, pre + 'ped', 'F0', 'cyl', [0.04, 0.25], [0, 0, -0.125], { c: 'body', op });
        link(S, pre + '2', O, kk.E, o.c || 'j2', 0.032); link(S, pre + '3', kk.E, kk.W, o.c || 'j3', 0.028);
        if (op != null) { S.bodies['b' + pre + '2'].op = op; S.bodies['b' + pre + '3'].op = op; }
        pt(S, pre + 'jE', kk.E, o.noLabel ? '' : 'J3', { c: 'j3', r: 0.5 });
        pt(S, pre + 'jW', kk.W, o.noLabel ? '' : 'wrist', { c: 'j5', r: 0.5 });
        if (o.frames) { fr(S, pre + 'F3', fromRp(kk.R03, kk.W), { label: '3', size: 0.1 }); fr(S, pre + 'F6', fromRp(kk.R06, kk.W), { label: '6', size: 0.14 }); }
      };
      const mid = scl(Pt, 0.5);
      return {
        view: { pos: add(mid, [1.1, -1.4, 1.0]), target: mid },
        answers: ['θ₁', 'θ₂', 'θ₃', 'θ₄', 'θ₅', 'θ₆'].map((l, i) => ({ label: l, val: qs[i], unit: '°', tol: 0.5, dp: 2, mod: 360 })),
        init(S) { fr(S, 'F0', I(), { label: '0', size: 0.14 }); fr(S, 'TG', Td, { label: '', name: 'goal {6}', size: 0.14, ghost: true }); pt(S, 'tg', Pt, 'goal', { c: 'k', r: 0.4 }); },
        steps: [
          {
            t: 'Problem',
            html: head(8, 20, R`<p>Use the notation and equations of the lecture section "Pieper's solution when three axes intersect." For a six-revolute-joint manipulator, the last three joint axes intersect at the end-effector origin. The parameters required for the wrist-center position are</p>
              <p style="text-align:center">\(a_1 = 0,\quad d_2 = 0,\quad \alpha_1 = -90^\circ,\)<br>\(f_1 = ${a2} + ${a3}\cos\theta_3 + ${d4}\sin\theta_3,\)<br>\(f_2 = ${a3}\sin\theta_3 - ${d4}\cos\theta_3,\)<br>\(f_3 = 0.\)</p>
              <p>All lengths are in meters. The arm and wrist orientations are \({}^0_3R = R_z(\theta_1)R_x(-90^\circ)R_z(\theta_2+\theta_3)\), \({}^3_6R = R_z(\theta_4)R_y(\theta_5)R_z(\theta_6)\). The desired end-effector pose is</p><p style="text-align:center">\({}^0_6T_d = ${pmat(Td)}.\)</p><p> Determine all six joint angles using Pieper's method, subject to \(-90^\circ\le\theta_1\le 90^\circ,\ 0^\circ\le\theta_2\le 90^\circ,\ 0^\circ<\theta_5<180^\circ\). Report all joint angles in \([-180^\circ, 180^\circ)\).</p>`)
          },
          {
            t: '0 · 전략과 쓸 식',
            tex: R`\begin{aligned}&\text{(1) 위치: } {}^0P = {}^0_1T\,{}^1_2T\begin{bmatrix}f_1\\f_2\\f_3\\1\end{bmatrix},\quad \begin{cases} g_1 = c_2f_1 - s_2f_2\\ g_2 = f_3 = 0\\ g_3 = -(s_2f_1 + c_2f_2)\end{cases}\Rightarrow\ x = c_1g_1,\ y = s_1g_1,\ z = g_3\\ &\text{(2) 방향: } {}^0_6R = {}^0_3R\ {}^3_6R\ \Rightarrow\ {}^3_6R = ({}^0_3R)^T\,{}^0_6R_d\end{aligned}`,
            html: R`<p><b>왜 나눠 푸나.</b> 손목 세 축(4, 5, 6)이 끝점 원점에서 만나므로 θ₄~θ₆을 아무리 돌려도 끝점 <b>위치</b>는 그대로입니다. 그래서</p>
            <ul style="margin:0 0 8px 18px;padding:0"><li>목표 위치 \(P = (${Pt.map(v => f(v, 4)).join(',\\ ')})\) = 손목 중심 → 팔(θ₁, θ₂, θ₃)만으로 맞춥니다 (단계 ①~④).</li><li>남은 방향 차이 → 손목(θ₄, θ₅, θ₆)이 맞춥니다 (단계 ⑤~⑦).</li></ul>
            <p><b>(1)의 출처.</b> f = (f₁, f₂, f₃)는 손목 중심을 프레임 {2}에서 본 좌표(θ₃만의 함수, 문제에 주어짐). 여기에 \({}^1_2T\) (α₁ = −90°, a₁ = d₂ = 0)를 곱한 것이 g, 다시 \({}^0_1T = R_Z(\theta_1)\)을 곱한 것이 (x, y, z)입니다. 이 세 줄 식이 아래 모든 단계의 출발점입니다.</p>`
          },
          {
            t: '① r을 전개하면 θ₃만 남는다',
            do(S) { body(S, 'sph', 'F0', 'sph', [Math.sqrt(s.rr)], [0, 0, 0], { c: 'acc', op: 0.08 }); },
            tex: R`\begin{aligned} r &= x^2+y^2+z^2 = g_1^2 + g_3^2 = f_1^2 + f_2^2\\ &= (${a2} + ${a3}c_3 + ${d4}s_3)^2 + (${a3}s_3 - ${d4}c_3)^2\\ &= ${a2}^2 + ${a3}^2(c_3^2+s_3^2) + ${d4}^2(s_3^2+c_3^2)\\ &\quad + 2(${a2})(${a3}c_3 + ${d4}s_3) + \underbrace{2(${a3})(${d4})(c_3s_3 - s_3c_3)}_{=0}\\ &= ${f(a2 * a2 + a3 * a3 + d4 * d4, 4)} + ${f(s.A, 4)}c_3 + ${f(s.B, 4)}s_3\end{aligned}`,
            html: R`<p>회전(θ₁, θ₂)은 길이를 바꾸지 않으므로 \(g_1^2 + g_3^2 = f_1^2 + f_2^2\). 즉 원점에서 손목 중심까지 거리의 제곱 r은 팔꿈치(θ₃)만의 함수입니다. 투명한 구(반지름 \(\sqrt r\))가 손목 중심이 있어야 할 곳입니다.</p>
            <p><b>숫자:</b> \(r = ${pn(Pt[0])}^2 + ${pn(Pt[1])}^2 + ${pn(Pt[2])}^2 = ${f(s.rr, 5)}\). 음수는 괄호로 감싼 뒤 제곱(계산기에서 (−)0.1² = −0.01이 되는 실수 주의).</p>`
          },
          {
            t: '② a cos θ₃ + b sin θ₃ = c → 반각 치환',
            assume: [{ t: R`실수해 조건: \(a^2+b^2-c^2 = ${f(s.disc, 6)} \ge 0\)`, ok: s.disc >= 0 }],
            tex: R`\begin{aligned}&\underbrace{${f(s.A, 4)}}_{a}\,c_3 + \underbrace{${f(s.B, 4)}}_{b}\,s_3 = \underbrace{${f(s.rr, 5)} - ${f(a2 * a2 + a3 * a3 + d4 * d4, 4)}}_{c\ =\ ${f(s.C, 5)}}\\ &u = \tan\tfrac{\theta_3}{2},\quad c_3 = \frac{1-u^2}{1+u^2},\quad s_3 = \frac{2u}{1+u^2}\ \xrightarrow{\times(1+u^2)}\ a(1-u^2) + 2bu = c(1+u^2)\\ &\Rightarrow\ (a+c)u^2 - 2bu + (c-a) = 0:\quad ${f(s.A + s.C, 5)}\,u^2 ${s.B >= 0 ? '-' : '+'} ${f(Math.abs(2 * s.B), 4)}\,u ${s.C - s.A >= 0 ? '+' : '-'} ${f(Math.abs(s.C - s.A), 5)} = 0\\ &u = \frac{b \pm \sqrt{a^2+b^2-c^2}}{a+c} = \frac{${f(s.B, 4)} \pm ${f(Math.sqrt(Math.max(0, s.disc)), 5)}}{${f(s.A + s.C, 5)}} = ${f(s.cands[0].u, 5)}\ \text{or}\ ${f(s.cands[1].u, 5)}\\ &\theta_3 = 2\tan^{-1}u = \ha{${deg(s.cands[0].t3)}}\ \text{or}\ \ha{${deg(s.cands[1].t3)}}\end{aligned}`,
            html: R`<p>a = 2a₂a₃, b = 2a₂d₄, c = r − a₂² − a₃² − d₄². 반각 공식을 넣고 양변에 (1 + u²)를 곱하면 u의 2차방정식이 됩니다. 해가 두 개인 이유: 같은 거리 √r를 팔꿈치를 "펴서"도 "굽혀서"도 만들 수 있기 때문. 어느 쪽인지는 ④에서 θ₂ 조건으로 고릅니다.</p>
            <p class="note">계산기: MODE 5 (EQN) → 3 (aX²+bX+c=0)에 \(a+c,\ -2b,\ c-a\)를 넣으면 X1, X2 = u. 이어서 COMP에서 2 tan⁻¹(u).</p>`
          },
          {
            t: '③ θ₁ — 위에서 내려다본 방향',
            assume: [{ t: R`\(-90^\circ\le\theta_1 = ${f(g.t1, 2)}^\circ\le 90^\circ\) → \(g_1 > 0\)인 해 (θ₁ ± 180°, g₁ < 0 해는 제외)`, ok: g.t1 >= -90 && g.t1 <= 90 }],
            tex: R`x = c_1g_1,\ \ y = s_1g_1\ \Rightarrow\ \theta_1 = \text{atan2}(y, x) = \text{atan2}(${f(Pt[1], 4)},\ ${f(Pt[0], 4)}) = \ha{${deg(g.t1)}},\qquad g_1 = +\sqrt{x^2+y^2} = ${f(s.g1, 4)}`,
            html: R`<p>g₂ = 0이므로 손목 중심은 θ₁ 방향의 수직 평면 위에만 있습니다. 위에서 보면 원점에서 손목 중심까지 수평 거리가 g₁, 그 방향이 θ₁. 계산기: Pol(x, y) → r = g₁, θ = θ₁이 한 번에.</p>`
          },
          {
            t: '④ θ₂ — 수직 평면 안의 회전, 해 고르기',
            do(S) { delete S.bodies.sph; arm(S, qs, 'g'); arm(S, [b.t1, b.t2, b.t3, 0, 90, 0], 'x', { op: 0.25, c: 'warn', noLabel: true }); },
            assume: [{ t: R`\(0^\circ \le \theta_2 \le 90^\circ\): θ₃ = ${f(g.t3, 2)}°일 때 θ₂ = ${f(g.t2, 2)}° ✓`, ok: true }, { t: R`θ₃ = ${f(b.t3, 2)}°이면 θ₂ = ${f(b.t2, 2)}° → 조건 위반 (붉은 팔)`, ok: false }],
            tex: R`\begin{aligned}&g_1 = c_2f_1 - s_2f_2,\ \ -z = s_2f_1 + c_2f_2\ \Rightarrow\ \begin{bmatrix}g_1\\-z\end{bmatrix} = \begin{bmatrix}c_2&-s_2\\s_2&c_2\end{bmatrix}\begin{bmatrix}f_1\\f_2\end{bmatrix}\ \Rightarrow\ \theta_2 = \text{atan2}(-z, g_1) - \text{atan2}(f_2, f_1)\\ &\text{atan2}(-z, g_1) = \text{atan2}(${f(-Pt[2], 4)},\ ${f(s.g1, 4)}) = ${deg(atan2d(-Pt[2], s.g1))}\\ &\theta_3 = ${deg(g.t3)}:\ f_1 = ${f(g.f1, 4)},\ f_2 = ${f(g.f2, 4)},\ \text{atan2}(f_2,f_1) = ${deg(atan2d(g.f2, g.f1))}\ \Rightarrow\ \theta_2 = \ha{${deg(g.t2)}}\ \checkmark\\ &\theta_3 = ${deg(b.t3)}:\ f_1 = ${f(b.f1, 4)},\ f_2 = ${f(b.f2, 4)},\ \text{atan2}(f_2,f_1) = ${deg(atan2d(b.f2, b.f1))}\ \Rightarrow\ \theta_2 = ${deg(b.t2)}\ \times\end{aligned}`,
            html: R`<p>(1)의 g₁, g₃ 식을 다시 쓰면 2D 회전행렬 꼴: <b>벡터 (f₁, f₂)를 θ₂만큼 돌리면 (g₁, −z)</b>. −z인 이유는 α₁ = −90°라 Ŷ₁이 아래를 향하기 때문입니다. 회전 후 각도 − 회전 전 각도 = θ₂.</p>
            <p>θ₃ 후보 둘 다 계산해 조건을 만족하는 쪽을 고릅니다. 계산기: Pol(g₁, −z)의 θ를 A에 저장 → Pol(f₁, f₂)의 θ를 빼기.</p>`
          },
          {
            t: '⑤ ⁰₃R — 공식에 바로 대입',
            tex: R`\begin{aligned}{}^0_3R &= R_z(\theta_1)R_x(-90^\circ)R_z(\theta_{23}) = \begin{bmatrix} c_1c_{23} & -c_1s_{23} & -s_1\\ s_1c_{23} & -s_1s_{23} & c_1\\ -s_{23} & -c_{23} & 0\end{bmatrix}\\ &\theta_{23} = \theta_2+\theta_3 = ${deg(g.t2 + g.t3)}\\ &c_1 = ${f(Math.cos(g.t1 * D), 4)},\quad s_1 = ${f(Math.sin(g.t1 * D), 4)}\\ &c_{23} = ${f(Math.cos((g.t2 + g.t3) * D), 4)},\quad s_{23} = ${f(Math.sin((g.t2 + g.t3) * D), 4)}\\ &\Rightarrow\ {}^0_3R = ${tm3(g.R03)}\end{aligned}`,
            html: R`<p>세 행렬을 매번 곱할 필요 없이 위 일반식을 한 번 외워 두고(또는 열 고쳐 쓰기로 30초 만에 만들고) c₁, s₁, c₂₃, s₂₃만 대입하면 됩니다. 3열 \((-s_1, c_1, 0)\)은 θ₂, θ₃와 무관합니다.</p>`
          },
          {
            t: '⑥ ³₆R = (⁰₃R)ᵀ ⁰₆R_d',
            tex: R`\begin{aligned}&({}^3_6R)_{ij} = (\,{}^0_3R\text{의 } i\text{열}\,)\cdot(\,{}^0_6R_d\text{의 } j\text{열}\,)\\ &\text{예: } r_{33} = (${f(g.R03[0][2], 4)})(${f(Rt[0][2], 4)}) + (${f(g.R03[1][2], 4)})(${f(Rt[1][2], 4)}) + (0)(${f(Rt[2][2], 4)}) = ${f(g.R36[2][2], 4)}\\ &{}^3_6R = ${tm3(g.R36)}\end{aligned}`,
            html: R`<p>\({}^0_6R = {}^0_3R\,{}^3_6R\)의 양변 왼쪽에 \(({}^0_3R)^{-1} = ({}^0_3R)^T\)를 곱합니다(회전행렬의 역 = 전치). 전치를 따로 쓰지 말고 "⁰₃R의 열 · R_d의 열" 내적으로 계산하면 실수가 줄어듭니다. 실제로 필요한 건 3열(r₁₃, r₂₃, r₃₃)과 3행(r₃₁, r₃₂)의 다섯 칸뿐입니다.</p>
            <p class="note">계산기: MODE 6 → MatA = ⁰₃R, MatB = ⁰₆R_d → Trn(MatA) × MatB.</p>`
          },
          {
            t: '⑦ θ₅, θ₄, θ₆ — ZYZ에서 뽑기',
            do(S) { drop(S, 'bodies'); drop(S, 'pts'); drop(S, 'frames', 'g'); arm(S, qs, 'g', { frames: true }); pt(S, 'tg', Pt, '', { c: 'k', r: 0.2 }); },
            assume: [{ t: R`\(0^\circ<\theta_5<180^\circ\) → \(s_5 = +\sqrt{r_{31}^2+r_{32}^2} > 0\)`, ok: g.t5 > 0 && g.t5 < 180 }],
            tex: R`\begin{aligned}&R_z(\theta_4)R_y(\theta_5)R_z(\theta_6) = \begin{bmatrix} \cdot & \cdot & c_4s_5\\ \cdot & \cdot & s_4s_5\\ -s_5c_6 & s_5s_6 & c_5\end{bmatrix}\\ &\theta_5 = \text{atan2}\big(\sqrt{r_{31}^2+r_{32}^2},\ r_{33}\big) = \text{atan2}\big(\sqrt{${pn(g.R36[2][0])}^2 + ${pn(g.R36[2][1])}^2},\ ${f(g.R36[2][2], 4)}\big) = \ha{${deg(g.t5)}}\\ &\theta_4 = \text{atan2}(r_{23},\ r_{13}) = \text{atan2}(${f(g.R36[1][2], 4)},\ ${f(g.R36[0][2], 4)}) = \ha{${deg(g.t4)}}\\ &\theta_6 = \text{atan2}(r_{32},\ -r_{31}) = \text{atan2}(${f(g.R36[2][1], 4)},\ ${f(-g.R36[2][0], 4)}) = \ha{${deg(g.t6)}}\end{aligned}`,
            html: R`<p>각도가 적게 섞인 칸만 씁니다: r₃₃ = c₅, 3열 (c₄s₅, s₄s₅), 3행 (−s₅c₆, s₅s₆). \(r_{31}^2 + r_{32}^2 = s_5^2(c_6^2+s_6^2) = s_5^2\)이고 조건상 s₅ > 0이라 양의 제곱근. s₅ > 0이면 나눠도 부호가 안 바뀌므로 \(\theta_4 = \text{atan2}(r_{23}/s_5,\ r_{13}/s_5) = \text{atan2}(r_{23}, r_{13})\), θ₆도 같은 이유.</p>
            <p class="note">s₅ < 0을 택하면 (θ₄ + 180°, −θ₅, θ₆ + 180°)가 같은 방향을 만드는 쌍둥이 해입니다. 0° < θ₅ < 180° 조건이 그 해를 버립니다.</p>`
          },
          {
            t: '⑧ 답과 검산',
            tex: R`\begin{aligned}&(\theta_1,\dots,\theta_6) = \ha{(${qs.map(v => f(v, 2)).join(',\\ ')})^\circ}\\ &\text{위치 검산: } g_1 = c_2f_1 - s_2f_2 = ${f(Math.cos(g.t2 * D) * g.f1 - Math.sin(g.t2 * D) * g.f2, 4)},\ \ z = -(s_2f_1 + c_2f_2) = ${f(-(Math.sin(g.t2 * D) * g.f1 + Math.cos(g.t2 * D) * g.f2), 4)}\\ &\qquad\Rightarrow (c_1g_1,\ s_1g_1,\ z) = (${f(Math.cos(g.t1 * D) * (Math.cos(g.t2 * D) * g.f1 - Math.sin(g.t2 * D) * g.f2), 4)},\ ${f(Math.sin(g.t1 * D) * (Math.cos(g.t2 * D) * g.f1 - Math.sin(g.t2 * D) * g.f2), 4)},\ ${f(-(Math.sin(g.t2 * D) * g.f1 + Math.cos(g.t2 * D) * g.f2), 4)})\ \approx P\ \checkmark\\ &\text{방향 검산: } {}^3_6R\text{의 } r_{33} = c_5 = ${f(Math.cos(g.t5 * D), 4)},\ \ r_{13} = c_4s_5 = ${f(Math.cos(g.t4 * D) * Math.sin(g.t5 * D), 4)}\ \checkmark\end{aligned}`,
            html: R`<p>구한 각을 (1)의 식에 다시 넣어 목표 위치가 나오는지 확인합니다. 3D: 손목 중심의 프레임 {6}이 흐린 목표 프레임과 겹칩니다.</p>
            <p><b>쓴 식 요약</b>: ① r = f₁² + f₂² ② (a+c)u² − 2bu + (c−a) = 0 ③ θ₁ = atan2(y, x) ④ θ₂ = atan2(−z, g₁) − atan2(f₂, f₁) ⑤ ⁰₃R 일반식 ⑥ ³₆R = (⁰₃R)ᵀR_d ⑦ ZYZ: θ₅ = atan2(√(r₃₁²+r₃₂²), r₃₃), θ₄ = atan2(r₂₃, r₁₃), θ₆ = atan2(r₃₂, −r₃₁)</p>`
          }
        ]
      };
    }
  });

  window.COURSE.chapters.push({ id: 'exam', title: 'Mock Exam · 과제 스타일 모의고사', short: '📝 모의고사', sections });
  window.EXAM = { pFK, pIK, TEMPL, CONSTS };
})();
