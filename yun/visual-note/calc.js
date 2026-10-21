/* =========================================================
   CASIO fx-570ES PLUS (2nd edition) key-sequence guides
   - a "계산기" chapter
   - calc panels attached to existing example / practice steps (hooks)
   Menu numbers checked against CASIO's official fx-570ES PLUS manual:
     MODE 1 COMP · 5 EQN · 6 MATRIX · 8 VECTOR, SHIFT MODE 3 Deg
     SHIFT 4 (MATRIX): 1 Dim 2 Data 3 MatA 4 MatB 5 MatC 6 MatAns 7 det 8 Trn
     matrix size menu: 1 3×3 … 3 3×1 … 5 2×2 ; SHIFT 5 (VECTOR): 1 Dim 2 Data 3 VctA 4 VctB 5 VctC 6 VctAns 7 Dot ; size 1:3 2:2
   ========================================================= */
(function () {
  'use strict';
  const R = String.raw;
  const { I, mul, rx, ry, rz, tr, inv, app, R3, p3, fromRp, RT, mv3, col, unit, cross, dot, norm, scl, add, sub, atan2d, f, tm, tv, D } = M;
  const K = window.KIN;
  const { fr, pt, vec, ln, arc, body, drop } = K;
  const X = [1, 0, 0], Yv = [0, 1, 0], Z = [0, 0, 1], O = [0, 0, 0];

  /* 10-significant-digit display like the LCD */
  const lcd = x => { if (!isFinite(x)) return 'Math ERROR'; if (Math.abs(x) < 5e-10) return '0'; let s = Number(x.toPrecision(10)).toString(); return s.replace('-', '−'); };
  const n4 = x => f(x, 4).replace('-', '−');
  const row = (keys, d, out) => ({ keys, d, out });
  const mat = (A, n) => ({ mat: A.map(r => r.map(v => n4(v))) });
  // typed number for key chips: negative numbers need the (−) key
  const num = (x, n) => { const s = f(x, n == null ? 4 : n); return s[0] === '-' ? '(−) ' + s.slice(1) : s; };
  const sq = (x, n) => (x < 0 ? `((−) ${f(-x, n == null ? 4 : n)})²` : `${f(x, n == null ? 4 : n)}²`);
  const entries = Arow => Arow.map(v => `«${v}» =`).join(' ');

  const C = {
    deg: row('SHIFT MODE:SETUP 3:Deg', '각도 단위를 도(°)로. 화면 위에 <b>D</b>가 보이면 됩니다. 로봇공학 계산은 전부 Deg 기준입니다.'),
    comp: row('MODE 1:COMP', '일반 계산 모드'),
    matMode: (which, size) => row(`MODE 6:MATRIX ${which} ${size}`, '행렬 모드에 들어가 첫 행렬의 크기를 고르면 행렬 편집 화면이 열립니다.'),
    matDim: (which, size) => row(`AC SHIFT 4:MATRIX 1:Dim ${which} ${size}`, '다른 행렬 변수의 크기를 정하고 편집 화면을 엽니다.'),
    vecMode: row('MODE 8:VECTOR 1:VctA 1:3', '벡터 모드, VctA를 3차원으로'),
    vecDim: which => row(`AC SHIFT 5:VECTOR 1:Dim ${which} 1:3`, '다른 벡터 변수를 3차원으로')
  };
  const matTyping = (A, label) => row(A.map(r => entries(r)).join(' '), `${label}: 1행 왼쪽→오른쪽, 그다음 2행… 순서로 입력하고 각각 <span class="kk">=</span>`);

  /* ---------- reusable guide builders ---------- */
  // R (3×3, given as typed strings) times p (3×1) [+ q (3×1)]
  function guideRp(Rtyped, Rnum, pTyped, qTyped, result, opts) {
    opts = opts || {};
    const rows = [C.deg, C.matMode('1:MatA', '1:3×3'), matTyping(Rtyped, 'MatA (회전행렬)'), C.matDim('2:MatB', '3:3×1'), row(pTyped.map(v => `«${v}» =`).join(' '), 'MatB = 옮길 점(3×1)')];
    if (qTyped) {
      rows.push(C.matDim('3:MatC', '3:3×1'), row(qTyped.map(v => `«${v}» =`).join(' '), 'MatC = 원점 위치(평행이동, 3×1)'));
      rows.push(row('AC SHIFT 4:MATRIX 3:MatA × SHIFT 4:MATRIX 4:MatB + SHIFT 4:MATRIX 5:MatC =', R`4×4를 쓰지 않고 \(R\,{}^BP + P_{ORG}\)을 한 줄로: 블록 계산(4행 0 0 0 1은 생략).`, mat(result.map(v => [v]))));
    } else rows.push(row('AC SHIFT 4:MATRIX 3:MatA × SHIFT 4:MATRIX 4:MatB =', '행렬 곱 → 결과가 MatAns 화면에 나옵니다.', mat(result.map(v => [v]))));
    if (opts.note) rows.push(row('', opts.note));
    return rows;
  }
  const ang = th => (typeof th === 'number' ? (th < 0 ? `((−) ${f(-th, 4)})` : ` ${f(th, 4)}`) : ` ${th}`);
  const RzTyped = t => { const a = ang(t); return [[`cos${a}`, `(−) sin${a}`, '0'], [`sin${a}`, `cos${a}`, '0'], ['0', '0', '1']]; };
  const RxTyped = t => { const a = ang(t); return [['1', '0', '0'], ['0', `cos${a}`, `(−) sin${a}`], ['0', `sin${a}`, `cos${a}`]]; };
  const RyTyped = t => { const a = ang(t); return [[`cos${a}`, '0', `sin${a}`], ['0', '1', '0'], [`(−) sin${a}`, '0', `cos${a}`]]; };
  const numTyped = A => A.map(r => r.map(v => num(v, 4)));
  // Pol(x, y) → θ = atan2(y, x)
  const pol = (xT, yT, xv, yv, what) => row(`SHIFT +:Pol «${xT}» SHIFT ):, «${yT}» ) =`, `${what || ''} Pol(x, y)의 θ가 곧 atan2(y, x)입니다. 순서 주의: <b>x(코사인 쪽)가 먼저</b>, y(사인 쪽)가 둘째.`, `r=${lcd(Math.hypot(xv, yv))}\nθ=${lcd(atan2d(yv, xv))}`);

  const sections = [];
  const sec = o => sections.push(o);
  const empty = S => { fr(S, 'A', I(), { size: 1 }); };

  /* ================= C0 basics ================= */
  sec({
    id: 'k-setup', group: 'lecture', ref: '시작 전 설정', title: 'K.1 기본 설정과 키 읽는 법',
    view: { pos: [3.2, -3.6, 2.6], target: [0.3, 0.3, 0.3] }, grid: [2, 0.5], scale: 0.6, fsize: 1,
    build() {
      return {
        init(S) { empty(S); },
        steps: [{
          t: '각도 단위, 모드, 변수 키',
          tex: R`\text{fx-570ES PLUS:}\quad \text{MODE }1\ \text{COMP},\ \ 5\ \text{EQN},\ \ 6\ \text{MATRIX},\ \ 8\ \text{VECTOR}`,
          html: R`<p>노트의 모든 각도는 도(°) 단위이므로 계산 전에 반드시 <b>Deg</b>로 맞춥니다. 키 그림에서 노란 글씨(위 라벨)는 <span class="kk sh">SHIFT</span>를 먼저 누른 뒤의 기능, 숫자 키 아래 작은 글씨는 화면 메뉴 항목 이름입니다.</p>
          <p>자주 쓰는 변수 키: A = <span class="kk">(−)</span>, B = <span class="kk">°'"</span>, C = <span class="kk">hyp</span>, D = <span class="kk">sin</span>, X = <span class="kk">)</span>, Y = <span class="kk">S⇔D</span>. 저장은 <span class="kk sh">SHIFT</span><span class="kk">RCL</span>(STO) + 변수 키, 불러오기는 <span class="kk">ALPHA</span> + 변수 키입니다.</p>
          <p class="note">결과가 \(\frac{\sqrt3}{2}\)처럼 분수·근호로 나오면 <span class="kk">S⇔D</span>로 소수와 바꿀 수 있습니다.</p>`,
          calc: [C.deg, C.comp,
            row('«0.6» SHIFT RCL:STO (−):A', '숫자를 변수 A에 저장 (예: 링크 길이)', 'Ans→A\n0.6'),
            row('«2» × ALPHA (−):A =', '저장한 변수를 식에 사용', '1.2'),
            row('S⇔D', '분수/근호 ↔ 소수 표시 전환')]
        }]
      };
    }
  });

  /* ================= C1 Pol = atan2 ================= */
  sec({
    id: 'k-pol', group: 'lecture', ref: '역기구학·고정각 공통', title: 'K.2 atan2는 Pol( )로',
    view: { pos: [0.5, -3.5, 11], target: [0.5, 2.6, 0] }, grid: [6, 1], scale: 1, fsize: 1.2,
    build() {
      const pts = [[4, 5], [-3, 4]];
      const scene = (S, i) => {
        empty(S);
        const [x, y] = pts[i], th = atan2d(y, x);
        pt(S, 'p', [x, y, 0], `(${x}, ${y})`, { c: 'k' });
        vec(S, 'r', O, [x, y, 0], { c: 'k', label: 'r = ' + f(Math.hypot(x, y), 3) });
        arc(S, 'th', { n: Z, u: X, ang: th, r: 1.3, label: 'θ = ' + f(th, 2) + '°', col: 'acc' });
        if (i === 1) arc(S, 'bad', { n: Z, u: X, ang: atan2d(4, -3) - 180, r: 0.9, label: 'tan⁻¹(4/−3) = ' + f(Math.atan(4 / -3) / D, 2) + '° (틀림)', col: 'warn' });
      };
      return {
        init(S) { scene(S, 0); },
        steps: [
          {
            t: 'Pol(x, y)의 θ = atan2(y, x)',
            tex: R`\text{Pol}(x, y) \;\to\; r = \sqrt{x^2+y^2},\quad \theta = \text{atan2}(y, x)\in(-180^\circ, 180^\circ]`,
            html: R`<p>계산기에는 atan2 키가 없지만, 직교좌표를 극좌표로 바꾸는 <b>Pol</b>이 정확히 같은 일을 합니다. 원통형 로봇(과제 1)의 \(r,\alpha\)도 한 번에 나옵니다. 결과의 r은 변수 X, θ는 변수 Y에 자동 저장됩니다.</p>`,
            calc: [C.deg, C.comp, pol('4', '5', 4, 5, '목표 (4, 5)의 방위각:'), row('RCL S⇔D:Y', 'θ를 다시 보고 싶을 때 (Y에 저장됨)', lcd(atan2d(5, 4))), row('SHIFT −:Rec «6.403» SHIFT ):, «51.34» ) =', '거꾸로 Rec(r, θ)는 (x, y)', `X=${lcd(6.403 * Math.cos(51.34 * D))}\nY=${lcd(6.403 * Math.sin(51.34 * D))}`)]
          },
          {
            t: 'tan⁻¹(y/x)를 쓰면 안 되는 이유',
            do(S) { drop(S, 'arcs'); drop(S, 'vecs'); drop(S, 'pts'); scene(S, 1); },
            tex: R`(x, y) = (-3, 4):\quad \tan^{-1}\!\frac{4}{-3} = -53.13^\circ\ (\times),\qquad \text{Pol}(-3, 4)\ \Rightarrow\ \theta = 126.87^\circ\ (\checkmark)`,
            html: R`<p>\(\tan^{-1}\)은 y/x 한 값만 보므로 x < 0인 2·3사분면을 구분하지 못합니다(붉은 호). Pol은 x, y의 부호를 따로 보므로 항상 올바른 사분면을 줍니다. 그래서 강의의 ATAN2(y, x)는 모두 <b>Pol(x, y)의 θ</b>로 계산하세요.</p>`,
            calc: [row('SHIFT tan:tan⁻¹ «4 ÷ (−) 3» ) =', '틀린 예', lcd(Math.atan(-4 / 3) / D)), pol('(−) 3', '4', -3, 4, '맞는 예:')]
          }
        ]
      };
    }
  });

  /* ================= C2 MATRIX 3×3 ================= */
  sec({
    id: 'k-mat', group: 'lecture', ref: '슬라이드 8 예제', title: 'K.3 MATRIX: 회전행렬 × 점',
    view: { pos: [2.2, -3.6, 3.4], target: [-0.2, 0.9, 0] }, grid: [2.5, 0.5], scale: 0.7, fsize: 1,
    build() {
      const T = rz(30), BP = [0, 2, 0], AP = app(T, BP);
      return {
        init(S) { fr(S, 'A', I()); fr(S, 'B', T); pt(S, 'P', { f: 'B', p: BP }, 'P'); vec(S, 'AP', O, AP, { label: '<sup>A</sup>P' }); },
        steps: [
          {
            t: 'ᴬP = R_Z(30°)·ᴮP를 계산기로',
            tex: R`{}^AP = \underbrace{\begin{bmatrix}\cos30^\circ & -\sin30^\circ & 0\\ \sin30^\circ & \cos30^\circ & 0\\ 0&0&1\end{bmatrix}}_{\text{MatA}}\underbrace{\begin{bmatrix}0\\2\\0\end{bmatrix}}_{\text{MatB}} = ${tv(AP, 4)}`,
            html: R`<p>fx-570ES PLUS의 행렬은 <b>최대 3×3</b>이고 변수는 MatA, MatB, MatC(+ 결과 MatAns) 네 개입니다. 회전행렬은 3×3, 점은 3×1로 넣으면 그대로 곱할 수 있습니다.</p>
            <p class="note">행렬 편집 화면에서는 cos 30 같은 식을 바로 넣을 수 있지만 Pol·Rec는 넣을 수 없습니다.</p>`,
            calc: guideRp(RzTyped(30), R3(T), ['0', '2', '0'], null, AP)
          },
          {
            t: '전치(역회전)와 행렬식 검산',
            tex: R`{}^BP = \Rm{A}{B}^{T}\,{}^AP,\qquad \det R = 1`,
            html: R`<p>역회전은 전치(Trn)로 합니다(1.4절). 행렬식이 1인지 보면 회전행렬을 잘못 입력했는지 바로 알 수 있습니다.</p>`,
            calc: [row('AC SHIFT 4:MATRIX 7:det SHIFT 4:MATRIX 3:MatA ) =', '입력 실수 검사: 회전행렬이면 1', '1'),
              row('AC SHIFT 4:MATRIX 8:Trn SHIFT 4:MATRIX 3:MatA ) × SHIFT 4:MATRIX 6:MatAns =', R`바로 전 결과(MatAns = \({}^AP\))를 다시 {B}로 → (0, 2, 0)이 나오면 정답. ※ 직전 계산이 \({}^AP\)일 때`, mat([[0], [2], [0]]))]
          }
        ]
      };
    }
  });

  /* ================= C3 4×4 by blocks ================= */
  sec({
    id: 'k-block', group: 'lecture', ref: '슬라이드 14, 18, 21', title: 'K.4 4×4 동차변환은 3×3 블록으로',
    view: { pos: [9, -11, 15], target: [6, 6, 0] }, grid: [14, 2], scale: 2.4, fsize: 2.6,
    build() {
      const T = mul(tr(10, 5, 0), rz(30)), BP = [3, 7, 0], AP = app(T, BP);
      const TAB = mul(tr(2.2, 0, 0.3), rz(50)), TBC = mul(tr(1.6, 0.4, 0.9), rx(90)), TAC = mul(TAB, TBC);
      const Ti = mul(tr(4, 3, 0), rz(30)), Tinv = inv(Ti);
      return {
        init(S) { fr(S, 'A', I()); fr(S, 'B', T); pt(S, 'P', { f: 'B', p: BP }, 'P'); vec(S, 'AP', O, { f: 'B', p: BP }, { c: 'ink', w: 1.2, label: '<sup>A</sup>P' }); vec(S, 'AB', O, { f: 'B' }, { c: 'acc', label: 'P_BORG' }); },
        steps: [
          {
            t: '점 변환: R·P + p 한 줄로 (슬라이드 14)',
            tex: R`\begin{bmatrix}{}^AP\\1\end{bmatrix} = \begin{bmatrix}R & p\\ 0 & 1\end{bmatrix}\begin{bmatrix}{}^BP\\1\end{bmatrix}\iff {}^AP = \underbrace{R}_{\text{MatA}}\underbrace{{}^BP}_{\text{MatB}} + \underbrace{p}_{\text{MatC}} = ${tv(AP, 4)}`,
            html: R`<p>계산기는 4×4 행렬을 못 다루지만, 동차변환의 4행은 항상 (0 0 0 1)이라 블록으로 나누면 3×3과 3×1만으로 똑같은 결과가 나옵니다.</p>`,
            calc: guideRp(RzTyped(30), R3(T), ['3', '7', '0'], ['10', '5', '0'], AP)
          },
          {
            t: '두 변환의 합성: R은 R끼리, p는 R·p + p',
            view: { pos: [6.5, -7.5, 6.5], target: [2, 0.8, 0.8] },
            do(S) { drop(S, 'pts'); drop(S, 'vecs'); fr(S, 'B', TAB); fr(S, 'C', TAC); vec(S, 'AC', O, { f: 'C' }, { c: 'acc', label: '<sup>A</sup>P_CORG' }); },
            tex: R`{}^A_CT = {}^A_BT\,{}^B_CT\ \Rightarrow\ \underbrace{{}^A_BR\,{}^B_CR}_{\text{MatA}\times\text{MatB}} = ${tm(R3(TAC), 3)},\quad \underbrace{{}^A_BR\,{}^BP_{CORG} + {}^AP_{BORG}}_{\text{MatA}\times\text{MatB} + \text{MatC}} = ${tv(p3(TAC), 3)}`,
            html: R`<p>1.8절 예: \({}^A_BT = \text{Trans}(2.2, 0, 0.3)R_Z(50^\circ)\), \({}^B_CT = \text{Trans}(1.6, 0.4, 0.9)R_X(90^\circ)\). 회전부와 위치부를 따로 두 번 계산합니다. MatB를 3×3 → 3×1로 바꿔 다시 쓰는 것이 요령입니다.</p>`,
            calc: [C.deg, C.matMode('1:MatA', '1:3×3'), matTyping(RzTyped(50), 'MatA = R_AB'), C.matDim('2:MatB', '1:3×3'), matTyping(RxTyped(90), 'MatB = R_BC'),
              row('AC SHIFT 4:MATRIX 3:MatA × SHIFT 4:MATRIX 4:MatB =', '회전부 R_AC (종이에 적어 둠)', mat(R3(TAC))),
              C.matDim('2:MatB', '3:3×1'), row('«1.6» = «0.4» = «0.9» =', 'MatB를 3×1 = P_CORG(B 기준)로 다시 정의'),
              C.matDim('3:MatC', '3:3×1'), row('«2.2» = «0» = «0.3» =', 'MatC = P_BORG'),
              row('AC SHIFT 4:MATRIX 3:MatA × SHIFT 4:MATRIX 4:MatB + SHIFT 4:MATRIX 5:MatC =', '위치부 P_CORG(A 기준)', mat(p3(TAC).map(v => [v])))]
          },
          {
            t: '역변환: Trn과 −Trn·p (슬라이드 21)',
            view: { pos: [3.5, -9, 8], target: [2.2, 1.6, 0.4] },
            do(S) { drop(S, 'vecs'); delete S.frames.C; fr(S, 'B', Ti); vec(S, 'AB', O, { f: 'B' }, { c: 'acc', label: '<sup>A</sup>P_BORG = (4, 3, 0)' }); },
            tex: R`{}^B_AT = \begin{bmatrix}R^T & -R^T p\\ 0&1\end{bmatrix}:\quad R^T = ${tm(R3(Tinv), 3)},\quad -R^Tp = ${tv(p3(Tinv), 3)}`,
            html: R`<p>4×4 역행렬 대신 전치와 3×1 곱 하나면 됩니다. MatA에 R, MatC에 p를 넣고 \(-\text{Trn}(\text{MatA})\times\text{MatC}\) (계산기에서는 앞에 \((-1)\times\)로).</p>`,
            calc: [C.deg, C.matMode('1:MatA', '1:3×3'), matTyping(RzTyped(30), 'MatA = R'), C.matDim('3:MatC', '3:3×1'), row('«4» = «3» = «0» =', 'MatC = p'),
              row('AC SHIFT 4:MATRIX 8:Trn SHIFT 4:MATRIX 3:MatA ) =', '회전부 Rᵀ', mat(R3(Tinv))),
              row('AC «(−) 1» × SHIFT 4:MATRIX 8:Trn SHIFT 4:MATRIX 3:MatA ) × SHIFT 4:MATRIX 5:MatC =', '위치부 −Rᵀp: 행렬 앞에 (−)만 붙이면 안 되므로 (−)1을 곱함 → 슬라이드 답 (−4.964, −0.598, 0)', mat(p3(Tinv).map(v => [v])))]
          }
        ]
      };
    }
  });

  /* ================= C4 VECTOR ================= */
  sec({
    id: 'k-vec', group: 'lecture', ref: '1.2절, 나사 문제', title: 'K.5 VECTOR: 내적·외적·크기',
    view: { pos: [2.9, -2.6, 2.1], target: [0.15, 0.15, 0.3] }, grid: [1.5, 0.25], scale: 0.55, fsize: 1,
    build() {
      const Rb = mul(rz(40), ry(-30)), XB = col(Rb, 0), YB = col(Rb, 1), ZB = col(Rb, 2);
      const k = [2, 1, 2];
      return {
        init(S) { fr(S, 'A', I()); fr(S, 'B', Rb); },
        steps: [
          {
            t: '방향 코사인 = 내적, 각도 = cos⁻¹',
            do(S) { arc(S, 'th', { n: unit(cross(X, XB)), u: X, ang: Math.acos(XB[0]) / D, r: 0.45, label: 'θ₁₁' }); },
            tex: R`r_{11} = \Xh_B\cdot\Xh_A = ${f(XB[0], 4)},\quad \theta_{11} = \cos^{-1}r_{11} = ${f(Math.acos(XB[0]) / D, 2)}^\circ`,
            html: R`<p>1.2절의 \(\Xh_B = (${XB.map(v => f(v, 4)).join(', ')})\)과 \(\Xh_A = (1, 0, 0)\)의 내적이 \(r_{11}\)이고, 크기가 1이므로 \(\cos^{-1}\)하면 두 축 사이 각입니다.</p>`,
            calc: [C.deg, C.vecMode, row(XB.map(v => `«${num(v, 4)}» =`).join(' '), 'VctA = X̂_B'), C.vecDim('2:VctB'), row('«1» = «0» = «0» =', 'VctB = X̂_A'),
              row('AC SHIFT 5:VECTOR 3:VctA SHIFT 5:VECTOR 7:Dot SHIFT 5:VECTOR 4:VctB =', '내적 = r₁₁', lcd(XB[0])),
              row('SHIFT cos:cos⁻¹ Ans ) =', '두 축 사이 각', lcd(Math.acos(XB[0]) / D))]
          },
          {
            t: '외적으로 오른손 좌표계 검산',
            tex: R`\Xh_B\times\Yh_B = \Zh_B = ${tv(ZB, 4)}`,
            html: R`<p>회전행렬의 1열 × 2열 = 3열이어야 합니다. 행렬을 손으로 만든 뒤 이 검산을 하면 부호 실수를 잡을 수 있습니다.</p>`,
            calc: [row('AC SHIFT 5:VECTOR 2:Data 2:VctB', 'VctB 편집 화면을 다시 열어'), row(YB.map(v => `«${num(v, 4)}» =`).join(' '), 'VctB = Ŷ_B'),
              row('AC SHIFT 5:VECTOR 3:VctA × SHIFT 5:VECTOR 4:VctB =', '외적 → Ẑ_B와 같으면 OK', mat(ZB.map(v => [v])))]
          },
          {
            t: '크기 |k|와 단위벡터 (나사 문제)',
            do(S) { ln(S, 'k', O, scl(unit(k), 1.6), { c: 'k', dash: false, label: 'k', lt: 1 }); },
            tex: R`|k| = \sqrt{2^2+1^2+2^2} = 3,\qquad \hat k = k/|k| = (0.6667,\ 0.3333,\ 0.6667)`,
            html: R`<p>Abs(벡터)가 크기입니다. 단위벡터는 VctA ÷ Abs(VctA).</p>`,
            calc: [row('AC SHIFT 5:VECTOR 2:Data 1:VctA', 'VctA 편집'), row('«2» = «1» = «2» =', 'VctA = k'),
              row('AC SHIFT hyp:Abs SHIFT 5:VECTOR 3:VctA ) =', '|k|', '3'),
              row('SHIFT 5:VECTOR 3:VctA ÷ SHIFT hyp:Abs SHIFT 5:VECTOR 3:VctA ) =', '단위벡터 k̂', mat([[2 / 3], [1 / 3], [2 / 3]]))]
          }
        ]
      };
    }
  });

  /* ================= C5 EQN: a cos + b sin = c ================= */
  function eqnNumbers(a, b, c) {
    const A2 = a + c, B2 = -2 * b, C2 = c - a, disc = B2 * B2 - 4 * A2 * C2;
    const u1 = (-B2 + Math.sqrt(disc)) / (2 * A2), u2 = (-B2 - Math.sqrt(disc)) / (2 * A2);
    return { A2, B2, C2, u1, u2, t1: 2 * Math.atan(u1) / D, t2: 2 * Math.atan(u2) / D };
  }
  function eqnGuide(a, b, c) {
    const e = eqnNumbers(a, b, c);
    return [C.deg, row('MODE 5:EQN 3:aX²+bX+c=0', '2차방정식 모드'),
      row(`«${num(e.A2, 5)}» = «${num(e.B2, 5)}» = «${num(e.C2, 5)}» =`, R`계수 \(a+c,\ -2b,\ c-a\)를 차례로 입력`, `X1=${lcd(e.u1)}`),
      row('=', '두 번째 해', `X2=${lcd(e.u2)}`),
      row(`MODE 1:COMP «2» SHIFT tan:tan⁻¹ «${num(e.u1, 6)}» ) =`, R`\(\theta = 2\tan^{-1}u\) (X1)`, lcd(e.t1)),
      row(`«2» SHIFT tan:tan⁻¹ «${num(e.u2, 6)}» ) =`, R`\(\theta = 2\tan^{-1}u\) (X2)`, lcd(e.t2)),
      row(`«${num(a, 5)} cos(» ALPHA ):X «) + ${num(b, 5)} sin(» ALPHA ):X «)» ALPHA CALC:= «${num(c, 5)}» SHIFT CALC:SOLVE`, '다른 방법: COMP에서 식을 그대로 넣고 SOLVE(초기값을 바꿔 두 번). 2차방정식 쪽이 두 해를 한 번에 줘서 편합니다.', `X=${lcd(e.t1)} 등`)];
  }
  sec({
    id: 'k-eqn', group: 'lecture', ref: '슬라이드 18 (Pieper)', title: 'K.6 a cos θ + b sin θ = c 풀기',
    view: { pos: [0.4, -1.0, 3.6], target: [0, 0, 0] }, grid: [1.5, 0.25], scale: 0.45, fsize: 0.6,
    build() {
      const a = 0.225, b = 0.108, c = 0.04058627739577428;
      const e = eqnNumbers(a, b, c);
      const nrm = Math.hypot(a, b), d = c / nrm, nh = [a / nrm, b / nrm, 0], tg = [-nh[1], nh[0], 0];
      return {
        init(S) {
          fr(S, 'A', I(), { size: 0.6, axes: ['cos θ', 'sin θ', 'Z'], sub: false, label: '' });
          arc(S, 'circ', { n: Z, u: X, ang: 359.5, r: 1, noHead: true, col: 'mute', label: '단위원 (cos θ, sin θ)' });
          ln(S, 'line', add(scl(nh, d), scl(tg, -1.5)), add(scl(nh, d), scl(tg, 1.5)), { c: 'acc', dash: false, label: 'a·x + b·y = c', lt: 0.9 });
          [e.t1, e.t2].forEach((t, i) => { const p = [Math.cos(t * D), Math.sin(t * D), 0]; pt(S, 's' + i, p, 'θ = ' + f(t, 2) + '°', { c: 'k' }); vec(S, 'v' + i, O, p, { c: 'k', w: 0.6 }); });
        },
        steps: [{
          t: '반각 치환 → 2차방정식 (EQN 모드)',
          tex: R`a\cos\theta + b\sin\theta = c,\ \ u = \tan\tfrac{\theta}{2}\ \Rightarrow\ (a+c)u^2 - 2bu + (c-a) = 0\qquad (a, b, c) = (${f(a, 4)},\ ${f(b, 4)},\ ${f(c, 5)})`,
          html: R`<p>3.5절 Pieper 예의 숫자입니다. 그림에서 단위원 위의 점 \((\cos\theta, \sin\theta)\) 중 직선 \(a x + b y = c\) 위에 있는 두 점이 두 해입니다(보라 점). 이것이 θ₃ 후보가 두 개 나오는 기하학적 이유입니다.</p>
          <p class="note">\(a^2+b^2 < c^2\)이면 직선이 원과 만나지 않아 해가 없습니다(EQN 모드가 복소수 해를 표시).</p>`,
          calc: eqnGuide(a, b, c)
        }]
      };
    }
  });

  /* ================= C6 IK routine ================= */
  sec({
    id: 'k-ik', group: 'lecture', ref: '3.3절 숫자', title: 'K.7 역기구학 계산 루틴 (평면 3R)',
    view: { pos: [0.7, -5.6, 0.8], target: [0.5, 0, 0.8] }, grid: [3, 0.5], scale: 0.7, fsize: 1,
    build() {
      const x = 1.0, y = 1.6, phi = 75, l1 = 1.2, l2 = 1.0;
      const c2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2), s2 = Math.sqrt(1 - c2 * c2), t2 = atan2d(s2, c2);
      const k1 = l1 + l2 * c2, k2 = l2 * s2, t1 = atan2d(y, x) - atan2d(k2, k1), t3 = phi - t1 - t2;
      const rows = [{ al: 0, a: 0, d: 0, th: 0, type: 'R' }, { al: 0, a: l1, d: 0, th: 0, type: 'R' }, { al: 0, a: l2, d: 0, th: 0, type: 'R' }];
      return {
        init(S) { fr(S, 'W', I(), { hidden: true }); K.robot(S, rows, [t1, t2, t3], { base: rx(90), tool: 0.35, fs: 0.35, w: 0.08, jr: 0.1, jh: 0.16, noDrag: true }); },
        steps: [{
          t: 'c₂ → θ₂ → θ₁ → θ₃ 키 순서',
          tex: R`c_2 = ${f(c2, 4)},\ s_2 = ${f(s2, 4)},\ \theta_2 = ${f(t2, 2)}^\circ,\ \ \theta_1 = ${f(atan2d(y, x), 2)}^\circ - ${f(atan2d(k2, k1), 2)}^\circ = ${f(t1, 2)}^\circ,\ \ \theta_3 = ${f(t3, 2)}^\circ`,
          html: R`<p>중간값을 변수에 저장해 두면 반올림 오차 없이 끝까지 갈 수 있습니다. 목표 \((x, y, \phi) = (1.0, 1.6, 75^\circ)\), \(l_1 = 1.2,\ l_2 = 1.0\), \(s_2 > 0\) 해.</p>`,
          calc: [C.deg, C.comp,
            row('«(1² + 1.6² − 1.2² − 1²) ÷ (2 × 1.2 × 1)» = SHIFT RCL:STO (−):A', 'c₂를 A에 저장', lcd(c2)),
            row('√ «1 −» ALPHA (−):A x² ) = SHIFT RCL:STO °\'":B', 's₂ = +√(1−c₂²)를 B에 저장', lcd(s2)),
            row('SHIFT +:Pol ALPHA (−):A SHIFT ):, ALPHA °\'":B ) =', 'θ₂ = atan2(s₂, c₂)', `r=1\nθ=${lcd(t2)}`),
            row('SHIFT +:Pol «1» SHIFT ):, «1.6» ) = RCL S⇔D:Y SHIFT RCL:STO hyp:C', 'β = atan2(y, x)를 C에 저장', lcd(atan2d(y, x))),
            row('SHIFT +:Pol «1.2 + 1 ×» ALPHA (−):A SHIFT ):, «1 ×» ALPHA °\'":B ) =', 'γ = atan2(k₂, k₁) (Y에 저장됨)', `θ=${lcd(atan2d(k2, k1))}`),
            row('ALPHA hyp:C − ALPHA S⇔D:Y = SHIFT RCL:STO sin:D', 'θ₁ = β − γ를 D에 저장', lcd(t1)),
            row(`«75 −» ALPHA sin:D «− ${f(t2, 4)}» =`, 'θ₃ = φ − θ₁ − θ₂', lcd(t3))]
        }]
      };
    }
  });

  window.COURSE.chapters.push({ id: 'calc', title: 'CASIO fx-570ES PLUS로 계산하기', short: '🧮 계산기', sections });

  /* =========================================================
     Hooks: attach calc panels to steps of existing sections
     ========================================================= */
  const byId = {}; window.COURSE.chapters.forEach(c => c.sections.forEach(s => byId[s.id] = s));
  function hook(id, fn) {
    const s = byId[id]; if (!s) return;
    const orig = s.build;
    s.build = P => { const b = orig(P); try { fn(b, P); } catch (e) { console.warn('calc hook', id, e); } return b; };
  }
  const setCalc = (b, i, rows) => { if (b.steps[i]) b.steps[i].calc = rows; };

  hook('c1-ex8', b => setCalc(b, 3, guideRp(RzTyped(30), null, ['0', '2', '0'], null, app(rz(30), [0, 2, 0]))));
  hook('c1-ex14', b => setCalc(b, 3, guideRp(RzTyped(30), null, ['3', '7', '0'], ['10', '5', '0'], app(mul(tr(10, 5, 0), rz(30)), [3, 7, 0]))));
  hook('c1-tinv', b => {
    const Ti = inv(mul(tr(4, 3, 0), rz(30)));
    setCalc(b, 3, [C.deg, C.matMode('1:MatA', '1:3×3'), matTyping(RzTyped(30), 'MatA = R'), C.matDim('3:MatC', '3:3×1'), row('«4» = «3» = «0» =', 'MatC = P_BORG'),
      row('AC SHIFT 4:MATRIX 8:Trn SHIFT 4:MATRIX 3:MatA ) =', 'Rᵀ', mat(R3(Ti))),
      row('AC «(−) 1» × SHIFT 4:MATRIX 8:Trn SHIFT 4:MATRIX 3:MatA ) × SHIFT 4:MATRIX 5:MatC =', '−Rᵀp', mat(p3(Ti).map(v => [v])))]);
  });
  hook('c1-hw-cyl', (b, P) => {
    const al = atan2d(P.y, P.x);
    setCalc(b, 4, [C.deg, C.comp, pol(num(P.x, 0), num(P.y, 0), P.x, P.y, 'r과 α를 한 번에:'), row('', `r = ${lcd(Math.hypot(P.x, P.y))}, α = ${lcd(al)}°, l = z = ${P.z}`)]);
    setCalc(b, 5, [row(`«cos ${f(al, 4)}» =`, 'Cα (Pol 직후라면 «cos(» ALPHA S⇔D:Y «)»로 Y를 바로 써도 됨)', lcd(Math.cos(al * D))), row(`«sin ${f(al, 4)}» =`, 'Sα', lcd(Math.sin(al * D)))]);
  });
  hook('c1-hw-rpy', (b, P) => {
    const r = P.R, cb = Math.hypot(r[0][0], r[1][0]);
    setCalc(b, 1, [C.deg, C.comp, pol(`√(${sq(r[0][0])} + ${sq(r[1][0])})`, num(-r[2][0]), cb, -r[2][0], 'β = atan2(−r₃₁, √(r₁₁²+r₂₁²)):')]);
    setCalc(b, 2, [pol(num(r[0][0]), num(r[1][0]), r[0][0], r[1][0], 'α: cβ > 0이므로 나누지 않아도 같은 각 → Pol(r₁₁, r₂₁)'), row('', '음수 입력 예: −0.259 → (−) 0.259')]);
    setCalc(b, 3, [pol(num(r[2][2]), num(r[2][1]), r[2][2], r[2][1], 'γ = Pol(r₃₃, r₃₂)')]);
  });
  hook('c1-hw-screw', (b, P) => {
    const k = P.k;
    setCalc(b, 1, [C.deg, C.comp, pol(num(k[0], 0), num(k[1], 0), k[0], k[1], 'α = atan2(k_y, k_x):'), pol(num(k[2], 0), `√(${sq(k[0], 0)} + ${sq(k[1], 0)})`, k[2], Math.hypot(k[0], k[1]), 'β = atan2(√(k_x²+k_y²), k_z) → Pol(k_z, √…):')]);
    const kh = unit(k), th = P.adv / P.lead * 360, c = Math.cos(th * D), v = 1 - c;
    setCalc(b, 4, [row(`«${f(kh[0], 6)}² × (1 − cos ${f(th, 3)}) + cos ${f(th, 3)}» =`, 'r₁₁ = k_x² vθ + cθ (나머지 원소도 같은 방식)', lcd(kh[0] * kh[0] * v + c)),
      row('', '또는 MATRIX로 R_z(α)R_y(β)R_z(θ)R_y(−β)R_z(−α)를 MatA·MatB·MatC와 MatAns로 차례로 곱해도 같은 결과입니다.')]);
  });
  hook('c1-hw-chain', (b, P) => {
    const TH5 = inv(P.T5H);
    setCalc(b, 2, [C.matMode('1:MatA', '1:3×3'), matTyping(numTyped(R3(P.T5H)), 'MatA = ⁵_HR'), C.matDim('3:MatC', '3:3×1'), row(p3(P.T5H).map(v => `«${num(v, 3)}» =`).join(' '), 'MatC = ⁵P_HORG'),
      row('AC SHIFT 4:MATRIX 8:Trn SHIFT 4:MATRIX 3:MatA ) =', 'ᴴ_5R = Rᵀ', mat(R3(TH5))),
      row('AC «(−) 1» × SHIFT 4:MATRIX 8:Trn SHIFT 4:MATRIX 3:MatA ) × SHIFT 4:MATRIX 5:MatC =', '−Rᵀp', mat(p3(TH5).map(v => [v]))),
      row('', '⁴_HT의 역 ᴱ_HT도 같은 방법. 두 변환의 곱은 K.4절처럼 회전부(MatA×MatB)와 위치부(MatA×MatB+MatC)로 나눠 세 번 반복합니다.')]);
  });
  hook('c2-hw-3dof', b => {
    const { Pg, Rg, t2, rho, t1n, t3, phi } = b.calcData;
    setCalc(b, 8, [C.deg, C.comp, row(`SHIFT sin:sin⁻¹ «${num(Pg[2], 3)} ÷ 0.25» ) =`, 'c₂ > 0 가정 → sin⁻¹이 바로 답 (sin⁻¹은 −90°~90°만 돌려줌)', lcd(t2)), row('SHIFT RCL:STO (−):A', 'θ₂를 A에 저장')]);
    setCalc(b, 9, [row(`«0.35 + 0.25 cos» ALPHA (−):A «)» = SHIFT RCL:STO °'":B`, 'ρ = L₁ + L₂c₂ → B', lcd(rho)),
      pol(num(Pg[0], 3), num(Pg[1], 3), Pg[0], Pg[1], '① 목표 방위각 atan2(y, x):'), row('RCL S⇔D:Y SHIFT RCL:STO hyp:C', '→ C'),
      row('SHIFT +:Pol ALPHA °\'":B SHIFT ):, «(−) 0.05» ) =', '② 팔 자체 방위각 atan2(−D, ρ)', `θ=${lcd(atan2d(-0.05, rho))}`),
      row('ALPHA hyp:C − ALPHA S⇔D:Y =', 'θ₁ = ① − ② (±180° 밖이면 360° 더하거나 빼기)', lcd(t1n))]);
    setCalc(b, 10, [pol(num(Rg[2][1], 4), num(Rg[2][0], 4), Rg[2][1], Rg[2][0], 'θ₂+θ₃ = atan2(r₃₁, r₃₂) → Pol(r₃₂, r₃₁):'), row('ALPHA S⇔D:Y − ALPHA (−):A =', 'θ₃ = (θ₂+θ₃) − θ₂', lcd(t3))]);
  });
  hook('c2-hw-scara', (b, P) => {
    const x = 0.6 * Math.cos(P.t1 * D) + 0.5 * Math.cos((P.t1 + P.t2) * D), y = 0.6 * Math.sin(P.t1 * D) + 0.5 * Math.sin((P.t1 + P.t2) * D);
    setCalc(b, 7, [C.deg, C.comp, row(`«0.6 cos ${num(P.t1, 0)} + 0.5 cos(${num(P.t1, 0)} + ${num(P.t2, 0)})» =`, 'x', lcd(x)), row(`«0.6 sin ${num(P.t1, 0)} + 0.5 sin(${num(P.t1, 0)} + ${num(P.t2, 0)})» =`, 'y', lcd(y)), row(`«cos(${num(P.t1, 0)} + ${num(P.t2, 0)} − ${num(P.t4, 0)})» =`, 'r₁₁ = c_φ', lcd(Math.cos((P.t1 + P.t2 - P.t4) * D)))]);
  });
  const planarHooks = (b, x, y, phi, steps) => {
    const l1 = 1.2, l2 = 1.0, c2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2), s2 = Math.sqrt(Math.max(0, 1 - c2 * c2)), t2 = atan2d(s2, c2), k1 = l1 + l2 * c2, k2 = l2 * s2, t1 = atan2d(y, x) - atan2d(k2, k1);
    setCalc(b, steps[0], [C.deg, C.comp, row(`«(${sq(x, 2)} + ${sq(y, 2)} − 1.2² − 1²) ÷ 2.4» = SHIFT RCL:STO (−):A`, 'c₂ → A', lcd(c2)), row('√ «1 −» ALPHA (−):A x² ) = SHIFT RCL:STO °\'":B', 's₂ = +√(1−c₂²) → B', lcd(s2)), row('SHIFT +:Pol ALPHA (−):A SHIFT ):, ALPHA °\'":B ) =', 'θ₂ = Pol(c₂, s₂)의 θ', `θ=${lcd(t2)}`)]);
    setCalc(b, steps[1], [pol(num(x, 2), num(y, 2), x, y, '① β = atan2(y, x):'), row('RCL S⇔D:Y SHIFT RCL:STO hyp:C', 'β → C'), row('SHIFT +:Pol «1.2 + 1 ×» ALPHA (−):A SHIFT ):, ALPHA °\'":B ) =', '② γ = atan2(k₂, k₁)', `θ=${lcd(atan2d(k2, k1))}`), row('ALPHA hyp:C − ALPHA S⇔D:Y =', 'θ₁ = β − γ', lcd(t1))]);
    setCalc(b, steps[2], [row(`«${num(phi, 0)} − (${num(t1, 4)}) − ${f(t2, 4)}» =`, 'θ₃', lcd(phi - t1 - t2))]);
  };
  hook('c3-alg', b => planarHooks(b, 1.0, 1.6, 75, [2, 4, 5]));
  hook('c3-hw-3r', (b, P) => planarHooks(b, P.x, P.y, P.phi, [1, 2, 3]));
  hook('c3-hw-pieper', b => {
    const { Pt, Rt, sol, g, bad } = b.calcData;
    setCalc(b, 1, [C.deg, C.comp, row(`«${sq(Pt[0])} + ${sq(Pt[1])} + ${sq(Pt[2])}» =`, R`\(r = x^2+y^2+z^2\) — 음수는 반드시 괄호로 감싼 뒤 제곱: (−)0.1² = −0.01이 되어 버립니다`, lcd(sol.rr)),
      row('«Ans − 0.45² − 0.25² − 0.12²» =', R`\(c = r - a_2^2 - a_3^2 - d_4^2\),  \(a = 2a_2a_3 = 0.225,\ b = 2a_2d_4 = 0.108\)`, lcd(sol.C))].concat(eqnGuide(sol.A, sol.B, sol.C).slice(1)));
    setCalc(b, 2, [C.comp, pol(num(Pt[0], 4), num(Pt[1], 4), Pt[0], Pt[1], 'θ₁ = Pol(x, y)의 θ, 그리고 r이 바로 g₁:')]);
    const t3 = num(g.t3, 4);
    setCalc(b, 3, [pol(f(sol.g1, 4), num(-Pt[2], 4), sol.g1, -Pt[2], '① atan2(−z, g₁):'), row('RCL S⇔D:Y SHIFT RCL:STO (−):A', '→ A'),
      pol(`0.45 + 0.25 cos(${t3}) + 0.12 sin(${t3})`, `0.25 sin(${t3}) − 0.12 cos(${t3})`, g.f1, g.f2, '② atan2(f₂, f₁) (선택한 θ₃로):'),
      row('ALPHA (−):A − ALPHA S⇔D:Y =', `θ₂ = ① − ②. 다른 후보 θ₃ = ${f(bad.t3, 2)}°로 반복하면 ${f(bad.t2, 2)}° → 조건 위반`, lcd(g.t2))]);
    setCalc(b, 4, [C.deg, C.matMode('1:MatA', '1:3×3'), matTyping(RzTyped(g.t1), 'MatA = R_Z(θ₁)'), C.matDim('2:MatB', '1:3×3'), matTyping(numTyped(R3(rx(-90))), 'MatB = R_X(−90°) (cos(−90°) = 0, sin(−90°) = −1을 바로 넣음)'),
      row('AC SHIFT 4:MATRIX 3:MatA × SHIFT 4:MATRIX 4:MatB =', 'R_Z(θ₁)R_X(−90°) → MatAns'),
      C.matDim('3:MatC', '1:3×3'), matTyping(RzTyped(g.t2 + g.t3), 'MatC = R_Z(θ₂+θ₃)'),
      row('AC SHIFT 4:MATRIX 6:MatAns × SHIFT 4:MATRIX 5:MatC =', '⁰₃R (MatAns)', mat(g.R03)),
      row('SHIFT RCL:STO (−):MatA', 'MatAns 화면에서 바로 MatA로 복사 (다시 입력할 필요 없음)'), C.matDim('2:MatB', '1:3×3'), matTyping(numTyped(Rt), 'MatB = 목표 ⁰₆R'),
      row('AC SHIFT 4:MATRIX 8:Trn SHIFT 4:MATRIX 3:MatA ) × SHIFT 4:MATRIX 4:MatB =', '³₆R = (⁰₃R)ᵀ ⁰₆R', mat(g.R36))]);
    const R36 = g.R36;
    setCalc(b, 5, [C.comp, pol(num(R36[2][2], 4), `√(${sq(R36[2][0])} + ${sq(R36[2][1])})`, R36[2][2], Math.hypot(R36[2][0], R36[2][1]), 'θ₅ = atan2(√(r₃₁²+r₃₂²), r₃₃) → Pol(r₃₃, √…):'),
      pol(num(R36[0][2], 4), num(R36[1][2], 4), R36[0][2], R36[1][2], 'θ₄ = Pol(r₁₃, r₂₃):'), pol(num(-R36[2][0], 4), num(R36[2][1], 4), -R36[2][0], R36[2][1], 'θ₆ = Pol(−r₃₁, r₃₂):')]);
  });
  function mulR(a, b) { return [0, 1, 2].map(i => [0, 1, 2].map(j => a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j])); }
  window.CALC = { lcd, eqnNumbers };
})();
