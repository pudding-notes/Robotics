/* =========================================================
   Step-by-step self-check ("직접 풀기" mode)
   - expression parser (numbers, symbols c1, s23, L1 …, sin/cos in degrees)
   - check specs attached to practice / exam steps
   ========================================================= */
(function () {
  'use strict';
  const { mul, rx, ry, rz, tr, inv, app, R3, p3, rotK, unit, norm, atan2d, D } = M;
  const K = window.KIN;

  /* ---------------- expression parser ---------------- */
  const FUNCS = { sin: x => Math.sin(x * D), cos: x => Math.cos(x * D), tan: x => Math.tan(x * D), sqrt: Math.sqrt, abs: Math.abs };
  function normalize(s) {
    return String(s).replace(/[−–—]/g, '-').replace(/[·×]/g, '*').replace(/÷/g, '/').replace(/²/g, '^2').replace(/³/g, '^3')
      .replace(/√/g, 'sqrt').replace(/π/g, 'pi').replace(/°/g, '').replace(/,/g, '.').trim();
  }
  function tokenize(src, names) {
    const s = normalize(src), out = [];
    let i = 0;
    const known = names.slice().sort((a, b) => b.length - a.length);
    while (i < s.length) {
      const ch = s[i];
      if (/\s/.test(ch)) { i++; continue; }
      const mNum = s.slice(i).match(/^(\d+\.?\d*|\.\d+)(e[+-]?\d+)?/i);
      if (mNum) { out.push({ t: 'num', v: parseFloat(mNum[0]) }); i += mNum[0].length; continue; }
      if (/[A-Za-z_]/.test(ch)) {
        const word = s.slice(i).match(/^[A-Za-z_][A-Za-z0-9_]*/)[0];
        let w = word;
        while (w.length) {
          const hit = known.find(n => w.startsWith(n));
          if (!hit) throw new Error(`알 수 없는 기호 '${w}'`);
          out.push(FUNCS[hit] ? { t: 'fn', v: hit } : { t: 'id', v: hit });
          w = w.slice(hit.length);
        }
        i += word.length; continue;
      }
      if ('+-*/^()'.includes(ch)) { out.push({ t: ch }); i++; continue; }
      throw new Error(`알 수 없는 문자 '${ch}'`);
    }
    return out;
  }
  function parse(src, vars) {
    const names = Object.keys(FUNCS).concat(['pi'], vars || []);
    const tk = tokenize(src, names);
    if (!tk.length) throw new Error('비어 있음');
    let p = 0;
    const peek = () => tk[p], eat = t => { if (!tk[p] || tk[p].t !== t) throw new Error('식 형식 오류'); p++; };
    const startsPrimary = x => x && (x.t === 'num' || x.t === 'id' || x.t === 'fn' || x.t === '(');
    function expr() { let f = term(); while (peek() && (peek().t === '+' || peek().t === '-')) { const op = tk[p++].t, a = f, b = term(); f = op === '+' ? e => a(e) + b(e) : e => a(e) - b(e); } return f; }
    function term() {
      let f = unary();
      for (;;) {
        const x = peek();
        if (x && (x.t === '*' || x.t === '/')) { p++; const a = f, b = unary(); f = x.t === '*' ? e => a(e) * b(e) : e => a(e) / b(e); }
        else if (startsPrimary(x)) { const a = f, b = power(); f = e => a(e) * b(e); }
        else return f;
      }
    }
    function unary() { const x = peek(); if (x && (x.t === '-' || x.t === '+')) { p++; const a = unary(); return x.t === '-' ? e => -a(e) : a; } return power(); }
    function power() { const b = primary(); if (peek() && peek().t === '^') { p++; const ex = unary(); return e => Math.pow(b(e), ex(e)); } return b; }
    function primary() {
      const x = tk[p++];
      if (!x) throw new Error('식이 끝나지 않음');
      if (x.t === 'num') return () => x.v;
      if (x.t === 'id') return x.v === 'pi' ? () => Math.PI : e => { if (!(x.v in e)) throw new Error(`'${x.v}'는 이 칸에서 쓸 수 없음`); return e[x.v]; };
      if (x.t === 'fn') { eat('('); const a = expr(); eat(')'); const fn = FUNCS[x.v]; return e => fn(a(e)); }
      if (x.t === '(') { const a = expr(); eat(')'); return a; }
      throw new Error('식 형식 오류');
    }
    const f = expr();
    if (p !== tk.length) throw new Error('식 형식 오류');
    return f;
  }

  /* ---------------- symbol environments ---------------- */
  const rnd = (a, b) => a + Math.random() * (b - a);
  // angle symbols: c1, s1, …, c12, s12, c23, c123, …
  function angleEnv(n, extra) {
    const t = []; for (let i = 1; i <= n; i++) t[i] = rnd(-170, 170);
    const e = Object.assign({}, extra || {});
    e._t = t;
    const C = x => Math.cos(x * D), S = x => Math.sin(x * D);
    for (let i = 1; i <= n; i++) { e['c' + i] = C(t[i]); e['s' + i] = S(t[i]); e['t' + i] = t[i]; }
    for (let i = 1; i <= n; i++) for (let j = i + 1; j <= n; j++) {
      let sum = 0, key = ''; for (let k = i; k <= j; k++) { sum += t[k]; key += k; }
      e['c' + key] = C(sum); e['s' + key] = S(sum);
    }
    return e;
  }
  const angVars = n => { const v = []; for (let i = 1; i <= n; i++) v.push('c' + i, 's' + i, 't' + i); for (let i = 1; i <= n; i++) for (let j = i + 1; j <= n; j++) { let key = ''; for (let k = i; k <= j; k++) key += k; v.push('c' + key, 's' + key); } return v; };

  /* ---------------- spec helpers ---------------- */
  const num = (label, val, o) => Object.assign({ type: 'num', label, val, tol: 0.01 }, o || {});
  const ang = (label, val, o) => Object.assign({ type: 'num', label, val, tol: 0.3, unit: '°', mod: 360 }, o || {});
  const mat = (label, A, o) => Object.assign({ type: 'mat', label, val: A, tol: 0.005 }, o || {});
  const vec3 = (label, v, o) => mat(label, v.map(x => [x]), o);
  const set = (label, vals, o) => Object.assign({ type: 'set', label, val: vals, tol: 0.3, unit: '°', mod: 360 }, o || {});
  // symbolic matrix: fn(env) → numeric matrix ; env from sampler
  const sym = (label, rows, cols, fn, sampler, vars, o) => Object.assign({ type: 'sym', label, rows, cols, fn, sampler, vars }, o || {});
  const self = () => ({ self: true, prompt: '이 단계는 직접 작성(표·그림)하는 단계입니다. 종이에 작성한 뒤 <b>풀이 보기</b>로 비교하세요.', items: [] });
  const ck = (prompt, ...items) => ({ prompt, items });
  const sub = (T, a, b) => mul(inv(T[a]), T[b]);

  // generic D-H symbolic chain check: rows with fixed lengths; prismatic d from env
  function dhSampler(nRev, extra) { return () => angleEnv(nRev, typeof extra === 'function' ? extra() : extra); }
  function dhChain(rows, env, map) {
    // map: i → {th: env angle index | null, d: env key | null}
    const q = rows.map((r, i) => (r.type === 'P' ? env[map[i]] : env._t[map[i]]));
    return K.chainT(rows, q).Ts;
  }
  const r4 = T => T.map(r => r.slice());

  /* ---------------- hooks ---------------- */
  const byId = {}; window.COURSE.chapters.forEach(c => c.sections.forEach(s => byId[s.id] = s));
  function hook(id, fn) {
    const s = byId[id]; if (!s) return;
    const orig = s.build;
    s.build = P => { const b = orig(P); try { const m = fn(b, P); Object.keys(m).forEach(i => { if (b.steps[i]) b.steps[i].check = m[i]; }); } catch (e) { console.warn('solve hook', id, e); } return b; };
    s.solvable = true;
  }
  const symHint = list => `기호: ${list}. 곱은 * 또는 붙여 쓰기(예: c1c2, L1*c1). 관절각은 t1, t2 …로 cos(t1+t2-t4)처럼 써도 됩니다(도 단위).`;

  /* ===== Ch1 practice ===== */
  hook('c1-hw-cyl', (b, P) => {
    const al = atan2d(P.y, P.x), r = Math.hypot(P.x, P.y), l = P.z;
    const smp = () => { const a = rnd(-170, 170); return { r: rnd(1, 9), l: rnd(1, 9), ca: Math.cos(a * D), sa: Math.sin(a * D), _a: a }; };
    const V = ['r', 'l', 'ca', 'sa'], H = symHint('r, l, ca (= cos α), sa (= sin α)');
    const T = mul(tr(0, 0, l), rz(al), tr(r, 0, 0));
    return {
      1: ck('T₁ = Trans(x̂, r)를 4×4 행렬로 쓰시오.', sym('T₁', 4, 4, e => tr(e.r, 0, 0), smp, V, { hint: H })),
      2: ck('T₂ = Rot(ẑ, α)와 T₂T₁을 쓰시오.', sym('T₂', 4, 4, e => rz(e._a), smp, V, { hint: H }), sym('T₂T₁', 4, 4, e => mul(rz(e._a), tr(e.r, 0, 0)), smp, V, { hint: H })),
      3: ck('전체 변환 T = T₃T₂T₁을 쓰시오.', sym('T', 4, 4, e => mul(tr(0, 0, e.l), rz(e._a), tr(e.r, 0, 0)), smp, V, { hint: H })),
      4: ck('손 원점을 목표 위치에 놓는 관절 변수를 구하시오 (r > 0).', ang('α', al), num('r', r), num('l', l)),
      5: ck('관절 변수를 넣은 T를 숫자로 쓰시오.', mat('T', r4(T))),
      6: ck('방향을 원래 좌표계와 같게 만든 최종 T를 쓰시오.', mat('T_final', r4(mul(T, rz(-al)))))
    };
  });
  hook('c1-hw-rpy', (b, P) => {
    const r = P.R, cb = Math.hypot(r[0][0], r[1][0]);
    return {
      1: ck('β를 구하시오 (−90° < β < 90°).', ang('β', atan2d(-r[2][0], cb))),
      2: ck('α를 구하시오.', ang('α', atan2d(r[1][0], r[0][0]))),
      3: ck('γ를 구하시오.', ang('γ', atan2d(r[2][1], r[2][2])))
    };
  });
  hook('c1-hw-chain', (b, P) => {
    const TEH = inv(P.THE), TH5 = inv(P.T5H), TEO = mul(TEH, TH5, P.T5C, P.TCO);
    return {
      1: self(),
      2: ck('필요한 역변환 두 개를 구하시오.', mat('ᴱ_HT', r4(TEH)), mat('ᴴ_5T', r4(TH5))),
      3: ck('ᴱ_OT를 구하시오.', mat('ᴱ_OT', r4(TEO)))
    };
  });
  hook('c1-hw-screw', (b, P) => {
    const k = P.k, kh = unit(k), th = P.adv / P.lead * 360;
    return {
      1: ck('보조각 α, β를 구하시오.', ang('α', atan2d(k[1], k[0])), ang('β', atan2d(Math.hypot(k[0], k[1]), k[2]))),
      2: ck('단위벡터 k̂를 구하시오.', vec3('k̂', kh)),
      3: ck('회전각 θ를 구하시오.', ang('θ', th)),
      4: ck('회전행렬 R_k,θ를 구하시오.', mat('R', R3(rotK(kh, th))))
    };
  });

  /* ===== Ch2 practice ===== */
  hook('c2-hw-3dof', (b, P) => {
    const L1 = 0.35, L2 = 0.25, Dd = 0.05;
    const rows = [{ al: 0, a: 0, d: 0, th: 0, type: 'R' }, { al: 90, a: L1, d: Dd, th: 0, type: 'R' }, { al: 0, a: L2, d: 0, th: 0, type: 'R' }];
    const env = () => angleEnv(3, { L1, L2, D: Dd });
    const V = angVars(3).concat(['L1', 'L2', 'D']), H = symHint('c1, s1, c2, s2, c3, s3, c23, s23, L1, L2, D');
    const Tq = e => dhChain(rows, e, [1, 2, 3]);
    const cd = b.calcData;
    return {
      2: self(),
      3: ck('⁰₁T를 쓰시오.', sym('⁰₁T', 4, 4, e => Tq(e)[1], env, V, { hint: H })),
      4: ck('¹₂T를 쓰시오.', sym('¹₂T', 4, 4, e => sub(Tq(e), 1, 2), env, V, { hint: H })),
      5: ck('²₃T를 쓰시오.', sym('²₃T', 4, 4, e => sub(Tq(e), 2, 3), env, V, { hint: H })),
      6: ck('⁰₂T = ⁰₁T ¹₂T를 쓰시오.', sym('⁰₂T', 4, 4, e => Tq(e)[2], env, V, { hint: H })),
      7: ck('⁰₃T를 쓰시오.', sym('⁰₃T', 4, 4, e => Tq(e)[3], env, V, { hint: H })),
      8: ck('θ₂를 구하시오 (c₂ > 0).', ang('θ₂', cd.t2)),
      9: ck('θ₁을 구하시오.', ang('θ₁', cd.t1n)),
      10: ck('θ₃을 구하시오.', ang('θ₃', cd.t3))
    };
  });
  hook('c2-hw-scara', (b, P) => {
    const L1 = 0.6, L2 = 0.5;
    const rows = [{ al: 0, a: 0, d: 0, th: 0, type: 'R' }, { al: 0, a: L1, d: 0, th: 0, type: 'R' }, { al: 180, a: L2, d: 0, th: 0, type: 'P' }, { al: 0, a: 0, d: 0, th: 0, type: 'R' }];
    const env = () => { const e = angleEnv(4, { L1, L2, d3: rnd(0.05, 0.5) }); return e; };
    const V = angVars(4).concat(['L1', 'L2', 'd3']), H = symHint('c1, s1, c2, s2, c12, s12, c4, s4, L1, L2, d3');
    const Tq = e => dhChain(rows, e, [1, 2, 'd3', 4]);
    const Tn = K.chainT(rows, [P.t1, P.t2, P.d3, P.t4]).Ts;
    return {
      1: self(),
      2: ck('⁰₁T를 쓰시오.', sym('⁰₁T', 4, 4, e => Tq(e)[1], env, V, { hint: H })),
      3: ck('¹₂T를 쓰시오.', sym('¹₂T', 4, 4, e => sub(Tq(e), 1, 2), env, V, { hint: H })),
      4: ck('²₃T를 쓰시오.', sym('²₃T', 4, 4, e => sub(Tq(e), 2, 3), env, V, { hint: H })),
      5: ck('³₄T를 쓰시오.', sym('³₄T', 4, 4, e => sub(Tq(e), 3, 4), env, V, { hint: H })),
      6: ck('⁰₂T와 ⁰₃T를 쓰시오.', sym('⁰₂T', 4, 4, e => Tq(e)[2], env, V, { hint: H }), sym('⁰₃T', 4, 4, e => Tq(e)[3], env, V, { hint: H })),
      7: ck('주어진 관절값에서 ⁰₄T를 숫자로 쓰시오.', mat('⁰₄T', r4(Tn[4])))
    };
  });

  /* ===== Ch3 practice ===== */
  hook('c3-hw-pieper', (b, P) => {
    const { Pt, sol, g } = b.calcData;
    return {
      1: ck('r과 a, b, c를 구하고, θ₃ 후보 두 개를 구하시오 (순서 무관).', num('r', sol.rr, { tol: 0.001 }), num('a', sol.A, { tol: 0.001 }), num('b', sol.B, { tol: 0.001 }), num('c', sol.C, { tol: 0.001 }), set('θ₃ 후보', [sol.cands[0].t3, sol.cands[1].t3])),
      2: ck('θ₁을 구하시오.', ang('θ₁', g.t1)),
      3: ck('조건을 만족하는 θ₂를 구하시오.', ang('θ₂', g.t2)),
      4: ck('⁰₃R과 ³₆R을 구하시오.', mat('⁰₃R', g.R03), mat('³₆R', g.R36)),
      5: ck('θ₄, θ₅, θ₆을 구하시오.', ang('θ₄', g.t4), ang('θ₅', g.t5), ang('θ₆', g.t6))
    };
  });
  hook('c3-hw-3r', (b, P) => {
    const l1 = 1.2, l2 = 1.0, c2 = (P.x * P.x + P.y * P.y - l1 * l1 - l2 * l2) / (2 * l1 * l2), s2 = Math.sqrt(Math.max(0, 1 - c2 * c2));
    const t2 = atan2d(s2, c2), t1 = atan2d(P.y, P.x) - atan2d(l2 * s2, l1 + l2 * c2), t3 = P.phi - t1 - t2;
    return {
      1: ck('c₂와 θ₂를 구하시오 (θ₂ > 0).', num('c₂', c2, { tol: 0.002 }), ang('θ₂', t2)),
      2: ck('θ₁을 구하시오.', ang('θ₁', t1)),
      3: ck('θ₃을 구하시오.', ang('θ₃', t3))
    };
  });

  /* ===== Mock exam ===== */
  const AX = { x: rx, y: ry, z: rz };
  hook('x-p1', (b, P) => {
    const { p, a1, a2, t1, t2, d } = P;
    const R1 = AX[a1](t1), R2 = AX[a2](t2), Tt = tr(...d);
    return {
      1: self(),
      2: ck(`(a) 첫 회전 후 점의 좌표를 구하시오.`, vec3('p', app(R1, p), { tol: 0.01 })),
      3: ck(`(a) 두 번째 회전 후 점의 좌표를 구하시오.`, vec3('p', app(mul(R2, R1), p), { tol: 0.01 })),
      4: ck('(a) p_xyz를 구하시오.', vec3('p_xyz', app(mul(Tt, R2, R1), p), { tol: 0.01 })),
      5: self(),
      6: ck('(b) p_xyz를 구하시오.', vec3('p_xyz', app(mul(R1, Tt, R2), p), { tol: 0.01 }))
    };
  });
  hook('x-p2', (b, P) => {
    const T = mul(tr(...P.p), rz(P.th)), Ti = inv(T);
    return {
      1: ck('(a) ᴬ_BT를 쓰시오.', mat('ᴬ_BT', r4(T))),
      2: ck('(b) ᴬP를 구하시오.', vec3('ᴬP', app(T, P.bp), { tol: 0.01 })),
      3: ck('(c) ᴮ_AT를 쓰시오.', mat('ᴮ_AT', r4(Ti), { tol: 0.01 }))
    };
  });
  hook('x-p3', (b, P) => {
    const r = R3(mul(rz(P.a), ry(P.b), rx(P.g))).map(row => row.map(v => M.round(v, 4)));
    const cb = Math.hypot(r[0][0], r[1][0]);
    return {
      1: self(),
      2: ck('β를 구하시오.', ang('β', atan2d(-r[2][0], cb))),
      3: ck('α를 구하시오.', ang('α', atan2d(r[1][0], r[0][0]))),
      4: ck('γ를 구하시오.', ang('γ', atan2d(r[2][1], r[2][2])))
    };
  });
  hook('x-p4', (b, P) => {
    const kh = unit(P.k), c = Math.cos(P.th * D), s = Math.sin(P.th * D);
    return {
      1: ck('(a) 단위벡터 K̂를 구하시오.', vec3('K̂', kh)),
      2: ck('공식에 쓸 c, s, v를 구하시오.', num('c', c, { tol: 0.002 }), num('s', s, { tol: 0.002 }), num('v', 1 - c, { tol: 0.002 })),
      3: ck('(b) R_K(θ) 전체를 구하시오.', mat('R_K(θ)', R3(rotK(kh, P.th))))
    };
  });
  hook('x-p5', (b, P) => {
    const al = atan2d(P.y, P.x);
    const smp = () => { const a = rnd(-170, 170); return { r: rnd(1, 9), l: rnd(1, 9), ca: Math.cos(a * D), sa: Math.sin(a * D), _a: a }; };
    return {
      1: ck('(a) ⁰_HT를 r, α, l로 쓰시오.', sym('⁰_HT', 4, 4, e => mul(tr(0, 0, e.l), rz(e._a), tr(e.r, 0, 0)), smp, ['r', 'l', 'ca', 'sa'], { hint: symHint('r, l, ca (= cos α), sa (= sin α)') })),
      2: ck('(b) α, r, l을 구하시오.', ang('α', al), num('r', Math.hypot(P.x, P.y)), num('l', P.z))
    };
  });
  hook('x-p6', (b, P) => {
    const tp = window.EXAM.TEMPL[P.kind], rows = tp.rows(P.L), n = rows.length;
    const nR = rows.filter(r => r.type === 'R').length;
    const map = []; let ri = 0; rows.forEach(r => { map.push(r.type === 'P' ? 'd3' : ++ri); });
    const consts = {}; if (P.kind === 'scara') { consts.L1 = P.L[0]; consts.L2 = P.L[1]; } if (P.kind === 'rrr') { consts.L2 = P.L[0]; consts.L3 = P.L[1]; }
    const env = () => angleEnv(nR, Object.assign({ d3: rnd(0.1, 0.6) }, consts));
    // map angle symbols to joint numbers (θ₁, θ₂, θ₄ for SCARA): rename indices so c4 means θ₄
    const jointIdx = rows.map((r, i) => i + 1).filter((j, i) => rows[i].type === 'R');
    const env2 = () => { const e0 = env(); const t = []; jointIdx.forEach((j, i) => { t[j] = e0._t[i + 1]; }); const e = angleEnv(0, Object.assign({}, consts, { d3: e0.d3 })); e._t = t; const C = x => Math.cos(x * D), S = x => Math.sin(x * D); jointIdx.forEach(j => { e['c' + j] = C(t[j]); e['s' + j] = S(t[j]); e['t' + j] = t[j]; }); for (let i = 0; i < jointIdx.length; i++) for (let k = i + 1; k < jointIdx.length; k++) { const js = jointIdx.slice(i, k + 1), key = js.join(''); const sum = js.reduce((a, j) => a + t[j], 0); e['c' + key] = C(sum); e['s' + key] = S(sum); } return e; };
    const vars = []; jointIdx.forEach(j => vars.push('c' + j, 's' + j, 't' + j)); for (let i = 0; i < jointIdx.length; i++) for (let k = i + 1; k < jointIdx.length; k++) { const key = jointIdx.slice(i, k + 1).join(''); vars.push('c' + key, 's' + key); }
    vars.push(...Object.keys(consts), 'd3');
    const Tq = e => K.chainT(rows, rows.map((r, i) => (r.type === 'P' ? e.d3 : e._t[i + 1]))).Ts[n];
    const pos = app(K.chainT(rows, P.q).Ts[n], [tp.tipX ? tp.tipX(P.L) : 0, 0, 0]);
    return {
      1: self(),
      2: ck(`(b) ⁰_${n}T를 쓰시오.`, sym(`⁰_${n}T`, 4, 4, Tq, env2, vars, { hint: symHint(vars.slice(0, 8).join(', ') + (vars.length > 8 ? ' …' : '')) })),
      3: ck('(c) 위치를 구하시오.', num('x', pos[0], { tol: 0.005, unit: 'm' }), num('y', pos[1], { tol: 0.005, unit: 'm' }), num('z', pos[2], { tol: 0.005, unit: 'm' }))
    };
  });
  hook('x-p7', (b, P) => {
    const { L1, L2 } = P;
    const rows = [{ al: 0, a: 0, d: 0, th: 0, type: 'R' }, { al: 90, a: L1, d: 0, th: 0, type: 'R' }, { al: 0, a: L2, d: 0, th: 0, type: 'R' }];
    const env = () => angleEnv(3, { L1, L2 });
    const V = angVars(3).concat(['L1', 'L2']);
    const ans = b.answers.map(a => a.val);
    return {
      1: ck('(b) ⁰₃T를 쓰시오.', sym('⁰₃T', 4, 4, e => dhChain(rows, e, [1, 2, 3])[3], env, V, { hint: symHint('c1, s1, c2, s2, c23, s23, L1, L2') })),
      2: ck('(c) θ₂, θ₁을 구하시오.', ang('θ₂', ans[1], { tol: 0.4 }), ang('θ₁', ans[0], { tol: 0.4 })),
      3: ck('(d) θ₃을 구하시오.', ang('θ₃', ans[2], { tol: 0.4 }))
    };
  });
  hook('x-p8', (b, P) => {
    const c = P.c, k = window.EXAM.pFK(c, P.q);
    const Pt = k.W.map(v => M.round(v, 4)), Rt = k.R06.map(r => r.map(v => M.round(v, 4)));
    const s = window.EXAM.pIK(c, Pt, Rt);
    const gi = s.cands.findIndex(z => z.t2 >= -0.3 && z.t2 <= 90.3 && z.t1 >= -90 && z.t1 <= 90);
    const g = s.cands[gi < 0 ? 0 : gi];
    return {
      2: ck('r과 a, b, c를 구하시오 (a cos θ₃ + b sin θ₃ = c).', num('r', s.rr, { tol: 0.001 }), num('a', s.A, { tol: 0.001 }), num('b', s.B, { tol: 0.001 }), num('c', s.C, { tol: 0.001 })),
      3: ck('θ₃ 후보 두 개를 구하시오 (순서 무관).', set('θ₃ 후보', [s.cands[0].t3, s.cands[1].t3], { tol: 0.5 })),
      4: ck('θ₁을 구하시오.', ang('θ₁', g.t1, { tol: 0.5 })),
      5: ck('조건을 만족하는 θ₂를 구하시오.', ang('θ₂', g.t2, { tol: 0.5 })),
      6: ck('⁰₃R을 구하시오.', mat('⁰₃R', g.R03, { tol: 0.006 })),
      7: ck('³₆R을 구하시오.', mat('³₆R', g.R36, { tol: 0.006 })),
      8: ck('θ₄, θ₅, θ₆을 구하시오.', ang('θ₄', g.t4, { tol: 0.5 }), ang('θ₅', g.t5, { tol: 0.5 }), ang('θ₆', g.t6, { tol: 0.5 }))
    };
  });

  /* ---------------- grading ---------------- */
  const angDiff = (a, b, mod) => mod ? Math.abs(((a - b) % mod + mod * 1.5) % mod - mod / 2) : Math.abs(a - b);
  function evalNum(str) { const fn = parse(str, []); const v = fn({}); if (!isFinite(v)) throw new Error('값이 아님'); return v; }
  // returns array of {ok, msg} per cell (matrices flattened row-major) for an item given raw strings
  function gradeItem(it, raw) {
    if (it.type === 'num') {
      try { const v = evalNum(raw[0]); return [{ ok: angDiff(v, it.val, it.mod) <= it.tol }]; } catch (e) { return [{ ok: false, err: e.message }]; }
    }
    if (it.type === 'set') {
      const vals = raw.map(r => { try { return evalNum(r); } catch (e) { return NaN; } });
      const used = new Set();
      return vals.map(v => { if (isNaN(v)) return { ok: false, err: '숫자를 입력' }; const j = it.val.findIndex((t, i) => !used.has(i) && angDiff(v, t, it.mod) <= it.tol); if (j >= 0) { used.add(j); return { ok: true }; } return { ok: false }; });
    }
    if (it.type === 'mat') {
      const flat = it.val.flat();
      return raw.map((r, i) => { try { const v = evalNum(r); return { ok: Math.abs(v - flat[i]) <= it.tol }; } catch (e) { return { ok: false, err: e.message }; } });
    }
    if (it.type === 'sym') {
      const envs = []; for (let k = 0; k < 5; k++) envs.push(it.sampler());
      const exp = envs.map(e => it.fn(e).slice(0, it.rows).map(r => r.slice(0, it.cols)).flat());
      return raw.map((r, i) => {
        let fn; try { fn = parse(r, it.vars); } catch (e) { return { ok: false, err: e.message }; }
        try { return { ok: envs.every((e, k) => Math.abs(fn(e) - exp[k][i]) <= 1e-6 * (1 + Math.abs(exp[k][i]))) }; } catch (e) { return { ok: false, err: e.message }; }
      });
    }
    return [];
  }
  const cellsOf = it => it.type === 'num' ? 1 : it.type === 'set' ? it.val.length : it.type === 'sym' ? it.rows * it.cols : it.val.length * it.val[0].length;
  const shapeOf = it => it.type === 'sym' ? [it.rows, it.cols] : it.type === 'mat' ? [it.val.length, it.val[0].length] : [1, cellsOf(it)];
  window.SOLVE = { parse, gradeItem, cellsOf, shapeOf };
})();
