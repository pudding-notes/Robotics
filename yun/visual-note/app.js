/* Robotics visual notes — UI / step player */
(function () {
  'use strict';
  const $ = id => document.getElementById(id);
  const COURSE = window.COURSE;
  const store = {
    get(k) { try { return localStorage.getItem(k); } catch (e) { return null; } },
    set(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* ignore */ } }
  };

  let solveMode = store.get('rob.solve') !== '0', revealed = {}, svRaw = {};
  let chap = COURSE.chapters[0], sec = null, built = null, states = [], bases = [], k = 0, params = null, paramMode = 'orig';

  /* ---------- MathJax ---------- */
  let mjReady = (window.MathJax && MathJax.startup && MathJax.startup.promise) || Promise.resolve();
  let mjQueue = Promise.resolve();
  function typeset(el) {
    mjQueue = mjQueue.then(() => mjReady).then(() => {
      if (!window.MathJax || !MathJax.typesetPromise) return;
      MathJax.typesetClear && MathJax.typesetClear([el]);
      return MathJax.typesetPromise([el]);
    }).catch(err => console.warn('MathJax', err));
    return mjQueue;
  }

  /* ---------- nav ---------- */
  const GROUPS = { lecture: '강의 흐름', example: '강의 예제', practice: '과제형 연습' };
  function buildChapters() {
    const box = $('chapters'); box.innerHTML = '';
    COURSE.chapters.concat(COURSE.upcoming || []).forEach(c => {
      const b = document.createElement('button');
      b.textContent = c.short || c.title;
      b.title = c.title;
      if (!c.sections) { b.disabled = true; b.title = c.title + ' (준비 중)'; }
      b.setAttribute('aria-current', c === chap ? 'true' : 'false');
      b.onclick = () => { chap = c; buildChapters(); buildRail(); openSection(c.sections[0].id); };
      box.appendChild(b);
    });
  }
  function buildRail() {
    const rail = $('rail'), sel = $('railsel');
    rail.innerHTML = ''; sel.innerHTML = '';
    Object.keys(GROUPS).forEach(g => {
      const list = chap.sections.filter(s => s.group === g);
      if (!list.length) return;
      const h = document.createElement('h3'); h.textContent = GROUPS[g]; rail.appendChild(h);
      const og = document.createElement('optgroup'); og.label = GROUPS[g]; sel.appendChild(og);
      list.forEach(s => {
        const b = document.createElement('button');
        b.dataset.id = s.id;
        b.innerHTML = `${s.title}<span>${s.ref || ''}</span>`;
        b.onclick = () => openSection(s.id);
        rail.appendChild(b);
        const o = document.createElement('option'); o.value = s.id; o.textContent = s.title; og.appendChild(o);
      });
    });
    sel.onchange = () => openSection(sel.value);
  }
  function markRail() {
    document.querySelectorAll('#rail button').forEach(b => b.setAttribute('aria-current', b.dataset.id === sec.id ? 'true' : 'false'));
    $('railsel').value = sec.id;
  }

  /* ---------- section build ---------- */
  function computeStates() {
    const S0 = ENG.emptyState();
    built.init(S0);
    states = [];
    let S = S0;
    bases = [];
    built.steps.forEach(st => {
      S = structuredClone(S);
      if (st.do) st.do(S);
      if (st.live) {
        bases.push(structuredClone(S));
        st._p = st._p || Object.fromEntries((st.live.params || []).map(q => [q.id, q.val]));
        if (st.live.init) st.live.init(st._p);
        st.live.apply(S, st._p);
      } else bases.push(null);
      states.push(S);
    });
    built.S0 = S0;
  }

  function openSection(id, opts = {}) {
    const s = chap.sections.find(x => x.id === id) || chap.sections[0];
    sec = s; paramMode = 'orig';
    params = s.params ? (s.params.exam ? (paramMode = 'rand', s.params.rand()) : s.params.orig) : null;
    rebuild(true);
    try { history.replaceState(null, '', '#' + s.id); } catch (e) { /* ignore */ }
    store.set('rob.sec', s.id);
    markRail();
    $('cardScroll').scrollTop = 0;
  }

  function rebuild(newSection) {
    built = sec.build(params); revealed = {}; svRaw = {};
    computeStates();
    ENG.setScale(built.scale || sec.scale || 1, built.fsize || sec.fsize || 1);
    ENG.setGrid(...(built.grid || sec.grid || [6, 1]));
    if (newSection) ENG.clearPools();
    if (newSection || built.view) ENG.setView(built.view || sec.view || {}, false);
    $('eyebrow').textContent = `${chap.short} · ${GROUPS[sec.group]}${sec.ref ? ' · ' + sec.ref : ''}`;
    $('secTitle').textContent = sec.title;
    buildParams();
    buildAnswers();
    buildDots();
    go(0, true);
  }

  function buildParams() {
    const box = $('params');
    if (!sec.params) { box.hidden = true; return; }
    box.hidden = false; box.innerHTML = '';
    const mk = (label, mode, fn) => {
      const b = document.createElement('button'); b.textContent = label;
      b.setAttribute('aria-pressed', paramMode === mode ? 'true' : 'false');
      b.onclick = fn; box.appendChild(b);
    };
    if (sec.solvable) {
      const t = document.createElement('button'); t.className = 'sv-toggle'; t.setAttribute('aria-pressed', solveMode ? 'true' : 'false');
      t.textContent = solveMode ? '✍️ 단계별 직접 풀기: 켜짐' : '✍️ 단계별 직접 풀기: 꺼짐';
      t.onclick = () => { solveMode = !solveMode; store.set('rob.solve', solveMode ? '1' : '0'); buildParams(); buildAnswers(); go(k, true); };
      box.appendChild(t);
    }
    if (sec.params.exam) { mk('🎲 새 문제 (숫자 다시 뽑기)', 'x', () => { paramMode = 'rand'; params = sec.params.rand(); rebuild(false); }); return; }
    mk(sec.params.origLabel || '원래 숫자', 'orig', () => { paramMode = 'orig'; params = sec.params.orig; rebuild(false); });
    mk('🎲 새 숫자로 바꾸기', 'rand', () => { paramMode = 'rand'; params = sec.params.rand(); rebuild(false); });
  }

  function buildAnswers() {
    const box = $('answers');
    const ans = built.answers;
    if (!ans || !ans.length || (solveMode && sec.solvable)) { box.hidden = true; return; }
    const ex = sec.params && sec.params.exam;
    box.hidden = false; box.open = false;
    box.innerHTML = `<summary>${ex ? '답안 입력 · 채점' : '직접 풀고 답 확인하기'}</summary>
      <p style="margin:6px 0 4px;font-size:13px;color:var(--mute)">풀이 단계를 넘기기 전에 손으로 먼저 풀어 보세요. 허용 오차 안이면 정답 처리됩니다.</p>`;
    ans.forEach((a, i) => {
      const r = document.createElement('div'); r.className = 'row';
      r.innerHTML = `<label for="ans${i}">${a.label}</label><input id="ans${i}" inputmode="decimal" autocomplete="off"><span>${a.unit || ''}</span><span class="res" id="res${i}"></span>`;
      box.appendChild(r);
    });
    const acts = document.createElement('div'); acts.className = 'acts';
    acts.innerHTML = `<button class="btn primary" id="ansCheck">채점</button><button class="btn" id="ansShow">정답 보기</button>`;
    box.appendChild(acts);
    typeset(box);
    $('ansCheck').onclick = () => ans.forEach((a, i) => {
      const v = parseFloat(($('ans' + i).value || '').replace(',', '.'));
      const res = $('res' + i);
      if (isNaN(v)) { res.textContent = '값을 입력하세요'; res.className = 'res bad'; return; }
      const tol = a.tol != null ? a.tol : Math.max(0.02, Math.abs(a.val) * 0.01);
      const ok = Math.abs(v - a.val) <= tol || (a.mod && Math.abs(((v - a.val) % a.mod + a.mod * 1.5) % a.mod - a.mod / 2) <= tol);
      res.textContent = ok ? '정답' : '다시 확인';
      res.className = 'res ' + (ok ? 'ok' : 'bad');
    });
    $('ansShow').onclick = () => ans.forEach((a, i) => { const res = $('res' + i); res.textContent = '정답: ' + M.f(a.val, a.dp != null ? a.dp : 3) + (a.unit ? ' ' + a.unit : ''); res.className = 'res'; });
  }

  function buildDots() {
    const box = $('dots'); box.innerHTML = '';
    built.steps.forEach((st, i) => {
      const b = document.createElement('button');
      b.textContent = i + 1; b.title = st.t || '';
      b.setAttribute('aria-label', `단계 ${i + 1}: ${st.t || ''}`);
      b.onclick = () => go(i);
      box.appendChild(b);
    });
  }

  /* ---------- stepping ---------- */
  let tweenId = 0;
  function tweenLive(st, from, dur) {
    const my = ++tweenId, to = Object.assign({}, st._p), t0 = performance.now(), kk = k;
    const reduce = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    const tick = now => {
      if (my !== tweenId || kk !== k) return;
      const u = reduce ? 1 : Math.min(1, (now - t0) / dur), e = u < 0.5 ? 4 * u * u * u : 1 - Math.pow(-2 * u + 2, 3) / 2;
      const p = {};
      for (const key in to) p[key] = (typeof to[key] === 'number' && typeof from[key] === 'number') ? from[key] + (to[key] - from[key]) * e : to[key];
      const S = structuredClone(bases[kk]); st.live.apply(S, p); ENG.snap(S);
      if (u < 1) requestAnimationFrame(tick); else liveRefresh();
    };
    requestAnimationFrame(tick);
  }
  function go(n, first) {
    tweenId++;
    n = Math.max(0, Math.min(built.steps.length - 1, n));
    const prevK = k; k = n;
    const st = built.steps[k];
    if (!first && built.steps.some(x => x.view)) ENG.setView(st.view || built.view || sec.view || {}, false);
    let before;
    if (first || n === prevK + 1 || n === prevK) {
      before = structuredClone(n === 0 ? built.S0 : shownState(n - 1));
      if (st.pre) st.pre(before);
      if (st.live && st.live.from) tweenLive(st, typeof st.live.from === 'function' ? st.live.from(st._p) : st.live.from, st.dur || 1600);
      else ENG.play(before, shownState(n), isHidden(n) ? null : st.motion, st.dur);
    } else {
      // jump: short morph without scripted motion
      ENG.play(structuredClone(states[prevK] || built.S0), shownState(n), null, 600);
    }
    renderCard();
  }

  function renderCard() {
    const st = built.steps[k];
    document.querySelectorAll('#dots button').forEach((b, i) => {
      b.setAttribute('aria-current', i === k ? 'true' : 'false');
      b.classList.toggle('done', i < k);
    });
    $('stepTitle').textContent = st.t || '';
    $('vpStep').textContent = `${k + 1}/${built.steps.length} · ${st.t || ''}`;
    const tex = st.live && st.live.tex ? st.live.tex(st._p) : st.tex;
    $('formulaWrap').hidden = !tex;
    renderTex(tex);
    updateLegend(states[k]);
    $('expl').innerHTML = (st.live && st.live.html ? st.live.html(st._p) : st.html) || '';
    typeset($('expl'));
    buildLive(st);
    renderAssume(st);
    renderVpInfo(st);
    renderCalc(st);
    $('prev').disabled = k === 0;
    $('next').disabled = k === built.steps.length - 1;
    $('next').textContent = k === built.steps.length - 1 ? '끝' : '다음 ▶';
    const w = states[k].watch;
    $('watch').hidden = !w;
    if (w) { $('watchT').innerHTML = w.title || ''; typeset($('watchT')); }
    renderSolve(st);
  }





  /* ---------- step-by-step self check ---------- */
  function shownState(n) { let m = n; while (m >= 0 && isHidden(m)) m--; return m < 0 ? built.S0 : states[m]; }
  function isHidden(n) { const st = built.steps[n]; return !!(solveMode && st && st.check && !revealed[n]); }
  const escA = t => String(t).replace(/&/g, '&amp;').replace(/"/g, '&quot;').replace(/</g, '&lt;');
  function reveal(n, quiet) {
    if (revealed[n]) return;
    revealed[n] = true;
    if (n === k) { revealed[n] = false; const from = structuredClone(shownState(n)); revealed[n] = true; ENG.play(from, states[n], built.steps[n].motion, built.steps[n].dur); renderCard(); }
  }
  function renderSolve(st) {
    const box = $('solve');
    const on = solveMode && st.check;
    box.hidden = !on;
    $('expl').hidden = false;
    if (!on) return;
    const hide = !revealed[k];
    if (hide) {
      $('formulaWrap').hidden = true; $('expl').hidden = true; $('assume').hidden = true; $('calc').hidden = true; $('watch').hidden = true;
      $('vpInfo').hidden = true;
    } else { $('expl').hidden = false; }
    const sp = st.check, raw = svRaw[k] = svRaw[k] || sp.items.map(it => Array(SOLVE.cellsOf(it)).fill(''));
    let h = `<div class="sv-h">✍️ 직접 풀기${revealed[k] ? ' <span class="sv-done">풀이 펼침</span>' : ''}</div><div class="sv-q">${sp.prompt}</div>`;
    sp.items.forEach((it, i) => {
      const [r, c] = SOLVE.shapeOf(it), isM = it.type === 'mat' || it.type === 'sym';
      h += `<div class="sv-item"><div class="sv-l">${escA(it.label)}${isM ? ` <span class="sv-dim">(${r}×${c}, 모든 칸)</span>` : ''}</div>`;
      h += `<div class="sv-grid${isM ? ' mat' : ''}" style="grid-template-columns:repeat(${c}, ${isM ? (it.type === 'sym' ? 'minmax(58px,1fr)' : 'minmax(48px,1fr)') : 'minmax(70px,110px)'})">`;
      for (let j = 0; j < r * c; j++) h += `<input class="sv-in" id="sv_${i}_${j}" data-i="${i}" data-j="${j}" value="${escA(raw[i][j])}" autocomplete="off" spellcheck="false" inputmode="${it.type === 'sym' ? 'text' : 'decimal'}">`;
      h += `</div>${it.unit ? `<span class="sv-u">${it.unit}</span>` : ''}</div>`;
      if (it.hint) h += `<div class="sv-hint">${it.hint}</div>`;
    });
    h += `<div class="sv-acts">${sp.self ? '' : '<button class="btn primary" id="svCheck">채점</button>'}<button class="btn" id="svShow">${revealed[k] ? '풀이 펼쳐짐' : '풀이 보기'}</button><span class="sv-msg" id="svMsg"></span></div>`;
    if (!sp.self) h += `<div class="sv-hint">숫자 칸에는 식도 됩니다 (예: sqrt(3)/2, cos(30) — 각도는 도 단위). 칸이 초록이면 맞음, 빨강이면 다시 확인.</div>`;
    box.innerHTML = h;
    box.querySelectorAll('.sv-in').forEach(inp => {
      inp.oninput = () => { raw[+inp.dataset.i][+inp.dataset.j] = inp.value; inp.classList.remove('ok', 'bad'); };
      inp.onkeydown = e => { if (e.key === 'Enter') { e.preventDefault(); grade(); } };
    });
    const grade = () => {
      let all = true, empty = false;
      sp.items.forEach((it, i) => {
        if (raw[i].some(v => !String(v).trim())) empty = true;
        const res = SOLVE.gradeItem(it, raw[i]);
        res.forEach((r2, j) => { const el = $(`sv_${i}_${j}`); el.classList.toggle('ok', r2.ok); el.classList.toggle('bad', !r2.ok); el.title = r2.err || ''; if (!r2.ok) all = false; });
      });
      const msg = $('svMsg');
      if (empty) { msg.textContent = '빈칸이 있어요 — 모든 칸을 채워 주세요'; msg.className = 'sv-msg bad'; return; }
      if (all) { msg.textContent = '모두 정답!'; msg.className = 'sv-msg ok'; if (!revealed[k]) setTimeout(() => reveal(k), 500); }
      else { const n = box.querySelectorAll('.sv-in.bad').length; msg.textContent = `${n}칸 다시 확인`; msg.className = 'sv-msg bad'; }
    };
    if ($('svCheck')) $('svCheck').onclick = grade;
    $('svShow').onclick = () => reveal(k);
    // keep previous marks after re-render
    if (revealed[k] && !sp.self && raw.some(r2 => r2.some(v => String(v).trim()))) grade();
  }
  /* ---------- viewport info card (what the 3D shows, in numbers) ---------- */
  function renderVpInfo(st) {
    const box = $('vpInfo');
    const v = st.live && st.live.vpInfo ? st.live.vpInfo(st._p) : (typeof st.vpInfo === 'function' ? st.vpInfo() : st.vpInfo);
    box.hidden = !v;
    if (!v) return;
    if (v === box.dataset.h) return;
    box.dataset.h = v; box.innerHTML = v; typeset(box);
  }
  /* ---------- CASIO fx-570ES PLUS key guide ---------- */
  const escH = t => String(t).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
  function keyTokens(str) {
    const out = []; let i = 0;
    while (i < str.length) {
      if (str[i] === ' ') { i++; continue; }
      if (str[i] === '«') { const j = str.indexOf('»', i); out.push({ typed: str.slice(i + 1, j) }); i = j + 1; continue; }
      let j = str.indexOf(' ', i); if (j < 0) j = str.length;
      const t = str.slice(i, j), c = t.indexOf(':', 1);
      out.push(c > 0 ? { key: t.slice(0, c), sub: t.slice(c + 1) } : { key: t });
      i = j;
    }
    return out;
  }
  function keyHtml(t) {
    if (t.typed != null) return `<span class="typed" title="이대로 입력">${escH(t.typed)}</span>`;
    const k = t.key;
    const cls = k === 'SHIFT' ? 'sh' : k === 'ALPHA' ? 'al' : k === '=' ? 'eq' : k === 'AC' ? 'ac' : /^[0-9.]$/.test(k) ? 'num' : /^(MODE|RCL|S⇔D|CALC)$/.test(k) ? 'md' : '';
    return `<span class="kk ${cls}${t.sub ? ' wl' : ''}">${escH(k)}${t.sub ? `<small>${escH(t.sub)}</small>` : ''}</span>`;
  }
  function lcdHtml(o) {
    if (o == null || o === '') return '';
    if (o.mat) return `<div class="lcdbox" title="계산기 화면"><table class="br">${o.mat.map(r => `<tr>${r.map(v => `<td>${escH(v)}</td>`).join('')}</tr>`).join('')}</table></div>`;
    return `<div class="lcdbox" title="계산기 화면">${escH(o)}</div>`;
  }
  function renderCalc(st) {
    const box = $('calc'), rows = st.calc;
    box.hidden = !rows || !rows.length;
    if (box.hidden) return;
    $('calcList').innerHTML = rows.map(r => {
      const keys = r.keys ? keyTokens(r.keys) : [];
      return `<li class="${keys.length ? '' : 'nokey'}">${keys.length ? `<div class="ks">${keys.map(keyHtml).join('')}</div>` : ''}${lcdHtml(r.out)}<div class="cd">${r.d || ''}</div></li>`;
    }).join('');
    const inCalc = chap && chap.id === 'calc';
    const pref = store.get('rob.calcOpen');
    box.open = inCalc || pref === '1';
    typeset($('calcList'));
  }

  $('calc').addEventListener('toggle', () => { if (!(chap && chap.id === 'calc')) store.set('rob.calcOpen', $('calc').open ? '1' : '0'); });
  /* ---------- formula rendering (off-screen, no flicker) ---------- */
  let texBusy = false, texNext = null, lastTex = '';
  /* Break a display formula into lines at top-level separators (outside braces, matrices, \left..\right).
     level 1: \qquad and ",\quad"   level 2: + ⇒ arrows   level 3: + "=" (except the first in a row) */
  function breakTex(tex, level) {
    const s = tex.trim();
    let base = 0, env = '', body = s;
    const m = s.match(/^\\begin\{(aligned|gathered)\}/);
    if (m && s.endsWith('\\end{' + m[1] + '}')) {
      let d = 0, ok = true; const re = /\\(begin|end)\{[a-z*]+\}/g; let t;
      while ((t = re.exec(s))) { d += t[1] === 'begin' ? 1 : -1; if (d === 0 && re.lastIndex < s.length) { ok = false; break; } }
      if (ok) { base = 1; env = m[1]; body = s.slice(m[0].length, s.length - ('\\end{' + env + '}').length); }
    }
    // env stack: each entry {name, br}; breakable when innermost env is aligned/gathered (or none) and braces balanced relative to it
    const stack = [{ name: base ? env : 'top', br: 0 }];
    let out = '', i = 0, br = 0, lr = 0, changed = false;
    const rowEq = [0];
    const cur = () => stack[stack.length - 1];
    const brk = () => lr === 0 && br === cur().br && /^(top|aligned|gathered)$/.test(cur().name);
    const NL = () => (cur().name === 'gathered' ? '\\\\ ' : '\\\\ &');
    while (i < body.length) {
      const rest = body.slice(i);
      let mm;
      if ((mm = rest.match(/^\\(begin|end)\{([a-z*]+)\}/))) {
        if (mm[1] === 'begin') {
          if (level >= 4 && brk() && mm[2] === 'bmatrix' && /(\\end\{bmatrix\}|\})\s*$/.test(out) && !/=\s*$/.test(out)) { out += NL() + '\\qquad\\cdot '; changed = true; }
          stack.push({ name: mm[2], br }); rowEq.push(0);
        } else { stack.pop(); rowEq.pop(); }
        out += mm[0]; i += mm[0].length; continue;
      }
      if ((mm = rest.match(/^\\(left|right)(\\[a-zA-Z]+|.)/))) { lr += mm[1] === 'left' ? 1 : -1; out += mm[0]; i += mm[0].length; continue; }
      if (rest.startsWith('\\\\')) { if (brk()) rowEq[rowEq.length - 1] = 0; out += '\\\\'; i += 2; continue; }
      if ((mm = rest.match(/^\\[a-zA-Z]+/))) {
        const cmd = mm[0];
        if (brk() && level >= 1 && cmd === '\\qquad' && out.trim().length) { out += NL() + ' '; i += cmd.length; rowEq[rowEq.length - 1] = 0; changed = true; continue; }
        if (brk() && level >= 2 && /^\\(Rightarrow|Longrightarrow|Longleftrightarrow|xrightarrow)$/.test(cmd) && out.trim().length && !/(&|\\\\)\s*$/.test(out)) { out += NL() + '\\qquad ' + cmd; i += cmd.length; changed = true; continue; }
        if (level >= 4 && brk() && cmd === '\\underbrace' && /\}\s*$/.test(out) && !/=\s*$/.test(out)) { out += NL() + '\\qquad\\cdot '; changed = true; }
        out += cmd; i += cmd.length; continue;
      }
      const ch = body[i];
      if (ch === '\\') { out += body.slice(i, i + 2); i += 2; continue; }
      if (ch === '{') br++; else if (ch === '}') br--;
      if (brk() && level >= 1 && rest.startsWith(',\\quad')) { out += ',' + NL() + ' '; i += 6; rowEq[rowEq.length - 1] = 0; changed = true; continue; }
      if (brk() && level >= 1 && (mm = rest.match(/^,(\\ ){2,}/))) { out += ',' + NL() + ' '; i += mm[0].length; rowEq[rowEq.length - 1] = 0; changed = true; continue; }
      if (brk() && ch === '=') {
        const prev = out.replace(/\s+$/, '').slice(-1);
        if (prev !== '<' && prev !== '>' && prev !== '!') {
          rowEq[rowEq.length - 1]++;
          if (level >= 3 && rowEq[rowEq.length - 1] > 1 && prev !== '&') { out += NL() + '\\qquad ='; i++; changed = true; continue; }
        }
      }
      out += ch; i++;
    }
    if (!changed) return null;
    return base ? `\\begin{${env}}${out}\\end{${env}}` : `\\begin{aligned}&${out}\\end{aligned}`;
  }

  function renderTex(tex) {
    texNext = tex == null ? '' : tex;
    if (texBusy) return;
    texBusy = true;
    const t = texNext; texNext = null;
    const fEl = $('formula');
    const measure = async (src) => {
      const node = await MathJax.tex2svgPromise(src, { display: true });
      const box = $('texMeasure'); box.innerHTML = ''; box.appendChild(node);
      const svg = node.querySelector('svg');
      const w = svg ? svg.getBoundingClientRect().width : 0;
      return { node, w };
    };
    mjReady.then(async () => {
      if (!t) { fEl.innerHTML = ''; return; }
      if (!window.MathJax || !MathJax.tex2svgPromise) { fEl.textContent = t; return; }
      $('texMeasure').style.width = fEl.clientWidth + 'px';
      const avail = fEl.clientWidth - 8;
      let best = await measure(t), src = t;
      const ratio0 = avail > 0 && best.w > 0 ? avail / best.w : 1;
      if (ratio0 < 0.86) {
        for (let lv = 1; lv <= 4; lv++) {
          const b = breakTex(t, lv); if (!b || b === src) continue;
          let r; try { r = await measure(b); } catch (e) { continue; }
          if (!r.node.querySelector('mjx-merror, [data-mjx-error]') && r.w < best.w - 1) { best = r; src = b; }
          if (avail / best.w >= 0.86) break;
        }
      }
      fEl.innerHTML = ''; fEl.appendChild(best.node);
      best.node.style.fontSize = '';
      fEl.dataset.ratio0 = ratio0.toFixed(3);
      fEl.dataset.ratio = (avail > 0 && best.w > 0) ? (avail / best.w).toFixed(3) : '1';
      if (best.w > avail && avail > 0) best.node.style.fontSize = Math.max(76, Math.floor(94 * avail / best.w)) + '%';
      lastTex = t;
    }).catch(err => console.warn('MathJax', err)).then(() => {
      texBusy = false;
      if (texNext !== null) renderTex(texNext);
    });
  }




  /* ---------- big formula view ---------- */
  $('fzBtn').onclick = () => {
    if (!lastTex) return;
    const ov = document.createElement('div'); ov.className = 'fz';
    ov.innerHTML = '<div class="fz-in" role="dialog" aria-label="수식 크게 보기"><button class="fz-close">닫기 ✕</button><div class="fz-body"></div></div>';
    document.body.appendChild(ov);
    const close = () => ov.remove();
    ov.querySelector('.fz-close').onclick = close;
    ov.addEventListener('click', e => { if (e.target === ov) close(); });
    document.addEventListener('keydown', function esc(e) { if (e.key === 'Escape') { close(); document.removeEventListener('keydown', esc); } });
    MathJax.tex2svgPromise(lastTex, { display: true }).then(n => ov.querySelector('.fz-body').appendChild(n));
  };

  /* ---------- resizable card panel ---------- */
  (function () {
    const sp = $('split'), app = $('app');
    const saved = parseInt(store.get('rob.cardw'), 10);
    if (saved > 300) app.style.setProperty('--cardw', saved + 'px');
    let drag = null;
    sp.addEventListener('pointerdown', e => { drag = { x: e.clientX, w: document.querySelector('section.card').getBoundingClientRect().width }; sp.classList.add('on'); sp.setPointerCapture(e.pointerId); e.preventDefault(); });
    sp.addEventListener('pointermove', e => { if (!drag) return; const w = Math.max(340, Math.min(window.innerWidth - 420, drag.w + e.clientX - drag.x)); app.style.setProperty('--cardw', w + 'px'); });
    const end = () => { if (!drag) return; drag = null; sp.classList.remove('on'); store.set('rob.cardw', parseInt(getComputedStyle(app).getPropertyValue('--cardw'), 10)); if (built && built.steps[k] && built.steps[k].live && built.steps[k].live.tex) renderTex(built.steps[k].live.tex(built.steps[k]._p)); else if (built) renderTex(built.steps[k].tex); };
    sp.addEventListener('pointerup', end); sp.addEventListener('pointercancel', end);
  })();

  /* ---------- joint colour legend ---------- */
  function updateLegend(S) {
    const box = $('jleg'); if (!S) { box.innerHTML = ''; return; }
    const set = new Set();
    for (const id in S.bodies) { const b = S.bodies[id]; if (b && /^j\d$/.test(b.c || '') && (b.op == null || b.op > 0.2)) set.add(+b.c.slice(1)); }
    const arr = [...set].sort();
    box.innerHTML = arr.length ? arr.map(i => `<span style="background:var(--j${i})">J${i}</span>`).join('') : '';
  }

  /* ---------- assumptions box ---------- */
  function renderAssume(st) {
    const box = $('assume');
    const list = st.live && st.live.assume ? st.live.assume(st._p) : (typeof st.assume === 'function' ? st.assume() : st.assume);
    if (!list || !list.length) { box.hidden = true; box.innerHTML = ''; box.dataset.h = ''; return; }
    const html = '<div class="as-head">가정 · 조건 <span>(3D에 함께 표시)</span></div>' + list.map(a => {
      const o = typeof a === 'string' ? { t: a } : a;
      const cls = o.ok === true ? 'ok' : o.ok === false ? 'bad' : '';
      const mark = o.ok === true ? '✓' : o.ok === false ? '✗' : '•';
      return `<div class="as-row ${cls}"><span class="as-m">${mark}</span><span>${o.t}</span></div>`;
    }).join('');
    if (box.dataset.h === html) return;
    box.dataset.h = html; box.hidden = false; box.innerHTML = html; typeset(box);
  }

  /* ---------- live controls (sliders + 3D drag) ---------- */
  function liveRefresh(fromDrag) {
    const st = built.steps[k];
    if (!st.live) return;
    const S = structuredClone(bases[k]);
    st.live.apply(S, st._p);
    states[k] = S;
    ENG.snap(S);
    updateLegend(S);
    if (st.live.tex) renderTex(st.live.tex(st._p));
    if (st.live.html) { const h = st.live.html(st._p); if (h !== $('expl').dataset.h) { $('expl').dataset.h = h; $('expl').innerHTML = h; typeset($('expl')); } }
    if (st.live.onChange) st.live.onChange(st._p, $('liveCustom'));
    if (st.live.assume) renderAssume(st);
    if (st.live.vpInfo) renderVpInfo(st);
    (st.live.params || []).forEach(q => {
      const r = document.getElementById('lv_' + q.id), n = document.getElementById('lvn_' + q.id);
      if (r && document.activeElement !== r) r.value = st._p[q.id];
      if (n && document.activeElement !== n) n.value = +(+st._p[q.id]).toFixed(q.dp != null ? q.dp : 2);
    });
  }
  function buildLive(st) {
    const box = $('live');
    if (!st.live) { box.hidden = true; box.innerHTML = ''; ENG.dragHandler = null; return; }
    box.hidden = false;
    const L = st.live;
    const drag = L.drag && Object.keys(L.drag).length;
    box.innerHTML = `<div class="lv-head"><b>직접 바꿔 보기</b><span>${drag ? '슬라이더를 움직이거나 3D 화면에서 링크를 잡고 끌어 보세요' : '슬라이더를 움직여 보세요'}</span><button class="btn sm" id="lvReset">기본값</button></div>`;
    (L.params || []).forEach(q => {
      const row = document.createElement('div'); row.className = 'lv-row';
      row.innerHTML = `<label for="lv_${q.id}">${q.label}</label><input type="range" id="lv_${q.id}" min="${q.min}" max="${q.max}" step="${q.step || 1}" value="${st._p[q.id]}"><input type="number" class="num" id="lvn_${q.id}" min="${q.min}" max="${q.max}" step="${q.step || 1}" value="${st._p[q.id]}"><span class="u">${q.unit || ''}</span>`;
      box.appendChild(row);
      const set = v => { v = Math.max(q.min, Math.min(q.max, parseFloat(v))); if (isNaN(v)) return; tweenId++; st._p[q.id] = v; st._p._active = q.id; liveRefresh(); };
      row.querySelector('input[type=range]').addEventListener('input', ev => set(ev.target.value));
      row.querySelector('input.num').addEventListener('change', ev => set(ev.target.value));
    });
    const cu = document.createElement('div'); cu.id = 'liveCustom'; box.appendChild(cu);
    if (L.custom) L.custom(cu, st._p, () => liveRefresh());
    $('lvReset').onclick = () => {
      Object.assign(st._p, Object.fromEntries((L.params || []).map(q => [q.id, q.val]))); delete st._p._active;
      if (L.reset) L.reset(st._p);
      if (L.custom) { cu.innerHTML = ''; L.custom(cu, st._p, () => liveRefresh()); }
      liveRefresh();
    };
    typeset(box);
    ENG.dragHandler = drag ? makeDrag(st) : null;
  }
  function makeDrag(st) {
    const V = M; let ctx = null;
    const axisOf = spec => { const T = ENG.F[spec.frame]; if (!T) return null; return { c: V.p3(T), k: V.unit(V.col(T, 2)) }; };
    const onPlane = (R, ax) => { const den = V.dot(R.d, ax.k); if (Math.abs(den) < 1e-4) return null; const t = V.dot(V.sub(ax.c, R.o), ax.k) / den; const q = V.add(R.o, V.scl(R.d, t)); const v = V.sub(q, ax.c); return V.sub(v, V.scl(ax.k, V.dot(v, ax.k))); };
    const onLine = (R, ax) => { const w = V.sub(R.o, ax.c), b = V.dot(ax.k, R.d), d = V.dot(ax.k, w), e = V.dot(R.d, w), den = 1 - b * b; if (Math.abs(den) < 1e-6) return null; return (d - b * e) / den; };
    return {
      start(id, R) {
        const spec = st.live.drag[id]; if (!spec) return;
        tweenId++;
        const ax = axisOf(spec); if (!ax) return;
        st._p._active = id;
        if (spec.type === 'plane') {
          const T = ENG.F[spec.frame], X = V.col(T, 0), Y = V.col(T, 1);
          const v = onPlane(R, ax); if (!v) return;
          const cur = spec.get(st._p);
          ctx = { id, spec, ax, X, Y, off: [V.dot(v, X) - cur[0], V.dot(v, Y) - cur[1]] };
          return;
        }
        ctx = { id, spec, ax, v: spec.type === 'lin' ? onLine(R, ax) : onPlane(R, ax) };
      },
      move(id, R) {
        if (!ctx) return;
        if (ctx.spec.type === 'plane') {
          const v = onPlane(R, ctx.ax); if (!v) return;
          ctx.spec.set(st._p, [V.dot(v, ctx.X) - ctx.off[0], V.dot(v, ctx.Y) - ctx.off[1]]);
          liveRefresh(true); return;
        }
        const q = (st.live.params || []).find(x => x.id === ctx.id) || ctx.spec;
        const get = () => ctx.spec.get ? ctx.spec.get(st._p) : st._p[ctx.id];
        const put = v => { if (ctx.spec.set) ctx.spec.set(st._p, v); else st._p[ctx.id] = v; };
        let val = get();
        if (ctx.spec.type === 'lin') {
          const s = onLine(R, ctx.ax); if (s == null || ctx.v == null) return;
          val += (s - ctx.v) * (ctx.spec.sign || 1); ctx.v = s;
        } else {
          const v = onPlane(R, ctx.ax); if (!v || !ctx.v || V.norm(v) < 1e-6) return;
          const ang = Math.atan2(V.dot(V.cross(ctx.v, v), ctx.ax.k), V.dot(ctx.v, v)) / V.D;
          val += ang * (ctx.spec.sign || 1); ctx.v = v;
        }
        val = Math.max(q.min != null ? q.min : -1e9, Math.min(q.max != null ? q.max : 1e9, val));
        put(val);
        liveRefresh(true);
      },
      end() { ctx = null; (st.live.params || []).forEach(x => { if (x.step >= 1) st._p[x.id] = Math.round(st._p[x.id]); }); if (st.live.dragEnd) st.live.dragEnd(st._p); liveRefresh(); }
    };
  }

  /* live matrix readout */
  ENG.onTick = (F, after) => {
    const w = after.watch;
    if (!w || !F[w.f]) return;
    const T = M.mul(w.rel && F[w.rel] ? M.inv(F[w.rel]) : M.I(), F[w.f]);
    const cells = [];
    T.forEach((r, i) => r.forEach((x, j) => {
      const v = Math.abs(x) < 5e-4 ? 0 : x;
      cells.push(`<span class="${j === 3 && i < 3 ? 'c3' : ''}">${(i === 3) ? (j === 3 ? '1' : '0') : v.toFixed(3)}</span>`);
    }));
    $('watchM').innerHTML = cells.join('');
  };

  $('prev').onclick = () => go(k - 1);
  $('next').onclick = () => go(k + 1);
  $('replay').onclick = () => { const n = k; k = n - 1; go(n, true); };
  $('resetView').onclick = () => ENG.setView((built && built.steps[k] && built.steps[k].view) || (built && built.view) || sec.view || {}, false);
  document.addEventListener('keydown', e => {
    if (e.target && /INPUT|SELECT|TEXTAREA/.test(e.target.tagName)) return;
    if (e.key === 'ArrowRight') { go(k + 1); e.preventDefault(); }
    if (e.key === 'ArrowLeft') { go(k - 1); e.preventDefault(); }
  });

  /* theme changes */
  const mq = window.matchMedia('(prefers-color-scheme: dark)');
  const rt = () => setTimeout(() => ENG.retheme(), 30);
  mq.addEventListener && mq.addEventListener('change', rt);
  new MutationObserver(rt).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme', 'class', 'style'] });

  /* boot */
  ENG.initScene($('vp'));
  const hash = (location.hash || '').replace('#', '');
  const want = [hash, store.get('rob.sec')].find(id => id && COURSE.chapters.some(c => c.sections.some(s => s.id === id)));
  if (want) chap = COURSE.chapters.find(c => c.sections.some(s => s.id === want));
  buildChapters(); buildRail();
  openSection(want || chap.sections[0].id);
})();
