/* =========================================================
   Robotics visual notes — math + 3D step engine
   ========================================================= */
(function () {
  'use strict';

  /* ---------------- math (row-major 4x4 nested arrays) ---------------- */
  const D = Math.PI / 180;
  const I = () => [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
  const mul2 = (a, b) => a.map(r => [0, 1, 2, 3].map(j => r[0] * b[0][j] + r[1] * b[1][j] + r[2] * b[2][j] + r[3] * b[3][j]));
  const mul = (...ms) => ms.reduce(mul2);
  const cs = d => [Math.cos(d * D), Math.sin(d * D)];
  const rx = d => { const [c, s] = cs(d); return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]; };
  const ry = d => { const [c, s] = cs(d); return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]; };
  const rz = d => { const [c, s] = cs(d); return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]; };
  const tr = (x, y, z) => Array.isArray(x) ? tr(x[0], x[1], x[2]) : [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]];
  const norm = v => Math.hypot(v[0], v[1], v[2]);
  const unit = v => { const n = norm(v) || 1; return v.map(x => x / n); };
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const add = (a, b) => a.map((x, i) => x + b[i]);
  const sub = (a, b) => a.map((x, i) => x - b[i]);
  const scl = (a, s) => a.map(x => x * s);
  const rotK = (k, d) => {
    const [x, y, z] = unit(k), [c, s] = cs(d), v = 1 - c;
    return [
      [x * x * v + c, x * y * v - z * s, x * z * v + y * s, 0],
      [x * y * v + z * s, y * y * v + c, y * z * v - x * s, 0],
      [x * z * v - y * s, y * z * v + x * s, z * z * v + c, 0],
      [0, 0, 0, 1]];
  };
  const R3 = T => T.slice(0, 3).map(r => r.slice(0, 3));
  const p3 = T => [T[0][3], T[1][3], T[2][3]];
  const fromRp = (R, p) => [[...R[0], p[0]], [...R[1], p[1]], [...R[2], p[2]], [0, 0, 0, 1]];
  const RT = R => [0, 1, 2].map(i => [0, 1, 2].map(j => R[j][i]));
  const mv3 = (R, v) => R.map(r => r[0] * v[0] + r[1] * v[1] + r[2] * v[2]);
  const inv = T => { const Rt = RT(R3(T)); return fromRp(Rt, scl(mv3(Rt, p3(T)), -1)); };
  const app = (T, p) => [0, 1, 2].map(i => T[i][0] * p[0] + T[i][1] * p[1] + T[i][2] * p[2] + T[i][3]);
  const dir = (T, v) => [0, 1, 2].map(i => T[i][0] * v[0] + T[i][1] * v[1] + T[i][2] * v[2]);
  const col = (T, j) => [T[0][j], T[1][j], T[2][j]];
  const toAA = T => {
    const R = R3(T);
    const tc = Math.max(-1, Math.min(1, (R[0][0] + R[1][1] + R[2][2] - 1) / 2));
    const th = Math.acos(tc);
    if (th < 1e-6) return { axis: [0, 0, 1], deg: 0 };
    if (Math.PI - th < 1e-4) {
      const xx = Math.sqrt(Math.max(0, (R[0][0] + 1) / 2)), yy = Math.sqrt(Math.max(0, (R[1][1] + 1) / 2)), zz = Math.sqrt(Math.max(0, (R[2][2] + 1) / 2));
      let ax;
      if (xx >= yy && xx >= zz) ax = [xx, R[0][1] / (2 * xx), R[0][2] / (2 * xx)];
      else if (yy >= zz) ax = [R[0][1] / (2 * yy), yy, R[1][2] / (2 * yy)];
      else ax = [R[0][2] / (2 * zz), R[1][2] / (2 * zz), zz];
      return { axis: unit(ax), deg: 180 };
    }
    const k = [R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]];
    return { axis: unit(k), deg: th / D };
  };
  const atan2d = (y, x) => Math.atan2(y, x) / D;
  const round = (x, n = 3) => { const f = Math.pow(10, n); const v = Math.round(x * f) / f; return Object.is(v, -0) ? 0 : v; };
  const f = (x, n = 3) => {
    let v = round(x, n);
    if (Math.abs(v) < Math.pow(10, -n) / 2) v = 0;
    let s = v.toFixed(n);
    if (s.indexOf('.') >= 0) s = s.replace(/0+$/, '').replace(/\.$/, '');
    if (s === '-0') s = '0';
    return s;
  };
  const fx = (x, n = 3) => { const s = f(x, n); return s; };
  /* TeX helpers */
  const tm = (A, n = 3, cls) => '\\begin{bmatrix}' + A.map((r, i) => r.map((x, j) => {
    const s = f(x, n); const c = cls && cls(i, j); return c ? `\\class{${c}}{${s}}` : s;
  }).join(' & ')).join(' \\\\ ') + '\\end{bmatrix}';
  const tv = (v, n = 3, cls) => '\\begin{bmatrix}' + v.map((x, i) => { const s = f(x, n); const c = cls && cls(i); return c ? `\\class{${c}}{${s}}` : s; }).join(' \\\\ ') + '\\end{bmatrix}';
  const tvr = (v, n = 3) => '\\begin{bmatrix}' + v.map(x => f(x, n)).join(' & ') + '\\end{bmatrix}';

  const MM = { D, I, mul, rx, ry, rz, tr, rotK, R3, p3, fromRp, RT, mv3, inv, app, dir, col, toAA, norm, unit, dot, cross, add, sub, scl, atan2d, round, f, fx, tm, tv, tvr };
  window.M = MM;

  /* ---------------- palette ---------------- */
  const cssv = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
  let PAL = {};
  const ROLES = ['x', 'y', 'z', 'pt', 'k', 'ink', 'mute', 'acc', 'grid', 'grid2', 'body', 'body2', 'warn', 'ok', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6'];
  function readPal() { ROLES.forEach(r => PAL[r] = cssv('--' + (r === 'k' ? 'kc' : r === 'x' ? 'xc' : r === 'y' ? 'yc' : r === 'z' ? 'zc' : r))); }
  const color = c => new THREE.Color(PAL[c] || c || PAL.ink);

  /* ---------------- three scene ---------------- */
  let renderer, labelR, scene, camera, controls, root, gridObj, host;
  let unitScale = 1; // size of arrows radius etc.
  const pools = { frames: {}, pts: {}, vecs: {}, lines: {}, arcs: {}, bodies: {} };

  function initScene(el) {
    host = el;
    readPal();
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setClearColor(0x000000, 0);
    el.appendChild(renderer.domElement);
    labelR = new THREE.CSS2DRenderer();
    labelR.domElement.className = 'labels';
    el.appendChild(labelR.domElement);
    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(38, 1, 0.05, 1000);
    camera.up.set(0, 0, 1);
    camera.position.set(6, -5, 4.5);
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.12;
    scene.add(new THREE.HemisphereLight(0xffffff, 0x7a8294, 0.95));
    const dl = new THREE.DirectionalLight(0xffffff, 0.55); dl.position.set(4, -6, 10); scene.add(dl);
    root = new THREE.Group(); scene.add(root);
    const ro = new ResizeObserver(resize); ro.observe(el); resize();
    requestAnimationFrame(loop);
    initDrag();
  }
  function resize() {
    if (!host) return;
    const w = host.clientWidth, h = host.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h); labelR.setSize(w, h);
    camera.aspect = w / h; camera.updateProjectionMatrix();
  }

  function setGrid(size, step) {
    if (gridObj) { root.remove(gridObj); gridObj.geometry.dispose(); }
    const div = Math.round(size / step) * 2;
    gridObj = new THREE.GridHelper(size * 2, div, color('grid2'), color('grid'));
    gridObj.rotation.x = Math.PI / 2;
    gridObj.material.transparent = true; gridObj.material.opacity = 0.9; gridObj.material.depthWrite = false;
    gridObj.renderOrder = -1;
    root.add(gridObj);
  }

  /* --- primitives --- */
  const Y = new THREE.Vector3(0, 1, 0);
  const shaftG = new THREE.CylinderGeometry(1, 1, 1, 14); shaftG.translate(0, 0.5, 0);
  const headG = new THREE.ConeGeometry(1, 1, 18); headG.translate(0, -0.5, 0);
  const sphG = new THREE.SphereGeometry(1, 20, 14);
  const boxG = new THREE.BoxGeometry(1, 1, 1);
  const cylG = new THREE.CylinderGeometry(1, 1, 1, 28); cylG.rotateX(Math.PI / 2);

  function mat(c) { return new THREE.MeshLambertMaterial({ color: color(c), transparent: true }); }
  function setOp(m, op) { m.opacity = op; m.depthWrite = op > 0.98; }

  function makeLabel(cls) {
    const d = document.createElement('div'); d.className = 'lab ' + (cls || '');
    const o = new THREE.CSS2DObject(d); o.userData.html = null; return o;
  }
  function setLabel(o, html, op, colRole) {
    if (o.userData.html !== html) { o.element.innerHTML = html || ''; o.userData.html = html; }
    o.visible = !!html && op > 0.04;
    o.element.style.opacity = op;
    if (colRole !== undefined && o.userData.col !== colRole) { o.element.style.color = colRole ? (PAL[colRole] || colRole) : ''; o.userData.col = colRole; }
  }

  function Arrow(c) {
    const g = new THREE.Group();
    const m = mat(c);
    const shaft = new THREE.Mesh(shaftG, m), head = new THREE.Mesh(headG, m);
    g.add(shaft, head);
    const q = new THREE.Quaternion(), v = new THREE.Vector3();
    return {
      g, m,
      set(a, b, r, op) {
        v.set(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
        const L = v.length();
        if (L < 1e-6 || op < 0.01) { g.visible = false; return; }
        g.visible = true;
        const hl = Math.min(L * 0.45, r * 5.2), hr = r * 2.5;
        shaft.scale.set(r, Math.max(L - hl, 1e-4), r);
        head.scale.set(hr, hl, hr); head.position.set(0, L, 0);
        g.position.set(a[0], a[1], a[2]);
        q.setFromUnitVectors(Y, v.normalize()); g.quaternion.copy(q);
        setOp(m, op);
      },
      color(c2) { m.color.copy(color(c2)); },
      dispose() { m.dispose(); }
    };
  }

  /* --- frame visual --- */
  function FrameViz() {
    const g = new THREE.Group(); g.matrixAutoUpdate = false;
    const ax = [Arrow('x'), Arrow('y'), Arrow('z')];
    ax.forEach(a => g.add(a.g));
    const om = mat('ink'); const o = new THREE.Mesh(sphG, om); g.add(o);
    const name = makeLabel('fname'); g.add(name);
    const al = [makeLabel('ax'), makeLabel('ax'), makeLabel('ax')]; al.forEach(l => g.add(l));
    root.add(g);
    const m4 = new THREE.Matrix4();
    let colKey = null;
    return {
      update(s, T, op) {
        const ck = s.cols ? s.cols.join(',') : '';
        if (ck !== colKey) { colKey = ck; for (let i = 0; i < 3; i++) ax[i].color(s.cols ? s.cols[i] : ['x', 'y', 'z'][i]); }
        m4.set(...T[0], ...T[1], ...T[2], ...T[3]);
        g.matrix.copy(m4); g.matrixWorldNeedsUpdate = true;
        const vis = !s.hidden && op > 0.01;
        g.visible = vis;
        const L = (s.size || 1) * fsize, r = 0.022 * unitScale * (s.thin ? 0.7 : 1);
        const roles = ['x', 'y', 'z'];
        const names = s.axes || ['X', 'Y', 'Z'];
        for (let i = 0; i < 3; i++) {
          const e = [0, 0, 0]; e[i] = L;
          let aop = op;
          if (s.hl && s.hl.indexOf(roles[i]) < 0) aop *= 0.22;
          ax[i].set([0, 0, 0], e, r, vis ? aop : 0);
          al[i].position.set(e[0] * 1.13, e[1] * 1.13, e[2] * 1.13);
          const sub = s.sub === false ? '' : (s.sub !== undefined ? s.sub : (s.label || ''));
          const txt = s.noAxisLabels ? '' : `${names[i]}${sub !== '' ? '<sub>' + sub + '</sub>' : ''}`;
          setLabel(al[i], vis ? txt : '', aop * (s.ghost ? 0.8 : 1), s.cols ? s.cols[i] : roles[i]);
        }
        o.scale.setScalar(r * 2.2); setOp(om, vis ? op : 0); o.visible = vis;
        name.position.set(-0.12 * L, -0.12 * L, -0.16 * L);
        setLabel(name, vis && s.label !== '' ? (s.name || `{${s.label}}`) : '', op);
      },
      recolor() { colKey = null; ax[0].color('x'); ax[1].color('y'); ax[2].color('z'); om.color.copy(color('ink')); },
      dispose() { root.remove(g); ax.forEach(a => a.dispose()); om.dispose(); [name, ...al].forEach(l => l.element.remove()); }
    };
  }

  function PtViz() {
    const m = mat('pt'); const s = new THREE.Mesh(sphG, m); root.add(s);
    const l = makeLabel('pt'); root.add(l);
    let c0 = 'pt';
    return {
      update(sp, p, op) {
        if (sp.c !== c0) { m.color.copy(color(sp.c || 'pt')); c0 = sp.c; }
        s.visible = op > 0.01; s.position.set(...p); s.scale.setScalar(0.07 * unitScale * (sp.r || 1)); setOp(m, op);
        l.position.set(p[0], p[1], p[2] + 0.14 * unitScale);
        setLabel(l, sp.label, op, sp.c || 'pt');
      },
      recolor() { m.color.copy(color(c0 || 'pt')); },
      dispose() { root.remove(s); root.remove(l); m.dispose(); l.element.remove(); }
    };
  }

  function VecViz() {
    const a = Arrow('pt'); root.add(a.g);
    const l = makeLabel('vec'); root.add(l);
    let c0 = null;
    return {
      update(sp, A, B, op) {
        if (sp.c !== c0) { a.color(sp.c || 'pt'); c0 = sp.c; }
        a.set(A, B, 0.02 * unitScale * (sp.w || 1), op);
        const t = sp.lt != null ? sp.lt : 0.55;
        const off = sp.lo || [0, 0, 0.12];
        l.position.set(A[0] + (B[0] - A[0]) * t + off[0] * unitScale, A[1] + (B[1] - A[1]) * t + off[1] * unitScale, A[2] + (B[2] - A[2]) * t + off[2] * unitScale);
        setLabel(l, sp.label, op, sp.c || 'pt');
      },
      recolor() { a.color(c0 || 'pt'); },
      dispose() { root.remove(a.g); a.dispose(); root.remove(l); l.element.remove(); }
    };
  }

  function LineViz() {
    const geo = new THREE.BufferGeometry(); geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
    const mD = new THREE.LineDashedMaterial({ color: color('mute'), dashSize: 0.08, gapSize: 0.06, transparent: true });
    const mS = new THREE.LineBasicMaterial({ color: color('mute'), transparent: true });
    const line = new THREE.Line(geo, mD); root.add(line);
    const l = makeLabel('ln'); root.add(l);
    let c0 = null;
    return {
      update(sp, A, B, op) {
        const m = sp.dash === false ? mS : mD;
        if (line.material !== m) line.material = m;
        if (sp.c !== c0) { mD.color.copy(color(sp.c || 'mute')); mS.color.copy(color(sp.c || 'mute')); c0 = sp.c; }
        mD.dashSize = 0.09 * unitScale; mD.gapSize = 0.065 * unitScale;
        const p = geo.attributes.position.array; p.set([...A, ...B]); geo.attributes.position.needsUpdate = true;
        geo.computeBoundingSphere(); line.computeLineDistances();
        m.opacity = op; line.visible = op > 0.01;
        const t = sp.lt != null ? sp.lt : 0.5, off = sp.lo || [0, 0, 0.1];
        l.position.set(A[0] + (B[0] - A[0]) * t + off[0] * unitScale, A[1] + (B[1] - A[1]) * t + off[1] * unitScale, A[2] + (B[2] - A[2]) * t + off[2] * unitScale);
        setLabel(l, sp.label, op, sp.c || 'mute');
      },
      recolor() { mD.color.copy(color(c0 || 'mute')); mS.color.copy(color(c0 || 'mute')); },
      dispose() { root.remove(line); geo.dispose(); mD.dispose(); mS.dispose(); root.remove(l); l.element.remove(); }
    };
  }

  function ArcViz() {
    const N = 40;
    const m = mat('acc');
    let tube = null;
    const hm = mat('acc'); const head = new THREE.Mesh(headG, hm); root.add(head);
    const l = makeLabel('arc'); root.add(l);
    let c0 = null; const q = new THREE.Quaternion(), v = new THREE.Vector3();
    return {
      update(sp, C, n, u, op, angOverride) {
        if (sp.col !== c0) { m.color.copy(color(sp.col || 'acc')); hm.color.copy(color(sp.col || 'acc')); c0 = sp.col; }
        const ang = (angOverride != null ? angOverride : sp.ang) * D, r = sp.r || 1;
        const w = unit(cross(n, u));
        const pt = t => add(C, add(scl(u, r * Math.cos(t)), scl(w, r * Math.sin(t))));
        const vis = op > 0.01 && Math.abs(ang) > 1e-3;
        if (tube) { root.remove(tube); tube.geometry.dispose(); tube = null; }
        if (vis) {
          const pts = []; for (let i = 0; i < N; i++) { const p = pt(ang * i / (N - 1)); pts.push(new THREE.Vector3(p[0], p[1], p[2])); }
          const g = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), N, 0.011 * unitScale, 6, false);
          tube = new THREE.Mesh(g, m); root.add(tube);
        }
        setOp(m, op);
        const sg = Math.sign(ang || 1);
        const end = pt(ang), tan = add(scl(u, -Math.sin(ang) * sg), scl(w, Math.cos(ang) * sg));
        const hl = 0.13 * unitScale, hr = 0.045 * unitScale;
        head.scale.set(hr, hl, hr); head.position.set(...add(end, scl(unit(tan), hl * 0.5)));
        v.set(...unit(tan)); q.setFromUnitVectors(Y, v); head.quaternion.copy(q);
        head.visible = vis && !sp.noHead; setOp(hm, op);
        const mid = pt(ang / 2), lp = add(C, scl(sub(mid, C), 1 + 0.28 * unitScale / Math.max(r, 0.3 * unitScale)));
        l.position.set(...lp);
        setLabel(l, sp.label, op, sp.col || 'acc');
      },
      recolor() { m.color.copy(color(c0 || 'acc')); hm.color.copy(color(c0 || 'acc')); },
      dispose() { if (tube) { root.remove(tube); tube.geometry.dispose(); } m.dispose(); root.remove(head); hm.dispose(); root.remove(l); l.element.remove(); }
    };
  }

  function BodyViz() {
    const m = mat('body');
    const mesh = new THREE.Mesh(boxG, m); mesh.matrixAutoUpdate = false; root.add(mesh);
    let shape = 'box', c0 = 'body', glow = false, hov = false;
    const paint = () => { if (hov) m.emissive.setHex(0x3a3f66); else if (glow) m.emissive.copy(color('acc')).multiplyScalar(0.55); else m.emissive.setHex(0x000000); };
    const m4 = new THREE.Matrix4(), s4 = new THREE.Matrix4();
    return {
      update(sp, T, size, pos, op) {
        if (sp.shape !== shape) { mesh.geometry = sp.shape === 'cyl' ? cylG : sp.shape === 'sph' ? sphG : boxG; shape = sp.shape; }
        if ((sp.c || 'body') !== c0) { c0 = sp.c || 'body'; m.color.copy(color(c0)); }
        const W = mul(T, tr(pos));
        m4.set(...W[0], ...W[1], ...W[2], ...W[3]);
        const sz = sp.shape === 'cyl' ? [size[0], size[0], size[1]] : sp.shape === 'sph' ? [size[0], size[0], size[0]] : size;
        s4.makeScale(Math.max(sz[0], 1e-4), Math.max(sz[1], 1e-4), Math.max(sz[2], 1e-4));
        mesh.matrix.copy(m4).multiply(s4); mesh.matrixWorldNeedsUpdate = true;
        mesh.visible = op > 0.01; setOp(m, op);
        mesh.userData.drag = sp.drag || null;
        glow = !!sp.glow; paint();
      },
      mesh,
      hover(on) { hov = on; paint(); },
      recolor() { m.color.copy(color(c0)); },
      dispose() { root.remove(mesh); m.dispose(); }
    };
  }

  const MAKERS = { frames: FrameViz, pts: PtViz, vecs: VecViz, lines: LineViz, arcs: ArcViz, bodies: BodyViz };
  function getViz(kind, id) { return pools[kind][id] || (pools[kind][id] = MAKERS[kind]()); }
  function clearPools() { for (const k in pools) { for (const id in pools[k]) pools[k][id].dispose(); pools[k] = {}; } }

  /* ---------------- interpolation ---------------- */
  const tA = new THREE.Matrix4(), tB = new THREE.Matrix4();
  const pa = new THREE.Vector3(), pb = new THREE.Vector3(), qa = new THREE.Quaternion(), qb = new THREE.Quaternion(), sa = new THREE.Vector3(), sb = new THREE.Vector3(), tmpM = new THREE.Matrix4();
  function interpT(A, B, u) {
    tA.set(...A[0], ...A[1], ...A[2], ...A[3]); tB.set(...B[0], ...B[1], ...B[2], ...B[3]);
    tA.decompose(pa, qa, sa); tB.decompose(pb, qb, sb);
    pa.lerp(pb, u); qa.slerp(qb, u);
    tmpM.compose(pa, qa, new THREE.Vector3(1, 1, 1));
    const e = tmpM.elements; // column-major
    return [[e[0], e[4], e[8], e[12]], [e[1], e[5], e[9], e[13]], [e[2], e[6], e[10], e[14]], [0, 0, 0, 1]];
  }
  const lerp = (a, b, u) => a + (b - a) * u;
  const lerpV = (a, b, u) => a.map((x, i) => lerp(x, b[i], u));
  const ease = u => u < 0.5 ? 4 * u * u * u : 1 - Math.pow(-2 * u + 2, 3) / 2;
  const same = (A, B) => { for (let i = 0; i < 3; i++) for (let j = 0; j < 4; j++) if (Math.abs(A[i][j] - B[i][j]) > 1e-9) return false; return true; };

  function prepMotion(motion) {
    return (motion || []).map(m => {
      let axis = [0, 0, 1], deg = 0, t = m.tr || [0, 0, 0];
      if (m.rot) { axis = m.rot[0]; deg = m.rot[1]; }
      if (m.rotM) { const aa = toAA(m.rotM); axis = aa.axis; deg = aa.deg; }
      return { fs: [].concat(m.f), axis, deg, t, about: m.about || 'world', w: m.w || 1, with: !!m.with };
    });
  }
  function slots(items) { // assign time windows; 'with' shares the previous item's window
    const sl = [];
    items.forEach(it => { if (it.with && sl.length) sl[sl.length - 1].push(it); else sl.push([it]); });
    const ws = sl.map(g => Math.max(...g.map(i => i.w))), tot = ws.reduce((a, b) => a + b, 0) || 1;
    let acc = 0;
    sl.forEach((g, i) => { const s0 = acc / tot; acc += ws[i]; const s1 = acc / tot; g.forEach(it => { it.s0 = s0; it.s1 = s1; }); });
    return tot;
  }
  const G = (it, s) => mul(tr(scl(it.t, s)), rotK(it.axis, it.deg * s));
  function applyG(T, Gm, about) {
    if (about === 'local') return mul(T, Gm);
    if (about === 'worldDir') { const p = p3(T); return mul(tr(p), Gm, tr(scl(p, -1)), T); }
    return mul(Gm, T);
  }

  /* ---------------- resolve refs ---------------- */
  function resolve(ref, F) {
    if (!ref) return [0, 0, 0];
    if (Array.isArray(ref)) return ref;
    const T = F[ref.f] || I();
    return app(T, ref.p || [0, 0, 0]);
  }
  function resolveDir(ref, F, fId) { // dir in frame fId (or world)
    if (!fId) return ref;
    return dir(F[fId] || I(), ref);
  }

  /* ---------------- player core ---------------- */
  let anim = null, shown = null, onTick = null, lastF = {};
  let fsize = 1;
  function emptyState() { return { frames: {}, pts: {}, vecs: {}, lines: {}, arcs: {}, bodies: {}, watch: null }; }

  function computeF(before, after, items, u) {
    const F = {};
    const ids = new Set([...Object.keys(before.frames), ...Object.keys(after.frames)]);
    ids.forEach(id => {
      const b = before.frames[id], a = after.frames[id];
      if (u >= 1) { F[id] = (a || b).T; return; }
      const moving = items.some(it => it.fs.indexOf(id) >= 0);
      if (moving) {
        let T = (b || a).T;
        for (const it of items) {
          if (it.fs.indexOf(id) < 0) continue;
          const s = ease(Math.max(0, Math.min(1, (u - it.s0) / (it.s1 - it.s0))));
          if (s > 0) T = applyG(T, G(it, s), it.about);
        }
        F[id] = T;
      } else if (a && b) F[id] = same(a.T, b.T) ? a.T : interpT(b.T, a.T, ease(u));
      else F[id] = (a || b).T;
    });
    return F;
  }

  function opOf(b, a, u) {
    const e = ease(Math.min(1, u * 1.6));
    const ob = b ? (b.op != null ? b.op : (b.ghost ? 0.35 : 1)) : 0;
    const oa = a ? (a.op != null ? a.op : (a.ghost ? 0.35 : 1)) : 0;
    if (a && b) return lerp(ob, oa, ease(u));
    if (a) return oa * e;
    return ob * (1 - e);
  }

  function draw(before, after, F, u) {
    const used = { frames: new Set(), pts: new Set(), vecs: new Set(), lines: new Set(), arcs: new Set(), bodies: new Set() };
    const eu = ease(u);
    // frames
    for (const id of new Set([...Object.keys(before.frames), ...Object.keys(after.frames)])) {
      const b = before.frames[id], a = after.frames[id];
      const op = opOf(b, a, u);
      const sp = a || b;
      used.frames.add(id); getViz('frames', id).update(sp, F[id], op);
    }
    const ref2 = (rb, ra) => (Array.isArray(rb) && Array.isArray(ra)) ? lerpV(rb, ra, eu) : resolve(ra || rb, F);
    for (const id of new Set([...Object.keys(before.pts), ...Object.keys(after.pts)])) {
      const b = before.pts[id], a = after.pts[id], sp = a || b;
      const p = (a && b) ? ref2(b.at, a.at) : resolve(sp.at, F);
      used.pts.add(id); getViz('pts', id).update(sp, p, opOf(b, a, u));
    }
    for (const kind of ['vecs', 'lines']) {
      for (const id of new Set([...Object.keys(before[kind]), ...Object.keys(after[kind])])) {
        const b = before[kind][id], a = after[kind][id], sp = a || b;
        const A = (a && b) ? ref2(b.a, a.a) : resolve(sp.a, F);
        const B = (a && b) ? ref2(b.b, a.b) : resolve(sp.b, F);
        used[kind].add(id); getViz(kind, id).update(sp, A, B, opOf(b, a, u));
      }
    }
    for (const id of new Set([...Object.keys(before.arcs), ...Object.keys(after.arcs)])) {
      const b = before.arcs[id], a = after.arcs[id], sp = a || b;
      const C = resolve(sp.f ? { f: sp.f, p: sp.c || [0, 0, 0] } : (sp.c || [0, 0, 0]), F);
      const n = unit(resolveDir(sp.n, F, sp.f)), u0 = unit(resolveDir(sp.u, F, sp.f));
      let ang = null;
      if (a && b && b.ang !== a.ang && !sp.grow) ang = lerp(b.ang, a.ang, eu);
      if (a && !b && sp.grow) ang = a.ang * eu;
      used.arcs.add(id); getViz('arcs', id).update(sp, C, n, u0, opOf(b, a, u), ang);
    }
    for (const id of new Set([...Object.keys(before.bodies), ...Object.keys(after.bodies)])) {
      const b = before.bodies[id], a = after.bodies[id], sp = a || b;
      const T = F[sp.f] || I();
      const size = (a && b) ? lerpV(b.size, a.size, eu) : sp.size;
      const pos = (a && b) ? lerpV(b.pos || [0, 0, 0], a.pos || [0, 0, 0], eu) : (sp.pos || [0, 0, 0]);
      used.bodies.add(id); getViz('bodies', id).update(sp, T, size, pos, opOf(b, a, u));
    }
    // dispose unused
    for (const k in pools) for (const id in pools[k]) if (!used[k].has(id)) { pools[k][id].dispose(); delete pools[k][id]; }
    lastF = F;
    if (onTick) onTick(F, after, u);
  }

  function play(before, after, motion, dur) {
    const items = prepMotion(motion);
    const tot = slots(items);
    const reduce = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    let d = dur != null ? dur : Math.max(900, (items.length ? tot : 0) * 1150);
    if (reduce) d = 1;
    anim = { before, after, items, t0: performance.now(), dur: d };
    shown = after;
  }
  function snap(state) { anim = null; shown = state; draw(state, state, computeF(state, state, [], 1), 1); }

  /* camera animation */
  let camAnim = null;
  function setView(v, instant) {
    const pos = new THREE.Vector3(...(v.pos || [6, -5, 4.5])), tgt = new THREE.Vector3(...(v.target || [0, 0, 0]));
    if (instant) { camera.position.copy(pos); controls.target.copy(tgt); controls.update(); camAnim = null; return; }
    camAnim = { p0: camera.position.clone(), t0v: controls.target.clone(), p1: pos, t1: tgt, start: performance.now(), dur: 800 };
  }

  function loop(now) {
    requestAnimationFrame(loop);
    if (camAnim) {
      const u = Math.min(1, (now - camAnim.start) / camAnim.dur), e = ease(u);
      camera.position.lerpVectors(camAnim.p0, camAnim.p1, e);
      controls.target.lerpVectors(camAnim.t0v, camAnim.t1, e);
      if (u >= 1) camAnim = null;
    }
    if (anim) {
      const u = Math.min(1, (now - anim.t0) / anim.dur);
      draw(anim.before, anim.after, computeF(anim.before, anim.after, anim.items, u), u);
      if (u >= 1) anim = null;
    }
    controls.update();
    renderer.render(scene, camera);
    labelR.render(scene, camera);
  }


  /* ---------------- direct manipulation (drag joints) ---------------- */
  let dragH = null, dragging = null, hoverId = null;
  const ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
  function pickAt(ev) {
    const r = renderer.domElement.getBoundingClientRect();
    ndc.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
    ray.setFromCamera(ndc, camera);
    const o = ray.ray.origin, d = ray.ray.direction;
    const R = { o: [o.x, o.y, o.z], d: [d.x, d.y, d.z] };
    const meshes = [];
    for (const id in pools.bodies) { const b = pools.bodies[id]; if (b.mesh.visible && b.mesh.userData.drag) meshes.push(b.mesh); }
    const hit = meshes.length ? ray.intersectObjects(meshes, false)[0] : null;
    let key = null;
    if (hit) for (const id in pools.bodies) if (pools.bodies[id].mesh === hit.object) key = id;
    return { R, hit: hit ? { drag: hit.object.userData.drag, body: key, p: [hit.point.x, hit.point.y, hit.point.z] } : null };
  }
  function setHover(id) {
    if (hoverId === id) return;
    if (hoverId && pools.bodies[hoverId]) pools.bodies[hoverId].hover(false);
    hoverId = id;
    if (id && pools.bodies[id]) pools.bodies[id].hover(true);
  }
  function initDrag() {
    const el = renderer.domElement;
    el.addEventListener('pointerdown', ev => {
      if (!dragH || ev.button !== 0) return;
      const { R, hit } = pickAt(ev);
      if (!hit) return;
      ev.stopImmediatePropagation(); ev.preventDefault();
      dragging = { id: hit.drag, body: hit.body, pid: ev.pointerId };
      controls.enabled = false;
      try { el.setPointerCapture(ev.pointerId); } catch (e) { /* ignore */ }
      el.style.cursor = 'grabbing';
      dragH.start(hit.drag, R, hit.p);
    }, true);
    el.addEventListener('pointermove', ev => {
      if (dragging) { const { R } = pickAt(ev); dragH.move(dragging.id, R); return; }
      if (!dragH) { setHover(null); return; }
      const { hit } = pickAt(ev);
      setHover(hit ? hit.body : null);
      el.style.cursor = hit ? 'grab' : '';
    });
    const end = ev => {
      if (!dragging) return;
      try { el.releasePointerCapture(dragging.pid); } catch (e) { /* ignore */ }
      dragging = null; controls.enabled = true; el.style.cursor = '';
      dragH && dragH.end && dragH.end();
    };
    el.addEventListener('pointerup', end); el.addEventListener('pointercancel', end);
  }

  function retheme() {
    readPal();
    for (const k in pools) for (const id in pools[k]) pools[k][id].recolor && pools[k][id].recolor();
    if (gridObj && ENG._grid) setGrid(ENG._grid[0], ENG._grid[1]);
    if (shown && !anim) snap(shown);
  }

  const ENG = {
    initScene, setGrid(size, step) { ENG._grid = [size, step]; setGrid(size, step); },
    setScale(s, fs) { unitScale = s; fsize = fs || 1; },
    emptyState, play, snap, setView, clearPools, retheme,
    set onTick(fn) { onTick = fn; },
    set dragHandler(h) { dragH = h; if (!h) setHover(null); },
    get F() { return lastF; },
    toScreen(p) { const v = new THREE.Vector3(p[0], p[1], p[2]).project(camera); const r = renderer.domElement.getBoundingClientRect(); return [r.left + (v.x + 1) / 2 * r.width, r.top + (1 - v.y) / 2 * r.height]; },
    get animating() { return !!anim; }
  };
  window.ENG = ENG;
})();
