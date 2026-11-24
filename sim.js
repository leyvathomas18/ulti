/* TANG BOT · Simulador de circuitos
   Motor: análisis nodal modificado (MNA) + Newton-Raphson (diodos, Zener, BJT) + integración trapezoidal (C, L). */
(function (root) {
'use strict';
const T = {
  R: { n: 'Resistencia', p: 'R', v: 1000, lab: 'Resistencia (Ω)' },
  V: { n: 'Batería', p: 'V', v: 5, lab: 'Voltaje (V)' },
  AC: { n: 'Fuente AC', p: 'AC', v: 5, f: 60, lab: 'Amplitud pico (V)' },
  SW: { n: 'Interruptor', p: 'S' },
  SW2: { n: 'Palanca (izq/der)', p: 'P' },
  C: { n: 'Capacitor', p: 'C', v: 1e-6, lab: 'Capacitancia (F)' },
  L: { n: 'Bobina', p: 'L', v: 0.01, lab: 'Inductancia (H)' },
  D: { n: 'Diodo', p: 'D' },
  Z: { n: 'Diodo Zener', p: 'DZ', v: 5.1, lab: 'Voltaje Zener (V)' },
  Q: { n: 'Transistor NPN', p: 'Q', v: 100, lab: 'Ganancia β' },
  G: { n: 'Tierra (GND)', p: 'GND' }
};
const ICON = { R: '▭', V: '🔋', AC: '∿', SW: '⏻', SW2: '⇄', C: '⊣⊢', L: '⌇', D: '▷|', Z: '▷Z', Q: 'Q', G: '⏚' };
const LP = { R: [[0,0],[4,0]], V: [[0,0],[4,0]], AC: [[0,0],[4,0]], SW: [[0,0],[4,0]], SW2: [[2,4],[0,0],[4,0]], C: [[0,0],[4,0]],
  L: [[0,0],[4,0]], D: [[0,0],[4,0]], Z: [[0,0],[4,0]], Q: [[0,2],[2,0],[2,4]], G: [[0,0]] }; // Q: B, C, E
const rot = (c, p) => { let [x, y] = p; for (let i = 0; i < c.r; i++) [x, y] = [-y, x]; return [c.x + x, c.y + y]; };
const two = c => LP[c.t].length === 2;
const pins = c => {
  if (two(c)) { if (c.x2 === undefined) { const q = rot(c, [4, 0]); c.x2 = q[0]; c.y2 = q[1]; } return [[c.x, c.y], [c.x2, c.y2]]; }
  return LP[c.t].map(p => rot(c, p));
};

const VT = .02585, IS = 1e-14, GM = 1e-12;
const ex = v => Math.exp(Math.min(v, 40));
function dio(c, v) {
  const e = ex(v / VT); let i = IS * (e - 1), g = IS * e / VT;
  if (c.t === 'Z') { const z = ex(-(v + c.val) / (2 * VT)); i -= 5e-3 * z; g += 5e-3 * z / (2 * VT); }
  return [i, g + GM];
}
function solve(A, B) {
  const n = B.length;
  for (let i = 0; i < n; i++) {
    let p = i;
    for (let r = i + 1; r < n; r++) if (Math.abs(A[r][i]) > Math.abs(A[p][i])) p = r;
    [A[i], A[p]] = [A[p], A[i]]; [B[i], B[p]] = [B[p], B[i]];
    const d = A[i][i] || 1e-18;
    for (let r = i + 1; r < n; r++) {
      const f = A[r][i] / d;
      if (f) { for (let c = i; c < n; c++) A[r][c] -= f * A[i][c]; B[r] -= f * B[i]; }
    }
  }
  const X = new Float64Array(n);
  for (let i = n - 1; i >= 0; i--) {
    let s = B[i];
    for (let c = i + 1; c < n; c++) s -= A[i][c] * X[c];
    X[i] = s / (A[i][i] || 1e-18);
  }
  return X;
}

function make(comps, wires) {
  const par = [], M = new Map();
  const id = (x, y) => { const k = x + ',' + y; if (!M.has(k)) { M.set(k, par.length); par.push(par.length); } return M.get(k); };
  const f = a => { while (par[a] !== a) a = par[a] = par[par[a]]; return a; };
  const un = (a, b) => { par[f(a)] = f(b); };
  wires.forEach(w => un(id(w.a[0], w.a[1]), id(w.b[0], w.b[1])));
  let g0 = -1;
  comps.forEach(c => {
    c.pi = pins(c).map(p => id(p[0], p[1]));
    if (c.t === 'G') { if (g0 < 0) g0 = c.pi[0]; else un(c.pi[0], g0); }
  });
  // Cables libres: un extremo o terminal que cae sobre otro cable (en cualquier ángulo) queda conectado a él
  const onSeg = (p, w) => { const dx = w.b[0] - w.a[0], dy = w.b[1] - w.a[1], l = dx * dx + dy * dy; if (!l) return false;
    const u = Math.max(0, Math.min(1, ((p[0] - w.a[0]) * dx + (p[1] - w.a[1]) * dy) / l)); return Math.hypot(p[0] - w.a[0] - u * dx, p[1] - w.a[1] - u * dy) < .05; };
  const P = []; wires.forEach(w => P.push(w.a, w.b)); comps.forEach(c => pins(c).forEach(p => P.push(p)));
  wires.forEach(w => P.forEach(p => { if (onSeg(p, w)) un(id(p[0], p[1]), id(w.a[0], w.a[1])); }));
  if (g0 < 0) return { error: 'Falta un símbolo de tierra (GND).' };
  const nm = new Map([[f(g0), 0]]); let N = 1;
  comps.forEach(c => { c.n = c.pi.map(p => { const r = f(p); if (!nm.has(r)) nm.set(r, N++); return nm.get(r); }); });
  const src = comps.filter(c => c.t === 'V' || c.t === 'AC');
  src.forEach((c, k) => { c.k = N - 1 + k; });
  const S = N - 1 + src.length;
  const nl = comps.filter(c => c.t === 'D' || c.t === 'Z' || c.t === 'Q');
  comps.forEach(c => { c.vp = c.t === 'C' ? (c.v0 || 0) : 0; c.ip = c.t === 'L' ? (c.i0 || 0) : 0; c.vj = [0, 0]; });
  let x = new Float64Array(S), t = 0, bad = false;
  const V = n => (n ? x[n - 1] : 0);

  function step(dt) {
    t += dt;
    let xn = x;
    for (let it = 0; it < 100; it++) {
      const A = Array.from({ length: S }, () => new Float64Array(S)), B = new Float64Array(S);
      const G = (a, b, g) => { if (a) A[a-1][a-1] += g; if (b) A[b-1][b-1] += g; if (a && b) { A[a-1][b-1] -= g; A[b-1][a-1] -= g; } };
      const J = (a, b, i) => { if (a) B[a-1] -= i; if (b) B[b-1] += i; }; // fuente de corriente a→b
      for (let k = 1; k < N; k++) A[k-1][k-1] += GM;
      for (const c of comps) {
        const [a, b] = c.n;
        switch (c.t) {
          case 'R': G(a, b, 1 / c.val); break;
          case 'SW': G(a, b, c.on ? 1e3 : 1e-9); break;
          case 'SW2': G(c.n[0], c.n[1], c.pos ? 1e-9 : 1e3); G(c.n[0], c.n[2], c.pos ? 1e3 : 1e-9); break;
          case 'C': { const g = 2 * c.val / dt; G(a, b, g); J(a, b, -(g * c.vp + c.ip)); break; }
          case 'L': { const g = dt / (2 * c.val); G(a, b, g); J(a, b, c.ip + g * c.vp); break; }
          case 'V': case 'AC': {
            const E = c.t === 'V' ? c.val : c.val * Math.sin(2 * Math.PI * c.f * t), k = c.k;
            if (a) { A[a-1][k] += 1; A[k][a-1] += 1; }
            if (b) { A[b-1][k] -= 1; A[k][b-1] -= 1; }
            B[k] = E; break;
          }
          case 'D': case 'Z': { const v = c.vj[0], [i, g] = dio(c, v); G(a, b, g); J(a, b, i - g * v); break; }
          case 'Q': {
            const [vbe, vbc] = c.vj, bf = c.val, ef = ex(vbe / VT), er = ex(vbc / VT);
            const If = IS * (ef - 1), Ir = IS * (er - 1), gf = IS * ef / VT, gr = IS * er / VT;
            const Ib = If / bf + Ir, Ic = If - 2 * Ir;
            const rb = [gf / bf + gr, -gr, -gf / bf], rc = [gf - 2 * gr, 2 * gr, -gf];
            const re = rb.map((q, j) => -(q + rc[j])), v0 = [vbe, vbe - vbc, 0];
            [[c.n[0], rb, Ib], [c.n[1], rc, Ic], [c.n[2], re, -(Ib + Ic)]].forEach(([nk, row, I0]) => {
              if (!nk) return;
              let eq = I0;
              for (let j = 0; j < 3; j++) { eq -= row[j] * v0[j]; if (c.n[j]) A[nk-1][c.n[j]-1] += row[j]; }
              B[nk-1] -= eq;
            });
          }
        }
      }
      xn = solve(A, B);
      const Vn = n => (n ? xn[n - 1] : 0);
      let ok = true;
      for (const c of nl) {
        const nv = c.t === 'Q' ? [Vn(c.n[0]) - Vn(c.n[2]), Vn(c.n[0]) - Vn(c.n[1])] : [Vn(c.n[0]) - Vn(c.n[1]), 0];
        for (let j = 0; j < (c.t === 'Q' ? 2 : 1); j++) {
          const d = nv[j] - c.vj[j];
          if (Math.abs(d) > 1e-6) ok = false;
          c.vj[j] += Math.max(-.5, Math.min(.5, d));
        }
      }
      if (ok) break;
    }
    x = xn;
    for (const c of comps) {
      const v = V(c.n[0]) - V(c.n[1]);
      if (c.t === 'C') { const g = 2 * c.val / dt; c.ip = g * v - (g * c.vp + c.ip); c.vp = v; }
      else if (c.t === 'L') { const g = dt / (2 * c.val); c.ip = g * v + c.ip + g * c.vp; c.vp = v; }
    }
    if (x.some(Number.isNaN)) bad = true;
  }
  function read(c) {
    const v = V(c.n[0]) - V(c.n[1]);
    switch (c.t) {
      case 'R': return { v, i: v / c.val };
      case 'SW': return { v, i: v * (c.on ? 1e3 : 1e-9) };
      case 'SW2': { const w = V(c.n[0]) - V(c.n[c.pos ? 2 : 1]); return { v: w, i: w * 1e3 }; }
      case 'C': case 'L': return { v, i: c.ip };
      case 'V': case 'AC': return { v, i: -x[c.k] };
      case 'D': case 'Z': return { v, i: dio(c, v)[0] };
      case 'Q': {
        const vbe = V(c.n[0]) - V(c.n[2]), vbc = V(c.n[0]) - V(c.n[1]);
        return { v: V(c.n[1]) - V(c.n[2]), i: IS * (ex(vbe / VT) - 1) - 2 * IS * (ex(vbc / VT) - 1) };
      }
    }
    return { v: 0, i: 0 };
  }
  return { step, read, get t() { return t; }, get bad() { return bad; } };
}

function eng(v) {
  if (!isFinite(v)) return '—';
  const a = Math.abs(v); if (a < 1e-15) return '0';
  for (const [m, s] of [[1e9,'G'],[1e6,'M'],[1e3,'k'],[1,''],[1e-3,'m'],[1e-6,'µ'],[1e-9,'n'],[1e-12,'p']])
    if (a >= m * .9995) return +(v / m).toPrecision(3) + s;
  return v.toExponential(1);
}
function parse(s) {
  const m = String(s).trim().replace(',', '.').match(/^(-?\d*\.?\d+(?:e[-+]?\d+)?)\s*([pnuµmkMG]?)/);
  if (!m) return NaN;
  return +m[1] * ({ p: 1e-12, n: 1e-9, u: 1e-6, 'µ': 1e-6, m: 1e-3, k: 1e3, M: 1e6, G: 1e9 }[m[2]] || 1);
}
root.Sim = { T, pins, make, eng, parse };
if (typeof document !== 'undefined') ui();

/* ---------------------------- Interfaz ---------------------------- */
function ui() {
  const $ = id => document.getElementById(id), cv = $('simCv'), cx = cv.getContext('2d'), sc = $('scopeCv'), sx = sc.getContext('2d'), G = 20;
  let comps = [], wires = [], sel = null, tool = null, sim = null, run = false, dt = 1e-5, spf = 200, cnt = 0, ptr = [0, 0], wr = null, dr = null, uid = 0;
  let tdiv = 5e-3, lastS = -1, scopeShow = 'all', hov = null, ptrF = [0, 0], lastTap = { c: null, t: 0 }, wrFresh = false;
  let multi = [], rb = null, clip = null, pasteN = 0, undoS = [], redoS = [], lastSt = '';
  const isSel = o => (sel && sel.o === o) || multi.some(m => m.o === o);
  const selItems = () => multi.length ? multi : sel ? [sel] : []; const hist = new Map(), num = {};

  const ln = (c, pts) => { c.beginPath(); pts.forEach((p, i) => i ? c.lineTo(p[0], p[1]) : c.moveTo(p[0], p[1])); c.stroke(); };
  const SYM = {
    R: c => ln(c, [[0,0],[.7,0],[1,-.4],[1.5,.4],[2,-.4],[2.5,.4],[3,-.4],[3.3,0],[4,0]]),
    V: c => { ln(c, [[0,0],[1.7,0]]); ln(c, [[1.7,-.8],[1.7,.8]]); ln(c, [[2.3,-.4],[2.3,.4]]); ln(c, [[2.3,0],[4,0]]); c.font = '.7px sans-serif'; c.fillText('+', 1, -.3); },
    AC: c => { ln(c, [[0,0],[1.1,0]]); ln(c, [[2.9,0],[4,0]]); c.beginPath(); c.arc(2, 0, .9, 0, 7); c.stroke();
      const p = []; for (let i = 0; i <= 20; i++) p.push([1.4 + i * .03, -Math.sin(i / 20 * 2 * Math.PI) * .4]); ln(c, p); },
    SW2: (c, k) => { ln(c, [[2,4],[2,3]]); ln(c, [[0,0],[0,1]]); ln(c, [[4,0],[4,1]]);
      c.beginPath(); c.arc(0, 1, .16, 0, 7); c.stroke(); c.beginPath(); c.arc(4, 1, .16, 0, 7); c.stroke();
      ln(c, [[2,3],k.pos ? [3.85,1.1] : [.15,1.1]]); c.beginPath(); c.arc(2, 3, .2, 0, 7); c.fill();
      c.font = '.6px sans-serif'; c.fillText('A', -.15, -.3); c.fillText('B', 3.85, -.3); },
    SW: (c, k) => { ln(c, [[0,0],[1,0]]); ln(c, [[3,0],[4,0]]); ln(c, k.on ? [[1,0],[3,0]] : [[1,0],[2.8,-.9]]); },
    C: c => { ln(c, [[0,0],[1.8,0]]); ln(c, [[2.2,0],[4,0]]); ln(c, [[1.8,-.9],[1.8,.9]]); ln(c, [[2.2,-.9],[2.2,.9]]); },
    L: c => { c.beginPath(); c.moveTo(0, 0); c.lineTo(.8, 0); for (let k = 0; k < 4; k++) c.arc(1.1 + .6 * k, 0, .3, Math.PI, 0); c.lineTo(4, 0); c.stroke(); },
    D: c => { ln(c, [[0,0],[1.5,0]]); ln(c, [[2.5,0],[4,0]]); c.beginPath(); c.moveTo(1.5, -.7); c.lineTo(1.5, .7); c.lineTo(2.5, 0); c.closePath(); c.stroke(); ln(c, [[2.5,-.7],[2.5,.7]]); },
    Z: (c, k) => { SYM.D(c, k); ln(c, [[2.5,-.7],[2.2,-.9]]); ln(c, [[2.5,.7],[2.8,.9]]); },
    Q: c => { ln(c, [[0,2],[1.3,2]]); ln(c, [[1.3,1],[1.3,3]]); ln(c, [[1.3,1.4],[2,.7],[2,0]]); ln(c, [[1.3,2.6],[2,3.3],[2,4]]);
      c.beginPath(); c.moveTo(1.95, 3.25); c.lineTo(1.59, 3.11); c.lineTo(1.81, 2.89); c.closePath(); c.fill(); },
    G: c => { ln(c, [[0,0],[0,.6]]); ln(c, [[-.7,.6],[.7,.6]]); ln(c, [[-.45,.9],[.45,.9]]); ln(c, [[-.2,1.2],[.2,1.2]]); }
  };
  const valStr = c => ({ R: eng(c.val) + 'Ω', V: eng(c.val) + 'V', AC: eng(c.val) + 'V ' + eng(c.f) + 'Hz', C: eng(c.val) + 'F', L: eng(c.val) + 'H',
    Z: eng(c.val) + 'V', Q: 'β=' + c.val, SW: c.on ? 'cerrado' : 'abierto', SW2: c.pos ? 'derecha' : 'izquierda' }[c.t] || '');
  const pu = k => (k === 'v' ? 'V' : 'A');

  function fit(c) {
    const d = devicePixelRatio || 1, r = c.getBoundingClientRect(), w = Math.round(r.width * d), h = Math.round(r.height * d);
    if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
    return d;
  }
  function drawComp(c, al) {
    const on = isSel(c);
    cx.save(); cx.globalAlpha = al || 1; cx.strokeStyle = cx.fillStyle = on ? '#ffb300' : '#e6f1ff';
    cx.translate(c.x * G, c.y * G); cx.lineJoin = 'round';
    if (two(c)) {
      pins(c); const dx = c.x2 - c.x, dy = c.y2 - c.y, L = Math.hypot(dx, dy) || 1, k = Math.min(1, L / 4), e = (L - 4 * k) / 2;
      cx.rotate(Math.atan2(dy, dx)); cx.scale(G, G); cx.lineWidth = 2 / G;
      if (e > 0) { ln(cx, [[0, 0], [e, 0]]); ln(cx, [[e + 4 * k, 0], [L, 0]]); }
      cx.translate(e, 0); cx.scale(k, k); cx.lineWidth = 2 / (G * k);
    } else { cx.rotate(c.r * Math.PI / 2); cx.scale(G, G); cx.lineWidth = 2 / G; }
    SYM[c.t](cx, c); cx.restore();
    const ps = pins(c);
    cx.save(); cx.globalAlpha = al || 1; cx.fillStyle = '#00e5ff';
    ps.forEach(p => { cx.beginPath(); cx.arc(p[0] * G, p[1] * G, 3, 0, 7); cx.fill(); });
    if (c.t !== 'G') {
      const mx = ps.reduce((s, p) => s + p[0], 0) / ps.length * G, my = ps.reduce((s, p) => s + p[1], 0) / ps.length * G;
      const horiz = ps.length !== 3 && ps[0][1] === ps[1][1];
      const x = ps.length === 3 ? Math.max(...ps.map(p => p[0])) * G + 8 : horiz ? mx : mx + 20, y = ps.length === 3 ? my - 6 : horiz ? my - 26 : my - 6;
      cx.textAlign = horiz ? 'center' : 'left'; cx.font = '12px Inter,sans-serif'; cx.fillStyle = on ? '#ffb300' : '#c9d8f2';
      cx.fillText(c.name + ' ' + valStr(c), x, y);
      if (sim && !sim.error && sim.t > 0) { const m = sim.read(c); cx.font = '11px "JetBrains Mono",monospace'; cx.fillStyle = '#00e5ff'; cx.fillText(eng(m.v) + 'V  ' + eng(m.i) + 'A', x, y + 14); }
    }
    cx.restore();
  }
  const wpath = w => [w.a, w.b];
  function draw() {
    const d = fit(cv); cx.setTransform(d, 0, 0, d, 0, 0);
    const W = cv.width / d, H = cv.height / d; cx.clearRect(0, 0, W, H);
    cx.fillStyle = 'rgba(0,229,255,.25)';
    for (let x = 0; x < W; x += G) for (let y = 0; y < H; y += G) cx.fillRect(x - .5, y - .5, 1.5, 1.5);
    const dw = (w, col) => { cx.strokeStyle = col; cx.lineWidth = 2; ln(cx, wpath(w).map(p => [p[0] * G, p[1] * G])); };
    wires.forEach(w => dw(w, isSel(w) ? '#ffb300' : '#4dd0e1'));
    comps.forEach(c => drawComp(c));
    cx.fillStyle = '#4dd0e1'; wires.forEach(w => [w.a, w.b].forEach(p => { cx.beginPath(); cx.arc(p[0] * G, p[1] * G, isSel(w) ? 5 : 3, 0, 7); cx.fill(); }));
    if (wr) { cx.setLineDash([5, 4]); dw(wr, '#ff4081'); cx.setLineDash([]); }
    if (tool === 'W' || wr) { const q = nearPort(ptrF, .6); if (q) { cx.strokeStyle = '#ff4081'; cx.lineWidth = 2; cx.beginPath(); cx.arc(q[0] * G, q[1] * G, 8, 0, 7); cx.stroke(); } }
    if (!tool && !dr && !rb && !wr) { const h = pinHit(ptrF) || wireEnd(ptrF); if (h) { cx.strokeStyle = '#ffb300'; cx.lineWidth = 2; cx.beginPath(); cx.arc(h.pt[0] * G, h.pt[1] * G, 8, 0, 7); cx.stroke(); } }
    if (rb) { const x = Math.min(rb.a[0], rb.b[0]) * G, y = Math.min(rb.a[1], rb.b[1]) * G, w = Math.abs(rb.a[0] - rb.b[0]) * G, h = Math.abs(rb.a[1] - rb.b[1]) * G;
      cx.fillStyle = 'rgba(0,229,255,.08)'; cx.fillRect(x, y, w, h); cx.setLineDash([5, 4]); cx.strokeStyle = '#00e5ff'; cx.lineWidth = 1; cx.strokeRect(x, y, w, h); cx.setLineDash([]); }
    if (tool && tool !== 'W') drawComp({ t: tool, x: ptr[0], y: ptr[1], x2: ptr[0] + 4, y2: ptr[1], r: 0, val: T[tool].v, f: T[tool].f, on: false, name: T[tool].p }, .45);
  }
  const hint = c => ({ R: 'Ley de Ohm: V = I·R. La energía se disipa en forma de calor.', V: 'Fuente de voltaje constante. Entrega la potencia P = V·I.', AC: 'Voltaje que sube y baja en forma de onda senoidal.',
    SW: 'Cerrado deja pasar la corriente; abierto la corta.', SW2: 'La palanca conecta el común con A (izquierda) o B (derecha). Haz clic para moverla.',
    C: 'Se carga o se descarga: la corriente es grande cuando el voltaje cambia rápido (i = C·dv/dt).', L: 'Se opone a los cambios de corriente (v = L·di/dt) y guarda energía en su campo magnético.',
    D: 'Solo deja pasar corriente de ánodo a cátodo.', Z: 'Como un diodo, pero en inversa mantiene casi constante su voltaje Zener.', Q: 'Una corriente pequeña de base controla una corriente mayor de colector.' }[c.t] || '');
  function wrapTxt(t, w, maxL) { const ls = []; let cur = ''; t.split(' ').forEach(x => { const q = cur ? cur + ' ' + x : x; if (sx.measureText(q).width > w && cur) { ls.push(cur); cur = x; } else cur = q; }); ls.push(cur); return ls.slice(0, maxL); }
  function drawScope() {
    const W = sc.clientWidth || 600;
    let list = scopeShow === 'sel' && sel && sel.k === 'c' ? [sel.o] : comps.filter(c => c.t !== 'G'); list = list.slice(0, 6);
    const rc = hov || (sel && sel.k === 'c' ? sel.o : null) || list[0] || null, cells = list.length + (rc ? 1 : 0);
    const cols = Math.max(1, Math.min(cells || 1, Math.floor(W / 170))), rows = Math.ceil((cells || 1) / cols), CH = 150, TOP = 24, Ht = TOP + rows * CH + 4, cw = W / cols;
    if (sc._h !== Ht) { sc._h = Ht; sc.style.height = Ht + 'px'; }
    const d = fit(sc); sx.setTransform(d, 0, 0, d, 0, 0); sx.fillStyle = '#030712'; sx.fillRect(0, 0, W, Ht);
    const tn = sim && !sim.error ? sim.t : 0, win = tdiv * 10, t0 = Math.max(0, tn - win);
    sx.textAlign = 'left'; sx.font = '600 11px Inter,sans-serif'; sx.fillStyle = '#00e5ff'; sx.fillText('━ V = voltaje', 8, 16); sx.fillStyle = '#ffd54f'; sx.fillText('━ I = corriente', 98, 16);
    sx.fillStyle = '#8fa3c7'; sx.textAlign = 'right'; sx.font = '11px Inter,sans-serif'; sx.fillText(`cada cuadro = ${eng(tdiv)}s · t = ${eng(tn)}s`, W - 8, 16); sx.textAlign = 'left';
    const box0 = n => [(n % cols) * cw, TOP + Math.floor(n / cols) * CH];
    list.forEach((c, n) => {
      const [x0, y0] = box0(n), h = hist.get(c.id) || [], px = x0 + 7, py = y0 + 46, pw = cw - 14, ph = CH - 54;
      let mv = 1e-12, mi = 1e-12; h.forEach(q => { mv = Math.max(mv, Math.abs(q[1])); mi = Math.max(mi, Math.abs(q[2])); });
      sx.save(); sx.beginPath(); sx.rect(x0 + 2, y0 + 2, cw - 4, CH - 4); sx.clip();
      sx.strokeStyle = isSel(c) ? '#ffb300' : 'rgba(0,229,255,.25)'; sx.lineWidth = 1; sx.strokeRect(x0 + 2.5, y0 + 2.5, cw - 5, CH - 5);
      sx.font = '600 12px Inter,sans-serif'; sx.fillStyle = '#fff'; sx.fillText(`${c.name}, ${valStr(c)}`, x0 + 8, y0 + 18);
      sx.font = '10.5px "JetBrains Mono",monospace'; sx.fillStyle = '#00e5ff'; sx.fillText(`Máx V=${eng(mv < 1e-11 ? 0 : mv)}V`, x0 + 8, y0 + 31); sx.fillStyle = '#ffd54f'; sx.fillText(`Máx I=${eng(mi < 1e-11 ? 0 : mi)}A`, x0 + 8, y0 + 43);
      sx.strokeStyle = 'rgba(0,229,255,.13)'; for (let i = 0; i <= 10; i++) ln(sx, [[px + i * pw / 10, py], [px + i * pw / 10, py + ph]]);
      sx.strokeStyle = 'rgba(0,229,255,.4)'; ln(sx, [[px, py + ph / 2], [px + pw, py + ph / 2]]);
      if (h.length > 1) [[1, mv, '#00e5ff'], [2, mi, '#ffd54f']].forEach(([k, m, col]) => { sx.strokeStyle = col; sx.lineWidth = 1.6; ln(sx, h.map(q => [px + (q[0] - t0) / win * pw, py + ph / 2 - q[k] / m * (ph / 2 - 3)])); });
      else { sx.font = '11px Inter,sans-serif'; sx.fillStyle = '#6f84a8'; sx.fillText('Pulsa ▶ Iniciar', px + 6, py + ph / 2 - 6); }
      sx.restore();
    });
    if (rc) {
      const n = list.length, [x0, y0] = box0(n), m = sim && !sim.error && sim.t > 0 ? sim.read(rc) : null;
      sx.save(); sx.beginPath(); sx.rect(x0 + 2, y0 + 2, cw - 4, CH - 4); sx.clip();
      sx.fillStyle = 'rgba(0,229,255,.07)'; sx.fillRect(x0 + 2, y0 + 2, cw - 4, CH - 4); sx.strokeStyle = 'rgba(0,229,255,.5)'; sx.strokeRect(x0 + 2.5, y0 + 2.5, cw - 5, CH - 5);
      sx.font = '600 12px Inter,sans-serif'; sx.fillStyle = '#fff'; sx.fillText(`${T[rc.t].n} ${rc.name}`, x0 + 8, y0 + 18);
      sx.font = '11px "JetBrains Mono",monospace'; sx.fillStyle = '#c9d8f2';
      const L = m ? [`I  = ${eng(m.i)}A`, `Vd = ${eng(m.v)}V`, `P  = ${eng(m.v * m.i)}W`] : ['I  = —', 'Vd = —', 'P  = —'];
      if (rc.t === 'C') L.push(`C = ${eng(rc.val)}F` + (m ? ` · E=${eng(.5 * rc.val * m.v * m.v)}J` : ''));
      if (rc.t === 'L') L.push(`L = ${eng(rc.val)}H` + (m ? ` · E=${eng(.5 * rc.val * m.i * m.i)}J` : ''));
      if (rc.t === 'R') L.push(`R = ${eng(rc.val)}Ω`);
      L.forEach((q, k) => sx.fillText(q, x0 + 8, y0 + 36 + k * 14));
      sx.font = '11px Inter,sans-serif'; sx.fillStyle = '#9fb3c8';
      wrapTxt(hint(rc), cw - 18, L.length > 3 ? 3 : 4).forEach((q, k) => sx.fillText(q, x0 + 8, y0 + 40 + L.length * 14 + k * 13));
      sx.restore();
    }
  }

  function rebuild() { sim = make(comps, wires); hist.clear(); lastS = -1; cnt = 0; }
  $('scopeShow').onchange = e => { scopeShow = e.target.value; };

  /* Paleta */
  const palEl = $('pal');
  Object.keys(T).forEach(k => {
    const b = document.createElement('button'); b.className = 'pal'; b.dataset.k = k; b.innerHTML = `<b>${ICON[k]}</b>${T[k].n}`;
    b.onclick = () => { tool = tool === k ? null : k; pal(); }; palEl.appendChild(b);
  });
  { const b = document.createElement('button'); b.className = 'pal'; b.dataset.k = 'W'; b.innerHTML = '<b>╱</b>Cable (libre)'; b.onclick = () => { tool = tool === 'W' ? null : 'W'; wr = null; pal(); }; palEl.prepend(b); }
  const pal = () => palEl.querySelectorAll('.pal').forEach(b => b.classList.toggle('on', b.dataset.k === tool));

  /* Editor de valores (doble clic) */
  const FIELDS = { R: [['val', 'Resistencia (Ω)']], V: [['val', 'Voltaje (V)']], AC: [['val', 'Amplitud pico (V)'], ['f', 'Frecuencia (Hz)']],
    C: [['val', 'Capacidad (F)'], ['v0', 'Tensión inicial al reiniciar (V)', 1]], L: [['val', 'Inductancia (H)'], ['i0', 'Corriente inicial al reiniciar (A)', 1]],
    Z: [['val', 'Voltaje Zener (V)']], Q: [['val', 'Ganancia β']] };
  let dlg = null;
  const closeEdit = () => { if (dlg) { dlg.remove(); dlg = null; } };
  function openEdit(c) {
    closeEdit(); const fs = FIELDS[c.t]; if (!fs) return;
    dlg = document.createElement('div'); dlg.className = 'editdlg';
    dlg.innerHTML = `<h4>Editar ${T[c.t].n}</h4><label class="lb">Nombre<input data-k="name" value="${String(c.name).replace(/"/g, '&quot;')}"></label>` +
      fs.map(([k, lab]) => `<label class="lb">${lab}<input data-k="${k}" value="${eng(c[k] || 0)}"></label>`).join('') +
      '<p class="muted">Sufijos: p, n, u, m, k, M (ej.: 15u = 15 µF)</p><div class="eb"><button class="sb" data-a="ap">Aplicar</button><button class="sb go" data-a="ok">OK</button><button class="sb" data-a="no">Cancelar</button></div>';
    $('viewSim').appendChild(dlg);
    const apply = () => { let re = false;
      dlg.querySelectorAll('input').forEach(i => { const k = i.dataset.k; if (k === 'name') { c.name = i.value.trim() || c.name; return; }
        const f = fs.find(x => x[0] === k), v = parse(i.value); if (isFinite(v) && (f[2] || v > 0)) { if (c[k] !== v && f[2]) re = true; c[k] = v; } i.value = eng(c[k] || 0); });
      if (re) rebuild(); props(); commit(); };
    dlg.onclick = e => { const a = e.target.dataset.a; if (a === 'ap') apply(); else if (a === 'ok') { apply(); closeEdit(); } else if (a === 'no') closeEdit(); };
    dlg.onkeydown = e => { if (e.key === 'Enter') { apply(); closeEdit(); } else if (e.key === 'Escape') closeEdit(); };
    const first = dlg.querySelectorAll('input')[1]; if (first) { first.focus(); first.select(); }
  }

  /* Propiedades */
  function props() {
    const p = $('props'); btns();
    if (multi.length) { p.innerHTML = `<h4>${multi.length} elementos seleccionados</h4><p class="muted">Arrastra uno para mover todo el grupo. Ctrl+C copia, Ctrl+V pega, Supr elimina.</p>`; return; }
    if (!sel) { p.innerHTML = '<p class="muted">Selecciona un componente para editar sus propiedades.</p>'; return; }
    if (sel.k === 'w') { p.innerHTML = '<p class="muted">Cable seleccionado. Arrastra sus extremos para estirarlo o el cuerpo para moverlo. Supr lo elimina.</p>'; return; }
    const c = sel.o, t = T[c.t]; let h = `<h4>${c.name} · ${t.n}</h4>`;
    if (c.t === 'SW') h += `<button id="swb" class="sb">${c.on ? 'Abrir' : 'Cerrar'} interruptor</button>`;
    if (c.t === 'SW2') h += `<button id="swb" class="sb">⇄ Mover palanca a la ${c.pos ? 'izquierda' : 'derecha'}</button>`;
    if (FIELDS[c.t]) h += '<button id="edb" class="sb" style="margin:8px 0 2px">✏️ Editar valores…</button><p class="muted">Doble clic sobre el componente también abre el editor.</p>';
    if (t.lab) h += `<label class="lb">${t.lab}<input id="pv" value="${eng(c.val)}"></label>`;
    if (c.t === 'AC') h += `<label class="lb">Frecuencia (Hz)<input id="pf" value="${eng(c.f)}"></label>`;
    if (t.lab) h += '<p class="muted">Acepta sufijos: p, n, u, m, k, M. Ej.: 4.7k, 10u.</p>';
    p.innerHTML = h;
    const sw = $('swb'); if (sw) sw.onclick = () => { if (c.t === 'SW2') c.pos = !c.pos; else c.on = !c.on; props(); }; const eb = $('edb'); if (eb) eb.onclick = () => openEdit(c);
    const set = (id, key) => { const e = $(id); if (e) e.onchange = () => { const v = parse(e.value); if (v > 0) { c[key] = v; commit(); } e.value = eng(c[key]); }; };
    set('pv', 'val'); set('pf', 'f');
  }

  /* Historial (retroceder / avanzar) */
  const KEYS = ['id', 't', 'x', 'y', 'x2', 'y2', 'r', 'val', 'f', 'on', 'pos', 'name', 'v0', 'i0'];
  const pick = c => { const o = {}; KEYS.forEach(k => { if (c[k] !== undefined) o[k] = c[k]; }); return o; };
  const ser = () => JSON.stringify({ comps: comps.map(pick), wires: wires.map(w => ({ a: w.a, b: w.b })), uid, num });
  function btns() {
    const set = (id, off) => { const e = $(id); if (e) e.disabled = off; };
    set('bUndo', !undoS.length); set('bRedo', !redoS.length); set('bCopy', !selItems().length); set('bPaste', !clip);
  }
  function commit() {
    const q = ser(); if (q === lastSt) return;
    undoS.push(lastSt); if (undoS.length > 200) undoS.shift(); lastSt = q; redoS = []; btns();
  }
  function load(q) {
    const d = JSON.parse(q); comps = d.comps; wires = d.wires; uid = d.uid;
    Object.keys(num).forEach(k => delete num[k]); Object.assign(num, d.num);
    sel = null; multi = []; hov = null; dr = null; wr = null; rb = null; closeEdit(); props(); rebuild();
  }
  function undo() { if (!undoS.length) return; redoS.push(lastSt); lastSt = undoS.pop(); load(lastSt); btns(); }
  function redo() { if (!redoS.length) return; undoS.push(lastSt); lastSt = redoS.pop(); load(lastSt); btns(); }
  function setSel(items) { if (items.length === 1) { sel = items[0]; multi = []; } else { sel = null; multi = items; } props(); }

  /* Edición */
  const wp = e => { const r = cv.getBoundingClientRect(); return [(e.clientX - r.left) / G, (e.clientY - r.top) / G]; };
  const ports = () => { const a = []; comps.forEach(c => pins(c).forEach(p => a.push(p))); wires.forEach(w => a.push(w.a, w.b)); return a; };
  const nearPort = (p, r = .45) => { let b = null, bd = r; ports().forEach(q => { const d = Math.hypot(q[0] - p[0], q[1] - p[1]); if (d < bd) { bd = d; b = q; } }); return b; };
  const snapPt = p => { const q = nearPort(p, .6); return q ? q.slice() : [Math.round(p[0] * 2) / 2, Math.round(p[1] * 2) / 2]; };
  const same = (a, b) => Math.abs(a[0] - b[0]) < 1e-6 && Math.abs(a[1] - b[1]) < 1e-6;
  const box = c => { const q = (c.t === 'G' ? [[0,0],[0,1.3]] : LP[c.t]).map(p => rot(c, p)), xs = q.map(a => a[0]), ys = q.map(a => a[1]);
    return [Math.min(...xs) - .6, Math.min(...ys) - .6, Math.max(...xs) + .6, Math.max(...ys) + .6]; };
  const segD = (p, a, b) => { const dx = b[0] - a[0], dy = b[1] - a[1], l = dx * dx + dy * dy || 1, u = Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l));
    return Math.hypot(p[0] - a[0] - u * dx, p[1] - a[1] - u * dy); };
  const hitC = p => [...comps].reverse().find(c => { if (two(c)) { const q = pins(c); return segD(p, q[0], q[1]) < .8; }
    const b = box(c); return p[0] >= b[0] && p[0] <= b[2] && p[1] >= b[1] && p[1] <= b[3]; });
  const hitW = p => wires.find(w => segD(p, w.a, w.b) < .35);
  // Terminales que se pueden estirar: extremos de componentes de 2 terminales y extremos de cables
  const pinHit = p => { let b = null, bd = .5; comps.forEach(c => { if (!two(c)) return; pins(c).forEach((q, i) => { const d = Math.hypot(q[0] - p[0], q[1] - p[1]); if (d < bd) { bd = d; b = { c, i, pt: q.slice() }; } }); }); return b; };
  const wireEnd = p => { let b = null, bd = .5; wires.forEach(w => ['a', 'b'].forEach(k => { const d = Math.hypot(w[k][0] - p[0], w[k][1] - p[1]); if (d < bd) { bd = d; b = { w, pt: w[k].slice() }; } })); return b; };
  const wiresAt = pt => { const r = []; wires.forEach(w => ['a', 'b'].forEach(k => { if (same(w[k], pt)) r.push({ w, k }); })); return r; };
  const snapDrag = (p, cur, step) => { let b = null, bd = .5; ports().forEach(q => { if (same(q, cur)) return; const d = Math.hypot(q[0] - p[0], q[1] - p[1]); if (d < bd) { bd = d; b = q; } });
    return b ? b.slice() : [Math.round(p[0] / step) * step, Math.round(p[1] / step) * step]; };

  function place(s) {
    const t = T[tool], c = { id: ++uid, t: tool, x: s[0], y: s[1], r: 0, val: t.v, f: t.f, on: false, pos: false, name: tool === 'G' ? 'GND' : t.p + (num[tool] = (num[tool] || 0) + 1) };
    if (LP[tool].length === 2) { c.x2 = s[0] + 4; c.y2 = s[1]; }
    comps.push(c); sel = { k: 'c', o: c }; multi = []; tool = null; pal(); props(); rebuild(); commit();
  }
  // Devuelve true si fue doble clic (abre el editor)
  function downComp(c) {
    const now = performance.now(); sel = { k: 'c', o: c }; multi = []; props();
    if (c.t !== 'SW' && c.t !== 'SW2' && lastTap.c === c && now - lastTap.t < 380) { lastTap = { c: null, t: 0 }; openEdit(c); return true; }
    lastTap = { c, t: now }; return false;
  }
  function startMove(items, p) {
    dr = { mode: 'move', p0: p, m: false, c: items.length === 1 && items[0].k === 'c' ? items[0].o : null,
      items: items.map(i => ({ i, o0: JSON.parse(JSON.stringify(i.k === 'c' ? pick(i.o) : { a: i.o.a, b: i.o.b })) })) };
  }
  cv.onpointerdown = e => {
    cv.setPointerCapture(e.pointerId); const p = wp(e), s = p.map(Math.round);
    if (tool === 'W') { const q = snapPt(p); if (!wr) { wr = { a: q, b: q.slice() }; wrFresh = true; } else { wr.b = q; wrFresh = false; } return; }
    if (tool) { place(s); return; }
    const c0 = hitC(p), w0 = c0 ? null : hitW(p), ho = c0 || w0;
    if (multi.length && ho && isSel(ho)) { startMove(multi, p); return; }
    // Estirar: jalar el círculo de un componente o el extremo de un cable
    const ph = pinHit(p);
    if (ph) {
      if (downComp(ph.c)) return;
      dr = { mode: 'pin', c: ph.c, i: ph.i, cur: ph.pt, carry: wiresAt(ph.pt), m: false }; return;
    }
    const we = wireEnd(p);
    if (we) { sel = { k: 'w', o: we.w }; multi = []; props(); dr = { mode: 'wend', cur: we.pt, carry: wiresAt(we.pt), m: false }; return; }
    const q = nearPort(p);
    if (q) { wr = { a: q.slice(), b: q.slice() }; sel = null; multi = []; props(); return; }
    multi = [];
    if (c0) { if (downComp(c0)) return; startMove([sel], p); return; }
    if (w0) { sel = { k: 'w', o: w0 }; props(); startMove([sel], p); return; }
    sel = null; props(); rb = { a: p, b: p };
  };
  cv.onpointermove = e => {
    const p = wp(e); ptr = p.map(Math.round); ptrF = p;
    if (wr) wr.b = snapPt(p);
    if (rb) rb.b = p;
    if (dr) {
      if (dr.mode === 'move') {
        const dx = Math.round(p[0] - dr.p0[0]), dy = Math.round(p[1] - dr.p0[1]);
        dr.items.forEach(({ i, o0 }) => {
          if (i.k === 'c') { const c = i.o; c.x = o0.x + dx; c.y = o0.y + dy; if (o0.x2 !== undefined) { c.x2 = o0.x2 + dx; c.y2 = o0.y2 + dy; } }
          else { i.o.a = [o0.a[0] + dx, o0.a[1] + dy]; i.o.b = [o0.b[0] + dx, o0.b[1] + dy]; }
        });
        if (dx || dy) dr.m = true;
      } else {
        const t = snapDrag(p, dr.cur, dr.mode === 'pin' ? 1 : .5);
        if (!same(t, dr.cur)) {
          if (dr.mode === 'pin') {
            const o = dr.i ? [dr.c.x, dr.c.y] : [dr.c.x2, dr.c.y2];
            if (Math.hypot(t[0] - o[0], t[1] - o[1]) >= 1) {
              if (dr.i) { dr.c.x2 = t[0]; dr.c.y2 = t[1]; } else { dr.c.x = t[0]; dr.c.y = t[1]; }
              dr.carry.forEach(({ w, k }) => { w[k] = t.slice(); }); dr.cur = t; dr.m = true;
            }
          } else { dr.carry.forEach(({ w, k }) => { w[k] = t.slice(); }); dr.cur = t; dr.m = true; }
        }
      }
    } else hov = hitC(p) || null;
  };
  cv.onpointerleave = () => { hov = null; };
  cv.oncontextmenu = e => { e.preventDefault(); wr = null; };
  cv.ondblclick = () => { if (tool === 'W') wr = null; };
  function finishBand() {
    const x0 = Math.min(rb.a[0], rb.b[0]), x1 = Math.max(rb.a[0], rb.b[0]), y0 = Math.min(rb.a[1], rb.b[1]), y1 = Math.max(rb.a[1], rb.b[1]);
    if (x1 - x0 < .3 && y1 - y0 < .3) return;
    const inR = q => q[0] >= x0 && q[0] <= x1 && q[1] >= y0 && q[1] <= y1;
    const mid = c => { const q = pins(c); return [q.reduce((a, b) => a + b[0], 0) / q.length, q.reduce((a, b) => a + b[1], 0) / q.length]; };
    setSel([...comps.filter(c => inR(mid(c))).map(o => ({ k: 'c', o })), ...wires.filter(w => inR(w.a) && inR(w.b)).map(o => ({ k: 'w', o }))]);
  }
  cv.onpointerup = () => {
    if (rb) { finishBand(); rb = null; }
    if (wr) {
      const moved = Math.hypot(wr.b[0] - wr.a[0], wr.b[1] - wr.a[1]) > .3;
      if (tool === 'W') {
        if (moved) { wires.push({ a: wr.a, b: wr.b }); rebuild(); commit(); wr = wrFresh ? null : { a: wr.b.slice(), b: wr.b.slice() }; }
        else if (!wrFresh) wr = null;
        wrFresh = false;
      } else { if (moved) { wires.push({ a: wr.a, b: wr.b }); rebuild(); commit(); } wr = null; }
    }
    if (dr) {
      if (dr.m) { rebuild(); commit(); }
      else if (dr.c && dr.c.t === 'SW') { dr.c.on = !dr.c.on; props(); }
      else if (dr.c && dr.c.t === 'SW2') { dr.c.pos = !dr.c.pos; props(); }
      dr = null;
    }
  };
  function rotate() {
    if (!sel || sel.k !== 'c') return; const c = sel.o;
    if (two(c)) { pins(c); const dx = c.x2 - c.x, dy = c.y2 - c.y; c.x2 = c.x - dy; c.y2 = c.y + dx; } else c.r = (c.r + 1) % 4;
    rebuild(); commit();
  }
  function del() {
    const it = selItems(); if (!it.length) return;
    comps = comps.filter(c => !it.some(i => i.o === c)); wires = wires.filter(w => !it.some(i => i.o === w));
    sel = null; multi = []; props(); rebuild(); commit();
  }
  function clearAll() {
    if (!comps.length && !wires.length) return;
    if (!confirm('¿Borrar todo el circuito?\nPuedes recuperarlo con el botón ↶ (retroceder).')) return;
    comps = []; wires = []; sel = null; multi = []; Object.keys(num).forEach(k => delete num[k]); run = false; props(); rebuild(); commit(); $('st').textContent = 'Circuito borrado';
  }
  function copy() {
    const it = selItems(); if (!it.length) { $('st').textContent = 'Selecciona algo para copiar (arrastra un recuadro sobre el circuito).'; return; }
    clip = { comps: it.filter(i => i.k === 'c').map(i => pick(i.o)), wires: it.filter(i => i.k === 'w').map(i => ({ a: i.o.a.slice(), b: i.o.b.slice() })) };
    pasteN = 0; btns(); $('st').textContent = 'Copiado: ' + it.length + ' elemento(s). Pulsa Pegar.';
  }
  function paste() {
    if (!clip) return; pasteN++; const o = 2 * pasteN, items = [];
    clip.comps.forEach(d => {
      const c = Object.assign({}, d, { id: ++uid, x: d.x + o, y: d.y + o });
      if (d.x2 !== undefined) { c.x2 = d.x2 + o; c.y2 = d.y2 + o; }
      if (c.t !== 'G') c.name = T[c.t].p + (num[c.t] = (num[c.t] || 0) + 1);
      comps.push(c); items.push({ k: 'c', o: c });
    });
    clip.wires.forEach(w => { const n = { a: [w.a[0] + o, w.a[1] + o], b: [w.b[0] + o, w.b[1] + o] }; wires.push(n); items.push({ k: 'w', o: n }); });
    setSel(items); rebuild(); commit();
  }
  function selAll() { setSel([...comps.map(o => ({ k: 'c', o })), ...wires.map(o => ({ k: 'w', o }))]); }
  $('bRot').onclick = rotate; $('bDel').onclick = del; $('bUndo').onclick = undo; $('bRedo').onclick = redo;
  $('bCopy').onclick = copy; $('bPaste').onclick = paste; $('bClear').onclick = clearAll;
  addEventListener('keydown', e => {
    if ($('viewSim').hidden || /INPUT|SELECT|TEXTAREA/.test(e.target.tagName)) return;
    if (e.ctrlKey || e.metaKey) {
      const k = e.key.toLowerCase(), h = { z: () => (e.shiftKey ? redo() : undo()), y: redo, c: copy, v: paste, a: selAll }[k];
      if (h) { e.preventDefault(); h(); } return;
    }
    if (e.key === 'r' || e.key === 'R') rotate();
    else if (e.key === 'Delete' || e.key === 'Backspace') del();
    else if (e.key === 'w' || e.key === 'W') { tool = tool === 'W' ? null : 'W'; wr = null; pal(); }
    else if (e.key === 'Escape') { tool = null; wr = null; rb = null; pal(); }
  });

  /* Control de simulación */
  const st = $('st');
  $('bRun').onclick = () => {
    if (!comps.length) { st.textContent = 'Agrega componentes primero.'; return; }
    if (!sim) rebuild();
    if (sim.error) { st.textContent = sim.error; return; }
    run = true;
  };
  $('bPause').onclick = () => { run = false; };
  $('bReset').onclick = () => { run = false; rebuild(); st.textContent = 'Reiniciado: t = 0'; };
  $('dtS').onchange = e => { dt = +e.target.value; clr(); };
  const clr = () => { hist.clear(); lastS = -1; };
  const TD = [1e-5,2e-5,5e-5,1e-4,2e-4,5e-4,1e-3,2e-3,5e-3,1e-2,2e-2,5e-2,.1,.2,.5,1];
  $('tdS').innerHTML = TD.map(v => `<option value="${v}"${v === tdiv ? ' selected' : ''}>${eng(v)}s</option>`).join('');
  $('tdS').onchange = e => { tdiv = +e.target.value; clr(); };
  $('spd').oninput = e => { spf = +e.target.value; };

function frame() {
    if (run && sim && !sim.error) {
      const t0 = performance.now(), minI = Math.max(dt, tdiv / 80);
      for (let i = 0; i < spf; i++) {
        sim.step(dt);
        if (lastS < 0 || sim.t - lastS >= minI - dt / 2) {
          lastS = sim.t;
          comps.forEach(c => { if (c.t === 'G') return; let h = hist.get(c.id); if (!h) { h = []; hist.set(c.id, h); } const m = sim.read(c); h.push([sim.t, m.v, m.i]);
            while (h.length && h[0][0] < sim.t - tdiv * 10 - minI) h.shift(); });
        }
        if (i % 10 === 9 && performance.now() - t0 > 12) break;
      }
      if (sim.bad) { run = false; st.textContent = 'La simulación no convergió. Revisa el circuito.'; }
      else st.textContent = '● Simulando · t = ' + eng(sim.t) + 's';
    } else if (sim && sim.error) st.textContent = sim.error;
    else if (sim && sim.t > 0) st.textContent = '⏸ En pausa · t = ' + eng(sim.t) + 's';
    draw(); drawScope(); requestAnimationFrame(frame);
  }
  lastSt = ser(); props(); frame();
}
})(typeof window !== 'undefined' ? window : globalThis);
