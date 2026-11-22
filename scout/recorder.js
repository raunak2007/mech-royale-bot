// Passive recorder for the organizers' public spectator feed (wss://watch.joinbuilders.org/live).
// Runs in a browser tab on the watch origin. Keeps the last 12 matches in memory as typed arrays.
(() => {
  if (window.REC && window.REC.ws) { try { window.REC.ws.onclose = null; window.REC.ws.close(); } catch (e) {} }
  const R = window.REC = { all: [], cur: null, chan: null, chanLog: [], nlines: 0, ws: null, started: Date.now(), reconnects: 0 };
  const newMatch = (h) => ({ hdr: h, chan: R.chan, t: [], m: [], z: [], ev: [], d: [], n: [], ended: false, at: Date.now() });
  const onLine = (l) => {
    if (!l) return; let o; try { o = JSON.parse(l); } catch (e) { return; }
    R.nlines++;
    if (o.type === "channel") { R.chan = o; R.chanLog.push([Date.now(), o.phase, o.match, o.title, o.next_at]); if (R.chanLog.length > 300) R.chanLog.shift(); return; }
    if (o.type === "header") {
      const last = R.all[R.all.length - 1];
      if (last && last.hdr.seed === o.seed) { R.cur = newMatch(o); R.all[R.all.length - 1] = R.cur; }
      else { if (R.cur) window.recFinal(R.cur); R.cur = newMatch(o); R.all.push(R.cur); if (R.all.length > 12) R.all.shift(); }
      return;
    }
    const c = R.cur; if (!c || o.t === undefined || !o.m) return;
    if (c.t.length && o.t <= c.t[c.t.length - 1]) return;
    c.t.push(o.t);
    const n = o.m.length, a = new Float32Array(n * 13);
    for (let i = 0; i < n; i++) { const r = o.m[i]; for (let k = 0; k < 13; k++) a[i * 13 + k] = r[k]; }
    c.m.push(a);
    c.z.push(o.z ? [o.z[0], o.z[1], o.z[2]] : null);
    if (o.ev) for (const e of o.ev) { c.ev.push(e); if (e.e === "end") { c.ended = true; window.recFinal(c); } }
    if (o.d) c.d.push([o.t, o.d]);
    if (o.t % 20 === 0 && o.n) c.n.push([o.t, o.n]);
  };
  window.recFinal = (c) => {
    if (c.saved || c.t.length < 50) return; c.saved = true;
    try {
      const S = window.summ(c);
      const rec = { at: c.at, seed: c.hdr.seed, title: (c.chan && c.chan.title) || "", no: c.chan && c.chan.match, T: c.t[c.t.length - 1] / 20, bots: c.hdr.bots,
        rows: S.map(s => [s.name, s.place, s.died, s.by, s.cubes, Math.round(s.mhp), s.kills, Math.round(s.dd), Math.round(s.dt), Math.round(s.dn), Math.round(100 * s.elev / Math.max(1, s.alive)), Math.round(s.dist), s.up.join(",")]) };
      const all = JSON.parse(localStorage.getItem("REC_SUMM") || "[]"); all.push(rec); while (all.length > 150) all.shift();
      localStorage.setItem("REC_SUMM", JSON.stringify(all));
    } catch (e) { R.saveErr = String(e); }
  };
  window.summ = (c) => {
    const H = c.hdr, F = c.t.length;
    const S = H.mechs.map((m, k) => ({ id: m.id, name: m.name, bot: (H.bots || [])[k], kills: 0, dd: 0, dt: 0, dn: 0, up: [], died: null, place: null, by: null, elev: 0, air: 0, alive: 0, dist: 0, firstElev: null, cubes: 0, mhp: 0, w: "", lp: null }));
    const idx = {}; S.forEach((s, k) => idx[s.id] = k);
    for (let f = 0; f < F; f++) { const a = c.m[f]; for (let i = 0; i < a.length; i += 13) { const s = S[idx[a[i]]]; if (!s) continue; s.alive++; const g = a[i + 8] & 1; const h = a[i + 3] - window.terr(a[i + 1], a[i + 2]); if (g && h > 5) { s.elev++; if (s.firstElev === null) s.firstElev = c.t[f] / 20; } if (!g) s.air++; if (s.lp) s.dist += Math.hypot(a[i + 1] - s.lp[0], a[i + 2] - s.lp[1]); s.lp = [a[i + 1], a[i + 2]]; s.cubes = a[i + 9]; s.mhp = a[i + 10]; s.w = a[i + 11] + "/" + a[i + 12]; s.hp = a[i + 6]; } }
    for (const e of c.ev) {
      if (e.e === "hit") { const a = S[idx[e.src]], b = S[idx[e.dst]]; if (a && b) { a.dd += e.amt; b.dt += e.amt; } else if (a) a.dn += e.amt; }
      else if (e.e === "death" && e.place !== undefined) { const s = S[idx[e.id]]; if (s) { s.died = e.t / 20; s.place = e.place; s.by = e.by; const k = S[idx[e.by]]; if (k) k.kills++; } }
      else if (e.e === "upgrade") { const s = S[idx[e.id]]; if (s) s.up.push(String(e.up).slice(0, 5)); }
    }
    return S;
  };
  window.terr = (x, y) => { const i = Math.round(x + 513), j = Math.round(y + 334); if (i < 0 || j < 0 || i >= 1070 || j >= 724 || !window.TERR) return 0; return window.TERR[j * 1070 + i] * 0.01; };
  const connect = () => {
    const ws = new WebSocket("wss://watch.joinbuilders.org/live"); R.ws = ws;
    ws.onmessage = (e) => { if (typeof e.data === "string") { const parts = e.data.split("\n"); for (const l of parts) onLine(l); } };
    ws.onclose = () => { R.reconnects++; setTimeout(() => { if (R.ws === ws) connect(); }, 3000); };
  };
  connect();
  if (!window.TERR) fetch("/arena/v2/terrain.i16").then(r => r.arrayBuffer()).then(b => { window.TERR = new Int16Array(b); });
  if (!window.WORLD) fetch("/arena/v2/world.json").then(r => r.json()).then(w => { window.WORLD = w; });
  return "recorder installed";
})()
