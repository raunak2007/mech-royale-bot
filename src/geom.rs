//! Static map: terrain height grid, building prisms rasterised to a 1 m grid,
//! line of sight, wall distance. Everything here is derived from the two files
//! the server mounts at hello.map.dir (v2/world.json, v2/terrain.i16).

use crate::json::{self, J};

pub const STEP_UP: f32 = 1.2;
/// Elevated solids (bridges) with at least this much room under them are walked under.
pub const UNDER_CLEAR: f32 = 11.5;

static EMBED_WORLD: &str = include_str!("../arena/v2/world.json");
static EMBED_TERRAIN: &[u8] = include_bytes!("../arena/v2/terrain.i16");

pub struct Prism {
    pub kind: u8, // 0 building, 1 stadium, 2 bridge
    pub base: f32,
    pub top: f32,
    pub ring: Vec<[f32; 2]>,
    pub holes: Vec<Vec<[f32; 2]>>,
}

pub struct Map {
    pub min_x: f64,
    pub min_y: f64,
    pub nx: usize,
    pub ny: usize,
    pub bx0: f64,
    pub bx1: f64,
    pub by0: f64,
    pub by1: f64,
    /// terrain height at each vertex (m)
    pub terr: Vec<f32>,
    /// top of the ground-attached solid at each vertex (== terr where there is none)
    pub top: Vec<f32>,
    /// elevated solid (bridge / overhang): [ebase, etop]; ebase = +inf where there is none
    pub ebase: Vec<f32>,
    pub etop: Vec<f32>,
    /// 1 where a ground-level mech cannot stand (building footprint or outside the map)
    pub wall: Vec<u8>,
    /// distance (m) from each vertex to the nearest wall vertex
    pub wdist: Vec<f32>,
    /// the surface a ground-level mech walks on at each vertex
    pub walk: Vec<f32>,
    /// tree crowns over each vertex: [cbase, ctop] (cbase = +inf where there is none)
    pub cbase: Vec<f32>,
    pub ctop: Vec<f32>,
    pub prisms: Vec<Prism>,
}

/// Metres of crown a sight line can cross before it is blocked (protocol §8: soft canopy).
pub const CANOPY_BLOCK_M: f64 = 3.0;

fn poly(v: &J) -> Vec<[f32; 2]> {
    let a = v.arr();
    let mut out = Vec::with_capacity(a.len() / 2);
    let mut i = 0;
    while i + 1 < a.len() {
        out.push([a[i].f() as f32, a[i + 1].f() as f32]);
        i += 2;
    }
    out
}

impl Map {
    pub fn load(dir: &str) -> Map {
        let wj = std::fs::read_to_string(format!("{dir}/v2/world.json")).ok();
        let tb = std::fs::read(format!("{dir}/v2/terrain.i16")).ok();
        if let (Some(w), Some(t)) = (wj, tb) {
            match Map::from_bytes(&w, &t) {
                Ok(m) => return m,
                Err(e) => eprintln!("map at {dir} unusable ({e}); using the embedded copy"),
            }
        } else {
            eprintln!("map at {dir} unreadable; using the embedded copy");
        }
        Map::from_bytes(EMBED_WORLD, EMBED_TERRAIN).expect("embedded map")
    }

    pub fn embedded() -> Map {
        Map::from_bytes(EMBED_WORLD, EMBED_TERRAIN).expect("embedded map")
    }

    pub fn from_bytes(world: &str, terrain: &[u8]) -> Result<Map, String> {
        let w = json::parse(world)?;
        let t = w.get("terrain");
        let nx = t.get("nx").f() as usize;
        let ny = t.get("ny").f() as usize;
        let step = t.get("step").fo(1.0);
        let scale = t.get("scale").fo(0.01) as f32;
        if nx < 8 || ny < 8 || (step - 1.0).abs() > 1e-6 {
            return Err(format!("unexpected terrain grid {nx}x{ny} step {step}"));
        }
        if terrain.len() < nx * ny * 2 {
            return Err("terrain file too short".into());
        }
        let min_x = t.get("minX").f();
        let min_y = t.get("minY").f();
        let n = nx * ny;
        let mut terr = vec![0f32; n];
        for (k, h) in terr.iter_mut().enumerate() {
            *h = i16::from_le_bytes([terrain[2 * k], terrain[2 * k + 1]]) as f32 * scale;
        }
        let b = w.get("bounds");
        let (bx0, bx1) = (b.get("minX").fo(min_x), b.get("maxX").fo(min_x + nx as f64 - 1.0));
        let (by0, by1) = (b.get("minY").fo(min_y), b.get("maxY").fo(min_y + ny as f64 - 1.0));

        let mut prisms = Vec::new();
        for p in w.get("prisms").arr() {
            let kind = match p.get("kind").s() {
                "building" => 0,
                "stadium" => 1,
                "bridge" => 2,
                _ => 0,
            };
            let ring = poly(p.get("ring"));
            if ring.len() < 3 {
                continue;
            }
            let holes = p.get("holes").arr().iter().map(poly).filter(|h| h.len() >= 3).collect();
            prisms.push(Prism { kind, base: p.get("base").f() as f32, top: p.get("top").f() as f32, ring, holes });
        }

        let mut top = terr.clone();
        let mut ebase = vec![f32::INFINITY; n];
        let mut etop = vec![f32::NEG_INFINITY; n];
        let mut xs: Vec<f32> = Vec::new();
        for p in &prisms {
            let (mut y0, mut y1) = (f32::INFINITY, f32::NEG_INFINITY);
            for q in &p.ring {
                y0 = y0.min(q[1]);
                y1 = y1.max(q[1]);
            }
            let j0 = ((y0 as f64 - min_y).ceil().max(0.0)) as usize;
            let j1 = ((y1 as f64 - min_y).floor().min(ny as f64 - 1.0)).max(0.0) as usize;
            for j in j0..=j1.min(ny - 1) {
                let y = (min_y + j as f64) as f32 + 1e-4;
                xs.clear();
                let mut edges = |r: &Vec<[f32; 2]>| {
                    let m = r.len();
                    for k in 0..m {
                        let a = r[k];
                        let c = r[(k + 1) % m];
                        if (a[1] <= y) != (c[1] <= y) {
                            xs.push(a[0] + (y - a[1]) * (c[0] - a[0]) / (c[1] - a[1]));
                        }
                    }
                };
                edges(&p.ring);
                for h in &p.holes {
                    edges(h);
                }
                xs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                let mut k = 0;
                while k + 1 < xs.len() {
                    let i0 = ((xs[k] as f64 - min_x).ceil().max(0.0)) as usize;
                    let i1f = (xs[k + 1] as f64 - min_x).floor();
                    if i1f >= 0.0 {
                        let i1 = (i1f as usize).min(nx - 1);
                        for i in i0..=i1 {
                            let v = j * nx + i;
                            if p.base <= terr[v] + 1.5 {
                                if p.top > top[v] {
                                    top[v] = p.top;
                                }
                            } else {
                                if p.base < ebase[v] {
                                    ebase[v] = p.base;
                                }
                                if p.top > etop[v] {
                                    etop[v] = p.top;
                                }
                            }
                        }
                    }
                    k += 2;
                }
            }
        }
        // An elevated solid resting on (or overlapping) the solid below is just more building.
        for v in 0..n {
            if ebase[v].is_finite() && ebase[v] <= top[v] + 0.5 {
                if etop[v] > top[v] {
                    top[v] = etop[v];
                }
                ebase[v] = f32::INFINITY;
                etop[v] = f32::NEG_INFINITY;
            }
        }

        let mut wall = vec![0u8; n];
        let mut walk = vec![0f32; n];
        for j in 0..ny {
            for i in 0..nx {
                let v = j * nx + i;
                let x = min_x + i as f64;
                let y = min_y + j as f64;
                let outside = x < bx0 + 0.5 || x > bx1 - 0.5 || y < by0 + 0.5 || y > by1 - 0.5;
                // Low bridges and skywalks cannot be walked under: walls for the ground layer.
                let low_deck = ebase[v].is_finite() && ebase[v] - top[v] < UNDER_CLEAR;
                if outside || top[v] > terr[v] + STEP_UP || low_deck {
                    wall[v] = 1;
                }
                walk[v] = top[v];
            }
        }
        let wdist = edt(&wall, nx, ny);
        // tree crowns (they soften sight, never stop shots)
        let mut cbase = vec![f32::INFINITY; n];
        let mut ctop = vec![f32::NEG_INFINITY; n];
        let tr = w.get("trees");
        let fields: Vec<&str> = tr.get("fields").arr().iter().map(|f| f.s()).collect();
        let col = |name: &str| fields.iter().position(|f| *f == name);
        if let (Some(cz), Some(ch), Some(cr), Some(cb), Some(cx), Some(cy)) = (col("z"), col("height"), col("crownR"), col("crownBase"), col("crownX"), col("crownY")) {
            let data = tr.get("data").arr();
            let nf = fields.len().max(1);
            let mut k = 0;
            while k + nf <= data.len() {
                let (z, h, r, b) = (data[k + cz].f(), data[k + ch].f(), data[k + cr].f(), data[k + cb].f());
                let (x, y) = (data[k + cx].f(), data[k + cy].f());
                k += nf;
                if r <= 0.0 || h <= b {
                    continue;
                }
                let i0 = ((x - r - min_x).ceil().max(0.0)) as usize;
                let i1 = ((x + r - min_x).floor().min(nx as f64 - 1.0)).max(0.0) as usize;
                let j0 = ((y - r - min_y).ceil().max(0.0)) as usize;
                let j1 = ((y + r - min_y).floor().min(ny as f64 - 1.0)).max(0.0) as usize;
                for j in j0..=j1 {
                    for i in i0..=i1 {
                        let (dx, dy) = (min_x + i as f64 - x, min_y + j as f64 - y);
                        if dx * dx + dy * dy <= r * r {
                            let v = j * nx + i;
                            cbase[v] = cbase[v].min((z + b) as f32);
                            ctop[v] = ctop[v].max((z + h) as f32);
                        }
                    }
                }
            }
        }
        Ok(Map { min_x, min_y, nx, ny, bx0, bx1, by0, by1, terr, top, ebase, etop, wall, wdist, walk, cbase, ctop, prisms })
    }

    #[inline]
    pub fn vidx(&self, x: f64, y: f64) -> Option<usize> {
        let i = (x - self.min_x).round();
        let j = (y - self.min_y).round();
        if i < 0.0 || j < 0.0 || i >= self.nx as f64 || j >= self.ny as f64 {
            None
        } else {
            Some(j as usize * self.nx + i as usize)
        }
    }

    /// bilinear terrain height
    pub fn ground(&self, x: f64, y: f64) -> f64 {
        let fx = (x - self.min_x).clamp(0.0, self.nx as f64 - 1.001);
        let fy = (y - self.min_y).clamp(0.0, self.ny as f64 - 1.001);
        let i = fx as usize;
        let j = fy as usize;
        let tx = fx - i as f64;
        let ty = fy - j as f64;
        let h = |a: usize, b: usize| self.terr[b * self.nx + a] as f64;
        h(i, j) * (1.0 - tx) * (1.0 - ty) + h(i + 1, j) * tx * (1.0 - ty) + h(i, j + 1) * (1.0 - tx) * ty + h(i + 1, j + 1) * tx * ty
    }

    /// walking surface for a ground-level mech (nearest vertex)
    pub fn walk_z(&self, x: f64, y: f64) -> f64 {
        match self.vidx(x, y) {
            Some(v) => self.walk[v] as f64,
            None => 0.0,
        }
    }

    pub fn wall_dist(&self, x: f64, y: f64) -> f32 {
        match self.vidx(x, y) {
            Some(v) => self.wdist[v],
            None => 0.0,
        }
    }

    /// True when the straight segment a→b is not blocked by terrain, buildings or bridges.
    /// Trees are ignored (they never stop shots, and only soften sight).
    pub fn los(&self, a: [f64; 3], b: [f64; 3]) -> bool {
        self.los_step(a, b, 1.0)
    }

    pub fn los_step(&self, a: [f64; 3], b: [f64; 3], step: f64) -> bool {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let dz = b[2] - a[2];
        let d = (dx * dx + dy * dy).sqrt();
        let n = (d / step).ceil() as usize;
        if n <= 1 {
            return true;
        }
        let inv = 1.0 / n as f64;
        let (nx, ny) = (self.nx as f64, self.ny as f64);
        for k in 1..n {
            let t = k as f64 * inv;
            let i = (a[0] + dx * t - self.min_x).round();
            let j = (a[1] + dy * t - self.min_y).round();
            if i < 0.0 || j < 0.0 || i >= nx || j >= ny {
                continue;
            }
            let v = j as usize * self.nx + i as usize;
            let z = (a[2] + dz * t) as f32;
            if z < self.top[v] {
                return false;
            }
            if z >= self.ebase[v] && z <= self.etop[v] {
                return false;
            }
        }
        true
    }

    /// Sight line: like los(), and also blocked after CANOPY_BLOCK_M metres of tree crown.
    pub fn los_vis(&self, a: [f64; 3], b: [f64; 3]) -> bool {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let dz = b[2] - a[2];
        let d = (dx * dx + dy * dy).sqrt();
        let n = d.ceil() as usize;
        if n <= 1 {
            return true;
        }
        let inv = 1.0 / n as f64;
        let seg = (d * d + dz * dz).sqrt() * inv;
        let (nx, ny) = (self.nx as f64, self.ny as f64);
        let mut crown = 0.0;
        for k in 1..n {
            let t = k as f64 * inv;
            let i = (a[0] + dx * t - self.min_x).round();
            let j = (a[1] + dy * t - self.min_y).round();
            if i < 0.0 || j < 0.0 || i >= nx || j >= ny {
                continue;
            }
            let v = j as usize * self.nx + i as usize;
            let z = (a[2] + dz * t) as f32;
            if z < self.top[v] || (z >= self.ebase[v] && z <= self.etop[v]) {
                return false;
            }
            if z >= self.cbase[v] && z <= self.ctop[v] {
                crown += seg;
                if crown > CANOPY_BLOCK_M {
                    return false;
                }
            }
        }
        true
    }

    /// Free length of a horizontal ray at height z from (x, y) along yaw, up to max_d.
    pub fn ray_len(&self, x: f64, y: f64, z: f32, yaw: f64, max_d: f64, step: f64) -> f64 {
        let (c, s) = (yaw.cos(), yaw.sin());
        let n = (max_d / step) as usize;
        for k in 1..=n {
            let d = k as f64 * step;
            let i = (x + c * d - self.min_x).round();
            let j = (y + s * d - self.min_y).round();
            if i < 0.0 || j < 0.0 || i >= self.nx as f64 || j >= self.ny as f64 {
                return d;
            }
            let v = j as usize * self.nx + i as usize;
            if z < self.top[v] || (z >= self.ebase[v] && z <= self.etop[v]) {
                return d;
            }
        }
        max_d
    }

    /// Bilinear wall distance and its gradient (points away from walls).
    pub fn wdist_grad(&self, x: f64, y: f64) -> (f64, f64, f64) {
        let fx = (x - self.min_x).clamp(0.0, self.nx as f64 - 1.001);
        let fy = (y - self.min_y).clamp(0.0, self.ny as f64 - 1.001);
        let i = fx as usize;
        let j = fy as usize;
        let tx = fx - i as f64;
        let ty = fy - j as f64;
        let h = |a: usize, b: usize| self.wdist[b * self.nx + a] as f64;
        let (a, b, c, d) = (h(i, j), h(i + 1, j), h(i, j + 1), h(i + 1, j + 1));
        let v = a * (1.0 - tx) * (1.0 - ty) + b * tx * (1.0 - ty) + c * (1.0 - tx) * ty + d * tx * ty;
        let gx = (b - a) * (1.0 - ty) + (d - c) * ty;
        let gy = (c - a) * (1.0 - tx) + (d - b) * tx;
        (v, gx, gy)
    }

    /// Highest surface a mech with feet at z can stand on at (x, y).
    pub fn surface(&self, x: f64, y: f64, z: f64) -> (f64, bool) {
        let g = self.ground(x, y);
        let mut best = g;
        let mut solid = false;
        if let Some(v) = self.vidx(x, y) {
            let lim = z + STEP_UP as f64 + 1e-6;
            let tp = self.top[v] as f64;
            if tp > self.terr[v] as f64 + 0.01 && tp <= lim && tp > best {
                best = tp;
                solid = true;
            }
            let et = self.etop[v] as f64;
            if self.ebase[v].is_finite() && et <= lim && et > best {
                best = et;
                solid = true;
            }
        }
        (best, solid)
    }

    /// Does a cylinder (feet z, radius r, height h) at (x, y) intersect a wall? Returns the
    /// unit normal pointing from the nearest blocking vertex to the centre.
    pub fn blocked(&self, x: f64, y: f64, z: f64, r: f64, h: f64) -> Option<(f64, f64)> {
        let i0 = ((x - r - self.min_x).ceil().max(0.0)) as usize;
        let i1 = ((x + r - self.min_x).floor().min(self.nx as f64 - 1.0)).max(0.0) as usize;
        let j0 = ((y - r - self.min_y).ceil().max(0.0)) as usize;
        let j1 = ((y + r - self.min_y).floor().min(self.ny as f64 - 1.0)).max(0.0) as usize;
        if x < self.bx0 + r || x > self.bx1 - r || y < self.by0 + r || y > self.by1 - r {
            let cx = (self.bx0 + self.bx1) * 0.5;
            let cy = (self.by0 + self.by1) * 0.5;
            let (dx, dy) = (cx - x, cy - y);
            let d = (dx * dx + dy * dy).sqrt().max(1e-6);
            // push back toward the inside along the violated axis
            let mut n = (0.0, 0.0);
            if x < self.bx0 + r {
                n.0 = 1.0;
            } else if x > self.bx1 - r {
                n.0 = -1.0;
            }
            if y < self.by0 + r {
                n.1 = 1.0;
            } else if y > self.by1 - r {
                n.1 = -1.0;
            }
            let l = (n.0 * n.0 + n.1 * n.1 as f64).sqrt();
            if l > 0.0 {
                return Some((n.0 / l, n.1 / l));
            }
            return Some((dx / d, dy / d));
        }
        let lim = (z + STEP_UP as f64) as f32;
        let head = (z + h) as f32;
        let mut best: Option<(f64, f64, f64)> = None;
        for j in j0..=j1 {
            for i in i0..=i1 {
                let v = j * self.nx + i;
                let solid = self.top[v] > lim && self.top[v] > self.terr[v] + 0.01;
                let elev = self.ebase[v] < head && self.etop[v] > lim;
                if !solid && !elev {
                    continue;
                }
                let vx = self.min_x + i as f64;
                let vy = self.min_y + j as f64;
                let d2 = (x - vx) * (x - vx) + (y - vy) * (y - vy);
                if d2 <= r * r && best.map_or(true, |b| d2 < b.0) {
                    best = Some((d2, x - vx, y - vy));
                }
            }
        }
        best.map(|(d2, dx, dy)| {
            let d = d2.sqrt().max(1e-6);
            (dx / d, dy / d)
        })
    }

    /// Fraction along a→b where the segment first hits terrain or a solid (None if clear).
    pub fn ray_hit(&self, a: [f64; 3], b: [f64; 3]) -> Option<f64> {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let dz = b[2] - a[2];
        let d = (dx * dx + dy * dy + dz * dz).sqrt();
        let n = (d / 0.75).ceil().max(1.0) as usize;
        for k in 1..=n {
            let t = k as f64 / n as f64;
            let (x, y, z) = (a[0] + dx * t, a[1] + dy * t, (a[2] + dz * t) as f32);
            match self.vidx(x, y) {
                None => return Some(t),
                Some(v) => {
                    if z < self.top[v] || (z >= self.ebase[v] && z <= self.etop[v]) {
                        return Some(t);
                    }
                }
            }
        }
        None
    }
}

/// Exact Euclidean distance transform (Felzenszwalb & Huttenlocher), distances in cells.
pub fn edt(wall: &[u8], nx: usize, ny: usize) -> Vec<f32> {
    const BIG: f32 = 1e12;
    let n = nx * ny;
    let mut f: Vec<f32> = wall.iter().map(|&w| if w != 0 { 0.0 } else { BIG }).collect();
    let m = nx.max(ny);
    let mut d = vec![0f32; m];
    let mut src = vec![0f32; m];
    let mut v = vec![0usize; m];
    let mut z = vec![0f32; m + 1];
    let mut pass = |src: &[f32], d: &mut [f32], len: usize| {
        let mut k = 0usize;
        v[0] = 0;
        z[0] = f32::NEG_INFINITY;
        z[1] = f32::INFINITY;
        for q in 1..len {
            loop {
                let p = v[k];
                let s = ((src[q] + (q * q) as f32) - (src[p] + (p * p) as f32)) / (2.0 * q as f32 - 2.0 * p as f32);
                if s <= z[k] && k > 0 {
                    k -= 1;
                } else {
                    k += 1;
                    v[k] = q;
                    z[k] = s;
                    z[k + 1] = f32::INFINITY;
                    break;
                }
            }
        }
        k = 0;
        for q in 0..len {
            while z[k + 1] < q as f32 {
                k += 1;
            }
            let p = v[k];
            let dq = q as f32 - p as f32;
            d[q] = dq * dq + src[p];
        }
    };
    // columns
    for i in 0..nx {
        for j in 0..ny {
            src[j] = f[j * nx + i];
        }
        pass(&src[..ny], &mut d[..ny], ny);
        for j in 0..ny {
            f[j * nx + i] = d[j];
        }
    }
    // rows
    for j in 0..ny {
        src[..nx].copy_from_slice(&f[j * nx..j * nx + nx]);
        pass(&src[..nx], &mut d[..nx], nx);
        f[j * nx..j * nx + nx].copy_from_slice(&d[..nx]);
    }
    let _ = n;
    f.iter().map(|&x| x.min(1e8).sqrt()).collect()
}
