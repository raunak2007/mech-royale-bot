//! Ground navigation on a 2 m grid: A* between points and a bounded Dijkstra
//! flood used by the position planner.

use crate::geom::{Map, STEP_UP};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

pub const CELL: f64 = 2.0;
pub const DIRS: [(i32, i32); 8] = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)];
const DLEN: [f32; 8] = [2.0, 2.828, 2.0, 2.828, 2.0, 2.828, 2.0, 2.828];

#[derive(Copy, Clone)]
struct Node {
    f: f32,
    c: u32,
}
impl PartialEq for Node {
    fn eq(&self, o: &Self) -> bool {
        self.f == o.f
    }
}
impl Eq for Node {}
impl PartialOrd for Node {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Node {
    fn cmp(&self, o: &Self) -> Ordering {
        o.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
    }
}

pub struct Nav {
    pub w: usize,
    pub h: usize,
    pub min_x: f64,
    pub min_y: f64,
    /// wall distance at the cell centre (m)
    pub clear: Vec<f32>,
    /// walking height at the cell centre
    pub wz: Vec<f32>,
    /// bit d set: stepping from this cell in DIRS[d] is allowed (height + wall rule)
    pub edge: Vec<u8>,
    g: Vec<f32>,
    par: Vec<u32>,
    stamp: Vec<u32>,
    done: Vec<u32>,
    gen: u32,
    heap: BinaryHeap<Node>,
    /// cells reached by the last flood, with cost
    pub reached: Vec<(u32, f32)>,
    /// connected component of each cell at base clearance (0 = not passable)
    pub comp: Vec<u32>,
    /// id of the largest component
    pub main_comp: u32,
}

impl Nav {
    pub fn new(m: &Map) -> Nav {
        let w = (m.nx + 1) / 2;
        let h = (m.ny + 1) / 2;
        let n = w * h;
        let mut clear = vec![0f32; n];
        let mut wz = vec![0f32; n];
        let mut edge = vec![0u8; n];
        for j in 0..h {
            for i in 0..w {
                let v = (2 * j).min(m.ny - 1) * m.nx + (2 * i).min(m.nx - 1);
                clear[j * w + i] = if m.wall[v] != 0 { 0.0 } else { m.wdist[v] };
                wz[j * w + i] = m.walk[v];
            }
        }
        for j in 0..h {
            for i in 0..w {
                let (vi, vj) = (2 * i as i32, 2 * j as i32);
                if vi >= m.nx as i32 || vj >= m.ny as i32 {
                    continue;
                }
                let v0 = vj as usize * m.nx + vi as usize;
                if m.wall[v0] != 0 {
                    continue;
                }
                let mut mask = 0u8;
                for (d, (dx, dy)) in DIRS.iter().enumerate() {
                    let (ti, tj) = (i as i32 + dx, j as i32 + dy);
                    if ti < 0 || tj < 0 || ti >= w as i32 || tj >= h as i32 {
                        continue;
                    }
                    let (ai, aj) = (vi + dx, vj + dy);
                    let (bi, bj) = (vi + 2 * dx, vj + 2 * dy);
                    if bi >= m.nx as i32 || bj >= m.ny as i32 {
                        continue;
                    }
                    let v1 = aj as usize * m.nx + ai as usize;
                    let v2 = bj as usize * m.nx + bi as usize;
                    if m.wall[v1] != 0 || m.wall[v2] != 0 {
                        continue;
                    }
                    if m.walk[v1] - m.walk[v0] > STEP_UP || m.walk[v2] - m.walk[v1] > STEP_UP {
                        continue;
                    }
                    mask |= 1 << d;
                }
                edge[j * w + i] = mask;
            }
        }
        Nav {
            w,
            h,
            min_x: m.min_x,
            min_y: m.min_y,
            clear,
            wz,
            edge,
            g: vec![0.0; n],
            par: vec![u32::MAX; n],
            stamp: vec![0; n],
            done: vec![0; n],
            gen: 0,
            heap: BinaryHeap::new(),
            reached: Vec::new(),
            comp: Vec::new(),
            main_comp: 0,
        }
        .with_components(3.4)
    }

    fn with_components(mut self, need: f32) -> Nav {
        let n = self.w * self.h;
        let mut comp = vec![0u32; n];
        let mut sizes: Vec<usize> = vec![0];
        let mut stack: Vec<usize> = Vec::new();
        for s in 0..n {
            if comp[s] != 0 || self.clear[s] < need {
                continue;
            }
            let id = sizes.len() as u32;
            let mut size = 0usize;
            comp[s] = id;
            stack.push(s);
            while let Some(c) = stack.pop() {
                size += 1;
                let (i, j) = ((c % self.w) as i32, (c / self.w) as i32);
                for d in 0..8 {
                    if self.edge[c] & (1 << d) == 0 {
                        continue;
                    }
                    let t = ((j + DIRS[d].1) as usize) * self.w + (i + DIRS[d].0) as usize;
                    // only count two-way links so a pit you can drop into is its own component
                    let back = self.edge[t] & (1 << ((d + 4) % 8)) != 0;
                    if comp[t] == 0 && self.clear[t] >= need && back {
                        comp[t] = id;
                        stack.push(t);
                    }
                }
            }
            sizes.push(size);
        }
        self.main_comp = (1..sizes.len()).max_by_key(|&k| sizes[k]).unwrap_or(0) as u32;
        self.comp = comp;
        self
    }

    #[inline]
    pub fn cell_of(&self, x: f64, y: f64) -> u32 {
        let i = ((x - self.min_x) / CELL).round().clamp(0.0, self.w as f64 - 1.0) as usize;
        let j = ((y - self.min_y) / CELL).round().clamp(0.0, self.h as f64 - 1.0) as usize;
        (j * self.w + i) as u32
    }
    #[inline]
    pub fn pos_of(&self, c: u32) -> (f64, f64) {
        let i = c as usize % self.w;
        let j = c as usize / self.w;
        (self.min_x + i as f64 * CELL, self.min_y + j as f64 * CELL)
    }

    /// Nearest cell with clearance >= need, searching rings up to `rings` cells out.
    pub fn snap(&self, c: u32, need: f32, rings: i32) -> Option<u32> {
        if self.clear[c as usize] >= need {
            return Some(c);
        }
        let (ci, cj) = ((c as usize % self.w) as i32, (c as usize / self.w) as i32);
        for r in 1..=rings {
            let mut best: Option<(i32, u32)> = None;
            for dj in -r..=r {
                for di in -r..=r {
                    if di.abs() != r && dj.abs() != r {
                        continue;
                    }
                    let (i, j) = (ci + di, cj + dj);
                    if i < 0 || j < 0 || i >= self.w as i32 || j >= self.h as i32 {
                        continue;
                    }
                    let k = (j as usize * self.w + i as usize) as u32;
                    if self.clear[k as usize] >= need {
                        let d2 = di * di + dj * dj;
                        if best.map_or(true, |b| d2 < b.0) {
                            best = Some((d2, k));
                        }
                    }
                }
            }
            if let Some(b) = best {
                return Some(b.1);
            }
        }
        None
    }

    fn begin(&mut self) {
        self.gen = self.gen.wrapping_add(1);
        if self.gen == 0 {
            self.stamp.iter_mut().for_each(|s| *s = 0);
            self.done.iter_mut().for_each(|s| *s = 0);
            self.gen = 1;
        }
        self.heap.clear();
    }

    #[inline]
    fn step_cost(&self, to: usize, d: usize, need: f32, extra: Option<&[f32]>) -> f32 {
        let mut c = DLEN[d];
        if self.clear[to] < need + 1.5 {
            c *= 1.6;
        }
        if let Some(e) = extra {
            c += e[to];
        }
        c
    }

    /// A* from a to b. Returns cell centres from the start cell to the goal cell.
    pub fn path(&mut self, a: (f64, f64), b: (f64, f64), need: f32, extra: Option<&[f32]>, max_expand: usize) -> Option<Vec<u32>> {
        let s0 = self.cell_of(a.0, a.1);
        let start = self.snap(s0, need, 12)?;
        let g0 = self.cell_of(b.0, b.1);
        let goal = self.snap(g0, need, 70)?;
        self.begin();
        let gen = self.gen;
        let (gi, gj) = ((goal as usize % self.w) as f32, (goal as usize / self.w) as f32);
        let w = self.w;
        let hfn = |c: u32| -> f32 {
            let dx = ((c as usize % w) as f32 - gi).abs();
            let dy = ((c as usize / w) as f32 - gj).abs();
            let (mn, mx) = if dx < dy { (dx, dy) } else { (dy, dx) };
            (mx - mn) * 2.0 + mn * 2.828
        };
        const WGT: f32 = 1.35;
        self.g[start as usize] = 0.0;
        self.par[start as usize] = u32::MAX;
        self.stamp[start as usize] = gen;
        self.heap.push(Node { f: hfn(start) * WGT, c: start });
        let mut expanded = 0usize;
        let mut found = false;
        let mut capped = false;
        let mut closest = (hfn(start), start);
        while let Some(Node { c, .. }) = self.heap.pop() {
            let ci = c as usize;
            if self.done[ci] == gen {
                continue;
            }
            self.done[ci] = gen;
            if c == goal {
                found = true;
                break;
            }
            let hc = hfn(c);
            if hc < closest.0 {
                closest = (hc, c);
            }
            expanded += 1;
            if expanded > max_expand {
                capped = true;
                break;
            }
            let mask = self.edge[ci];
            let (i, j) = ((ci % w) as i32, (ci / w) as i32);
            for d in 0..8 {
                if mask & (1 << d) == 0 {
                    continue;
                }
                let t = ((j + DIRS[d].1) as usize) * w + (i + DIRS[d].0) as usize;
                if self.clear[t] < need || self.done[t] == gen {
                    continue;
                }
                let ng = self.g[ci] + self.step_cost(t, d, need, extra);
                if self.stamp[t] != gen || ng < self.g[t] {
                    self.stamp[t] = gen;
                    self.g[t] = ng;
                    self.par[t] = c;
                    self.heap.push(Node { f: ng + hfn(t as u32) * WGT, c: t as u32 });
                }
            }
        }
        if !found && !(capped && closest.1 != start) {
            return None;
        }
        // out of budget: head for the closest point reached and replan from there
        let mut out = self.trace(if found { goal } else { closest.1 });
        if start != s0 {
            out.insert(0, s0);
        }
        Some(out)
    }

    /// A* from a to the nearest (by walking cost) cell inside the disk (centre c, radius r).
    pub fn path_to_disk(&mut self, a: (f64, f64), c: (f64, f64), r: f64, need: f32, extra: Option<&[f32]>, max_expand: usize) -> Option<Vec<u32>> {
        let s0 = self.cell_of(a.0, a.1);
        let start = self.snap(s0, need, 12)?;
        self.begin();
        let gen = self.gen;
        let w = self.w;
        let (min_x, min_y) = (self.min_x, self.min_y);
        let hfn = |k: u32| -> f32 {
            let x = min_x + (k as usize % w) as f64 * CELL;
            let y = min_y + (k as usize / w) as f64 * CELL;
            (((x - c.0).powi(2) + (y - c.1).powi(2)).sqrt() - r).max(0.0) as f32
        };
        const WGT: f32 = 1.3;
        self.g[start as usize] = 0.0;
        self.par[start as usize] = u32::MAX;
        self.stamp[start as usize] = gen;
        self.heap.push(Node { f: hfn(start) * WGT, c: start });
        let mut expanded = 0usize;
        let mut found: Option<u32> = None;
        let mut capped = false;
        let mut closest = (hfn(start), start);
        while let Some(Node { c: k, .. }) = self.heap.pop() {
            let ci = k as usize;
            if self.done[ci] == gen {
                continue;
            }
            self.done[ci] = gen;
            let hc = hfn(k);
            if hc <= 0.0 {
                found = Some(k);
                break;
            }
            if hc < closest.0 {
                closest = (hc, k);
            }
            expanded += 1;
            if expanded > max_expand {
                capped = true;
                break;
            }
            let mask = self.edge[ci];
            let (i, j) = ((ci % w) as i32, (ci / w) as i32);
            for d in 0..8 {
                if mask & (1 << d) == 0 {
                    continue;
                }
                let t = ((j + DIRS[d].1) as usize) * w + (i + DIRS[d].0) as usize;
                if self.clear[t] < need || self.done[t] == gen {
                    continue;
                }
                let ng = self.g[ci] + self.step_cost(t, d, need, extra);
                if self.stamp[t] != gen || ng < self.g[t] {
                    self.stamp[t] = gen;
                    self.g[t] = ng;
                    self.par[t] = k;
                    self.heap.push(Node { f: ng + hfn(t as u32) * WGT, c: t as u32 });
                }
            }
        }
        let end = match found {
            Some(k) => k,
            None if capped && closest.1 != start => closest.1,
            None => return None,
        };
        let mut out = self.trace(end);
        if start != s0 {
            out.insert(0, s0);
        }
        Some(out)
    }

    /// Bounded Dijkstra from a. Fills `reached` with (cell, cost) for every cell within max_cost.
    pub fn flood(&mut self, a: (f64, f64), need: f32, max_cost: f32, extra: Option<&[f32]>) -> Option<u32> {
        self.reached.clear();
        let s0 = self.cell_of(a.0, a.1);
        let start = self.snap(s0, need, 12)?;
        self.begin();
        let gen = self.gen;
        let w = self.w;
        self.g[start as usize] = 0.0;
        self.par[start as usize] = u32::MAX;
        self.stamp[start as usize] = gen;
        self.heap.push(Node { f: 0.0, c: start });
        while let Some(Node { c, f }) = self.heap.pop() {
            let ci = c as usize;
            if self.done[ci] == gen {
                continue;
            }
            self.done[ci] = gen;
            self.reached.push((c, f));
            let mask = self.edge[ci];
            let (i, j) = ((ci % w) as i32, (ci / w) as i32);
            for d in 0..8 {
                if mask & (1 << d) == 0 {
                    continue;
                }
                let t = ((j + DIRS[d].1) as usize) * w + (i + DIRS[d].0) as usize;
                if self.clear[t] < need || self.done[t] == gen {
                    continue;
                }
                let ng = f + self.step_cost(t, d, need, extra);
                if ng > max_cost {
                    continue;
                }
                if self.stamp[t] != gen || ng < self.g[t] {
                    self.stamp[t] = gen;
                    self.g[t] = ng;
                    self.par[t] = c;
                    self.heap.push(Node { f: ng, c: t as u32 });
                }
            }
        }
        Some(start)
    }

    /// Path from the last search's start to `c` (valid right after path()/flood()).
    pub fn trace(&self, c: u32) -> Vec<u32> {
        let mut out = vec![c];
        let mut cur = c;
        let mut guard = 0;
        while self.par[cur as usize] != u32::MAX && guard < 100_000 {
            cur = self.par[cur as usize];
            out.push(cur);
            guard += 1;
        }
        out.reverse();
        out
    }

    /// cost of `c` in the last flood, if it was reached
    pub fn flood_cost(&self, c: u32) -> Option<f32> {
        if self.done[c as usize] == self.gen {
            Some(self.g[c as usize])
        } else {
            None
        }
    }
}

/// Can a mech of clearance `need` walk the straight segment a→b on the ground?
pub fn free_line(m: &Map, a: (f64, f64), b: (f64, f64), need: f32) -> bool {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let d = (dx * dx + dy * dy).sqrt();
    let n = d.ceil().max(1.0) as usize;
    let mut prev = match m.vidx(a.0, a.1) {
        Some(v) => m.walk[v],
        None => return false,
    };
    for k in 1..=n {
        let t = k as f64 / n as f64;
        let v = match m.vidx(a.0 + dx * t, a.1 + dy * t) {
            Some(v) => v,
            None => return false,
        };
        if m.wall[v] != 0 || m.wdist[v] < need {
            return false;
        }
        let z = m.walk[v];
        if z - prev > STEP_UP {
            return false;
        }
        prev = z;
    }
    true
}
