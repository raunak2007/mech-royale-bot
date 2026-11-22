//! Local simulator of the mechfight/1 rules as written in protocol.md. Testing only: it is an
//! approximation of the real server, and the "house" bots are a guess at the built-in ones.

use crate::brain::{wrap, Action, Brain, Rng, GUN, LASER, LIGHTNING, MORTAR};
use crate::geom::Map;
use crate::json::{self, J};
use crate::nav::Nav;
use std::collections::{HashMap, VecDeque};
use std::f64::consts::PI;
use std::sync::Arc;

pub static HELLO: &str = include_str!("../simdata/hello.json");
pub const KINDS: [&str; 4] = ["gun", "mortar", "laser", "lightning"];
const DT: f64 = 0.05;

pub trait Ctl {
    fn act(&mut self, obs: &J) -> Action;
}
impl Ctl for Brain {
    fn act(&mut self, obs: &J) -> Action {
        self.decide(obs)
    }
}

fn r2(x: f64) -> f64 {
    (x * 100.0).round() / 100.0
}
fn r3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}
fn num(x: f64) -> J {
    J::Num(x)
}
fn jv3(a: [f64; 3]) -> J {
    J::Arr(vec![num(r2(a[0])), num(r2(a[1])), num(r2(a[2]))])
}
fn jv2(a: [f64; 2]) -> J {
    J::Arr(vec![num(r3(a[0])), num(r3(a[1]))])
}
fn js(x: &str) -> J {
    J::Str(x.to_string())
}
fn obj(v: Vec<(&str, J)>) -> J {
    J::Obj(v.into_iter().map(|(k, v)| (k.to_string(), v)).collect())
}
fn hyp(a: f64, b: f64) -> f64 {
    (a * a + b * b).sqrt()
}
fn d2(a: &[f64], b: &[f64]) -> f64 {
    hyp(a[0] - b[0], a[1] - b[1])
}
fn d3(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}
pub fn gauss(r: &mut Rng) -> f64 {
    let u1 = r.f().max(1e-12);
    let u2 = r.f();
    (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
}
fn dir3(aim: [f64; 2]) -> [f64; 3] {
    [aim[1].cos() * aim[0].cos(), aim[1].cos() * aim[0].sin(), aim[1].sin()]
}
type Stats = HashMap<String, f64>;
fn stats_of(j: &J) -> Stats {
    let mut m = Stats::new();
    if let J::Obj(v) = j {
        for (k, x) in v {
            if let Some(n) = x.num() {
                m.insert(k.clone(), n);
            }
        }
    }
    m
}
fn g(m: &Stats, k: &str) -> f64 {
    m.get(k).copied().unwrap_or(0.0)
}

#[derive(Clone, Debug)]
pub struct Weapon {
    pub kind: usize,
    pub cooldown: f64,
    pub ammo: f64,
    pub reload: f64,
    pub charge: f64,
    pub held_full: f64,
}

pub struct Mech {
    pub id: usize,
    pub alive: bool,
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub aim: [f64; 2],
    pub cmd_aim: [f64; 2],
    pub hp: f64,
    pub energy: f64,
    pub grounded: bool,
    pub cubes: f64,
    pub st: Stats,
    pub wst: [Stats; 4],
    pub upg: Vec<String>,
    pub w: [Option<Weapon>; 2],
    pub act: Action,
    pub last_combat: f64,
    pub last_spend: f64,
    pub short_until: f64,
    pub equip_until: [f64; 2],
    pub events: Vec<J>,
    pub sounds: Vec<J>,
    pub beams: Vec<J>,
    pub memory: HashMap<usize, f64>,
    pub offers: VecDeque<(u32, f64, Vec<String>)>,
    pub offered: usize,
    pub place: usize,
    pub kills: u32,
    pub death_t: f64,
    pub death_by: String,
    pub step_t: f64,
    pub dmg_dealt: f64,
    pub dmg_taken: f64,
    pub zone_dmg: f64,
    pub stuck_ticks: u32,
    /// gun rounds fired, and gun rounds that hit a mech
    pub shots: u32,
    pub hits: u32,
}
impl Mech {
    pub fn size(&self) -> f64 {
        1.0 + 0.05 * self.cubes
    }
    pub fn radius(&self) -> f64 {
        g(&self.st, "radius") * self.size()
    }
    pub fn height(&self) -> f64 {
        g(&self.st, "height") * self.size()
    }
    pub fn max_hp(&self) -> f64 {
        g(&self.st, "hp") * (1.0 + 0.04 * self.cubes.min(10.0))
    }
    pub fn eye(&self) -> [f64; 3] {
        [self.pos[0], self.pos[1], self.pos[2] + g(&self.st, "eye_height") * self.size()]
    }
    pub fn center(&self) -> [f64; 3] {
        [self.pos[0], self.pos[1], self.pos[2] + self.height() * 0.5]
    }
    pub fn top(&self) -> [f64; 3] {
        [self.pos[0], self.pos[1], self.pos[2] + self.height()]
    }
    pub fn muzzle(&self) -> [f64; 3] {
        [self.pos[0], self.pos[1], self.pos[2] + g(&self.st, "muzzle_height") * self.size()]
    }
    fn has(&self, kind: usize) -> bool {
        self.w.iter().any(|w| w.as_ref().map_or(false, |w| w.kind == kind))
    }
    fn dmg_mult(&self) -> f64 {
        1.0 + 0.04 * self.cubes.min(10.0)
    }
}

pub struct Npc {
    pub id: usize,
    pub alive: bool,
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub aim: [f64; 2],
    pub hp: f64,
    pub home: [f64; 2],
    pub target: Option<usize>,
    pub cooldown: f64,
    pub goal: [f64; 2],
}
pub struct CrateE {
    pub id: usize,
    pub pos: [f64; 3],
    pub hp: f64,
}
pub struct CubeE {
    pub id: usize,
    pub pos: [f64; 3],
}
pub struct PickE {
    pub id: usize,
    pub kind: usize,
    pub pos: [f64; 3],
}
pub struct Proj {
    pub id: usize,
    pub kind: usize, // 0 gun, 1 mortar, 9 npc
    pub owner: usize,
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub left: f64,
    pub dmg: f64,
    pub from: [f64; 3],
}
pub struct DropE {
    pub id: usize,
    pub pos: [f64; 3],
    pub lands_at: f64,
    pub progress: f64,
    pub holder: Option<usize>,
}

enum Shot {
    Bullet(usize, usize),
    Laser(usize, usize, f64),
    Zap(usize, usize),
    Shell(usize, usize),
}

#[derive(Clone, Copy, PartialEq)]
enum Tgt {
    Mech(usize),
    Npc(usize),
    Crate(usize),
}

pub struct Sim {
    pub map: Arc<Map>,
    pub hello: J,
    pub rng: Rng,
    pub t: f64,
    pub tick: u64,
    pub mechs: Vec<Mech>,
    pub npcs: Vec<Npc>,
    pub crates: Vec<CrateE>,
    pub cubes: Vec<CubeE>,
    pub pickups: Vec<PickE>,
    pub projs: Vec<Proj>,
    pub drops: Vec<DropE>,
    phases: Vec<[f64; 4]>,
    pub zc: [f64; 2],
    pub zr: f64,
    pub zc0: [f64; 2],
    pub zr0: f64,
    pub znc: [f64; 2],
    pub znr: f64,
    pub phase: usize,
    pub phase_t: f64,
    pub zdps: f64,
    next_id: usize,
    pub alive: usize,
    open: Vec<[f64; 3]>,
    roofs: Vec<[f64; 3]>,
    drop_times: Vec<f64>,
    pub death_log: Vec<(f64, usize, String)>,
    pub verbose: bool,
    /// gun shots fired / hit on mechs, by range bucket of 50 m; laser fired / hit
    pub gun_fired: [u32; 6],
    pub gun_hit: [u32; 6],
    pub laser_fired: u32,
    pub laser_hit: u32,
}

impl Sim {
    pub fn new(map: Arc<Map>, seed: u64, n_mechs: usize) -> Sim {
        let hello = json::parse(HELLO).expect("hello");
        let cfg = hello.get("config").clone();
        let mut rng = Rng(seed.wrapping_mul(0x9E3779B97F4A7C15) ^ 0xD1B54A32D192ED03 | 1);
        for _ in 0..8 {
            rng.next();
        }
        let nav = Nav::new(&map);
        let mut open = Vec::new();
        for c in 0..nav.w * nav.h {
            if nav.clear[c] >= 4.0 && nav.comp[c] == nav.main_comp {
                let (x, y) = nav.pos_of(c as u32);
                if x > map.bx0 + 30.0 && x < map.bx1 - 30.0 && y > map.by0 + 30.0 && y < map.by1 - 30.0 {
                    open.push([x, y, nav.wz[c] as f64]);
                }
            }
        }
        let mut roofs = Vec::new();
        for j in (4..map.ny - 4).step_by(6) {
            for i in (4..map.nx - 4).step_by(6) {
                let v = j * map.nx + i;
                if map.wall[v] != 0 && map.top[v] > map.terr[v] + 3.0 && map.top[v] < 60.0 && map.wdist[v] == 0.0 {
                    // interior roof point: all neighbours at the same height
                    let same = [v - 3, v + 3, v - 3 * map.nx, v + 3 * map.nx].iter().all(|&u| (map.top[u] - map.top[v]).abs() < 0.1);
                    if same {
                        roofs.push([map.min_x + i as f64, map.min_y + j as f64, map.top[v] as f64]);
                    }
                }
            }
        }
        let z = cfg.get("zone");
        let zc = z.get("start_center").v2();
        let zr = z.get("start_radius").fo(560.0);
        let phases: Vec<[f64; 4]> = z.get("phases").arr().iter().map(|p| [p.get("hold_s").f(), p.get("shrink_s").f(), p.get("radius").f(), p.get("dps").f()]).collect();
        let mut sim = Sim {
            map,
            hello,
            rng,
            t: 0.0,
            tick: 0,
            mechs: Vec::new(),
            npcs: Vec::new(),
            crates: Vec::new(),
            cubes: Vec::new(),
            pickups: Vec::new(),
            projs: Vec::new(),
            drops: Vec::new(),
            phases,
            zc,
            zr,
            zc0: zc,
            zr0: zr,
            znc: zc,
            znr: zr,
            phase: 0,
            phase_t: 0.0,
            zdps: 0.0,
            next_id: 100000,
            alive: n_mechs,
            open,
            roofs,
            drop_times: cfg.get("drops").get("times").arr().iter().map(|x| x.f()).collect(),
            death_log: Vec::new(),
            verbose: false,
            gun_fired: [0; 6],
            gun_hit: [0; 6],
            laser_fired: 0,
            laser_hit: 0,
        };
        sim.pick_next_circle();
        // spawns
        let frac = cfg.get("match").get("spawn_zone_fraction").fo(0.8);
        let sep = cfg.get("match").get("spawn_min_separation").fo(130.0);
        let mut spawns: Vec<[f64; 3]> = Vec::new();
        let mut tries = 0;
        while spawns.len() < n_mechs && tries < 200000 {
            tries += 1;
            let p = sim.open[(sim.rng.f() * sim.open.len() as f64) as usize % sim.open.len()];
            if d2(&p, &zc) > zr * frac {
                continue;
            }
            let min_sep = if tries > 100000 { sep * 0.6 } else { sep };
            if spawns.iter().any(|q| d2(q, &p) < min_sep) {
                continue;
            }
            spawns.push(p);
        }
        let mst = stats_of(cfg.get("mech"));
        let wst = [stats_of(cfg.get("weapons").get("gun")), stats_of(cfg.get("weapons").get("mortar")), stats_of(cfg.get("weapons").get("laser")), stats_of(cfg.get("weapons").get("lightning"))];
        for (i, p) in spawns.iter().enumerate() {
            let yaw = sim.rng.range(-PI, PI);
            sim.mechs.push(Mech {
                id: i,
                alive: true,
                pos: *p,
                vel: [0.0; 3],
                aim: [yaw, 0.0],
                cmd_aim: [yaw, 0.0],
                hp: g(&mst, "hp"),
                energy: g(&mst, "energy"),
                grounded: true,
                cubes: 0.0,
                st: mst.clone(),
                wst: wst.clone(),
                upg: Vec::new(),
                w: [Some(Weapon { kind: GUN, cooldown: 0.0, ammo: g(&wst[GUN], "mag"), reload: 0.0, charge: 0.0, held_full: 0.0 }), None],
                act: Action::default(),
                last_combat: -100.0,
                last_spend: -100.0,
                short_until: -1.0,
                equip_until: [0.0; 2],
                events: Vec::new(),
                sounds: Vec::new(),
                beams: Vec::new(),
                memory: HashMap::new(),
                offers: VecDeque::new(),
                offered: 0,
                place: 0,
                kills: 0,
                death_t: -1.0,
                death_by: String::new(),
                step_t: 0.0,
                dmg_dealt: 0.0,
                dmg_taken: 0.0,
                zone_dmg: 0.0,
                stuck_ticks: 0,
                shots: 0,
                hits: 0,
            });
        }
        // crates, loot, drones
        let n_crates = cfg.get("crates").get("count").fo(36.0) as usize;
        let roof_frac = cfg.get("crates").get("roof_fraction").fo(0.25);
        for _ in 0..n_crates {
            let p = sim.rand_spot(roof_frac);
            let id = 1000 + sim.crates.len();
            sim.crates.push(CrateE { id, pos: p, hp: cfg.get("crates").get("hp").fo(300.0) });
        }
        let n_loot = cfg.get("loot").get("floor_count").fo(40.0) as usize;
        let lroof = cfg.get("loot").get("floor_roof_fraction").fo(0.25);
        for k in 0..n_loot {
            let p = sim.rand_spot(lroof);
            let kind = 1 + (sim.rng.f() * 3.0) as usize % 3;
            sim.pickups.push(PickE { id: 8000 + k, kind, pos: p });
        }
        let n_npc = cfg.get("npc").get("count").fo(12.0) as usize;
        for k in 0..n_npc {
            let p = sim.rand_spot(0.0);
            sim.npcs.push(Npc { id: 100 + k, alive: true, pos: p, vel: [0.0; 3], aim: [0.0, 0.0], hp: cfg.get("npc").get("hp").fo(350.0), home: [p[0], p[1]], target: None, cooldown: 0.0, goal: [p[0], p[1]] });
        }
        sim
    }

    fn rand_spot(&mut self, roof_frac: f64) -> [f64; 3] {
        for _ in 0..1000 {
            let roof = self.rng.f() < roof_frac && !self.roofs.is_empty();
            let p = if roof { self.roofs[(self.rng.f() * self.roofs.len() as f64) as usize % self.roofs.len()] } else { self.open[(self.rng.f() * self.open.len() as f64) as usize % self.open.len()] };
            if d2(&p, &self.zc0) <= self.zr0 * 0.95 {
                return p;
            }
        }
        self.open[0]
    }

    fn cfgf(&self, a: &str, b: &str, d: f64) -> f64 {
        self.hello.get("config").get(a).get(b).fo(d)
    }

    fn pick_next_circle(&mut self) {
        if self.phase >= self.phases.len() {
            self.znc = self.zc;
            self.znr = 0.0;
            return;
        }
        let nr = self.phases[self.phase][2];
        let slack = (self.zr - nr).max(0.0);
        for _ in 0..200 {
            let a = self.rng.range(-PI, PI);
            let d = slack * self.rng.f().sqrt();
            let c = [self.zc[0] + a.cos() * d, self.zc[1] + a.sin() * d];
            let inside = c[0] > self.map.bx0 + 40.0 && c[0] < self.map.bx1 - 40.0 && c[1] > self.map.by0 + 40.0 && c[1] < self.map.by1 - 40.0;
            let walk = self.map.wall_dist(c[0], c[1]) >= 4.0;
            if inside && (walk || nr > 30.0) {
                self.znc = c;
                self.znr = nr;
                return;
            }
        }
        self.znc = self.zc;
        self.znr = nr;
    }

    fn zone_times(&self) -> (f64, f64) {
        if self.phase >= self.phases.len() {
            return (self.phase_t, self.phase_t);
        }
        let p = self.phases[self.phase];
        (self.phase_t + p[0], self.phase_t + p[0] + p[1])
    }

    fn step_zone(&mut self) {
        if self.phase >= self.phases.len() {
            self.zr = 0.0;
            return;
        }
        let (t0, t1) = self.zone_times();
        self.zdps = self.phases[self.phase][3];
        if self.t >= t1 {
            self.zc = self.znc;
            self.zr = self.znr;
            self.phase += 1;
            self.phase_t = t1;
            self.zc0 = self.zc;
            self.zr0 = self.zr;
            self.pick_next_circle();
        } else if self.t > t0 {
            let f = (self.t - t0) / (t1 - t0);
            self.zc = [self.zc0[0] + (self.znc[0] - self.zc0[0]) * f, self.zc0[1] + (self.znc[1] - self.zc0[1]) * f];
            self.zr = self.zr0 + (self.znr - self.zr0) * f;
        }
    }

    // ------------------------------------------------------------------ observation

    fn can_see(&self, m: &Mech, center: [f64; 3], top: [f64; 3]) -> bool {
        let d = d3(&m.eye(), &center);
        let near = self.cfgf("vision", "near_range", 45.0) + g(&m.st, "near_range_bonus");
        let range = self.cfgf("vision", "range", 200.0) + g(&m.st, "vision_bonus");
        if d > range {
            return false;
        }
        if d > near {
            let b = (center[1] - m.pos[1]).atan2(center[0] - m.pos[0]);
            if wrap(b - m.aim[0]).abs() > self.cfgf("vision", "fov_deg", 120.0).to_radians() * 0.5 {
                return false;
            }
        }
        let eye = m.eye();
        self.map.los_vis(eye, center) || self.map.los_vis(eye, top)
    }

    pub fn obs_for(&mut self, i: usize) -> J {
        let t = self.t;
        let memory_s = self.cfgf("vision", "memory_s", 5.0);
        let mut seen_now: Vec<usize> = Vec::new();
        let mut mechs_j = Vec::new();
        {
            let m = &self.mechs[i];
            for o in &self.mechs {
                if o.id == i || !o.alive {
                    continue;
                }
                let vis = self.can_see(m, o.center(), o.top());
                if vis {
                    seen_now.push(o.id);
                }
                if vis || m.memory.get(&o.id).map_or(false, |&ts| t - ts <= memory_s) {
                    let weapons: Vec<J> = o.w.iter().flatten().map(|w| js(KINDS[w.kind])).collect();
                    let charging = o.w.iter().flatten().any(|w| w.kind == LASER && w.charge > 0.0);
                    mechs_j.push(obj(vec![
                        ("id", num(o.id as f64)),
                        ("kind", js("mech")),
                        ("pos", jv3(o.pos)),
                        ("vel", jv3(o.vel)),
                        ("aim", jv2(o.aim)),
                        ("hp", num(r2(o.hp))),
                        ("max_hp", num(r2(o.max_hp()))),
                        ("cubes", num(o.cubes)),
                        ("weapons", J::Arr(weapons)),
                        ("charging", J::Bool(charging)),
                    ]));
                }
            }
            for o in &self.npcs {
                if !o.alive {
                    continue;
                }
                let c = [o.pos[0], o.pos[1], o.pos[2] + 1.75];
                let vis = self.can_see(m, c, [o.pos[0], o.pos[1], o.pos[2] + 3.5]);
                if vis {
                    seen_now.push(o.id);
                }
                if vis || m.memory.get(&o.id).map_or(false, |&ts| t - ts <= memory_s) {
                    mechs_j.push(obj(vec![("id", num(o.id as f64)), ("kind", js("npc")), ("pos", jv3(o.pos)), ("vel", jv3(o.vel)), ("aim", jv2(o.aim)), ("hp", num(o.hp)), ("max_hp", num(350.0))]));
                }
            }
        }
        for id in seen_now {
            self.mechs[i].memory.insert(id, t);
        }
        let m = &self.mechs[i];
        let item_vis = |p: [f64; 3], h: f64| -> bool { self.can_see(m, [p[0], p[1], p[2] + h * 0.5], [p[0], p[1], p[2] + h]) };
        let crates_j: Vec<J> = self.crates.iter().filter(|c| item_vis(c.pos, 2.4)).map(|c| obj(vec![("id", num(c.id as f64)), ("pos", jv3(c.pos)), ("hp", num(c.hp))])).collect();
        let cubes_j: Vec<J> = self.cubes.iter().filter(|c| item_vis(c.pos, 1.0)).map(|c| obj(vec![("id", num(c.id as f64)), ("pos", jv3(c.pos))])).collect();
        let picks_j: Vec<J> = self.pickups.iter().filter(|c| item_vis(c.pos, 1.0)).map(|c| obj(vec![("id", num(c.id as f64)), ("kind", js(KINDS[c.kind])), ("pos", jv3(c.pos))])).collect();
        let projs_j: Vec<J> = self
            .projs
            .iter()
            .filter(|p| p.owner != i && item_vis(p.pos, 0.2))
            .map(|p| obj(vec![("id", num(p.id as f64)), ("kind", js(if p.kind == 0 { "gun" } else if p.kind == 1 { "mortar" } else { "npc" })), ("pos", jv3(p.pos)), ("vel", jv3(p.vel))]))
            .collect();
        let drops_j: Vec<J> = self
            .drops
            .iter()
            .map(|d| {
                obj(vec![
                    ("id", num(d.id as f64)),
                    ("pos", jv3(d.pos)),
                    ("radius", num(self.cfgf("drops", "radius", 8.0))),
                    ("lands_in", num(r2((d.lands_at - t).max(0.0)))),
                    ("capture_s", num(self.cfgf("drops", "capture_s", 4.0))),
                    ("progress", num(r2(d.progress))),
                    ("holder", d.holder.map_or(J::Null, |h| num(h as f64))),
                ])
            })
            .collect();
        let sz = m.size();
        let mut st: Vec<(String, J)> = m.st.iter().map(|(k, v)| (k.clone(), num(*v))).collect();
        for (k, v) in st.iter_mut() {
            if matches!(k.as_str(), "radius" | "height" | "eye_height" | "muzzle_height") {
                *v = num(r3(v.f() * sz));
            }
        }
        let weapons_j: Vec<J> = m
            .w
            .iter()
            .map(|w| match w {
                None => J::Null,
                Some(w) => {
                    let mut ws: Vec<(String, J)> = m.wst[w.kind].iter().map(|(k, v)| (k.clone(), num(*v))).collect();
                    if w.kind != LIGHTNING && w.kind != LASER {
                        // damage after cube scaling is not shown by the server; keep raw
                    }
                    ws.sort_by(|a, b| a.0.cmp(&b.0));
                    let mut o = vec![("kind", js(KINDS[w.kind])), ("cooldown", num(r2(w.cooldown))), ("stats", J::Obj(ws))];
                    if w.kind == GUN {
                        o.push(("ammo", num(w.ammo)));
                        o.push(("reload", num(r2(w.reload))));
                    }
                    if w.kind == LASER {
                        o.push(("charge", num(r2(w.charge))));
                    }
                    obj(o)
                }
            })
            .collect();
        let (t0, t1) = self.zone_times();
        let in_zone = d2(&m.pos, &self.zc) <= self.zr;
        let offer = match m.offers.front() {
            None => J::Null,
            Some((id, exp, ch)) => obj(vec![
                ("id", num(*id as f64)),
                ("expires", num(r2(*exp))),
                ("choices", J::Arr(ch.iter().map(|c| obj(vec![("id", js(c)), ("name", js(c)), ("desc", js(""))])).collect())),
            ]),
        };
        let o = obj(vec![
            ("type", js("obs")),
            ("tick", num(self.tick as f64)),
            ("t", num(r2(t))),
            ("alive", num(self.alive as f64)),
            (
                "self",
                obj(vec![
                    ("id", num(i as f64)),
                    ("pos", jv3(m.pos)),
                    ("vel", jv3(m.vel)),
                    ("aim", jv2(m.aim)),
                    ("hp", num(r2(m.hp))),
                    ("max_hp", num(r2(m.max_hp()))),
                    ("energy", num(r2(m.energy))),
                    ("max_energy", num(g(&m.st, "energy"))),
                    ("grounded", J::Bool(m.grounded)),
                    ("cubes", num(m.cubes)),
                    ("upgrades", J::Arr(m.upg.iter().map(|u| js(u)).collect())),
                    ("in_zone", J::Bool(in_zone)),
                    ("weapons", J::Arr(weapons_j)),
                    ("stats", J::Obj(st)),
                ]),
            ),
            ("mechs", J::Arr(mechs_j)),
            ("projectiles", J::Arr(projs_j)),
            ("crates", J::Arr(crates_j)),
            ("cubes", J::Arr(cubes_j)),
            ("pickups", J::Arr(picks_j)),
            ("sounds", J::Arr(m.sounds.clone())),
            ("beams", J::Arr(m.beams.clone())),
            ("events", J::Arr(m.events.clone())),
            ("drops", J::Arr(drops_j)),
            (
                "zone",
                obj(vec![
                    ("center", J::Arr(vec![num(r2(self.zc[0])), num(r2(self.zc[1]))])),
                    ("radius", num(r2(self.zr))),
                    ("next_center", J::Arr(vec![num(r2(self.znc[0])), num(r2(self.znc[1]))])),
                    ("next_radius", num(self.znr)),
                    ("shrink_start", num(t0)),
                    ("shrink_end", num(t1)),
                    ("dps", num(self.zdps)),
                ]),
            ),
            ("offer", offer),
        ]);
        let m = &mut self.mechs[i];
        m.sounds.clear();
        m.beams.clear();
        m.events.clear();
        o
    }

    // ------------------------------------------------------------------ helpers

    fn sound(&mut self, kind: &str, at: [f64; 3], by: Option<usize>) {
        let range = self.hello.get("config").get("sound").get("range").get(kind).fo(0.0);
        let near = self.cfgf("sound", "near", 60.0);
        let mid = self.cfgf("sound", "mid", 160.0);
        let noise = self.cfgf("sound", "bearing_noise_deg", 7.0).to_radians();
        for k in 0..self.mechs.len() {
            if !self.mechs[k].alive || Some(k) == by {
                continue;
            }
            let d = d3(&self.mechs[k].pos, &at);
            if d > range {
                continue;
            }
            let e = gauss(&mut self.rng) * noise * g(&self.mechs[k].st, "hearing").max(0.1);
            let m = &mut self.mechs[k];
            let b = wrap((at[1] - m.pos[1]).atan2(at[0] - m.pos[0]) + e);
            m.sounds.push(obj(vec![("kind", js(kind)), ("bearing", num(r3(b))), ("dist", js(if d < near { "near" } else if d < mid { "mid" } else { "far" })), ("age", num(0.05))]));
        }
    }

    fn gain_cubes(&mut self, i: usize, n: f64) {
        let thr: Vec<f64> = self.hello.get("config").get("cubes").get("upgrade_thresholds").arr().iter().map(|x| x.f()).collect();
        let before = self.mechs[i].max_hp();
        self.mechs[i].cubes += n;
        let after = self.mechs[i].max_hp();
        self.mechs[i].hp += (after - before).max(0.0);
        loop {
            let k = self.mechs[i].offered;
            if k < thr.len() && self.mechs[i].cubes >= thr[k] {
                self.mechs[i].offered += 1;
                // three distinct upgrades the mech can still take
                let mut pool: Vec<String> = Vec::new();
                for u in self.hello.get("config").get("upgrades").arr() {
                    let id = u.get("id").s();
                    let have = self.mechs[i].upg.iter().filter(|x| x.as_str() == id).count() as f64;
                    if have >= u.get("max").fo(1.0) {
                        continue;
                    }
                    let req = u.get("requires").s();
                    if !req.is_empty() {
                        let kind = KINDS.iter().position(|x| *x == req).unwrap_or(0);
                        if !self.mechs[i].has(kind) {
                            continue;
                        }
                    }
                    pool.push(id.to_string());
                }
                let mut ch = Vec::new();
                while ch.len() < 3 && !pool.is_empty() {
                    let j = (self.rng.f() * pool.len() as f64) as usize % pool.len();
                    ch.push(pool.swap_remove(j));
                }
                let id = self.mechs[i].offered as u32;
                self.mechs[i].offers.push_back((id, -1.0, ch));
            } else {
                break;
            }
        }
    }

    fn apply_upgrade(&mut self, i: usize, id: &str) {
        let ups = self.hello.get("config").get("upgrades").arr().to_vec();
        for u in &ups {
            if u.get("id").s() != id {
                continue;
            }
            let before = self.mechs[i].max_hp();
            for e in u.get("effects").arr() {
                let path = e.get("path").s();
                let parts: Vec<&str> = path.split('.').collect();
                let m = &mut self.mechs[i];
                let tgt: Option<&mut f64> = if parts.len() == 2 && parts[0] == "mech" {
                    Some(m.st.entry(parts[1].to_string()).or_insert(0.0))
                } else if parts.len() == 3 && parts[0] == "weapons" {
                    let kind = KINDS.iter().position(|x| *x == parts[1]).unwrap_or(0);
                    Some(m.wst[kind].entry(parts[2].to_string()).or_insert(0.0))
                } else {
                    None
                };
                if let Some(x) = tgt {
                    if let Some(mul) = e.get("mul").num() {
                        *x *= mul;
                    }
                    if let Some(add) = e.get("add").num() {
                        *x += add;
                    }
                }
            }
            let after = self.mechs[i].max_hp();
            let m = &mut self.mechs[i];
            m.hp += (after - before).max(0.0);
            m.upg.push(id.to_string());
            m.events.push(obj(vec![("type", js("upgrade")), ("id", js(id))]));
        }
    }

    fn hurt_mech(&mut self, k: usize, base: f64, weapon: &str, by: Option<usize>, from: [f64; 3]) {
        if !self.mechs[k].alive {
            return;
        }
        let t = self.t;
        let (amount, rear, bearing) = {
            let m = &self.mechs[k];
            let bearing = (from[1] - m.pos[1]).atan2(from[0] - m.pos[0]);
            let rear = wrap(bearing - m.aim[0]).abs() > self.cfgf("mech", "front_arc_deg", 180.0).to_radians() * 0.5;
            let mut a = base * (1.0 - g(&m.st, "armor")).max(0.1);
            if rear {
                a *= self.cfgf("mech", "rear_mult", 1.5);
            }
            (a, rear, bearing)
        };
        let self_hit = by == Some(k);
        {
            let m = &mut self.mechs[k];
            m.hp -= amount;
            m.dmg_taken += amount;
            m.last_combat = t;
            let mut ev = vec![("type", js("damaged")), ("amount", num(r2(amount))), ("weapon", js(weapon)), ("bearing", num(r3(bearing))), ("by", by.map_or(J::Null, |b| num(b as f64)))];
            if rear {
                ev.push(("rear", J::Bool(true)));
            }
            m.events.push(obj(ev));
        }
        if let Some(b) = by {
            if b < self.mechs.len() && !self_hit {
                let s = &mut self.mechs[b];
                s.last_combat = t;
                s.dmg_dealt += amount;
                let mut ev = vec![("type", js("hit")), ("target", num(k as f64)), ("amount", num(r2(amount))), ("weapon", js(weapon))];
                if rear {
                    ev.push(("rear", J::Bool(true)));
                }
                s.events.push(obj(ev));
            }
        }
        if self.mechs[k].hp <= 0.0 {
            self.mechs[k].death_by = match by {
                Some(b) if b < 100 => format!("{weapon} by {b}"),
                _ => weapon.to_string(),
            };
            if let Some(b) = by {
                if b < self.mechs.len() && b != k {
                    self.mechs[b].kills += 1;
                    self.mechs[b].events.push(obj(vec![("type", js("kill")), ("target", num(k as f64))]));
                }
            }
        }
    }

    fn hurt(&mut self, tg: Tgt, base: f64, weapon: &str, by: Option<usize>, from: [f64; 3]) {
        match tg {
            Tgt::Mech(k) => self.hurt_mech(k, base, weapon, by, from),
            Tgt::Npc(k) => {
                let n = &mut self.npcs[k];
                if !n.alive {
                    return;
                }
                n.hp -= base;
                if let Some(b) = by {
                    if b < 100 {
                        n.target = Some(b);
                        if b < self.mechs.len() {
                            self.mechs[b].last_combat = self.t;
                            self.mechs[b].events.push(obj(vec![("type", js("hit")), ("target", num(n.id as f64)), ("amount", num(base)), ("weapon", js(weapon))]));
                        }
                    }
                }
                if n.hp <= 0.0 {
                    n.alive = false;
                    let p = n.pos;
                    for q in 0..2 {
                        let id = self.next_id;
                        self.next_id += 1;
                        self.cubes.push(CubeE { id: 5000 + id, pos: [p[0] + q as f64 * 1.5, p[1], p[2]] });
                    }
                }
            }
            Tgt::Crate(k) => {
                let c = &mut self.crates[k];
                c.hp -= base;
                if let Some(b) = by {
                    if b < self.mechs.len() {
                        self.mechs[b].events.push(obj(vec![("type", js("hit")), ("target", num(c.id as f64)), ("amount", num(base)), ("weapon", js(weapon))]));
                    }
                }
            }
        }
    }

    fn tgt_cyl(&self, tg: Tgt) -> ([f64; 3], f64, f64) {
        match tg {
            Tgt::Mech(k) => (self.mechs[k].pos, self.mechs[k].radius(), self.mechs[k].height()),
            Tgt::Npc(k) => (self.npcs[k].pos, 1.8, 3.5),
            Tgt::Crate(k) => (self.crates[k].pos, 1.4, 2.4),
        }
    }

    fn targets(&self, except: Option<usize>) -> Vec<Tgt> {
        let mut v = Vec::new();
        for m in &self.mechs {
            if m.alive && Some(m.id) != except {
                v.push(Tgt::Mech(m.id));
            }
        }
        for (k, n) in self.npcs.iter().enumerate() {
            if n.alive {
                v.push(Tgt::Npc(k));
            }
        }
        for (k, c) in self.crates.iter().enumerate() {
            if c.hp > 0.0 {
                v.push(Tgt::Crate(k));
            }
        }
        v
    }

    /// First cylinder hit by the segment a→b (fraction, target).
    fn seg_hit(&self, a: [f64; 3], b: [f64; 3], pad: f64, except: Option<usize>, npc_except: Option<usize>) -> Option<(f64, Tgt)> {
        let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let aa = d[0] * d[0] + d[1] * d[1];
        let mut best: Option<(f64, Tgt)> = None;
        for tg in self.targets(except) {
            if let (Tgt::Npc(k), Some(ne)) = (tg, npc_except) {
                if k == ne {
                    continue;
                }
            }
            let (c, r, h) = self.tgt_cyl(tg);
            let rr = r + pad;
            let f = [a[0] - c[0], a[1] - c[1]];
            if (f[0] * f[0] + f[1] * f[1]).sqrt() > (aa.sqrt() + rr + 1.0) {
                continue;
            }
            let bb = 2.0 * (f[0] * d[0] + f[1] * d[1]);
            let cc = f[0] * f[0] + f[1] * f[1] - rr * rr;
            let s = if cc <= 0.0 {
                0.0
            } else {
                if aa < 1e-12 {
                    continue;
                }
                let disc = bb * bb - 4.0 * aa * cc;
                if disc < 0.0 {
                    continue;
                }
                let s = (-bb - disc.sqrt()) / (2.0 * aa);
                if !(0.0..=1.0).contains(&s) {
                    continue;
                }
                s
            };
            // z test at entry and at the point of closest approach inside the circle
            let s2 = if aa > 1e-12 { (-bb / (2.0 * aa)).clamp(s, 1.0) } else { s };
            let z1 = a[2] + d[2] * s;
            let z2 = a[2] + d[2] * s2;
            let (lo, hi) = (z1.min(z2), z1.max(z2));
            if hi < c[2] - pad || lo > c[2] + h + pad {
                continue;
            }
            if best.map_or(true, |x| s < x.0) {
                best = Some((s, tg));
            }
        }
        best
    }

    fn drop_weapon_at(&mut self, kind: usize, pos: [f64; 3]) -> usize {
        let id = 8000 + self.next_id;
        self.next_id += 1;
        self.pickups.push(PickE { id, kind, pos });
        id
    }

    fn explode(&mut self, p: [f64; 3], owner: usize, direct: Option<Tgt>) {
        self.sound("explosion", p, None);
        let (dmg, splash, radius, edge, knock, knock_up, drain, mult) = {
            let o = &self.mechs[owner];
            let w = &o.wst[MORTAR];
            (g(w, "damage"), g(w, "splash"), g(w, "splash_radius"), g(w, "splash_edge"), g(w, "knock"), g(w, "knock_up"), g(w, "drain"), o.dmg_mult())
        };
        let src = [p[0], p[1], p[2] + 0.6];
        for tg in self.targets(None) {
            let (c, r, h) = self.tgt_cyl(tg);
            let zc = p[2].clamp(c[2], c[2] + h);
            let dh = (d2(&c, &p) - r).max(0.0);
            let d = hyp(dh, p[2] - zc);
            let is_direct = direct == Some(tg);
            if d > radius && !is_direct {
                continue;
            }
            let center = [c[0], c[1], c[2] + h * 0.5];
            if !is_direct && !self.map.los(src, center) {
                continue;
            }
            let fall = 1.0 - (1.0 - edge) * (d / radius).min(1.0);
            let mut amount = splash * fall * mult;
            if is_direct {
                amount += dmg * mult;
            }
            if let Tgt::Mech(k) = tg {
                if k == owner {
                    amount *= self.cfgf("mech", "self_damage_mult", 0.5);
                }
                let m = &mut self.mechs[k];
                let (dx, dy) = (m.pos[0] - p[0], m.pos[1] - p[1]);
                let l = hyp(dx, dy);
                let (ux, uy) = if l > 0.1 { (dx / l, dy / l) } else { (1.0, 0.0) };
                m.vel[0] += ux * knock * fall;
                m.vel[1] += uy * knock * fall;
                m.vel[2] += knock_up * fall;
                m.grounded = false;
                m.energy = (m.energy - drain * fall).max(0.0);
                m.events.push(obj(vec![("type", js("effect")), ("effect", js("concuss")), ("by", num(owner as f64))]));
            }
            self.hurt(tg, amount, "mortar", Some(owner), src);
        }
    }

    // ------------------------------------------------------------------ one tick

    pub fn apply_action(&mut self, i: usize, a: Action) {
        let t = self.t;
        // upgrade pick
        if let Some(p) = a.pick {
            if let Some((_, _, ch)) = self.mechs[i].offers.front().cloned() {
                if let Some(id) = ch.get(p).or(ch.first()) {
                    self.mechs[i].offers.pop_front();
                    let id = id.clone();
                    self.apply_upgrade(i, &id);
                }
            }
        }
        // take / drop
        if let Some(id) = a.take {
            let id = id as usize;
            let reason: Option<&str> = match self.pickups.iter().position(|p| p.id == id) {
                None => Some("gone"),
                Some(pi) => {
                    let pk = (self.pickups[pi].kind, self.pickups[pi].pos);
                    let m = &self.mechs[i];
                    if d2(&pk.1, &m.pos) > self.cfgf("loot", "pickup_radius", 4.0) || (pk.1[2] - m.pos[2]).abs() > 4.0 {
                        Some("range")
                    } else if m.has(pk.0) {
                        Some("have")
                    } else {
                        let empty = m.w.iter().position(|w| w.is_none());
                        let slot = empty.or(a.drop.filter(|&s| s < 2));
                        match slot {
                            None => Some("full"),
                            Some(s) => {
                                let busy = m.w[s].as_ref().map_or(false, |w| w.kind == LASER && w.charge > 0.0);
                                if busy {
                                    Some("busy")
                                } else {
                                    let pos = m.pos;
                                    if let Some(old) = self.mechs[i].w[s].take() {
                                        let nid = self.drop_weapon_at(old.kind, pos);
                                        self.mechs[i].events.push(obj(vec![("type", js("dropped")), ("id", num(nid as f64)), ("kind", js(KINDS[old.kind])), ("slot", num(s as f64))]));
                                    }
                                    let pi = self.pickups.iter().position(|p| p.id == id).unwrap();
                                    self.pickups.swap_remove(pi);
                                    let m = &mut self.mechs[i];
                                    let cd = g(&m.wst[pk.0], "cooldown");
                                    m.w[s] = Some(Weapon { kind: pk.0, cooldown: 0.0, ammo: g(&m.wst[GUN], "mag"), reload: 0.0, charge: 0.0, held_full: 0.0 });
                                    m.equip_until[s] = t + self.hello.get("config").get("loot").get("equip_s").fo(1.0).max(cd);
                                    m.events.push(obj(vec![("type", js("took")), ("id", num(id as f64)), ("kind", js(KINDS[pk.0])), ("slot", num(s as f64))]));
                                    None
                                }
                            }
                        }
                    }
                }
            };
            if let Some(r) = reason {
                self.mechs[i].events.push(obj(vec![("type", js("take_failed")), ("id", num(id as f64)), ("reason", js(r))]));
            }
        } else if let Some(s) = a.drop {
            if s < 2 {
                let busy = self.mechs[i].w[s].as_ref().map_or(false, |w| w.kind == LASER && w.charge > 0.0);
                if !busy {
                    let pos = self.mechs[i].pos;
                    if let Some(old) = self.mechs[i].w[s].take() {
                        self.drop_weapon_at(old.kind, pos);
                    }
                }
            }
        }
        let m = &mut self.mechs[i];
        m.cmd_aim = [wrap(a.aim[0]), a.aim[1]];
        m.act = a;
    }

    fn step_mech(&mut self, i: usize, shots: &mut Vec<Shot>) {
        let t = self.t;
        let map = self.map.clone();
        let mut snd: Vec<(&'static str, [f64; 3])> = Vec::new();
        {
            let m = &mut self.mechs[i];
            let tr = g(&m.st, "turn_rate");
            let pr = g(&m.st, "pitch_rate");
            let dy = wrap(m.cmd_aim[0] - m.aim[0]);
            m.aim[0] = wrap(m.aim[0] + dy.clamp(-tr * DT, tr * DT));
            let cp = m.cmd_aim[1].clamp(g(&m.st, "min_pitch"), g(&m.st, "max_pitch"));
            m.aim[1] += (cp - m.aim[1]).clamp(-pr * DT, pr * DT);

            let (mut mx, mut my) = (m.act.mv[0], m.act.mv[1]);
            let len = hyp(mx, my);
            if len > 1.0 {
                mx /= len;
                my /= len;
            }
            let moving = len > 0.01;
            let charging = m.w.iter().flatten().any(|w| w.kind == LASER && w.charge > 0.0);
            let mut speed = g(&m.st, "walk_speed");
            let mut boosting = false;
            if m.act.boost && m.grounded && moving && m.energy > 0.0 {
                speed = g(&m.st, "boost_speed");
                boosting = true;
            }
            if charging {
                speed *= g(&m.st, "charge_move_mult");
            }
            let (wx, wy) = (mx * speed, my * speed);
            let acc = if m.grounded { g(&m.st, "ground_accel") } else { g(&m.st, "air_accel") } * DT;
            let (dx, dyv) = (wx - m.vel[0], wy - m.vel[1]);
            let dl = hyp(dx, dyv);
            if dl <= acc {
                m.vel[0] = wx;
                m.vel[1] = wy;
            } else {
                m.vel[0] += dx / dl * acc;
                m.vel[1] += dyv / dl * acc;
            }
            if m.act.jump && m.grounded {
                m.vel[2] = g(&m.st, "jump_speed");
                m.grounded = false;
            }
            let mut jet = false;
            if !m.grounded {
                if m.act.boost && m.energy > 0.0 {
                    m.vel[2] = (m.vel[2] + g(&m.st, "jet_accel") * DT).min(g(&m.st, "max_rise_speed"));
                    jet = true;
                }
                m.vel[2] -= g(&m.st, "gravity") * DT;
            }
            if boosting {
                m.energy -= g(&m.st, "boost_drain") * DT;
                m.last_spend = t;
            }
            if jet {
                m.energy -= g(&m.st, "jet_drain") * DT;
                m.last_spend = t;
            }
            m.energy = m.energy.max(0.0);
            if t - m.last_spend >= g(&m.st, "energy_regen_delay") {
                m.energy = (m.energy + g(&m.st, "energy_regen") * DT).min(g(&m.st, "energy"));
            }
            // horizontal move with sliding
            let (r, h) = (m.radius(), m.height());
            let (x, y, z) = (m.pos[0], m.pos[1], m.pos[2]);
            let mut nx = x + m.vel[0] * DT;
            let mut ny = y + m.vel[1] * DT;
            // At ground level slide smoothly along walls using the wall-distance field.
            let on_ground_level = m.grounded && (z - map.ground(x, y)).abs() < 1.5;
            if on_ground_level {
                let (wd, gx, gy) = map.wdist_grad(nx, ny);
                // wall vertices sit up to ~0.5 m inside the true face
                let lim = r - 0.4;
                if wd < lim {
                    let gl = hyp(gx, gy);
                    if gl > 1e-6 {
                        let (ux, uy) = (gx / gl, gy / gl);
                        let vn = m.vel[0] * ux + m.vel[1] * uy;
                        if vn < 0.0 {
                            m.vel[0] -= vn * ux;
                            m.vel[1] -= vn * uy;
                        }
                        nx = x + m.vel[0] * DT + ux * (lim - wd).min(0.3);
                        ny = y + m.vel[1] * DT + uy * (lim - wd).min(0.3);
                    }
                }
            }
            if let Some(n) = map.blocked(nx, ny, z, r - 0.45, h) {
                let vn = m.vel[0] * n.0 + m.vel[1] * n.1;
                if vn < 0.0 {
                    m.vel[0] -= vn * n.0;
                    m.vel[1] -= vn * n.1;
                }
                nx = x + m.vel[0] * DT;
                ny = y + m.vel[1] * DT;
                if map.blocked(nx, ny, z, r - 0.45, h).is_some() {
                    // try to escape along the normal a little (already overlapping)
                    if map.blocked(x, y, z, r - 0.45, h).is_some() {
                        nx = x + n.0 * 0.3;
                        ny = y + n.1 * 0.3;
                    } else {
                        nx = x;
                        ny = y;
                    }
                    m.vel[0] = 0.0;
                    m.vel[1] = 0.0;
                }
            }
            let step_up = g(&m.st, "step_up");
            if m.grounded {
                let (surf, solid) = map.surface(nx, ny, z);
                let rise = surf - z;
                let hs = hyp(nx - x, ny - y);
                let steep = !solid && rise > hs * g(&m.st, "max_slope_deg").to_radians().tan() + 0.03;
                if rise > step_up || steep {
                    nx = x;
                    ny = y;
                    m.vel[0] = 0.0;
                    m.vel[1] = 0.0;
                } else if rise < -0.7 {
                    m.grounded = false;
                    m.vel[2] = 0.0;
                } else {
                    m.pos[2] = surf;
                }
            } else {
                let nz = z + m.vel[2] * DT;
                let (surf, _) = map.surface(nx, ny, z.max(nz));
                if nz <= surf && m.vel[2] <= 0.0 {
                    m.pos[2] = surf;
                    m.vel[2] = 0.0;
                    m.grounded = true;
                } else {
                    m.pos[2] = nz;
                }
            }
            if moving && hyp(nx - x, ny - y) < 0.02 {
                m.stuck_ticks += 1;
            }
            m.pos[0] = nx;
            m.pos[1] = ny;
            // footsteps
            let sp = hyp(m.vel[0], m.vel[1]);
            if m.grounded && sp > 5.0 && t - m.step_t >= 0.6 {
                m.step_t = t;
                snd.push((if boosting { "boost" } else { "step" }, m.pos));
            }
            if jet && t - m.step_t >= 0.6 {
                m.step_t = t;
                snd.push(("jet", m.pos));
            }
            // weapons
            let short = t < m.short_until;
            for s in 0..2 {
                let equip_ok = t >= m.equip_until[s];
                let fire = m.act.fire[s];
                let want_reload = m.act.reload;
                let energy = m.energy;
                let wst = m.wst.clone();
                if let Some(w) = m.w[s].as_mut() {
                    // As the real server does (public replays: a held gun fires on ticks 1, 6, 11, 16,
                    // 21, 25, 30, ...): the cooldown counts down every tick, the weapon fires on the tick
                    // it runs out, and the overshoot carries into the next shot so the cadence is exact.
                    w.cooldown -= DT;
                    let over = w.cooldown.min(0.0).max(-DT);
                    if w.cooldown < 0.0 {
                        w.cooldown = 0.0;
                    }
                    let ws = &wst[w.kind];
                    match w.kind {
                        GUN => {
                            if w.reload > 0.0 {
                                w.reload -= DT;
                                if w.reload <= 0.0 {
                                    w.reload = 0.0;
                                    w.ammo = g(ws, "mag");
                                }
                            } else if want_reload && w.ammo < g(ws, "mag") && !fire {
                                w.reload = g(ws, "reload");
                                snd.push(("reload", m.pos));
                            } else if fire && w.cooldown <= 0.0 && equip_ok && w.ammo > 0.0 {
                                w.ammo -= 1.0;
                                w.cooldown = g(ws, "cooldown") + over;
                                if w.ammo <= 0.0 {
                                    w.reload = g(ws, "reload");
                                }
                                shots.push(Shot::Bullet(i, s));
                            }
                        }
                        LASER => {
                            let ct = g(ws, "charge");
                            if fire && w.cooldown <= 0.0 && equip_ok && !short {
                                if w.charge <= 0.0 {
                                    snd.push(("laser_charge", m.pos));
                                }
                                if w.charge >= ct {
                                    w.held_full += DT;
                                    if w.held_full >= g(ws, "hold_s") {
                                        shots.push(Shot::Laser(i, s, 1.0));
                                        w.charge = 0.0;
                                        w.held_full = 0.0;
                                        w.cooldown = g(ws, "cooldown");
                                    }
                                } else {
                                    w.charge = (w.charge + DT).min(ct);
                                }
                            } else if w.charge > 0.0 {
                                if short {
                                    w.charge = 0.0;
                                    w.held_full = 0.0;
                                } else if w.charge >= g(ws, "min_charge") {
                                    let frac = g(ws, "min_frac") + (1.0 - g(ws, "min_frac")) * (w.charge / ct).min(1.0);
                                    shots.push(Shot::Laser(i, s, frac));
                                    w.charge = 0.0;
                                    w.held_full = 0.0;
                                    w.cooldown = g(ws, "cooldown");
                                } else {
                                    w.charge += DT;
                                }
                            }
                        }
                        LIGHTNING => {
                            if fire && w.cooldown <= 0.0 && equip_ok && energy >= g(ws, "energy") {
                                w.cooldown = g(ws, "cooldown");
                                shots.push(Shot::Zap(i, s));
                            }
                        }
                        MORTAR => {
                            if fire && w.cooldown <= 0.0 && equip_ok {
                                w.cooldown = g(ws, "cooldown");
                                shots.push(Shot::Shell(i, s));
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
        for (k, p) in snd {
            self.sound(k, p, Some(i));
        }
    }

    fn resolve_shot(&mut self, shot: Shot) {
        match shot {
            Shot::Bullet(i, _) => {
                let (o, aim, r, speed, range, spread, dmg) = {
                    let m = &self.mechs[i];
                    let w = &m.wst[GUN];
                    (m.muzzle(), m.aim, m.radius(), g(w, "speed"), g(w, "range"), g(w, "spread_deg").to_radians(), g(w, "damage") * m.dmg_mult())
                };
                let a = [aim[0] + gauss(&mut self.rng) * spread, aim[1] + gauss(&mut self.rng) * spread];
                let d = dir3(a);
                let off = r + 0.5;
                let start = [o[0] + d[0] * off, o[1] + d[1] * off, o[2] + d[2] * off];
                let start = if self.map.los(o, start) { start } else { o };
                let id = self.next_id;
                self.next_id += 1;
                self.projs.push(Proj { id, kind: 0, owner: i, pos: start, vel: [d[0] * speed, d[1] * speed, d[2] * speed], left: range, dmg, from: o });
                self.mechs[i].shots += 1;
                self.sound("gun", o, Some(i));
                let mut best = 1e9;
                for m in &self.mechs {
                    if m.alive && m.id != i {
                        let b = (m.pos[1] - o[1]).atan2(m.pos[0] - o[0]);
                        let dd = d2(&m.pos, &o);
                        if wrap(b - aim[0]).abs() < 0.2 && dd < best {
                            best = dd;
                        }
                    }
                }
                if best < 1e8 {
                    self.gun_fired[((best / 50.0) as usize).min(5)] += 1;
                }
            }
            Shot::Shell(i, _) => {
                let (aim, r, h, pos, speed) = {
                    let m = &self.mechs[i];
                    (m.aim, m.radius(), m.height(), m.pos, g(&m.wst[MORTAR], "speed"))
                };
                let o = [pos[0], pos[1], pos[2] + h + 0.5];
                let d = dir3(aim);
                let off = r + 0.5;
                let start = [o[0] + d[0] * off, o[1] + d[1] * off, o[2] + d[2] * off];
                let id = self.next_id;
                self.next_id += 1;
                self.projs.push(Proj { id, kind: 1, owner: i, pos: start, vel: [d[0] * speed, d[1] * speed, d[2] * speed], left: 7.0, dmg: 0.0, from: o });
                self.sound("mortar", o, Some(i));
            }
            Shot::Laser(i, _, frac) => {
                let (o, aim, range, dmg, cook) = {
                    let m = &self.mechs[i];
                    let w = &m.wst[LASER];
                    (m.muzzle(), m.aim, g(w, "range"), g(w, "damage") * frac * m.dmg_mult(), g(w, "cookoff"))
                };
                let d = dir3(aim);
                let end = [o[0] + d[0] * range, o[1] + d[1] * range, o[2] + d[2] * range];
                let tw = self.map.ray_hit(o, end).unwrap_or(1.0);
                let hit = self.seg_hit(o, end, 0.0, Some(i), None).filter(|h| h.0 <= tw);
                let tf = hit.map_or(tw, |h| h.0);
                let hp = [o[0] + d[0] * range * tf, o[1] + d[1] * range * tf, o[2] + d[2] * range * tf];
                self.sound("laser", o, Some(i));
                self.laser_fired += 1;
                if let Some((_, Tgt::Mech(_))) = hit {
                    self.laser_hit += 1;
                }
                if let Some((_, tg)) = hit {
                    let mut amount = dmg;
                    if let Tgt::Mech(k) = tg {
                        if self.mechs[k].has(MORTAR) {
                            amount *= 1.0 + cook;
                            self.mechs[k].events.push(obj(vec![("type", js("effect")), ("effect", js("cookoff")), ("by", num(i as f64))]));
                        }
                    }
                    self.hurt(tg, amount, "laser", Some(i), o);
                }
                self.beam(i, "laser", vec![o, hp]);
            }
            Shot::Zap(i, _) => {
                let (o, aim, range, cone, dmg, chain, chain_range, fall, cost, short_s) = {
                    let m = &self.mechs[i];
                    let w = &m.wst[LIGHTNING];
                    (m.muzzle(), m.aim, g(w, "range"), g(w, "cone_deg").to_radians(), g(w, "damage") * m.dmg_mult(), g(w, "chain") as usize, g(w, "chain_range"), g(w, "chain_falloff"), g(w, "energy"), g(w, "short_s"))
                };
                self.mechs[i].energy = (self.mechs[i].energy - cost).max(0.0);
                self.mechs[i].last_spend = self.t;
                self.sound("lightning", o, Some(i));
                let mut hit: Vec<Tgt> = Vec::new();
                let mut path = vec![o];
                let mut from = o;
                let mut amount = dmg;
                for jump in 0..=chain {
                    let mut best: Option<(f64, Tgt, [f64; 3])> = None;
                    for tg in self.targets(Some(i)) {
                        if hit.contains(&tg) {
                            continue;
                        }
                        let (c, _, h) = self.tgt_cyl(tg);
                        let center = [c[0], c[1], c[2] + h * 0.5];
                        let d = d3(&from, &center);
                        if jump == 0 {
                            if d > range {
                                continue;
                            }
                            let b = (center[1] - o[1]).atan2(center[0] - o[0]);
                            if wrap(b - aim[0]).abs() > cone * 0.5 {
                                continue;
                            }
                        } else if d > chain_range {
                            continue;
                        }
                        if !self.map.los(from, center) {
                            continue;
                        }
                        if best.map_or(true, |x| d < x.0) {
                            best = Some((d, tg, center));
                        }
                    }
                    match best {
                        None => break,
                        Some((_, tg, center)) => {
                            if let Tgt::Mech(k) = tg {
                                if self.mechs[k].has(LASER) {
                                    let t = self.t;
                                    let m = &mut self.mechs[k];
                                    m.short_until = t + short_s;
                                    for w in m.w.iter_mut().flatten() {
                                        if w.kind == LASER {
                                            w.charge = 0.0;
                                            w.held_full = 0.0;
                                        }
                                    }
                                    m.events.push(obj(vec![("type", js("effect")), ("effect", js("short")), ("by", num(i as f64))]));
                                }
                            }
                            self.hurt(tg, amount, "lightning", Some(i), from);
                            hit.push(tg);
                            path.push(center);
                            from = center;
                            amount *= fall;
                        }
                    }
                }
                if path.len() == 1 {
                    let d = dir3(aim);
                    path.push([o[0] + d[0] * range, o[1] + d[1] * range, o[2] + d[2] * range]);
                }
                self.beam(i, "lightning", path);
            }
        }
    }

    fn beam(&mut self, by: usize, kind: &str, path: Vec<[f64; 3]>) {
        for k in 0..self.mechs.len() {
            if k == by || !self.mechs[k].alive {
                continue;
            }
            let m = &self.mechs[k];
            let vis = path.iter().any(|p| self.can_see(m, *p, *p));
            if vis {
                let j = obj(vec![("by", num(by as f64)), ("kind", js(kind)), ("path", J::Arr(path.iter().map(|p| jv3(*p)).collect())), ("age", num(0.05))]);
                self.mechs[k].beams.push(j);
            }
        }
    }

    fn step_projs(&mut self) {
        let mut k = 0;
        while k < self.projs.len() {
            let (kind, owner, p0, mut v, dmg, from) = {
                let p = &self.projs[k];
                (p.kind, p.owner, p.pos, p.vel, p.dmg, p.from)
            };
            if kind == 1 {
                v[2] -= self.mechs[owner.min(self.mechs.len() - 1)].wst[MORTAR].get("gravity").copied().unwrap_or(22.0) * DT;
            }
            let p1 = [p0[0] + v[0] * DT, p0[1] + v[1] * DT, p0[2] + v[2] * DT];
            let tw = self.map.ray_hit(p0, p1);
            let (me_except, npc_except) = if kind == 9 { (None, Some(owner)) } else { (Some(owner), None) };
            let th = self.seg_hit(p0, p1, 0.3, me_except, npc_except);
            let mut remove = false;
            let first_wall = tw.map_or(2.0, |x| x);
            let first_hit = th.map_or(2.0, |x| x.0);
            if first_hit <= first_wall && first_hit <= 1.0 {
                let tg = th.unwrap().1;
                let hp = [p0[0] + v[0] * DT * first_hit, p0[1] + v[1] * DT * first_hit, p0[2] + v[2] * DT * first_hit];
                match kind {
                    0 => {
                        if let Tgt::Mech(k) = tg {
                            let dd = d2(&self.mechs[k].pos, &from);
                            self.gun_hit[((dd / 50.0) as usize).min(5)] += 1;
                            self.mechs[owner].hits += 1;
                        }
                        self.hurt(tg, dmg, "gun", Some(owner), from)
                    }
                    9 => self.hurt(tg, dmg, "npc", Some(100 + owner), from),
                    _ => self.explode(hp, owner, Some(tg)),
                }
                remove = true;
            } else if first_wall <= 1.0 {
                if kind == 1 {
                    let f = (first_wall - 0.02).max(0.0);
                    let hp = [p0[0] + v[0] * DT * f, p0[1] + v[1] * DT * f, p0[2] + v[2] * DT * f];
                    self.explode(hp, owner, None);
                }
                remove = true;
            }
            if !remove {
                let p = &mut self.projs[k];
                p.pos = p1;
                p.vel = v;
                if kind == 1 {
                    p.left -= DT;
                } else {
                    p.left -= hyp(hyp(v[0], v[1]), v[2]) * DT;
                }
                if p.left <= 0.0 {
                    remove = true;
                }
            }
            if remove {
                self.projs.swap_remove(k);
            } else {
                k += 1;
            }
        }
    }

    fn step_npcs(&mut self) {
        let leash = self.cfgf("npc", "leash", 30.0);
        let chase = self.cfgf("npc", "chase_leash", 60.0);
        let aggro = self.cfgf("npc", "aggro_range", 45.0);
        let deaggro = self.cfgf("npc", "deaggro_range", 90.0);
        let speed = self.cfgf("npc", "speed", 5.0);
        let pspeed = self.cfgf("npc", "projectile_speed", 90.0);
        let spread = self.cfgf("npc", "spread_deg", 2.5).to_radians();
        let dmg = self.cfgf("npc", "damage", 25.0);
        let cd = self.cfgf("npc", "cooldown", 1.0);
        for k in 0..self.npcs.len() {
            if !self.npcs[k].alive {
                continue;
            }
            let (pos, home) = (self.npcs[k].pos, self.npcs[k].home);
            let dh = d2(&pos, &home);
            let eye = [pos[0], pos[1], pos[2] + 2.5];
            if let Some(tg) = self.npcs[k].target {
                let ok = tg < self.mechs.len() && self.mechs[tg].alive && d2(&self.mechs[tg].pos, &pos) <= deaggro && dh <= chase;
                if !ok {
                    self.npcs[k].target = None;
                }
            }
            if self.npcs[k].target.is_none() && dh <= leash {
                let mut best: Option<(f64, usize)> = None;
                for m in &self.mechs {
                    if !m.alive {
                        continue;
                    }
                    let d = d2(&m.pos, &pos);
                    if d <= aggro && self.map.los(eye, m.center()) && best.map_or(true, |b| d < b.0) {
                        best = Some((d, m.id));
                    }
                }
                self.npcs[k].target = best.map(|b| b.1);
            }
            let mut want = [0.0, 0.0];
            let mut shot: Option<[f64; 3]> = None;
            if let Some(tg) = self.npcs[k].target {
                let tc = self.mechs[tg].center();
                let d = d2(&tc, &pos);
                if d > 35.0 && dh < chase - 3.0 {
                    want = [(tc[0] - pos[0]) / d * speed, (tc[1] - pos[1]) / d * speed];
                }
                let n = &mut self.npcs[k];
                n.aim = [(tc[1] - pos[1]).atan2(tc[0] - pos[0]), 0.0];
                if n.cooldown <= 0.0 && self.map.los(eye, tc) {
                    n.cooldown = cd;
                    shot = Some(tc);
                }
            } else {
                let goal = self.npcs[k].goal;
                let d = d2(&goal, &pos);
                if d < 2.0 || self.rng.f() < 0.002 {
                    let a = self.rng.range(-PI, PI);
                    let r = self.rng.range(0.0, leash * 0.8);
                    self.npcs[k].goal = [home[0] + a.cos() * r, home[1] + a.sin() * r];
                } else {
                    want = [(goal[0] - pos[0]) / d * speed * 0.5, (goal[1] - pos[1]) / d * speed * 0.5];
                }
            }
            let n = &mut self.npcs[k];
            n.cooldown = (n.cooldown - DT).max(0.0);
            let (nx, ny) = (pos[0] + want[0] * DT, pos[1] + want[1] * DT);
            if self.map.wall_dist(nx, ny) >= 2.5 {
                n.pos = [nx, ny, self.map.ground(nx, ny)];
                n.vel = [want[0], want[1], 0.0];
            } else {
                n.vel = [0.0; 3];
                n.goal = home;
            }
            if let Some(tc) = shot {
                let o = eye;
                let dd = d3(&o, &tc).max(0.1);
                let yaw = (tc[1] - o[1]).atan2(tc[0] - o[0]) + gauss(&mut self.rng) * spread;
                let pitch = ((tc[2] - o[2]) / dd).asin() + gauss(&mut self.rng) * spread;
                let d = dir3([yaw, pitch]);
                let id = self.next_id;
                self.next_id += 1;
                self.projs.push(Proj { id, kind: 9, owner: k, pos: [o[0] + d[0] * 2.5, o[1] + d[1] * 2.5, o[2] + d[2] * 2.5], vel: [d[0] * pspeed, d[1] * pspeed, d[2] * pspeed], left: 200.0, dmg, from: o });
                self.sound("npc_gun", o, None);
            }
        }
    }

    pub fn step(&mut self) {
        let t = self.t;
        let n = self.mechs.len();
        let mut shots = Vec::new();
        for i in 0..n {
            if self.mechs[i].alive {
                self.step_mech(i, &mut shots);
            }
        }
        // push overlapping mechs apart
        for a in 0..n {
            for b in a + 1..n {
                if !self.mechs[a].alive || !self.mechs[b].alive {
                    continue;
                }
                let (pa, pb) = (self.mechs[a].pos, self.mechs[b].pos);
                let rr = self.mechs[a].radius() + self.mechs[b].radius();
                let d = d2(&pa, &pb);
                if d < rr && (pa[2] - pb[2]).abs() < 9.0 {
                    let (ux, uy) = if d > 0.01 { ((pa[0] - pb[0]) / d, (pa[1] - pb[1]) / d) } else { (1.0, 0.0) };
                    let push = (rr - d) * 0.5;
                    for (k, sgn) in [(a, 1.0), (b, -1.0)] {
                        let m = &mut self.mechs[k];
                        let (nx, ny) = (m.pos[0] + ux * push * sgn, m.pos[1] + uy * push * sgn);
                        if self.map.blocked(nx, ny, m.pos[2], m.radius(), m.height()).is_none() {
                            m.pos[0] = nx;
                            m.pos[1] = ny;
                        }
                    }
                }
            }
        }
        for s in shots {
            self.resolve_shot(s);
        }
        self.step_npcs();
        self.step_projs();
        // crates destroyed
        let mut k = 0;
        while k < self.crates.len() {
            if self.crates[k].hp <= 0.0 {
                let p = self.crates[k].pos;
                self.crates.swap_remove(k);
                self.sound("crate", p, None);
                let id = self.next_id;
                self.next_id += 1;
                self.cubes.push(CubeE { id: 5000 + id, pos: p });
                if self.rng.f() < self.cfgf("loot", "crate_weapon_chance", 0.5) {
                    let kind = 1 + (self.rng.f() * 3.0) as usize % 3;
                    self.drop_weapon_at(kind, [p[0] + 1.0, p[1], p[2]]);
                }
            } else {
                k += 1;
            }
        }
        // zone, regen
        self.step_zone();
        for i in 0..n {
            if !self.mechs[i].alive {
                continue;
            }
            let inside = d2(&self.mechs[i].pos, &self.zc) <= self.zr;
            let m = &mut self.mechs[i];
            if !inside {
                let d = self.zdps * DT;
                m.hp -= d;
                m.zone_dmg += d;
                m.last_combat = t;
                if m.hp <= 0.0 && m.death_by.is_empty() {
                    m.death_by = "zone".into();
                }
            } else if t - m.last_combat >= g(&m.st, "regen_delay") {
                let mh = m.max_hp();
                m.hp = (m.hp + g(&m.st, "regen_per_s") * DT).min(mh);
            }
        }
        // cubes
        let cr = self.cfgf("cubes", "pickup_radius", 3.5);
        let mut k = 0;
        while k < self.cubes.len() {
            let p = self.cubes[k].pos;
            let mut best: Option<(f64, usize)> = None;
            for m in &self.mechs {
                if m.alive {
                    let d = d2(&m.pos, &p);
                    if d <= cr && (m.pos[2] - p[2]).abs() <= 4.0 && best.map_or(true, |b| d < b.0) {
                        best = Some((d, m.id));
                    }
                }
            }
            if let Some((_, i)) = best {
                self.cubes.swap_remove(k);
                self.gain_cubes(i, 1.0);
                let c = self.mechs[i].cubes;
                self.mechs[i].events.push(obj(vec![("type", js("pickup")), ("cubes", num(c))]));
            } else {
                k += 1;
            }
        }
        // supply drops
        if let Some(&dt0) = self.drop_times.first() {
            if t >= dt0 {
                self.drop_times.remove(0);
                let spread = self.cfgf("drops", "spread", 0.7);
                let mut pos = [self.znc[0], self.znc[1], self.map.ground(self.znc[0], self.znc[1])];
                for _ in 0..300 {
                    let p = self.open[(self.rng.f() * self.open.len() as f64) as usize % self.open.len()];
                    if d2(&p, &self.znc) <= self.znr * spread {
                        pos = p;
                        break;
                    }
                }
                let id = 20000 + self.drops.len() + self.drop_times.len() * 7;
                self.drops.push(DropE { id, pos, lands_at: t + self.cfgf("drops", "fall_s", 25.0), progress: 0.0, holder: None });
            }
        }
        let ring = self.cfgf("drops", "radius", 8.0);
        let cap = self.cfgf("drops", "capture_s", 4.0);
        let mut k = 0;
        while k < self.drops.len() {
            let mut done: Option<usize> = None;
            if t >= self.drops[k].lands_at {
                let p = self.drops[k].pos;
                let inside: Vec<usize> = self.mechs.iter().filter(|m| m.alive && d2(&m.pos, &p) <= ring && (m.pos[2] - p[2]).abs() < 6.0).map(|m| m.id).collect();
                let d = &mut self.drops[k];
                if inside.len() == 1 {
                    if d.holder == Some(inside[0]) {
                        d.progress += DT;
                    } else {
                        d.holder = Some(inside[0]);
                        d.progress = 0.0;
                    }
                    if d.progress >= cap {
                        done = Some(inside[0]);
                    }
                } else if inside.is_empty() {
                    d.holder = None;
                    d.progress = 0.0;
                }
            }
            if let Some(i) = done {
                let id = self.drops[k].id;
                self.drops.swap_remove(k);
                let c = self.cfgf("drops", "cubes", 3.0);
                self.gain_cubes(i, c);
                let m = &mut self.mechs[i];
                m.hp = m.max_hp();
                m.energy = g(&m.st, "energy");
                let cubes = m.cubes;
                m.events.push(obj(vec![("type", js("drop")), ("id", num(id as f64)), ("cubes", num(cubes))]));
            } else {
                k += 1;
            }
        }
        // offers time out
        for i in 0..n {
            if !self.mechs[i].alive {
                continue;
            }
            let timeout = self.cfgf("bot", "offer_timeout_s", 5.0);
            let mut expired: Option<String> = None;
            if let Some(front) = self.mechs[i].offers.front_mut() {
                if front.1 < 0.0 {
                    front.1 = t + timeout;
                } else if t >= front.1 {
                    expired = front.2.first().cloned();
                }
            }
            if let Some(id) = expired {
                self.mechs[i].offers.pop_front();
                self.apply_upgrade(i, &id);
            }
        }
        // deaths
        let dead: Vec<usize> = (0..n).filter(|&i| self.mechs[i].alive && self.mechs[i].hp <= 0.0).collect();
        if !dead.is_empty() {
            let place = self.alive - dead.len() + 1;
            for &i in &dead {
                let (pos, cubes, weapons) = {
                    let m = &mut self.mechs[i];
                    m.alive = false;
                    m.place = place;
                    m.death_t = t;
                    if m.death_by.is_empty() {
                        m.death_by = "?".into();
                    }
                    (m.pos, m.cubes, m.w.iter().flatten().map(|w| w.kind).filter(|&k| k != GUN).collect::<Vec<_>>())
                };
                self.death_log.push((t, i, self.mechs[i].death_by.clone()));
                self.sound("death", pos, Some(i));
                let nc = cubes as usize + self.cfgf("cubes", "death_drop_extra", 2.0) as usize;
                for q in 0..nc {
                    let a = q as f64 * 2.4;
                    let id = self.next_id;
                    self.next_id += 1;
                    self.cubes.push(CubeE { id: 5000 + id, pos: [pos[0] + a.cos() * 1.5, pos[1] + a.sin() * 1.5, pos[2]] });
                }
                for kind in weapons {
                    self.drop_weapon_at(kind, pos);
                }
            }
            self.alive -= dead.len();
        }
        self.t += DT;
        self.tick += 1;
    }

    pub fn finished(&self) -> bool {
        self.alive <= 1 || self.t >= self.cfgf("match", "duration_s", 480.0)
    }

    /// Give survivors their places once the match is over.
    pub fn finalize(&mut self) {
        let mut left: Vec<usize> = (0..self.mechs.len()).filter(|&i| self.mechs[i].alive).collect();
        left.sort_by(|&a, &b| self.mechs[b].hp.partial_cmp(&self.mechs[a].hp).unwrap_or(std::cmp::Ordering::Equal));
        for (rank, i) in left.iter().enumerate() {
            self.mechs[*i].place = rank + 1;
            self.mechs[*i].death_t = self.t;
            self.mechs[*i].death_by = "survived".into();
        }
    }
}

// ---------------------------------------------------------------------- opponents

/// The kit's example bot, ported: walk straight at the next centre, shoot the nearest mech.
pub struct Example {
    reach: f64,
}
impl Example {
    pub fn new() -> Example {
        Example { reach: 4.0 }
    }
}
impl Ctl for Example {
    fn act(&mut self, o: &J) -> Action {
        let me = o.get("self");
        let p = me.get("pos").v3();
        let goal = o.get("zone").get("next_center").v2();
        let mut a = Action::default();
        if !o.get("offer").is_null() {
            a.pick = Some(0);
        }
        let near: Vec<&J> = o.get("pickups").arr().iter().filter(|q| d2(&q.get("pos").v3(), &p) < self.reach).collect();
        if me.get("weapons").at(1).is_null() && !near.is_empty() && near[0].get("kind").s() != "gun" {
            a.take = Some(near[0].get("id").f() as i64);
        }
        let yaw = (goal[1] - p[1]).atan2(goal[0] - p[0]);
        a.mv = [yaw.cos(), yaw.sin()];
        a.aim = [yaw, 0.0];
        a.boost = !me.get("in_zone").b();
        a.jump = (o.get("tick").f() as i64) % 40 == 0;
        let mut best: Option<(f64, [f64; 3])> = None;
        for m in o.get("mechs").arr() {
            if m.get("kind").s() != "mech" {
                continue;
            }
            let q = m.get("pos").v3();
            let d = d3(&q, &p);
            if best.map_or(true, |b| d < b.0) {
                best = Some((d, q));
            }
        }
        if let Some((_, q)) = best {
            a.aim = [(q[1] - p[1]).atan2(q[0] - p[0]), (q[2] - p[2] - 2.5).atan2(d2(&q, &p))];
            a.fire = [true, true];
        }
        a
    }
}

/// A guess at the built-in bots, shaped by the one recorded match in the kit: boosts toward the
/// next centre, picks up a second weapon, attacks what it sees at long range with gun and laser
/// while strafing in short random bursts, and chases.
pub struct House {
    map: Arc<Map>,
    nav: Nav,
    rng: Rng,
    path: Vec<(f64, f64)>,
    path_i: usize,
    plan_t: f64,
    target: i64,
    target_t: f64,
    strafe_dir: f64,
    strafe_until: f64,
    strafe_boost: bool,
    noise: [f64; 2],
    noise_t: f64,
    pub skill: f64,
    pub aggro: f64,
    last_aim: [f64; 2],
    wander: (f64, f64),
    wander_t: f64,
    last_pos: [f64; 2],
    still: u32,
    pub duty: f64,
    pub dodge: bool,
    pub hunt: bool,
    /// fire only when the target has been moving steadily (0 = always fire when aligned)
    pub gate: f64,
    /// retreat below this HP fraction
    pub retreat: f64,
    tvel: [f64; 2],
    tvel_id: i64,
    steady_t: f64,
    flee_until: f64,
    /// only ever target this mech id (stress test); -1 = anyone
    pub focus: i64,
    hunt_pt: (f64, f64),
    hunt_t: f64,
    attack_until: f64,
    rest_until: f64,
    strafe_speed: f64,
    /// 0 = heads for the middle of the next circle when idle, 1 = roams the whole circle; values between = chance of being a roamer
    pub roam: f64,
    roamer: bool,
    /// distance it tries to close to in a fight
    pub push: f64,
    /// 1 = full turn at the start and every 11 s when it has no target
    pub look: f64,
}
impl House {
    pub fn new(map: Arc<Map>, seed: u64, skill: f64, aggro: f64) -> House {
        let nav = Nav::new(&map);
        House {
            map,
            nav,
            rng: Rng(seed | 1),
            path: Vec::new(),
            path_i: 0,
            plan_t: -100.0,
            target: -1,
            target_t: -100.0,
            strafe_dir: 1.0,
            strafe_until: -1.0,
            strafe_boost: false,
            noise: [0.0; 2],
            noise_t: -100.0,
            skill,
            aggro,
            last_aim: [0.0; 2],
            wander: (0.0, 0.0),
            wander_t: -100.0,
            last_pos: [0.0; 2],
            still: 0,
            duty: 0.5,
            dodge: true,
            hunt: true,
            gate: 1.0,
            retreat: 0.4,
            tvel: [0.0; 2],
            tvel_id: -1,
            steady_t: 0.0,
            flee_until: -1.0,
            focus: -1,
            hunt_pt: (0.0, 0.0),
            hunt_t: -100.0,
            attack_until: -1.0,
            rest_until: -1.0,
            strafe_speed: 1.0,
            roam: 0.5,
            roamer: (seed >> 17) & 1 == 1,
            push: 50.0,
            look: 0.0,
        }
    }
    fn route(&mut self, from: [f64; 2], to: (f64, f64), t: f64) {
        if t - self.plan_t < 0.7 && !self.path.is_empty() {
            return;
        }
        self.plan_t = t;
        self.path.clear();
        self.path_i = 0;
        if let Some(cells) = self.nav.path((from[0], from[1]), to, 3.4, None, 200_000) {
            for c in cells {
                self.path.push(self.nav.pos_of(c));
            }
        } else {
            self.path.push(to);
        }
    }
    fn follow(&mut self, p: [f64; 2]) -> [f64; 2] {
        let n = self.path.len();
        if n == 0 {
            return [0.0, 0.0];
        }
        while self.path_i + 1 < n {
            let a = self.path[self.path_i];
            let b = self.path[self.path_i + 1];
            if hyp(a.0 - p[0], a.1 - p[1]) < 1.6 || hyp(b.0 - p[0], b.1 - p[1]) < hyp(a.0 - p[0], a.1 - p[1]) {
                self.path_i += 1;
            } else {
                break;
            }
        }
        let mut k = self.path_i;
        for cand in self.path_i + 1..(self.path_i + 12).min(n) {
            if crate::nav::free_line(&self.map, (p[0], p[1]), self.path[cand], 3.0) {
                k = cand;
            }
        }
        let tg = self.path[k];
        let d = hyp(tg.0 - p[0], tg.1 - p[1]);
        if d < 0.8 {
            return [0.0, 0.0];
        }
        [(tg.0 - p[0]) / d, (tg.1 - p[1]) / d]
    }
}
impl Ctl for House {
    fn act(&mut self, o: &J) -> Action {
        let t = o.get("t").f();
        let me = o.get("self");
        let p = me.get("pos").v3();
        let p2 = [p[0], p[1]];
        let st = me.get("stats");
        let eye = [p[0], p[1], p[2] + st.get("eye_height").fo(8.0)];
        let muzzle = [p[0], p[1], p[2] + st.get("muzzle_height").fo(7.0)];
        let aim_now = me.get("aim").v2();
        let energy = me.get("energy").f();
        let zone = o.get("zone");
        let nc = zone.get("next_center").v2();
        let nr = zone.get("next_radius").f();
        let mut a = Action { aim: self.last_aim, ..Default::default() };
        if !o.get("offer").is_null() {
            a.pick = Some((self.rng.f() * 3.0) as usize % 3);
        }
        for e in o.get("events").arr() {
            if e.get("type").s() == "damaged" {
                if let Some(by) = e.get("by").num() {
                    if by < 100.0 && (self.focus < 0 || by as i64 == self.focus) {
                        self.target = by as i64;
                        self.target_t = t;
                    }
                }
            }
        }
        // remember where the last nearby shooting came from
        if self.hunt {
            for sd in o.get("sounds").arr() {
                let k = sd.get("kind").s();
                let dist = sd.get("dist").s();
                let loud = matches!(k, "gun" | "laser" | "lightning" | "mortar" | "laser_charge");
                let close = matches!(k, "step" | "boost" | "reload" | "jet");
                if (loud && dist != "far") || (close && dist == "near") {
                    let b = sd.get("bearing").f();
                    let r = if dist == "near" { 35.0 } else { 105.0 };
                    let cand = (p[0] + b.cos() * r, p[1] + b.sin() * r);
                    // prefer the nearest reports
                    if t - self.hunt_t > 3.0 || dist == "near" {
                        self.hunt_pt = cand;
                        self.hunt_t = t;
                    }
                }
            }
        }
        // targets
        let mut cur: Option<&J> = None;
        let mut nearest: Option<(f64, &J)> = None;
        for m in o.get("mechs").arr() {
            if m.get("kind").s() != "mech" || (self.focus >= 0 && m.get("id").f() as i64 != self.focus) {
                continue;
            }
            let q = m.get("pos").v3();
            let c = [q[0], q[1], q[2] + 4.5];
            let vis = self.map.los(eye, c);
            let d = d3(&c, &eye);
            if m.get("id").f() as i64 == self.target {
                cur = Some(m);
                if vis {
                    self.target_t = t;
                }
            }
            if vis && d <= self.aggro && nearest.map_or(true, |b| d < b.0) {
                nearest = Some((d, m));
            }
        }
        if cur.is_none() || t - self.target_t > 4.0 {
            self.target = -1;
            cur = None;
            if let Some((_, m)) = nearest {
                self.target = m.get("id").f() as i64;
                self.target_t = t;
                cur = Some(m);
            }
        }
        let in_next = d2(&p2, &nc) <= nr - 5.0;
        let zone_left = {
            let c = zone.get("center").v2();
            let r = zone.get("radius").f();
            let t1 = zone.get("shrink_end").f();
            if d2(&p2, &c) > r { 0.0 } else if in_next { 1e6 } else { (t1 - t).max(0.0) * ((r - d2(&p2, &c)) / (r - nr).max(1.0)).clamp(0.0, 1.0) + (zone.get("shrink_start").f() - t).max(0.0) }
        };
        let mut mv = [0.0, 0.0];
        let mut boost = false;
        let weapons: Vec<(usize, &J)> = (0..2).filter_map(|i| { let w = me.get("weapons").at(i); if w.is_null() { None } else { Some((i, w)) } }).collect();
        let mut fire = [false, false];
        let must_zone = !in_next && zone_left < 10.0;
        if let Some(m) = cur {
            let q = m.get("pos").v3();
            let v = m.get("vel").v3();
            let c = [q[0], q[1], q[2] + 4.5];
            let vis = self.map.los(muzzle, c);
            let d = d2(&q, &p);
            // shoots when it has a good solution: the target's velocity has held steady for a moment
            let tid = m.get("id").f() as i64;
            if tid != self.tvel_id {
                self.tvel_id = tid;
                self.tvel = [v[0], v[1]];
                self.steady_t = t;
            }
            if hyp(v[0] - self.tvel[0], v[1] - self.tvel[1]) > 2.5 {
                self.steady_t = t;
                self.tvel = [v[0], v[1]];
            }
            let steady = t - self.steady_t >= 0.2 * self.gate || d < 70.0 || self.gate <= 0.0;
            let _ = (self.attack_until, self.rest_until, self.duty);
            let attacking = steady;
            if vis {
                // aim noise resampled a few times a second
                if t - self.noise_t > 0.3 {
                    self.noise_t = t;
                    self.noise = [gauss(&mut self.rng) * self.skill, gauss(&mut self.rng) * self.skill * 0.5];
                }
                let mut aim_gun = [0.0, 0.0];
                for (i, w) in &weapons {
                    let ws = w.get("stats");
                    match w.get("kind").s() {
                        "gun" => {
                            let sp = ws.get("speed").fo(252.0);
                            let mut tt = d / sp;
                            let mut tp = c;
                            for _ in 0..2 {
                                tp = [c[0] + v[0] * tt, c[1] + v[1] * tt, c[2]];
                                tt = d3(&tp, &muzzle) / sp;
                            }
                            let dh = d2(&tp, &p).max(0.1);
                            aim_gun = [(tp[1] - p[1]).atan2(tp[0] - p[0]) + self.noise[0], (tp[2] - muzzle[2]).atan2(dh) + self.noise[1]];
                            let err = hyp(wrap(aim_gun[0] - aim_now[0]), aim_gun[1] - aim_now[1]);
                            if attacking && err < (2.6 / d.max(1.0)).atan() + 0.01 && d < ws.get("range").fo(230.0) {
                                fire[*i] = true;
                            }
                            a.aim = aim_gun;
                        }
                        "laser" => {
                            let direct = [(c[1] - p[1]).atan2(c[0] - p[0]) + self.noise[0] * 0.5, (c[2] - muzzle[2]).atan2(d.max(0.1))];
                            let ch = w.get("charge").f();
                            let full = ch >= ws.get("charge").fo(0.75) - 0.06;
                            if w.get("cooldown").f() <= 0.2 && d < ws.get("range").fo(280.0) && (attacking || ch > 0.0) {
                                if full {
                                    a.aim = direct;
                                    let err = hyp(wrap(direct[0] - aim_now[0]), direct[1] - aim_now[1]);
                                    fire[*i] = err > (2.2 / d.max(1.0)).atan();
                                } else {
                                    fire[*i] = true;
                                }
                            }
                        }
                        "lightning" => {
                            if d < ws.get("range").fo(45.0) - 2.0 && wrap(aim_gun[0] - aim_now[0]).abs() < 0.2 && energy > 6.0 {
                                fire[*i] = true;
                            }
                        }
                        "mortar" => {
                            let (vv, gg) = (ws.get("speed").fo(72.0), ws.get("gravity").fo(22.0));
                            let h = q[2] - (p[2] + 9.5);
                            let disc = vv.powi(4) - gg * (gg * d * d + 2.0 * h * vv * vv);
                            let gun_reloading = weapons.iter().any(|(_, g)| g.get("kind").s() == "gun" && g.get("reload").f() > 0.3);
                            if disc > 0.0 && d > 14.0 && w.get("cooldown").f() <= 0.1 && gun_reloading {
                                let th = ((vv * vv - disc.sqrt()) / (gg * d)).atan();
                                let tt = d / (vv * th.cos());
                                let tp = [q[0] + v[0] * tt, q[1] + v[1] * tt];
                                let ma = [(tp[1] - p[1]).atan2(tp[0] - p[0]), th];
                                a.aim = ma;
                                if hyp(wrap(ma[0] - aim_now[0]), ma[1] - aim_now[1]) < 0.04 {
                                    fire[*i] = true;
                                }
                            }
                        }
                        _ => {}
                    }
                }
                // strafe in short random bursts, hold a middle distance
                if t > self.strafe_until {
                    self.strafe_dir = if self.rng.f() < 0.5 { 1.0 } else { -1.0 };
                    self.strafe_until = t + self.rng.range(0.25, 0.6);
                    self.strafe_boost = self.rng.f() < 0.3;
                    self.strafe_speed = self.rng.range(0.5, 1.0);
                }
                let b = (q[1] - p[1]).atan2(q[0] - p[0]);
                let radial = if d > self.push + 25.0 { 0.9 } else if d > self.push + 8.0 { 0.45 } else if d < self.push - 20.0 { -0.5 } else { 0.0 };
                for _ in 0..2 {
                    let sa = b + self.strafe_dir * PI / 2.0;
                    let (sx, sy) = (sa.cos() + b.cos() * radial, sa.sin() + b.sin() * radial);
                    let l = hyp(sx, sy);
                    let ahead = (p[0] + sx / l * 5.0, p[1] + sy / l * 5.0);
                    if self.map.wall_dist(ahead.0, ahead.1) >= 3.2 {
                        mv = [sx / l * self.strafe_speed, sy / l * self.strafe_speed];
                        break;
                    }
                    self.strafe_dir = -self.strafe_dir;
                }
                boost = self.strafe_boost && energy > 40.0;
            } else {
                // chase
                self.route(p2, (q[0], q[1]), t);
                mv = self.follow(p2);
                boost = energy > 50.0;
                a.aim = [(q[1] - p[1]).atan2(q[0] - p[0]), 0.0];
            }
            let hpf = me.get("hp").f() / me.get("max_hp").fo(1400.0).max(1.0);
            let hit_now = o.get("events").arr().iter().any(|e| e.get("type").s() == "damaged");
            if hpf < self.retreat && hit_now && t > self.flee_until + 6.0 {
                self.flee_until = t + self.rng.range(3.0, 5.0);
            }
            if t < self.flee_until {
                // back off out of sight for a few seconds
                let b = (p[1] - q[1]).atan2(p[0] - q[0]) + self.strafe_dir * 0.6;
                let ahead = (p[0] + b.cos() * 5.0, p[1] + b.sin() * 5.0);
                if self.map.wall_dist(ahead.0, ahead.1) >= 3.2 {
                    mv = [b.cos(), b.sin()];
                }
                boost = energy > 10.0;
                a.jump = me.get("grounded").b() && self.rng.f() < 0.05;
            }
            if must_zone {
                self.route(p2, (nc[0], nc[1]), t);
                mv = self.follow(p2);
                boost = energy > 10.0;
            }
        } else {
            // loot nearby, else head for the middle of the next circle
            let mut goal: Option<(f64, f64)> = None;
            if me.get("weapons").at(1).is_null() && !must_zone {
                let mut best = 60.0;
                for pk in o.get("pickups").arr() {
                    let q = pk.get("pos").v3();
                    let d = d2(&q, &p);
                    if d < best && (q[2] - p[2]).abs() < 3.0 {
                        best = d;
                        goal = Some((q[0], q[1]));
                        if d < 3.6 {
                            a.take = Some(pk.get("id").f() as i64);
                        }
                    }
                }
            }
            if goal.is_none() && self.hunt && t - self.hunt_t < 9.0 && !must_zone && d2(&[self.hunt_pt.0, self.hunt_pt.1], &nc) <= nr + 40.0 {
                if d2(&p2, &[self.hunt_pt.0, self.hunt_pt.1]) > 12.0 {
                    goal = Some(self.hunt_pt);
                    boost = energy > 50.0;
                } else {
                    self.hunt_t = -100.0;
                }
            }
            let roamer = if self.roam >= 1.0 { true } else if self.roam <= 0.0 { false } else { self.roamer };
            if goal.is_none() && roamer && !must_zone {
                let zc = zone.get("center").v2();
                let zr = zone.get("radius").f().min(520.0);
                let far = d2(&p2, &[self.wander.0, self.wander.1]);
                if t - self.wander_t > 25.0 || far < 15.0 || d2(&[self.wander.0, self.wander.1], &nc) > nr + 30.0 {
                    self.wander_t = t;
                    for _ in 0..8 {
                        let ang = self.rng.range(-PI, PI);
                        let r = zr.min(nr + 30.0) * self.rng.f().sqrt() * 0.9;
                        let base = if nr < zr { nc } else { zc };
                        let cand = (base[0] + ang.cos() * r, base[1] + ang.sin() * r);
                        if self.map.wall_dist(cand.0, cand.1) >= 3.5 {
                            self.wander = cand;
                            break;
                        }
                    }
                }
                goal = Some(self.wander);
                boost = energy > 35.0;
            }
            if goal.is_none() {
                let dc = d2(&p2, &nc);
                if dc > (0.35 * nr).max(8.0) {
                    goal = Some((nc[0], nc[1]));
                    boost = energy > 60.0 || must_zone;
                } else {
                    if t - self.wander_t > 6.0 {
                        self.wander_t = t;
                        let ang = self.rng.range(-PI, PI);
                        let r = self.rng.range(0.0, 0.3 * nr);
                        self.wander = (nc[0] + ang.cos() * r, nc[1] + ang.sin() * r);
                    }
                    goal = Some(self.wander);
                }
            }
            if let Some(gp) = goal {
                self.route(p2, gp, t);
                mv = self.follow(p2);
            }
            let base = if hyp(mv[0], mv[1]) > 0.1 { mv[1].atan2(mv[0]) } else { self.last_aim[0] + 0.15 };
            a.aim = [base + 0.6 * (t * 1.7).sin(), 0.0];
            if self.look > 0.5 && (t < 1.7 || (t % 11.0) < 1.7) {
                // look all the way round at the start and now and then
                a.aim = [aim_now[0] + 1.3, 0.0];
            }
            for (i, w) in &weapons {
                if w.get("kind").s() == "gun" && w.get("ammo").f() < 6.0 {
                    a.reload = true;
                    let _ = i;
                }
            }
        }
        // crude unstick
        if hyp(mv[0], mv[1]) > 0.5 && d2(&self.last_pos, &p2) < 0.05 {
            self.still += 1;
        } else {
            self.still = 0;
        }
        self.last_pos = p2;
        if self.still > 8 {
            a.jump = true;
            boost = energy > 5.0;
            let ang = self.rng.range(-PI, PI);
            mv = [ang.cos(), ang.sin()];
            self.plan_t = -100.0;
        }
        if self.dodge {
            let inc = crate::brain::read_projectiles(o);
            if !inc.is_empty() && me.get("grounded").b() {
                let v = me.get("vel").v3();
                if let Some((dir, b)) = crate::brain::evade(&self.map, p, v, st.get("radius").fo(2.5), st.get("height").fo(9.0), st.get("walk_speed").fo(11.7), st.get("boost_speed").fo(19.5), energy, mv, boost, &inc, 7.0) {
                    mv = dir;
                    boost = b;
                }
            }
        }
        a.mv = mv;
        a.boost = boost;
        a.fire = fire;
        self.last_aim = a.aim;
        a
    }
}
