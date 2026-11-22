//! Test harness: local matches against guessed opponents. Not part of the submitted bot.
use mechbot::brain::{Brain, Params};
use mechbot::geom::Map;
use mechbot::json;
use mechbot::sim::{Ctl, Example, House, Sim, HELLO};
use std::io::Write;
use std::sync::Arc;

struct Spec {
    kind: String,
    params: String,
}

fn parse_spec(s: &str) -> Vec<Spec> {
    let mut out = Vec::new();
    for tok in s.split(',') {
        let (body, n) = match tok.split_once('*') {
            Some((b, n)) => (b, n.parse::<usize>().unwrap_or(1)),
            None => (tok, 1),
        };
        let (kind, params) = match body.split_once('[') {
            Some((k, p)) => (k.to_string(), p.trim_end_matches(']').replace(';', "\n")),
            None => (body.to_string(), String::new()),
        };
        for _ in 0..n {
            out.push(Spec { kind: kind.clone(), params: params.clone() });
        }
    }
    out
}

fn make(spec: &Spec, map: &Arc<Map>, id: usize, seed: u64) -> Box<dyn Ctl> {
    let mut hello = json::parse(HELLO).unwrap();
    if let json::J::Obj(v) = &mut hello {
        for (k, x) in v.iter_mut() {
            if k == "slot" {
                *x = json::J::Num(id as f64);
            }
            if k == "seed" {
                *x = json::J::Num((seed * 31 + id as u64 * 7919) as f64);
            }
        }
    }
    match spec.kind.as_str() {
        "brain" => {
            let mut p = Params::default();
            p.apply(&spec.params);
            Box::new(Brain::new(&hello, map.clone(), p))
        }
        "example" => Box::new(Example::new()),
        _ => {
            let mut skill = 0.008;
            let mut aggro = 200.0;
            for line in spec.params.lines() {
                if let Some((k, v)) = line.split_once('=') {
                    match k.trim() {
                        "skill" => skill = v.trim().parse().unwrap_or(skill),
                        "aggro" => aggro = v.trim().parse().unwrap_or(aggro),
                        _ => {}
                    }
                }
            }
            let mut h = House::new(map.clone(), seed.wrapping_mul(6364136223846793005).wrapping_add(id as u64 * 1442695040888963407), skill, aggro);
            for line in spec.params.lines() {
                if let Some((k, v)) = line.split_once('=') {
                    if k.trim() == "duty" {
                        h.duty = v.trim().parse().unwrap_or(h.duty);
                    }
                    if k.trim() == "dodge" {
                        h.dodge = v.trim() != "0";
                    }
                    if k.trim() == "hunt" {
                        h.hunt = v.trim() != "0";
                    }
                    if k.trim() == "look" {
                        h.look = v.trim().parse().unwrap_or(h.look);
                    }
                    if k.trim() == "roam" {
                        h.roam = v.trim().parse().unwrap_or(h.roam);
                    }
                    if k.trim() == "push" {
                        h.push = v.trim().parse().unwrap_or(h.push);
                    }
                    if k.trim() == "focus" {
                        h.focus = v.trim().parse().unwrap_or(-1);
                    }
                    if k.trim() == "gate" {
                        h.gate = v.trim().parse().unwrap_or(h.gate);
                    }
                    if k.trim() == "retreat" {
                        h.retreat = v.trim().parse().unwrap_or(h.retreat);
                    }
                }
            }
            Box::new(h)
        }
    }
}

/// Engagement profile of one kind of bot, by range bucket (<40, <80, <120, <200 m):
/// frames, closing speed, lateral speed, speed, boosting, airborne, shots, hits.
static PROF: std::sync::Mutex<[[f64; 8]; 4]> = std::sync::Mutex::new([[0.0; 8]; 4]);

fn prof_bucket(d: f64) -> Option<usize> {
    [40.0, 80.0, 120.0, 200.0].iter().position(|&b| d < b)
}

struct Out {
    place: usize,
    death_t: f64,
    death_by: String,
    deaths: Vec<f64>,
    max_us: u128,
    sum_us: u128,
    decisions: u64,
    zone_dmg: f64,
    stuck: u32,
    cubes: f64,
    kills: u32,
    dmg: f64,
}

fn run(map: &Arc<Map>, seed: u64, specs: &[Spec], trace: Option<&str>, verbose: bool) -> Out {
    let n = specs.len();
    let mut sim = Sim::new(map.clone(), seed, n);
    if let Ok(sp) = std::env::var("SIM_SPAWN") {
        let v: Vec<f64> = sp.split(',').filter_map(|x| x.parse().ok()).collect();
        if v.len() >= 2 {
            sim.mechs[0].pos = [v[0], v[1], map.ground(v[0], v[1])];
        }
    }
    if let Ok(v) = std::env::var("SIM_HOUSE_LASER") {
        // share of house bots that start with a laser already in hand
        let share: f64 = v.parse().unwrap_or(1.0);
        for i in 0..n {
            if specs[i].kind == "house" && ((i * 7919 + seed as usize * 31) % 100) as f64 / 100.0 < share {
                sim.mechs[i].w[1] = Some(mechbot::sim::Weapon { kind: 2, cooldown: 0.0, ammo: 0.0, reload: 0.0, charge: 0.0, held_full: 0.0 });
            }
        }
    }
    let mut ctls: Vec<Box<dyn Ctl>> = (0..n).map(|i| make(&specs[i], map, i, seed)).collect();
    let mut tr = trace.map(|p| std::fs::File::create(p).unwrap());
    let (mut max_us, mut sum_us, mut decisions) = (0u128, 0u128, 0u64);
    let prof_kind = std::env::var("SIM_PROF").ok();
    let mut prev: Vec<([f64; 3], u32, u32)> = sim.mechs.iter().map(|m| (m.pos, m.shots, m.hits)).collect();
    while !sim.finished() {
        if sim.tick % 2 == 0 {
            if let Some(kind) = prof_kind.as_deref() {
                let mut acc = PROF.lock().unwrap();
                for i in 0..n {
                    let m = &sim.mechs[i];
                    if !m.alive || specs[i].kind != kind {
                        continue;
                    }
                    let eye = [m.pos[0], m.pos[1], m.pos[2] + 8.0];
                    let mut tg: Option<(f64, usize, f64)> = None;
                    for j in 0..n {
                        let o = &sim.mechs[j];
                        if j == i || !o.alive {
                            continue;
                        }
                        let d = (o.pos[0] - m.pos[0]).hypot(o.pos[1] - m.pos[1]);
                        if d > 200.0 {
                            continue;
                        }
                        let aa = mechbot::brain::wrap(m.aim[0] - (o.pos[1] - m.pos[1]).atan2(o.pos[0] - m.pos[0])).abs();
                        if aa < 0.08 && tg.map_or(true, |t| aa < t.0) && map.los(eye, [o.pos[0], o.pos[1], o.pos[2] + 4.5]) {
                            tg = Some((aa, j, d));
                        }
                    }
                    if let Some((_, j, d)) = tg {
                        if let Some(b) = prof_bucket(d) {
                            let o = &sim.mechs[j];
                            let (pa, pb) = (prev[i].0, prev[j].0);
                            let d0 = (pb[0] - pa[0]).hypot(pb[1] - pa[1]);
                            let (vx, vy) = ((m.pos[0] - pa[0]) / 0.1, (m.pos[1] - pa[1]) / 0.1);
                            let (ux, uy) = ((o.pos[0] - m.pos[0]) / d.max(0.1), (o.pos[1] - m.pos[1]) / d.max(0.1));
                            let a = &mut acc[b];
                            a[0] += 1.0;
                            a[1] += (d0 - d) / 0.1;
                            a[2] += (-vx * uy + vy * ux).abs();
                            a[3] += vx.hypot(vy);
                            a[4] += if m.act.boost { 1.0 } else { 0.0 };
                            a[5] += if m.grounded { 0.0 } else { 1.0 };
                            a[6] += (m.shots - prev[i].1) as f64;
                            a[7] += (m.hits - prev[i].2) as f64;
                        }
                    }
                }
                for i in 0..n {
                    let m = &sim.mechs[i];
                    prev[i] = (m.pos, m.shots, m.hits);
                }
            }
            for i in 0..n {
                if !sim.mechs[i].alive {
                    continue;
                }
                let obs = sim.obs_for(i);
                let t0 = std::time::Instant::now();
                let a = ctls[i].act(&obs);
                if i == 0 {
                    let us = t0.elapsed().as_micros();
                    max_us = max_us.max(us);
                    sum_us += us;
                    decisions += 1;
                    if let Some(f) = tr.as_mut() {
                        let m = &sim.mechs[0];
                        let _ = writeln!(f, "{:.2},{:.2},{:.2},{:.2},{:.0},{},{},{}", sim.t, m.pos[0], m.pos[1], m.pos[2], m.hp, sim.alive, a.label, us);
                    }
                }
                sim.apply_action(i, a);
            }
        }
        sim.step();
    }
    sim.finalize();
    if verbose {
        println!("  zone c ({:.0},{:.0}) r {:.0} next ({:.0},{:.0}) r {:.0} phase {} t {:.1}", sim.zc[0], sim.zc[1], sim.zr, sim.znc[0], sim.znc[1], sim.znr, sim.phase, sim.t);
        for m in &sim.mechs {
            println!(
                "  mech {:2} {:8} place {:2} t {:6.1} by {:18} kills {} dmg {:6.0} taken {:6.0} zone {:5.0} cubes {} at ({:.0},{:.0},{:.0}) upg {:?} w {:?}",
                m.id,
                specs[m.id].kind,
                m.place,
                m.death_t,
                m.death_by,
                m.kills,
                m.dmg_dealt,
                m.dmg_taken,
                m.zone_dmg,
                m.cubes,
                m.pos[0],
                m.pos[1],
                m.pos[2],
                m.upg,
                m.w.iter().flatten().map(|w| mechbot::sim::KINDS[w.kind]).collect::<Vec<_>>()
            );
        }
    }
    if verbose {
        println!("  gun fired by range bucket {:?} hit {:?} | laser fired {} hit {}", sim.gun_fired, sim.gun_hit, sim.laser_fired, sim.laser_hit);
    }
    let m = &sim.mechs[0];
    let mut deaths: Vec<f64> = sim.death_log.iter().map(|d| d.0).collect();
    deaths.sort_by(|a, b| a.partial_cmp(b).unwrap());
    Out { place: m.place, death_t: m.death_t, death_by: m.death_by.clone(), deaths, max_us, sum_us, decisions, zone_dmg: m.zone_dmg, stuck: m.stuck_ticks, cubes: m.cubes, kills: m.kills, dmg: m.dmg_dealt }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let cmd = args.get(1).map(|s| s.as_str()).unwrap_or("");
    match cmd {
        "map" => {
            let m = Map::load(&args[2]);
            let mut out = Vec::with_capacity(m.nx * m.ny * 3 + 32);
            out.extend_from_slice(format!("P6\n{} {}\n255\n", m.nx, m.ny).as_bytes());
            for j in (0..m.ny).rev() {
                for i in 0..m.nx {
                    let v = j * m.nx + i;
                    let px: [u8; 3] = if m.wall[v] != 0 {
                        let h = ((m.top[v] - m.terr[v]) * 3.0).clamp(0.0, 200.0) as u8;
                        [40 + h / 4, 30, 30 + h]
                    } else {
                        let t = ((m.terr[v] + 10.0) * 6.0).clamp(0.0, 255.0) as u8;
                        let c = if m.wdist[v] < 3.2 { 60 } else { 0 };
                        [200 - c, (120 + t / 2).min(255) - c, 150 - c]
                    };
                    out.extend_from_slice(&px);
                }
            }
            std::fs::File::create(&args[3]).unwrap().write_all(&out).unwrap();
        }
        "comps" => {
            let m = Map::load("arena");
            let nav = mechbot::nav::Nav::new(&m);
            let mut sizes: std::collections::BTreeMap<u32, usize> = Default::default();
            for &c in &nav.comp {
                if c != 0 {
                    *sizes.entry(c).or_insert(0) += 1;
                }
            }
            let mut v: Vec<(usize, u32)> = sizes.iter().map(|(k, n)| (*n, *k)).collect();
            v.sort_by(|a, b| b.cmp(a));
            println!("components: {} main {} sizes {:?}", v.len(), nav.main_comp, v.iter().take(12).collect::<Vec<_>>());
            if let Some(path) = args.get(2) {
                let mut out = Vec::new();
                out.extend_from_slice(format!("P6\n{} {}\n255\n", nav.w * 2, nav.h * 2).as_bytes());
                for j in (0..nav.h * 2).rev() {
                    for i in 0..nav.w * 2 {
                        let c = (j / 2) * nav.w + i / 2;
                        let k = nav.comp[c];
                        let px: [u8; 3] = if k == 0 {
                            if nav.clear[c] <= 0.0 { [30, 30, 60] } else { [110, 110, 110] }
                        } else if k == nav.main_comp {
                            [215, 215, 200]
                        } else {
                            let h = k.wrapping_mul(2654435761);
                            [(h & 0xff) as u8 | 0x80, ((h >> 8) & 0xff) as u8 / 2, ((h >> 16) & 0xff) as u8 | 0x40]
                        };
                        out.extend_from_slice(&px);
                    }
                }
                std::fs::File::create(path).unwrap().write_all(&out).unwrap();
            }
            for (n, k) in v.iter().skip(1).take(10) {
                let c = nav.comp.iter().position(|x| x == k).unwrap();
                println!("  comp {} size {} e.g. at {:?}", k, n, nav.pos_of(c as u32));
            }
        }
        "visat" => {
            // share of ground 50..195 m away that can see a mech standing at each given point
            let m = Map::load("arena");
            let mut buf: Vec<u16> = Vec::new();
            let mut k = 2;
            while k + 1 < args.len() {
                let (x, y): (f64, f64) = (args[k].parse().unwrap(), args[k + 1].parse().unwrap());
                k += 2;
                let z = m.walk_z(x, y) as f64;
                let (mid, top) = ([x, y, z + 4.5], [x, y, z + 8.9]);
                let mut res = Vec::new();
                for &cm in &[3.0f64, 4.0, 5.0, 1e9] {
                    let (mut seen, mut n, mut seen_cyl) = (0, 0, 0);
                    for &r in &[50.0f64, 70.0, 95.0, 125.0, 160.0, 195.0] {
                        for j in 0..32 {
                            let a = j as f64 * std::f64::consts::PI / 16.0 + r * 0.01;
                            let (vx, vy) = (x + a.cos() * r, y + a.sin() * r);
                            if vx < m.bx0 + 1.0 || vx > m.bx1 - 1.0 || vy < m.by0 + 1.0 || vy > m.by1 - 1.0 || m.wall_dist(vx, vy) < 1.0 {
                                continue;
                            }
                            let eye = [vx, vy, m.walk_z(vx, vy) as f64 + 8.0];
                            n += 1;
                            if (m.canopy_len(eye, mid, cm, &mut buf) < cm && m.los(eye, mid)) || (m.canopy_len(eye, top, cm, &mut buf) < cm && m.los(eye, top)) {
                                seen += 1;
                            }
                            if m.los_vis_m(eye, mid, cm) || m.los_vis_m(eye, top, cm) {
                                seen_cyl += 1;
                            }
                        }
                    }
                    res.push(format!("{}m: ell {:.2} cyl {:.2}", if cm > 1e8 { "none".to_string() } else { format!("{cm}") }, seen as f64 / n.max(1) as f64, seen_cyl as f64 / n.max(1) as f64));
                }
                println!("({x:.0},{y:.0}) wall {:.0} m | {}", m.wall_dist(x, y), res.join(" | "));
            }
        }
        "hides" => {
            let t0 = std::time::Instant::now();
            let m = Map::load("arena");
            let nav = mechbot::nav::Nav::new(&m);
            let t1 = std::time::Instant::now();
            let cm: f64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(4.6);
            let h = mechbot::brain::find_hides(&m, &nav, cm);
            println!("trees {} | hide spots {} (perfect {}) | map+nav {:?}, hides {:?}", m.trees.len(), h.len(), h.iter().filter(|x| x.vis == 0.0).count(), t1 - t0, t1.elapsed());
            let near = |x: f64, y: f64| h.iter().filter(|s| (s.x - x).hypot(s.y - y) < 40.0).map(|s| format!("({:.0},{:.0}) vis {:.2}", s.x, s.y, s.vis)).collect::<Vec<_>>();
            println!("near (235,-240): {:?}", near(235.0, -240.0));
            // how far is the nearest hide spot from random walkable points
            let mut ds: Vec<f64> = Vec::new();
            let mut k = 0u64;
            while ds.len() < 400 {
                k += 1;
                let x = -400.0 + ((k * 7919) % 880) as f64;
                let y = -300.0 + ((k * 104729) % 660) as f64;
                if m.wall_dist(x, y) < 3.5 {
                    continue;
                }
                ds.push(h.iter().map(|s| (s.x - x).hypot(s.y - y)).fold(1e9, f64::min));
            }
            ds.sort_by(|a, b| a.partial_cmp(b).unwrap());
            println!("distance to nearest hide spot: median {:.0} m, p90 {:.0} m, max {:.0} m", ds[200], ds[360], ds[399]);
        }
        "roofs" => {
            let t0 = std::time::Instant::now();
            let m = Map::load("arena");
            println!("map load incl. roofs: {:?}; regions {}", t0.elapsed(), m.roofs.len());
            let mut good: Vec<&mechbot::geom::Roof> = m.roofs.iter().filter(|r| r.depth >= 7.0 && r.rise >= 9.0 && !r.access.is_empty()).collect();
            good.sort_by(|a, b| b.depth.partial_cmp(&a.depth).unwrap());
            println!("usable (depth>=7, rise>=9, has access): {}", good.len());
            let climbable = good.iter().filter(|r| r.rise <= 48.0).count();
            println!("  of which rise<=48 m: {}", climbable);
            for r in good.iter().take(14) {
                println!("  roof {} top {:.1} rise {:.1} depth {:.1} area {:.0} best ({:.0},{:.0}) access {}", r.id, r.top, r.rise, r.depth, r.area, r.best.0, r.best.1, r.access.len());
            }
            let hist = |lo: f32, hi: f32| good.iter().filter(|r| r.rise >= lo && r.rise < hi).count();
            println!("rise histogram: 9-15: {}, 15-25: {}, 25-35: {}, 35-48: {}, 48+: {}", hist(9.0, 15.0), hist(15.0, 25.0), hist(25.0, 35.0), hist(35.0, 48.0), hist(48.0, 999.0));
        }
        "patch" => dump_patch(args[2].parse().unwrap(), args[3].parse().unwrap()),
        "replay" => {
            // feed a recorded stream of real observations to the brain and print what it decides
            let text = std::fs::read_to_string(&args[2]).unwrap();
            let map = Arc::new(Map::load("arena"));
            let mut prm = Params::default();
            prm.apply(&args.get(3).cloned().unwrap_or_default().replace(';', "\n"));
            prm.debug = 1.0;
            let mut brain: Option<Brain> = None;
            for line in text.lines() {
                let m = match json::parse(line) {
                    Ok(m) => m,
                    Err(_) => continue,
                };
                match m.get("type").s() {
                    "hello" => brain = Some(Brain::new(&m, map.clone(), prm.clone())),
                    "obs" => {
                        if let Some(b) = brain.as_mut() {
                            let a = b.decide(&m);
                            let me = m.get("self");
                            println!(
                                "t={:5.1} hp={:6.0} mv=({:5.2},{:5.2}) boost={} jump={} aim=({:5.2},{:5.2}) fire={:?} rl={} | {}",
                                m.get("t").f(),
                                me.get("hp").f(),
                                a.mv[0],
                                a.mv[1],
                                a.boost as u8,
                                a.jump as u8,
                                a.aim[0],
                                a.aim[1],
                                a.fire,
                                a.reload as u8,
                                a.label
                            );
                        }
                    }
                    _ => {}
                }
            }
        }
        "match" => {
            let map = Arc::new(Map::load("arena"));
            let seed: u64 = args[2].parse().unwrap();
            let specs = parse_spec(&args[3]);
            let o = run(&map, seed, &specs, args.get(4).map(|s| s.as_str()), true);
            println!("subject place {} t {:.1} by {} | decide mean {} us max {} us | deaths at {:?}", o.place, o.death_t, o.death_by, o.sum_us / o.decisions.max(1) as u128, o.max_us, o.deaths.iter().map(|d| d.round() as i64).collect::<Vec<_>>());
        }
        "batch" => {
            let map = Arc::new(Map::load("arena"));
            let n: u64 = args[2].parse().unwrap();
            let seed0: u64 = args[3].parse().unwrap();
            let specs = parse_spec(&args[4]);
            let mut pts = 0.0;
            let mut places = vec![0u32; specs.len() + 1];
            let mut by: std::collections::BTreeMap<String, u32> = Default::default();
            let (mut max_us, mut sum_us, mut dec) = (0u128, 0u128, 0u64);
            let mut death_sum = 0.0;
            let mut nth = vec![0.0f64; specs.len()];
            let (mut zone, mut stuck, mut cubes, mut kills, mut dmg) = (0.0, 0u64, 0.0, 0u32, 0.0);
            for k in 0..n {
                let o = run(&map, seed0 + k, &specs, None, false);
                if std::env::var("SIM_VERBOSE").is_ok() {
                    println!("seed {} place {} t {:.0} by {} kills {} dmg {:.0} zone {:.0}", seed0 + k, o.place, o.death_t, o.death_by, o.kills, o.dmg, o.zone_dmg);
                }
                pts += (specs.len() - o.place) as f64;
                places[o.place] += 1;
                let cause = if o.death_by.contains(" by ") { o.death_by.split(' ').next().unwrap_or("?").to_string() } else { o.death_by.clone() };
                *by.entry(cause).or_insert(0) += 1;
                max_us = max_us.max(o.max_us);
                sum_us += o.sum_us;
                dec += o.decisions;
                death_sum += o.death_t;
                for (i, d) in o.deaths.iter().enumerate() {
                    if i < nth.len() {
                        nth[i] += d;
                    }
                }
                zone += o.zone_dmg;
                stuck += o.stuck as u64;
                cubes += o.cubes;
                kills += o.kills;
                dmg += o.dmg;
            }
            let nf = n as f64;
            println!("{} x{}: mean points {:.2} | mean death t {:.0}s | places {:?}", specs[0].kind, n, pts / nf, death_sum / nf, &places[1..]);
            println!("  causes {:?} | zone dmg {:.0} | stuck ticks {:.0} | cubes {:.1} | kills {:.2} | dmg dealt {:.0}", by, zone / nf, stuck as f64 / nf, cubes / nf, kills as f64 / nf, dmg / nf);
            println!("  decide mean {} us, max {} us | mean time of k-th death: {:?}", sum_us / dec.max(1) as u128, max_us, nth.iter().take(15).map(|d| (d / nf).round() as i64).collect::<Vec<_>>());
        }
        _ => eprintln!("usage: sim map <dir> <out.ppm> | match <seed> <spec> [trace.csv] | batch <n> <seed0> <spec>"),
    }
    if std::env::var("SIM_PROF").is_ok() {
        let acc = PROF.lock().unwrap();
        for (b, name) in ["<40", "<80", "<120", "<200"].iter().enumerate() {
            let a = acc[b];
            let f = a[0].max(1.0);
            println!("  prof {:5} s{:.0} close{:.1} lat{:.1} spd{:.1} bst{:.2} air{:.2} shots/s{:.2} hits/s{:.2}", name, a[0] / 10.0, a[1] / f, a[2] / f, a[3] / f, a[4] / f, a[5] / f, a[6] / (f / 10.0), a[7] / (f / 10.0));
        }
    }
}
// (debug helper appended) print a patch of the nav grid
#[allow(dead_code)]
pub fn dump_patch(cx: f64, cy: f64) {
    let m = Map::load("arena");
    let nav = mechbot::nav::Nav::new(&m);
    let c = nav.cell_of(cx, cy) as usize;
    let (ci, cj) = ((c % nav.w) as i32, (c / nav.w) as i32);
    for dj in (-14..=14).rev() {
        let mut line = String::new();
        for di in -30..=30 {
            let (i, j) = (ci + di, cj + dj);
            let k = j as usize * nav.w + i as usize;
            let ch = if di == 0 && dj == 0 { '@' } else if nav.clear[k] <= 0.0 { '#' } else if nav.clear[k] < 2.8 { '+' } else if nav.clear[k] < 3.4 { ':' } else if nav.comp[k] == nav.main_comp { '.' } else { 'o' };
            line.push(ch);
        }
        println!("{line}");
    }
    let v = m.vidx(cx, cy).unwrap();
    println!("at ({cx},{cy}): terr {:.1} top {:.1} wdist {:.1}; tallest within 30 m: {:.1}", m.terr[v], m.top[v], m.wdist[v], (0..m.nx * m.ny).filter(|&u| { let x = m.min_x + (u % m.nx) as f64; let y = m.min_y + (u / m.nx) as f64; (x - cx).hypot(y - cy) < 30.0 }).map(|u| m.top[u]).fold(f32::MIN, f32::max));
}
