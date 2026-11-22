//! Decision logic: world tracking, position planning, movement, aiming and fire control.

use crate::geom::Map;
use std::sync::Arc;
use crate::json::J;
use crate::nav::{self, Nav};
use std::f64::consts::PI;
use std::fmt::Write as _;

pub const GUN: usize = 0;
pub const MORTAR: usize = 1;
pub const LASER: usize = 2;
pub const LIGHTNING: usize = 3;

pub fn wrap(a: f64) -> f64 {
    let mut a = a % (2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    } else if a <= -PI {
        a += 2.0 * PI;
    }
    a
}
#[inline]
fn hyp(a: f64, b: f64) -> f64 {
    (a * a + b * b).sqrt()
}
#[inline]
fn d2(a: &[f64], b: &[f64]) -> f64 {
    hyp(a[0] - b[0], a[1] - b[1])
}
fn fin(x: f64) -> f64 {
    if x.is_finite() {
        x
    } else {
        0.0
    }
}
fn wkind(s: &str) -> usize {
    match s {
        "mortar" => MORTAR,
        "laser" => LASER,
        "lightning" => LIGHTNING,
        _ => GUN,
    }
}

pub struct Rng(pub u64);
impl Rng {
    pub fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }
    pub fn f(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.f()
    }
}

/// One incoming projectile as seen in an observation.
#[derive(Clone, Copy, Debug)]
pub struct Incoming {
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub mortar: bool,
}

pub fn read_projectiles(o: &J) -> Vec<Incoming> {
    o.get("projectiles").arr().iter().map(|p| Incoming { pos: p.get("pos").v3(), vel: p.get("vel").v3(), mortar: p.get("kind").s() == "mortar" }).collect()
}

/// Where a mech ends up after `t` seconds if it starts at p with velocity v0 and steers for velocity u.
fn pos_after(p: [f64; 2], v0: [f64; 2], u: [f64; 2], accel: f64, t: f64) -> [f64; 2] {
    let dv = [u[0] - v0[0], u[1] - v0[1]];
    let l = hyp(dv[0], dv[1]);
    if l < 1e-6 {
        return [p[0] + v0[0] * t, p[1] + v0[1] * t];
    }
    let tau = l / accel;
    let ta = t.min(tau);
    let a = [dv[0] / l * accel, dv[1] / l * accel];
    let mut x = p[0] + v0[0] * ta + 0.5 * a[0] * ta * ta;
    let mut y = p[1] + v0[1] * ta + 0.5 * a[1] * ta * ta;
    if t > tau {
        x += u[0] * (t - tau);
        y += u[1] * (t - tau);
    }
    [x, y]
}

/// Sidestep incoming bullets and shells. `intent` is the move we would otherwise make (unit-ish
/// vector scaled by speed fraction). Returns a replacement (move, boost) when the intent gets hit.
#[allow(clippy::too_many_arguments)]
pub fn evade(map: &Map, pos: [f64; 3], vel: [f64; 3], radius: f64, height: f64, walk: f64, boost_speed: f64, energy: f64, intent: [f64; 2], intent_boost: bool, projs: &[Incoming], splash: f64) -> Option<([f64; 2], bool)> {
    if projs.is_empty() {
        return None;
    }
    let hit_r = radius + 0.9;
    let p2 = [pos[0], pos[1]];
    let v0 = [vel[0], vel[1]];
    // (time, kind, data)
    let mut threats: Vec<(f64, bool, [f64; 3], [f64; 3])> = Vec::new();
    for pr in projs {
        if pr.mortar {
            // time to reach our feet level under gravity 22
            let g = 22.0;
            let dz = pr.pos[2] - (pos[2] + 1.0);
            let disc = pr.vel[2] * pr.vel[2] + 2.0 * g * dz;
            if disc < 0.0 {
                continue;
            }
            let t = (pr.vel[2] + disc.sqrt()) / g;
            if !(0.0..=3.0).contains(&t) {
                continue;
            }
            let land = [pr.pos[0] + pr.vel[0] * t, pr.pos[1] + pr.vel[1] * t, 0.0];
            if d2(&land, &p2) < splash + radius + 14.0 {
                threats.push((t, true, land, [0.0; 3]));
            }
        } else {
            let sp2 = pr.vel[0] * pr.vel[0] + pr.vel[1] * pr.vel[1];
            if sp2 < 1.0 {
                continue;
            }
            let t = ((p2[0] - pr.pos[0]) * pr.vel[0] + (p2[1] - pr.pos[1]) * pr.vel[1]) / sp2;
            if !(0.0..=1.3).contains(&t) {
                continue;
            }
            let z = pr.pos[2] + pr.vel[2] * t;
            if z < pos[2] - 1.5 || z > pos[2] + height + 1.5 {
                continue;
            }
            // quick reject: far from the line even if we ran flat out
            let cx = pr.pos[0] + pr.vel[0] * t;
            let cy = pr.pos[1] + pr.vel[1] * t;
            if hyp(cx - p2[0], cy - p2[1]) > hit_r + boost_speed * t + 2.0 {
                continue;
            }
            threats.push((t, false, pr.pos, pr.vel));
        }
    }
    if threats.is_empty() {
        return None;
    }
    let score = |u: [f64; 2]| -> f64 {
        let mut worst = f64::INFINITY;
        for (t, mortar, a, v) in &threats {
            let q = pos_after(p2, v0, u, 58.5, *t);
            let miss = if *mortar {
                // want to be outside the blast: scale so that "hit_r" means the splash edge
                d2(&q, a) * hit_r / (splash + radius)
            } else {
                // distance from q to the bullet line
                let l = hyp(v[0], v[1]).max(1e-6);
                ((q[0] - a[0]) * v[1] - (q[1] - a[1]) * v[0]).abs() / l
            };
            if miss < worst {
                worst = miss;
            }
        }
        worst
    };
    let il = hyp(intent[0], intent[1]);
    let ispeed = if intent_boost { boost_speed } else { walk };
    let base = score([intent[0] * ispeed, intent[1] * ispeed]);
    if base >= hit_r {
        return None;
    }
    let mut best: Option<(f64, [f64; 2], bool)> = None;
    let can_boost = energy > 12.0;
    for k in 0..16 {
        let a = k as f64 * PI / 8.0;
        let dir = [a.cos(), a.sin()];
        if map.wall_dist(pos[0] + dir[0] * 3.5, pos[1] + dir[1] * 3.5) < (radius + 0.5) as f32 {
            continue;
        }
        for &b in &[false, true] {
            if b && !can_boost {
                continue;
            }
            let sp = if b { boost_speed } else { walk };
            let sc = score([dir[0] * sp, dir[1] * sp]).min(hit_r + 2.0);
            let align = if il > 0.05 { (dir[0] * intent[0] + dir[1] * intent[1]) / il } else { 0.0 };
            let total = sc + 0.35 * align - if b { 0.25 } else { 0.0 };
            if best.map_or(true, |x| total > x.0) {
                best = Some((total, dir, b));
            }
        }
    }
    best.filter(|b| b.0 > base + 0.3).map(|b| (b.1, b.2))
}

/// Spots under the trees where a mech cannot be picked out from a distance. Crowns are ellipsoids;
/// a sight line is lost after `canopy_m` metres of foliage. For each spot we keep the share of
/// surrounding ground (50 to 195 m out) from which an eye could still see the mech's middle or top.
pub fn find_hides(map: &Map, nav: &Nav, canopy_m: f64) -> Vec<HideSpot> {
    const RINGS: [f64; 6] = [50.0, 70.0, 95.0, 125.0, 160.0, 195.0];
    const ND: usize = 32;
    let mut cand: Vec<(f64, f64)> = Vec::new();
    let mut mark = vec![false; nav.w * nav.h];
    for tr in &map.trees {
        if tr.r < 2.5 || tr.top < tr.z + 9.0 {
            continue;
        }
        let r = tr.r as f64 + 1.5;
        let mut y = tr.y - r;
        while y <= tr.y + r {
            let mut x = tr.x - r;
            while x <= tr.x + r {
                if (x - tr.x).hypot(y - tr.y) <= r && x > map.bx0 + 6.0 && x < map.bx1 - 6.0 && y > map.by0 + 6.0 && y < map.by1 - 6.0 {
                    let c = nav.cell_of(x, y) as usize;
                    if c < mark.len() && !mark[c] && nav.clear[c] >= 3.2 && nav.comp[c] == nav.main_comp {
                        mark[c] = true;
                        cand.push(nav.pos_of(c as u32));
                    }
                }
                x += 2.0;
            }
            y += 2.0;
        }
    }
    let mut seen_buf: Vec<u16> = Vec::new();
    let mut spots: Vec<HideSpot> = Vec::new();
    for &(x, y) in &cand {
        let z = map.walk_z(x, y) as f64;
        let mid = [x, y, z + 4.5];
        let top = [x, y, z + 8.9];
        let (mut seen, mut n) = (0u32, 0u32);
        'rings: for &r in &RINGS {
            for k in 0..ND {
                let a = k as f64 * 2.0 * PI / ND as f64 + r * 0.01;
                let (vx, vy) = (x + a.cos() * r, y + a.sin() * r);
                if vx < map.bx0 + 1.0 || vx > map.bx1 - 1.0 || vy < map.by0 + 1.0 || vy > map.by1 - 1.0 || map.wall_dist(vx, vy) < 1.0 {
                    continue;
                }
                let eye = [vx, vy, map.walk_z(vx, vy) as f64 + 8.0];
                n += 1;
                let vis_mid = map.canopy_len(eye, mid, canopy_m, &mut seen_buf) < canopy_m && map.los(eye, mid);
                let vis_top = !vis_mid && map.canopy_len(eye, top, canopy_m, &mut seen_buf) < canopy_m && map.los(eye, top);
                if vis_mid || vis_top {
                    seen += 1;
                    if seen > 9 {
                        break 'rings;
                    }
                }
            }
        }
        if n < 40 || seen > 9 {
            continue;
        }
        let vis = seen as f32 / n as f32;
        if vis <= 0.045 {
            spots.push(HideSpot { x, y, vis });
        }
    }
    // keep the best of each neighbourhood
    spots.sort_by(|a, b| a.vis.partial_cmp(&b.vis).unwrap_or(std::cmp::Ordering::Equal));
    let mut out: Vec<HideSpot> = Vec::new();
    for s in spots {
        if !out.iter().any(|o| (o.x - s.x).hypot(o.y - s.y) < 7.0) {
            out.push(s);
        }
    }
    out
}

/// Tunables. Every one can be overridden from params.txt next to the binary (key=value lines).
#[derive(Clone, Debug)]
pub struct Params {
    /// 0: never fire first. 1: also finish weak targets. 2: also ambush unaware targets.
    pub ambush: f64,
    pub farm: f64,
    pub drops: f64,
    pub w_travel: f64,
    pub w_expo: f64,
    pub w_vis: f64,
    pub w_near: f64,
    pub center_avoid: f64,
    pub fight_margin: f64,
    pub finish_hp: f64,
    pub edge_margin: f64,
    pub end_radius: f64,
    pub spin: f64,
    pub cover_reach: f64,
    pub w_detour: f64,
    /// 1: move for the zone only when its edge is about to arrive, instead of at once
    pub lazy: f64,
    /// how far (walking metres) we will move to a better hiding spot when nothing is happening; 0 = stay put
    pub relocate: f64,
    /// open fire on a mech that can see us if the duel looks good (0 = wait until it attacks)
    pub sees_engage: f64,
    /// open fire on any mech closer than this (0 = wait until it attacks)
    pub pb_range: f64,
    /// diagnostics: idle = send nothing but the tick; nomove = never move; pacifist = never fire
    pub idle: f64,
    pub nomove: f64,
    pub pacifist: f64,
    pub debug: f64,
    /// 1: never stand and trade shots. Run from anyone who sees us, keep moving, keep our distance.
    pub kite: f64,
    /// how much being within far_d metres of a known enemy costs a spot
    pub w_far: f64,
    pub far_d: f64,
    /// seconds an unseen enemy stays on the books
    pub mem: f64,
    /// how far (walking metres) the escape search looks
    pub flee_reach: f64,
    /// treat nearby footsteps, jets and gunfire as enemies of unknown identity
    pub snd: f64,
    /// while kiting, shoot back only at attackers closer than this
    pub kite_fire: f64,
    /// a known enemy closer than this (even out of sight) gets us moving
    pub near_d: f64,
    /// distance we try to hold in a fight (0 = old rule: only close from 125 m, back off under 35 m)
    pub fight_dist: f64,
    /// 1: shoot crates and drones whenever no enemy is within 120 m, not only when nobody is in sight
    pub farm_loose: f64,
    /// 1: run straight to a better weapon we know of (within seek_range metres) when nobody is on us
    pub seek_w: f64,
    pub seek_range: f64,
    /// distance to hold in a fight while we carry a laser
    pub laser_dist: f64,
    /// 1: with a laser, open fire on any mech in range
    pub laser_engage: f64,
    /// gun stays quiet beyond this range unless the target is nearly still (0 = no limit)
    pub gun_max: f64,
    /// 1: look all the way round at the start and every so often
    pub scan: f64,
    /// 1: when nothing is happening, stand inside a dense tree crown (sight lines through foliage are blocked)
    pub tree: f64,
    /// how far we will go for a tree, and how long after the start we sprint for it
    pub tree_range: f64,
    pub tree_rush: f64,
    /// cost per metre of a known enemy being closer than 150 m to a tree
    pub w_tree_enemy: f64,
    /// foliage depth (m) we count on to hide us when ranking trees
    pub canopy_m: f64,
    /// diagnostic: 1 = if we ever answer late or skip a tick in the first 30 s, play dead for the rest of the match
    pub probe_late: f64,
    /// break off a fight below this share of max HP (0 = never), stay out of sight until back above resume_hp
    pub retreat_hp: f64,
    pub resume_hp: f64,
    /// 1: when nothing is happening, jet onto a deep flat roof and stand in the middle of it
    pub roof: f64,
    pub roof_depth: f64,
    pub roof_min_rise: f64,
    pub roof_range: f64,
    /// seconds between what we see and our shot leaving the barrel; both mechs keep moving meanwhile
    pub lat: f64,
    /// 1: when nobody is near, walk up to the nearest crate, drone or loose cube and take the cubes
    pub farm_seek: f64,
    pub farm_range: f64,
    /// 1: with nothing known to farm and nobody around, walk to a new spot to look for crates and drones
    pub explore: f64,
    /// farm only while no mech is closer than this
    pub farm_clear: f64,
    /// 1: whenever we are shooting at a mech, move like a fight (strafe, close to fighting distance) even before it shoots back
    pub press: f64,
    /// chance that a strafe burst is a sprint, and the slowest a burst may be (share of walking speed)
    pub sb: f64,
    pub strafe_min: f64,
    /// diagnostics that report through the score: 1 = play dead if our first 16 gun shots at mechs hit less
    /// than probe_thr; 2 = play dead if lasers did at least probe_thr of the first 300 damage we took;
    /// 3 = play dead if that first 300 damage came from two or more mechs
    pub probe: f64,
    pub probe_thr: f64,
    pub probe_amt: f64,
    pub probe_win: f64,
    pub probe_share: f64,
    /// 1: one fight at a time. With two or more mechs on us, break off and get out of sight; and never
    /// open a second front while one mech is already on us or two are in view.
    pub solo: f64,
    /// seconds to stay out of sight (no farming, no exploring) after breaking off
    pub lay_low: f64,
    /// keep exploring and farming outside this share of the next circle's radius (the middle is crowded)
    pub ring: f64,
    /// 1: top the gun up whenever nothing is in sight, so every fight starts on a full magazine
    pub topup: f64,
    /// seconds from the start during which getting a second weapon comes before everything:
    /// no opening fire, and the run for the weapon continues under fire unless a mech is on top of us
    pub arm_first: f64,
    /// seconds from the start during which we never open fire first (we still answer anyone who attacks from close)
    pub peace: f64,
    /// 1: during the peace, keep exploring away from every mech we know of instead of stopping when one is near
    pub away: f64,
    /// weight on picking sheltered spots (short sight lines) as exploring goals; 0 = any spot
    pub xlow: f64,
    /// 1: fight from cover. Find a spot the target cannot see next to one we can shoot from; wait out
    /// of sight until the weapons are ready, step out, shoot for at most peek_exp seconds, step back.
    pub peek: f64,
    pub peek_exp: f64,
    pub peek_reach: f64,
    /// behind cover and under this share of health: stay there instead of stepping out again
    pub peek_nurse: f64,
    /// 1: roam the whole current circle while there is time, and head for the next one only when the
    /// wall is due (the default walks inside the next circle the moment we are outside it)
    pub zroam: f64,
    pub zroam_pad: f64,
    /// 1: holding fire to fetch a weapon only applies while we actually know of one to fetch
    pub arm_known: f64,
    /// 1: the look around at the start, and each one that is due, comes before watching a far mech
    pub scan_first: f64,
    /// 1: a mech only counts as an attacker once it has hit us or is charging a beam at us (being
    /// looked at is not enough); how long that lasts after its last hit is `grudge` seconds
    pub calm_aim: f64,
    pub grudge: f64,
    /// 1: holding a lightning gun, close to `zap_dist` on whoever we are fighting instead of standing off
    pub zap: f64,
    pub zap_dist: f64,
    /// 1: without lightning of our own, stay `zap_fear_d` away from a fight target that holds one
    pub zap_fear: f64,
    pub zap_fear_d: f64,
    /// 1: finish what is already hurt. A mech under `chase_frac` of its health is fair game while we are
    /// above half ours, we close in on it, and we follow it for the 5 s the server keeps showing us
    /// where it is after it ducks out of sight. Every mech that dies before us is a place.
    pub chase: f64,
    pub chase_frac: f64,
    /// Keep this many metres from every mech we know of while we are not fighting (0 = off). In the
    /// public replays a mech whose nearest neighbour is over 150 m away takes almost no damage, one with
    /// a neighbour inside 100 m takes 30 a second. Nothing is fetched closer to a mech than this, and
    /// with one inside it we walk to the reachable spot furthest from all of them.
    pub space: f64,
    /// 1: take upgrades in an order that keeps us alive (hull, armour, repair, legs) before damage
    pub upg: f64,
    /// 1: go and capture supply drops (3 cubes, full health and energy for standing in the ring alone
    /// for 4 s). In the public replays the same one or two top bots take all three in a match.
    pub drop_go: f64,
    pub drop_range: f64,
    /// 1: when a mech dies within earshot (the sound carries 600 m and tells us roughly where), go and
    /// look: a dead mech leaves its cubes plus two, and every weapon it held
    pub scav: f64,
    /// no shooting at crates or drones before this time (a gun is heard 320 m away)
    pub farm_start: f64,
    /// seconds between full looks around (the server keeps a mech we have seen on our list for 5 s, so
    /// under 5 means we always know who is within sight). 0 = the old 9 s, and never while farming.
    pub scan_every: f64,
    /// Keep out of sight of mechs that are too far away to fight (a practice house bot opens up with a
    /// laser from 150 m a few seconds after the start). 1: while we have no laser, 2: always. A mech with
    /// a line on us from beyond `shy_d` means: get behind something (cover within outr_reach); and we
    /// only go after things that mechs seen in the last few seconds cannot see.
    pub shy: f64,
    pub shy_d: f64,
    /// 1: the long-range hush on the gun (gun_max) only applies to mechs that are not shooting at us;
    /// one that is gets everything we have (a practice house bot lands gun rounds from 150 m)
    pub gun_far: f64,
    /// 1: a laser is working on us from beyond `outr_d` and we have none: do not stand and trade (a
    /// practice house bot lands every beam from 150 m). Get behind cover within `outr_reach`, stay
    /// out of that mech's view while it is around, and only step out for it once it is in gun range.
    pub outr: f64,
    pub outr_d: f64,
    pub outr_reach: f64,
    /// below this share of health, with cover within `last_reach` and nobody on top of us, stop trading
    /// and get out of sight: every second we outlive someone else is a place (0 = fight to the end)
    pub last_hp: f64,
    pub last_reach: f64,
    /// 1: fetching a weapon never means running past someone who is shooting at us: under fire we only
    /// keep going when it is a few steps away or the shooter is far off, and we shoot back as we run
    pub arm_safe: f64,
    /// while healing below 60% health, fetch weapons only within this distance (cubes within 2/3 of it)
    pub heal_reach: f64,
    /// 1: aim and fire by the server's real tick order (public replays: the torso turns, the mech moves,
    /// then the weapon fires, on the first or second tick after a decision). Lead to the tick the shot
    /// leaves on, and pull the trigger whenever the torso can get there by then.
    pub tick_aim: f64,
    /// how much we want a laser and a lightning gun (3 and 2 by default). In the public replays of the
    /// top teams the laser does half of all damage and lightning 2%.
    pub laser_rank: f64,
    pub lightning_rank: f64,
    /// how far we go for a laser while we have none (seek_range for everything else)
    pub laser_range: f64,
    /// how much we want a mortar (laser 3, lightning 2). 0 = never carry one: a laser hit does 60% more
    /// damage to a mech that holds a mortar, and nearly everyone ends up with a laser.
    pub mortar_rank: f64,
    /// 1: only go after weapons, crates and cubes that lie in the walkable region we are standing in
    pub reach_fix: f64,
    /// seconds of target motion the beam is aimed ahead by when we let go (0.1 was the old value)
    pub laser_lead: f64,
    /// 1: while healing, walk to the most closed-in spot nearby instead of standing where we are
    pub heal_hide: f64,
    /// below this share of full health, with nobody on us, stop shooting crates and drones and stop
    /// opening fights until healed: health only comes back after 5 s without dealing or taking damage
    pub heal: f64,
}
impl Default for Params {
    fn default() -> Self {
        Params {
            ambush: 1.0,
            farm: 1.0,
            drops: 1.0,
            w_travel: 0.25,
            w_expo: 60.0,
            w_vis: 130.0,
            w_near: 1.5,
            center_avoid: 50.0,
            fight_margin: 1.15,
            finish_hp: 420.0,
            edge_margin: 0.2,
            end_radius: 30.0,
            spin: 1.6,
            cover_reach: 90.0,
            w_detour: 0.3,
            lazy: 0.0,
            relocate: 170.0,
            sees_engage: 1.0,
            pb_range: 38.0,
            idle: 0.0,
            nomove: 0.0,
            pacifist: 0.0,
            debug: 0.0,
            kite: 0.0,
            w_far: 0.0,
            far_d: 190.0,
            mem: 8.0,
            flee_reach: 130.0,
            snd: 0.0,
            kite_fire: 70.0,
            near_d: 110.0,
            fight_dist: 0.0,
            farm_loose: 0.0,
            seek_w: 0.0,
            seek_range: 160.0,
            laser_dist: 120.0,
            laser_engage: 0.0,
            gun_max: 0.0,
            scan: 0.0,
            tree: 0.0,
            tree_range: 350.0,
            tree_rush: 25.0,
            w_tree_enemy: 1.2,
            canopy_m: 4.6,
            probe_late: 0.0,
            retreat_hp: 0.0,
            resume_hp: 0.88,
            roof: 0.0,
            roof_depth: 10.0,
            roof_min_rise: 12.0,
            roof_range: 260.0,
            lat: 0.0,
            farm_seek: 0.0,
            farm_range: 150.0,
            explore: 0.0,
            farm_clear: 120.0,
            press: 0.0,
            sb: 0.25,
            strafe_min: 0.6,
            probe: 0.0,
            probe_thr: 0.35,
            probe_amt: 500.0,
            probe_win: 4.0,
            probe_share: 0.5,
            solo: 0.0,
            lay_low: 10.0,
            ring: 0.0,
            topup: 0.0,
            arm_first: 0.0,
            peace: 0.0,
            away: 0.0,
            xlow: 0.0,
            peek: 0.0,
            peek_exp: 1.6,
            peek_reach: 24.0,
            peek_nurse: 0.45,
            zroam: 0.0,
            zroam_pad: 6.0,
            arm_known: 0.0,
            scan_first: 0.0,
            heal_hide: 0.0,
            laser_lead: 0.1,
            reach_fix: 0.0,
            mortar_rank: 1.0,
            laser_rank: 3.0,
            lightning_rank: 2.0,
            laser_range: 0.0,
            tick_aim: 0.0,
            arm_safe: 0.0,
            heal_reach: 0.0,
            last_hp: 0.0,
            last_reach: 40.0,
            gun_far: 0.0,
            chase: 0.0,
            chase_frac: 0.4,
            space: 0.0,
            calm_aim: 0.0,
            grudge: 4.0,
            zap: 0.0,
            zap_dist: 30.0,
            zap_fear: 0.0,
            zap_fear_d: 70.0,
            upg: 0.0,
            drop_go: 0.0,
            drop_range: 240.0,
            scav: 0.0,
            farm_start: 0.0,
            scan_every: 0.0,
            shy: 0.0,
            shy_d: 80.0,
            outr: 0.0,
            outr_d: 95.0,
            outr_reach: 70.0,
            heal: 0.0,
        }
    }
}
impl Params {
    pub fn apply(&mut self, text: &str) {
        for line in text.lines() {
            let line = line.split('#').next().unwrap_or("").trim();
            let mut it = line.splitn(2, '=');
            let (k, v) = match (it.next(), it.next()) {
                (Some(k), Some(v)) => (k.trim(), v.trim()),
                _ => continue,
            };
            let v: f64 = match v.parse() {
                Ok(v) => v,
                Err(_) => continue,
            };
            match k {
                "ambush" => self.ambush = v,
                "farm" => self.farm = v,
                "drops" => self.drops = v,
                "w_travel" => self.w_travel = v,
                "w_expo" => self.w_expo = v,
                "w_vis" => self.w_vis = v,
                "w_near" => self.w_near = v,
                "center_avoid" => self.center_avoid = v,
                "fight_margin" => self.fight_margin = v,
                "finish_hp" => self.finish_hp = v,
                "edge_margin" => self.edge_margin = v,
                "end_radius" => self.end_radius = v,
                "spin" => self.spin = v,
                "cover_reach" => self.cover_reach = v,
                "w_detour" => self.w_detour = v,
                "lazy" => self.lazy = v,
                "relocate" => self.relocate = v,
                "sees_engage" => self.sees_engage = v,
                "pb_range" => self.pb_range = v,
                "idle" => self.idle = v,
                "nomove" => self.nomove = v,
                "pacifist" => self.pacifist = v,
                "debug" => self.debug = v,
                "kite" => self.kite = v,
                "w_far" => self.w_far = v,
                "far_d" => self.far_d = v,
                "mem" => self.mem = v,
                "flee_reach" => self.flee_reach = v,
                "snd" => self.snd = v,
                "kite_fire" => self.kite_fire = v,
                "near_d" => self.near_d = v,
                "fight_dist" => self.fight_dist = v,
                "farm_loose" => self.farm_loose = v,
                "seek_w" => self.seek_w = v,
                "seek_range" => self.seek_range = v,
                "laser_dist" => self.laser_dist = v,
                "laser_engage" => self.laser_engage = v,
                "gun_max" => self.gun_max = v,
                "scan" => self.scan = v,
                "tree" => self.tree = v,
                "tree_range" => self.tree_range = v,
                "tree_rush" => self.tree_rush = v,
                "w_tree_enemy" => self.w_tree_enemy = v,
                "canopy_m" => self.canopy_m = v,
                "probe_late" => self.probe_late = v,
                "retreat_hp" => self.retreat_hp = v,
                "resume_hp" => self.resume_hp = v,
                "roof" => self.roof = v,
                "roof_depth" => self.roof_depth = v,
                "roof_min_rise" => self.roof_min_rise = v,
                "roof_range" => self.roof_range = v,
                "lat" => self.lat = v,
                "farm_seek" => self.farm_seek = v,
                "farm_range" => self.farm_range = v,
                "explore" => self.explore = v,
                "farm_clear" => self.farm_clear = v,
                "press" => self.press = v,
                "sb" => self.sb = v,
                "strafe_min" => self.strafe_min = v,
                "probe" => self.probe = v,
                "probe_thr" => self.probe_thr = v,
                "probe_amt" => self.probe_amt = v,
                "probe_win" => self.probe_win = v,
                "probe_share" => self.probe_share = v,
                "solo" => self.solo = v,
                "lay_low" => self.lay_low = v,
                "ring" => self.ring = v,
                "topup" => self.topup = v,
                "arm_first" => self.arm_first = v,
                "peace" => self.peace = v,
                "away" => self.away = v,
                "xlow" => self.xlow = v,
                "peek" => self.peek = v,
                "peek_exp" => self.peek_exp = v,
                "peek_reach" => self.peek_reach = v,
                "peek_nurse" => self.peek_nurse = v,
                "heal" => self.heal = v,
                "zroam" => self.zroam = v,
                "zroam_pad" => self.zroam_pad = v,
                "arm_known" => self.arm_known = v,
                "scan_first" => self.scan_first = v,
                "heal_hide" => self.heal_hide = v,
                "laser_lead" => self.laser_lead = v,
                "reach_fix" => self.reach_fix = v,
                "mortar_rank" => self.mortar_rank = v,
                "laser_rank" => self.laser_rank = v,
                "lightning_rank" => self.lightning_rank = v,
                "laser_range" => self.laser_range = v,
                "tick_aim" => self.tick_aim = v,
                "arm_safe" => self.arm_safe = v,
                "heal_reach" => self.heal_reach = v,
                "last_hp" => self.last_hp = v,
                "last_reach" => self.last_reach = v,
                "outr" => self.outr = v,
                "gun_far" => self.gun_far = v,
                "shy" => self.shy = v,
                "scan_every" => self.scan_every = v,
                "farm_start" => self.farm_start = v,
                "scav" => self.scav = v,
                "drop_go" => self.drop_go = v,
                "upg" => self.upg = v,
                "calm_aim" => self.calm_aim = v,
                "space" => self.space = v,
                "chase" => self.chase = v,
                "chase_frac" => self.chase_frac = v,
                "grudge" => self.grudge = v,
                "zap" => self.zap = v,
                "zap_dist" => self.zap_dist = v,
                "zap_fear" => self.zap_fear = v,
                "zap_fear_d" => self.zap_fear_d = v,
                "drop_range" => self.drop_range = v,
                "shy_d" => self.shy_d = v,
                "outr_d" => self.outr_d = v,
                "outr_reach" => self.outr_reach = v,
                _ => {}
            }
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Action {
    pub mv: [f64; 2],
    pub boost: bool,
    pub jump: bool,
    pub aim: [f64; 2],
    pub fire: [bool; 2],
    pub reload: bool,
    pub pick: Option<usize>,
    pub take: Option<i64>,
    pub drop: Option<usize>,
    pub label: String,
}
impl Action {
    pub fn to_json(&self, tick: f64) -> String {
        let mut s = String::with_capacity(200);
        let _ = write!(s, "{{\"tick\":{}", tick as i64);
        let (mut mx, mut my) = (fin(self.mv[0]), fin(self.mv[1]));
        let l = hyp(mx, my);
        if l > 1.0 {
            mx /= l;
            my /= l;
        }
        let _ = write!(s, ",\"move\":[{:.3},{:.3}]", mx, my);
        let _ = write!(s, ",\"boost\":{},\"jump\":{}", self.boost, self.jump);
        let _ = write!(s, ",\"aim\":[{:.4},{:.4}]", fin(wrap(self.aim[0])), fin(self.aim[1]).clamp(-1.5, 1.5));
        let _ = write!(s, ",\"fire\":[{},{}]", self.fire[0], self.fire[1]);
        if self.reload {
            s.push_str(",\"reload\":true");
        }
        if let Some(p) = self.pick {
            let _ = write!(s, ",\"pick\":{}", p);
        }
        if let Some(t) = self.take {
            let _ = write!(s, ",\"take\":{}", t);
        }
        if let Some(d) = self.drop {
            let _ = write!(s, ",\"drop\":{}", d);
        }
        if !self.label.is_empty() {
            let clean: String = self.label.chars().filter(|c| c.is_ascii() && *c != '"' && *c != '\\' && !c.is_ascii_control()).take(70).collect();
            let _ = write!(s, ",\"debug\":{{\"label\":\"{}\"}}", clean);
        }
        s.push('}');
        s
    }
}

#[derive(Clone, Debug, Default)]
pub struct Wpn {
    pub present: bool,
    pub kind: usize,
    pub cooldown: f64,
    pub ammo: f64,
    pub reload: f64,
    pub charge: f64,
    pub damage: f64,
    pub cd: f64,
    pub speed: f64,
    pub range: f64,
    pub mag: f64,
    pub charge_t: f64,
    pub min_charge: f64,
    pub hold_s: f64,
    pub cone_deg: f64,
    pub energy: f64,
    pub gravity: f64,
    pub splash_radius: f64,
}

#[derive(Clone, Debug, Default)]
pub struct Me {
    pub id: i64,
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub aim: [f64; 2],
    pub hp: f64,
    pub max_hp: f64,
    pub energy: f64,
    pub max_energy: f64,
    pub grounded: bool,
    pub cubes: f64,
    pub in_zone: bool,
    pub w: [Wpn; 2],
    pub radius: f64,
    pub height: f64,
    pub eye_h: f64,
    pub muzzle_h: f64,
    pub walk_speed: f64,
    pub boost_speed: f64,
    pub turn_rate: f64,
    pub pitch_rate: f64,
    pub min_pitch: f64,
    pub max_pitch: f64,
}
impl Me {
    fn slot_of(&self, kind: usize) -> Option<usize> {
        (0..2).find(|&i| self.w[i].present && self.w[i].kind == kind)
    }
    fn size(&self) -> f64 {
        self.radius / 2.5
    }
}

#[derive(Clone, Debug)]
pub struct Track {
    pub id: i64,
    pub npc: bool,
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub aim: [f64; 2],
    pub hp: f64,
    pub max_hp: f64,
    pub cubes: f64,
    pub has: [bool; 4],
    pub charging: bool,
    pub seen_t: f64,
    pub first_t: f64,
    pub hit_us_t: f64,
    pub hp_drop_t: f64,
    pub aim_us: u32,
}
impl Track {
    fn size(&self) -> f64 {
        if self.npc {
            1.0
        } else {
            1.0 + 0.05 * self.cubes
        }
    }
    fn center_h(&self) -> f64 {
        if self.npc {
            1.75
        } else {
            4.5 * self.size()
        }
    }
    fn radius(&self) -> f64 {
        if self.npc {
            1.8
        } else {
            2.5 * self.size()
        }
    }
    fn eye(&self) -> [f64; 3] {
        [self.pos[0], self.pos[1], self.pos[2] + 8.0 * self.size()]
    }
    fn center(&self) -> [f64; 3] {
        [self.pos[0], self.pos[1], self.pos[2] + self.center_h()]
    }
}

#[derive(Clone, Debug)]
pub struct Item {
    pub id: i64,
    pub pos: [f64; 3],
    pub kind: usize,
    pub hp: f64,
    pub seen_t: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Zone {
    pub c: [f64; 2],
    pub r: f64,
    pub nc: [f64; 2],
    pub nr: f64,
    pub t0: f64,
    pub t1: f64,
    pub dps: f64,
}
impl Zone {
    fn read(z: &J) -> Zone {
        Zone {
            c: z.get("center").v2(),
            r: z.get("radius").fo(1000.0),
            nc: z.get("next_center").v2(),
            nr: z.get("next_radius").fo(0.0),
            t0: z.get("shrink_start").f(),
            t1: z.get("shrink_end").f(),
            dps: z.get("dps").f(),
        }
    }
    /// circle at future time tau, given the obs was taken at `now`
    fn at(&self, now: f64, tau: f64) -> ([f64; 2], f64) {
        let start = now.max(self.t0);
        if tau <= start || self.t1 <= start {
            if tau >= self.t1 && self.t1 <= start {
                return (self.nc, self.nr);
            }
            return (self.c, self.r);
        }
        let f = ((tau - start) / (self.t1 - start)).clamp(0.0, 1.0);
        ([self.c[0] + (self.nc[0] - self.c[0]) * f, self.c[1] + (self.nc[1] - self.c[1]) * f], self.r + (self.nr - self.r) * f)
    }
    /// seconds from now until p is outside the zone (0 if already outside, big if safe this phase)
    fn time_left(&self, now: f64, p: &[f64]) -> f64 {
        if d2(p, &self.c) > self.r {
            return 0.0;
        }
        if d2(p, &self.nc) <= self.nr {
            return 1e6;
        }
        let start = now.max(self.t0);
        let n = 24;
        for k in 1..=n {
            let tau = start + (self.t1 - start) * k as f64 / n as f64;
            let (c, r) = self.at(now, tau);
            if d2(p, &c) > r {
                return (tau - now).max(0.0);
            }
        }
        (self.t1 - now).max(0.0)
    }
}

struct Threat {
    eye: [f64; 3],
    pos: [f64; 2],
    w: f64,
    range: f64,
    age: f64,
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Mode {
    Hide,
    Flee,
    Fight,
    ZoneRun,
    End,
    Seek,
    Tree,
    Roof,
}

/// A spot inside a tree crown: where, and what share of surrounding ground can still see a mech there.
#[derive(Clone, Copy, Debug)]
pub struct HideSpot {
    pub x: f64,
    pub y: f64,
    pub vis: f32,
}

pub struct Brain {
    pub prm: Params,
    pub map: Arc<Map>,
    pub nav: Nav,
    expo: Vec<f32>,
    ew: usize,
    /// extra path cost per nav cell: open ground costs more than ground next to cover
    path_cost: Vec<f32>,
    goal_hidden: bool,
    goal_cost: f64,
    /// walking distance from the centre of the next circle to every cell (inf = not reached)
    center_field: Vec<f32>,
    field_key: [f64; 3],
    rng: Rng,
    slot: i64,
    // config
    fov: f64,
    vis_range: f64,
    near_range: f64,
    loot_radius: f64,
    cube_radius: f64,
    stat_cap: f64,
    ground_accel: f64,
    air_accel: f64,
    charge_move: f64,
    npc_aggro: f64,
    upgrades_pref: Vec<&'static str>,
    // state
    tracks: Vec<Track>,
    pickups: Vec<Item>,
    crates: Vec<Item>,
    cubes: Vec<Item>,
    last_aim: [f64; 2],
    last_mv: [f64; 2],
    last_dmg_t: f64,
    last_dmg_bearing: f64,
    last_dmg_by: i64,
    dmg_recent: f64,
    sound_bearing: f64,
    sound_t: f64,
    path: Vec<(f64, f64)>,
    path_i: usize,
    goal: Option<u32>,
    plan_t: f64,
    mode: Mode,
    hist: Vec<(f64, [f64; 2], bool)>,
    unstick_until: f64,
    unstick_dir: [f64; 2],
    stuck_count: u32,
    stuck_t: f64,
    strafe_dir: f64,
    strafe_vec: [f64; 2],
    strafe_until: f64,
    strafe_speed: f64,
    strafe_boost: bool,
    flee_planned: bool,
    spin_yaw: f64,
    laser_hold: bool,
    target_id: i64,
    held_cover_t: f64,
    take_fail_t: f64,
    take_fail_id: i64,
    fly_goal: Option<(f64, f64)>,
    zone_plen: f64,
    zone_plen_t: f64,
    zone_moving: bool,
    npc_dmg_t: f64,
    jink_dir: f64,
    jink_until: f64,
    /// recent sounds that mean a mech: (time, bearing, assumed distance, where we stood)
    snds: Vec<(f64, f64, f64, [f64; 2])>,
    seek_id: i64,
    scan_until: f64,
    scan_t: f64,
    pub hides: Vec<HideSpot>,
    hide_i: Option<usize>,
    hide_t: f64,
    pub force_idle: bool,
    retreating: bool,
    roof_i: Option<usize>,
    roof_t: f64,
    roof_bad: Vec<usize>,
    explore_goal: Option<(f64, f64)>,
    explore_t: f64,
    /// things we gave up walking to: (id, until)
    seek_ban: Vec<(i64, f64)>,
    seek_best_d: f64,
    seek_prog_t: f64,
    st_dmg: [f64; 4],
    /// of the first 300 damage: from mechs we had not seen, from beyond 90 m, from the rear half
    st_unseen: f64,
    st_far: f64,
    /// all damage taken from mechs and dealt to mechs so far
    st_taken_all: f64,
    st_dealt_all: f64,
    st_attackers: Vec<i64>,
    /// the same kind of bookkeeping, but only from the moment `probe_thr` opponents are dead
    tail_att: Vec<i64>,
    /// last time each mech hurt us (id, t)
    tail_hits: Vec<(i64, f64)>,
    tail_dmg: [f64; 4],
    tail_near: f64,
    tail_total: f64,
    st_gun_hits: u32,
    st_gun_shots: u32,
    st_prev_ammo: f64,
    st_had_tgt: bool,
    st_done: bool,
    lay_low_until: f64,
    peek: Option<Peek>,
    peek_charge: bool,
    healing: bool,
    heal_spot: Option<(f64, f64, f64)>,
    /// where mechs died lately: (x, y, time, how sure: 1 = we were watching it, 0 = by ear)
    death_sites: Vec<(f64, f64, f64, u8)>,
    /// supply drops: (id, when we first saw it on the ground)
    drop_seen: Vec<(i64, f64)>,
    pub last_decide_us: u128,
}

/// A cover spot `h` the target cannot see, the firing spot `p` next to it, and where we are in the cycle
/// (0 = getting out of sight, 1 = out of sight and readying, 2 = stepped out and shooting).
#[derive(Clone, Copy, Debug)]
struct Peek {
    h: (f64, f64),
    p: (f64, f64),
    tgt: i64,
    pick_t: f64,
    phase: u8,
    phase_t: f64,
    hp0: f64,
}

impl Brain {
    pub fn new(hello: &J, map: Arc<Map>, prm: Params) -> Brain {
        let cfg = hello.get("config");
        let nav = Nav::new(&map);
        // Exposure: mean free sight line, on the 4 m lattice. 0 = boxed in, 1 = open field.
        let ew = (nav.w + 1) / 2;
        let eh = (nav.h + 1) / 2;
        let mut expo = vec![1f32; ew * eh];
        const ND: usize = 16;
        let dirs: Vec<f64> = (0..ND).map(|k| k as f64 * 2.0 * PI / ND as f64).collect();
        for j in 0..eh {
            for i in 0..ew {
                let c = (2 * j).min(nav.h - 1) * nav.w + (2 * i).min(nav.w - 1);
                if nav.clear[c] < 3.0 {
                    continue;
                }
                let (x, y) = nav.pos_of(c as u32);
                let z = nav.wz[c] + 7.0;
                let mut sum = 0.0;
                for &a in &dirs {
                    sum += map.ray_len(x, y, z, a, 180.0, 3.0);
                }
                expo[j * ew + i] = (sum / (ND as f64 * 180.0)) as f32;
            }
        }
        let seed = hello.get("seed").fo(12345.0) as u64;
        let mut path_cost = vec![0f32; nav.w * nav.h];
        for j in 0..nav.h {
            for i in 0..nav.w {
                let e = expo[(j / 2).min(eh - 1) * ew + (i / 2).min(ew - 1)];
                path_cost[j * nav.w + i] = 0.6 * e * e;
            }
        }
        let hides = if prm.tree > 0.5 { find_hides(&map, &nav, prm.canopy_m) } else { Vec::new() };
        Brain {
            prm,
            map,
            nav,
            expo,
            ew,
            path_cost,
            goal_hidden: false,
            goal_cost: 0.0,
            center_field: Vec::new(),
            field_key: [f64::NAN; 3],
            rng: Rng(seed.wrapping_mul(0x9E3779B97F4A7C15) | 1),
            slot: hello.get("slot").fo(-1.0) as i64,
            fov: cfg.get("vision").get("fov_deg").fo(120.0).to_radians(),
            vis_range: cfg.get("vision").get("range").fo(200.0),
            near_range: cfg.get("vision").get("near_range").fo(45.0),
            loot_radius: cfg.get("loot").get("pickup_radius").fo(4.0),
            cube_radius: cfg.get("cubes").get("pickup_radius").fo(3.5),
            stat_cap: cfg.get("cubes").get("stat_cap").fo(10.0),
            ground_accel: cfg.get("mech").get("ground_accel").fo(58.5),
            air_accel: cfg.get("mech").get("air_accel").fo(12.0),
            charge_move: cfg.get("mech").get("charge_move_mult").fo(0.5),
            npc_aggro: cfg.get("npc").get("aggro_range").fo(45.0),
            upgrades_pref: vec![
                "plating", "laser_damage", "laser_charge", "armor", "nanites", "gun_damage", "gun_rate", "servos", "lightning_efficiency",
                "lightning_chain", "lightning_range", "gyros", "sensors", "reactor", "capacitor", "gun_mag", "jets", "gun_velocity", "laser_pierce",
                "mortar_reload", "mortar_splash", "mortar_velocity",
            ],
            tracks: Vec::new(),
            pickups: Vec::new(),
            crates: Vec::new(),
            cubes: Vec::new(),
            last_aim: [0.0, 0.0],
            last_mv: [0.0, 0.0],
            last_dmg_t: -100.0,
            last_dmg_bearing: 0.0,
            last_dmg_by: -1,
            dmg_recent: 0.0,
            sound_bearing: 0.0,
            sound_t: -100.0,
            path: Vec::new(),
            path_i: 0,
            goal: None,
            plan_t: -100.0,
            mode: Mode::Hide,
            hist: Vec::new(),
            unstick_until: -1.0,
            unstick_dir: [0.0, 0.0],
            stuck_count: 0,
            stuck_t: -100.0,
            strafe_dir: 1.0,
            strafe_vec: [0.0, 0.0],
            strafe_until: -1.0,
            strafe_speed: 1.0,
            strafe_boost: false,
            flee_planned: false,
            spin_yaw: 0.0,
            laser_hold: false,
            target_id: -1,
            held_cover_t: -100.0,
            take_fail_t: -100.0,
            take_fail_id: -1,
            fly_goal: None,
            zone_plen: 0.0,
            zone_plen_t: -100.0,
            zone_moving: false,
            npc_dmg_t: -100.0,
            jink_dir: 1.0,
            jink_until: -1.0,
            snds: Vec::new(),
            seek_id: -1,
            scan_until: -1.0,
            scan_t: -100.0,
            hides,
            hide_i: None,
            hide_t: -100.0,
            force_idle: false,
            retreating: false,
            roof_i: None,
            roof_t: -100.0,
            roof_bad: Vec::new(),
            explore_goal: None,
            explore_t: -100.0,
            seek_ban: Vec::new(),
            seek_best_d: 1e9,
            seek_prog_t: 0.0,
            st_dmg: [0.0; 4],
            st_unseen: 0.0,
            st_far: 0.0,
            st_taken_all: 0.0,
            st_dealt_all: 0.0,
            st_attackers: Vec::new(),
            tail_att: Vec::new(),
            tail_hits: Vec::new(),
            tail_dmg: [0.0; 4],
            tail_near: 0.0,
            tail_total: 0.0,
            st_gun_hits: 0,
            st_gun_shots: 0,
            st_prev_ammo: -1.0,
            st_had_tgt: false,
            st_done: false,
            lay_low_until: -1.0,
            peek: None,
            peek_charge: false,
            healing: false,
            heal_spot: None,
            death_sites: Vec::new(),
            drop_seen: Vec::new(),
            last_decide_us: 0,
        }
    }

    fn expo_at(&self, c: u32) -> f64 {
        let i = (c as usize % self.nav.w) / 2;
        let j = (c as usize / self.nav.w) / 2;
        self.expo.get(j * self.ew + i).copied().unwrap_or(1.0) as f64
    }

    fn read_self(&self, o: &J) -> Me {
        let s = o.get("self");
        let st = s.get("stats");
        let mut me = Me {
            id: s.get("id").fo(self.slot as f64) as i64,
            pos: s.get("pos").v3(),
            vel: s.get("vel").v3(),
            aim: s.get("aim").v2(),
            hp: s.get("hp").f(),
            max_hp: s.get("max_hp").fo(1400.0),
            energy: s.get("energy").f(),
            max_energy: s.get("max_energy").fo(100.0),
            grounded: s.get("grounded").b(),
            cubes: s.get("cubes").f(),
            in_zone: s.get("in_zone").b(),
            w: [Wpn::default(), Wpn::default()],
            radius: st.get("radius").fo(2.5),
            height: st.get("height").fo(9.0),
            eye_h: st.get("eye_height").fo(8.0),
            muzzle_h: st.get("muzzle_height").fo(7.0),
            walk_speed: st.get("walk_speed").fo(11.7),
            boost_speed: st.get("boost_speed").fo(19.5),
            turn_rate: st.get("turn_rate").fo(4.0),
            pitch_rate: st.get("pitch_rate").fo(3.0),
            min_pitch: st.get("min_pitch").fo(-0.7),
            max_pitch: st.get("max_pitch").fo(1.3),
        };
        for i in 0..2 {
            let w = s.get("weapons").at(i);
            if w.is_null() {
                continue;
            }
            let ws = w.get("stats");
            me.w[i] = Wpn {
                present: true,
                kind: wkind(w.get("kind").s()),
                cooldown: w.get("cooldown").f(),
                ammo: w.get("ammo").fo(1.0),
                reload: w.get("reload").f(),
                charge: w.get("charge").f(),
                damage: ws.get("damage").f(),
                cd: ws.get("cooldown").fo(0.25),
                speed: ws.get("speed").fo(252.0),
                range: ws.get("range").fo(230.0),
                mag: ws.get("mag").fo(12.0),
                charge_t: ws.get("charge").fo(0.75),
                min_charge: ws.get("min_charge").fo(0.15),
                hold_s: ws.get("hold_s").fo(2.0),
                cone_deg: ws.get("cone_deg").fo(30.0),
                energy: ws.get("energy").fo(3.0),
                gravity: ws.get("gravity").fo(22.0),
                splash_radius: ws.get("splash_radius").fo(7.0),
            };
            if me.w[i].kind == MORTAR && ws.get("range").is_null() {
                me.w[i].range = me.w[i].speed * me.w[i].speed / me.w[i].gravity.max(1.0) * 0.95;
            }
        }
        me
    }

    fn update_tracks(&mut self, o: &J, t: f64) {
        for m in o.get("mechs").arr() {
            let id = m.get("id").fo(-1.0) as i64;
            if id < 0 {
                continue;
            }
            let npc = m.get("kind").s() == "npc";
            let mut has = [false; 4];
            for w in m.get("weapons").arr() {
                has[wkind(w.s())] = true;
            }
            if npc {
                has = [false; 4];
            }
            let hp = m.get("hp").fo(if npc { 350.0 } else { 1400.0 });
            let idx = match self.tracks.iter().position(|e| e.id == id) {
                Some(i) => i,
                None => {
                    self.tracks.push(Track {
                        id,
                        npc,
                        pos: [0.0; 3],
                        vel: [0.0; 3],
                        aim: [0.0; 2],
                        hp,
                        max_hp: hp,
                        cubes: 0.0,
                        has,
                        charging: false,
                        seen_t: t,
                        first_t: t,
                        hit_us_t: -100.0,
                        hp_drop_t: -100.0,
                        aim_us: 0,
                    });
                    self.tracks.len() - 1
                }
            };
            let e = &mut self.tracks[idx];
            if hp < e.hp - 0.5 {
                e.hp_drop_t = t;
            }
            e.pos = m.get("pos").v3();
            e.vel = m.get("vel").v3();
            e.aim = m.get("aim").v2();
            e.hp = hp;
            e.max_hp = m.get("max_hp").fo(e.max_hp.max(hp));
            e.cubes = m.get("cubes").f();
            e.has = has;
            e.charging = m.get("charging").b();
            e.seen_t = t;
        }
        let mem = self.prm.mem.max(4.0);
        self.tracks.retain(|e| t - e.seen_t <= if e.npc { 20.0 } else { mem });
    }

    fn update_items(&mut self, o: &J, me: &Me, t: f64) {
        fn merge(list: &mut Vec<Item>, src: &[J], t: f64, with_kind: bool) {
            for p in src {
                let id = p.get("id").fo(-1.0) as i64;
                let it = Item { id, pos: p.get("pos").v3(), kind: if with_kind { wkind(p.get("kind").s()) } else { 0 }, hp: p.get("hp").fo(300.0), seen_t: t };
                match list.iter_mut().find(|q| q.id == id) {
                    Some(q) => *q = it,
                    None => list.push(it),
                }
            }
        }
        merge(&mut self.pickups, o.get("pickups").arr(), t, true);
        merge(&mut self.crates, o.get("crates").arr(), t, false);
        merge(&mut self.cubes, o.get("cubes").arr(), t, false);
        // Forget things we should be seeing but are not (taken or destroyed), and stale entries.
        let eye = [me.pos[0], me.pos[1], me.pos[2] + me.eye_h];
        let near = self.near_range - 3.0;
        let map = &self.map;
        let gone = |it: &Item, ttl: f64| -> bool {
            if it.seen_t >= t {
                return false;
            }
            if t - it.seen_t > ttl {
                return true;
            }
            let d = d2(&it.pos, &me.pos);
            d < near && t - it.seen_t > 0.35 && map.los(eye, [it.pos[0], it.pos[1], it.pos[2] + 1.0])
        };
        self.pickups.retain(|it| !gone(it, 90.0));
        self.crates.retain(|it| !gone(it, 240.0));
        self.cubes.retain(|it| !gone(it, 60.0));
    }

    fn read_events(&mut self, o: &J, me: &Me, t: f64) {
        self.dmg_recent *= 0.9;
        for e in o.get("events").arr() {
            match e.get("type").s() {
                "hit" => {
                    if e.get("target").f() < 100.0 {
                        self.st_dealt_all += e.get("amount").f();
                        if e.get("weapon").s() == "gun" {
                            self.st_gun_hits += 1;
                        }
                    }
                }
                "damaged" => {
                    let by = e.get("by");
                    if by.is_null() {
                        continue; // zone
                    }
                    let id = by.f() as i64;
                    let amt = e.get("amount").f();
                    if id >= 100 {
                        // a drone: remember it, but it is not worth running from
                        self.npc_dmg_t = t;
                        if let Some(tr) = self.tracks.iter_mut().find(|q| q.id == id) {
                            tr.hit_us_t = t;
                        }
                        continue;
                    }
                    self.dmg_recent += amt;
                    self.last_dmg_t = t;
                    self.last_dmg_by = id;
                    self.st_taken_all += amt;
                    if self.prm.probe >= 11.5 && 16.0 - o.get("alive").fo(16.0) >= self.prm.probe_thr - 0.5 {
                        if !self.tail_att.contains(&id) {
                            self.tail_att.push(id);
                        }
                        match self.tail_hits.iter_mut().find(|h| h.0 == id) {
                            Some(h) => h.1 = t,
                            None => self.tail_hits.push((id, t)),
                        }
                        self.tail_dmg[wkind(e.get("weapon").s())] += amt;
                        self.tail_total += amt;
                        if let Some(tr) = self.tracks.iter().find(|q| q.id == id && t - q.seen_t < 3.0) {
                            if d2(&tr.pos, &me.pos) < 50.0 {
                                self.tail_near += amt;
                            }
                        }
                    }
                    if self.st_dmg.iter().sum::<f64>() < 300.0 {
                        self.st_dmg[wkind(e.get("weapon").s())] += amt;
                        if !self.st_attackers.contains(&id) {
                            self.st_attackers.push(id);
                        }
                        match self.tracks.iter().find(|q| q.id == id && t - q.seen_t < 3.0) {
                            Some(tr) => {
                                if d2(&tr.pos, &me.pos) > 90.0 {
                                    self.st_far += amt;
                                }
                            }
                            None => self.st_unseen += amt,
                        }
                    }
                    if let Some(b) = e.get("bearing").num() {
                        self.last_dmg_bearing = b;
                    }
                    if let Some(tr) = self.tracks.iter_mut().find(|q| q.id == id) {
                        tr.hit_us_t = t;
                    } else if id < 100 {
                        // unseen attacker: remember a phantom along the bearing
                        let b = self.last_dmg_bearing;
                        let d = 110.0;
                        let px = me.pos[0] + b.cos() * d;
                        let py = me.pos[1] + b.sin() * d;
                        let pz = self.map.walk_z(px, py);
                        let mut has = [false; 4];
                        has[wkind(e.get("weapon").s())] = true;
                        has[GUN] = true;
                        self.tracks.push(Track {
                            id,
                            npc: false,
                            pos: [px, py, pz],
                            vel: [0.0; 3],
                            aim: [wrap(b + PI), 0.0],
                            hp: 1400.0,
                            max_hp: 1400.0,
                            cubes: 0.0,
                            has,
                            charging: false,
                            seen_t: t - 2.0,
                            first_t: t,
                            hit_us_t: t,
                            hp_drop_t: -100.0,
                            aim_us: 0,
                        });
                    }
                }
                "take_failed" => {
                    self.take_fail_t = t;
                    self.take_fail_id = e.get("id").fo(-1.0) as i64;
                    let id = self.take_fail_id;
                    if e.get("reason").s() == "gone" {
                        self.pickups.retain(|p| p.id != id);
                    }
                }
                "took" => {
                    let id = e.get("id").fo(-1.0) as i64;
                    self.pickups.retain(|p| p.id != id);
                }
                _ => {}
            }
        }
        for s in o.get("sounds").arr() {
            let k = s.get("kind").s();
            let near = s.get("dist").s() == "near";
            if k == "death" && self.prm.scav > 0.5 && s.get("dist").s() != "far" {
                let b = s.get("bearing").f();
                let (lo, hi) = if near { (0.0, 75.0) } else { (45.0, 185.0) };
                // a mech we were tracking in that direction is the likeliest one
                let mut best: Option<(f64, [f64; 3])> = None;
                for e in &self.tracks {
                    if e.npc || t - e.seen_t > 6.0 {
                        continue;
                    }
                    let d = d2(&e.pos, &me.pos);
                    let off = wrap((e.pos[1] - me.pos[1]).atan2(e.pos[0] - me.pos[0]) - b).abs();
                    if d >= lo && d <= hi && off < 0.3 && best.map_or(true, |x| off < x.0) {
                        best = Some((off, e.pos));
                    }
                }
                let site = match best {
                    Some((_, p)) => (p[0], p[1], t, 1u8),
                    None => {
                        let r = if near { 35.0 } else { 105.0 };
                        (me.pos[0] + b.cos() * r, me.pos[1] + b.sin() * r, t, 0u8)
                    }
                };
                self.death_sites.push(site);
                if self.death_sites.len() > 8 {
                    self.death_sites.remove(0);
                }
            }
            if near && matches!(k, "step" | "boost" | "jet" | "reload" | "laser_charge" | "gun" | "lightning" | "laser" | "mortar") {
                self.sound_bearing = s.get("bearing").f();
                self.sound_t = t;
            }
            if self.prm.snd > 0.5 {
                let dist = s.get("dist").s();
                let moving = matches!(k, "step" | "boost" | "jet" | "reload" | "laser_charge");
                let firing = matches!(k, "gun" | "lightning" | "laser" | "mortar");
                if (moving || firing) && dist != "far" {
                    let r = if dist == "near" { 35.0 } else { 105.0 };
                    self.snds.push((t, s.get("bearing").f(), r, [me.pos[0], me.pos[1]]));
                }
            }
        }
        self.snds.retain(|x| t - x.0 <= 6.0);
        if self.snds.len() > 24 {
            let cut = self.snds.len() - 24;
            self.snds.drain(0..cut);
        }
    }

    // ------------------------------------------------------------------ aiming helpers

    fn lead(&self, me: &Me, origin_h: f64, tp: [f64; 3], tv: [f64; 3], speed: f64) -> ([f64; 2], f64, [f64; 3]) {
        let lat = self.prm.lat;
        self.lead_at(me, origin_h, tp, tv, speed, lat, [me.vel[0] * lat, me.vel[1] * lat])
    }

    /// Where we will have moved to `ticks` ticks from now, walking towards the commanded velocity.
    fn own_shift(&self, me: &Me, cmd: [f64; 2], ticks: usize) -> [f64; 2] {
        let dt = 0.05;
        let acc = if me.grounded { self.ground_accel } else { self.air_accel };
        let (mut vx, mut vy) = (me.vel[0], me.vel[1]);
        let (mut x, mut y) = (0.0, 0.0);
        for _ in 0..ticks {
            let (dx, dy) = (cmd[0] - vx, cmd[1] - vy);
            let d = hyp(dx, dy);
            let step = (acc * dt).min(d);
            if d > 1e-9 {
                vx += dx / d * step;
                vy += dy / d * step;
            }
            x += vx * dt;
            y += vy * dt;
        }
        [x, y]
    }

    fn lead_at(&self, me: &Me, origin_h: f64, tp: [f64; 3], tv: [f64; 3], speed: f64, lat: f64, shift: [f64; 2]) -> ([f64; 2], f64, [f64; 3]) {
        // the shot leaves `lat` seconds from now: from where we will be, at where the target will be
        let o = [me.pos[0] + shift[0], me.pos[1] + shift[1], me.pos[2] + origin_h];
        let tp = if speed > 1.0 && lat > 0.0 { [tp[0] + tv[0] * lat, tp[1] + tv[1] * lat, tp[2]] } else { tp };
        let mut p = tp;
        let mut tt = 0.0;
        if speed > 1.0 {
            for _ in 0..3 {
                let d = ((p[0] - o[0]).powi(2) + (p[1] - o[1]).powi(2) + (p[2] - o[2]).powi(2)).sqrt();
                tt = d / speed;
                let vz = if lat > 0.0 { (tv[2] * tt).clamp(-3.0, 3.0) } else { 0.0 };
                p = [tp[0] + tv[0] * tt, tp[1] + tv[1] * tt, tp[2] + vz];
            }
        }
        let _ = tt;
        let dx = p[0] - o[0];
        let dy = p[1] - o[1];
        let dz = p[2] - o[2];
        let dh = hyp(dx, dy);
        ([dy.atan2(dx), dz.atan2(dh.max(0.01))], hyp(dh, dz), p)
    }

    fn aim_err(&self, me: &Me, want: [f64; 2]) -> f64 {
        let dy = wrap(want[0] - me.aim[0]) * me.aim[1].cos();
        let dp = want[1] - me.aim[1];
        hyp(dy, dp)
    }

    // ------------------------------------------------------------------ planner

    fn build_threats(&self, me: &Me, t: f64, fight_id: i64) -> Vec<Threat> {
        let mut out: Vec<(f64, Threat)> = Vec::new();
        for e in &self.tracks {
            if e.npc {
                continue;
            }
            let age = t - e.seen_t;
            let fade = (1.0 - age / self.prm.mem.max(4.0)).clamp(0.0, 1.0);
            let hostile = t - e.hit_us_t < 4.0 || e.aim_us >= 3;
            let kite = self.prm.kite > 0.5;
            let mut w = if hostile { 1.0 } else if kite { 0.8 } else { 0.5 } * fade;
            if e.id == fight_id {
                w *= 0.15;
            }
            if w < 0.03 {
                continue;
            }
            let range = if e.has[LASER] { 290.0 } else { 240.0 };
            // where it probably is by now
            let lead = if kite { age.min(1.5) } else { 0.0 };
            let pos = [e.pos[0] + e.vel[0] * lead, e.pos[1] + e.vel[1] * lead];
            let eye = [pos[0], pos[1], e.eye()[2]];
            let d = d2(&pos, &me.pos);
            out.push((d - w * 100.0, Threat { eye, pos, w, range, age }));
        }
        if self.prm.snd > 0.5 {
            for &(st, b, r, from) in &self.snds {
                let age = t - st;
                let pos = [from[0] + b.cos() * r, from[1] + b.sin() * r];
                // skip the ones a tracked mech already explains
                if out.iter().any(|o| d2(&o.1.pos, &pos) < 45.0) {
                    continue;
                }
                let w = 0.35 * (1.0 - age / 6.0).clamp(0.0, 1.0);
                if w < 0.03 {
                    continue;
                }
                let z = self.map.walk_z(pos[0], pos[1]) as f64 + 8.0;
                let d = d2(&pos, &me.pos);
                out.push((d - w * 100.0, Threat { eye: [pos[0], pos[1], z], pos, w, range: 200.0, age: age + 2.0 }));
            }
        }
        out.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        out.truncate(if self.prm.kite > 0.5 { 6 } else { 5 });
        out.into_iter().map(|x| x.1).collect()
    }

    /// Cost of standing at nav cell c (lower is better), not counting travel.
    fn cell_cost(&self, c: u32, me: &Me, zone: &Zone, threats: &[Threat], vis_mult: f64, safe_r: f64, t: f64) -> f64 {
        let (x, y) = self.nav.pos_of(c);
        let z = self.nav.wz[c as usize] as f64;
        let mut s = self.prm.w_expo * self.expo_at(c);
        let dc = d2(&[x, y], &zone.nc);
        s += 4.0 * (dc - safe_r).max(0.0);
        // Early on the middle of the next circle is where everyone is headed.
        if zone.nr > 100.0 {
            let inner = 0.55 * zone.nr;
            if dc < inner {
                s += self.prm.center_avoid * (inner - dc) / inner;
            }
        }
        // long way round to the middle = a trap when the circle moves
        if let Some(&f) = self.center_field.get(c as usize) {
            let detour = if f.is_finite() { (f as f64 - dc).max(0.0) } else { 400.0 };
            s += self.prm.w_detour * detour.min(400.0);
        }
        let top = [x, y, z + me.height - 0.3];
        for th in threats {
            let d = d2(&[x, y], &th.pos);
            if d < 60.0 {
                s += th.w * (60.0 - d) * self.prm.w_near;
            }
            if self.prm.w_far > 0.0 {
                let de = (d - 9.0 * th.age).max(0.0);
                if de < self.prm.far_d {
                    s += self.prm.w_far * th.w * (1.0 - de / self.prm.far_d);
                }
            }
            if d < th.range {
                if self.map.los_step(th.eye, top, 1.5) {
                    let fall = if d < 130.0 { 1.0 } else if d < 210.0 { 1.0 - 0.6 * (d - 130.0) / 80.0 } else { 0.25 };
                    s += self.prm.w_vis * vis_mult * th.w * fall;
                }
            }
        }
        for e in &self.tracks {
            if e.npc && t - e.seen_t < 15.0 {
                let d = d2(&[x, y], &e.pos);
                if d < self.npc_aggro + 8.0 {
                    s += 25.0;
                }
            }
        }
        s
    }

    fn set_path_cells(&mut self, cells: &[u32], me: &Me) {
        self.path.clear();
        for &c in cells {
            self.path.push(self.nav.pos_of(c));
        }
        self.path_i = 0;
        // skip the start cell if we are already past it
        if self.path.len() >= 2 {
            let a = self.path[0];
            let b = self.path[1];
            let da = hyp(a.0 - me.pos[0], a.1 - me.pos[1]);
            let ab = hyp(b.0 - a.0, b.1 - a.1);
            let db = hyp(b.0 - me.pos[0], b.1 - me.pos[1]);
            if da < 2.5 || db < ab {
                self.path_i = 1;
            }
        }
    }

    /// Walking distances from the next centre. Hiding spots that are close as the crow flies but
    /// a long walk from the middle are traps when the circle moves.
    fn update_center_field(&mut self, zone: &Zone) -> bool {
        let key = [zone.nc[0], zone.nc[1], zone.nr];
        if key == self.field_key {
            return false;
        }
        self.field_key = key;
        let n = self.nav.w * self.nav.h;
        if self.center_field.len() != n {
            self.center_field = vec![f32::INFINITY; n];
        } else {
            self.center_field.iter_mut().for_each(|x| *x = f32::INFINITY);
        }
        let reach = (zone.nr * 2.2 + 120.0).min(900.0) as f32;
        let c0 = self.nav.cell_of(zone.nc[0], zone.nc[1]);
        if let Some(c) = self.nav.snap(c0, 3.4, 70) {
            let p = self.nav.pos_of(c);
            if self.nav.flood(p, 3.4, reach, None).is_some() {
                for &(cell, cost) in &self.nav.reached {
                    self.center_field[cell as usize] = cost;
                }
            }
        }
        true
    }

    fn plan(&mut self, me: &Me, zone: &Zone, t: f64, vis_mult: f64, fight_id: i64, loot_ok: bool) {
        if self.update_center_field(zone) {
            // that flood was this tick's heavy lifting; plan on the next one
            self.plan_t = t - 1.0;
            return;
        }
        let need = (me.radius + 0.9) as f32;
        let threats = self.build_threats(me, t, fight_id);
        let margin = (self.prm.edge_margin * zone.nr).clamp(3.0, 15.0);
        let safe_r = (zone.nr - margin).max(0.0);
        let max_cost = if vis_mult > 1.5 { (self.prm.flee_reach as f32).clamp(60.0, 320.0) } else { (self.prm.relocate as f32).clamp(20.0, 170.0) };
        let p = (me.pos[0], me.pos[1]);
        let pc = std::mem::take(&mut self.path_cost);
        let ok = self.nav.flood(p, need, max_cost, Some(&pc)).is_some();
        self.path_cost = pc;
        if !ok {
            self.goal_hidden = false;
            return;
        }
        let me_out = (d2(&me.pos, &zone.nc) - safe_r).max(0.0);
        let w = self.nav.w;
        let reached = std::mem::take(&mut self.nav.reached);
        let mut best: Option<(f64, u32)> = None;
        // loot candidates
        let mut loot: Vec<(u32, f64)> = Vec::new();
        if loot_ok {
            for it in &self.pickups {
                let want = self.want_pickup(me, it.kind);
                if want <= 0.0 || (it.id == self.take_fail_id && t - self.take_fail_t < 6.0) {
                    continue;
                }
                if (it.pos[2] - self.map.walk_z(it.pos[0], it.pos[1])).abs() > 3.0 {
                    continue;
                }
                loot.push((self.nav.cell_of(it.pos[0], it.pos[1]), 45.0 + 25.0 * want));
            }
            if me.cubes < self.stat_cap {
                for it in &self.cubes {
                    if (it.pos[2] - self.map.walk_z(it.pos[0], it.pos[1])).abs() > 3.0 {
                        continue;
                    }
                    loot.push((self.nav.cell_of(it.pos[0], it.pos[1]), 40.0));
                }
                if self.prm.farm > 0.5 {
                    for it in &self.crates {
                        if (it.pos[2] - self.map.walk_z(it.pos[0], it.pos[1])).abs() > 3.0 {
                            continue;
                        }
                        loot.push((self.nav.cell_of(it.pos[0], it.pos[1]), 30.0));
                    }
                }
            }
        }
        let time_cap = std::time::Instant::now();
        for (n, &(c, cost)) in reached.iter().enumerate() {
            let ci = c as usize;
            let (i, j) = (ci % w, ci / w);
            let lattice = if cost > 80.0 { i % 4 == 0 && j % 4 == 0 } else { i % 2 == 0 && j % 2 == 0 };
            let mut bonus = 0.0;
            if !loot.is_empty() {
                for &(lc, b) in &loot {
                    if lc == c {
                        bonus = f64::max(bonus, b);
                    } else {
                        let (li, lj) = ((lc as usize % w) as i64, (lc as usize / w) as i64);
                        if (li - i as i64).abs() <= 1 && (lj - j as i64).abs() <= 1 && lattice {
                            bonus = f64::max(bonus, b * 0.8);
                        }
                    }
                }
            }
            if !lattice && bonus == 0.0 && n != 0 {
                continue;
            }
            let (x, y) = self.nav.pos_of(c);
            // Never plan to stand outside the circle we are being squeezed into once we are in it.
            let out = (d2(&[x, y], &zone.nc) - safe_r).max(0.0);
            if me_out <= 0.0 && out > 0.0 {
                continue;
            }
            let arrive = cost as f64 / me.walk_speed;
            if zone.time_left(t, &[x, y]) < arrive + 3.0 && out > 0.0 && me_out <= out {
                continue;
            }
            let mut s = self.prm.w_travel * cost as f64 + self.cell_cost(c, me, zone, &threats, vis_mult, safe_r, t) - bonus;
            if Some(c) == self.goal {
                s -= 10.0;
            }
            if best.map_or(true, |b| s < b.0) {
                best = Some((s, c));
            }
            if n % 128 == 127 && time_cap.elapsed().as_micros() > 7000 {
                break;
            }
        }
        self.nav.reached = reached;
        self.goal_hidden = false;
        if let Some((_, c)) = best {
            self.goal = Some(c);
            let cells = self.nav.trace(c);
            // is the chosen spot out of every dangerous threat's sight, and how far is it?
            let (x, y) = self.nav.pos_of(c);
            let z = self.nav.wz[c as usize] as f64;
            let top = [x, y, z + me.height - 0.3];
            let mid = [x, y, z + me.height * 0.5];
            self.goal_hidden = !threats.iter().any(|th| th.w >= 0.4 && d2(&[x, y], &th.pos) < th.range && (self.map.los(th.eye, top) || self.map.los(th.eye, mid)));
            self.goal_cost = self.nav.flood_cost(c).unwrap_or(999.0) as f64;
            self.set_path_cells(&cells, me);
        }
        self.plan_t = t;
    }

    /// The tree crown to stand in right now: close, well hidden, inside the circle, away from known enemies.
    fn pick_hide(&self, me: &Me, zone: &Zone, t: f64) -> Option<usize> {
        if self.hides.is_empty() {
            return None;
        }
        let threats = self.build_threats(me, t, -1);
        let margin = (self.prm.edge_margin * zone.nr).clamp(3.0, 15.0);
        let safe_r = (zone.nr - margin - 4.0).max(0.0);
        let mut best: Option<(f64, usize)> = None;
        for (k, h) in self.hides.iter().enumerate() {
            let d = hyp(h.x - me.pos[0], h.y - me.pos[1]);
            if d > self.prm.tree_range {
                continue;
            }
            let p = [h.x, h.y];
            if d2(&p, &zone.c) > zone.r - 6.0 {
                continue;
            }
            let dn = d2(&p, &zone.nc);
            let mut c = d * 1.25 + 700.0 * h.vis as f64;
            if dn > safe_r {
                // outside the next circle: only while there is time to use it
                let left = zone.time_left(t, &p);
                if left < d / me.walk_speed + 30.0 {
                    continue;
                }
                c += 40.0 + (dn - safe_r) * 0.4;
            }
            if zone.nr > 100.0 {
                let inner = 0.5 * zone.nr;
                if dn < inner {
                    c += self.prm.center_avoid * (inner - dn) / inner;
                }
            }
            for th in &threats {
                let dd = d2(&p, &th.pos);
                if dd < 150.0 {
                    c += th.w * (150.0 - dd) * self.prm.w_tree_enemy;
                }
            }
            if Some(k) == self.hide_i {
                c -= 30.0;
            }
            if best.map_or(true, |b| c < b.0) {
                best = Some((c, k));
            }
        }
        best.map(|b| b.1)
    }

    /// Can a mech whose eye is at `eye` see or shoot any part of us if we stand at `q`?
    fn peek_hidden(&self, me: &Me, eye: [f64; 3], q: (f64, f64)) -> bool {
        let z = self.map.walk_z(q.0, q.1) as f64;
        let top = [q.0, q.1, z + me.height - 0.3];
        let mid = [q.0, q.1, z + me.height * 0.5];
        if self.map.los(eye, top) || self.map.los(eye, mid) {
            return false;
        }
        // the sides of the body as well: a shoulder past the corner can still be shot
        let (dx, dy) = (q.0 - eye[0], q.1 - eye[1]);
        let l = hyp(dx, dy).max(1e-6);
        let (nx, ny) = (-dy / l * (me.radius - 0.2), dx / l * (me.radius - 0.2));
        for sgn in [-1.0, 1.0] {
            let side = [q.0 + nx * sgn, q.1 + ny * sgn, z + me.height * 0.6];
            if self.map.los(eye, side) {
                return false;
            }
        }
        true
    }

    /// Does our muzzle at `q` have a clear line to the body of `e`?
    fn peek_shot(&self, me: &Me, e: &Track, q: (f64, f64)) -> bool {
        let z = self.map.walk_z(q.0, q.1) as f64;
        self.map.los([q.0, q.1, z + me.muzzle_h], e.center())
    }

    /// Nearby cover to fight `e` from: (score, hidden spot, firing spot), a few metres apart on a
    /// straight walk. Lower score is better.
    fn find_peek(&mut self, me: &Me, e: &Track, others: &[[f64; 3]]) -> Option<(f64, (f64, f64), (f64, f64))> {
        let need = (me.radius + 0.9) as f32;
        let here = (me.pos[0], me.pos[1]);
        self.nav.flood(here, need, self.prm.peek_reach as f32, None)?;
        let reached = std::mem::take(&mut self.nav.reached);
        let eye = e.eye();
        let mut hid: Vec<(f64, (f64, f64))> = Vec::new();
        let mut pk: Vec<((f64, f64), f64)> = Vec::new();
        for &(c, cost) in reached.iter().take(420) {
            let q = self.nav.pos_of(c);
            if self.peek_hidden(me, eye, q) {
                let exposed = others.iter().filter(|o| !self.peek_hidden(me, **o, q)).count();
                hid.push((cost as f64 + 18.0 * exposed as f64, q));
            } else if self.peek_shot(me, e, q) {
                pk.push((q, hyp(q.0 - e.pos[0], q.1 - e.pos[1])));
            }
        }
        self.nav.reached = reached;
        let line = (need - 0.4).max(2.6);
        let mut best: Option<(f64, (f64, f64), (f64, f64))> = None;
        for &(hc, hq) in &hid {
            // we must be able to walk there in a straight line, directly or by way of the firing spot
            let direct = nav::free_line(&self.map, here, hq, line);
            for &(pq, pd) in &pk {
                let d = hyp(hq.0 - pq.0, hq.1 - pq.1);
                if !(2.5..=8.0).contains(&d) {
                    continue;
                }
                let sc = hc + d * 2.5 + if pd < 45.0 { (45.0 - pd) * 0.6 } else { 0.0 };
                if best.map_or(false, |b| sc >= b.0) {
                    continue;
                }
                if !nav::free_line(&self.map, hq, pq, line) {
                    continue;
                }
                if !direct && !nav::free_line(&self.map, here, pq, line) {
                    continue;
                }
                best = Some((sc, hq, pq));
            }
        }
        best
    }

    /// One step of fighting from cover. None = no usable cover here, fight in the open as usual.
    fn peek_move(&mut self, me: &Me, e: &Track, others: &[[f64; 3]], t: f64) -> Option<([f64; 2], bool, f64)> {
        let dist = d2(&e.pos, &me.pos);
        if dist < 38.0 || t - e.seen_t > 1.0 {
            self.peek = None;
            return None;
        }
        // cover is for when somebody is actually shooting at us; otherwise keep the initiative
        let hot = t - e.hit_us_t < 5.0 || e.aim_us >= 3 || t - self.last_dmg_t < 3.0;
        if !hot && self.peek.map_or(true, |k| k.tgt != e.id) {
            self.peek = None;
            return None;
        }
        // keep the pair we have while it still works against where the target is now
        let eye = e.eye();
        let mut keep = false;
        if let Some(k) = self.peek {
            if k.tgt == e.id {
                keep = self.peek_hidden(me, eye, k.h) && self.peek_shot(me, e, k.p);
            }
        }
        let due = self.peek.map_or(true, |k| k.tgt != e.id || t - k.pick_t > if keep { 1.2 } else { 0.25 });
        if due {
            let found = self.find_peek(me, e, others);
            let old = self.peek.filter(|k| k.tgt == e.id);
            match (found, old) {
                (Some((sc, h, p)), Some(k)) => {
                    // only move house for a clearly better pair
                    let cur = if keep { hyp(k.h.0 - me.pos[0], k.h.1 - me.pos[1]) + 2.5 * hyp(k.h.0 - k.p.0, k.h.1 - k.p.1) } else { 1e9 };
                    if sc + 8.0 < cur {
                        // already out and the new firing spot is a few steps away: carry on shooting from there
                        let near_p = hyp(p.0 - me.pos[0], p.1 - me.pos[1]) < 7.0 && nav::free_line(&self.map, (me.pos[0], me.pos[1]), p, (me.radius + 0.5) as f32);
                        let phase = if k.phase == 2 && !near_p { 0 } else { k.phase };
                        self.peek = Some(Peek { h, p, tgt: e.id, pick_t: t, phase, phase_t: k.phase_t, hp0: k.hp0 });
                    } else {
                        self.peek = Some(Peek { pick_t: t, ..k });
                    }
                }
                (Some((_, h, p)), None) => {
                    self.peek = Some(Peek { h, p, tgt: e.id, pick_t: t, phase: 0, phase_t: t, hp0: me.hp });
                }
                (None, Some(k)) if keep => {
                    self.peek = Some(Peek { pick_t: t, ..k });
                }
                _ => {
                    self.peek = None;
                    return None;
                }
            }
        }
        let mut k = self.peek?;
        let here = (me.pos[0], me.pos[1]);
        let line = (me.radius + 0.5) as f32;
        let seen_now = !self.peek_hidden(me, eye, here);
        let gun = me.slot_of(GUN);
        let gun_ready = gun.map_or(false, |g| me.w[g].reload <= 0.0 && me.w[g].ammo >= me.w[g].mag * 0.7);
        let gun_dry = gun.map_or(true, |g| me.w[g].reload > 0.0 || me.w[g].ammo < 1.0);
        let laser = me.slot_of(LASER);
        let (lcd, lch, lct) = laser.map_or((9.0, 0.0, 1.0), |l| (me.w[l].cooldown, me.w[l].charge, me.w[l].charge_t));
        let hp_frac = me.hp / me.max_hp.max(1.0);
        let go = |to: (f64, f64)| -> [f64; 2] {
            let d = hyp(to.0 - here.0, to.1 - here.1);
            if d < 0.3 {
                [0.0, 0.0]
            } else {
                [(to.0 - here.0) / d, (to.1 - here.1) / d]
            }
        };
        let dh = hyp(k.h.0 - here.0, k.h.1 - here.1);
        let dp = hyp(k.p.0 - here.0, k.p.1 - here.1);
        let (mut mv, mut boost, mut sp) = ([0.0, 0.0], false, 1.0);
        match k.phase {
            0 => {
                if dh < 1.2 || (!seen_now && dh < 3.0) {
                    k.phase = 1;
                    k.phase_t = t;
                } else if nav::free_line(&self.map, here, k.h, line) {
                    mv = go(k.h);
                    boost = seen_now && dh > 5.0 && me.energy > 12.0;
                } else if dp > 1.0 && nav::free_line(&self.map, here, k.p, line) {
                    mv = go(k.p);
                    boost = seen_now && dp > 5.0 && me.energy > 12.0;
                } else {
                    self.peek = None;
                    return None;
                }
            }
            1 => {
                if seen_now && dh > 0.8 && nav::free_line(&self.map, here, k.h, line) {
                    mv = go(k.h);
                } else if dh > 2.5 && nav::free_line(&self.map, here, k.h, line) {
                    mv = go(k.h);
                    sp = 0.6;
                }
                let waited = t - k.phase_t;
                // a beam being held on our corner: let it go off into the wall first
                let their_beam = e.has[LASER] && e.charging && waited < 2.6;
                // badly hurt: stay put, they have to come to us and health comes back after 5 quiet seconds
                let nurse = hp_frac < self.prm.peek_nurse && waited < 14.0;
                // nothing to answer with at this range: stepping out only feeds their beam
                let eff = laser.is_some() || dist <= self.prm.outr_d || self.prm.outr < 0.5;
                let want_out = !their_beam && !nurse && eff;
                if laser.is_some() && lcd <= 0.2 && want_out {
                    self.peek_charge = true;
                }
                let ready = if laser.is_some() && lcd <= 1.3 { lcd <= 0.2 && lch >= lct * 0.3 } else { gun_ready };
                if want_out && ((waited >= 0.3 && ready) || waited > 3.4) {
                    k.phase = 2;
                    k.phase_t = t;
                    k.hp0 = me.hp;
                }
            }
            _ => {
                if laser.is_some() && lcd <= 0.2 {
                    self.peek_charge = true;
                }
                // far enough out once the muzzle has a line; keep the rest of the wall as cover
                let have_line = self.map.los([me.pos[0], me.pos[1], me.pos[2] + me.muzzle_h], e.center());
                if !have_line && dp > 0.8 {
                    if nav::free_line(&self.map, here, k.p, line) {
                        mv = go(k.p);
                    } else {
                        self.peek = None;
                        return None;
                    }
                }
                let out = t - k.phase_t;
                let laser_fired = laser.is_some() && lcd > 0.6 && lcd < 5.0;
                let done = if laser.is_some() {
                    (laser_fired && (gun_dry || out > 1.0)) || out > self.prm.peek_exp.max(1.0) + 0.6
                } else {
                    (gun_dry && out > 0.3) || out > self.prm.peek_exp.max(1.0) + 1.4
                } || me.hp < k.hp0 - 0.26 * me.max_hp;
                if done {
                    k.phase = 0;
                    k.phase_t = t;
                }
            }
        }
        self.peek = Some(k);
        Some((mv, boost, sp))
    }

    /// Are we standing on roof `ri`?
    fn on_roof(&self, me: &Me, ri: usize) -> bool {
        let r = &self.map.roofs[ri];
        if !me.grounded || me.pos[2] < r.top as f64 - 1.5 {
            return false;
        }
        match self.map.vidx(me.pos[0], me.pos[1]) {
            Some(v) => self.map.roof_id[v] == r.id,
            None => false,
        }
    }

    /// The roof to camp on: near, deep, tall enough to matter, low enough to reach, inside the circle.
    fn pick_roof(&self, me: &Me, zone: &Zone, t: f64) -> Option<usize> {
        let threats = self.build_threats(me, t, -1);
        let margin = (self.prm.edge_margin * zone.nr).clamp(3.0, 15.0);
        let safe_r = (zone.nr - margin - 6.0).max(0.0);
        let mut best: Option<(f64, usize)> = None;
        for (k, r) in self.map.roofs.iter().enumerate() {
            if (r.depth as f64) < self.prm.roof_depth || (r.rise as f64) < self.prm.roof_min_rise || r.rise > 46.0 || r.access.is_empty() || self.roof_bad.contains(&k) {
                continue;
            }
            let p = [r.best.0, r.best.1];
            let on = self.on_roof(me, k);
            let d = if on { 0.0 } else { r.access.iter().map(|a| hyp(a.0 - me.pos[0], a.1 - me.pos[1])).fold(1e9, f64::min) };
            if d > self.prm.roof_range || d2(&p, &zone.c) > zone.r - 8.0 {
                continue;
            }
            let dn = d2(&p, &zone.nc);
            let mut c = d * 1.2 - (r.depth as f64).min(30.0) * 2.5 - (r.rise as f64).min(40.0) * 1.2;
            if dn > safe_r {
                let left = zone.time_left(t, &p);
                if left < d / me.walk_speed + 45.0 {
                    continue;
                }
                c += 50.0 + (dn - safe_r) * 0.4;
            }
            for th in &threats {
                let dd = d2(&p, &th.pos);
                if dd < 120.0 {
                    c += th.w * (120.0 - dd) * 0.8;
                }
            }
            if Some(k) == self.roof_i {
                c -= 40.0;
            }
            if on {
                c -= 60.0;
            }
            if best.map_or(true, |b| c < b.0) {
                best = Some((c, k));
            }
        }
        best.map(|b| b.1)
    }

    /// Walkable region id under (x, y) at about height z: the nearest roomy cell within a few metres.
    fn region_at(&self, x: f64, y: f64, z: f64) -> u32 {
        let c = self.nav.cell_of(x, y) as usize;
        let (ci, cj) = ((c % self.nav.w) as i32, (c / self.nav.w) as i32);
        let mut best: Option<(i32, u32)> = None;
        for dj in -2..=2i32 {
            for di in -2..=2i32 {
                let (i, j) = (ci + di, cj + dj);
                if i < 0 || j < 0 || i >= self.nav.w as i32 || j >= self.nav.h as i32 {
                    continue;
                }
                let k = j as usize * self.nav.w + i as usize;
                let id = self.nav.comp[k];
                if id == 0 || (self.nav.wz[k] as f64 - z).abs() > 3.0 {
                    continue;
                }
                let dd = di * di + dj * dj;
                if best.map_or(true, |b| dd < b.0) {
                    best = Some((dd, id));
                }
            }
        }
        best.map_or(0, |b| b.1)
    }

    /// Can we walk to something lying at `p`? (A weapon on a roof sits on "the ground" of that roof.)
    fn walkable_item(&self, me: &Me, p: &[f64; 3]) -> bool {
        if (p[2] - self.map.walk_z(p[0], p[1]) as f64).abs() > 3.0 {
            return false;
        }
        if self.prm.reach_fix < 0.5 {
            return true;
        }
        let mine = self.region_at(me.pos[0], me.pos[1], me.pos[2]);
        let its = self.region_at(p[0], p[1], p[2]);
        its != 0 && (mine == 0 || mine == its)
    }

    fn want_pickup(&self, me: &Me, kind: usize) -> f64 {
        let (mr, lr, zr) = (self.prm.mortar_rank, self.prm.laser_rank, self.prm.lightning_rank);
        let rank = |k: usize| -> f64 {
            match k {
                LASER => lr,
                LIGHTNING => zr,
                MORTAR => mr,
                _ => 0.0,
            }
        };
        if kind == GUN || me.slot_of(kind).is_some() || rank(kind) <= 0.0 {
            return 0.0;
        }
        if !me.w[0].present || !me.w[1].present {
            return rank(kind);
        }
        // both full: swap only up the ranking, replacing the non-gun slot
        let other = if me.w[0].kind == GUN { 1 } else { 0 };
        if rank(kind) > rank(me.w[other].kind) {
            rank(kind) - rank(me.w[other].kind)
        } else {
            0.0
        }
    }

    fn route_to(&mut self, me: &Me, goal: (f64, f64)) -> bool {
        let need = (me.radius + 0.9) as f32;
        let pc = std::mem::take(&mut self.path_cost);
        let mut found = self.nav.path((me.pos[0], me.pos[1]), goal, need, Some(&pc), 30_000);
        if found.is_none() {
            // pockets joined by a gap we fit through but the planner's margin rejects
            found = self.nav.path((me.pos[0], me.pos[1]), goal, (me.radius + 0.3) as f32, Some(&pc), 30_000);
        }
        self.path_cost = pc;
        match found {
            Some(cells) => {
                self.set_path_cells(&cells, me);
                if let Some(last) = self.path.last_mut() {
                    // finish on the exact point when it is walkable
                    if self.map.wall_dist(goal.0, goal.1) >= need - 0.5 {
                        *last = goal;
                    }
                }
                true
            }
            None => false,
        }
    }

    /// Remaining length of the current path, measured along it.
    fn path_len(&self, me: &Me) -> f64 {
        if self.path.is_empty() || self.path_i >= self.path.len() {
            return 0.0;
        }
        let mut prev = (me.pos[0], me.pos[1]);
        let mut sum = 0.0;
        for k in self.path_i..self.path.len() {
            let q = self.path[k];
            sum += hyp(q.0 - prev.0, q.1 - prev.1);
            prev = q;
        }
        sum
    }

    /// Steering target along the current path: (direction, remaining distance to the end)
    fn follow(&mut self, me: &Me) -> ([f64; 2], f64) {
        if self.path.is_empty() {
            return ([0.0, 0.0], 0.0);
        }
        let p = (me.pos[0], me.pos[1]);
        let n = self.path.len();
        // advance while the next waypoint is closer than the current one or we are on top of it
        while self.path_i + 1 < n {
            let a = self.path[self.path_i];
            let b = self.path[self.path_i + 1];
            let da = hyp(a.0 - p.0, a.1 - p.1);
            let db = hyp(b.0 - p.0, b.1 - p.1);
            if da < 1.6 || db < da {
                self.path_i += 1;
            } else {
                break;
            }
        }
        let need = (me.radius + 0.45) as f32;
        let mut k = self.path_i;
        let lim = (self.path_i + 14).min(n - 1);
        let mut cand = self.path_i + 1;
        while cand <= lim {
            if nav::free_line(&self.map, p, self.path[cand], need) {
                k = cand;
            } else if cand > self.path_i + 3 {
                break;
            }
            cand += 1;
        }
        let tgt = self.path[k];
        let end = self.path[n - 1];
        let rem = hyp(end.0 - p.0, end.1 - p.1);
        let (dx, dy) = (tgt.0 - p.0, tgt.1 - p.1);
        let d = hyp(dx, dy);
        if d < 0.6 && k == n - 1 {
            return ([0.0, 0.0], rem);
        }
        ([dx / d.max(1e-6), dy / d.max(1e-6)], rem)
    }

    // ------------------------------------------------------------------ main

    pub fn decide(&mut self, o: &J) -> Action {
        let t0 = std::time::Instant::now();
        let t = o.get("t").f();
        let me = self.read_self(o);
        self.peek_charge = false;
        if self.prm.probe > 0.5 && !self.st_done {
            if let Some(gs) = me.slot_of(GUN) {
                let ammo = me.w[gs].ammo;
                if self.st_prev_ammo >= 0.0 && ammo < self.st_prev_ammo && self.st_had_tgt {
                    self.st_gun_shots += (self.st_prev_ammo - ammo).round() as u32;
                }
                self.st_prev_ammo = ammo;
            }
            let taken: f64 = self.st_dmg.iter().sum();
            let kind = self.prm.probe.round() as i32;
            if kind == 1 && self.st_gun_shots >= 16 {
                self.st_done = true;
                // allow for rounds still in the air: judge on what has landed by now plus a grace of two
                if ((self.st_gun_hits + 2) as f64) / (self.st_gun_shots as f64) < self.prm.probe_thr {
                    self.force_idle = true;
                }
            } else if kind == 2 && taken >= 300.0 {
                self.st_done = true;
                if self.st_dmg[LASER] / taken >= self.prm.probe_thr {
                    self.force_idle = true;
                }
            } else if kind == 3 && taken >= 300.0 {
                self.st_done = true;
                if self.st_attackers.len() >= 2 {
                    self.force_idle = true;
                }
            } else if kind == 10 && t >= self.prm.probe_thr {
                // when do we usually die? stop playing at a fixed time and see what it costs
                self.st_done = true;
                self.force_idle = true;
            } else if kind == 11 && 16.0 - o.get("alive").fo(16.0) >= self.prm.probe_thr - 0.5 {
                // how many of the others do we usually outlive? stop once that many are dead
                self.st_done = true;
                self.force_idle = true;
            } else if kind == 12 && self.tail_hits.iter().filter(|h| t - h.1 < self.prm.probe_win).count() >= 2 {
                // once `probe_thr` are dead: are we then set upon by two at a time (both within the last few seconds)?
                self.st_done = true;
                self.force_idle = true;
            } else if (13..=15).contains(&kind) && self.tail_total >= self.prm.probe_amt {
                // what the damage after that point is made of: 13 laser, 14 lightning, 15 from inside 50 m
                self.st_done = true;
                let share = match kind {
                    13 => self.tail_dmg[LASER],
                    14 => self.tail_dmg[LIGHTNING],
                    _ => self.tail_near,
                } / self.tail_total.max(1.0);
                if share >= self.prm.probe_share {
                    self.force_idle = true;
                }
            } else if kind == 9 && self.st_taken_all >= 400.0 {
                // how the trade is going by the time we are down 400
                self.st_done = true;
                if self.st_dealt_all / self.st_taken_all < self.prm.probe_thr {
                    self.force_idle = true;
                }
            } else if kind == 7 && taken >= 300.0 {
                self.st_done = true;
                if self.st_far / taken >= self.prm.probe_thr {
                    self.force_idle = true;
                }
            } else if kind == 8 && taken >= 300.0 {
                self.st_done = true;
                if self.st_unseen / taken >= self.prm.probe_thr {
                    self.force_idle = true;
                }
            } else if kind == 4 && t >= self.prm.probe_thr {
                self.st_done = true;
                if me.cubes < 0.5 {
                    self.force_idle = true;
                }
            } else if kind == 5 && t >= self.prm.probe_thr {
                self.st_done = true;
                if !me.w[1].present {
                    self.force_idle = true;
                }
            } else if kind == 6 && (t >= self.prm.probe_thr || taken >= 300.0) {
                self.st_done = true;
                if taken >= 300.0 {
                    self.force_idle = true;
                }
            }
        }
        if self.prm.idle > 0.5 || self.force_idle {
            // diagnostic statue: hold the spawn facing, do nothing at all
            return Action { aim: me.aim, ..Default::default() };
        }
        self.update_tracks(o, t);
        self.update_items(o, &me, t);
        self.read_events(o, &me, t);
        let zone = Zone::read(o.get("zone"));
        let alive = o.get("alive").fo(16.0);
        let mut act = Action { aim: self.last_aim, ..Default::default() };

        // ---- upgrades
        let offer = o.get("offer");
        if !offer.is_null() {
            let ch = offer.get("choices").arr();
            let mut best = (usize::MAX, 0usize);
            const SURVIVE: [&str; 22] = [
                "plating", "armor", "nanites", "servos", "laser_damage", "gun_mag", "laser_charge", "gun_damage", "gun_rate", "sensors", "reactor",
                "capacitor", "gyros", "jets", "gun_velocity", "laser_pierce", "lightning_efficiency", "lightning_chain", "lightning_range",
                "mortar_reload", "mortar_splash", "mortar_velocity",
            ];
            for (i, c) in ch.iter().enumerate() {
                let id = c.get("id").s();
                let r = if self.prm.upg > 0.5 { SURVIVE.iter().position(|p| *p == id).unwrap_or(100) } else { self.upgrades_pref.iter().position(|p| *p == id).unwrap_or(100) };
                if r < best.0 {
                    best = (r, i);
                }
            }
            if !ch.is_empty() {
                act.pick = Some(best.1);
            }
        }

        // ---- who can see us, who is after us
        let my_eye = [me.pos[0], me.pos[1], me.pos[2] + me.eye_h];
        let my_muzzle = [me.pos[0], me.pos[1], me.pos[2] + me.muzzle_h];
        let my_center = [me.pos[0], me.pos[1], me.pos[2] + me.height * 0.5];
        let my_top = [me.pos[0], me.pos[1], me.pos[2] + me.height - 0.2];
        struct Info {
            i: usize,
            d: f64,
            sees: bool,
            hostile: bool,
            shot: bool,
            los: bool,
            listed: bool,
            their_dps: f64,
        }
        let mut infos: Vec<Info> = Vec::new();
        let cube_mult = 1.0 + 0.04 * me.cubes.min(self.stat_cap);
        for i in 0..self.tracks.len() {
            if self.tracks[i].npc {
                continue;
            }
            let (eye, center, id, aim0, charging, has, cubes, listed) = {
                let e = &self.tracks[i];
                (e.eye(), e.center(), e.id, e.aim[0], e.charging, e.has, e.cubes, e.seen_t >= t)
            };
            if id == me.id {
                continue;
            }
            let d = d2(&center, &me.pos);
            let bearing = (me.pos[1] - center[1]).atan2(me.pos[0] - center[0]);
            let off = wrap(aim0 - bearing).abs();
            let los = self.map.los(eye, my_center) || self.map.los(eye, my_top);
            let in_fov = off <= self.fov * 0.5 + 0.1;
            let sees = listed && los && (d <= self.near_range + 3.0 || (in_fov && d <= self.vis_range + 10.0));
            let aiming = listed && los && off < 0.15;
            {
                let e = &mut self.tracks[i];
                if listed {
                    if aiming {
                        e.aim_us += 1;
                    } else {
                        e.aim_us = 0;
                    }
                }
            }
            let e = &self.tracks[i];
            let hostile = t - e.hit_us_t < self.prm.grudge || (self.prm.calm_aim < 0.5 && e.aim_us >= 3) || (charging && off < 0.3 && los && listed);
            let shot = listed && self.map.los(my_muzzle, center);
            let acc = (0.85 - d / 400.0).clamp(0.3, 0.85);
            let mut their_dps = 0.0;
            if has[GUN] {
                their_dps += 187.0 * acc * 0.9;
            }
            if has[LASER] {
                their_dps += 105.0;
            }
            if has[LIGHTNING] && d < 50.0 {
                their_dps += 220.0;
            }
            if has[MORTAR] {
                their_dps += 40.0;
            }
            their_dps *= 1.0 + 0.04 * cubes.min(10.0);
            infos.push(Info { i, d, sees, hostile, shot, los, listed, their_dps });
        }
        let my_dps_at = |d: f64| -> f64 {
            let acc = (0.85 - d / 400.0).clamp(0.3, 0.85);
            let mut dps = 0.0;
            for w in &me.w {
                if !w.present {
                    continue;
                }
                match w.kind {
                    GUN => dps += w.damage / w.cd.max(0.05) * acc * (w.mag * w.cd / (w.mag * w.cd + 2.4)).max(0.5),
                    LASER => {
                        if d < w.range {
                            dps += w.damage / (w.charge_t + 1.65)
                        }
                    }
                    LIGHTNING => {
                        if d < w.range - 2.0 {
                            dps += w.damage / w.cd.max(0.05)
                        }
                    }
                    _ => dps += 30.0,
                }
            }
            dps * cube_mult
        };

        let under_fire = t - self.last_dmg_t < 2.5;
        let hp_frac = me.hp / me.max_hp.max(1.0);
        let endgame = zone.nr <= self.prm.end_radius || zone.r <= self.prm.end_radius + 5.0;
        let hostile_los: Vec<usize> = (0..infos.len()).filter(|&k| infos[k].hostile && infos[k].los && infos[k].listed).collect();
        let seen_by: Vec<usize> = (0..infos.len()).filter(|&k| infos[k].sees).collect();
        if self.prm.retreat_hp > 0.0 && !endgame {
            if !self.retreating && hp_frac < self.prm.retreat_hp && (!hostile_los.is_empty() || under_fire) {
                self.retreating = true;
            } else if self.retreating && hp_frac >= self.prm.resume_hp {
                self.retreating = false;
            }
        } else {
            self.retreating = false;
        }
        let retreating = self.retreating;
        if self.prm.heal > 0.0 && !endgame {
            if hp_frac < self.prm.heal {
                self.healing = true;
            } else if hp_frac >= 0.97 {
                self.healing = false;
            }
        } else {
            self.healing = false;
        }
        let healing = self.healing && !under_fire && hostile_los.is_empty() && me.in_zone;
        // still on the starting gun early in the match, with a better weapon known nearby
        let weapon_known = self.prm.arm_known < 0.5
            || self.pickups.iter().any(|it| {
                self.want_pickup(&me, it.kind) > 0.0
                    && d2(&it.pos, &me.pos) < self.prm.seek_range * 1.4
                    && d2(&it.pos, &zone.c) < zone.r - 5.0
                    && self.walkable_item(&me, &it.pos)
                    && !self.seek_ban.iter().any(|b| b.0 == it.id)
            });
        let arming = self.prm.arm_first > 0.0 && t < self.prm.arm_first && !me.w[1].present && !endgame && weapon_known;
        let solo = self.prm.solo > 0.5 && !endgame;
        let recent_attackers = self.tracks.iter().filter(|e| !e.npc && t - e.hit_us_t < 4.0).count();
        let outnumbered = solo && (hostile_los.len() >= 2 || recent_attackers >= 2);
        if outnumbered {
            self.lay_low_until = t + self.prm.lay_low;
        }
        let laying_low = solo && t < self.lay_low_until;
        let in_view = infos.iter().filter(|f| f.listed && f.los && f.d < 220.0).count();

        // ---- pick a fight target (index into infos)
        let mut tgt: Option<usize> = None;
        let mut tgt_score = f64::NEG_INFINITY;
        let recovering = hp_frac < 0.5 && !under_fire;
        for (k, inf) in infos.iter().enumerate() {
            if !inf.shot || !inf.listed {
                continue;
            }
            let e = &self.tracks[inf.i];
            let max_range = me.w.iter().filter(|w| w.present && w.kind != LIGHTNING && w.kind != MORTAR).map(|w| w.range).fold(0.0, f64::max);
            if inf.d > max_range.max(60.0) + 5.0 {
                continue;
            }
            let ttk = e.hp / my_dps_at(inf.d).max(1.0);
            let ttd = me.hp / inf.their_dps.max(1.0);
            let favorable = ttd > ttk * self.prm.fight_margin;
            let unaware = !inf.sees;
            // a long gun burst mostly announces us; finish at range only with a beam
            let finish = e.hp <= self.prm.finish_hp * cube_mult && (inf.d < 130.0 || (me.slot_of(LASER).is_some() && inf.d < 250.0));
            let engage = if self.prm.kite > 0.5 {
                // keeping away: answer only the ones close enough to hit us (and to be hit)
                endgame || inf.d < self.prm.pb_range || (inf.hostile && (inf.d < self.prm.kite_fire || e.hp <= 300.0 * cube_mult)) || (self.prm.ambush >= 1.0 && finish && inf.d < 90.0)
            } else {
                inf.hostile
                    || endgame
                    || (self.prm.laser_engage > 0.5 && me.slot_of(LASER).is_some() && inf.d < 245.0 && hp_frac > 0.4)
                    || inf.d < self.prm.pb_range
                    || (self.prm.sees_engage > 0.5 && inf.sees && inf.d < 175.0 && favorable)
                    || (self.prm.ambush >= 1.0 && finish)
                    || (self.prm.ambush >= 2.0 && unaware && inf.d < 140.0 && hp_frac > 0.8 && favorable && infos.len() <= 2)
            };
            let peek_tgt = self.peek.map_or(false, |k| k.tgt == e.id);
            let prey = self.prm.chase > 0.5 && !endgame && e.hp < self.prm.chase_frac * e.max_hp && inf.d < 210.0 && hp_frac > 0.5;
            let engage = engage || peek_tgt || prey;
            if !engage {
                continue;
            }
            if (recovering || healing) && !inf.hostile && !peek_tgt && !prey && !endgame && inf.d > self.prm.pb_range {
                continue;
            }
            if retreating && inf.d > 38.0 {
                continue;
            }
            if arming && !(inf.hostile && (inf.d < 80.0 || self.prm.arm_safe > 0.5)) && inf.d > 45.0 {
                continue;
            }
            if t < self.prm.peace && !endgame && !(inf.hostile && inf.d < 90.0) && inf.d > 45.0 {
                continue;
            }
            if solo && !inf.hostile && inf.d > 40.0 && (!hostile_los.is_empty() || in_view >= 2 || laying_low) {
                continue;
            }
            if outnumbered && inf.d > 45.0 {
                continue;
            }
            let mut sc = 0.0;
            if inf.hostile {
                sc += 100.0;
            }
            sc += (1.0 - e.hp / e.max_hp.max(1.0)) * 60.0;
            if e.hp < 350.0 {
                sc += 80.0;
            }
            sc -= inf.d * 0.25;
            if unaware {
                sc += 20.0;
            }
            if e.id == self.target_id {
                sc += 15.0;
            }
            if sc > tgt_score {
                tgt_score = sc;
                tgt = Some(k);
            }
        }
        self.target_id = tgt.map(|k| self.tracks[infos[k].i].id).unwrap_or(-1);

        // ---- stance
        let mut fight_id = -1i64;
        let mut vis_mult = 1.0;
        let mut mode = Mode::Hide;
        let kite = self.prm.kite > 0.5;
        // keeping away: anyone who sees us, or is known to be close, is reason enough to move
        let mut near_track = false;
        let mut close_track = false;
        if kite {
            for e in &self.tracks {
                if e.npc || e.id == me.id {
                    continue;
                }
                let age = t - e.seen_t;
                if age > 6.0 {
                    continue;
                }
                let lead = age.min(1.5);
                let d = hyp(e.pos[0] + e.vel[0] * lead - me.pos[0], e.pos[1] + e.vel[1] * lead - me.pos[1]) - 9.0 * age;
                if d < self.prm.near_d {
                    near_track = true;
                }
                if d < 90.0 {
                    close_track = true;
                }
            }
        }
        let desperate = self.prm.last_hp > 0.0 && hp_frac < self.prm.last_hp && !endgame;
        let no_laser = me.slot_of(LASER).is_none();
        let outr_on = self.prm.outr > 0.5 && no_laser && !endgame;
        let shy_on = self.prm.shy > 0.5 && !endgame && (self.prm.shy > 1.5 || no_laser);
        // someone has a line on us from too far away to be worth fighting
        let far_watch = shy_on && infos.iter().any(|f| f.listed && f.los && f.d > self.prm.shy_d && f.d < 260.0);
        let outranged = (outr_on && hostile_los.iter().any(|&k| infos[k].d > self.prm.outr_d && (self.prm.outr > 1.5 || self.tracks[infos[k].i].has[LASER])))
            || (shy_on && !hostile_los.is_empty() && hostile_los.iter().all(|&k| infos[k].d > self.prm.shy_d));
        // where the mechs we have to stay hidden from look from
        let beam_eyes: Vec<[f64; 3]> = if shy_on {
            self.tracks.iter().filter(|e| !e.npc && t - e.seen_t < 8.0 && d2(&e.pos, &me.pos) < 260.0 && d2(&e.pos, &me.pos) > 30.0).map(|e| e.eye()).collect()
        } else if outr_on {
            self.tracks.iter().filter(|e| !e.npc && (e.has[LASER] || self.prm.outr > 1.5) && t - e.hit_us_t < 12.0 && t - e.seen_t < 12.0).map(|e| e.eye()).collect()
        } else {
            Vec::new()
        };
        let beam_sees = |p: [f64; 3], map: &Map| beam_eyes.iter().any(|eye| map.los(*eye, [p[0], p[1], p[2] + 4.5]));
        let space = if endgame { 0.0 } else { self.prm.space };
        let crowd: Vec<[f64; 2]> = if space > 0.0 { self.tracks.iter().filter(|e| !e.npc && t - e.seen_t < 6.0).map(|e| [e.pos[0], e.pos[1]]).collect() } else { Vec::new() };
        let crowded = |p: &[f64; 3]| crowd.iter().any(|c| hyp(p[0] - c[0], p[1] - c[1]) < space);
        let exposed = !hostile_los.is_empty() || under_fire || !seen_by.is_empty();
        let threatened = retreating || !hostile_los.is_empty() || under_fire || far_watch || (kite && (!seen_by.is_empty() || near_track));
        if threatened {
            mode = Mode::Flee;
            vis_mult = 2.5;
            // the hostile we would rather shoot: closest to dying per second of our fire
            let pick = hostile_los.iter().copied().min_by(|&a, &b| {
                let ka = self.tracks[infos[a].i].hp / my_dps_at(infos[a].d).max(1.0);
                let kb = self.tracks[infos[b].i].hp / my_dps_at(infos[b].d).max(1.0);
                ka.partial_cmp(&kb).unwrap_or(std::cmp::Ordering::Equal)
            });
            if let Some(hk) = pick {
                let inf = &infos[hk];
                let e = &self.tracks[inf.i];
                let ttk = e.hp / my_dps_at(inf.d).max(1.0);
                let all_dps: f64 = hostile_los.iter().map(|&k| infos[k].their_dps).sum();
                let ttd = me.hp / all_dps.max(1.0);
                let stand = t - self.held_cover_t < 0.6 && inf.d < 90.0 && hp_frac > 0.3;
                let favorable = ttd > ttk * self.prm.fight_margin && hp_frac > 0.35;
                let point_blank = hostile_los.iter().any(|&k| infos[k].d < 42.0);
                // running only helps when there is cover to run to
                let cover_near = self.goal_hidden && self.goal_cost < self.prm.cover_reach && t - self.plan_t < 0.5 && self.flee_planned;
                if outnumbered {
                    if hostile_los.iter().any(|&k| infos[k].d < 30.0) {
                        mode = Mode::Fight;
                        fight_id = e.id;
                        vis_mult = 2.0;
                    }
                } else if retreating {
                    // getting away comes first; turn and fight only when one is on top of us
                    if hostile_los.iter().any(|&k| infos[k].d < 32.0) {
                        mode = Mode::Fight;
                        fight_id = e.id;
                        vis_mult = 2.0;
                    }
                } else if kite {
                    let pb = hostile_los.iter().any(|&k| infos[k].d < self.prm.pb_range.max(20.0));
                    let cornered = !cover_near && inf.d < 55.0 && t - self.plan_t < 0.5 && self.flee_planned;
                    if pb || cornered || (e.hp < 260.0 && inf.d < 90.0) {
                        mode = Mode::Fight;
                        fight_id = e.id;
                        vis_mult = 2.0;
                    }
                } else if outranged && !desperate {
                    let cover = self.goal_hidden && self.goal_cost < self.prm.outr_reach && t - self.plan_t < 0.5 && self.flee_planned;
                    if point_blank || !cover {
                        mode = Mode::Fight;
                        fight_id = e.id;
                        vis_mult = 2.0;
                    }
                } else if desperate {
                    // nearly dead: a wall between us and them is worth more than the last few shots
                    let cover = self.goal_hidden && self.goal_cost < self.prm.last_reach && t - self.plan_t < 0.5 && self.flee_planned;
                    if point_blank || e.hp < 200.0 || !cover {
                        mode = Mode::Fight;
                        fight_id = e.id;
                        vis_mult = 2.0;
                    }
                } else if favorable || e.hp < 300.0 || stand || point_blank || !cover_near {
                    mode = Mode::Fight;
                    fight_id = e.id;
                    vis_mult = if favorable { 1.0 } else { 2.0 };
                }
            }
        }
        if self.prm.press > 0.5 && mode == Mode::Hide && !retreating {
            if let Some(k) = tgt {
                mode = Mode::Fight;
                fight_id = self.tracks[infos[k].i].id;
            }
        }
        // fighting from cover: while we are out of sight between two looks, the fight is still on
        if self.prm.peek > 0.5 && mode != Mode::Fight {
            match self.peek {
                Some(k) if !retreating && !outnumbered && !desperate && !outranged && t - k.pick_t < 1.6 && self.tracks.iter().any(|e| e.id == k.tgt && t - e.seen_t < 0.5) => {
                    mode = Mode::Fight;
                    fight_id = k.tgt;
                }
                _ => self.peek = None,
            }
        }
        let margin = (self.prm.edge_margin * zone.nr).clamp(3.0, 15.0);
        let safe_r = (zone.nr - margin).max(0.0);
        let me_out = (d2(&me.pos, &zone.nc) - safe_r).max(0.0);
        let zone_left = zone.time_left(t, &[me.pos[0], me.pos[1]]);
        let lazy = self.prm.lazy > 0.5;
        // only while the wall is slow and mild (the first two circles); later ones get the careful rule
        let zroam = self.prm.zroam > 0.5 && !lazy && zone.dps < 50.0;
        let (zone_urgent, go_now) = if !lazy && !zroam {
            // walking routes bend around buildings: budget well over the straight-line time
            (me_out > 0.0 && (!me.in_zone || zone_left < me_out / me.walk_speed * 2.2 + 8.0), me_out > 3.0)
        } else if me_out <= 3.0 {
            self.zone_moving = false;
            (false, false)
        } else {
            // Stay put until the wall is nearly on us, then walk in once. Know the real walking distance.
            if t - self.zone_plen_t > 2.0 || self.zone_plen <= 0.0 {
                let need = (me.radius + 0.9) as f32;
                let pc = std::mem::take(&mut self.path_cost);
                let disk_r = (safe_r - 5.0).max(1.0);
                let found = self.nav.path_to_disk((me.pos[0], me.pos[1]), (zone.nc[0], zone.nc[1]), disk_r, need, Some(&pc), 30_000);
                self.path_cost = pc;
                self.zone_plen = match found {
                    Some(cells) => {
                        let mut prev = (me.pos[0], me.pos[1]);
                        let mut sum = 0.0;
                        for c in cells {
                            let q = self.nav.pos_of(c);
                            sum += hyp(q.0 - prev.0, q.1 - prev.1);
                            prev = q;
                        }
                        sum.max(me_out)
                    }
                    None => me_out * 1.6 + 20.0,
                };
                self.zone_plen_t = t;
            }
            let tw = self.zone_plen / me.walk_speed;
            let hot = zone.dps >= 60.0;
            let pad = if zroam { self.prm.zroam_pad } else { 0.0 };
            let lead = 3.0 + (tw - me_out / me.walk_speed).clamp(0.0, 12.0) * 0.5 + if hot { 2.5 } else { 0.0 } + pad;
            let t_dep = zone.t1 - tw * 1.25 - if hot { 8.0 } else { 4.0 } - pad;
            let go = !me.in_zone || zone_left < lead || t >= t_dep || self.zone_moving;
            self.zone_moving = go;
            (!me.in_zone || zone_left < lead * 0.5, go)
        };
        if endgame {
            mode = Mode::End;
        } else if zone_urgent || (go_now && mode != Mode::Fight) {
            // outside the next circle: get inside first, then look for a spot
            mode = Mode::ZoneRun;
        }

        // ---- a better weapon within reach of a short run is worth more than anything else we could be doing
        let mut seek_goal: Option<((f64, f64), i64)> = None;
        if self.prm.seek_w > 0.5 && !laying_low && (mode == Mode::Hide || mode == Mode::Flee || (arming && mode == Mode::Fight)) {
            let hostile_close = hostile_los.iter().any(|&k| infos[k].d < if arming { 40.0 } else { 75.0 });
            let mut best: Option<(f64, (f64, f64), i64, f64)> = None;
            self.seek_ban.retain(|b| b.1 > t);
            for it in &self.pickups {
                let want = self.want_pickup(&me, it.kind);
                if want <= 0.0 || (it.id == self.take_fail_id && t - self.take_fail_t < 6.0) || self.seek_ban.iter().any(|b| b.0 == it.id) {
                    continue;
                }
                if !self.walkable_item(&me, &it.pos) {
                    continue;
                }
                let d = d2(&it.pos, &me.pos);
                let reach = if it.kind == LASER && self.prm.laser_range > 0.0 && me.slot_of(LASER).is_none() { self.prm.laser_range } else { self.prm.seek_range * if arming { 1.4 } else { 1.0 } };
                if d > reach || d2(&it.pos, &zone.c) > zone.r - 5.0 {
                    continue;
                }
                if zroam && zone.time_left(t, &[it.pos[0], it.pos[1]]) < d / me.walk_speed * 1.5 + 6.0 {
                    continue;
                }
                if d > 18.0 && beam_sees(it.pos, &self.map) {
                    continue;
                }
                if far_watch && d > 60.0 {
                    // being watched right now: nothing that far is worth the walk in the open
                    continue;
                }
                if d > 25.0 && crowded(&it.pos) {
                    continue;
                }
                let sc = want / (d + 25.0) + if it.id == self.seek_id { 0.004 } else { 0.0 };
                if best.map_or(true, |b| sc > b.0) {
                    best = Some((sc, (it.pos[0], it.pos[1]), it.id, d));
                }
            }
            if let Some((_, gp, id, d)) = best {
                let go = if self.prm.arm_safe > 0.5 && mode != Mode::Hide {
                    // someone has a line on us: only if it is nearly in hand or they are too far to hurt much
                    let nearest = hostile_los.iter().map(|&k| infos[k].d).fold(f64::INFINITY, f64::min);
                    d < 28.0 || (nearest > 115.0 && (d < 70.0 || arming))
                } else {
                    mode == Mode::Hide || (!hostile_close && (d < 70.0 || arming))
                };
                let far_for_now = healing && self.prm.heal_reach > 0.0 && hp_frac < 0.6 && d > self.prm.heal_reach;
                let cover_first = outranged && mode == Mode::Flee && d > 28.0;
                if go && !far_for_now && !cover_first {
                    seek_goal = Some((gp, id));
                }
            }
        }
        // ---- a hurt mech that has just ducked out of sight: the server still tells us where it is
        if self.prm.chase > 0.5 && mode == Mode::Hide && !endgame && hp_frac > 0.55 && seek_goal.map_or(true, |g| hyp(g.0 .0 - me.pos[0], g.0 .1 - me.pos[1]) > 30.0) {
            let mut best: Option<(f64, (f64, f64), i64)> = None;
            for e in &self.tracks {
                if e.npc || t - e.seen_t > 0.3 || e.hp >= self.prm.chase_frac * e.max_hp || t - e.hp_drop_t > 9.0 {
                    continue;
                }
                let d = d2(&e.pos, &me.pos);
                if d > 220.0 || d2(&e.pos, &zone.c) > zone.r - 5.0 {
                    continue;
                }
                let sid = -100 - e.id;
                if self.seek_ban.iter().any(|b| b.0 == sid) {
                    continue;
                }
                let key = e.hp + d;
                if best.map_or(true, |b| key < b.0) {
                    best = Some((key, (e.pos[0], e.pos[1]), sid));
                }
            }
            if let Some((_, gp, sid)) = best {
                seek_goal = Some((gp, sid));
            }
        }
        let arming_run = arming && seek_goal.is_some();
        // ---- a supply drop we can be at when it lands
        let mut drop_hold = false;
        let mut drop_goal = false;
        if self.prm.drop_go > 0.5 && (mode == Mode::Hide || mode == Mode::Flee) && !laying_low && !endgame {
            let mut best: Option<(f64, (f64, f64), i64, f64)> = None;
            for dr in o.get("drops").arr() {
                let p = dr.get("pos").v3();
                let id = dr.get("id").f() as i64;
                let lands = dr.get("lands_in").fo(0.0);
                let r = dr.get("radius").fo(8.0);
                let d = d2(&p, &me.pos);
                if d > self.prm.drop_range || d2(&p, &zone.c) > zone.r - 5.0 || self.seek_ban.iter().any(|b| b.0 == id) {
                    continue;
                }
                // somebody else is already standing on it and it is far: not worth the walk into a fight
                let taken = !dr.get("holder").is_null() && dr.get("holder").f() as i64 != me.id;
                if taken && d > 60.0 {
                    continue;
                }
                // set off so as to arrive about when it lands
                let eta = d / me.walk_speed * 1.25;
                if eta + 4.0 < lands {
                    continue;
                }
                if lands <= 0.0 {
                    if !self.drop_seen.iter().any(|x| x.0 == id) {
                        self.drop_seen.push((id, t));
                    }
                    // on the ground for half a minute and we still are not standing on it: let it go
                    let since = self.drop_seen.iter().find(|x| x.0 == id).map_or(0.0, |x| t - x.1);
                    if since > 30.0 && d > r {
                        continue;
                    }
                }
                if beam_sees(p, &self.map) && d > 40.0 && hp_frac < 0.5 {
                    continue;
                }
                if best.map_or(true, |b| d < b.0) {
                    best = Some((d, (p[0], p[1]), id, r));
                }
            }
            if let Some((d, gp, id, r)) = best {
                // a weapon a few steps away still comes first
                let weapon_first = seek_goal.map_or(false, |g| hyp(g.0 .0 - me.pos[0], g.0 .1 - me.pos[1]) < 30.0 && d > 30.0);
                if !weapon_first {
                    seek_goal = Some((gp, id));
                    drop_goal = true;
                    drop_hold = d < (r - 1.5).max(2.0);
                }
            }
        }
        // ---- cubes: crates, drones and loose cubes within a short walk, when no mech is near
        let mut farm_stop = false;
        let chasing = seek_goal.map_or(false, |g| g.1 <= -100);
        if self.prm.farm_seek > 0.5 && mode == Mode::Hide && !drop_goal && !chasing && !laying_low && hp_frac > 0.5 && !infos.iter().any(|f| f.listed && f.d < self.prm.farm_clear) {
            let mut best: Option<(f64, (f64, f64), i64, f64, f64)> = None;
            let weapon_sc = seek_goal.map(|g| {
                let d = hyp(g.0 .0 - me.pos[0], g.0 .1 - me.pos[1]);
                75.0 / (d + 30.0)
            });
            let ground_ok = |p: &[f64; 3], map: &Map| (p[2] - map.walk_z(p[0], p[1]) as f64).abs() <= 3.0;
            let mut consider = |val: f64, p: [f64; 3], id: i64, stand: f64, best: &mut Option<(f64, (f64, f64), i64, f64, f64)>| {
                if self.seek_ban.iter().any(|b| b.0 == id) {
                    return;
                }
                let d = d2(&p, &me.pos);
                if d > self.prm.farm_range || d2(&p, &zone.c) > zone.r - 5.0 {
                    return;
                }
                if d > 18.0 && beam_sees(p, &self.map) {
                    return;
                }
                if far_watch && d > 60.0 {
                    return;
                }
                if d > 25.0 && crowded(&p) {
                    return;
                }
                if zroam {
                    if zone.time_left(t, &[p[0], p[1]]) < d / me.walk_speed * 1.5 + 10.0 {
                        return;
                    }
                } else if zone.nr > 1.0 && d2(&p, &zone.nc) > zone.nr + 60.0 {
                    return;
                }
                let inner = self.prm.ring > 0.0 && zone.nr > 150.0 && d2(&p, &zone.nc) < self.prm.ring * zone.nr;
                let sc = val * if inner { 0.5 } else { 1.0 } / (d + 30.0) + if id == self.seek_id { 0.05 } else { 0.0 };
                if best.map_or(true, |b| sc > b.0) {
                    *best = Some((sc, (p[0], p[1]), id, d, stand));
                }
            };
            let cube_reach = if healing && self.prm.heal_reach > 0.0 && hp_frac < 0.6 { self.prm.heal_reach * 0.67 } else { f64::INFINITY };
            for it in &self.cubes {
                if ground_ok(&it.pos, &self.map) && self.walkable_item(&me, &it.pos) && d2(&it.pos, &me.pos) < cube_reach {
                    consider(45.0, it.pos, it.id, 0.0, &mut best);
                }
            }
            if me.cubes < self.stat_cap + 5.0 && !healing && t >= self.prm.farm_start {
                for it in &self.crates {
                    if ground_ok(&it.pos, &self.map) && self.walkable_item(&me, &it.pos) {
                        consider(32.0, it.pos, it.id, 16.0, &mut best);
                    }
                }
                for e in &self.tracks {
                    if e.npc && t - e.seen_t < 8.0 && ground_ok(&e.pos, &self.map) && self.walkable_item(&me, &e.pos) {
                        consider(50.0, e.pos, e.id, 52.0, &mut best);
                    }
                }
            }
            if let Some((sc, gp, id, d, stand)) = best {
                if weapon_sc.map_or(true, |w| sc > w && !arming_run) {
                    seek_goal = Some((gp, id));
                    let c = [gp.0, gp.1, self.map.walk_z(gp.0, gp.1) as f64 + 1.5];
                    farm_stop = stand > 0.0 && d < stand && self.map.los(my_muzzle, c);
                }
            }
        }
        // ---- nothing to pick up: go and look somewhere new. Early on (peace) we do this even with mechs
        // in sight, and pick the spot that keeps us furthest from every mech we know of.
        let peaceful = t < self.prm.peace && !endgame;
        let foes: Vec<[f64; 2]> = self.tracks.iter().filter(|e| !e.npc && t - e.seen_t < 10.0).map(|e| [e.pos[0], e.pos[1]]).collect();
        let foe_near = infos.iter().any(|f| f.listed && f.d < self.prm.farm_clear + 30.0);
        let me3 = [me.pos[0], me.pos[1], me.pos[2]];
        let spacing = space > 0.0 && crowded(&me3);
        if self.prm.explore > 0.5 && seek_goal.is_none() && mode == Mode::Hide && !laying_low && (!healing || spacing) && (hp_frac > 0.6 || peaceful || spacing) && (!foe_near || (peaceful && self.prm.away > 0.5) || spacing) {
            let margin = (self.prm.edge_margin * zone.nr).clamp(3.0, 15.0);
            let safe = (zone.nr - margin - 12.0).max(0.0);
            let away = (self.prm.away > 0.5 || !beam_eyes.is_empty() || space > 0.0) && !foes.is_empty();
            let stale = match self.explore_goal {
                Some(g) => {
                    hyp(g.0 - me.pos[0], g.1 - me.pos[1]) < 12.0
                        || t - self.explore_t > 22.0
                        || d2(&[g.0, g.1], &zone.nc) > safe
                        || (away && t - self.explore_t > 2.0 && foes.iter().any(|f| hyp(g.0 - f[0], g.1 - f[1]) < if space > 0.0 { space * 0.75 } else { 70.0 }))
                }
                None => true,
            };
            // been there, or too old to still hold anything
            let here2 = [me.pos[0], me.pos[1]];
            self.death_sites.retain(|ds| t - ds.2 < 40.0 && hyp(ds.0 - here2[0], ds.1 - here2[1]) > if ds.3 == 1 { 14.0 } else { 30.0 });
            let mut scav_goal: Option<(f64, f64)> = None;
            if self.prm.scav > 0.5 {
                let mut bd = f64::INFINITY;
                for ds in &self.death_sites {
                    let g = (ds.0, ds.1);
                    let d = hyp(g.0 - me.pos[0], g.1 - me.pos[1]);
                    if d2(&[g.0, g.1], &zone.nc) > safe || d >= bd {
                        continue;
                    }
                    if !beam_eyes.is_empty() && beam_sees([g.0, g.1, self.map.walk_z(g.0, g.1) as f64], &self.map) {
                        continue;
                    }
                    let c0 = self.nav.cell_of(g.0, g.1);
                    if let Some(c) = self.nav.snap(c0, 3.4, 4) {
                        if self.nav.comp[c as usize] == self.nav.main_comp {
                            scav_goal = Some(self.nav.pos_of(c));
                            bd = d;
                        }
                    }
                }
            }
            if let Some(g) = scav_goal {
                if self.explore_goal.map_or(true, |e| hyp(e.0 - g.0, e.1 - g.1) > 6.0) {
                    self.explore_goal = Some(g);
                    self.explore_t = t;
                }
            } else if stale {
                self.explore_goal = None;
                let mut best: Option<(f64, (f64, f64))> = None;
                for _ in 0..(if away || self.prm.xlow > 0.0 { 16 } else { 10 }) {
                    let a = self.rng.range(-PI, PI);
                    let r = self.rng.range(80.0, 150.0);
                    let g = (me.pos[0] + a.cos() * r, me.pos[1] + a.sin() * r);
                    if d2(&[g.0, g.1], &zone.nc) > safe || self.map.wall_dist(g.0, g.1) < 4.0 {
                        continue;
                    }
                    if self.prm.ring > 0.0 && zone.nr > 150.0 && d2(&[g.0, g.1], &zone.nc) < self.prm.ring * zone.nr {
                        continue;
                    }
                    let c = self.nav.cell_of(g.0, g.1) as usize;
                    if c >= self.nav.comp.len() || self.nav.comp[c] != self.nav.main_comp {
                        continue;
                    }
                    if !beam_eyes.is_empty() && beam_sees([g.0, g.1, self.map.walk_z(g.0, g.1) as f64], &self.map) {
                        continue;
                    }
                    if !away && self.prm.xlow <= 0.0 {
                        best = Some((0.0, g));
                        break;
                    }
                    // furthest from every known mech, and not walking past one to get there
                    let dmin = foes.iter().map(|f| hyp(g.0 - f[0], g.1 - f[1])).fold(1e9, f64::min);
                    let mid = ((me.pos[0] + g.0) * 0.5, (me.pos[1] + g.1) * 0.5);
                    let dmid = foes.iter().map(|f| hyp(mid.0 - f[0], mid.1 - f[1])).fold(1e9, f64::min);
                    let mut sc = if away { dmin.min(dmid * 1.5).min(250.0) } else { 0.0 };
                    if self.prm.xlow > 0.0 {
                        sc -= self.prm.xlow * 100.0 * self.expo_at(c as u32);
                    }
                    if best.map_or(true, |b| sc > b.0) {
                        best = Some((sc, g));
                    }
                }
                if let Some((_, g)) = best {
                    self.explore_goal = Some(g);
                    self.explore_t = t;
                }
            }
            if let Some(g) = self.explore_goal {
                seek_goal = Some((g, -2));
            }
        }
        // ---- hurt, nobody on us, nothing to fetch: wait it out somewhere with short sight lines
        if healing && self.prm.heal_hide > 0.5 && seek_goal.is_none() && mode == Mode::Hide && !self.seek_ban.iter().any(|b| b.0 == -3) {
            if self.heal_spot.map_or(true, |h| t - h.2 > 8.0) {
                let need = (me.radius + 0.9) as f32;
                let mut best: Option<(f64, (f64, f64))> = None;
                if self.nav.flood((me.pos[0], me.pos[1]), need, 45.0, None).is_some() {
                    let reached = std::mem::take(&mut self.nav.reached);
                    let margin = (self.prm.edge_margin * zone.nr).clamp(3.0, 15.0);
                    for &(c, cost) in &reached {
                        let q = self.nav.pos_of(c);
                        if zone.nr > 1.0 && d2(&[q.0, q.1], &zone.nc) > zone.nr - margin - 4.0 && zone.time_left(t, &[q.0, q.1]) < 45.0 {
                            continue;
                        }
                        let mut sc = self.expo_at(c) * 70.0 + cost as f64 * 0.6;
                        if !beam_eyes.is_empty() && beam_sees([q.0, q.1, self.map.walk_z(q.0, q.1) as f64], &self.map) {
                            sc += 200.0;
                        }
                        if best.map_or(true, |b| sc < b.0) {
                            best = Some((sc, q));
                        }
                    }
                    self.nav.reached = reached;
                }
                self.heal_spot = best.map(|b| (b.1 .0, b.1 .1, t));
            }
            if let Some((hx, hy, _)) = self.heal_spot {
                if hyp(hx - me.pos[0], hy - me.pos[1]) > 2.5 {
                    seek_goal = Some(((hx, hy), -3));
                }
            }
        } else if !healing {
            self.heal_spot = None;
        }
        if let Some((_, id)) = seek_goal {
            if self.seek_id != id {
                self.seek_id = id;
                self.plan_t = -100.0;
            }
            mode = Mode::Seek;
        }
        // ---- nothing happening: stand in a tree crown
        if self.prm.tree > 0.5 && mode == Mode::Hide {
            if t - self.hide_t > 1.0 || self.hide_i.is_none() {
                let pick = self.pick_hide(&me, &zone, t);
                if pick != self.hide_i {
                    self.plan_t = -100.0;
                }
                self.hide_i = pick;
                self.hide_t = t;
            }
            if self.hide_i.is_some() {
                mode = Mode::Tree;
            }
        }

        // ---- roof camping: go up when calm; once up, stay up unless the circle says otherwise
        if self.prm.roof > 0.5 && mode != Mode::ZoneRun && mode != Mode::End && mode != Mode::Seek {
            let up = self.roof_i.map_or(false, |k| self.on_roof(&me, k));
            if mode == Mode::Hide || mode == Mode::Tree || up || (!me.grounded && self.roof_i.is_some() && self.mode == Mode::Roof) {
                if t - self.roof_t > 1.5 || self.roof_i.is_none() {
                    let pick = self.pick_roof(&me, &zone, t);
                    if pick != self.roof_i {
                        self.plan_t = -100.0;
                    }
                    self.roof_i = pick;
                    self.roof_t = t;
                }
                if self.roof_i.is_some() {
                    mode = Mode::Roof;
                }
            }
        }

        // ---- movement
        self.fly_goal = None;
        let p2 = [me.pos[0], me.pos[1]];
        let mut mv = [0.0, 0.0];
        let mut boost = false;
        let mut speed_frac: f64 = 1.0;
        match mode {
            Mode::End => {
                let goal = (zone.nc[0], zone.nc[1]);
                if t - self.plan_t > 0.45 || self.path.is_empty() || self.mode != Mode::End {
                    if !self.route_to(&me, goal) {
                        self.path.clear();
                    }
                    self.plan_t = t;
                    self.goal = None;
                }
                let (dir, rem) = self.follow(&me);
                mv = dir;
                if self.path.is_empty() && d2(&p2, &zone.nc) > 1.5 {
                    self.fly_goal = Some((zone.nc[0], zone.nc[1]));
                }
                let left = zone.time_left(t, &p2);
                boost = !me.in_zone || left < rem / me.walk_speed + 1.5;
                if rem < 3.0 {
                    speed_frac = (rem / 3.0).clamp(0.25, 1.0);
                }
            }
            Mode::ZoneRun => {
                if t - self.plan_t > 0.9 || self.path.is_empty() || self.mode != Mode::ZoneRun {
                    let need = (me.radius + 0.9) as f32;
                    let pc = std::mem::take(&mut self.path_cost);
                    let disk_r = (safe_r - 5.0).max(1.0);
                    let mut found = self.nav.path_to_disk((p2[0], p2[1]), (zone.nc[0], zone.nc[1]), disk_r, need, Some(&pc), 30_000);
                    if found.is_none() {
                        found = self.nav.path_to_disk((p2[0], p2[1]), (zone.nc[0], zone.nc[1]), disk_r, (me.radius + 0.3) as f32, Some(&pc), 30_000);
                    }
                    self.path_cost = pc;
                    match found {
                        Some(cells) => self.set_path_cells(&cells, &me),
                        None => self.path.clear(),
                    }
                    self.plan_t = t;
                    self.goal = None;
                }
                let (dir, rem) = self.follow(&me);
                mv = dir;
                if self.path.is_empty() {
                    self.fly_goal = Some((zone.nc[0], zone.nc[1]));
                }
                let left = zone.time_left(t, &p2);
                let plen = self.path_len(&me).max(rem);
                let need_t = plen / me.walk_speed;
                boost = if lazy { !me.in_zone || left < 1.5 || under_fire } else { !me.in_zone || left < need_t + 5.0 || under_fire };
            }
            Mode::Seek => {
                let gp = seek_goal.map(|g| g.0).unwrap_or((me.pos[0], me.pos[1]));
                let gd = hyp(gp.0 - me.pos[0], gp.1 - me.pos[1]);
                if self.mode != Mode::Seek || self.plan_t < -50.0 {
                    self.seek_best_d = gd;
                    self.seek_prog_t = t;
                }
                if gd < self.seek_best_d - 0.7 {
                    self.seek_best_d = gd;
                    self.seek_prog_t = t;
                }
                if t - self.plan_t > 0.6 || self.path.is_empty() || self.mode != Mode::Seek {
                    if !self.route_to(&me, gp) {
                        self.path.clear();
                        self.seek_ban.push((self.seek_id, t + 30.0));
                    }
                    self.plan_t = t;
                    self.goal = None;
                }
                let (dir, rem) = self.follow(&me);
                mv = dir;
                boost = rem > 8.0 && me.energy > 12.0 && self.seek_id != -3 && (self.seek_id != -2 || ((self.prm.away > 0.5 || spacing) && !seen_by.is_empty() && me.energy > 35.0));
                if rem < 2.5 {
                    speed_frac = 0.5;
                }
                if hyp(mv[0], mv[1]) < 0.1 && gd > 1.5 {
                    // the path ends short of it: walk the last metres straight
                    mv = [(gp.0 - me.pos[0]) / gd, (gp.1 - me.pos[1]) / gd];
                    speed_frac = 0.7;
                }
                if drop_hold {
                    // inside the ring: stay in, keep moving
                    boost = false;
                    self.seek_prog_t = t;
                    let (rx, ry) = (me.pos[0] - gp.0, me.pos[1] - gp.1);
                    let rr = hyp(rx, ry);
                    if rr < 1.0 {
                        mv = [1.0, 0.0];
                    } else {
                        // a slow circle about 2.5 m out
                        let (tx, ty) = (-ry / rr, rx / rr);
                        let pull = ((2.5 - rr) * 0.6).clamp(-2.0, 1.0);
                        let v = [tx + rx / rr * pull, ty + ry / rr * pull];
                        let l = hyp(v[0], v[1]).max(1e-6);
                        mv = [v[0] / l, v[1] / l];
                    }
                    speed_frac = if rr > 4.5 { 0.7 } else { 0.32 };
                } else if farm_stop {
                    // close enough to shoot it: keep circling so we are never a still target
                    boost = false;
                    self.seek_prog_t = self.seek_prog_t.max(t - 3.0);
                    let b = (gp.1 - me.pos[1]).atan2(gp.0 - me.pos[0]);
                    if t > self.strafe_until {
                        self.strafe_until = t + self.rng.range(0.6, 1.3);
                        self.strafe_dir = if self.rng.f() < 0.5 { 1.0 } else { -1.0 };
                    }
                    let here = (me.pos[0], me.pos[1]);
                    let need = (me.radius + 0.6) as f32;
                    let mut v = [-b.sin() * self.strafe_dir, b.cos() * self.strafe_dir];
                    if !nav::free_line(&self.map, here, (here.0 + v[0] * 5.0, here.1 + v[1] * 5.0), need) {
                        self.strafe_dir = -self.strafe_dir;
                        v = [-v[0], -v[1]];
                        if !nav::free_line(&self.map, here, (here.0 + v[0] * 5.0, here.1 + v[1] * 5.0), need) {
                            v = [0.0, 0.0];
                        }
                    }
                    mv = v;
                    speed_frac = 0.75;
                } else if t - self.seek_prog_t > 4.0 && self.seek_id != -2 {
                    // not getting any closer: forget this one for a while
                    self.seek_ban.push((self.seek_id, t + 40.0));
                    self.plan_t = -100.0;
                } else if t - self.seek_prog_t > 5.0 {
                    self.explore_goal = None;
                }
            }
            Mode::Roof => {
                let ri = self.roof_i.unwrap_or(0);
                let r = self.map.roofs[ri].clone();
                let best = r.best;
                if self.on_roof(&me, ri) {
                    let d = hyp(best.0 - me.pos[0], best.1 - me.pos[1]);
                    if d > 1.5 {
                        mv = [(best.0 - me.pos[0]) / d, (best.1 - me.pos[1]) / d];
                        if d < 4.0 {
                            speed_frac = 0.5;
                        }
                    } else {
                        self.held_cover_t = t;
                    }
                    self.path.clear();
                } else {
                    let acc = r.access.iter().copied().min_by(|a, b| hyp(a.0 - me.pos[0], a.1 - me.pos[1]).partial_cmp(&hyp(b.0 - me.pos[0], b.1 - me.pos[1])).unwrap_or(std::cmp::Ordering::Equal));
                    let (ax, ay) = acc.map(|a| (a.0, a.1)).unwrap_or(best);
                    let da = hyp(ax - me.pos[0], ay - me.pos[1]);
                    if da < 5.0 || !me.grounded {
                        // at the wall (or already in the air): go over the edge
                        self.fly_goal = Some(best);
                    } else {
                        if t - self.plan_t > 0.8 || self.path.is_empty() || self.mode != Mode::Roof {
                            if !self.route_to(&me, (ax, ay)) {
                                self.path.clear();
                                self.roof_bad.push(ri);
                                self.roof_i = None;
                            }
                            self.plan_t = t;
                            self.goal = None;
                        }
                        let (dir, rem) = self.follow(&me);
                        mv = dir;
                        boost = rem > 12.0 && me.energy > 60.0 && (exposed || t < self.prm.tree_rush);
                    }
                }
            }
            Mode::Tree => {
                let h = self.hides[self.hide_i.unwrap_or(0)];
                let dist = hyp(h.x - me.pos[0], h.y - me.pos[1]);
                if dist > 1.0 {
                    if t - self.plan_t > 0.8 || self.path.is_empty() || self.mode != Mode::Tree {
                        if !self.route_to(&me, (h.x, h.y)) {
                            self.path.clear();
                        }
                        self.plan_t = t;
                        self.goal = None;
                    }
                    let (dir, rem) = self.follow(&me);
                    mv = dir;
                    if self.path.is_empty() {
                        mv = [(h.x - me.pos[0]) / dist, (h.y - me.pos[1]) / dist];
                    }
                    boost = rem > 10.0 && me.energy > 18.0 && (exposed || t < self.prm.tree_rush || !me.in_zone);
                    if rem < 3.0 {
                        speed_frac = (rem / 3.0).clamp(0.35, 1.0);
                    }
                } else {
                    self.held_cover_t = t;
                }
            }
            Mode::Hide | Mode::Flee | Mode::Fight => {
                if !threatened && self.prm.relocate <= 0.0 {
                    // statue: nothing is happening, so do nothing
                    self.path.clear();
                    self.goal = None;
                    self.plan_t = t;
                    self.flee_planned = false;
                }
                let period = if threatened { 0.2 } else if self.prm.relocate <= 0.0 { 1e9 } else { 0.45 };
                let force = (threatened && !self.flee_planned) || self.mode == Mode::End || self.mode == Mode::ZoneRun;
                if (t - self.plan_t >= period || force) && (threatened || self.prm.relocate > 0.0) {
                    let loot_ok = mode == Mode::Hide && !under_fire && hostile_los.is_empty();
                    // while threatened always plan for cover (ignoring nobody), so we know if any is near
                    let (vm, fid) = if threatened { (2.5, -1) } else { (vis_mult, fight_id) };
                    self.plan(&me, &zone, t, vm, fid, loot_ok);
                    self.flee_planned = threatened;
                }
                let (dir, rem) = self.follow(&me);
                mv = dir;
                let zap_rush = self.prm.zap > 0.5 && me.slot_of(LIGHTNING).is_some();
                if mode == Mode::Fight && (zap_rush || !(self.goal_hidden && self.goal_cost < 14.0)) {
                    // fighting in the open: hold ground and strafe instead of walking the cover path
                    mv = [0.0, 0.0];
                }
                let rem = if mode == Mode::Fight && hyp(mv[0], mv[1]) < 0.1 { 0.0 } else { rem };
                // keeping away with nowhere better to go: never stand still while someone has us in view
                let dance = kite && mode == Mode::Flee && rem < 2.0 && exposed;
                let mode_mv = if dance { Mode::Fight } else { mode };
                match mode_mv {
                    Mode::Flee => {
                        boost = if kite { rem > 4.0 && (exposed || close_track) } else { rem > 4.0 };
                    }
                    Mode::Fight => {
                        let mut peeked = false;
                        if self.prm.peek > 0.5 && !dance {
                            let trk = self.tracks.iter().find(|e| e.id == fight_id).or_else(|| hostile_los.first().map(|&hk| &self.tracks[infos[hk].i])).cloned();
                            if let Some(e) = trk {
                                let others: Vec<[f64; 3]> = self.tracks.iter().filter(|q| !q.npc && q.id != e.id && t - q.seen_t < 3.0 && (t - q.hit_us_t < 4.0 || q.aim_us >= 3)).take(2).map(|q| q.eye()).collect();
                                if let Some((pm, pb, ps)) = self.peek_move(&me, &e, &others, t) {
                                    mv = pm;
                                    boost = pb;
                                    speed_frac = ps;
                                    peeked = true;
                                }
                            }
                        }
                        // keep moving sideways to the threat; never stand still in someone's sights
                        if !peeked && rem < 2.0 {
                            let tr = self.tracks.iter().find(|e| e.id == fight_id).or_else(|| hostile_los.first().map(|&hk| &self.tracks[infos[hk].i])).or_else(|| seen_by.first().map(|&hk| &self.tracks[infos[hk].i]));
                            if let Some(e) = tr {
                                let b = (e.pos[1] - me.pos[1]).atan2(e.pos[0] - me.pos[0]);
                                let dist = d2(&e.pos, &me.pos);
                                let need = (me.radius + 0.6) as f32;
                                let here = (me.pos[0], me.pos[1]);
                                let ok = |v: [f64; 2], map: &Map| -> bool {
                                    let ahead = (here.0 + v[0] * 5.0, here.1 + v[1] * 5.0);
                                    let out = (d2(&[ahead.0, ahead.1], &zone.nc) - safe_r).max(0.0);
                                    nav::free_line(map, here, ahead, need) && out <= me_out + 0.5
                                };
                                let cur = self.strafe_vec;
                                let zap_me = self.prm.zap > 0.5 && me.slot_of(LIGHTNING).is_some();
                                let zap_foe = self.prm.zap_fear > 0.5 && !zap_me && e.has[LIGHTNING] && me.slot_of(LIGHTNING).is_none();
                                if t > self.strafe_until || hyp(cur[0], cur[1]) < 0.5 || !ok(cur, &self.map) {
                                    let mut best: Option<(f64, [f64; 2])> = None;
                                    for k in 0..12 {
                                        let a = k as f64 * PI / 6.0;
                                        let v = [a.cos(), a.sin()];
                                        if !ok(v, &self.map) {
                                            continue;
                                        }
                                        let side = (a - b).sin().abs();
                                        let toward = (a - b).cos();
                                        let fd = if me.slot_of(LASER).is_some() && self.prm.laser_dist > 0.0 && self.prm.fight_dist > 0.0 { self.prm.laser_dist } else { self.prm.fight_dist };
                                        let fd = if self.prm.chase > 0.5 && e.hp < self.prm.chase_frac * e.max_hp && hp_frac > 0.5 { fd.min(45.0) } else { fd };
                                        let fd = if zap_me { self.prm.zap_dist } else if zap_foe { fd.max(self.prm.zap_fear_d) } else { fd };
                                        let range_pref = if kite {
                                            -0.5 * toward
                                        } else if zap_me && dist > 43.0 {
                                            // out of arc range: go straight in
                                            1.7 * toward
                                        } else if zap_foe && dist < self.prm.zap_fear_d - 10.0 {
                                            -1.3 * toward
                                        } else if fd > 0.0 {
                                            if dist > fd + 40.0 { 0.9 * toward } else if dist > fd + 12.0 { 0.45 * toward } else if dist < fd - 20.0 { -0.45 * toward } else { 0.0 }
                                        } else if dist > 125.0 {
                                            0.35 * toward
                                        } else if dist < 35.0 {
                                            -0.35 * toward
                                        } else {
                                            0.0
                                        };
                                        // do not simply reverse: that passes back through the old line
                                        let turn = -0.25 * (v[0] * cur[0] + v[1] * cur[1]).abs();
                                        let sc = side + range_pref + turn + self.rng.f() * 0.7;
                                        if best.map_or(true, |x| sc > x.0) {
                                            best = Some((sc, v));
                                        }
                                    }
                                    self.strafe_vec = best.map_or([0.0, 0.0], |x| x.1);
                                    self.strafe_until = t + self.rng.range(0.25, 0.7);
                                    self.strafe_speed = self.rng.range(self.prm.strafe_min.clamp(0.3, 1.0), 1.0);
                                    self.strafe_boost = self.rng.f() < self.prm.sb && me.energy > 40.0;
                                    if zap_me {
                                        // the arc runs on the same energy as the jets: sprint in, then save it
                                        self.strafe_boost = dist > 46.0 && me.energy > 45.0;
                                        if dist > 46.0 {
                                            self.strafe_speed = 1.0;
                                        }
                                    } else if zap_foe && dist < self.prm.zap_fear_d - 14.0 {
                                        self.strafe_boost = me.energy > 15.0;
                                        self.strafe_speed = 1.0;
                                    }
                                }
                                mv = self.strafe_vec;
                                speed_frac = self.strafe_speed;
                                boost = self.strafe_boost;
                            }
                        }
                    }
                    _ => {
                        // Hide: walk; creep when someone is close enough to hear steps
                        let near_enemy = infos.iter().any(|f| f.d < 75.0 && f.listed);
                        if (near_enemy || lazy) && !under_fire {
                            // below the footstep threshold: silent
                            speed_frac = (4.7 / me.walk_speed).min(1.0);
                        }
                        let left = zone.time_left(t, &p2);
                        if !me.in_zone || left < 3.0 {
                            boost = true;
                            speed_frac = 1.0;
                        }
                        if rem < 1.2 {
                            mv = [0.0, 0.0];
                        }
                    }
                }
            }
        }
        if mode == Mode::Hide || mode == Mode::Flee {
            let hidden = hostile_los.is_empty() && hyp(mv[0], mv[1]) < 0.1;
            if hidden {
                self.held_cover_t = t;
            }
        }
        self.mode = mode;

        // ---- stuck recovery
        let commanded = hyp(mv[0], mv[1]) * speed_frac > 0.3;
        self.hist.push((t, p2, commanded));
        if self.hist.len() > 14 {
            self.hist.remove(0);
        }
        if t < self.unstick_until {
            mv = self.unstick_dir;
            speed_frac = 1.0;
            act.jump = me.grounded;
            if self.stuck_count >= 3 {
                boost = true; // jet over whatever it is
            }
        } else if self.hist.len() >= 12 && self.hist.iter().all(|h| h.2) {
            let first = self.hist[0].1;
            if d2(&first, &p2) < 1.2 {
                self.stuck_count = if t - self.stuck_t < 8.0 { self.stuck_count + 1 } else { 1 };
                self.stuck_t = t;
                let a = mv[1].atan2(mv[0]) + if self.rng.f() < 0.5 { 1.0 } else { -1.0 } * self.rng.range(0.9, 1.9);
                self.unstick_dir = [a.cos(), a.sin()];
                self.unstick_until = t + 0.6;
                self.hist.clear();
                self.plan_t = -100.0;
                self.path.clear();
                self.goal = None;
            }
        }
        if !me.grounded && me.vel[2] < -1.0 && self.stuck_count >= 3 && t - self.stuck_t < 3.0 {
            boost = true;
        }
        // ---- no walking route (sealed courtyard, roof, far side of a wall): go over the top
        if self.fly_goal.is_some() && t < self.unstick_until + 0.0 {
            // unstick has priority for its short window
        } else if let Some(goal) = self.fly_goal {
            let (fm, fb, fj) = self.fly(&me, goal);
            mv = fm;
            boost = fb;
            act.jump = fj;
            speed_frac = 1.0;
        }
        // ---- make ourselves hard to lead while someone has a line on us
        let pressured = !hostile_los.is_empty() || (under_fire && t - self.last_dmg_t < 1.2);
        if pressured && hyp(mv[0], mv[1]) > 0.5 && self.fly_goal.is_none() && t >= self.unstick_until && mode != Mode::Fight {
            if t > self.jink_until {
                self.jink_dir = if self.rng.f() < 0.5 { 1.0 } else { -1.0 };
                self.jink_until = t + self.rng.range(0.25, 0.6);
            }
            let side = [-mv[1] * self.jink_dir, mv[0] * self.jink_dir];
            let cand = [mv[0] + 0.75 * side[0], mv[1] + 0.75 * side[1]];
            let l = hyp(cand[0], cand[1]).max(1e-6);
            let cand = [cand[0] / l, cand[1] / l];
            if nav::free_line(&self.map, (me.pos[0], me.pos[1]), (me.pos[0] + cand[0] * 5.0, me.pos[1] + cand[1] * 5.0), (me.radius + 0.5) as f32) {
                mv = cand;
            }
        }
        // ---- sidestep bullets and shells we can see coming
        let incoming = read_projectiles(o);
        if !incoming.is_empty() && self.fly_goal.is_none() && me.grounded {
            let intent = [mv[0] * speed_frac, mv[1] * speed_frac];
            if let Some((dir, b)) = evade(&self.map, me.pos, me.vel, me.radius, me.height, me.walk_speed, me.boost_speed, me.energy, intent, boost, &incoming, 7.0) {
                mv = dir;
                speed_frac = 1.0;
                boost = b;
            }
        }
        if boost && me.energy < 4.0 {
            boost = false;
        }
        act.mv = [mv[0] * speed_frac, mv[1] * speed_frac];
        act.boost = boost;

        self.st_had_tgt = tgt.is_some();
        // ---- aim and fire
        let mut want_aim: Option<[f64; 2]> = None;
        let mut fire = [false, false];
        let mut label = String::new();
        if let Some(k) = tgt {
            let inf = &infos[k];
            let e = self.tracks[inf.i].clone();
            let tp = e.center();
            let tol_r = e.radius() * 0.85;
            // gun
            let mut gun_aim = None;
            let tick_aim = self.prm.tick_aim > 0.5;
            // our own velocity command for the coming ticks (already decided above)
            let my_cmd = {
                let charging = me.slot_of(LASER).map_or(false, |l| me.w[l].charge > 0.0);
                let sp = if act.boost && me.grounded { me.boost_speed } else { me.walk_speed } * if charging { self.charge_move } else { 1.0 };
                [act.mv[0] * sp, act.mv[1] * sp]
            };
            if let Some(gs) = me.slot_of(GUN) {
                let w = &me.w[gs];
                // which tick of the coming two the gun can fire on
                let k_shot: usize = if w.cooldown > 0.05 + 1e-6 && w.cooldown <= 0.10 + 1e-6 { 2 } else { 1 };
                let (a, dist, _) = if tick_aim {
                    let lat = 0.05 * k_shot as f64;
                    self.lead_at(&me, me.muzzle_h, tp, e.vel, w.speed, lat, self.own_shift(&me, my_cmd, k_shot))
                } else {
                    self.lead(&me, me.muzzle_h, tp, e.vel, w.speed)
                };
                gun_aim = Some(a);
                let tol = (tol_r / dist.max(1.0)).atan();
                let err = self.aim_err(&me, a);
                let quiet = self.prm.gun_max > 0.0 && dist > self.prm.gun_max && hyp(e.vel[0], e.vel[1]) > 4.0 && !(self.prm.gun_far > 0.5 && inf.hostile);
                // the torso turns before the shot leaves: what matters is whether it gets there in time
                let slew = if tick_aim { me.turn_rate * 0.05 * k_shot as f64 * 0.85 } else { me.turn_rate * w.cooldown.max(0.0) * 0.8 };
                if !quiet && dist <= w.range + 3.0 && w.reload <= 0.0 && w.ammo > 0.0 && err - slew <= tol {
                    fire[gs] = true;
                }
                if w.ammo <= 0.0 && w.reload <= 0.0 {
                    act.reload = true;
                }
            }
            let mut aim = gun_aim;
            // laser: hold to charge, release on target
            if let Some(ls) = me.slot_of(LASER) {
                let w = &me.w[ls];
                // the beam leaves a tick after we let go: aim where the target will be then
                let ll = if tick_aim { 0.05 } else { self.prm.laser_lead };
                let tpl = [tp[0] + e.vel[0] * ll, tp[1] + e.vel[1] * ll, tp[2] + e.vel[2] * ll * 0.5];
                let (a, dist, _) = if tick_aim {
                    // the beam leaves on the first tick after we let go, from where we stand then
                    self.lead_at(&me, me.muzzle_h, tpl, e.vel, 0.0, 0.05, self.own_shift(&me, my_cmd, 1))
                } else {
                    self.lead(&me, me.muzzle_h, tpl, e.vel, 0.0)
                };
                if dist <= w.range - 3.0 && w.cooldown <= 0.25 {
                    let tol = (tol_r / dist.max(1.0)).atan();
                    let err = self.aim_err(&me, a);
                    let full = w.charge >= w.charge_t - 0.06;
                    let near_full = w.charge >= w.charge_t - 0.2;
                    let slew = if tick_aim { me.turn_rate * 0.05 * 0.85 } else { 0.0 };
                    let release = full && err - slew <= tol;
                    // with the real order the torso swings onto the beam's line on the very tick it fires,
                    // so the gun keeps its own lead until then
                    if (if tick_aim { release } else { near_full }) || gun_aim.is_none() {
                        aim = Some(a);
                        if tick_aim && release {
                            if let (Some(gs), Some(ga)) = (me.slot_of(GUN), gun_aim) {
                                // a bullet sent down the beam's line would miss a moving target
                                let off = hyp(wrap(ga[0] - a[0]) * a[1].cos(), ga[1] - a[1]);
                                if off > tol {
                                    fire[gs] = false;
                                }
                            }
                        }
                    }
                    if release {
                        fire[ls] = false; // let go: fires now
                        self.laser_hold = false;
                    } else {
                        fire[ls] = true;
                        self.laser_hold = true;
                    }
                }
            }
            // lightning: instant, short range, wide cone
            if let Some(zs) = me.slot_of(LIGHTNING) {
                let w = &me.w[zs];
                let (a, dist, _) = self.lead(&me, me.muzzle_h, tp, e.vel, 0.0);
                let yaw_err = wrap(a[0] - me.aim[0]).abs();
                if dist <= w.range - 1.5 && yaw_err <= (w.cone_deg * 0.5 - 3.0).max(4.0).to_radians() && me.energy >= w.energy + 1.0 && w.cooldown <= 0.1 {
                    fire[zs] = true;
                }
                if aim.is_none() {
                    aim = Some(a);
                }
            }
            // mortar: lob when the gun cannot do the job
            if let Some(ms) = me.slot_of(MORTAR) {
                let w = &me.w[ms];
                let slow = hyp(e.vel[0], e.vel[1]) < 4.0;
                let gun_busy = me.slot_of(GUN).map_or(true, |g| me.w[g].reload > 0.3);
                if w.cooldown <= 0.3 && (slow || gun_busy || endgame) {
                    if let Some(a) = self.mortar_solution(&me, &e, w) {
                        let d = d2(&e.pos, &me.pos);
                        if d > w.splash_radius + 5.0 {
                            let err = self.aim_err(&me, a);
                            if gun_busy || err < 0.25 {
                                aim = Some(a);
                            }
                            if err < 0.03 && w.cooldown <= 0.05 {
                                fire[ms] = true;
                            }
                        }
                    }
                }
            }
            want_aim = aim;
            if self.prm.debug > 0.0 {
                let _ = write!(label, "T{} d{:.0} ", e.id, inf.d);
            }
        } else {
            // No target: let a held laser go only if nothing else, keep it quiet otherwise.
            if let Some(ls) = me.slot_of(LASER) {
                if self.peek_charge && me.w[ls].cooldown <= 0.2 && me.w[ls].charge < me.w[ls].charge_t + me.w[ls].hold_s - 0.4 {
                    // about to step out of cover: start the charge now
                    fire[ls] = true;
                    self.laser_hold = true;
                } else if me.w[ls].charge > 0.0 && self.laser_hold {
                    // keep holding while a hostile is about to appear, else release into nothing is unavoidable
                    let soon = infos.iter().any(|f| f.hostile && f.listed && f.d < 120.0);
                    if soon && me.w[ls].charge < me.w[ls].charge_t + me.w[ls].hold_s - 0.3 {
                        fire[ls] = true;
                    } else {
                        self.laser_hold = false;
                    }
                }
            }
            // Pre-aim at a hostile that is behind cover; else farm; else look around.
            let mut pre: Option<[f64; 2]> = None;
            let mut best_d = 1e9;
            if let Some(k) = self.peek {
                if let Some(e) = self.tracks.iter().find(|e| e.id == k.tgt) {
                    let (a, _, _) = self.lead(&me, me.muzzle_h, e.center(), [0.0; 3], 0.0);
                    pre = Some(a);
                    best_d = -1.0;
                }
            }
            for inf in &infos {
                if inf.hostile && inf.d < best_d && inf.d < 220.0 {
                    let e = &self.tracks[inf.i];
                    let (a, _, _) = self.lead(&me, me.muzzle_h, e.center(), [0.0; 3], 0.0);
                    pre = Some(a);
                    best_d = inf.d;
                    // start charging if it is about to come into view
                    if let Some(ls) = me.slot_of(LASER) {
                        if inf.listed && me.w[ls].cooldown <= 0.2 && inf.d < 150.0 {
                            let c1 = [e.pos[0] + e.vel[0] * 0.6, e.pos[1] + e.vel[1] * 0.6, e.pos[2] + e.center_h()];
                            if self.map.los(my_muzzle, c1) && me.w[ls].charge < me.w[ls].charge_t + me.w[ls].hold_s - 0.4 {
                                fire[ls] = true;
                                self.laser_hold = true;
                            }
                        }
                    }
                }
            }
            if pre.is_none() && self.prm.scan_every > 0.0 && self.prm.scan > 0.5 && !(under_fire || t - self.last_dmg_t < 4.0) {
                // a full turn every few seconds, even in the middle of shooting a crate
                let near = infos.iter().any(|f| f.listed && f.d < 90.0);
                if t >= self.scan_until && (t < 0.3 || (t - self.scan_t > self.prm.scan_every && !near)) {
                    self.scan_until = t + 1.65;
                    self.scan_t = t;
                }
                if t < self.scan_until && (t < 2.0 || !near) {
                    pre = Some([me.aim[0] + 1.3, 0.0]);
                }
            }
            if pre.is_none() {
                let calm = if self.prm.farm_loose > 0.5 {
                    !under_fire && hp_frac > 0.5 && !healing && (mode == Mode::Hide || mode == Mode::Seek) && !infos.iter().any(|f| f.listed && f.d < 120.0)
                } else {
                    !under_fire && hp_frac > 0.6 && mode == Mode::Hide && seen_by.is_empty() && infos.iter().filter(|f| f.listed).count() == 0
                };
                pre = self.farm_fire(&me, t, my_muzzle, &mut fire, calm && self.prm.farm > 0.5 && t >= self.prm.farm_start);
            }
            if pre.is_none() && self.prm.scan_first > 0.5 && self.prm.scan > 0.5 && !(under_fire || t - self.last_dmg_t < 4.0) {
                // know what is around us: the first look always, later ones unless a mech is close
                let near = infos.iter().any(|f| f.listed && f.d < 110.0);
                if t >= self.scan_until && (t < 0.3 || (t - self.scan_t > 9.0 && !near)) {
                    self.scan_until = t + 1.7;
                    self.scan_t = t;
                }
                if t < self.scan_until && (t < 2.1 || !near) {
                    pre = Some([me.aim[0] + 1.3, 0.0]);
                }
            }
            if pre.is_none() {
                if under_fire || t - self.last_dmg_t < 4.0 {
                    pre = Some([self.last_dmg_bearing, 0.0]);
                } else if let Some(inf) = infos.iter().filter(|f| f.listed).min_by(|a, b| a.d.partial_cmp(&b.d).unwrap_or(std::cmp::Ordering::Equal)) {
                    let e = &self.tracks[inf.i];
                    let (a, _, _) = self.lead(&me, me.muzzle_h, e.center(), [0.0; 3], 0.0);
                    pre = Some(a);
                } else if t - self.sound_t < 2.0 {
                    pre = Some([self.sound_bearing, 0.0]);
                }
            }
            if pre.is_none() && self.prm.scan > 0.5 {
                if t >= self.scan_until && (t < 0.3 || t - self.scan_t > 9.0) {
                    self.scan_until = t + 1.7;
                    self.scan_t = t;
                }
                if t < self.scan_until {
                    pre = Some([me.aim[0] + 1.3, 0.0]);
                }
            }
            if pre.is_none() {
                let moving = hyp(act.mv[0], act.mv[1]) > 0.2;
                if moving {
                    let base = act.mv[1].atan2(act.mv[0]);
                    pre = Some([base + 0.9 * (t * 2.2).sin(), 0.0]);
                } else {
                    self.spin_yaw = wrap(self.spin_yaw + self.prm.spin * 0.1);
                    pre = Some([self.spin_yaw, 0.0]);
                }
            } else {
                self.spin_yaw = me.aim[0];
            }
            want_aim = pre;
            // top up the magazine while nobody is looking
            if let Some(gs) = me.slot_of(GUN) {
                let w = &me.w[gs];
                if w.reload <= 0.0 && w.ammo < w.mag * 0.75 && hostile_los.is_empty() && !fire[gs] {
                    act.reload = true;
                }
            }
        }
        if let Some(a) = want_aim {
            act.aim = [wrap(a[0]), a[1].clamp(me.min_pitch, me.max_pitch)];
        }
        act.fire = fire;
        self.last_aim = act.aim;
        self.last_mv = act.mv;

        // ---- pick up weapons within reach
        if t - self.take_fail_t > 0.5 {
            let mut best: Option<(f64, i64)> = None;
            for it in &self.pickups {
                if it.seen_t < t - 0.5 {
                    continue;
                }
                if d2(&it.pos, &me.pos) <= self.loot_radius - 0.2 && (it.pos[2] - me.pos[2]).abs() <= 3.8 {
                    let w = self.want_pickup(&me, it.kind);
                    if w > 0.0 && best.map_or(true, |b| w > b.0) {
                        best = Some((w, it.id));
                    }
                }
            }
            if let Some((_, id)) = best {
                act.take = Some(id);
                if me.w[0].present && me.w[1].present {
                    let other = if me.w[0].kind == GUN { 1 } else { 0 };
                    if !(me.w[other].kind == LASER && me.w[other].charge > 0.0) {
                        act.drop = Some(other);
                    } else {
                        act.take = None;
                    }
                }
            }
        }

        if self.prm.nomove > 0.5 {
            act.mv = [0.0, 0.0];
            act.boost = false;
            act.jump = false;
        }
        if self.prm.topup > 0.5 && tgt.is_none() && !act.fire.iter().any(|&f| f) && !act.reload {
            if let Some(gs) = me.slot_of(GUN) {
                let w = &me.w[gs];
                if w.reload <= 0.0 && w.ammo < w.mag * 0.67 && !infos.iter().any(|f| f.listed && f.d < 220.0) {
                    act.reload = true;
                }
            }
        }
        if self.prm.pacifist > 0.5 {
            act.fire = [false, false];
            act.reload = false;
        }
        if self.prm.debug > 0.0 {
            let _ = write!(label, "{:?} hp{:.0} a{} h{} s{}", mode, me.hp, alive, hostile_los.len(), seen_by.len());
            if mode == Mode::Seek {
                let gd = seek_goal.map(|g| hyp(g.0 .0 - me.pos[0], g.0 .1 - me.pos[1])).unwrap_or(-1.0);
                let _ = write!(label, " id{} d{:.0} stop{} path{} mv{:.1}", self.seek_id, gd, farm_stop as u8, self.path.len(), hyp(act.mv[0], act.mv[1]));
            }
            let _ = write!(label, " fw{} or{} gh{} gc{:.0} ey{}", far_watch as u8, outranged as u8, self.goal_hidden as u8, self.goal_cost.min(999.0), beam_eyes.len());
            {
                let kn = self.pickups.iter().filter(|it| self.want_pickup(&me, it.kind) > 0.0).map(|it| d2(&it.pos, &me.pos)).fold(f64::INFINITY, f64::min);
                let _ = write!(label, " kp{} kd{:.0}", self.pickups.len(), kn);
            }
            if let Some(k) = self.peek {
                let _ = write!(label, " pk{} tg{} dh{:.0} dp{:.0} mv{:.1} f{}{}", k.phase, k.tgt, hyp(k.h.0 - me.pos[0], k.h.1 - me.pos[1]), hyp(k.p.0 - me.pos[0], k.p.1 - me.pos[1]), hyp(act.mv[0], act.mv[1]), act.fire[0] as u8, act.fire[1] as u8);
            }
            act.label = label;
        }
        self.last_decide_us = t0.elapsed().as_micros();
        act
    }

    /// Straight-line travel that jumps and jets over whatever is in the way.
    fn fly(&self, me: &Me, goal: (f64, f64)) -> ([f64; 2], bool, bool) {
        let (dx, dy) = (goal.0 - me.pos[0], goal.1 - me.pos[1]);
        let d = hyp(dx, dy);
        if d < 1.0 {
            return ([0.0, 0.0], false, false);
        }
        let dir = [dx / d, dy / d];
        let mut need = f64::NEG_INFINITY;
        let reach = d.min(16.0) as usize;
        for s in 1..=reach.max(1) {
            for off in [-(me.radius + 0.5), 0.0, me.radius + 0.5] {
                let x = me.pos[0] + dir[0] * s as f64 - dir[1] * off;
                let y = me.pos[1] + dir[1] * s as f64 + dir[0] * off;
                if let Some(v) = self.map.vidx(x, y) {
                    need = need.max(self.map.top[v] as f64);
                    if self.map.ebase[v].is_finite() {
                        need = need.max(self.map.etop[v] as f64);
                    }
                }
            }
        }
        if me.grounded {
            if me.pos[2] + 1.0 >= need {
                return (dir, false, false);
            }
            let climb = need + 3.0 - me.pos[2];
            let want = (climb * 1.7 + 14.0).min(me.max_energy - 1.0);
            if me.energy < want {
                return ([0.0, 0.0], false, false); // wait for the jets to recharge
            }
            return ([dir[0] * 0.2, dir[1] * 0.2], true, true);
        }
        // airborne: rise until the feet clear it, then hold that height while drifting across
        let target = need + 3.0;
        let rise = me.pos[2] < target && me.vel[2] < 6.0;
        let hold = me.pos[2] < target + 1.5 && me.vel[2] < 0.5;
        let fwd = if me.pos[2] + 0.5 >= need { 1.0 } else { 0.25 };
        ([dir[0] * fwd, dir[1] * fwd], rise || hold, false)
    }

    /// Low-arc mortar solution against e. None when out of reach.
    fn mortar_solution(&self, me: &Me, e: &Track, w: &Wpn) -> Option<[f64; 2]> {
        let v = w.speed.max(1.0);
        let g = w.gravity.max(0.1);
        let oz = me.pos[2] + me.height + 0.5;
        let mut p = [e.pos[0], e.pos[1], e.pos[2] + 0.5];
        let mut out = None;
        for _ in 0..3 {
            let dx = p[0] - me.pos[0];
            let dy = p[1] - me.pos[1];
            let d = hyp(dx, dy).max(0.1);
            let h = p[2] - oz;
            let disc = v.powi(4) - g * (g * d * d + 2.0 * h * v * v);
            if disc < 0.0 {
                return None;
            }
            let th = ((v * v - disc.sqrt()) / (g * d)).atan();
            let tt = d / (v * th.cos()).max(1.0);
            out = Some([dy.atan2(dx), th]);
            p = [e.pos[0] + e.vel[0] * tt, e.pos[1] + e.vel[1] * tt, e.pos[2] + 0.5];
        }
        out.filter(|a| a[1] <= me.max_pitch && a[1] >= me.min_pitch)
    }

    /// Shoot drones that are shooting at us, and (when `calm`) crates and idle drones for cubes.
    /// Returns the aim it wants.
    fn farm_fire(&mut self, me: &Me, t: f64, muzzle: [f64; 3], fire: &mut [bool; 2], calm: bool) -> Option<[f64; 2]> {
        // (distance, centre, velocity, radius, hp)
        let mut best: Option<(f64, [f64; 3], [f64; 3], f64, f64)> = None;
        for e in &self.tracks {
            if !e.npc || e.seen_t < t {
                continue;
            }
            let c = e.center();
            let d = d2(&c, &me.pos);
            let angry = t - e.hit_us_t < 6.0 || d < self.npc_aggro + 4.0;
            let wanted = angry || (calm && me.cubes < self.stat_cap && d < 100.0);
            if wanted && d < 150.0 && self.map.los(muzzle, c) {
                let key = if angry { d - 1000.0 } else { d + 20.0 };
                if best.as_ref().map_or(true, |b| key < b.0) {
                    best = Some((key, c, e.vel, 1.5, e.hp));
                }
            }
        }
        if calm && me.cubes < self.stat_cap {
            for it in &self.crates {
                if it.seen_t < t {
                    continue;
                }
                let c = [it.pos[0], it.pos[1], it.pos[2] + 1.2];
                let d = d2(&c, &me.pos);
                if d < 110.0 && d > 6.0 && self.map.los(muzzle, c) && best.as_ref().map_or(true, |b| d < b.0) {
                    best = Some((d, c, [0.0; 3], 1.2, it.hp));
                }
            }
        }
        let (_, c, v, r, hp) = best?;
        let mut aim = None;
        if let Some(gs) = me.slot_of(GUN) {
            let w = &me.w[gs];
            let (a, dist, _) = self.lead(me, me.muzzle_h, c, v, w.speed);
            let tol = (r * 0.8 / dist.max(1.0)).atan();
            let err = self.aim_err(me, a);
            if dist <= w.range && w.reload <= 0.0 && w.ammo > 0.0 && err - me.turn_rate * w.cooldown.max(0.0) * 0.8 <= tol {
                fire[gs] = true;
            }
            aim = Some(a);
        }
        if let Some(ls) = me.slot_of(LASER) {
            let w = &me.w[ls];
            let (a, dist, _) = self.lead(me, me.muzzle_h, c, v, 0.0);
            // worth a beam only when it does real work
            if dist <= w.range - 3.0 && w.cooldown <= 0.25 && hp > 100.0 {
                let tol = (r * 0.8 / dist.max(1.0)).atan();
                let full = w.charge >= w.charge_t - 0.06;
                if full || aim.is_none() {
                    aim = Some(a);
                }
                if full && self.aim_err(me, a) <= tol {
                    fire[ls] = false;
                    self.laser_hold = false;
                } else {
                    fire[ls] = true;
                    self.laser_hold = true;
                }
            }
        }
        if let Some(zs) = me.slot_of(LIGHTNING) {
            let w = &me.w[zs];
            let (a, dist, _) = self.lead(me, me.muzzle_h, c, v, 0.0);
            if dist <= w.range - 1.5 && wrap(a[0] - me.aim[0]).abs() <= (w.cone_deg * 0.5 - 3.0).max(4.0).to_radians() && me.energy >= w.energy + 20.0 && w.cooldown <= 0.1 {
                fire[zs] = true;
            }
            if aim.is_none() {
                aim = Some(a);
            }
        }
        aim
    }
}
