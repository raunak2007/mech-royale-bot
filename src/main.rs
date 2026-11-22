//! mechfight/1 bot: JSON lines on stdin/stdout. See protocol.md in the kit.

use mechbot::brain::{Brain, Params};
use mechbot::geom::Map;
use mechbot::json;
use std::io::{Read, Write};
use std::sync::mpsc;

fn load_params() -> Params {
    let mut p = Params::default();
    let mut tried = Vec::new();
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            tried.push(dir.join("params.txt"));
        }
    }
    tried.push(std::path::PathBuf::from("params.txt"));
    for path in tried {
        if let Ok(text) = std::fs::read_to_string(&path) {
            p.apply(&text);
            break;
        }
    }
    if let Ok(text) = std::env::var("MECH_PARAMS") {
        p.apply(&text.replace(',', "\n"));
    }
    p
}

fn main() {
    let (tx, rx) = mpsc::channel::<Option<Vec<String>>>();
    std::thread::spawn(move || {
        let mut stdin = std::io::stdin().lock();
        let mut buf = vec![0u8; 1 << 18];
        let mut pending: Vec<u8> = Vec::new();
        loop {
            match stdin.read(&mut buf) {
                Ok(0) | Err(_) => {
                    let _ = tx.send(None);
                    break;
                }
                Ok(n) => {
                    pending.extend_from_slice(&buf[..n]);
                    let mut lines = Vec::new();
                    let mut start = 0;
                    while let Some(off) = pending[start..].iter().position(|&b| b == b'\n') {
                        let line = String::from_utf8_lossy(&pending[start..start + off]).trim().to_string();
                        if !line.is_empty() {
                            lines.push(line);
                        }
                        start += off + 1;
                    }
                    pending.drain(..start);
                    if !lines.is_empty() && tx.send(Some(lines)).is_err() {
                        break;
                    }
                }
            }
        }
    });

    let stdout = std::io::stdout();
    let mut brain: Option<Brain> = None;
    let mut last = (String::from("[0,0]"), String::from("[0,0]"));
    let mut panics = 0u32;
    let mut eof = false;
    let send = |s: &str| -> bool {
        let mut out = stdout.lock();
        out.write_all(s.as_bytes()).is_ok() && out.write_all(b"\n").is_ok() && out.flush().is_ok()
    };
    while !eof {
        let mut lines = match rx.recv() {
            Ok(Some(v)) => v,
            _ => break,
        };
        loop {
            match rx.try_recv() {
                Ok(Some(v)) => lines.extend(v),
                Ok(None) => {
                    eof = true;
                    break;
                }
                Err(_) => break,
            }
        }
        let mut newest: Option<json::J> = None;
        let mut ended = false;
        for line in &lines {
            let msg = match json::parse(line) {
                Ok(m) => m,
                Err(_) => continue,
            };
            match msg.get("type").s() {
                "hello" => {
                    let t0 = std::time::Instant::now();
                    let dir = msg.get("map").get("dir").s().to_string();
                    let made = std::panic::catch_unwind(|| {
                        let map = Map::load(&dir);
                        Brain::new(&msg, std::sync::Arc::new(map), load_params())
                    });
                    match made {
                        Ok(b) => brain = Some(b),
                        Err(_) => eprintln!("init failed; running without a brain"),
                    }
                    eprintln!("ready in {:?}", t0.elapsed());
                    if !send("{\"type\":\"ready\",\"name\":\"scotty-edge\"}") {
                        return;
                    }
                }
                "obs" => newest = Some(msg),
                "end" => ended = true,
                _ => {}
            }
        }
        if let Some(o) = newest {
            let tick = o.get("tick").f();
            let line = match brain.as_mut() {
                Some(b) => match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| b.decide(&o))) {
                    Ok(a) => {
                        last = (format!("[{:.3},{:.3}]", a.mv[0], a.mv[1]), format!("[{:.4},{:.4}]", a.aim[0], a.aim[1]));
                        a.to_json(tick)
                    }
                    Err(_) => {
                        panics += 1;
                        if panics <= 5 {
                            eprintln!("decide panicked at tick {tick} ({panics} so far)");
                        }
                        format!("{{\"tick\":{},\"move\":{},\"aim\":{}}}", tick as i64, last.0, last.1)
                    }
                },
                None => format!("{{\"tick\":{}}}", tick as i64),
            };
            if !send(&line) {
                return;
            }
        }
        if ended {
            break;
        }
    }
    let _ = std::io::stdout().flush();
    std::process::exit(0);
}
