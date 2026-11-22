//! Minimal JSON value + parser. No dependencies.

#[derive(Debug, Clone, PartialEq)]
pub enum J {
    Null,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<J>),
    Obj(Vec<(String, J)>),
}

static NULL: J = J::Null;

impl J {
    pub fn get(&self, k: &str) -> &J {
        if let J::Obj(v) = self {
            for (kk, vv) in v {
                if kk == k {
                    return vv;
                }
            }
        }
        &NULL
    }
    pub fn at(&self, i: usize) -> &J {
        if let J::Arr(v) = self {
            v.get(i).unwrap_or(&NULL)
        } else {
            &NULL
        }
    }
    pub fn num(&self) -> Option<f64> {
        match self {
            J::Num(x) if x.is_finite() => Some(*x),
            _ => None,
        }
    }
    /// number or 0
    pub fn f(&self) -> f64 {
        self.num().unwrap_or(0.0)
    }
    /// number or default
    pub fn fo(&self, d: f64) -> f64 {
        self.num().unwrap_or(d)
    }
    pub fn b(&self) -> bool {
        matches!(self, J::Bool(true))
    }
    pub fn s(&self) -> &str {
        if let J::Str(s) = self {
            s
        } else {
            ""
        }
    }
    pub fn arr(&self) -> &[J] {
        if let J::Arr(v) = self {
            v
        } else {
            &[]
        }
    }
    pub fn is_null(&self) -> bool {
        matches!(self, J::Null)
    }
    pub fn v2(&self) -> [f64; 2] {
        [self.at(0).f(), self.at(1).f()]
    }
    pub fn v3(&self) -> [f64; 3] {
        [self.at(0).f(), self.at(1).f(), self.at(2).f()]
    }
}

struct P<'a> {
    b: &'a [u8],
    i: usize,
}

impl<'a> P<'a> {
    fn ws(&mut self) {
        while self.i < self.b.len() && matches!(self.b[self.i], b' ' | b'\n' | b'\r' | b'\t') {
            self.i += 1;
        }
    }
    fn val(&mut self, depth: u32) -> Result<J, String> {
        if depth > 64 {
            return Err("too deep".into());
        }
        self.ws();
        if self.i >= self.b.len() {
            return Err("eof".into());
        }
        match self.b[self.i] {
            b'{' => {
                self.i += 1;
                let mut v = Vec::new();
                self.ws();
                if self.i < self.b.len() && self.b[self.i] == b'}' {
                    self.i += 1;
                    return Ok(J::Obj(v));
                }
                loop {
                    self.ws();
                    let k = self.string()?;
                    self.ws();
                    if self.i >= self.b.len() || self.b[self.i] != b':' {
                        return Err("expected :".into());
                    }
                    self.i += 1;
                    let x = self.val(depth + 1)?;
                    v.push((k, x));
                    self.ws();
                    if self.i >= self.b.len() {
                        return Err("eof in object".into());
                    }
                    match self.b[self.i] {
                        b',' => self.i += 1,
                        b'}' => {
                            self.i += 1;
                            return Ok(J::Obj(v));
                        }
                        _ => return Err("expected , or }".into()),
                    }
                }
            }
            b'[' => {
                self.i += 1;
                let mut v = Vec::new();
                self.ws();
                if self.i < self.b.len() && self.b[self.i] == b']' {
                    self.i += 1;
                    return Ok(J::Arr(v));
                }
                loop {
                    let x = self.val(depth + 1)?;
                    v.push(x);
                    self.ws();
                    if self.i >= self.b.len() {
                        return Err("eof in array".into());
                    }
                    match self.b[self.i] {
                        b',' => self.i += 1,
                        b']' => {
                            self.i += 1;
                            return Ok(J::Arr(v));
                        }
                        _ => return Err("expected , or ]".into()),
                    }
                }
            }
            b'"' => Ok(J::Str(self.string()?)),
            b't' => self.lit("true", J::Bool(true)),
            b'f' => self.lit("false", J::Bool(false)),
            b'n' => self.lit("null", J::Null),
            _ => {
                let s = self.i;
                while self.i < self.b.len() && matches!(self.b[self.i], b'-' | b'+' | b'.' | b'e' | b'E' | b'0'..=b'9') {
                    self.i += 1;
                }
                if s == self.i {
                    return Err(format!("unexpected byte {} at {}", self.b[s], s));
                }
                let t = std::str::from_utf8(&self.b[s..self.i]).map_err(|e| e.to_string())?;
                t.parse::<f64>().map(J::Num).map_err(|e| format!("{e}: {t}"))
            }
        }
    }
    fn lit(&mut self, w: &str, v: J) -> Result<J, String> {
        if self.b.len() >= self.i + w.len() && &self.b[self.i..self.i + w.len()] == w.as_bytes() {
            self.i += w.len();
            Ok(v)
        } else {
            Err("bad literal".into())
        }
    }
    fn string(&mut self) -> Result<String, String> {
        if self.i >= self.b.len() || self.b[self.i] != b'"' {
            return Err("expected string".into());
        }
        self.i += 1;
        let mut out: Vec<u8> = Vec::new();
        loop {
            if self.i >= self.b.len() {
                return Err("eof in string".into());
            }
            let c = self.b[self.i];
            self.i += 1;
            match c {
                b'"' => break,
                b'\\' => {
                    if self.i >= self.b.len() {
                        return Err("eof in escape".into());
                    }
                    let e = self.b[self.i];
                    self.i += 1;
                    match e {
                        b'n' => out.push(b'\n'),
                        b't' => out.push(b'\t'),
                        b'r' => out.push(b'\r'),
                        b'b' => out.push(8),
                        b'f' => out.push(12),
                        b'u' => {
                            if self.i + 4 > self.b.len() {
                                return Err("bad \\u".into());
                            }
                            let h = std::str::from_utf8(&self.b[self.i..self.i + 4]).map_err(|e| e.to_string())?;
                            let cp = u32::from_str_radix(h, 16).map_err(|e| e.to_string())?;
                            self.i += 4;
                            let ch = char::from_u32(cp).unwrap_or('?');
                            let mut buf = [0u8; 4];
                            out.extend_from_slice(ch.encode_utf8(&mut buf).as_bytes());
                        }
                        other => out.push(other),
                    }
                }
                _ => out.push(c),
            }
        }
        Ok(String::from_utf8_lossy(&out).into_owned())
    }
}

pub fn parse(s: &str) -> Result<J, String> {
    let mut p = P { b: s.as_bytes(), i: 0 };
    let v = p.val(0)?;
    p.ws();
    Ok(v)
}
