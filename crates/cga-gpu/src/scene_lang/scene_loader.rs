use super::*;

pub struct SceneLoader {
    toks: Vec<CgsToken>,
    pos: usize,
    asset_root: String,
    scene: Scene,
    camera: Option<PerspectiveCamera>,
    modules: HashMap<String, Vec<CgsToken>>,
    params: HashMap<String, Vec<CgsToken>>,
    param_order: Vec<String>,
    collect: Vec<CollectedGeom>,
    collecting: bool,

    named: HashMap<String, Vec<Inst>>,

    pending: Vec<(String, [f64; 16])>,
}
impl SceneLoader {
    fn peek(&self) -> CgsToken {
        if self.pos >= self.toks.len() {
            return CgsToken {
                kind: TokenKind::Eof,
                text: String::new(),
                num: 0.0,
                line: 1,
            };
        }
        self.toks[self.pos].clone()
    }

    fn peek1(&self) -> CgsToken {
        if self.pos + 1 >= self.toks.len() {
            return CgsToken {
                kind: TokenKind::Eof,
                text: String::new(),
                num: 0.0,
                line: 1,
            };
        }
        self.toks[self.pos + 1].clone()
    }

    fn take(&mut self) -> CgsToken {
        let t = self.peek();
        self.pos += 1;
        t
    }

    fn expect(&mut self, sym: TokenKind) -> Result<(), String> {
        let t = self.take();
        if t.kind != sym {
            return Err(format!(
                "CGS line {}: expected {}, got {}",
                t.line, sym, t.kind
            ));
        }
        Ok(())
    }

    fn run_tokens(
        &mut self,
        toks: Vec<CgsToken>,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
    ) -> Result<(), String> {
        let saved = std::mem::replace(&mut self.toks, toks);
        let saved_pos = self.pos;
        self.pos = 0;
        while self.pos < self.toks.len() {
            self.statement(ctx, mat, scope)?;
        }
        self.toks = saved;
        self.pos = saved_pos;
        Ok(())
    }

    fn expr(
        &mut self,
        scope: &HashMap<String, CgsValue>,
        min_prec: i32,
    ) -> Result<CgsValue, String> {
        let mut lhs = self.unary(scope)?;
        loop {
            let t = self.peek();
            if t.kind != TokenKind::Op {
                return Ok(lhs);
            }
            let op = match cgs_binop_from_text(&t.text) {
                Some(op) => op,
                None => return Ok(lhs),
            };
            let prec = cgs_precedence(op);
            if prec < min_prec {
                return Ok(lhs);
            }
            self.take();
            let rhs = self.expr(scope, prec + 1)?;
            lhs = cgs_binop(op, lhs, rhs)?;
        }
    }

    fn unary(&mut self, scope: &HashMap<String, CgsValue>) -> Result<CgsValue, String> {
        let t = self.peek();
        if t.kind == TokenKind::Op && t.text == "-" {
            self.take();
            let v = self.expr(scope, 7)?;
            return cgs_neg(&v, t.line);
        }
        if t.kind == TokenKind::Op && t.text == "!" {
            self.take();
            let v = self.expr(scope, 7)?;
            return Ok(CgsValue::Bool(!cgs_truthy(&v)));
        }
        self.primary(scope)
    }

    fn primary(&mut self, scope: &HashMap<String, CgsValue>) -> Result<CgsValue, String> {
        let v = self.atom(scope)?;
        self.suffix_chain(v, scope)
    }

    fn atom(&mut self, scope: &HashMap<String, CgsValue>) -> Result<CgsValue, String> {
        let t = self.take();
        if t.kind == TokenKind::Number {
            return Ok(CgsValue::Num(t.num));
        }
        if t.kind == TokenKind::Lparen {
            let v = self.expr(scope, 1)?;
            self.expect(TokenKind::Rparen)?;
            return Ok(v);
        }
        if t.kind == TokenKind::Lbracket {
            return self.list_literal(scope, t.line);
        }
        if t.kind == TokenKind::Str {
            return Ok(CgsValue::Str(t.text));
        }
        if t.kind == TokenKind::Ident {
            if self.peek().kind == TokenKind::Assign {
                return Err(format!(
                    "CGS line {}: assignment is a statement and cannot be used in an expression",
                    t.line
                ));
            }
            if self.peek().kind == TokenKind::Lbracket {
                return Err(format!(
                    "CGS line {}: indexing is not supported — use comp(vector, index)",
                    t.line
                ));
            }
            if self.peek().kind == TokenKind::Lparen {
                if is_statement_only_fn(&t.text) {
                    return Err(format!(
                        "CGS line {}: {} is a statement and cannot be used in an expression",
                        t.line, t.text
                    ));
                }
                self.take();
                let (pos, kw) = self.paren_args(scope)?;
                return self.call_dispatch(&t.text, t.line, pos, kw);
            }
            if t.text == "true" {
                return Ok(CgsValue::Bool(true));
            }
            if t.text == "false" {
                return Ok(CgsValue::Bool(false));
            }
            if let Some(v) = scope.get(&t.text) {
                return Ok(v.clone());
            }
            return Err(format!(
                "CGS line {}: undefined variable {}",
                t.line, t.text
            ));
        }
        Err(format!(
            "CGS line {}: bad expression start {}",
            t.line, t.kind
        ))
    }

    fn call_dispatch(
        &mut self,
        n: &str,
        line: i32,
        pos: Vec<CgsValue>,
        kw: HashMap<String, CgsValue>,
    ) -> Result<CgsValue, String> {
        if is_statement_only_fn(n) {
            return Err(format!(
                "CGS line {line}: {n} is a statement and cannot be used in an expression"
            ));
        }
        let is_geom = GEOM_EXPR_NAMES.contains(&n)
            || QUERY_FNS.contains(&n)
            || matches!(
                n,
                "at" | "rot" | "scaled" | "difference" | "intersection" | "union"
            );
        if is_geom {
            if let Some(v) = self.geom_expr_call(n, pos.clone(), kw.clone(), line)? {
                return Ok(v);
            }
        }
        if !kw.is_empty() {
            return Err(format!(
                "CGS line {line}: function {n} takes no named arguments"
            ));
        }
        cgs_call_fn(n, &pos, line)
    }

    fn suffix_chain(
        &mut self,
        mut v: CgsValue,
        scope: &HashMap<String, CgsValue>,
    ) -> Result<CgsValue, String> {
        while self.peek().kind == TokenKind::Dot {
            self.take();
            let m = self.take();
            if m.kind != TokenKind::Ident {
                return Err(format!(
                    "CGS line {}: expected {}, got {}",
                    m.line,
                    TokenKind::Ident,
                    m.kind
                ));
            }
            self.expect(TokenKind::Lparen)?;
            let (mut pos, kw) = self.paren_args(scope)?;
            pos.insert(0, v);
            v = self.call_dispatch(&m.text, m.line, pos, kw)?;
        }
        Ok(v)
    }

    fn list_literal(
        &mut self,
        scope: &HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<CgsValue, String> {
        let first = self.expr(scope, 1)?;
        if self.peek().kind == TokenKind::Op && self.peek().text == ":" {
            self.take();
            let second = self.expr(scope, 1)?;
            let mut step = 1.0;
            let mut start_v = cgs_num(&first, line, "range")?;
            let mut stop_v = cgs_num(&second, line, "range")?;
            if self.peek().kind == TokenKind::Op && self.peek().text == ":" {
                self.take();
                let third = self.expr(scope, 1)?;
                start_v = cgs_num(&first, line, "range")?;
                step = cgs_num(&second, line, "range")?;
                stop_v = cgs_num(&third, line, "range")?;
            }
            self.expect(TokenKind::Rbracket)?;
            if step == 0.0 {
                return Err(format!("CGS line {line}: range step must not be 0"));
            }
            let mut out: Vec<CgsValue> = Vec::new();
            let mut v = start_v;
            if step > 0.0 {
                while v <= stop_v + 1e-12 {
                    out.push(CgsValue::Num(v));
                    v += step;
                }
            } else {
                while v >= stop_v - 1e-12 {
                    out.push(CgsValue::Num(v));
                    v += step;
                }
            }
            return Ok(CgsValue::List(out));
        }
        let mut items = vec![first];
        while self.peek().kind == TokenKind::Comma {
            self.take();
            items.push(self.expr(scope, 1)?);
        }
        self.expect(TokenKind::Rbracket)?;

        if items.len() == 3 {
            if let (CgsValue::Num(x), CgsValue::Num(y), CgsValue::Num(z)) =
                (&items[0], &items[1], &items[2])
            {
                return Ok(CgsValue::Vec3(CgsVec3 {
                    x: *x,
                    y: *y,
                    z: *z,
                }));
            }
        }
        Ok(CgsValue::List(items))
    }

    fn geom_expr_call(
        &mut self,
        name: &str,
        pos: Vec<CgsValue>,
        kw: HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<Option<CgsValue>, String> {
        match name {
            "at" | "rot" | "scaled" => {
                if !kw.is_empty() {
                    return Err(format!(
                        "CGS line {line}: {name} takes positional arguments only"
                    ));
                }
                let base = geom_val(pos.first().unwrap_or(&CgsValue::Num(0.0)), line, name)?;

                let m = match name {
                    "at" => {
                        if pos.len() != 2 {
                            return Err(format!("CGS line {line}: at needs (geometry, offset)"));
                        }
                        let off = cgs_vec3(&pos[1], line, "at.off")?;
                        mat4_mul(translate4(off), base.m4)
                    }
                    "rot" => {
                        if pos.len() != 3 {
                            return Err(format!(
                                "CGS line {line}: rot needs (geometry, axis, angle)"
                            ));
                        }
                        let ax = cgs_vec3(&pos[1], line, "rot.axis")?;
                        let ang = cgs_num(&pos[2], line, "rot.angle")?;
                        mat4_mul(Multivector::rotor(ax, ang).to_matrix(), base.m4)
                    }
                    _ => {
                        if pos.len() != 2 {
                            return Err(format!(
                                "CGS line {line}: scaled needs (geometry, factor)"
                            ));
                        }
                        let s4 = match &pos[1] {
                            CgsValue::Vec3(v3) => scale4([v3.x, v3.y, v3.z]),
                            CgsValue::List(_) => scale4(cgs_vec3(&pos[1], line, "scaled.s")?),
                            _ => {
                                let x = cgs_num(&pos[1], line, "scaled.s")?;
                                scale4([x, x, x])
                            }
                        };
                        mat4_mul(s4, base.m4)
                    }
                };
                Ok(Some(CgsValue::Geom(GeomVal {
                    geo: base.geo,
                    m4: m,
                })))
            }
            "difference" | "intersection" | "union" => {
                if !kw.is_empty() {
                    return Err(format!("CGS line {line}: {name} takes no named arguments"));
                }
                if pos.len() < 2 {
                    return Err(format!(
                        "CGS line {line}: {name} needs >= 2 geometry arguments"
                    ));
                }
                let op = match name {
                    "difference" => CsgOp::Difference,
                    "intersection" => CsgOp::Intersection,
                    _ => CsgOp::Union,
                };
                let mut kids: Vec<Geometry> = Vec::new();
                for v in &pos {
                    let g = geom_val(v, line, name)?;
                    if matches!(g.geo, Geometry::CircleGeometry(_)) {
                        return Err(format!(
                            "CGS line {line}: {name} children must be solids (circle is not)"
                        ));
                    }
                    if let Geometry::BezierPatchGeometry(b) = &g.geo {
                        if !b.is_solid() {
                            return Err(format!(
                                "CGS line {line}: {name} children must be solids (bezier surface with thickness=0 is not)"
                            ));
                        }
                    }
                    let (cm, cl) = decompose_rigid(g.m4);
                    kids.push(Geometry::AffineGeometry(AffineGeometry::with_motor(
                        g.geo, cm, cl,
                    )));
                }
                Ok(Some(CgsValue::Geom(GeomVal {
                    geo: Geometry::CsgGeometry(CsgGeometry::new(op, kids)),
                    m4: mat4_identity(),
                })))
            }
            _ if QUERY_FNS.contains(&name) => Ok(Some(self.eval_query(name, pos, kw, line)?)),
            _ if GEOM_EXPR_NAMES.contains(&name) => {
                let args = self.resolve(name, pos, kw, line)?;
                let geo = self.build_geometry(name, &args, line)?;
                Ok(Some(CgsValue::Geom(GeomVal {
                    geo,
                    m4: mat4_identity(),
                })))
            }
            _ => Ok(None),
        }
    }

    fn face_target<'a>(
        &'a self,
        v: &'a CgsValue,
        line: i32,
        what: &str,
    ) -> Result<([f64; 16], &'a Geometry), String> {
        match v {
            CgsValue::Str(s) => {
                let insts = match self.named.get(s) {
                    Some(list) if !list.is_empty() => list,
                    _ => {
                        return Err(format!("CGS line {line}: unknown reference \"{s}\""));
                    }
                };
                Ok((insts[0].world, &insts[0].geo))
            }
            CgsValue::Geom(g) => Ok((g.m4, &g.geo)),
            _ => Err(format!(
                "CGS line {line}: {what} needs a reference name or geometry, got {v}"
            )),
        }
    }

    fn drill_end_coord(
        &self,
        e: &DrillEnd,
        f: [f64; 16],
        ax: usize,
        line: i32,
    ) -> Result<f64, String> {
        match e {
            DrillEnd::Num(n) => Ok(*n),
            DrillEnd::Face { name, axis, sign } => {
                let insts = match self.named.get(name) {
                    Some(list) if !list.is_empty() => list,
                    _ => {
                        return Err(format!("CGS line {line}: unknown reference \"{name}\""));
                    }
                };
                let inst = &insts[0];
                let (p_local, _) = face_local(&inst.geo, *axis, *sign).ok_or_else(|| {
                    format!("CGS line {line}: drill face reference has no finite bounds")
                })?;
                let p_world = transform_point(inst.world, p_local);
                Ok(transform_point(mat4_inv(f), p_world)[ax])
            }
        }
    }

    fn query_target(
        &self,
        v: &CgsValue,
        line: i32,
        what: &str,
    ) -> Result<([f64; 16], Option<[[f64; 3]; 2]>), String> {
        match v {
            CgsValue::Str(s) => {
                let insts = match self.named.get(s) {
                    Some(list) if !list.is_empty() => list,
                    _ => {
                        return Err(format!("CGS line {line}: unknown reference \"{s}\""));
                    }
                };
                let mut acc: Option<[[f64; 3]; 2]> = None;
                for i in insts {
                    if let Some(b) = self
                        .local_bounds(&i.geo)
                        .map(|b| transform_bbox(b, i.world))
                    {
                        acc = Some(match acc {
                            None => b,
                            Some(a) => [
                                [
                                    a[0][0].min(b[0][0]),
                                    a[0][1].min(b[0][1]),
                                    a[0][2].min(b[0][2]),
                                ],
                                [
                                    a[1][0].max(b[1][0]),
                                    a[1][1].max(b[1][1]),
                                    a[1][2].max(b[1][2]),
                                ],
                            ],
                        });
                    }
                }
                Ok((insts[0].world, acc))
            }
            CgsValue::Geom(g) => Ok((
                g.m4,
                self.local_bounds(&g.geo).map(|b| transform_bbox(b, g.m4)),
            )),
            _ => Err(format!(
                "CGS line {line}: {what} needs a reference name or geometry, got {v}"
            )),
        }
    }

    fn local_bounds(&self, geo: &Geometry) -> Option<[[f64; 3]; 2]> {
        crate::geometry_ops::geom_bounds(&geo.identity_params())
    }

    fn eval_query(
        &self,
        name: &str,
        pos: Vec<CgsValue>,
        kw: HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<CgsValue, String> {
        if !kw.is_empty() {
            return Err(format!("CGS line {line}: {name} takes no named arguments"));
        }
        let mut pos = pos;

        if matches!(name, "face" | "fnrm") && pos.len() == 1 {
            if let CgsValue::Str(s) = &pos[0] {
                if let Some((nm, key)) = s.split_once(':') {
                    pos = vec![
                        CgsValue::Str(nm.to_string()),
                        CgsValue::Str(key.to_string()),
                    ];
                }
            }
        }
        let need = if matches!(name, "dist" | "face" | "fnrm") {
            2
        } else {
            1
        };
        if pos.len() != need {
            return Err(format!("CGS line {line}: {name} needs {need} argument(s)"));
        }
        if name == "instances" {
            let s = match &pos[0] {
                CgsValue::Str(s) => s,
                v => {
                    return Err(format!(
                        "CGS line {line}: instances needs a reference name, got {v}"
                    ));
                }
            };
            let insts = match self.named.get(s) {
                Some(list) if !list.is_empty() => list,
                _ => {
                    return Err(format!("CGS line {line}: unknown reference \"{s}\""));
                }
            };
            return Ok(CgsValue::List(
                insts
                    .iter()
                    .map(|i| {
                        CgsValue::Geom(GeomVal {
                            geo: i.geo.clone(),
                            m4: i.world,
                        })
                    })
                    .collect(),
            ));
        }
        if matches!(name, "face" | "fnrm") {
            let key = match &pos[1] {
                CgsValue::Str(s) => s.clone(),
                v => {
                    return Err(format!(
                        "CGS line {line}: {name} key must be a string like \"+x\", got {v}"
                    ));
                }
            };
            let (axis, sign) = parse_face_key(&key, line, name)?;
            let (m, geo) = self.face_target(&pos[0], line, name)?;
            let (p, n) = face_local(geo, axis, sign).ok_or_else(|| {
                format!("CGS line {line}: {name}: reference has no finite bounds")
            })?;
            let w = if name == "face" {
                transform_point(m, p)
            } else {
                transform_normal(m, n)
            };
            return Ok(CgsValue::Vec3(CgsVec3 {
                x: w[0],
                y: w[1],
                z: w[2],
            }));
        }
        if name == "dist" {
            let a = self.query_point(&pos[0], line, name)?;
            let b = self.query_point(&pos[1], line, name)?;
            let d = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
            return Ok(CgsValue::Num(
                (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt(),
            ));
        }
        if matches!(name, "xdir" | "ydir" | "zdir") {
            let (m, _) = self.query_target(&pos[0], line, name)?;

            let col = match name {
                "xdir" => [m[0], m[4], m[8]],
                "ydir" => [m[1], m[5], m[9]],
                _ => [m[2], m[6], m[10]],
            };
            let n = (col[0] * col[0] + col[1] * col[1] + col[2] * col[2]).sqrt();
            if n < 1e-12 {
                return Err(format!("CGS line {line}: {name} is degenerate"));
            }
            return Ok(CgsValue::Vec3(CgsVec3 {
                x: col[0] / n,
                y: col[1] / n,
                z: col[2] / n,
            }));
        }
        let (_, b) = self.query_target(&pos[0], line, name)?;
        let b = match b {
            Some(b) => b,
            None => {
                return Err(format!(
                    "CGS line {line}: {name}: reference has no finite bounds"
                ));
            }
        };
        let v = match name {
            "center" => [
                (b[0][0] + b[1][0]) / 2.0,
                (b[0][1] + b[1][1]) / 2.0,
                (b[0][2] + b[1][2]) / 2.0,
            ],
            "lo" => b[0],
            "hi" => b[1],
            _ => [b[1][0] - b[0][0], b[1][1] - b[0][1], b[1][2] - b[0][2]],
        };
        Ok(CgsValue::Vec3(CgsVec3 {
            x: v[0],
            y: v[1],
            z: v[2],
        }))
    }

    fn query_point(&self, v: &CgsValue, line: i32, what: &str) -> Result<[f64; 3], String> {
        if let CgsValue::Vec3(v3) = v {
            return Ok([v3.x, v3.y, v3.z]);
        }
        let (_, b) = self.query_target(v, line, what)?;
        let b = match b {
            Some(b) => b,
            None => {
                return Err(format!(
                    "CGS line {line}: {what}: reference has no finite bounds"
                ));
            }
        };
        Ok([
            (b[0][0] + b[1][0]) / 2.0,
            (b[0][1] + b[1][1]) / 2.0,
            (b[0][2] + b[1][2]) / 2.0,
        ])
    }

    fn register(&mut self, geo: &Geometry, world: [f64; 16], emit: [f64; 16]) {
        if self.pending.is_empty() {
            return;
        }
        for i in 0..self.pending.len() {
            let (name, entry) = self.pending[i].clone();
            let rel = mat4_mul(mat4_inv(entry), emit);
            self.named.entry(name).or_default().push(Inst {
                geo: geo.clone(),
                world,
                rel,
            });
        }
    }

    fn show_stmt(
        &mut self,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        self.take();
        let (pos, kw) = self.call_args(scope)?;
        if !kw.is_empty() || pos.len() != 1 {
            return Err(format!("CGS line {line}: show takes one geometry argument"));
        }
        let g = geom_val(&pos[0], line, "show")?;
        self.expect(TokenKind::Semi)?;
        self.add_geometry(g.geo, mat4_mul(ctx, g.m4), mat)
    }

    fn tag_stmt(
        &mut self,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        self.take();
        self.expect(TokenKind::Lparen)?;
        let nt = self.take();
        if nt.kind != TokenKind::Str {
            return Err(format!("CGS line {}: tag needs a name string", nt.line));
        }
        self.expect(TokenKind::Rparen)?;
        self.pending.push((nt.text, ctx));
        let r = self.body(ctx, mat, scope, line);
        self.pending.pop();
        r
    }

    fn drill_stmt(
        &mut self,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        self.take();
        let (pos, kw) = self.call_args(scope)?;
        if !pos.is_empty() {
            return Err(format!(
                "CGS line {line}: drill takes named arguments: r, through, axis[, from, to]"
            ));
        }
        let r = match kw.get("r") {
            Some(v) => cgs_num(v, line, "drill.r")?,
            None => return Err(format!("CGS line {line}: drill needs r=")),
        };
        if !(r > 0.0) {
            return Err(format!("CGS line {line}: drill.r must be > 0"));
        }

        let from_ref = drill_end_ref(kw.get("from"), line, "from")?;
        let to_ref = drill_end_ref(kw.get("to"), line, "to")?;
        let face_axes: Vec<(usize, f64)> = [&from_ref, &to_ref]
            .iter()
            .filter_map(|e| match e {
                Some(DrillEnd::Face { axis, sign, .. }) => Some((*axis, *sign)),
                _ => None,
            })
            .collect();

        let ax = match kw.get("axis") {
            Some(v) => {
                let a = cgs_num(v, line, "drill.axis")?;
                if a != 0.0 && a != 1.0 && a != 2.0 {
                    return Err(format!(
                        "CGS line {line}: drill.axis must be 0, 1 or 2 (X/Y/Z)"
                    ));
                }
                let ax = a as usize;
                if let Some(&(ka, ks)) = face_axes.first() {
                    if ka != ax {
                        return Err(format!(
                            "CGS line {line}: drill face key \"{}\" does not match axis={ax}",
                            face_key_str(ka, ks)
                        ));
                    }
                }
                ax
            }
            None => match face_axes.first() {
                None => {
                    return Err(format!("CGS line {line}: drill needs axis= (0/1/2)"));
                }
                Some(&(ka, ks)) => {
                    for &(kb, kb_sign) in &face_axes {
                        if kb != ka {
                            return Err(format!(
                                "CGS line {line}: from/to face references use different axes (\"{}\" vs \"{}\")",
                                face_key_str(ka, ks),
                                face_key_str(kb, kb_sign)
                            ));
                        }
                    }
                    ka
                }
            },
        };

        let (rel, fbox): ([f64; 16], Option<[[f64; 3]; 2]>) = match kw.get("through") {
            Some(CgsValue::Str(s)) => {
                let insts = match self.named.get(s) {
                    Some(list) if !list.is_empty() => list,
                    _ => return Err(format!("CGS line {line}: unknown reference \"{s}\"")),
                };
                let rel0 = insts[0].rel;
                let f = mat4_mul(ctx, rel0);
                let finv = mat4_inv(f);
                let mut acc: Option<[[f64; 3]; 2]> = None;
                for i in insts {
                    if let Some(b) = self
                        .local_bounds(&i.geo)
                        .map(|b| transform_bbox(b, i.world))
                    {
                        acc = Some(match acc {
                            None => b,
                            Some(a) => [
                                [
                                    a[0][0].min(b[0][0]),
                                    a[0][1].min(b[0][1]),
                                    a[0][2].min(b[0][2]),
                                ],
                                [
                                    a[1][0].max(b[1][0]),
                                    a[1][1].max(b[1][1]),
                                    a[1][2].max(b[1][2]),
                                ],
                            ],
                        });
                    }
                }
                let b = acc
                    .ok_or_else(|| format!("CGS line {line}: drill target has no finite bounds"))?;
                (rel0, Some(transform_bbox(b, finv)))
            }
            Some(CgsValue::Geom(g)) => {
                let b = self
                    .local_bounds(&g.geo)
                    .ok_or_else(|| format!("CGS line {line}: drill target has no finite bounds"))?;
                (g.m4, Some(b))
            }
            Some(other) => {
                return Err(format!(
                    "CGS line {line}: drill.through needs a reference name or geometry, got {other}"
                ));
            }
            None => (mat4_identity(), None),
        };

        if (from_ref.is_none() || to_ref.is_none()) && fbox.is_none() {
            return Err(format!(
                "CGS line {line}: drill needs through=<name or geometry>"
            ));
        }

        let f = mat4_mul(ctx, rel);
        let a0 = if let Some(e) = &from_ref {
            self.drill_end_coord(e, f, ax, line)?
        } else {
            fbox.as_ref()
                .map(|b| b[0][ax])
                .ok_or_else(|| format!("CGS line {line}: drill needs through=<name or geometry>"))?
        };
        let a1 = if let Some(e) = &to_ref {
            self.drill_end_coord(e, f, ax, line)?
        } else {
            fbox.as_ref()
                .map(|b| b[1][ax])
                .ok_or_else(|| format!("CGS line {line}: drill needs through=<name or geometry>"))?
        };
        if !(a1 > a0) {
            return Err(format!(
                "CGS line {line}: drill extent must be non-empty ({a0} .. {a1})"
            ));
        }
        let center = (a0 + a1) / 2.0;

        let rmat = match ax {
            0 => Multivector::rotor([0.0, 1.0, 0.0], std::f64::consts::FRAC_PI_2).to_matrix(),
            1 => Multivector::rotor([1.0, 0.0, 0.0], -std::f64::consts::FRAC_PI_2).to_matrix(),
            _ => mat4_identity(),
        };
        let mut off = [0.0, 0.0, 0.0];
        off[ax] = center;
        let w = mat4_mul(mat4_mul(f, translate4(off)), rmat);
        self.add_geometry(
            Geometry::CylinderGeometry(CylinderGeometry::new(r, a1 - a0)),
            w,
            mat,
        )?;
        self.expect(TokenKind::Semi)?;
        Ok(())
    }

    fn var_stmt(&mut self, scope: &mut HashMap<String, CgsValue>, line: i32) -> Result<(), String> {
        self.take();
        let nt = self.take();
        if nt.kind != TokenKind::Ident {
            return Err(format!("CGS line {line}: var needs a name"));
        }
        if self.peek().kind != TokenKind::Assign {
            return Err(format!(
                "CGS line {}: var {} needs = <number>",
                nt.line, nt.text
            ));
        }
        self.take();
        let v = self.expr(scope, 1)?;
        let n = match v {
            CgsValue::Num(n) => n,
            _ => {
                return Err(format!(
                    "CGS line {}: var {} must be a number (solve unknowns are numeric)",
                    line, nt.text
                ));
            }
        };
        self.expect(TokenKind::Semi)?;
        scope.insert(nt.text, CgsValue::Num(n));
        Ok(())
    }

    fn constrain_stmt(
        &mut self,
        scope: &mut HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        self.take();
        self.expect(TokenKind::Lparen)?;
        let mut unknowns: Vec<String> = Vec::new();
        if self.peek().kind == TokenKind::Rparen {
            return Err(format!(
                "CGS line {line}: constrain needs at least one unknown"
            ));
        }
        loop {
            let t = self.take();
            if t.kind != TokenKind::Ident {
                return Err(format!(
                    "CGS line {}: constrain unknown must be a name",
                    t.line
                ));
            }
            unknowns.push(t.text);
            if self.peek().kind == TokenKind::Comma {
                self.take();
            } else {
                break;
            }
        }
        self.expect(TokenKind::Rparen)?;
        self.expect(TokenKind::Lbrace)?;

        let start = self.pos;
        let mut depth = 1i32;
        while depth > 0 {
            if self.pos >= self.toks.len() {
                return Err(format!("CGS line {line}: unterminated constrain block"));
            }
            let k = self.take().kind;
            if k == TokenKind::Lbrace {
                depth += 1;
            } else if k == TokenKind::Rbrace {
                depth -= 1;
            }
        }
        let body = self.toks[start..self.pos - 1].to_vec();
        let st = self.take();
        if !(st.kind == TokenKind::Ident && st.text == "solve") {
            return Err(format!(
                "CGS line {}: constrain block must be closed with `solve;`",
                st.line
            ));
        }
        self.expect(TokenKind::Semi)?;

        let mut segs: Vec<Vec<CgsToken>> = Vec::new();
        let mut cur: Vec<CgsToken> = Vec::new();
        for t in body {
            if t.kind == TokenKind::Semi {
                if !cur.is_empty() {
                    segs.push(std::mem::take(&mut cur));
                }
            } else {
                cur.push(t);
            }
        }
        if !cur.is_empty() {
            segs.push(cur);
        }
        if segs.is_empty() {
            return Err(format!("CGS line {line}: constrain block is empty"));
        }
        let mut eqs: Vec<(Vec<CgsToken>, Vec<CgsToken>, i32, ConstrainRel)> = Vec::new();
        for seg in segs {
            let eline = seg[0].line;
            let mut depth = 0i32;

            let mut rel_at: Option<(usize, &str)> = None;
            for (i, t) in seg.iter().enumerate() {
                if matches!(t.kind, TokenKind::Lparen | TokenKind::Lbracket) {
                    depth += 1;
                } else if matches!(t.kind, TokenKind::Rparen | TokenKind::Rbracket) {
                    depth -= 1;
                } else if depth == 0
                    && t.kind == TokenKind::Op
                    && matches!(t.text.as_str(), "==" | "<" | ">" | "<=" | ">=" | "!=")
                {
                    if rel_at.is_some() {
                        return Err(format!(
                            "CGS line {}: one relation per constrain equation",
                            t.line
                        ));
                    }
                    rel_at = Some((i, t.text.as_str()));
                }
            }
            match rel_at {
                None => {
                    return Err(format!(
                        "CGS line {eline}: constrain equations must be `lhs == rhs;`"
                    ));
                }
                Some((i, op)) if op == "!=" => {
                    return Err(format!(
                        "CGS line {}: constrain does not support != — use ==, <= or >=",
                        seg[i].line
                    ));
                }
                Some((i, op)) => {
                    let (l, r) = if op == ">=" || op == ">" {
                        (seg[i + 1..].to_vec(), seg[..i].to_vec())
                    } else {
                        (seg[..i].to_vec(), seg[i + 1..].to_vec())
                    };
                    let rel = if op == "==" {
                        ConstrainRel::Eq
                    } else {
                        ConstrainRel::Le
                    };
                    eqs.push((l, r, eline, rel));
                }
            }
        }

        let mut x: Vec<f64> = Vec::new();
        for u in &unknowns {
            match scope.get(u) {
                Some(CgsValue::Num(n)) => x.push(*n),
                Some(_) => {
                    return Err(format!(
                        "CGS line {line}: constrain unknown {u} must be a number"
                    ));
                }
                None => {
                    return Err(format!(
                        "CGS line {line}: constrain unknown {u} is not defined"
                    ));
                }
            }
        }

        let base = scope.clone();
        let solved = self.constrain_solve(line, &unknowns, x, &eqs, &base)?;
        for (u, v) in unknowns.iter().zip(solved) {
            scope.insert(u.clone(), CgsValue::Num(v));
        }
        Ok(())
    }

    fn eval_token_expr(
        &mut self,
        toks: &[CgsToken],
        scope: &HashMap<String, CgsValue>,
    ) -> Result<CgsValue, String> {
        let saved_toks = std::mem::replace(&mut self.toks, toks.to_vec());
        let saved_pos = self.pos;
        self.pos = 0;
        let res = self.expr(scope, 1);
        let consumed = self.pos >= self.toks.len();
        self.toks = saved_toks;
        self.pos = saved_pos;
        let v = res?;
        if !consumed {
            return Err(format!(
                "CGS line {}: trailing tokens in constraint expression",
                toks[0].line
            ));
        }
        Ok(v)
    }

    fn constrain_residual(
        &mut self,
        eqs: &[(Vec<CgsToken>, Vec<CgsToken>, i32, ConstrainRel)],
        base_scope: &HashMap<String, CgsValue>,
        unknowns: &[String],
        x: &[f64],
    ) -> Result<Vec<f64>, String> {
        let mut scope = base_scope.clone();
        for (u, v) in unknowns.iter().zip(x.iter()) {
            scope.insert(u.clone(), CgsValue::Num(*v));
        }
        let mut out: Vec<f64> = Vec::new();
        for (lhs, rhs, line, rel) in eqs {
            let a = self.eval_token_expr(lhs, &scope)?;
            let b = self.eval_token_expr(rhs, &scope)?;
            let fa = flatten_val(&a, *line)?;
            let fb = flatten_val(&b, *line)?;
            if fa.len() != fb.len() {
                return Err(format!(
                    "CGS line {line}: constraint sides must have matching shapes ({} vs {})",
                    fa.len(),
                    fb.len()
                ));
            }

            for (u, v) in fa.iter().zip(fb.iter()) {
                out.push(match rel {
                    ConstrainRel::Eq => u - v,
                    ConstrainRel::Le => (u - v).max(0.0),
                });
            }
        }
        Ok(out)
    }

    fn constrain_solve(
        &mut self,
        line: i32,
        unknowns: &[String],
        x0: Vec<f64>,
        eqs: &[(Vec<CgsToken>, Vec<CgsToken>, i32, ConstrainRel)],
        base_scope: &HashMap<String, CgsValue>,
    ) -> Result<Vec<f64>, String> {
        let n = unknowns.len();
        let mut x = x0;
        let mut lambda = 1e-3f64;
        let mut r = self.constrain_residual(eqs, base_scope, unknowns, &x)?;
        let mut converged = maxabs(&r) < 1e-10;
        let mut iter = 0;
        let mut stalled = false;
        while !converged && iter < 50 && !stalled {
            iter += 1;
            let m = r.len();

            let mut jac = vec![vec![0.0f64; n]; m];
            for k in 0..n {
                let h = 1e-6 * x[k].abs().max(1.0);
                let mut xp = x.clone();
                xp[k] += h;
                let rp = self.constrain_residual(eqs, base_scope, unknowns, &xp)?;
                if rp.len() != m {
                    return Err(format!(
                        "CGS line {line}: constraint shape changed during solve"
                    ));
                }
                for i in 0..m {
                    jac[i][k] = (rp[i] - r[i]) / h;
                }
            }

            let mut stepped = false;
            while !stepped {
                let mut a = vec![vec![0.0f64; n]; n];
                let mut b = vec![0.0f64; n];
                for p in 0..n {
                    for q in 0..n {
                        let mut s = 0.0;
                        for i in 0..m {
                            s += jac[i][p] * jac[i][q];
                        }
                        a[p][q] = s;
                    }
                    a[p][p] += lambda;
                    let mut s = 0.0;
                    for i in 0..m {
                        s += jac[i][p] * r[i];
                    }
                    b[p] = -s;
                }
                match gauss_solve(a, b) {
                    Some(dx) => {
                        let mut xn = x.clone();
                        for k in 0..n {
                            xn[k] += dx[k];
                        }
                        let rn = self.constrain_residual(eqs, base_scope, unknowns, &xn)?;
                        if maxabs(&rn) < maxabs(&r) {
                            x = xn;
                            r = rn;
                            lambda = (lambda / 10.0).max(1e-12);
                            converged = maxabs(&r) < 1e-10;
                            stepped = true;
                        } else {
                            lambda *= 10.0;
                            if lambda > 1e14 {
                                stalled = true;
                                stepped = true;
                            }
                        }
                    }
                    None => {
                        lambda *= 10.0;
                        if lambda > 1e14 {
                            stalled = true;
                            stepped = true;
                        }
                    }
                }
            }
        }
        if !converged {
            return Err(format!(
                "CGS line {line}: constrain did not converge (|r|={:.3e}, iter={iter}, var={})",
                maxabs(&r),
                unknowns.join(",")
            ));
        }
        Ok(x)
    }

    fn peek_chain(&self) -> bool {
        if self.toks.get(self.pos).map(|t| t.kind) != Some(TokenKind::Ident) {
            return false;
        }
        let mut i = self.pos + 1;
        match self.toks.get(i).map(|t| t.kind) {
            Some(TokenKind::Dot) => return true,
            Some(TokenKind::Lparen) => {}
            _ => return false,
        }
        let mut depth = 0i32;
        while let Some(t) = self.toks.get(i) {
            match t.kind {
                TokenKind::Lparen => depth += 1,
                TokenKind::Rparen => {
                    depth -= 1;
                    if depth == 0 {
                        return self.toks.get(i + 1).map(|t| t.kind) == Some(TokenKind::Dot);
                    }
                }
                TokenKind::Eof => return false,
                _ => {}
            }
            i += 1;
        }
        false
    }

    fn statement(
        &mut self,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
    ) -> Result<(), String> {
        let t = self.peek();
        if t.kind == TokenKind::Lbrace {
            self.expect(TokenKind::Lbrace)?;
            while self.peek().kind != TokenKind::Rbrace {
                self.statement(ctx, mat, scope)?;
            }
            self.expect(TokenKind::Rbrace)?;
            return Ok(());
        }
        if t.kind == TokenKind::Ident {
            let name = t.text.as_str();

            if self.peek1().kind == TokenKind::Assign {
                self.take();
                self.take();
                let v = self.expr(scope, 1)?;
                self.expect(TokenKind::Semi)?;
                if name == "background" {
                    let c = cgs_num(&v, t.line, "background")?;
                    self.scene.background = Color::from_hex(c as i32);
                }
                scope.insert(name.to_string(), v);
                return Ok(());
            }
            if self.peek1().kind == TokenKind::Lbracket {
                return Err(format!(
                    "CGS line {}: indexing is not supported — use comp(vector, index)",
                    t.line
                ));
            }

            if self.peek_chain() {
                let v = self.expr(scope, 1)?;
                let g = geom_val(&v, t.line, "expression statement")?;
                self.expect(TokenKind::Semi)?;
                return self.add_geometry(g.geo, mat4_mul(ctx, g.m4), mat);
            }
            if name == "module" {
                self.module_def()?;
                return Ok(());
            }
            if name == "for" {
                self.for_loop(ctx, mat, scope)?;
                return Ok(());
            }
            if name == "if" {
                self.if_stmt(ctx, mat, scope)?;
                return Ok(());
            }
            if name == "echo" {
                self.echo_stmt(scope)?;
                return Ok(());
            }
            if name == "show" {
                self.show_stmt(ctx, mat, scope, t.line)?;
                return Ok(());
            }
            if name == "tag" {
                self.tag_stmt(ctx, mat, scope, t.line)?;
                return Ok(());
            }
            if name == "drill" {
                self.drill_stmt(ctx, mat, scope, t.line)?;
                return Ok(());
            }
            if name == "var" {
                self.var_stmt(scope, t.line)?;
                return Ok(());
            }
            if name == "constrain" {
                self.constrain_stmt(scope, t.line)?;
                return Ok(());
            }
            if name == "union" || name == "difference" || name == "intersection" {
                self.take();
                self.expect(TokenKind::Lparen)?;
                if self.peek().kind == TokenKind::Rparen {
                    self.take();
                    if name == "union" {
                        self.body(ctx, mat, scope, t.line)?;
                        return Ok(());
                    }
                    let op = if name == "difference" {
                        CsgOp::Difference
                    } else {
                        CsgOp::Intersection
                    };
                    self.csg_block(op, ctx, mat, scope, t.line)?;
                    return Ok(());
                }

                let (pos, kw) = self.paren_args(scope)?;
                let v = self
                    .geom_expr_call(name, pos, kw, t.line)?
                    .ok_or_else(|| format!("CGS line {}: {name} is not callable here", t.line))?;
                let g = geom_val(&v, t.line, name)?;
                self.expect(TokenKind::Semi)?;
                return self.add_geometry(g.geo, mat4_mul(ctx, g.m4), mat);
            }
        }
        let t = self.take();
        if t.kind != TokenKind::Ident {
            return Err(format!(
                "CGS line {}: expected statement name, got {}",
                t.line, t.kind
            ));
        }
        let (pos, kw) = self.call_args(scope)?;
        let name = t.text.clone();
        if self.modules.contains_key(&name) {
            self.module_call(&name, pos, kw, ctx, mat, t.line)?;
            return Ok(());
        }
        if is_expr_only_fn(&name) {
            return Err(format!(
                "CGS line {}: {} is an expression function and cannot be used as a statement",
                t.line, name
            ));
        }
        let args = self.resolve(&name, pos, kw, t.line)?;
        if name == "translate" {
            let v = cgs_vec3(
                &args.get("t").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "translate.t",
            )?;
            let m2 = mat4_mul(ctx, translate4(v));
            self.body(m2, mat, scope, t.line)?;
            return Ok(());
        }
        if name == "rotate" {
            let ax = cgs_vec3(
                &args.get("axis").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "rotate.axis",
            )?;
            let ang = cgs_num(
                &args.get("angle").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "rotate.angle",
            )?;
            let m2 = mat4_mul(ctx, Multivector::rotor(ax, ang).to_matrix());
            self.body(m2, mat, scope, t.line)?;
            return Ok(());
        }
        if name == "scale" {
            let sv = args.get("s").cloned().unwrap_or(CgsValue::Num(1.0));
            let s4 = match sv {
                CgsValue::Vec3(v3) => scale4([v3.x, v3.y, v3.z]),
                CgsValue::List(_) => {
                    let v = cgs_vec3(&sv, t.line, "scale.s")?;
                    scale4(v)
                }
                _ => {
                    let x = cgs_num(&sv, t.line, "scale.s")?;
                    scale4([x, x, x])
                }
            };
            let m2 = mat4_mul(ctx, s4);
            self.body(m2, mat, scope, t.line)?;
            return Ok(());
        }
        if name == "mirror" {
            let ax = cgs_vec3(
                &args.get("axis").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "mirror.axis",
            )?;
            let m2 = mat4_mul(ctx, mirror4(ax)?);
            self.body(m2, mat, scope, t.line)?;
            return Ok(());
        }
        if name == "material" {
            let mut merged: HashMap<String, CgsValue> = HashMap::new();
            for (k, v) in mat {
                merged.insert(k.clone(), v.clone());
            }
            for (k, v) in &args {
                merged.insert(k.clone(), v.clone());
            }
            self.body(ctx, &merged, scope, t.line)?;
            return Ok(());
        }
        if name == "background" {
            self.expect(TokenKind::Semi)?;
            let c = cgs_num(
                &args.get("color").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "color",
            )?;
            self.scene.background = Color::from_hex(c as i32);
            return Ok(());
        }
        if name == "camera" {
            self.expect(TokenKind::Semi)?;
            let fov = cgs_num(
                &args.get("fov").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "fov",
            )?;
            let aspect = cgs_num(
                &args.get("aspect").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "aspect",
            )?;
            let campos = cgs_vec3(
                &args.get("position").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "camera.position",
            )?;
            let tgt = cgs_vec3(
                &args.get("target").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "camera.target",
            )?;

            if fov <= 0.0 || fov >= 180.0 {
                return Err(format!(
                    "CGS line {}: camera.fov must be in (0, 180), got {}",
                    t.line, fov
                ));
            }
            if aspect <= 0.0 {
                return Err(format!(
                    "CGS line {}: camera.aspect must be > 0, got {}",
                    t.line, aspect
                ));
            }
            let mut cam =
                PerspectiveCamera::new(fov, aspect, 0.1, 100.0, campos, tgt, [0.0, 1.0, 0.0]);
            cam.look_at(tgt, None);
            self.camera = Some(cam);
            return Ok(());
        }
        if name.ends_with("_light") {
            self.expect(TokenKind::Semi)?;
            self.add_light(&name, &args, t.line)?;
            return Ok(());
        }
        self.expect(TokenKind::Semi)?;
        let geo = self.build_geometry(&name, &args, t.line)?;
        self.add_geometry(geo, ctx, mat)
    }

    fn body(
        &mut self,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        if self.peek().kind == TokenKind::Lbrace {
            self.expect(TokenKind::Lbrace)?;
            while self.peek().kind != TokenKind::Rbrace {
                self.statement(ctx, mat, scope)?;
            }
            self.expect(TokenKind::Rbrace)?;
        } else if self.peek().kind == TokenKind::Semi {
            return Err(format!(
                "CGS line {line}: modifier missing target statement"
            ));
        } else {
            self.statement(ctx, mat, scope)?;
        }
        Ok(())
    }

    fn for_loop(
        &mut self,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
    ) -> Result<(), String> {
        self.take();
        self.expect(TokenKind::Lparen)?;
        let vt = self.take();
        if vt.kind != TokenKind::Ident || self.peek().kind != TokenKind::Assign {
            return Err(format!("CGS line {}: for needs (var = list)", vt.line));
        }
        self.take();
        let values = self.expr(scope, 1)?;
        self.expect(TokenKind::Rparen)?;
        match values {
            CgsValue::Vec3(v3) => {
                let body = self.capture_statement();
                for v in [v3.x, v3.y, v3.z] {
                    scope.insert(vt.text.clone(), CgsValue::Num(v));
                    self.run_tokens(body.clone(), ctx, mat, scope)?;
                }
            }
            CgsValue::List(list) => {
                let body = self.capture_statement();
                for v in list {
                    scope.insert(vt.text.clone(), v);
                    self.run_tokens(body.clone(), ctx, mat, scope)?;
                }
            }
            _ => return Err(format!("CGS line {}: for needs a list", vt.line)),
        }
        Ok(())
    }

    fn if_stmt(
        &mut self,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
    ) -> Result<(), String> {
        self.take();
        self.expect(TokenKind::Lparen)?;
        let cond = cgs_truthy(&self.expr(scope, 1)?);
        self.expect(TokenKind::Rparen)?;
        if cond {
            self.statement(ctx, mat, scope)?;
            if self.peek().kind == TokenKind::Ident && self.peek().text == "else" {
                self.take();
                self.skip_statement();
            }
        } else {
            self.skip_statement();
            if self.peek().kind == TokenKind::Ident && self.peek().text == "else" {
                self.take();
                self.statement(ctx, mat, scope)?;
            }
        }
        Ok(())
    }

    fn echo_stmt(&mut self, scope: &HashMap<String, CgsValue>) -> Result<(), String> {
        self.take();
        self.expect(TokenKind::Lparen)?;
        let mut vals: Vec<CgsValue> = Vec::new();
        if self.peek().kind != TokenKind::Rparen {
            loop {
                vals.push(self.expr(scope, 1)?);
                if self.peek().kind == TokenKind::Comma {
                    self.take();
                } else {
                    break;
                }
            }
        }
        self.expect(TokenKind::Rparen)?;
        self.expect(TokenKind::Semi)?;
        print!("ECHO:");
        for v in &vals {
            print!(" {v}");
        }
        println!();
        Ok(())
    }

    fn module_def(&mut self) -> Result<(), String> {
        self.take();
        let nt = self.take();
        if nt.kind != TokenKind::Ident {
            return Err(format!("CGS line {}: module missing name", nt.line));
        }
        self.expect(TokenKind::Lparen)?;
        if self.peek().kind != TokenKind::Rparen {
            loop {
                let pt = self.take();
                if pt.kind != TokenKind::Ident {
                    return Err(format!(
                        "CGS line {}: module parameter must be a name",
                        pt.line
                    ));
                }
                let key = format!("{}:{}", nt.text, pt.text);
                if self.peek().kind == TokenKind::Assign {
                    self.take();
                    let def = self.capture_expr()?;
                    self.set_param(key, def);
                } else {
                    self.set_param(key, Vec::new());
                }
                if self.peek().kind == TokenKind::Comma {
                    self.take();
                } else {
                    break;
                }
            }
        }
        self.expect(TokenKind::Rparen)?;
        self.expect(TokenKind::Lbrace)?;
        let start = self.pos;
        let mut depth = 1;
        while depth > 0 {
            let k = self.take().kind;
            if k == TokenKind::Lbrace {
                depth += 1;
            } else if k == TokenKind::Rbrace {
                depth -= 1;
            }
        }
        self.modules
            .insert(nt.text.clone(), self.toks[start..self.pos - 1].to_vec());
        Ok(())
    }

    fn set_param(&mut self, key: String, val: Vec<CgsToken>) {
        if !self.params.contains_key(&key) {
            self.param_order.push(key.clone());
        }
        self.params.insert(key, val);
    }

    fn module_call(
        &mut self,
        name: &str,
        pos: Vec<CgsValue>,
        kw: HashMap<String, CgsValue>,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        self.expect(TokenKind::Semi)?;

        let body = match self.modules.get(name) {
            Some(b) => b.clone(),
            None => Vec::new(),
        };
        let mut scope: HashMap<String, CgsValue> = HashMap::new();
        scope.insert("pi".to_string(), CgsValue::Num(std::f64::consts::PI));

        let mut names: Vec<String> = Vec::new();
        let prefix = format!("{name}:");
        for k in &self.param_order {
            if k.starts_with(&prefix) {
                names.push(k[prefix.len()..].to_string());
            }
        }
        for (i, pname) in names.iter().enumerate() {
            if i < pos.len() {
                scope.insert(pname.clone(), pos[i].clone());
            }
        }
        for (k, v) in kw {
            scope.insert(k, v);
        }
        for pname in &names {
            if !scope.contains_key(pname) {
                let def = self
                    .params
                    .get(&format!("{name}:{pname}"))
                    .cloned()
                    .unwrap_or_default();
                if def.is_empty() {
                    return Err(format!(
                        "CGS line {line}: module {name} missing parameter {pname}"
                    ));
                }
                let v = self.eval_tokens(def, &scope)?;
                scope.insert(pname.clone(), v);
            }
        }
        self.run_tokens(body, ctx, mat, &mut scope)
    }

    fn capture_expr(&mut self) -> Result<Vec<CgsToken>, String> {
        let start = self.pos;
        let mut depth = 0;
        loop {
            let t = self.peek();
            if t.kind == TokenKind::Eof {
                return Err(format!("CGS line {}: unclosed expression", t.line));
            }
            if depth == 0
                && matches!(
                    t.kind,
                    TokenKind::Comma | TokenKind::Rparen | TokenKind::Semi
                )
            {
                break;
            }
            if matches!(t.kind, TokenKind::Lparen | TokenKind::Lbracket) {
                depth += 1;
            } else if matches!(t.kind, TokenKind::Rparen | TokenKind::Rbracket) {
                depth -= 1;
            }
            self.take();
        }
        Ok(self.toks[start..self.pos].to_vec())
    }

    fn eval_tokens(
        &mut self,
        toks: Vec<CgsToken>,
        scope: &HashMap<String, CgsValue>,
    ) -> Result<CgsValue, String> {
        let saved = std::mem::replace(&mut self.toks, toks);
        let saved_pos = self.pos;
        self.pos = 0;
        let v = self.expr(scope, 1)?;
        self.toks = saved;
        self.pos = saved_pos;
        Ok(v)
    }

    fn capture_statement(&mut self) -> Vec<CgsToken> {
        let start = self.pos;
        self.skip_statement();
        self.toks[start..self.pos].to_vec()
    }

    fn skip_statement(&mut self) {
        let t = self.peek();
        if t.kind == TokenKind::Lbrace {
            self.take();
            let mut depth = 1;
            while depth > 0 {
                let k = self.take().kind;
                if k == TokenKind::Lbrace {
                    depth += 1;
                } else if k == TokenKind::Rbrace {
                    depth -= 1;
                }
            }
            return;
        }
        if t.kind == TokenKind::Ident && t.text == "if" {
            self.take();
            self.skip_parens();
            self.skip_statement();
            if self.peek().kind == TokenKind::Ident && self.peek().text == "else" {
                self.take();
                self.skip_statement();
            }
            return;
        }
        if t.kind == TokenKind::Ident && (t.text == "for" || t.text == "union" || t.text == "echo")
        {
            self.take();
            self.skip_parens();
            if t.text == "echo" {
                let _ = self.expect(TokenKind::Semi);
            } else {
                self.skip_statement();
            }
            return;
        }
        if t.kind == TokenKind::Ident && t.text == "module" {
            self.take();
            self.take();
            self.skip_parens();
            self.skip_statement();
            return;
        }
        self.take();
        if self.peek().kind == TokenKind::Assign {
            while self.take().kind != TokenKind::Semi {}
            return;
        }
        if self.peek().kind == TokenKind::Lparen {
            self.skip_parens();
            let nxt = self.peek().kind;
            if nxt == TokenKind::Lbrace {
                self.skip_statement();
            } else if nxt == TokenKind::Semi {
                self.take();
            } else {
                self.skip_statement();
            }
            return;
        }
        while self.take().kind != TokenKind::Semi {}
    }

    fn skip_parens(&mut self) {
        if self.expect(TokenKind::Lparen).is_err() {
            return;
        }
        let mut depth = 1;
        while depth > 0 {
            let k = self.take().kind;
            if k == TokenKind::Lparen {
                depth += 1;
            } else if k == TokenKind::Rparen {
                depth -= 1;
            }
        }
    }

    fn call_args(
        &mut self,
        scope: &HashMap<String, CgsValue>,
    ) -> Result<(Vec<CgsValue>, HashMap<String, CgsValue>), String> {
        self.expect(TokenKind::Lparen)?;
        self.paren_args(scope)
    }

    fn paren_args(
        &mut self,
        scope: &HashMap<String, CgsValue>,
    ) -> Result<(Vec<CgsValue>, HashMap<String, CgsValue>), String> {
        let mut pos: Vec<CgsValue> = Vec::new();
        let mut kw: HashMap<String, CgsValue> = HashMap::new();
        if self.peek().kind != TokenKind::Rparen {
            loop {
                let t = self.peek();
                if t.kind == TokenKind::Ident && self.peek1().kind == TokenKind::Assign {
                    self.take();
                    self.take();
                    let v = self.expr(scope, 1)?;
                    kw.insert(t.text.clone(), v);
                } else {
                    pos.push(self.expr(scope, 1)?);
                }
                if self.peek().kind == TokenKind::Comma {
                    self.take();
                } else {
                    break;
                }
            }
        }
        self.expect(TokenKind::Rparen)?;
        Ok((pos, kw))
    }

    fn resolve(
        &self,
        name: &str,
        pos: Vec<CgsValue>,
        kw: HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<HashMap<String, CgsValue>, String> {
        let names = cgs_sig_names(name);
        let defaults = cgs_sig_defaults(name);
        let mut merged: HashMap<String, CgsValue> = HashMap::new();
        for (k, v) in &defaults {
            merged.insert(k.clone(), v.clone());
        }
        if pos.len() > names.len() {
            return Err(format!("CGS line {line}: {name} too many positional args"));
        }
        for (i, pname) in names.iter().enumerate() {
            if i < pos.len() {
                merged.insert(pname.to_string(), pos[i].clone());
            }
        }
        for (k, v) in kw {
            if !names.contains(&k.as_str()) && !defaults.contains_key(&k) {
                return Err(format!("CGS line {line}: {name} has no parameter {k}"));
            }
            merged.insert(k, v);
        }
        for pname in &names {
            if let Some(CgsValue::Num(x)) = merged.get(*pname) {
                if x.is_nan() && *pname == "cylinder.h" {
                    merged.insert(pname.to_string(), CgsValue::Num(-1.0));
                }
            }
        }
        Ok(merged)
    }

    fn add_geometry(
        &mut self,
        geo: Geometry,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
    ) -> Result<(), String> {
        self.register(&geo, ctx, ctx);
        if self.collecting {
            self.collect.push(CollectedGeom { geo, m4: ctx });
            return Ok(());
        }
        let (motor, lin) = decompose_rigid(ctx);
        let g2 = if is_identity3(lin) {
            geo
        } else {
            Geometry::AffineGeometry(AffineGeometry::new(geo, lin))
        };
        self.scene.add_mesh(Mesh::new(MeshParams {
            geometry: g2,
            material: self.build_material(mat)?,
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: Some(motor),
        }));
        Ok(())
    }

    fn csg_block(
        &mut self,
        op: CsgOp,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        let saved = std::mem::take(&mut self.collect);
        let was_collecting = self.collecting;
        self.collecting = true;
        self.body(ctx, mat, scope, line)?;
        let children = std::mem::replace(&mut self.collect, saved);
        self.collecting = was_collecting;
        if children.len() < 2 {
            return Err(format!(
                "CGS line {line}: {} needs >= 2 geometry children",
                csg_op_name(op)
            ));
        }
        let mut kids: Vec<Geometry> = Vec::new();
        for c in children {
            if matches!(c.geo, Geometry::CircleGeometry(_)) {
                return Err(format!(
                    "CGS line {line}: {} children must be solids (circle is not)",
                    csg_op_name(op)
                ));
            }
            if let Geometry::BezierPatchGeometry(b) = &c.geo {
                if !b.is_solid() {
                    return Err(format!(
                        "CGS line {line}: {} children must be solids (bezier surface with thickness=0 is not)",
                        csg_op_name(op)
                    ));
                }
            }
            let (cm, cl) = decompose_rigid(c.m4);
            kids.push(Geometry::AffineGeometry(AffineGeometry::with_motor(
                c.geo, cm, cl,
            )));
        }

        let result = Geometry::CsgGeometry(CsgGeometry::new(op, kids));
        self.register(&result, mat4_identity(), ctx);
        if was_collecting {
            self.collect.push(CollectedGeom {
                geo: result,
                m4: mat4_identity(),
            });
            return Ok(());
        }
        self.scene.add_mesh(Mesh::new(MeshParams {
            geometry: result,
            material: self.build_material(mat)?,
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: Some(Multivector::identity()),
        }));
        Ok(())
    }

    fn build_geometry(
        &mut self,
        name: &str,
        args: &HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<Geometry, String> {
        validate_geometry_params(name, args, line)?;
        match name {
            "sphere" => {
                let r = cgs_num(
                    &args.get("r").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "sphere.r",
                )?;
                Ok(Geometry::SphereGeometry(SphereGeometry::new(r)))
            }
            "plane" => {
                let n = cgs_vec3(
                    &args.get("n").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "plane.n",
                )?;
                let d = cgs_num(
                    &args.get("d").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "plane.d",
                )?;
                Ok(Geometry::PlaneGeometry(PlaneGeometry::new(n, d)))
            }
            "cylinder" => {
                let h = cgs_num(
                    &args.get("h").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cylinder.h",
                )?;
                let r = cgs_num(
                    &args.get("r").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cylinder.r",
                )?;
                Ok(Geometry::CylinderGeometry(CylinderGeometry::new(
                    r,
                    if h < 0.0 { -1.0 } else { h },
                )))
            }
            "box" => {
                let s = cgs_vec3(
                    &args.get("s").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "box.s",
                )?;
                Ok(Geometry::BoxGeometry(BoxGeometry::new(s[0], s[1], s[2])))
            }
            "circle" => {
                let r = cgs_num(
                    &args.get("r").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "circle.r",
                )?;
                Ok(Geometry::CircleGeometry(CircleGeometry::new(r)))
            }
            "cone" => {
                let r = cgs_num(
                    &args.get("r").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cone.r",
                )?;
                let h = cgs_num(
                    &args.get("h").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cone.h",
                )?;
                Ok(Geometry::ConeGeometry(ConeGeometry::new(r, h)))
            }
            "torus" => {
                let r1 = cgs_num(
                    &args.get("R").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "torus.R",
                )?;
                let r2 = cgs_num(
                    &args.get("r").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "torus.r",
                )?;
                Ok(Geometry::TorusGeometry(TorusGeometry::new(r1, r2)))
            }
            "cyclide" => {
                let a = cgs_num(
                    &args.get("a").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cyclide.a",
                )?;
                let b = cgs_num(
                    &args.get("b").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cyclide.b",
                )?;
                let d = cgs_num(
                    &args.get("d").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cyclide.d",
                )?;
                Ok(Geometry::CyclideGeometry(CyclideGeometry::new(
                    a,
                    b,
                    d,
                    [0.0, 0.0, 0.0],
                )))
            }
            "ellipsoid" => {
                let r = cgs_vec3(
                    &args.get("radii").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "ellipsoid.radii",
                )?;
                Ok(Geometry::EllipsoidGeometry(EllipsoidGeometry::new(
                    r[0], r[1], r[2],
                )))
            }
            "bezier" => {
                let raw = args.get("points").cloned().unwrap_or(CgsValue::Num(0.0));
                let pts = points16(&raw, line, "bezier.points")?;
                let thickness = cgs_num(
                    &args.get("thickness").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "bezier.thickness",
                )?;
                let div = cgs_num(
                    &args.get("div").cloned().unwrap_or(CgsValue::Num(8.0)),
                    line,
                    "bezier.div",
                )? as usize;
                Ok(Geometry::BezierPatchGeometry(BezierPatchGeometry::new(
                    &pts, thickness, div,
                )))
            }
            "extrude" => {
                let prof = profile2d(
                    &args.get("profile").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "extrude.profile",
                )?;
                let h = cgs_num(
                    &args.get("h").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "extrude.h",
                )?;
                let (v, f) = extrude(&prof, h);
                Ok(Geometry::TrimeshGeometry(TrimeshGeometry::new(&v, &f)))
            }
            "loft" => {
                let raw = args.get("profiles").cloned().unwrap_or(CgsValue::Num(0.0));
                match raw {
                    CgsValue::List(list) => {
                        if list.len() < 2 {
                            return Err(format!(
                                "CGS line {line}: loft.profiles needs >= 2 sections"
                            ));
                        }
                        let mut profiles: Vec<Vec<[f64; 2]>> = Vec::new();
                        for p in &list {
                            profiles.push(profile2d(p, line, "loft.profiles[i]")?);
                        }
                        let zsraw = args.get("zs").cloned().unwrap_or(CgsValue::Num(0.0));
                        match zsraw {
                            CgsValue::Vec3(v3) => {
                                let (v, f) = loft(&profiles, &[v3.x, v3.y, v3.z]);
                                Ok(Geometry::TrimeshGeometry(TrimeshGeometry::new(&v, &f)))
                            }
                            CgsValue::List(zlist) => {
                                let mut zs: Vec<f64> = Vec::new();
                                for z in &zlist {
                                    zs.push(cgs_num(z, line, "loft.zs[i]")?);
                                }
                                let (v, f) = loft(&profiles, &zs);
                                Ok(Geometry::TrimeshGeometry(TrimeshGeometry::new(&v, &f)))
                            }
                            _ => Err(format!("CGS line {line}: loft.zs needs a list")),
                        }
                    }
                    _ => Err(format!("CGS line {line}: loft.profiles needs a list")),
                }
            }
            "mesh" => {
                let p = args.get("file").cloned().unwrap_or(CgsValue::Num(0.0));
                match p {
                    CgsValue::Str(path) => {
                        if self.asset_root.is_empty() {
                            return Err(format!(
                                "CGS line {line}: mesh needs an explicit asset_root"
                            ));
                        }
                        let full = format!("{}/{}", self.asset_root, path);
                        if full.ends_with(".obj") {
                            let (v, f) = load_obj(&full)?;
                            return Ok(Geometry::TrimeshGeometry(TrimeshGeometry::new(&v, &f)));
                        }
                        if full.ends_with(".glb") || full.ends_with(".gltf") {
                            let loaded = load_gltf(&full)?;
                            return Ok(gltf_to_geometry(&loaded));
                        }
                        Err(format!(
                            "CGS line {line}: unsupported mesh file \"{path}\" (use .obj/.glb/.gltf)"
                        ))
                    }
                    _ => Err(format!("CGS line {line}: mesh.file needs a string path")),
                }
            }
            _ => Err(format!("CGS line {line}: unknown primitive {name}")),
        }
    }

    fn build_material(&self, mat: &HashMap<String, CgsValue>) -> Result<Material, String> {
        let color = match mat.get("color") {
            Some(v) => {
                let c = cgs_num(v, 0, "color")?;
                Color::from_hex(c as i32)
            }
            None => Color::from_hex(0xFFFFFF),
        };
        let mut tex = None;
        if let Some(v) = mat.get("map") {
            match v {
                CgsValue::Str(path) => {
                    if !path.is_empty() {
                        if self.asset_root.is_empty() {
                            return Err("CGS material.map needs an explicit asset_root".to_string());
                        }
                        tex = Some(texture_load(&format!("{}/{}", self.asset_root, path))?);
                    }
                }
                _ => return Err("CGS material.map needs a string path".to_string()),
            }
        }
        if let Some(u) = mat.get("unlit") {
            if cgs_truthy(u) {
                let op = match mat.get("opacity") {
                    Some(o) => cgs_num(o, 0, "opacity")?,
                    None => 1.0,
                };
                return Ok(Material::basic(color, clamp01(op)));
            }
        }
        let roughness = cgs_opt_num(
            &mat.get("roughness").cloned().unwrap_or(CgsValue::Num(-1.0)),
            0.5,
        );
        let metalness = cgs_opt_num(
            &mat.get("metalness").cloned().unwrap_or(CgsValue::Num(-1.0)),
            0.0,
        );
        let emissive = match mat.get("emissive") {
            Some(v) => {
                let ev = cgs_num(v, 0, "emissive")?;
                if ev < 0.0 {
                    Color::from_hex(0x000000)
                } else {
                    Color::from_hex(ev as i32)
                }
            }
            None => Color::from_hex(0x000000),
        };
        let opacity = cgs_opt_num(
            &mat.get("opacity").cloned().unwrap_or(CgsValue::Num(-1.0)),
            1.0,
        );
        let ior = cgs_opt_num(&mat.get("ior").cloned().unwrap_or(CgsValue::Num(-1.0)), 1.5);
        let absorption = cgs_opt_num(
            &mat.get("absorption")
                .cloned()
                .unwrap_or(CgsValue::Num(-1.0)),
            0.0,
        );
        let mut m = Material::standard(MaterialParams {
            color,
            roughness,
            metalness,
            emissive,
            opacity,
            ior,
            absorption,
        });
        if let Some(t) = tex {
            m.map = Some(t);
        }
        Ok(m)
    }

    fn add_light(
        &mut self,
        name: &str,
        args: &HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        let c = cgs_num(
            &args.get("color").cloned().unwrap_or(CgsValue::Num(0.0)),
            line,
            "color",
        )?;
        let color = Color::from_hex(c as i32);
        let intensity = cgs_num(
            &args.get("intensity").cloned().unwrap_or(CgsValue::Num(0.0)),
            line,
            "intensity",
        )?;
        if name == "directional_light" {
            let d = cgs_vec3(
                &args.get("direction").cloned().unwrap_or(CgsValue::Num(0.0)),
                line,
                "direction",
            )?;
            self.scene
                .add_light(Light::directional(color, intensity, d));
        } else if name == "point_light" {
            let p = cgs_vec3(
                &args.get("position").cloned().unwrap_or(CgsValue::Num(0.0)),
                line,
                "position",
            )?;
            self.scene.add_light(Light::point(color, intensity, p));
        } else {
            self.scene.add_light(Light::ambient(color, intensity));
        }
        Ok(())
    }
}

pub fn cgs_run_result(text: &str, asset_root: &str) -> Result<CgsRun, String> {
    let toks = cgs_lex(text)?;
    let mut l = SceneLoader {
        toks,
        pos: 0,
        asset_root: asset_root.to_string(),
        scene: Scene::new(None),
        camera: None,
        modules: HashMap::new(),
        params: HashMap::new(),
        param_order: Vec::new(),
        collect: Vec::new(),
        collecting: false,
        named: HashMap::new(),
        pending: Vec::new(),
    };
    let mut root_scope: HashMap<String, CgsValue> = HashMap::new();
    root_scope.insert("pi".to_string(), CgsValue::Num(std::f64::consts::PI));
    let toks = l.toks.clone();
    l.run_tokens(toks, mat4_identity(), &HashMap::new(), &mut root_scope)?;
    let cam = match l.camera {
        Some(c) => c,
        None => {
            let mut c2 = PerspectiveCamera::new(
                50.0,
                16.0 / 9.0,
                0.1,
                100.0,
                [0.0, 0.0, 5.0],
                [0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            );
            c2.look_at([0.0, 0.0, 0.0], None);
            c2
        }
    };
    let tags = l
        .named
        .into_iter()
        .map(|(name, insts)| {
            let v = insts
                .into_iter()
                .map(|i| TagInstance {
                    geo: i.geo,
                    world: i.world,
                    rel: i.rel,
                })
                .collect();
            (name, v)
        })
        .collect();
    Ok(CgsRun {
        scene: l.scene,
        camera: cam,
        tags,
    })
}
