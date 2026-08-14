"""CGS 解析 + 求值: token 流 → (Scene, PerspectiveCamera)。

v2 (OpenSCAD 对齐): 变量 / 算术表达式 / 数学函数 / for + range /
module 参数化子树 / if-else / echo / union 分组。

作用域约定 (文档即承诺): 全局单层作用域; {} 块不新开作用域 (块内
赋值泄漏到外层, 与 OpenSCAD 行为接近); for 循环变量写在包含作用域;
module 体 = 全局作用域的子作用域 + 形参绑定 (词法, 看不到调用点局部
变量)。motor/材质上下文在 module 调用点正常继承。
"""

import math
from typing import ClassVar

from cga.engine import (
    AmbientLight,
    BoxGeometry,
    CircleGeometry,
    Color,
    CylinderGeometry,
    DirectionalLight,
    Mesh,
    MeshBasicMaterial,
    MeshStandardMaterial,
    PerspectiveCamera,
    PlaneGeometry,
    PointLight,
    Scene,
    SphereGeometry,
)
from cga.motors import Motor
from cga.scene_lang.lexer import Lexer


class SceneLoader:
    """CGS 文本 → (Scene, PerspectiveCamera)。单遍解析即求值。"""

    SIGNATURES: ClassVar[dict[str, tuple[list[str], dict]]] = {
        # 图元
        "sphere": (["r"], {}),
        "plane": (["n"], {"d": 0.0}),
        "cylinder": (["r"], {"h": None}),  # h 给定 → 有限圆柱 (带端盖)
        "box": (["s"], {}),
        "circle": (["r"], {}),
        # 修饰符
        "translate": (["t"], {}),
        "rotate": (["axis", "angle"], {}),
        "material": (
            [],
            {
                "color": None,
                "roughness": None,
                "metalness": None,
                "emissive": None,
                "opacity": None,
                "ior": None,
                "absorption": None,
                "unlit": None,
            },
        ),
        # 灯光
        "directional_light": (["direction"], {"intensity": 1.0, "color": 0xFFFFFF}),
        "point_light": (["position"], {"intensity": 1.0, "color": 0xFFFFFF}),
        "ambient_light": ([], {"intensity": 0.3, "color": 0xFFFFFF}),
        # 场景设置
        "background": (["color"], {}),
        "camera": (
            [],
            {
                "fov": 50.0,
                "aspect": 16.0 / 9.0,
                "position": (0.0, 0.0, 5.0),
                "target": (0.0, 0.0, 0.0),
            },
        ),
    }
    MODIFIERS: ClassVar[tuple[str, ...]] = ("translate", "rotate", "material")

    # 表达式里的数学函数 (OpenSCAD 内建子集)
    FUNCTIONS: ClassVar[dict] = {
        "abs": abs,
        "sign": lambda x: (x > 0) - (x < 0),
        "sin": math.sin,
        "cos": math.cos,
        "tan": math.tan,
        "asin": math.asin,
        "acos": math.acos,
        "atan": math.atan,
        "atan2": math.atan2,
        "sqrt": math.sqrt,
        "exp": math.exp,
        "ln": math.log,
        "log": math.log10,
        "floor": math.floor,
        "ceil": math.ceil,
        "round": round,
        "pow": pow,
        "min": min,
        "max": max,
    }
    PRECEDENCE: ClassVar[dict[str, int]] = {
        "||": 1,
        "&&": 2,
        "==": 3,
        "!=": 3,
        "<": 4,
        "<=": 4,
        ">": 4,
        ">=": 4,
        "+": 5,
        "-": 5,
        "*": 6,
        "/": 6,
        "%": 6,
    }

    def __init__(self, tokens: list[tuple[str, object, int]]):
        self.toks = tokens
        self.pos = 0
        self.scene = Scene()
        self.camera: PerspectiveCamera | None = None
        self.modules: dict[str, tuple[list[tuple[str, list | None]], list]] = {}

    @staticmethod
    def load(text: str) -> tuple[Scene, PerspectiveCamera]:
        """解析 CGS 文本 → (Scene, 相机)。无 camera 语句时给默认机位。"""
        loader = SceneLoader(Lexer.tokenize(text))
        loader.run_tokens(loader.toks, Motor.identity(), {}, {"pi": math.pi})
        if loader.camera is None:
            loader.camera = PerspectiveCamera()
            loader.camera.look_at((0.0, 0.0, 0.0))
        return loader.scene, loader.camera

    # ── 词法游走 ──────────────────────────────────────────────────

    def peek(self, ahead: int = 0) -> tuple[str, object, int]:
        if self.pos + ahead >= len(self.toks):
            line = self.toks[-1][2] if self.toks else 1
            return ("eof", None, line)
        return self.toks[self.pos + ahead]

    def take(self) -> tuple[str, object, int]:
        tok = self.peek()
        self.pos += 1
        return tok

    def expect(self, sym: str) -> int:
        kind, _v, line = self.take()
        if kind != sym:
            raise ValueError(f"CGS 第{line}行: 期望 {sym!r}, 得到 {kind!r}")
        return line

    def run_tokens(self, toks: list, motor: Motor, mat: dict, scope: dict) -> None:
        """在指定上下文求值一段 token 流 (顶层 / for 体 / module 体共用)。"""
        saved_toks, saved_pos = self.toks, self.pos
        self.toks, self.pos = toks, 0
        while self.pos < len(self.toks):
            self.statement(motor, mat, scope)
        self.toks, self.pos = saved_toks, saved_pos

    # ── 表达式 (优先级爬升) ────────────────────────────────────────

    def expr(self, scope: dict, min_prec: int = 1):
        lhs = self.unary(scope)
        while True:
            kind, op, _line = self.peek()
            if kind != "op" or not isinstance(op, str):
                return lhs
            prec = self.PRECEDENCE.get(op, -1)
            if prec < min_prec:
                return lhs
            self.take()
            rhs = self.expr(scope, prec + 1)
            lhs = self.binop(op, lhs, rhs)

    def unary(self, scope: dict):
        kind, op, line = self.peek()
        if kind == "op" and op == "-":
            self.take()
            return self.arith_neg(self.expr(scope, 7), line)
        if kind == "op" and op == "!":
            self.take()
            return not self.truthy(self.expr(scope, 7))
        return self.primary(scope)

    def primary(self, scope: dict):
        kind, v, line = self.take()
        if kind == "number":
            return v
        if kind == "(":
            val = self.expr(scope)
            self.expect(")")
            return val
        if kind == "[":
            return self.list_literal(scope, line)
        if kind == "ident":
            if self.peek()[0] == "(":  # 函数调用
                self.take()
                args = []
                if self.peek()[0] != ")":
                    while True:
                        args.append(self.expr(scope))
                        if self.peek()[0] == ",":
                            self.take()
                        else:
                            break
                self.expect(")")
                return self.call_function(v, args, line)
            if v in ("true", "false"):
                return v == "true"
            if v in scope:
                return scope[v]
            raise ValueError(f"CGS 第{line}行: 未定义变量 {v!r}")
        raise ValueError(f"CGS 第{line}行: 非法表达式起点 {v!r}")

    def list_literal(self, scope: dict, line: int) -> list:
        """[a, b, ...] 或 range [初:止(:步)] (止包含, 与 OpenSCAD 一致)。"""
        first = self.expr(scope)
        if self.peek()[0] == "op" and self.peek()[1] == ":":
            # range: [初:止] 或 [初:步:止] (OpenSCAD 顺序, 止包含)
            self.take()
            second = self.expr(scope)
            step_v = 1.0
            if self.peek()[0] == "op" and self.peek()[1] == ":":
                self.take()
                third = self.expr(scope)
                start_v, step_v, stop_v = first, second, third
            else:
                start_v, stop_v = first, second
            self.expect("]")
            start = self.num(start_v, line, "range 起点")
            stop_f = self.num(stop_v, line, "range 止")
            step_f = self.num(step_v, line, "range 步长")
            if step_f == 0:
                raise ValueError(f"CGS 第{line}行: range 步长不能为 0")
            out = []
            v = start
            if step_f > 0:
                while v <= stop_f + 1e-12:
                    out.append(v)
                    v += step_f
            else:
                while v >= stop_f - 1e-12:
                    out.append(v)
                    v += step_f
            if len(out) > 1_000_000:
                raise ValueError(f"CGS 第{line}行: range 过长")
            return out
        items = [first]
        while self.peek()[0] == ",":
            self.take()
            items.append(self.expr(scope))
        self.expect("]")
        return items

    def call_function(self, name: str, args: list, line: int):
        if name == "len":
            return float(len(args[0]))
        if name == "norm":
            return math.sqrt(sum(self.num(x, line, "norm") ** 2 for x in args[0]))
        if name == "cross":
            a, b = args
            return [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]
        fn = self.FUNCTIONS.get(name)
        if fn is None:
            raise ValueError(f"CGS 第{line}行: 未知函数 {name!r}")
        flat = []
        for a in args:
            flat.extend(a) if isinstance(a, list) else flat.append(a)
        try:
            return float(fn(*flat))
        except (TypeError, ValueError) as e:
            raise ValueError(f"CGS 第{line}行: {name} 参数错误: {e}") from e

    # ── 运算语义 (数字 + 向量逐元素, 标量广播) ─────────────────────

    @staticmethod
    def truthy(v) -> bool:
        if isinstance(v, list):
            return len(v) > 0
        return bool(v)

    @staticmethod
    def arith_neg(v, line: int):
        if isinstance(v, list):
            return [-x for x in v]
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            return -v
        raise ValueError(f"CGS 第{line}行: 一元负号需要数字/向量")

    def binop(self, op: str, a, b):
        if op in ("==", "!="):
            eq = a == b
            return eq if op == "==" else not eq
        if op in ("&&", "||"):
            return (
                self.truthy(a) and self.truthy(b)
                if op == "&&"
                else self.truthy(a) or self.truthy(b)
            )
        if op in ("<", "<=", ">", ">="):
            return {"<": a < b, "<=": a <= b, ">": a > b, ">=": a >= b}[op]
        # 算术: 数字或向量逐元素 (标量广播)
        if isinstance(a, list) or isinstance(b, list):
            la = a if isinstance(a, list) else [a] * len(b)
            lb = b if isinstance(b, list) else [b] * len(a)
            if len(la) != len(lb):
                raise ValueError(f"CGS: 向量长度不匹配 ({len(la)} vs {len(lb)})")
            return [self.scalar_arith(op, x, y) for x, y in zip(la, lb)]
        return self.scalar_arith(op, a, b)

    @staticmethod
    def scalar_arith(op: str, a, b):
        if op == "+":
            return a + b
        if op == "-":
            return a - b
        if op == "*":
            return a * b
        if op == "/":
            return a / b
        return a % b

    # ── 语句 ──────────────────────────────────────────────────────

    def statement(self, motor: Motor, mat: dict, scope: dict) -> None:
        kind, name, line = self.peek()
        if kind == "{":  # 块语句 (if/for 分支可以直接是 {} 块)
            self.expect("{")
            while self.peek()[0] != "}":
                self.statement(motor, mat, scope)
            self.expect("}")
            return None
        if kind == "ident":
            if name == "module":
                return self.module_def()
            if name == "for":
                return self.for_loop(motor, mat, scope)
            if name == "if":
                return self.if_stmt(motor, mat, scope)
            if name == "echo":
                return self.echo_stmt(scope)
            if name == "union":
                # OpenSCAD 习惯写法; 本引擎无布尔, union = 分组别名
                self.take()
                self.expect("(")
                self.expect(")")
                return self.body(motor, mat, scope, line)
            if self.peek(1)[0] == "=":  # 赋值
                self.take()
                self.take()
                scope[name] = self.expr(scope)
                self.expect(";")
                return None
        # 调用语句 (图元/修饰符/灯光/场景设置/module 调用)
        self.take()
        if kind != "ident" or not isinstance(name, str):
            raise ValueError(f"CGS 第{line}行: 期望语句名, 得到 {name!r}")
        pos_args, kw_args = self.call_args(scope)
        if name in self.modules:
            return self.module_call(name, pos_args, kw_args, motor, mat, line)
        if name not in self.SIGNATURES:
            raise ValueError(f"CGS 第{line}行: 未知语句 {name!r}")
        args = self.resolve(name, pos_args, kw_args, line)
        if name == "translate":
            m = motor.gp(Motor.translator(self.vec3(args["t"], line, "translate.t")))
            return self.body(m, mat, scope, line)
        if name == "rotate":
            m = motor.gp(
                Motor.rotor(
                    self.vec3(args["axis"], line, "rotate.axis"),
                    self.num(args["angle"], line, "rotate.angle"),
                )
            )
            return self.body(m, mat, scope, line)
        if name == "material":
            merged = {**mat, **{k: v for k, v in args.items() if v is not None}}
            return self.body(motor, merged, scope, line)
        if name == "background":
            self.expect(";")
            self.scene.background = Color(int(self.num(args["color"], line, "color")))
            return None
        if name == "camera":
            self.expect(";")
            self.camera = PerspectiveCamera(
                fov=self.num(args["fov"], line, "fov"),
                aspect=self.num(args["aspect"], line, "aspect"),
                position=self.vec3(args["position"], line, "camera.position"),
                target=self.vec3(args["target"], line, "camera.target"),
            )
            self.camera.look_at(self.vec3(args["target"], line, "camera.target"))
            return None
        if name.endswith("_light"):
            self.expect(";")
            self.add_light(name, args, line)
            return None
        # 图元
        self.expect(";")
        geo = self.build_geometry(name, args, line)
        self.scene.add(Mesh(geo, self.build_material(mat), motor=motor))
        return None

    def body(self, motor: Motor, mat: dict, scope: dict, line: int) -> None:
        """修饰符/控制流目标: 下一条语句或 {} 块。"""
        if self.peek()[0] == "{":
            self.expect("{")
            while self.peek()[0] != "}":
                self.statement(motor, mat, scope)
            self.expect("}")
        elif self.peek()[0] == ";":
            raise ValueError(f"CGS 第{line}行: 修饰语句缺少目标语句")
        else:
            self.statement(motor, mat, scope)

    # ── 控制流 ────────────────────────────────────────────────────

    def if_stmt(self, motor: Motor, mat: dict, scope: dict) -> None:
        self.take()  # if
        self.expect("(")
        cond = self.truthy(self.expr(scope))
        self.expect(")")
        if cond:
            self.statement(motor, mat, scope)
            if self.peek()[:2] == ("ident", "else"):
                self.take()
                self.skip_statement()
        else:
            self.skip_statement()
            if self.peek()[:2] == ("ident", "else"):
                self.take()
                self.statement(motor, mat, scope)

    def for_loop(self, motor: Motor, mat: dict, scope: dict) -> None:
        self.take()  # for
        self.expect("(")
        kind, var, line = self.take()
        if kind != "ident" or self.peek()[0] != "=":
            raise ValueError(f"CGS 第{line}行: for 需要 (变量 = 列表)")
        self.take()
        values = self.expr(scope)
        self.expect(")")
        if not isinstance(values, list):
            raise ValueError(f"CGS 第{line}行: for 需要列表, 得到 {values!r}")
        body = self.capture_statement()
        for v in values:
            scope[var] = v
            self.run_tokens(body, motor, mat, scope)

    def echo_stmt(self, scope: dict) -> None:
        self.take()  # echo
        self.expect("(")
        vals = []
        if self.peek()[0] != ")":
            while True:
                vals.append(self.expr(scope))
                if self.peek()[0] == ",":
                    self.take()
                else:
                    break
        self.expect(")")
        self.expect(";")
        print("ECHO:", *vals)

    # ── module (参数化子树) ────────────────────────────────────────

    def module_def(self) -> None:
        """module 名(形参[=默认], ...) { 体 } —— 体捕获为 token 区间。"""
        self.take()  # module
        kind, name, line = self.take()
        if kind != "ident":
            raise ValueError(f"CGS 第{line}行: module 缺名字")
        self.expect("(")
        params: list[tuple[str, list | None]] = []
        if self.peek()[0] != ")":
            while True:
                k, pname, pline = self.take()
                if k != "ident":
                    raise ValueError(f"CGS 第{pline}行: module 形参需为名字")
                default = None
                if self.peek()[0] == "=":
                    self.take()
                    default = self.capture_expr_tokens(pline)
                params.append((pname, default))
                if self.peek()[0] == ",":
                    self.take()
                else:
                    break
        self.expect(")")
        self.expect("{")
        start = self.pos
        depth = 1
        while depth > 0:
            k = self.take()[0]
            if k == "{":
                depth += 1
            elif k == "}":
                depth -= 1
        self.modules[name] = (params, self.toks[start : self.pos - 1])

    def module_call(
        self,
        name: str,
        pos_args: list,
        kw_args: dict,
        motor: Motor,
        mat: dict,
        line: int,
    ) -> None:
        self.expect(";")
        params, body = self.modules[name]
        if len(pos_args) > len(params):
            raise ValueError(f"CGS 第{line}行: module {name} 参数过多")
        scope = {"pi": math.pi}  # 词法: 全局 + 形参, 看不到调用点局部
        for (pname, _default), v in zip(params, pos_args):
            scope[pname] = v
        for k, v in kw_args.items():
            if k not in [p for p, _ in params]:
                raise ValueError(f"CGS 第{line}行: module {name} 无形参 {k!r}")
            scope[k] = v
        for pname, default in params:
            if pname not in scope:
                if default is None:
                    raise ValueError(f"CGS 第{line}行: module {name} 缺参数 {pname!r}")
                scope[pname] = self.eval_tokens(default, scope)
        self.run_tokens(body, motor, mat, scope)

    # ── token 捕获与重放 ───────────────────────────────────────────

    def capture_expr_tokens(self, line: int) -> list:
        """捕获到 , ) ; 为止 (深度 0) 的表达式 token 区间。"""
        start = self.pos
        depth = 0
        while True:
            kind, _v, _ln = self.peek()
            if kind == "eof":
                raise ValueError(f"CGS 第{line}行: 表达式未闭合")
            if depth == 0 and kind in (",", ")", ";"):
                break
            if kind in ("(", "["):
                depth += 1
            elif kind in (")", "]"):
                depth -= 1
            self.take()
        return self.toks[start : self.pos]

    def capture_statement(self) -> list:
        """捕获"一条语句"的 token 区间 (修饰链/块/控制流都算一条)。"""
        start = self.pos
        self.skip_statement()
        return self.toks[start : self.pos]

    def skip_statement(self) -> None:
        """跳过一条语句 (不求值) —— 与 statement 同构。"""
        kind, name, _line = self.peek()
        if kind == "{":
            self.take()
            depth = 1
            while depth > 0:
                k = self.take()[0]
                if k == "{":
                    depth += 1
                elif k == "}":
                    depth -= 1
            return
        if kind == "ident" and name == "if":
            self.take()
            self.skip_parens()
            self.skip_statement()
            if self.peek()[:2] == ("ident", "else"):
                self.take()
                self.skip_statement()
            return
        if kind == "ident" and name in ("for", "union", "echo"):
            self.take()
            self.skip_parens()
            if name == "echo":
                self.expect(";")
            else:
                self.skip_statement()
            return
        if kind == "ident" and name == "module":
            self.take()
            self.take()  # 名字
            self.skip_parens()
            self.skip_statement()  # { } 块
            return
        # 调用或赋值: 吃掉到 ; 为止 (深度 0), 但修饰链需要先看有没有块
        self.take()
        if self.peek()[0] == "=":
            # 赋值: 表达式到 ;
            while self.take()[0] != ";":
                pass
            return
        if self.peek()[0] == "(":
            self.skip_parens()
            nxt = self.peek()[0]
            if nxt == "{":
                self.skip_statement()
            elif nxt == ";":
                self.take()
            else:
                self.skip_statement()  # 修饰链
            return
        while self.take()[0] != ";":
            pass

    def skip_parens(self) -> None:
        self.expect("(")
        depth = 1
        while depth > 0:
            k = self.take()[0]
            if k == "(":
                depth += 1
            elif k == ")":
                depth -= 1

    def eval_tokens(self, toks: list, scope: dict):
        """在独立 token 区间求值一个表达式 (module 默认值用)。"""
        saved_toks, saved_pos = self.toks, self.pos
        self.toks, self.pos = toks, 0
        val = self.expr(scope)
        self.toks, self.pos = saved_toks, saved_pos
        return val

    # ── 参数归一与求值 ─────────────────────────────────────────────

    def call_args(self, scope: dict) -> tuple[list, dict]:
        """'(' 后的调用参数 → (按位值列表, 命名值 dict), 表达式求值。"""
        self.expect("(")
        pos, kw = [], {}
        if self.peek()[0] != ")":
            while True:
                kind, v, _line = self.peek()
                if kind == "ident" and self.peek(1)[0] == "=":
                    self.take()
                    self.take()
                    kw[v] = self.expr(scope)
                else:
                    pos.append(self.expr(scope))
                if self.peek()[0] == ",":
                    self.take()
                else:
                    break
        self.expect(")")
        return pos, kw

    def resolve(self, name: str, pos: list, kw: dict, line: int) -> dict:
        """按 SIGNATURES 归一化参数 (按位映射 + 默认值 + 未知参数检查)。"""
        names, defaults = self.SIGNATURES[name]
        if len(pos) > len(names):
            raise ValueError(f"CGS 第{line}行: {name} 按位参数过多 (≤{len(names)})")
        merged = dict(defaults)
        given = set(names[: len(pos)])
        for pname, v in zip(names, pos):
            merged[pname] = v
        for k, v in kw.items():
            if k not in names and k not in defaults:
                raise ValueError(f"CGS 第{line}行: {name} 无参数 {k!r}")
            if k in given:
                raise ValueError(f"CGS 第{line}行: {name} 参数 {k!r} 重复")
            merged[k] = v
        missing = [pname for pname in names if merged.get(pname) is None]
        if missing:
            raise ValueError(f"CGS 第{line}行: {name} 缺少参数 {missing}")
        return merged

    @staticmethod
    def num(v, line: int, what: str) -> float:
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ValueError(f"CGS 第{line}行: {what} 需要数字, 得到 {v!r}")
        return float(v)

    @staticmethod
    def vec3(v, line: int, what: str) -> tuple[float, float, float]:
        if not isinstance(v, list) or len(v) != 3:
            raise ValueError(f"CGS 第{line}行: {what} 需要 [x,y,z], 得到 {v!r}")
        vals = [SceneLoader.num(x, line, what) for x in v]
        return (vals[0], vals[1], vals[2])

    # ── 构造 ──────────────────────────────────────────────────────

    def build_geometry(self, name: str, args: dict, line: int):
        if name == "sphere":
            return SphereGeometry(self.num(args["r"], line, "sphere.r"))
        if name == "plane":
            return PlaneGeometry(
                self.vec3(args["n"], line, "plane.n"),
                self.num(args["d"], line, "plane.d"),
            )
        if name == "cylinder":
            r = self.num(args["r"], line, "cylinder.r")
            h = args["h"]
            return CylinderGeometry(
                r, length=None if h is None else self.num(h, line, "cylinder.h")
            )
        if name == "box":
            return BoxGeometry(*self.vec3(args["s"], line, "box.s"))
        if name == "circle":
            return CircleGeometry(self.num(args["r"], line, "circle.r"))
        raise ValueError(f"CGS 第{line}行: 未知图元 {name!r}")

    @staticmethod
    def build_material(mat: dict):
        """材质字段 dict → Material (unlit=true → Basic, 否则 Standard)。"""
        color = mat.get("color", 0xFFFFFF)
        if mat.get("unlit"):
            opacity = mat.get("opacity")
            return MeshBasicMaterial(
                Color(color), opacity=1.0 if opacity is None else opacity
            )
        kw = {
            k: mat[k]
            for k in ("roughness", "metalness", "opacity", "ior", "absorption")
            if mat.get(k) is not None
        }
        if mat.get("emissive") is not None:
            kw["emissive"] = Color(mat["emissive"])
        return MeshStandardMaterial(Color(color), **kw)

    def add_light(self, name: str, args: dict, line: int) -> None:
        color = Color(int(self.num(args["color"], line, "color")))
        intensity = self.num(args["intensity"], line, "intensity")
        if name == "directional_light":
            d = self.vec3(args["direction"], line, "direction")
            self.scene.add(DirectionalLight(color, intensity, d))
        elif name == "point_light":
            p = self.vec3(args["position"], line, "position")
            self.scene.add(PointLight(color, intensity, p))
        else:
            self.scene.add(AmbientLight(color, intensity))
