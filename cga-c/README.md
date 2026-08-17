# ccga — C port of the `cga` project

C port of the 5D Conformal Geometric Algebra (CGA) project (`/Users/aki/code/cga`),
targeting Apple's **MLX C API** (`mlx-c`) as the GPU backend. Scope: full port —
algebra core + three.js-style ray-tracing engine + CGS scene language + CSG/modeling
+ mesh I/O — delivered as a C library plus a CLI that renders `.cgs` → PNG, with
**visual/structural parity** to the Python version (not bit-exact).

## Architecture

Two-layer design, mirroring where the Python actually spends its time:

1. **Core algebra in plain C** (`src/mv.c`, `src/algebra.c`) — the 32-component
   multivector, geometric product, motors (versors), and primitives. This is a
   faithful port of `cga/multivector.py` + `cga/motors.py` + `cga/algebra/*.py`.
   The geometric-product table is **generated from the authoritative Python**
   (`tools/gen_tables.py` → `src/gp_tables.h`) so the C port cannot drift from
   the reference basis ordering / product signs.

2. **Renderer via mlx-c** — per-pixel batched ray tracing. Per the engine spec,
   the rigid motor **collapses to a 4×4 `[R|t]`** (its `to_matrix()` value) and
   each blade type is conjugated in closed form; the full 32-component algebra is
   used for scene construction, then `ccga_motor_to_matrix()` feeds the GPU path.
   This avoids MLX GPU `matmul`'s bfloat16 downcast on 3×3 transforms.

## Layout

```
include/ccga/   public headers (mv.h, algebra.h, ...)
src/            implementation (mv.c, algebra.c, gp_tables.h generated, ...)
tools/          gen_tables.py (table generator), test_algebra.c + ref
vendor/         vendored single-file deps (PNG writer, ...)
Makefile        builds libccga.a + self-tests
```

## Build

```sh
make            # builds build/libccga.a
make test       # runs the algebra self-test (compare with tools/test_algebra_ref.py)
```

The algebra core is pure C (no mlx-c). The renderer links `-lmlxc` (Homebrew
`brew install mlx-c`).

## Validation

`make test` output is diffed byte-for-byte against `tools/test_algebra_ref.py`
(which runs the real Python core). Currently the gp/ip/op/reverse/dual, point/
plane/sphere/circle construction, motor rotor/translator/to_matrix, and
exp/log round-trip all match the reference exactly (float32).

## Status / roadmap

- [x] Core algebra (multivector, gp tables, ip/op, reverse/dual/meet, motors,
      primitives, cyclide) — validated vs Python.
- [ ] Engine: geometry intersection (sphere/plane/box/circle/cylinder/cone/
      torus/cyclide/trimesh), affine wrapper, CSG, materials/lights, renderer.
- [ ] Modeling: earclip + extrude/loft.
- [ ] Mesh I/O: OBJ + glTF/GLB.
- [ ] CGS scene language (lexer/parser/interpreter).
- [ ] CLI `.cgs` → PNG + SSAA + sRGB encode.
- [ ] End-to-end parity vs `examples/*.cgs`.

Reference specs from the recon pass are captured in this conversation (engine,
scene_lang, mesh_io/modeling/render); key port-order and parity gotchas are
mirrored inline in the relevant source files.
