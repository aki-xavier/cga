# CGS Editor (V port)

A web-based live-preview editor for the CGS scene language — the V replacement
for the Rust `gpui` editor (`editor/src/*.rs`, removed).

## Run

```sh
make editor              # or: v -gc boehm run editor/
# open http://127.0.0.1:8123
```

The server:

- `GET  /` — serves `web/index.html` (the editor UI: textarea + preview + sliders).
- `POST /render?w=&h=&aa=` — body = CGS text → PNG (renders splat layers too,
  via `render_scene_with_splats`).
- `POST /params` — body = CGS text → JSON of draggable numeric parameters.
- `GET  /health` — `ok`.

## Layout

- `server.v` — HTTP server (`net.http`) + CGS→PNG rendering via `cga`.
- `params.v` — numeric-parameter extraction (port of `editor/src/params.rs`).
- `highlight.v` — CGS syntax-highlighting lexer (port of `editor/src/highlight.rs`).
- `web/index.html` — the editor frontend (highlighting, sliders, debounced render).
- `*_test.v` — tests for params + highlight.

## Tests

```sh
cd editor && v -gc boehm test .
```
