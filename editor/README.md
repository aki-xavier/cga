# CGS Editor (V port)

A web-based live-preview editor for the CGS scene language — the V replacement
for the Rust `gpui` editor (`editor/src/*.rs`, removed).

## Run

```sh
make editor              # or: v -gc boehm run editor/
# open http://127.0.0.1:8123
```

The server:

- `GET  /` — serves `web/index.html` (the editor UI: textarea + preview).
- `POST /render?w=&h=&aa=` — body = CGS text → PNG.  CGS parse/eval errors and
  invalid render args return HTTP 400 with a readable message; the page shows
  it in a red overlay over the preview (keeping the last good render) and
  clears it on the next successful render.
- `GET  /health` — `ok`.

## Layout

- `server.v` — HTTP server (`net.http`) + CGS→PNG rendering via `cga`.
- `highlight.v` — CGS syntax-highlighting lexer (port of `editor/src/highlight.rs`).
- `web/index.html` — the editor frontend (highlighting, error overlay, debounced render).
- `highlight_test.v` — tests for the highlighter.

## Tests

```sh
cd editor && v -gc boehm test .
```
