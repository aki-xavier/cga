# Rust port build helper.
#
# Workspace layout: crates/cga-core (pure f64 CPU algebra), crates/cga-gpu
# (mlx-rs/Metal renderer), crates/cga-editor (CGS web editor),
# crates/cga-examples (demo CLIs). Requires Rust stable + macOS Apple Silicon
# (mlx-rs builds the MLX C++ core on first build — takes a few minutes once).

.PHONY: test run editor fmt

test:
	cargo test --workspace

run:
	cargo run --release -p cga-examples --bin render_cgs -- examples/cgs/orbit.cgs render_smoke.png 640 480 2

# The CGS editor web server (renders .cgs -> PNG at http://127.0.0.1:8123).
editor:
	cargo run --release -p cga-editor

fmt:
	cargo fmt --all
