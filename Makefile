
.PHONY: test run fmt

test:
	cargo test --workspace

run:
	cargo run --release -p cga-examples --bin render_cgs -- examples/cgs/orbit.cgs render_smoke.png 640 480 2

fmt:
	cargo fmt --all
