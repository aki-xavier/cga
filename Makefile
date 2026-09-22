
.PHONY: test run editor fmt

test:
	cargo test --workspace

run:
	cargo run --release -p cga-examples --bin render_cgs -- examples/cgs/orbit.cgs render_smoke.png 640 480 2


editor:
	cargo run --release -p cga-editor

fmt:
	cargo fmt --all
