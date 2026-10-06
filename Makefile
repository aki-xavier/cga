
.PHONY: test run fmt vendor-react

test:
	cargo test --workspace

run:
	cargo run --release -p cga-examples --bin render_jsx -- examples/jsx/orbit.jsx render_smoke.png 640 480 2

fmt:
	cargo fmt --all

# 重新生成内置 React 运行时（crates/cga-host/assets/react-runtime.*.js）
vendor-react:
	node scripts/vendor-react.mjs
