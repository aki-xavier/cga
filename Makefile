# V port build helper.
#
# The V module resolves its `mlx` dependency (and its own `cga` module) through
# the project-local `.vmodules` directory, so every build sets VMODULES.

VMODULES := $(CURDIR)/.vmodules

.PHONY: test run editor fmt

# `-no-memory-limit` because `v test .` compiles the whole cga + mlx modules in
# parallel (the v3 compiler's default 2.3 GiB guard trips on the generated GP
# table + 18 test files).
test:
	VMODULES=$(VMODULES) v -no-memory-limit test .

# `-gc boehm` avoids V 0.5.2's default `boehm_full_opt` GC, whose generated
# closure code fails to compile on macOS (emitting a spurious "C compiler bug
# report" and a fallback rebuild).
run:
	VMODULES=$(VMODULES) v -gc boehm run examples/render_smoke.v

# The CGS editor web server (renders .cgs -> PNG at http://127.0.0.1:8123).
editor:
	VMODULES=$(VMODULES) v -gc boehm run editor/

fmt:
	v fmt -w .
