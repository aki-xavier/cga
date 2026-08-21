# V port build helper.
#
# The `mlx` and `cga` modules resolve through V's default module path
# (`~/.vmodules`); symlink them there once and no VMODULES env var is needed:
#   ln -s ~/code/mlx-v ~/.vmodules/mlx
#   ln -s "$(pwd)"     ~/.vmodules/cga

.PHONY: test run editor fmt

# `-no-memory-limit` because `v test .` compiles the whole cga + mlx modules in
# parallel (the v3 compiler's default 2.3 GiB guard trips on the generated GP
# table + 23 test files).
test:
	v -gc boehm -no-memory-limit test .

# `-gc boehm` avoids V 0.5.2's default `boehm_full_opt` GC, whose generated
# closure code fails to compile on macOS (emitting a spurious "C compiler bug
# report" and a fallback rebuild).
run:
	v -gc boehm run examples/render_smoke.v

# The CGS editor web server (renders .cgs -> PNG at http://127.0.0.1:8123).
editor:
	v -gc boehm run editor/

fmt:
	v fmt -w .
