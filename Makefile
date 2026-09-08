# V port build helper.
#
# The `mlx` and `cga` modules resolve through V's default module path
# (`~/.vmodules`); symlink them there once and no VMODULES env var is needed:
#   ln -s ~/code/mlx-v ~/.vmodules/mlx
#   ln -s "$(pwd)"     ~/.vmodules/cga

.PHONY: test run editor fmt

test:
	v -no-memory-limit test .  # gpu module (mlx) exceeds the 4032 MiB safety ceiling

run:
	v run examples/render_smoke.v

# The CGS editor web server (renders .cgs -> PNG at http://127.0.0.1:8123).
editor:
	v run editor/

fmt:
	v fmt -w .
