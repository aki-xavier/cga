# V port build helper.
#
# The V module resolves its `mlx` dependency (and its own `cga` module) through
# the project-local `.vmodules` directory, so every build sets VMODULES.

VMODULES := $(CURDIR)/.vmodules

.PHONY: test run fmt

test:
	VMODULES=$(VMODULES) v test .

run:
	VMODULES=$(VMODULES) v run examples/render_smoke.v

fmt:
	v fmt -w .
