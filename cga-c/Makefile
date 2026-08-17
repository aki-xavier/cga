# ccga — C port of the cga project (build for the core algebra + engine).
#
# The core algebra (mv.c, algebra.c) is pure C. The renderer links the MLX C
# API (mlx-c) for GPU-accelerated per-pixel ray tracing.

CC ?= clang
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Iinclude
AR ?= ar

# mlx-c (installed via Homebrew)
MLXC_PREFIX ?= $(shell brew --prefix mlx-c 2>/dev/null || echo /opt/homebrew/opt/mlx-c)
MLXC_INC ?= $(MLXC_PREFIX)/include
MLXC_LIB ?= $(MLXC_PREFIX)/lib
MLXCFLAGS = -I$(MLXC_INC) -L$(MLXC_LIB) -lmlxc

CORE_SRC := src/mv.c src/algebra.c
CORE_OBJ := $(CORE_SRC:.c=.o)
RENDER_SRC := src/render.c
RENDER_OBJ := $(RENDER_SRC:.c=.o)
LIB := build/libccga.a

.PHONY: all clean test render-test smoke

all: $(LIB)

build:
	mkdir -p build

$(LIB): $(CORE_OBJ) | build
	$(AR) rcs $@ $(CORE_OBJ)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(RENDER_OBJ): $(RENDER_SRC) include/ccga/render.h
	$(CC) $(CFLAGS) -I$(MLXC_INC) -c $< -o $@

# Core algebra self-test (no mlx-c dependency).
test: $(LIB) tools/test_algebra.c
	$(CC) $(CFLAGS) tools/test_algebra.c $(LIB) -lm -o build/test_algebra
	./build/test_algebra

# mlx-c linkage smoke test.
smoke: tools/mlx_smoke.c
	$(CC) $(CFLAGS) -I$(MLXC_INC) tools/mlx_smoke.c -L$(MLXC_LIB) -lmlxc -o build/mlx_smoke
	./build/mlx_smoke

# Minimal sphere renderer end-to-end test.
render-test: $(LIB) $(RENDER_OBJ) tools/test_render.c
	$(CC) $(CFLAGS) -I$(MLXC_INC) tools/test_render.c $(LIB) $(RENDER_OBJ) -L$(MLXC_LIB) -lmlxc -lm -o build/test_render
	./build/test_render

clean:
	rm -rf build src/*.o
