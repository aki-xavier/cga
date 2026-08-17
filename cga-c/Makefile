# ccga — C port of the cga project (build for the core algebra + engine).
#
# The core algebra (mv.c, algebra.c) is pure C. The renderer links the MLX C
# API (mlx-c) for GPU-accelerated per-pixel ray tracing.

CC ?= clang
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Iinclude
AR ?= ar

SRC := src/mv.c src/algebra.c
OBJ := $(SRC:.c=.o)
LIB := build/libccga.a

.PHONY: all clean test

all: $(LIB)

build:
	mkdir -p build

$(LIB): $(OBJ) | build
	$(AR) rcs $@ $(OBJ)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

test: $(LIB) tools/test_algebra.c
	$(CC) $(CFLAGS) tools/test_algebra.c $(LIB) -lm -o build/test_algebra
	./build/test_algebra

clean:
	rm -rf build src/*.o
