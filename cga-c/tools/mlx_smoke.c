/* mlx-c smoke test: verify linking + array ops + readback on CPU and GPU. */
#include <stdio.h>

#include "mlx/c/mlx.h"

static int run_on(mlx_stream s, const char *label) {
  float a_data[4] = {1.0f, 2.0f, 3.0f, 4.0f};
  float b_data[4] = {10.0f, 20.0f, 30.0f, 40.0f};
  int shape[1] = {4};

  mlx_array a = mlx_array_new_data(a_data, shape, 1, MLX_FLOAT32);
  mlx_array b = mlx_array_new_data(b_data, shape, 1, MLX_FLOAT32);
  mlx_array sum = mlx_array_new();

  mlx_add(&sum, a, b, s);
  mlx_array_eval(sum);

  const float *out = mlx_array_data_float32(sum);
  if (out == NULL) {
    printf("[%s] FAIL: no data after eval\n", label);
    mlx_array_free(a);
    mlx_array_free(b);
    mlx_array_free(sum);
    return 1;
  }
  int ok = (out[0] == 11.0f && out[1] == 22.0f && out[2] == 33.0f && out[3] == 44.0f);
  printf("[%s] sum = [%g %g %g %g] -> %s\n", label, out[0], out[1], out[2], out[3],
         ok ? "OK" : "FAIL");

  mlx_array_free(a);
  mlx_array_free(b);
  mlx_array_free(sum);
  return ok ? 0 : 1;
}

int main(void) {
  bool cpu_avail = false, gpu_avail = false;
  mlx_device cpu = mlx_device_new_type(MLX_CPU, 0);
  mlx_device gpu = mlx_device_new_type(MLX_GPU, 0);
  mlx_device_is_available(&cpu_avail, cpu);
  mlx_device_is_available(&gpu_avail, gpu);
  printf("CPU available: %d, GPU available: %d\n", cpu_avail, gpu_avail);

  int rc = 0;
  if (cpu_avail) {
    mlx_stream cs = mlx_default_cpu_stream_new();
    rc |= run_on(cs, "CPU");
    mlx_stream_free(cs);
  }
  if (gpu_avail) {
    mlx_stream gs = mlx_default_gpu_stream_new();
    rc |= run_on(gs, "GPU");
    mlx_stream_free(gs);
  }

  mlx_device_free(cpu);
  mlx_device_free(gpu);
  printf(rc == 0 ? "mlx-c OK\n" : "mlx-c FAIL\n");
  return rc;
}
