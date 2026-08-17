/* Render two spheres to a PPM and print a few sample pixels. */
#include <stdio.h>
#include <stdlib.h>

#include "ccga/render.h"

static void write_ppm(const char *path, int w, int h, const float *rgb) {
  FILE *f = fopen(path, "wb");
  if (f == NULL) return;
  fprintf(f, "P6\n%d %d\n255\n", w, h);
  unsigned char *row = (unsigned char *)malloc((size_t)w * 3);
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      const float *p = &rgb[(y * w + x) * 3];
      row[x * 3 + 0] = (unsigned char)(p[0] * 255.0f + 0.5f);
      row[x * 3 + 1] = (unsigned char)(p[1] * 255.0f + 0.5f);
      row[x * 3 + 2] = (unsigned char)(p[2] * 255.0f + 0.5f);
    }
    fwrite(row, 1, (size_t)w * 3, f);
  }
  free(row);
  fclose(f);
}

int main(void) {
  const int w = 64, h = 48;
  ccga_camera cam = ccga_camera_from_fov(50.0f * 3.14159265f / 180.0f, (float)w / h, w, h);

  ccga_sphere spheres[2] = {
      {{0.0f, 0.0f, -5.0f}, 1.0f, {1.0f, 0.2f, 0.2f}},   /* red */
      {{1.6f, 0.2f, -6.0f}, 0.8f, {0.2f, 0.4f, 1.0f}},   /* blue */
  };
  float bg[3] = {0.1f, 0.15f, 0.2f};

  float *rgb = (float *)malloc((size_t)w * h * 3 * sizeof(float));
  int rc = ccga_render_spheres(&cam, spheres, 2, bg, w, h, rgb);
  printf("render rc=%d\n", rc);

  printf("center  (32,24): %.3f %.3f %.3f\n", rgb[(24 * w + 32) * 3], rgb[(24 * w + 32) * 3 + 1],
         rgb[(24 * w + 32) * 3 + 2]);
  printf("corner  (0,0)  : %.3f %.3f %.3f\n", rgb[0], rgb[1], rgb[2]);
  printf("red-hit (28,24): %.3f %.3f %.3f\n", rgb[(24 * w + 28) * 3], rgb[(24 * w + 28) * 3 + 1],
         rgb[(24 * w + 28) * 3 + 2]);

  write_ppm("/tmp/ccga_test.ppm", w, h, rgb);
  printf("wrote /tmp/ccga_test.ppm\n");
  free(rgb);
  return rc;
}
