#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>

#include "geometry.h"
#include "model.h"
#include "tgaimage.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
Model *model = NULL;
const int width = 800;
const int height = 800;

const int depth = 255;

Vec3f m2v(Matrix m) {
  return Vec3f(m[0][0] / m[3][0], m[1][0] / m[3][0], m[2][0] / m[3][0]);
}

Matrix v2m(Vec3f v) {
  Matrix m(4, 1);
  m[0][0] = v.x;
  m[1][0] = v.y;
  m[2][0] = v.z;
  m[3][0] = 1.f;
  return m;
}

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
  bool steep = false;
  if (std::abs(x0 - x1) < std::abs(y0 - y1)) {
    std::swap(x0, y0);
    std::swap(x1, y1);
    steep = true;
  }
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  for (int x = x0; x <= x1; x++) {
    float t = (x - x0) / (float)(x1 - x0);
    int y = y0 * (1. - t) + y1 * t;
    if (steep) {
      image.set(y, x, color);
    } else {
      image.set(x, y, color);
    }
  }
}

Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P) {
  Vec3f s[2];
  s[0].x = C.x - A.x;
  s[0].y = B.x - A.x;
  s[0].z = A.x - P.x;

  s[1].x = C.y - A.y;
  s[1].y = B.y - A.y;
  s[1].z = A.y - P.y;

  Vec3f u = s[0] ^ s[1];
  if (std::abs(u.z) > 1e-2)  // dont forget that u[2] is integer. If it is zero
                             // then triangle ABC is degenerate
    return Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
  return Vec3f(-1, 1, 1);  // in this case generate negative coordinates, it
                           // will be thrown away by the rasterizator
}

void triangle(Vec3f t0, Vec3f t1, Vec3f t2, Vec2f uv0, Vec2f uv1, Vec2f uv2,
              TGAImage &image, float intensity, int *zbuffer) {
  if (t0.y == t1.y && t0.y == t2.y)
    return;  // i dont care about degenerate triangles
  if (t0.y > t1.y) {
    std::swap(t0, t1);
    std::swap(uv0, uv1);
  }
  if (t0.y > t2.y) {
    std::swap(t0, t2);
    std::swap(uv0, uv2);
  }
  if (t1.y > t2.y) {
    std::swap(t1, t2);
    std::swap(uv1, uv2);
  }

  int total_height = t2.y - t0.y;
  for (int i = 0; i < total_height; i++) {
    bool second_half = i > t1.y - t0.y || t1.y == t0.y;
    int segment_height = second_half ? t2.y - t1.y : t1.y - t0.y;
    float alpha = (float)i / total_height;
    float beta = (float)(i - (second_half ? t1.y - t0.y : 0)) /
                 segment_height;  // be careful: with above conditions no
                                  // division by zero here
    Vec3f A = t0 + Vec3f(t2 - t0) * alpha;
    Vec3f B =
        second_half ? t1 + Vec3f(t2 - t1) * beta : t0 + Vec3f(t1 - t0) * beta;
    Vec2f uvA = uv0 + (uv2 - uv0) * alpha;
    Vec2f uvB =
        second_half ? uv1 + (uv2 - uv1) * beta : uv0 + (uv1 - uv0) * beta;
    if (A.x > B.x) {
      std::swap(A, B);
      std::swap(uvA, uvB);
    }
    for (int j = A.x; j <= B.x; j++) {
      float phi = B.x == A.x ? 1. : (float)(j - A.x) / (float)(B.x - A.x);
      Vec3f P = Vec3f(A) + Vec3f(B - A) * phi;
      Vec2f uvP = uvA + (uvB - uvA) * phi;
      int idx = P.x + P.y * width;
      if (zbuffer[idx] < P.z) {
        zbuffer[idx] = P.z;
        TGAColor color = model->diffuse(uvP);
        image.set(P.x, P.y,
                  TGAColor(color.r * intensity, color.g * intensity,
                           color.b * intensity));
      }
    }
  }
}

Matrix ViewPort(int x, int y, int w, int h) {
  Matrix m = Matrix::identity(4);
  m[0][3] = x + w / 2.f;
  m[1][3] = y + h / 2.f;
  m[2][3] = depth / 2.f;

  m[0][0] = w / 2.f;
  m[1][1] = h / 2.f;
  m[2][2] = depth / 2.f;
  return m;
}

Vec3f world2screen(Vec3f v) {
  return Vec3f(int((v.x + 1.) * width / 2. + .5),
               int((v.y + 1.) * height / 2. + .5), v.z);
}

int main(int argc, char **argv) {
  if (2 == argc) {
    model = new Model(argv[1]);
  } else {
    model = new Model("../obj/african_head.obj");
  }

  Vec3f light(0, 0, -1);
  Vec3f camera(0, 0, 3);

  Matrix perspective = Matrix::identity(4);
  perspective[3][2] = -1.0f / camera.z;

  int *zbuffer = new int[width * height];
  for (int i = 0; i < width * height; i++) {
    zbuffer[i] = std::numeric_limits<int>::min();
  }

  Matrix Projection = Matrix::identity(4);
  Matrix viewPort =
      ViewPort(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
  Projection[3][2] = -1.f / camera.z;

  TGAImage image(width, height, TGAImage::RGB);
  for (int i = 0; i < model->nfaces(); i++) {
    std::vector<int> face = model->face(i);
    Vec3f screen_coords[3];
    Vec3f world_coords[3];
    for (int j = 0; j < 3; j++) {
      Vec3f v = model->vert(face[j]);
      screen_coords[j] = m2v(viewPort * Projection * v2m(v));
      world_coords[j] = v;
    }
    Vec3f n = (world_coords[2] - world_coords[0]) ^
              (world_coords[1] - world_coords[0]);
    n.normalize();
    float intensity = n * light;
    if (intensity > 0) {
      Vec2f uv[3];
      for (int k = 0; k < 3; k++) {
        uv[k] = model->uv(i, k);
      }
      triangle(screen_coords[0], screen_coords[1], screen_coords[2], uv[0],
               uv[1], uv[2], image, intensity, zbuffer);
    }
  }
  image.flip_vertically();  // i want to have the origin at the left bottom
                            // corner of the image
  image.write_tga_file("output.tga");
  delete model;
  return 0;
}
