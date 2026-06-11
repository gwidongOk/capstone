#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "gnc_types.h"
#include <math.h>

#define PI_F 3.14159265f
#define DEG2RAD_F (PI_F / 180.0f)
#define RAD2DEG_F (180.0f / PI_F)

/* --- Scalar utilities --- */

static inline float clampf(float val, float lo, float hi) {
  if (val < lo)
    return lo;
  if (val > hi)
    return hi;
  return val;
}

static inline float signf(float val) {
  if (val > 0.0f)
    return 1.0f;
  if (val < 0.0f)
    return -1.0f;
  return 0.0f;
}

static inline float wrap_pi(float angle) {
  angle = fmodf(angle + PI_F, 2.0f * PI_F);
  if (angle < 0.0f)
    angle += 2.0f * PI_F;
  return angle - PI_F;
}

/* --- Vec3 operations --- */

static inline vec3_t vec3_add(vec3_t a, vec3_t b) {
  return (vec3_t){a.x + b.x, a.y + b.y, a.z + b.z};
}

static inline vec3_t vec3_sub(vec3_t a, vec3_t b) {
  return (vec3_t){a.x - b.x, a.y - b.y, a.z - b.z};
}

static inline vec3_t vec3_scale(vec3_t v, float s) {
  return (vec3_t){v.x * s, v.y * s, v.z * s};
}

static inline float vec3_dot(vec3_t a, vec3_t b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline vec3_t vec3_cross(vec3_t a, vec3_t b) {
  return (vec3_t){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                  a.x * b.y - a.y * b.x};
}

static inline float vec3_norm(vec3_t v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static inline vec3_t vec3_normalize(vec3_t v) {
  float n = vec3_norm(v);
  if (n < 1e-10f)
    return (vec3_t){0, 0, 0};
  return vec3_scale(v, 1.0f / n);
}

static inline float vec3_norm_xy(vec3_t v) {
  return sqrtf(v.x * v.x + v.y * v.y);
}

/* --- Quaternion operations --- */

static inline quat_t quat_normalize(quat_t q) {
  float n = sqrtf(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
  if (n < 1e-10f)
    return (quat_t){1, 0, 0, 0};
  float inv_n = 1.0f / n;
  return (quat_t){q.q0 * inv_n, q.q1 * inv_n, q.q2 * inv_n, q.q3 * inv_n};
}

/* Quaternion to DCM (Body <- NED), row-major float[9] */
static inline void quat_to_dcm_bn(quat_t q, float R[9]) {
  float q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;
  R[0] = 1 - 2 * (q2 * q2 + q3 * q3);
  R[1] = 2 * (q1 * q2 + q0 * q3);
  R[2] = 2 * (q1 * q3 - q0 * q2);
  R[3] = 2 * (q1 * q2 - q0 * q3);
  R[4] = 1 - 2 * (q1 * q1 + q3 * q3);
  R[5] = 2 * (q2 * q3 + q0 * q1);
  R[6] = 2 * (q1 * q3 + q0 * q2);
  R[7] = 2 * (q2 * q3 - q0 * q1);
  R[8] = 1 - 2 * (q1 * q1 + q2 * q2);
}

/* R_BN * v (NED -> Body) */
static inline vec3_t dcm_bn_mul_vec(const float R[9], vec3_t v) {
  return (vec3_t){R[0] * v.x + R[1] * v.y + R[2] * v.z,
                  R[3] * v.x + R[4] * v.y + R[5] * v.z,
                  R[6] * v.x + R[7] * v.y + R[8] * v.z};
}

/* R_BN^T * v (Body -> NED) */
static inline vec3_t dcm_nb_mul_vec(const float R[9], vec3_t v) {
  return (vec3_t){R[0] * v.x + R[3] * v.y + R[6] * v.z,
                  R[1] * v.x + R[4] * v.y + R[7] * v.z,
                  R[2] * v.x + R[5] * v.y + R[8] * v.z};
}

/* Quaternion to Euler [phi, theta, psi] (ZYX) */
static inline void quat_to_euler_zyx(quat_t q, float *phi, float *theta,
                                     float *psi) {
  float q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;
  *phi = atan2f(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
  *theta = asinf(clampf(2 * (q0 * q2 - q3 * q1), -1.0f, 1.0f));
  *psi = atan2f(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
}

/* --- Linear interpolation --- */

static inline float interp1_linear(const float *V_bp, const float *table, int n,
                                   float V) {
  if (V <= V_bp[0])
    return table[0];
  if (V >= V_bp[n - 1])
    return table[n - 1];

  int i = 0;
  while (i < n - 2 && V_bp[i + 1] < V)
    i++;

  float frac = (V - V_bp[i]) / (V_bp[i + 1] - V_bp[i]);
  return table[i] + frac * (table[i + 1] - table[i]);
}

/**
 * @brief 2D bilinear interpolation on a row-major table.
 *
 * Given breakpoints x_bp[nx] and y_bp[ny] and table z[nx*ny] stored
 * row-major (z[i*ny + j] = f(x_bp[i], y_bp[j])), returns the
 * bilinearly interpolated value at (x, y).
 *
 * Boundary: clamp (hold edge value outside breakpoint range).
 *
 * Math:
 *   f(x,y) = (1-tx)(1-ty)*z00 + tx*(1-ty)*z10
 *          + (1-tx)*ty*z01     + tx*ty*z11
 *
 *   where tx = (x - x_bp[i])/(x_bp[i+1] - x_bp[i])
 *         ty = (y - y_bp[j])/(y_bp[j+1] - y_bp[j])
 */
static inline float interp2_bilinear(const float *x_bp, int nx,
                                     const float *y_bp, int ny, const float *z,
                                     float x, float y) {
  /* Find x index */
  int ix = 0;
  if (x <= x_bp[0]) {
    ix = 0;
    x = x_bp[0];
  } else if (x >= x_bp[nx - 1]) {
    ix = nx - 2;
    x = x_bp[nx - 1];
  } else {
    while (ix < nx - 2 && x_bp[ix + 1] < x)
      ix++;
  }

  /* Find y index */
  int iy = 0;
  if (y <= y_bp[0]) {
    iy = 0;
    y = y_bp[0];
  } else if (y >= y_bp[ny - 1]) {
    iy = ny - 2;
    y = y_bp[ny - 1];
  } else {
    while (iy < ny - 2 && y_bp[iy + 1] < y)
      iy++;
  }

  /* Fractional positions */
  float dx = x_bp[ix + 1] - x_bp[ix];
  float dy = y_bp[iy + 1] - y_bp[iy];
  float tx = (dx > 1e-12f) ? (x - x_bp[ix]) / dx : 0.0f;
  float ty = (dy > 1e-12f) ? (y - y_bp[iy]) / dy : 0.0f;

  /* Four corner values */
  float z00 = z[ix * ny + iy];
  float z10 = z[(ix + 1) * ny + iy];
  float z01 = z[ix * ny + iy + 1];
  float z11 = z[(ix + 1) * ny + iy + 1];

  /* Bilinear */
  return (1.0f - tx) * (1.0f - ty) * z00 + tx * (1.0f - ty) * z10 +
         (1.0f - tx) * ty * z01 + tx * ty * z11;
}

#endif /* MATH_UTILS_H */