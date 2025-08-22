#include "shorthand.hpp"
#include "utils.hpp"
#include <cmath>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <vector>

using namespace geometry_msgs::msg;

// Linear blend of points A,B over parameter interval [ta, tb] at t
inline Vector3 lerp_interval(const Vector3 &A, const Vector3 &B, double ta,
                             double tb, double t) {
  const double denom = (tb - ta);
  if (std::abs(denom) < 1e-9)
    return A; // guard coincident knots
  const double wa = (tb - t) / denom;
  const double wb = (t - ta) / denom;
  return add(scale(A, wa), scale(B, wb));
}

// Translated from
// https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline#Code_example_in_Unreal_C++
// ---------------------------------
inline float get_t(float t, float alpha, const Vector3 &p0, const Vector3 &p1) {
  // ||p1 - p0||^alpha; using squared length to avoid an extra sqrt where
  // possible
  const Vector3 d = sub(p1, p0);
  const double a = dot(d, d); // = ||d||^2
  const double b =
      std::pow(a, 0.5 * alpha); // = (||d||^2)^(alpha/2) = ||d||^alpha
  return static_cast<float>(t + b);
}

inline Vector3 catmull_rom(const Vector3 &p0, const Vector3 &p1,
                           const Vector3 &p2, const Vector3 &p3,
                           float t /* 0..1 */, float alpha = 0.5f /* 0..1 */) {
  const float t0 = 0.0f;
  const float t1 = get_t(t0, alpha, p0, p1);
  const float t2 = get_t(t1, alpha, p1, p2);
  const float t3 = get_t(t2, alpha, p2, p3);

  // Map local t∈[0,1] to non-uniform parameter range [t1, t2]
  const double tt = static_cast<double>(t1 + (t2 - t1) * t);

  // de Casteljau-style evaluation with non-uniform knots
  const Vector3 A1 = lerp_interval(p0, p1, t0, t1, tt);
  const Vector3 A2 = lerp_interval(p1, p2, t1, t2, tt);
  const Vector3 A3 = lerp_interval(p2, p3, t2, t3, tt);

  const Vector3 B1 = lerp_interval(A1, A2, t0, t2, tt);
  const Vector3 B2 = lerp_interval(A2, A3, t1, t3, tt);

  const Vector3 C = lerp_interval(B1, B2, t1, t2, tt);
  return C;
}

// -----------------------

std::vector<Point> sample_segment_catmull_rom(const Point &p0, const Point &p1,
                                              const Point &p2, const Point &p3,
                                              int samples) {
  samples = std::max(samples, 2); // At least 2
  std::vector<Point> res;
  res.reserve(samples);
  for (int k = 0; k < samples; ++k) {
    float t = static_cast<float>(k) / static_cast<float>(samples - 1); // 0..1
    res.push_back(into_point(catmull_rom(into_vec(p0), into_vec(p1),
                                         into_vec(p2), into_vec(p3), t)));
  }
  return res;
}

std::vector<Point> sample_catmull_rom(const std::vector<Point> points) {
  std::vector<Point> res;

  const ulong size = points.size();
  for (ulong i = 0; i < size; i++) {
    Point p0 = points[i];
    Point p1 = points[(i + 1) % size];
    Point p2 = points[(i + 2) % size];
    Point p3 = points[(i + 3) % size];

    std::vector<Point> samples = sample_segment_catmull_rom(p0, p1, p2, p3, 10);
    res.insert(res.end(), samples.begin(), samples.end());
  }

  return res;
}
