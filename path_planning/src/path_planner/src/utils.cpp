#include <cmath>
#include <cstdlib>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <stdexcept>

#include "shorthand.hpp"
using namespace geometry_msgs::msg;

const double eps = 1e-10;

Vector3 cross(const Vector3 v1, const Vector3 v2) {
  Vector3 res = Vector3();

  res.set__x(v1.y * v2.z - v1.z * v2.y);
  res.set__y(v1.z * v2.x - v1.x * v2.z);
  res.set__z(v1.x * v2.y - v1.y * v2.x);

  return res;
}

Vector3 into_vec(const Point input) {
  Vector3 res = Vector3();
  res.set__x(input.x);
  res.set__y(input.y);
  res.set__z(input.z);

  return res;
}

double length(const Vector3 v) {
  return std::sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

Vector3 normalize(const Vector3 v) {
  double l = length(v);
  return Vec3(v.x / l, v.y / l, v.z / l);
}

// v1-v2
Vector3 diff(const Vector3 v1, const Vector3 v2) {
  return Vec3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

double dot(const Vector3 v1, const Vector3 v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

// Difference Vector pointing from p1 to p2
Vector3 diff(const Point p1, const Point p2) {
  return diff(into_vec(p1), into_vec(p2));
}

Quaternion rotation(const Point p1, const Point p2, const Point p3) {
  Vector3 v1 = diff(p2, p1);
  Vector3 v2 = diff(p3, p2);

  Vector3 a = normalize(v1);
  Vector3 b = normalize(v2);

  if (length(v1) < eps || length(v2) < eps)
    throw std::runtime_error{"Undefined Quaternion"};

  // equals cos(theta)
  double c = dot(a, b);

  // No rotation
  if (c > 1 - eps)
    return Quat(0, 0, 0, 1);

  // 180 turn
  if (c < eps - 1) {
    Vector3 basis = (std::abs(a.x) < 0.9) ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
    Vector3 axis = normalize(cross(a, basis));
    return Quat(axis.x, axis.y, axis.z, 0);
  }

  // Shortest arc
  Vector3 v = cross(a, b);
  double s = std::sqrt((1 + c) * 2);

  return Quat(v.x / s, v.y / s, v.z / s, s / 2);
}
