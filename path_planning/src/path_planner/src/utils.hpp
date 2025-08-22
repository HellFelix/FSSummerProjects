#include "shorthand.hpp"
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
using namespace geometry_msgs::msg;

Quaternion rotation(const Point p1, const Point p2, const Point p3);

double length(const Vector3 v);
Vector3 normalize(const Vector3 v);

Vector3 into_vec(const Point input);
Point into_point(const Vector3 input);

inline Vector3 scale(const Vector3 &a, double s) {
  return Vec3(a.x * s, a.y * s, a.z * s);
}
// v1+v2
inline Vector3 add(const Vector3 &v1, const Vector3 &v2) {
  return Vec3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}
// v1-v2
inline Vector3 sub(const Vector3 v1, const Vector3 v2) {
  return Vec3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}
inline double dot(const Vector3 v1, const Vector3 v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}
