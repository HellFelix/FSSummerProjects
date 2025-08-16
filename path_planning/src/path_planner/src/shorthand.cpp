#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <vector>

using namespace std_msgs::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

Point new_point(double x, double y, double z) {
  Point res = Point();
  res.set__x(x);
  res.set__y(y);
  res.set__z(z);
  return res;
}

Vector3 Vec3(double x, double y, double z) {
  Vector3 res = Vector3();
  res.set__x(x);
  res.set__y(y);
  res.set__z(z);
  return res;
}

Quaternion Quat(double x, double y, double z, double w) {
  Quaternion res = Quaternion();

  res.set__x(x);
  res.set__y(y);
  res.set__z(z);
  res.set__w(w);

  return res;
}

Pose new_pose(Point position, Quaternion orientation) {
  Pose res = Pose();

  res.set__position(position);
  res.set__orientation(orientation);

  return res;
}

PoseStamped new_pose_stamped(Header header, Pose pose) {
  PoseStamped res = PoseStamped();

  res.set__header(header);
  res.set__pose(pose);

  return res;
}

Path new_path(Header header, std::vector<PoseStamped> poses) {
  Path res = Path();

  res.set__header(header);
  res.set__poses(poses);

  return res;
}
