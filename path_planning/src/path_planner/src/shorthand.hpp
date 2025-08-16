#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>

using namespace std_msgs::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

Point new_point(double x, double y, double z);
Vector3 Vec3(double x, double y, double z);
Quaternion Quat(double x, double y, double z, double w);
Pose new_pose(Point position, Quaternion orientation);
PoseStamped new_pose_stamped(Header header, Pose pose);
Path new_path(Header header, std::vector<PoseStamped> poses);
