#include <cmath>
#include <cstddef>
#include <functional>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/detail/color_rgba__struct.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <vector>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

#include "rclcpp/rclcpp.hpp"
#include "shorthand.hpp"
#include "utils.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;

using namespace geometry_msgs::msg;
using namespace visualization_msgs::msg;
using namespace nav_msgs::msg;
using namespace std_msgs::msg;

Path into_path(const std::vector<Point> &points) {

  auto poses = std::vector<PoseStamped>();

  for (ulong i = 0; i < points.size(); i++) {
    Point prev = (i == 0) ? points.back() : points.at(i - 1);
    Point current = points.at(i);
    Point next = (i == points.size() - 1) ? points.at(0) : points.at(i + 1);

    Quaternion rot = rotation(prev, current, next);
    Pose pose = new_pose(current, rot);

    Header header = Header();
    header.set__frame_id("map");
    PoseStamped stamped = new_pose_stamped(header, pose);
    poses.push_back(stamped);
  }
  Header header = Header();
  header.set__frame_id("map");
  return new_path(header, poses);
}

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("path_planner") {
    subscription_ = this->create_subscription<MarkerArray>(
        "/cones", 10,
        std::bind(&MinimalSubscriber::subscription_callback, this, _1));

    publisher_ = this->create_publisher<Path>("/path/global", 10);
  }

private:
  Point find_mid(const Marker &m1, const Marker &m2) const {
    Point p1 = m1.pose.position;
    Point p2 = m2.pose.position;

    Point res = Point();

    res.set__x((p1.x + p2.x) / 2);
    res.set__y((p1.y + p2.y) / 2);
    res.set__z((p1.z + p2.z) / 2);

    return res;
  }

  // Returns the numerical value of the Euclidian distance between two markers
  double distance(const Marker &m1, const Marker &m2) const {
    Point p1 = m1.pose.position;
    Point p2 = m2.pose.position;
    return std::sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) +
                     pow(p1.z - p2.z, 2));
  }

  // Takes a reference to the marker in question and a reference to the entire
  // MarkerArray, and finds the closest inner marker to that (presumably outer)
  // marker.
  Marker find_closest_inner(Marker &origin, const MarkerArray &arr) const {
    Marker closest_marker = Marker();
    Pose closest_pose = Pose();
    Point closest_position = Point();
    closest_position.set__x(INFINITY);
    closest_position.set__y(INFINITY);
    closest_position.set__z(INFINITY);
    closest_marker.set__pose(closest_pose);

    double closest_distance = INFINITY;

    for (const Marker &m : arr.markers) {
      if (m.color.g != 0 && m.color.r != 0 && m.color.b == 0) {
        // Yellow => Inner
        double marker_distance = distance(origin, m);
        if (marker_distance < closest_distance) {
          closest_marker = m;
          closest_distance = marker_distance;
        }
      }
    }

    return closest_marker;
  }

  void subscription_callback(const MarkerArray &msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard array of size: '%ld'",
                msg.markers.size());

    std::vector<Point> centerline_vec = std::vector<Point>();

    for (Marker m : msg.markers) {
      if (m.color.b != 0 && m.color.g == 0 && m.color.r == 0) {
        // Blue => Outer

        Marker closest_inner = find_closest_inner(m, msg);

        Point midpoint = find_mid(m, closest_inner);

        // midpoint.header.frame_id = "map";
        // midpoint.ns = "path";
        // midpoint.type = Marker::SPHERE;
        // midpoint.action = Marker::ADD;
        // midpoint.set__id(id);
        // id++;
        //
        // std_msgs::msg::ColorRGBA color = std_msgs::msg::ColorRGBA();
        // color.set__a(1);
        // color.set__r(255);
        // color.set__g(0);
        // color.set__b(0);
        // midpoint.set__color(color);
        //
        // Vector3 scale = Vector3();
        // scale.set__x(1);
        // scale.set__y(1);
        // scale.set__z(1);
        // midpoint.scale = scale;

        centerline_vec.push_back(midpoint);
      }
    }

    // Path centerline_path = ;
    // centerline.set__markers(centerline_vec);
    publisher_->publish(into_path(centerline_vec));
  }

  rclcpp::Subscription<MarkerArray>::SharedPtr subscription_;
  rclcpp::Publisher<Path>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
