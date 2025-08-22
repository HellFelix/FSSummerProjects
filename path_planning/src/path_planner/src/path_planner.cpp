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

#include "catmull_rom_spline.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shorthand.hpp"
#include "utils.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;

using namespace geometry_msgs::msg;
using namespace visualization_msgs::msg;
using namespace nav_msgs::msg;
using namespace std_msgs::msg;

MarkerArray as_markers(std::vector<Point> points) {
  std::vector<Marker> markers;

  int id = 0;
  for (Point p : points) {
    Marker m = Marker();

    Pose pose = Pose();
    pose.set__position(p);
    m.set__pose(pose);

    m.header.frame_id = "map";
    m.ns = "path";
    m.type = Marker::SPHERE;
    m.action = Marker::ADD;
    m.set__id(id);
    id++;

    std_msgs::msg::ColorRGBA color = std_msgs::msg::ColorRGBA();
    color.set__a(1);
    color.set__r(255);
    color.set__g(0);
    color.set__b(0);
    m.set__color(color);

    Vector3 scale = Vector3();
    scale.set__x(1);
    scale.set__y(1);
    scale.set__z(1);
    m.scale = scale;

    markers.push_back(m);
  }

  MarkerArray res = MarkerArray();
  res.set__markers(markers);
  return res;
}

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("path_planner") {
    subscription_ = this->create_subscription<MarkerArray>(
        "/cones", 10,
        std::bind(&MinimalSubscriber::subscription_callback, this, _1));

    auto qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    publisher_ = this->create_publisher<Path>("/path/global", qos);
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

  Path into_path(const std::vector<Point> &points) const {
    auto poses = std::vector<PoseStamped>();

    Header header = Header();
    header.set__frame_id("map");
    header.set__stamp(this->now());

    for (ulong i = 0; i < points.size(); i++) {
      Point prev = (i == 0) ? points.back() : points[i - 1];
      Point current = points[i];
      Point next = (i == points.size() - 1) ? points[0] : points[i + 1];

      Quaternion rot = rotation(prev, current, next);

      Pose pose = new_pose(current, rot);

      PoseStamped stamped = new_pose_stamped(header, pose);
      poses.push_back(stamped);
    }

    // Close loop by repeating first pose at the end
    // I couldn't find another way of closing the loop. The path doesn't seem to
    // want to connect to itself...
    if (!poses.empty()) {
      poses.push_back(poses.front());
    }

    return new_path(header, poses);
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

        centerline_vec.push_back(midpoint);
      }
    }

    std::vector<Point> smoothed_vec = sample_catmull_rom(centerline_vec);
    publisher_->publish(this->into_path(smoothed_vec));
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
