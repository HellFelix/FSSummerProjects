#include <cmath>
#include <cstddef>
#include <functional>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/detail/color_rgba__struct.hpp>
#include <type_traits>
#include <vector>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;

namespace gm = geometry_msgs::msg;
namespace vm = visualization_msgs::msg;

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("path_planner") {
    subscription_ = this->create_subscription<vm::MarkerArray>(
        "/cones", 10,
        std::bind(&MinimalSubscriber::subscription_callback, this, _1));

    publisher_ = this->create_publisher<vm::MarkerArray>("/path/testing", 10);
  }

private:
  vm::Marker find_mid(const vm::Marker &m1, const vm::Marker &m2) const {
    gm::Point p1 = m1.pose.position;
    gm::Point p2 = m2.pose.position;

    gm::Point res_position = gm::Point();

    res_position.set__x((p1.x + p2.x) / 2);
    res_position.set__y((p1.y + p2.y) / 2);
    res_position.set__z((p1.z + p2.z) / 2);

    gm::Pose res_pose = gm::Pose();
    res_pose.set__position(res_position);
    vm::Marker res = vm::Marker();
    res.set__pose(res_pose);
    return res;
  }

  // Returns the numerical value of the Euclidian distance between two markers
  double distance(const vm::Marker &m1, const vm::Marker &m2) const {
    gm::Point p1 = m1.pose.position;
    gm::Point p2 = m2.pose.position;
    return std::sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) +
                     pow(p1.z - p2.z, 2));
  }

  // Takes a reference to the marker in question and a reference to the entire
  // MarkerArray, and finds the closest inner marker to that (presumably outer)
  // marker.
  vm::Marker find_closest_inner(vm::Marker &origin,
                                const vm::MarkerArray &arr) const {
    vm::Marker closest_marker = vm::Marker();
    gm::Pose closest_pose = gm::Pose();
    gm::Point closest_position = gm::Point();
    closest_position.set__x(INFINITY);
    closest_position.set__y(INFINITY);
    closest_position.set__z(INFINITY);
    closest_marker.set__pose(closest_pose);

    double closest_distance = INFINITY;

    for (const vm::Marker &m : arr.markers) {
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

  void subscription_callback(const vm::MarkerArray &msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard array of size: '%ld'",
                msg.markers.size());

    std::vector<vm::Marker> centerline_vec = std::vector<vm::Marker>();

    int id = 0;
    for (vm::Marker m : msg.markers) {
      if (m.color.b != 0 && m.color.g == 0 && m.color.r == 0) {
        // Blue => Outer

        vm::Marker closest_inner = find_closest_inner(m, msg);

        vm::Marker midpoint = find_mid(m, closest_inner);

        midpoint.header.frame_id = "map";
        midpoint.ns = "path";
        midpoint.type = vm::Marker::SPHERE;
        midpoint.action = vm::Marker::ADD;
        midpoint.set__id(id);
        id++;

        std_msgs::msg::ColorRGBA color = std_msgs::msg::ColorRGBA();
        color.set__a(1);
        color.set__r(255);
        color.set__g(0);
        color.set__b(0);
        midpoint.set__color(color);

        gm::Vector3 scale = gm::Vector3();
        scale.set__x(1);
        scale.set__y(1);
        scale.set__z(1);
        midpoint.scale = scale;

        centerline_vec.push_back(midpoint);
      }
    }

    vm::MarkerArray centerline = vm::MarkerArray();
    centerline.set__markers(centerline_vec);
    publisher_->publish(centerline);
  }

  rclcpp::Subscription<vm::MarkerArray>::SharedPtr subscription_;
  rclcpp::Publisher<vm::MarkerArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
