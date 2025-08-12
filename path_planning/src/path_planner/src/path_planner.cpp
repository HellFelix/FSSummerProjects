#include <functional>
#include <memory>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("path_planner") {
    subscription_ =
        this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/cones", 10,
            std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const visualization_msgs::msg::MarkerArray &msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard array of size: '%ld'",
                msg.markers.size());
  }
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
