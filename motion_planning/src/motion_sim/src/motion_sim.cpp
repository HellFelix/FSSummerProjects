#include <atomic>
#include <cmath>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <string>
#include <thread>

#include "Eigen/Dense"
#include "rclcpp/rclcpp.hpp"

#include "motion_sim/vehicle_model.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

class SimulationDynamics : public rclcpp::Node {
  public:
    explicit SimulationDynamics(const std::string &node_name,
                                bool intra_process_comms = false)
        : rclcpp::Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(
                                      intra_process_comms)) {
        // Delcare parameters
        declare_parameter<std::string>("frame_id.world",
                                       "UNSET_FRAME_ID_WORLD");
        declare_parameter<double>("steer.max", 0.35);
        declare_parameter<double>("steer.bias", 0.0);
        declare_parameter<std::string>("topic.controller.throttle",
                                       "UNSET_CONTROLLER_THROTTLE_TOPIC");
        declare_parameter<std::string>("topic.controller.steer",
                                       "UNSET_CONTROLLER_STEER_TOPIC");
        declare_parameter<std::string>("topic.sim_dynamics.scope",
                                       "UNSET_SIM_DYNAMICS_SCOPE_TOPIC");
        declare_parameter<std::string>("topic.odometry.odometry",
                                       "UNSET_ODOMETRY_ODOMETRY_TOPIC");
        declare_parameter<std::string>("model", "kbm");
        declare_parameter<std::vector<double>>("init_state", {0., 0., 0.});
        declare_parameter<int>("fps", 50);
        frame_id_ = get_parameter("frame_id.world").as_string();
        delta_max_ = get_parameter("steer.max").as_double();
        delta_bias_ = get_parameter("steer.bias").as_double();
        throttle_topic_ =
            get_parameter("topic.controller.throttle").as_string();
        steer_topic_ = get_parameter("topic.controller.steer").as_string();
        scope_topic_ = get_parameter("topic.sim_dynamics.scope").as_string();
        odometry_topic_ = get_parameter("topic.odometry.odometry").as_string();
        model_str_ = get_parameter("model").as_string();
        fps_ = get_parameter("fps").as_int();
        init_state_ = get_parameter("init_state").as_double_array();

        // Compute time step and initialize model
        dt_ = 1. / fps_;
    }

    void setup() {
        // Initialize model (done in configuration function since
        // shared_from_this will not work in constructor)
        rclcpp::Node::SharedPtr parent_node = shared_from_this();
        models_ = {{"kbm", std::make_shared<KinematicBicycleModel>(
                               parent_node, delta_max_, delta_bias_)},
                   {"hbm", std::make_shared<HybridBicycleModel>(
                               parent_node, delta_max_, delta_bias_)},
                   {"dbm", std::make_shared<DynamicBicycleModel>(
                               parent_node, delta_max_, delta_bias_)}};
        if (models_.find(model_str_) != models_.end()) {
            model_ = models_[model_str_];
        } else {
            model_ = models_["kinematic_bicycle"];
            RCLCPP_WARN(get_logger(),
                        "Invalid dynamic model '%s' specified, falling back to "
                        "kinematic bicycle model",
                        model_str_.c_str());
        }
        model_->initialize();
        model_->set_state(init_state_);

        // Initialize publishers, subscribers & Timers
        reset_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/reset", qos_,
            std::bind(&SimulationDynamics::reset_callback, this,
                      std::placeholders::_1));
        odometry_pub_ =
            create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, qos_);
        throttle_sub_ = create_subscription<std_msgs::msg::Float32>(
            throttle_topic_, qos_,
            std::bind(&SimulationDynamics::throttle_callback, this,
                      std::placeholders::_1));
        steer_sub_ = create_subscription<std_msgs::msg::Float32>(
            steer_topic_, qos_,
            std::bind(&SimulationDynamics::steer_callback, this,
                      std::placeholders::_1));
        odom_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1e3 * dt_)),
            std::bind(&SimulationDynamics::odom_callback, this));
    }

    void reset_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        // Reset the model state
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = 2. * atan2(msg->pose.pose.orientation.z,
                                  msg->pose.pose.orientation.w);
        std::vector<double> new_state = {x, y, theta};
        model_->set_state(new_state);
    }

    // Timed callback
    void odom_callback() {
        // Do not do anything if the node is not active
        // Update state
        model_->update(input_, dt_);

        // RCLCPP_INFO(get_logger(), "State: x=%.2f, y=%.2f, theta=%.2f",
        // model_->x(), model_->y(), model_->theta());

        // Publish odometry
        Eigen::VectorXd odom = model_->get_odom();
        double X = odom(0);
        double Y = odom(1);
        double theta = odom(2);
        double Xd = odom(3);
        double Yd = odom(4);
        double omega = odom(5);

        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.frame_id = frame_id_;
        odom_msg.header.stamp = get_clock()->now();
        odom_msg.pose.pose.orientation.z = sin(.5 * theta);
        odom_msg.pose.pose.orientation.w = cos(.5 * theta);
        odom_msg.pose.pose.position.x = X;
        odom_msg.pose.pose.position.y = Y;
        odom_msg.twist.twist.linear.x = Xd;
        odom_msg.twist.twist.linear.y = Yd;
        odom_msg.twist.twist.angular.z = omega;
        odometry_pub_->publish(odom_msg);
    }

    // Throttle input callback
    void throttle_callback(const std_msgs::msg::Float32::ConstSharedPtr msg) {
        // Sanitize input!
        double throttle = msg->data;
        if (std::isnan(throttle)) {
            RCLCPP_WARN(get_logger(), "Throttle input is NaN, ignoring");
            return;
        }

        if (std::abs(throttle) > 1.) {
            RCLCPP_WARN(get_logger(),
                        "Throttle input is out of bounds, clamping to [-1, 1]");
        }

        input_[0] = std::clamp(throttle, -1., 1.);
    }

    // Steering input callback
    void steer_callback(const std_msgs::msg::Float32::ConstSharedPtr msg) {
        // Sanitize input!
        double steer = msg->data;
        if (std::isnan(steer)) {
            RCLCPP_WARN(get_logger(), "Steer input is NaN, ignoring");
            return;
        }

        if (std::abs(steer) > 1.) {
            RCLCPP_WARN(get_logger(),
                        "Steer input is out of bounds, clamping to [-1, 1]");
        }

        input_[1] = std::clamp(steer, -1., 1.);
    }

    // QoS
    rclcpp::QoS qos_ = rclcpp::SensorDataQoS();

    // Frame ID for published topics
    std::string frame_id_;

    // Topics
    std::string throttle_topic_;
    std::string steer_topic_;
    std::string scope_topic_;
    std::string odometry_topic_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr reset_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steer_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr odom_timer_;

    // Model stuff
    std::map<std::string, std::shared_ptr<VehicleModel>> models_;
    std::shared_ptr<VehicleModel> model_;
    std::string model_str_;
    int fps_;
    double dt_;

    // State
    std::vector<double> init_state_;
    std::vector<double> input_ = {0., 0.};
    double delta_max_;
    double delta_bias_;
};

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto dynamics_node = std::make_shared<SimulationDynamics>("sim_dynamics");
    dynamics_node->setup();

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(dynamics_node);
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
