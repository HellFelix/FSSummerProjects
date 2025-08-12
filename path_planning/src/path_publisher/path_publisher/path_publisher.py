import os
import numpy as np
import pandas as pd
import rclpy
import rclpy.timer
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Duration
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray


class PathPublisher(Node):
    def __init__(self):
        super().__init__("path_publisher")

        # Declare publishers and timers
        self._cone_publisher: Publisher
        self._path_publisher: Publisher
        self._pub_timer: rclpy.timer.Timer
        self._qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Declare parameters
        self.declare_parameter("timer_period", value=5.0)
        self.declare_parameter("track.chosen", value=0)
        self.declare_parameter("track.random", value=False)
        self.declare_parameter("frame_id.world", value="map")
        self.declare_parameter("cone_dimensions.regular.radius", value=0.2)
        self.declare_parameter("cone_dimensions.regular.height", value=0.2)
        self.declare_parameter("cone_dimensions.large.radius", value=0.6)
        self.declare_parameter("cone_dimensions.large.height", value=0.6)
        self.declare_parameter("topic.path_publisher.cones", value="/cones")
        self.declare_parameter("topic.path_publisher.centerline", value="/centerline")

        # Used for creating unique ids for each marker
        self._counter = 0
        # Shutdown flag
        self._shutdown_request = False
        # Get parameters
        def get_param(name: str):
            return self.get_parameter(name).get_parameter_value()

        self._timer_period = get_param("timer_period").double_value
        self._chosen_track = get_param("track.chosen").integer_value
        self._random_track = get_param("track.random").bool_value
        self._frame_id = get_param("frame_id.world").string_value
        self._cone_radius = get_param("cone_dimensions.regular.radius").double_value
        self._cone_height = get_param("cone_dimensions.regular.height").double_value
        self._xl_cone_radius = get_param("cone_dimensions.large.radius").double_value
        self._xl_cone_height = get_param("cone_dimensions.large.height").double_value
        cone_topic = get_param("topic.path_publisher.cones").string_value
        centerline_topic = get_param("topic.path_publisher.centerline").string_value

        # Publishers & timers
        self._cone_publisher = self.create_publisher(
            msg_type=MarkerArray, topic=cone_topic, qos_profile=self._qos
        )
        self._path_publisher = self.create_publisher(
            msg_type=Path, topic=centerline_topic, qos_profile=self._qos
        )
        self._pub_timer = self.create_timer(
            timer_period_sec=self._timer_period, callback=self._timer_callback
        )

        blue, yellow, orange = ColorRGBA(), ColorRGBA(), ColorRGBA()
        blue.r, blue.g, blue.b, blue.a = 0.0, 0.0, 1.0, 1.0
        yellow.r, yellow.g, yellow.b, yellow.a = 1.0, 1.0, 0.0, 1.0
        orange.r, orange.g, orange.b, orange.a = 1.0, 0.5, 0.0, 1.0
        self._color_dict = {"b": blue, "y": yellow, "o": orange}

        # Find the path to the tracks folder and count how many there are
        self._track_dir = os.path.join(
            get_package_share_directory("path_publisher"), "tracks"
        )
        self._num_tracks = len(os.listdir(self._track_dir))
        # Load track (choose random one if requested)
        if self._random_track:
            self._chosen_track = np.random.randint(0, self._num_tracks)

        try:
            data = pd.read_csv(
                self._track_dir + "/track_{0:03d}.csv".format(self._chosen_track)
            )
        except FileNotFoundError:
            self.get_logger().error(f"Track with name 'track_{self._chosen_track:03d}.csv' not found")
            return TransitionCallbackReturn.FAILURE

        self.num_track_points = data.shape[0]
        self._cone_x = data["x"]
        self._cone_y = data["y"]
        self._cone_colors = data["color"]

    def _timer_callback(self) -> None:

        # Publish the chosen track as colored cyliners + reference path
        markers, center_points = [], []
        path_msg = Path()
        path_msg.header.frame_id = self._frame_id
        for i in range(self.num_track_points):
            # Check if's a centerline point
            if self._cone_colors[i] == "c":
                center_point = PoseStamped()
                center_point.header.frame_id = self._frame_id
                center_point.header.stamp = self.get_clock().now().to_msg()
                # Pose
                center_point.pose.position.x = self._cone_x[i]
                center_point.pose.position.y = self._cone_y[i]
                center_point.pose.position.z = 0.0
                # Compute orientation
                yaw = -0.5 * np.pi
                if center_points:
                    dx = self._cone_x[i] - center_points[-1].pose.position.x
                    dy = self._cone_y[i] - center_points[-1].pose.position.y
                    yaw = np.arctan2(dy, dx) - np.pi
                center_point.pose.orientation.w = np.cos(0.5 * yaw)
                center_point.pose.orientation.x = 0.0
                center_point.pose.orientation.y = 0.0
                center_point.pose.orientation.z = np.sin(0.5 * yaw)

                center_points.append(center_point)
            else:
                # Scale of the marker
                radius = (
                    self._xl_cone_radius
                    if self._cone_colors[i] == "o"
                    else self._cone_radius
                )
                height = (
                    self._xl_cone_height
                    if self._cone_colors[i] == "o"
                    else self._cone_height
                )
                # Create marker object
                marker = Marker()
                marker.type = Marker.CYLINDER
                # Position + orientation
                marker.scale.x, marker.scale.y, marker.scale.z = radius, radius, height
                marker.pose.position.x, marker.pose.position.y = (
                    self._cone_x[i],
                    self._cone_y[i],
                )
                marker.pose.position.z = height / 2
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                # Header
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                # Color, lifetime, namespace, id
                marker.color = self._color_dict[self._cone_colors[i]]
                marker.lifetime = Duration(
                    seconds=self._timer_period, nanoseconds=0
                ).to_msg()
                marker.ns = "track"
                marker.id = self._counter
                self._counter += 1

                markers.append(marker)

        cone_msg = MarkerArray()
        cone_msg.markers = markers
        path_msg.poses = center_points
        path_msg.header.stamp = self.get_clock().now().to_msg()
        self._path_publisher.publish(path_msg)
        self._cone_publisher.publish(cone_msg)


def main(args=None):
    rclpy.init(args=args)
    path_pub = PathPublisher()

    try:
        rclpy.spin(path_pub)
    except KeyboardInterrupt:
        pass
    finally:
        path_pub.destroy_node()


if __name__ == "__main__":
    main()
