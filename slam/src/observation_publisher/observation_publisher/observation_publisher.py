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
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray
import math
import random
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation

class ObservationPublisher(Node):
    current = 1
    current_pos = [0, 0]
    heading = 0.0
    def __init__(self):
        super().__init__("path_publisher")

        # Declare publishers and timers
        self._cone_publisher: Publisher
        self._path_publisher: Publisher
        self._pub_timer: rclpy.timer.Timer
        self._qos: QoSProfile = qos_profile_sensor_data

        # Declare parameters
        self.declare_parameter("timer_period", value=0.1)
        self.declare_parameter("track.chosen", value=0)
        self.declare_parameter("track.random", value=False)
        self.declare_parameter("frame_id.world", value="map")
        self.declare_parameter("topic.path_publisher.cones", value="/cones")
        self.declare_parameter("speed", value=10.1)

        self._counter = 0
        self._shutdown_request = False
        def get_param(name: str):
            return self.get_parameter(name).get_parameter_value()

        self._speed = get_param("speed").double_value
        self._timer_period = get_param("timer_period").double_value
        self._chosen_track = get_param("track.chosen").integer_value
        self._random_track = get_param("track.random").bool_value
        self._frame_id = get_param("frame_id.world").string_value
        cone_topic = get_param("topic.path_publisher.cones").string_value

        # Publishers & timers
        self._cone_publisher = self.create_publisher(
            msg_type=MarkerArray, topic=cone_topic, qos_profile=self._qos
        )
        self._pos_publisher = self.create_publisher(
            msg_type=PoseStamped, topic="/current_position", qos_profile=self._qos
        )
        self._pub_timer = self.create_timer(
            timer_period_sec=self._timer_period, callback=self._timer_callback
        )

        self._track_dir = os.path.join(
            get_package_share_directory("path_publisher"), "tracks"
        )
        self._num_tracks = len(os.listdir(self._track_dir))
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

        cones = data[data["color"] != "c"]
        centerline = data[data["color"] == "c"]
        self._cone_x = cones["x"]
        self._cone_y = cones["y"]
        self._centerline_x = centerline["x"]
        self._centerline_y = centerline["y"]
        self.current_pos[0] = self._centerline_x.iloc[0]
        self.current_pos[1] = self._centerline_y.iloc[0]

    def _timer_callback(self) -> None:
        ds = self._speed * self._timer_period
        moved_dist = 0
        while moved_dist < ds and self.current < len(self._centerline_x):
            next_x = self._centerline_x.iloc[self.current]
            next_y = self._centerline_y.iloc[self.current]
            dx = next_x - self.current_pos[0]
            dy = next_y - self.current_pos[1]
            dist = np.hypot(dx, dy)

            if dist <= ds - moved_dist:
                self.current_pos[0] = next_x
                self.current_pos[1] = next_y
                self.current += 1
                moved_dist += dist
            else:
                ratio = (ds - moved_dist) / dist
                self.current_pos[0] += dx * ratio
                self.current_pos[1] += dy * ratio
                moved_dist = ds
                self.heading = math.atan2(dy, dx)

        if self.current >= len(self._centerline_x):
            self.current = 0
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.pose.position.x = self.current_pos[0]
        msg.pose.position.y = self.current_pos[1]
        msg.pose.position.z = 0.0

        quat = Rotation.from_euler('z', self.heading).as_quat()

        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        self._pos_publisher.publish(msg)


        points_in_cone = self._points_in_cone(zip(self._cone_x, self._cone_y), 1.2, 100)
        points_in_cone = self.noisy_cone_points(points_in_cone, self.heading, 0.2, 200, 0.05)
        markers = []
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_all.header.frame_id = self._frame_id
        delete_all.header.stamp = self.get_clock().now().to_msg()

        markers.append(delete_all)
        for i, (px, py) in enumerate(points_in_cone):
            marker = Marker()
            marker.header.frame_id = self._frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cones"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = px
            marker.pose.position.y = py
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 0.5

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            markers.append(marker)

        marker_array = MarkerArray()
        marker_array.markers = markers

        self._cone_publisher.publish(marker_array)



    def _points_in_cone(self, points, cone_angle, cone_range):
        result = []
        cx, cy = self.current_pos
        for px, py in points:
            dx, dy = px - cx, py - cy
            dist = np.hypot(dx, dy)
            if dist > cone_range:
                continue
            angle = np.arctan2(dy, dx) - self.heading
            angle = (angle + np.pi) % (2 * np.pi) - np.pi
            if abs(angle) <= cone_angle / 2:
                result.append([px, py])
        return result

    
    def noisy_cone_points(self,points, heading, noise_std, max_dist, drop_rate):
        noisy_points = []
        for x, y in points:
            dx = x * math.cos(heading) + y * math.sin(heading)
            dy = -x * math.sin(heading) + y * math.cos(heading)
            dist = math.hypot(dx, dy)
            if dist > max_dist:
                continue
            if random.random() > math.exp(-drop_rate * dist):
                x_noisy = x + random.gauss(0, noise_std)
                y_noisy = y + random.gauss(0, noise_std)
                noisy_points.append((x_noisy, y_noisy))
        return noisy_points
            









def main(args=None):
    rclpy.init(args=args)
    observation_publisher = ObservationPublisher()

    try:
        rclpy.spin(observation_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        observation_publisher.destroy_node()


if __name__ == "__main__":
    main()
