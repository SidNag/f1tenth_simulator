"""Pure pursuit path follower for the F1TENTH car and simulator.

How it works (one control tick, run at `rate_hz`):

  1. Pose: look up the TF map_frame -> base_frame to get the car's (x, y, yaw).
     Whatever provides that TF (sim ground truth + static map->odom, SLAM,
     AMCL, Vicon, ...) the controller code stays the same.
  2. Nearest waypoint: search a small window ahead of the last nearest index
     (wrapping around the closed loop) so it never jumps across the track.
  3. Lookahead distance: Ld = lookahead_min + lookahead_gain * speed.
  4. Goal point: walk forward along the path to where it crosses the circle of
     radius Ld around the car (interpolated on the segment).
  5. Geometry: express the goal in the car frame (x forward, y left). The arc
     through the rear axle and the goal has curvature kappa = 2*y_c / Ld^2.
  6. Steering for a kinematic bicycle: delta = atan(wheelbase * kappa),
     clamped to +-max_steer.
  7. Safety: if the pose is missing/stale, or the deadman button is not held
     (when use_deadman is true), publish speed 0.

Outputs: AckermannDriveStamped on `drive_topic`, a sphere Marker at the goal
point on ~/lookahead, and the waypoint path as nav_msgs/Path on ~/path, so you
can see both in RViz.

Waypoint CSV format: columns x,y and optionally v (speed). A header row is
optional. The path is treated as a closed loop.
"""
import csv
import math
import os

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


def load_waypoints(path):
    """Read a CSV of x,y[,v]. Returns (N,2) xy array and (N,) speeds or None."""
    rows = []
    with open(os.path.expanduser(path)) as f:
        for row in csv.reader(f):
            if not row:
                continue
            try:
                rows.append([float(v) for v in row])
            except ValueError:
                continue  # header row such as "x,y"
    if len(rows) < 3:
        raise RuntimeError(f'{path}: need at least 3 waypoints, got {len(rows)}')
    data = np.array(rows)
    speeds = data[:, 2] if data.shape[1] >= 3 else None
    return data[:, :2], speeds


def yaw_from_quaternion(q):
    """Heading (rotation about z) from a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        p = self.declare_parameter
        self.drive_topic = p('drive_topic', '/ego_racecar/drive').value
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'ego_racecar/base_link').value
        self.waypoint_file = p('waypoint_file', '').value
        self.wheelbase = float(p('wheelbase', 0.33).value)
        self.max_steer = float(p('max_steer', 0.26).value)
        self.speed = float(p('speed', 1.5).value)
        self.use_csv_speed = bool(p('use_csv_speed', False).value)
        self.lookahead_min = float(p('lookahead_min', 0.6).value)
        self.lookahead_gain = float(p('lookahead_gain', 0.3).value)
        rate_hz = float(p('rate_hz', 50.0).value)
        self.use_deadman = bool(p('use_deadman', False).value)
        self.deadman_button = int(p('deadman_button', 5).value)
        joy_topic = p('joy_topic', '/joy').value
        self.tf_timeout = float(p('tf_timeout', 0.2).value)
        self.search_window = int(p('search_window', 50).value)

        self.xy, self.csv_speeds = load_waypoints(self.waypoint_file)
        self.n = len(self.xy)
        if self.use_csv_speed and self.csv_speeds is None:
            self.get_logger().warn('use_csv_speed is true but the CSV has no v column; using `speed`')
            self.use_csv_speed = False
        self.nearest_idx = None   # None until the first full search
        self.last_joy = None      # (time, button_held)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.marker_pub = self.create_publisher(Marker, '~/lookahead', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '~/path', latched)
        if self.use_deadman:
            self.create_subscription(Joy, joy_topic, self.joy_cb, 10)

        self.path_msg = self.make_path_msg()
        self.publish_path()
        self.create_timer(1.0, self.publish_path)
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f'Loaded {self.n} waypoints from {self.waypoint_file}; speed={self.speed} m/s, '
            f'Ld={self.lookahead_min}+{self.lookahead_gain}*v, deadman={self.use_deadman}')

    # ------------------------------------------------------------------ inputs
    def joy_cb(self, msg):
        held = len(msg.buttons) > self.deadman_button and msg.buttons[self.deadman_button] == 1
        self.last_joy = (self.get_clock().now(), held)

    def deadman_ok(self):
        if not self.use_deadman:
            return True
        if self.last_joy is None:
            return False
        stamp, held = self.last_joy
        return held and (self.get_clock().now() - stamp) < Duration(seconds=0.5)

    def get_pose(self):
        """Return (x, y, yaw) of base_frame in map_frame, or None if missing/stale."""
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception as e:
            self.get_logger().warn(f'No TF {self.map_frame}->{self.base_frame}: {e}',
                                   throttle_duration_sec=2.0)
            return None
        age = self.get_clock().now() - Time.from_msg(tf.header.stamp)
        if age > Duration(seconds=self.tf_timeout):
            self.get_logger().warn(f'TF is stale ({age.nanoseconds / 1e9:.2f} s old)',
                                   throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_from_quaternion(tf.transform.rotation)

    # ------------------------------------------------------------- path logic
    def find_nearest(self, x, y):
        """Index of the closest waypoint, searching only a window ahead of the last one."""
        if self.nearest_idx is None:
            candidates = np.arange(self.n)  # first tick: search everything
        else:
            candidates = (self.nearest_idx + np.arange(self.search_window)) % self.n
        d2 = np.sum((self.xy[candidates] - (x, y)) ** 2, axis=1)
        self.nearest_idx = int(candidates[np.argmin(d2)])
        return self.nearest_idx

    def find_goal(self, x, y, start_idx, ld):
        """Point where the path leaves the circle of radius ld around (x, y).

        Walks forward from start_idx; on the first segment whose end is outside
        the circle, solves |p0 + t*(p1-p0) - car| = ld for t in [0, 1].
        """
        car = np.array([x, y])
        for k in range(self.n):
            p0 = self.xy[(start_idx + k) % self.n]
            p1 = self.xy[(start_idx + k + 1) % self.n]
            if np.linalg.norm(p1 - car) < ld:
                continue
            d = p1 - p0
            f = p0 - car
            a = d @ d
            b = 2.0 * (f @ d)
            c = f @ f - ld * ld
            disc = b * b - 4.0 * a * c
            if a > 1e-12 and disc >= 0.0:
                t = (-b + math.sqrt(disc)) / (2.0 * a)  # the exit (farther) intersection
                if 0.0 <= t <= 1.0:
                    return p0 + t * d
            return p1  # start point already outside the circle; just aim at p1
        return self.xy[(start_idx + 1) % self.n]  # whole path inside the circle (tiny loop)

    # ------------------------------------------------------------- main loop
    def tick(self):
        pose = self.get_pose()
        if pose is None:
            self.publish_drive(0.0, 0.0)
            return
        x, y, yaw = pose

        idx = self.find_nearest(x, y)
        speed = float(self.csv_speeds[idx]) if self.use_csv_speed else self.speed
        ld = self.lookahead_min + self.lookahead_gain * speed
        gx, gy = self.find_goal(x, y, idx, ld)

        # Goal in the car frame (x forward, y left).
        dx, dy = gx - x, gy - y
        x_c = math.cos(yaw) * dx + math.sin(yaw) * dy
        y_c = -math.sin(yaw) * dx + math.cos(yaw) * dy
        dist = math.hypot(x_c, y_c)  # == ld except when aiming at a fallback point

        kappa = 2.0 * y_c / (dist * dist) if dist > 1e-6 else 0.0
        steer = math.atan(self.wheelbase * kappa)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        if not self.deadman_ok():
            speed = 0.0
        self.publish_drive(speed, steer)
        self.publish_marker(gx, gy)

    # ---------------------------------------------------------------- outputs
    def publish_drive(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)

    def publish_marker(self, gx, gy):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'lookahead', 0, Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y = float(gx), float(gy)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.8, 1.0
        self.marker_pub.publish(m)

    def make_path_msg(self):
        path = Path()
        path.header.frame_id = self.map_frame
        for px, py in self.xy:
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x, ps.pose.position.y = float(px), float(py)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)


def main(args=None):
    # Handle Ctrl-C ourselves (KeyboardInterrupt) so the ROS context is still valid
    # in `finally` and we can still publish/save before shutting down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Best effort: leave the car stopped.
        if rclpy.ok():
            node.publish_drive(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
