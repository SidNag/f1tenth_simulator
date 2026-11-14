"""Waypoint logger: record the car's path in the map frame as a CSV of x,y.

Drive a lap (keyboard, joystick, or any controller) while this runs. It looks up
the TF map_frame -> base_frame at 20 Hz and appends a point every time the car
has moved at least `spacing` metres since the last saved point.

Because it reads the same TF that pure_pursuit reads, the recorded path and the
controller always agree on what "map" means (sim ground truth, SLAM, AMCL, ...).

The CSV (header "x,y") is written every 100 points and again on Ctrl-C.
"""
import csv
import math
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'ego_racecar/base_link').value
        self.output_file = os.path.expanduser(
            self.declare_parameter('output_file', '~/waypoints.csv').value)
        self.spacing = float(self.declare_parameter('spacing', 0.05).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.points = []          # list of (x, y)
        self.last_saved_count = 0
        self.timer = self.create_timer(1.0 / 20.0, self.tick)
        self.get_logger().info(
            f'Logging {self.map_frame} -> {self.base_frame} every {self.spacing} m '
            f'to {self.output_file}')

    def tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception as e:  # TF not available yet
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        if self.points:
            px, py = self.points[-1]
            if math.hypot(x - px, y - py) < self.spacing:
                return
        self.points.append((x, y))
        if len(self.points) - self.last_saved_count >= 100:
            self.save()

    def save(self):
        if not self.points:
            return
        os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
        with open(self.output_file, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['x', 'y'])
            w.writerows((f'{x:.4f}', f'{y:.4f}') for x, y in self.points)
        self.last_saved_count = len(self.points)
        self.get_logger().info(f'Saved {len(self.points)} points to {self.output_file}')


def main(args=None):
    # Handle Ctrl-C ourselves (KeyboardInterrupt) so the ROS context is still valid
    # in `finally` and we can still publish/save before shutting down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WaypointLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
