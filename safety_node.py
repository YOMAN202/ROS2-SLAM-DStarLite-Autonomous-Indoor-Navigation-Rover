#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

class SafetyNode(Node):
    def __init__(self):
        super().__init__("lidar_safety_node")
        self.declare_parameter("stop_distance", 0.25)
        self.declare_parameter("warn_distance", 0.50)
        self.declare_parameter("safe_distance", 0.70)
        self.declare_parameter("stop_speed",    0.00)
        self.declare_parameter("warn_speed",    0.03)
        self.declare_parameter("front_angle_deg", 30.0)
        self.declare_parameter("rear_angle_deg", 30.0)

        self.stop_dist   = self.get_parameter("stop_distance").value
        self.warn_dist   = self.get_parameter("warn_distance").value
        self.stop_speed  = self.get_parameter("stop_speed").value
        self.warn_speed  = self.get_parameter("warn_speed").value
        self.front_angle = math.radians(self.get_parameter("front_angle_deg").value)
        self.rear_angle  = math.radians(self.get_parameter("rear_angle_deg").value)

        self.latest_cmd = Twist()
        self.front_dist = 999.0
        self.rear_dist  = 999.0
        self.state = "SAFE"

        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_raw", self.cmd_cb, 10)
        self.cmd_pub     = self.create_publisher(Twist, "/cmd_vel", 10)
        self.blocked_pub = self.create_publisher(Bool, "/obstacle_blocked", 10)
        self.create_timer(0.05, self.publish_safe_cmd)

        self.get_logger().info("LiDAR safety node started (3-state + rear check).")

    def scan_cb(self, msg):
        ranges = list(msg.ranges)
        angle_inc = msg.angle_increment
        angle_min = msg.angle_min
        front_min = 999.0
        rear_min  = 999.0

        for i, r in enumerate(ranges):
            if not math.isfinite(r):
                continue
            if not (msg.range_min < r < msg.range_max):
                continue
            angle = angle_min + i * angle_inc
            if abs(angle) <= self.front_angle:
                front_min = min(front_min, r)
            if abs(abs(angle) - math.pi) <= self.rear_angle:
                rear_min = min(rear_min, r)

        self.front_dist = front_min
        self.rear_dist  = rear_min

    def cmd_cb(self, msg):
        self.latest_cmd = msg

    def publish_safe_cmd(self):
        cmd = Twist()
        cmd.angular.z = self.latest_cmd.angular.z
        blocked = Bool()
        blocked.data = False

        linear = self.latest_cmd.linear.x

        going_forward  = linear > 0
        going_backward = linear < 0

        front_danger = self.front_dist < self.stop_dist
        front_warn   = self.front_dist < self.warn_dist
        rear_danger  = self.rear_dist  < self.stop_dist

        if going_forward and front_danger:
            self.state = "DANGER"
            cmd.linear.x = self.stop_speed
            cmd.angular.z = 0.0
            blocked.data = True
            self.get_logger().warn(
                f"FRONT DANGER {self.front_dist:.2f}m - STOP",
                throttle_duration_sec=1.0)
        elif going_backward and rear_danger:
            self.state = "DANGER"
            cmd.linear.x = self.stop_speed
            cmd.angular.z = 0.0
            blocked.data = True
            self.get_logger().warn(
                f"REAR DANGER {self.rear_dist:.2f}m - STOP",
                throttle_duration_sec=1.0)
        elif going_forward and front_warn:
            self.state = "WARNING"
            cmd.linear.x = self.warn_speed
            self.get_logger().info(
                f"FRONT WARNING {self.front_dist:.2f}m - SLOW",
                throttle_duration_sec=1.0)
        else:
            self.state = "SAFE"
            cmd.linear.x = linear

        self.cmd_pub.publish(cmd)
        self.blocked_pub.publish(blocked)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
