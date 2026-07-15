#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy.ndimage import distance_transform_edt
import math
import time

class CostmapNode(Node):
    def __init__(self):
        super().__init__("costmap_node")
        self.inflation_radius  = 0.08
        self.inflation_weight  = 80
        self.dynamic_decay     = 0.8
        self.history_increment = 5
        self.history_max       = 40
        self.update_rate       = 10.0
        self.map_msg         = None
        self.dynamic_layer   = None
        self.history_layer   = None
        self.last_robot_cell = None
        self.last_update_t   = time.time()
        self.create_subscription(OccupancyGrid, "/map",  self.map_cb,  10)
        self.create_subscription(LaserScan,     "/scan", self.scan_cb, 10)
        self.create_subscription(Odometry,      "/odom", self.odom_cb, 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/costmap", 10)
        self.create_timer(1.0 / self.update_rate, self.update_costmap)
        self.get_logger().info("Costmap node started. Waiting for /map...")

    def map_cb(self, msg):
        new_shape = (msg.info.height, msg.info.width)
        if self.dynamic_layer is None or self.dynamic_layer.shape != new_shape:
            self.get_logger().info(f"Map received: {msg.info.width}x{msg.info.height}")
            self.dynamic_layer = np.zeros(new_shape, dtype=np.float32)
            self.history_layer = np.zeros(new_shape, dtype=np.float32)
        self.map_msg = msg

    def scan_cb(self, msg):
        if self.map_msg is None or self.dynamic_layer is None:
            return
        info  = self.map_msg.info
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x   = r * math.cos(angle)
                y   = r * math.sin(angle)
                col = int((x - info.origin.position.x) / info.resolution)
                row = int((y - info.origin.position.y) / info.resolution)
                if 0 <= row < info.height and 0 <= col < info.width:
                    self.dynamic_layer[row, col] = min(self.dynamic_layer[row, col] + 60.0, 100.0)
            angle += msg.angle_increment

    def odom_cb(self, msg):
        if self.map_msg is None or self.history_layer is None:
            return
        info = self.map_msg.info
        x    = msg.pose.pose.position.x
        y    = msg.pose.pose.position.y
        col  = int((x - info.origin.position.x) / info.resolution)
        row  = int((y - info.origin.position.y) / info.resolution)
        if 0 <= row < info.height and 0 <= col < info.width:
            cell = (row, col)
            if cell != self.last_robot_cell:
                self.history_layer[row, col] = min(self.history_layer[row, col] + self.history_increment, self.history_max)
                self.last_robot_cell = cell

    def update_costmap(self):
        if self.map_msg is None:
            return
        now  = time.time()
        dt   = now - self.last_update_t
        self.last_update_t = now
        info = self.map_msg.info
        raw  = np.array(self.map_msg.data, dtype=np.int8).reshape(info.height, info.width)
        obstacle_mask   = (raw == 100).astype(np.uint8)
        dist_cells      = distance_transform_edt(1 - obstacle_mask)
        inflation_cells = self.inflation_radius / info.resolution
        inflation_layer = self.inflation_weight * np.exp(-3.0 * (dist_cells / inflation_cells) ** 2)
        inflation_layer[obstacle_mask == 1] = 100
        inflation_layer = np.clip(inflation_layer, 0, 100)
        self.dynamic_layer *= (self.dynamic_decay ** dt)
        costmap = inflation_layer + self.dynamic_layer + self.history_layer
        costmap = np.clip(costmap, 0, 100).astype(np.int8)
        costmap[raw == -1] = -1
        out = OccupancyGrid()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.info            = info
        out.data            = costmap.flatten().tolist()
        self.costmap_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = CostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
