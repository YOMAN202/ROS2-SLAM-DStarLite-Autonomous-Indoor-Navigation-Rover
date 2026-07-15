#!/usr/bin/env python3
import math
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_serial")
        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("wheel_base", 0.31)
        self.declare_parameter("max_linear_speed", 0.05)
        self.declare_parameter("max_angular_speed", 0.15)
        self.declare_parameter("min_motor_speed", 60)
        self.declare_parameter("max_motor_speed", 80)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.port            = self.get_parameter("serial_port").value
        self.baud            = self.get_parameter("baud_rate").value
        self.wheel_base      = self.get_parameter("wheel_base").value
        self.max_linear      = self.get_parameter("max_linear_speed").value
        self.max_angular     = self.get_parameter("max_angular_speed").value
        self.min_motor_speed = self.get_parameter("min_motor_speed").value
        self.max_motor_speed = self.get_parameter("max_motor_speed").value
        self.cmd_timeout     = self.get_parameter("cmd_timeout_sec").value
        self.ser = None
        self.connect_serial()
        self.last_cmd_time = time.time()
        self.last_sent     = (0, 0)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)
        self.create_timer(0.1, self.watchdog_check)
        self.get_logger().info(f"cmd_vel_to_serial started. Port={self.port} wheel_base={self.wheel_base}m")

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"Connected to ESP32 on {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            self.ser = None

    def clamp_motor_speed(self, speed):
        sign = 1 if speed >= 0 else -1
        mag  = abs(speed)
        if mag < 1:
            return 0
        if mag < self.min_motor_speed:
            mag = self.min_motor_speed
        mag = min(mag, self.max_motor_speed)
        return int(sign * mag)

    def cmd_vel_cb(self, msg):
        linear  = msg.linear.x
        angular = msg.angular.z
        linear  = max(-self.max_linear,  min(self.max_linear,  linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))
        left_mps  = linear - (angular * self.wheel_base / 2.0)
        right_mps = linear + (angular * self.wheel_base / 2.0)
        left_pct  = (left_mps  / self.max_linear) * 100.0 if self.max_linear > 0 else 0
        right_pct = (right_mps / self.max_linear) * 100.0 if self.max_linear > 0 else 0
        left_pct  = self.clamp_motor_speed(left_pct)
        right_pct = self.clamp_motor_speed(right_pct)
        self.send_motor_command(left_pct, right_pct)
        self.last_cmd_time = time.time()

    def send_motor_command(self, left, right):
        if (left, right) == self.last_sent and not (left == 0 and right == 0):
            return
        if self.ser is None:
            self.connect_serial()
            if self.ser is None:
                return
        cmd = f"m {left} {right}\n"
        try:
            self.ser.write(cmd.encode())
            self.last_sent = (left, right)
            self.get_logger().info(f"Sent: m {left} {right}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")
            self.ser = None

    def watchdog_check(self):
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            if self.last_sent != (0, 0):
                self.send_motor_command(0, 0)

    def stop_motors(self):
        if self.ser is not None:
            try:
                self.ser.write(b"x\n")
            except serial.SerialException:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
