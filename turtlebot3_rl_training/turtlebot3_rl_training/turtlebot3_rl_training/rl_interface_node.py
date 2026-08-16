#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, Bool


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RLInterfaceNode(Node):
    def __init__(self):
        super().__init__('rl_interface_node')

        self.declare_parameter('max_sensor_range', 3.5)
        self.declare_parameter('max_goal_distance', 5.0)
        self.declare_parameter('goal_tolerance', 0.18)
        self.declare_parameter('goal_model_name', 'goal_marker')
        self.declare_parameter('robot_model_name', 'turtlebot3')
        self.declare_parameter('publish_rate', 10.0)

        self.max_sensor_range = float(self.get_parameter('max_sensor_range').value)
        self.max_goal_distance = float(self.get_parameter('max_goal_distance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.goal_model_name = str(self.get_parameter('goal_model_name').value)
        self.robot_model_name = str(self.get_parameter('robot_model_name').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.scan_msg: Optional[LaserScan] = None
        self.odom_msg: Optional[Odometry] = None

        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None

        self.state_pub = self.create_publisher(
            Float32MultiArray,
            '/rl_state',
            10
        )

        self.goal_reached_pub = self.create_publisher(
            Bool,
            '/rl_goal_reached',
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_state
        )

        self.get_logger().info('RL Interface Node started.')
        self.get_logger().info('Publishing /rl_state and /rl_goal_reached.')

    def scan_callback(self, msg: LaserScan):
        self.scan_msg = msg

    def odom_callback(self, msg: Odometry):
        self.odom_msg = msg

    def model_states_callback(self, msg: ModelStates):
        if self.goal_model_name not in msg.name:
            return

        index = msg.name.index(self.goal_model_name)
        pose = msg.pose[index]

        self.goal_x = pose.position.x
        self.goal_y = pose.position.y

    def get_scan_distance_at_angle(self, target_angle: float) -> float:
        if self.scan_msg is None:
            return self.max_sensor_range

        scan = self.scan_msg

        if scan.angle_increment == 0.0:
            return self.max_sensor_range

        index = int(round((target_angle - scan.angle_min) / scan.angle_increment))
        index = max(0, min(index, len(scan.ranges) - 1))

        value = scan.ranges[index]

        if math.isinf(value) or math.isnan(value):
            return self.max_sensor_range

        value = max(scan.range_min, min(value, self.max_sensor_range))
        return value

    def normalize_distance(self, distance: float, max_distance: float) -> float:
        return max(0.0, min(distance / max_distance, 1.0))

    def publish_state(self):
        if self.odom_msg is None:
            return

        if self.goal_x is None or self.goal_y is None:
            return

        pose = self.odom_msg.pose.pose

        robot_x = pose.position.x
        robot_y = pose.position.y
        robot_yaw = yaw_from_quaternion(pose.orientation)

        d_front = self.get_scan_distance_at_angle(0.0)
        d_left = self.get_scan_distance_at_angle(math.pi / 4.0)
        d_right = self.get_scan_distance_at_angle(-math.pi / 4.0)

        dx = self.goal_x - robot_x
        dy = self.goal_y - robot_y

        d_goal = math.sqrt(dx * dx + dy * dy)
        theta_goal = math.atan2(dy, dx)
        theta_error = normalize_angle(theta_goal - robot_yaw)

        d_front_norm = self.normalize_distance(d_front, self.max_sensor_range)
        d_left_norm = self.normalize_distance(d_left, self.max_sensor_range)
        d_right_norm = self.normalize_distance(d_right, self.max_sensor_range)
        theta_goal_norm = theta_error / math.pi
        d_goal_norm = self.normalize_distance(d_goal, self.max_goal_distance)

        state_msg = Float32MultiArray()
        state_msg.data = [
            float(d_front_norm),
            float(d_left_norm),
            float(d_right_norm),
            float(theta_goal_norm),
            float(d_goal_norm)
        ]

        goal_reached = d_goal <= self.goal_tolerance

        goal_msg = Bool()
        goal_msg.data = bool(goal_reached)

        self.state_pub.publish(state_msg)
        self.goal_reached_pub.publish(goal_msg)


def main():
    rclpy.init()
    node = RLInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()