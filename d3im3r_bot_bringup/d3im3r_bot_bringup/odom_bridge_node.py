#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """
    Converts a yaw angle into a quaternion.

    Returns:
        (x, y, z, w)
    """
    half_yaw = yaw * 0.5

    qx = 0.0
    qy = 0.0
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)

    return qx, qy, qz, qw


def wrap_to_pi(angle: float) -> float:
    """
    Wraps angle to [-pi, pi].
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


class OdomBridgeNode(Node):
    """
    Bridge node for d3im3r_bot lightweight odometry.

    Subscribes:
        /odom_pose  geometry_msgs/msg/Vector3
            x = raw x [m]
            y = raw y [m]
            z = raw theta [rad]

        /odom_twist geometry_msgs/msg/Vector3
            x = linear velocity [m/s]
            y = angular velocity [rad/s]
            z = reserved

        /reset_odom_topic std_msgs/msg/Empty
            Resets the local odometry offset.

    Publishes:
        /odom nav_msgs/msg/Odometry

    Services:
        /reset_odom std_srvs/srv/Trigger

    Optional:
        Publishes TF odom -> base_link.
    """

    def __init__(self) -> None:
        super().__init__('d3im3r_odom_bridge')

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------
        self.declare_parameter('odom_pose_topic', '/odom_pose')
        self.declare_parameter('odom_twist_topic', '/odom_twist')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('reset_topic', '/reset_odom_topic')
        self.declare_parameter('reset_service', '/reset_odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate_hz', 30.0)

        # Covariance parameters
        self.declare_parameter('pose_covariance_xy', 0.02)
        self.declare_parameter('pose_covariance_yaw', 0.05)
        self.declare_parameter('twist_covariance_v', 0.02)
        self.declare_parameter('twist_covariance_w', 0.05)

        self.odom_pose_topic = self.get_parameter(
            'odom_pose_topic'
        ).get_parameter_value().string_value

        self.odom_twist_topic = self.get_parameter(
            'odom_twist_topic'
        ).get_parameter_value().string_value

        self.odom_topic = self.get_parameter(
            'odom_topic'
        ).get_parameter_value().string_value

        self.reset_topic = self.get_parameter(
            'reset_topic'
        ).get_parameter_value().string_value

        self.reset_service = self.get_parameter(
            'reset_service'
        ).get_parameter_value().string_value

        self.odom_frame = self.get_parameter(
            'odom_frame'
        ).get_parameter_value().string_value

        self.base_frame = self.get_parameter(
            'base_frame'
        ).get_parameter_value().string_value

        self.publish_tf = self.get_parameter(
            'publish_tf'
        ).get_parameter_value().bool_value

        publish_rate_hz = self.get_parameter(
            'publish_rate_hz'
        ).get_parameter_value().double_value

        self.pose_covariance_xy = self.get_parameter(
            'pose_covariance_xy'
        ).get_parameter_value().double_value

        self.pose_covariance_yaw = self.get_parameter(
            'pose_covariance_yaw'
        ).get_parameter_value().double_value

        self.twist_covariance_v = self.get_parameter(
            'twist_covariance_v'
        ).get_parameter_value().double_value

        self.twist_covariance_w = self.get_parameter(
            'twist_covariance_w'
        ).get_parameter_value().double_value

        # ------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------
        self.raw_pose: Optional[Vector3] = None
        self.raw_twist: Optional[Vector3] = None

        self.offset_locked = False
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_theta = 0.0

        # ------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------
        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            10,
        )

        self.pose_sub = self.create_subscription(
            Vector3,
            self.odom_pose_topic,
            self.odom_pose_callback,
            10,
        )

        self.twist_sub = self.create_subscription(
            Vector3,
            self.odom_twist_topic,
            self.odom_twist_callback,
            10,
        )

        self.reset_topic_sub = self.create_subscription(
            Empty,
            self.reset_topic,
            self.reset_topic_callback,
            10,
        )

        self.reset_srv = self.create_service(
            Trigger,
            self.reset_service,
            self.reset_service_callback,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        timer_period = 1.0 / max(publish_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('d3im3r odom bridge started.')
        self.get_logger().info(f'Subscribing to {self.odom_pose_topic}')
        self.get_logger().info(f'Subscribing to {self.odom_twist_topic}')
        self.get_logger().info(f'Publishing {self.odom_topic}')
        self.get_logger().info(f'Reset service: {self.reset_service}')
        self.get_logger().info(f'Reset topic: {self.reset_topic}')

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------
    def odom_pose_callback(self, msg: Vector3) -> None:
        self.raw_pose = msg

        if not self.offset_locked:
            self.set_offset_from_current_pose()

    def odom_twist_callback(self, msg: Vector3) -> None:
        self.raw_twist = msg

    def reset_topic_callback(self, msg: Empty) -> None:
        del msg
        ok = self.reset_local_odometry()

        if ok:
            self.get_logger().info('Odometry reset requested from topic.')
        else:
            self.get_logger().warn(
                'Odometry reset requested, but no pose has been received yet.'
            )

    def reset_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        ok = self.reset_local_odometry()

        response.success = ok

        if ok:
            response.message = 'Local odometry offset reset successfully.'
            self.get_logger().info(response.message)
        else:
            response.message = 'Cannot reset odometry: no /odom_pose received yet.'
            self.get_logger().warn(response.message)

        return response

    # ------------------------------------------------------------
    # Reset logic
    # ------------------------------------------------------------
    def set_offset_from_current_pose(self) -> bool:
        if self.raw_pose is None:
            return False

        self.offset_x = float(self.raw_pose.x)
        self.offset_y = float(self.raw_pose.y)
        self.offset_theta = float(self.raw_pose.z)

        self.offset_locked = True

        return True

    def reset_local_odometry(self) -> bool:
        """
        Resets the bridge odometry offset.

        This does not reset the ESP32 internal odometry.
        It makes the ROS 2 /odom output start from zero from
        the current robot pose.
        """
        return self.set_offset_from_current_pose()

    # ------------------------------------------------------------
    # Pose transformation
    # ------------------------------------------------------------
    def get_relative_pose(self) -> Optional[Tuple[float, float, float]]:
        if self.raw_pose is None:
            return None

        if not self.offset_locked:
            return None

        raw_x = float(self.raw_pose.x)
        raw_y = float(self.raw_pose.y)
        raw_theta = float(self.raw_pose.z)

        dx = raw_x - self.offset_x
        dy = raw_y - self.offset_y
        dtheta = wrap_to_pi(raw_theta - self.offset_theta)

        # Rotate translation into the local reset frame.
        c = math.cos(-self.offset_theta)
        s = math.sin(-self.offset_theta)

        x_rel = c * dx - s * dy
        y_rel = s * dx + c * dy

        return x_rel, y_rel, dtheta

    # ------------------------------------------------------------
    # Timer
    # ------------------------------------------------------------
    def timer_callback(self) -> None:
        relative_pose = self.get_relative_pose()

        if relative_pose is None:
            return

        x_rel, y_rel, theta_rel = relative_pose

        if self.raw_twist is None:
            v = 0.0
            w = 0.0
        else:
            v = float(self.raw_twist.x)
            w = float(self.raw_twist.y)

        now = self.get_clock().now().to_msg()

        qx, qy, qz, qw = yaw_to_quaternion(theta_rel)

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = x_rel
        odom_msg.pose.pose.position.y = y_rel
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = w

        self.fill_covariances(odom_msg)

        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            self.publish_transform(
                now=now,
                x=x_rel,
                y=y_rel,
                theta=theta_rel,
                qx=qx,
                qy=qy,
                qz=qz,
                qw=qw,
            )

    def fill_covariances(self, odom_msg: Odometry) -> None:
        # Pose covariance 6x6:
        # x, y, z, roll, pitch, yaw
        odom_msg.pose.covariance[0] = self.pose_covariance_xy
        odom_msg.pose.covariance[7] = self.pose_covariance_xy
        odom_msg.pose.covariance[35] = self.pose_covariance_yaw

        # High covariance for unused dimensions
        odom_msg.pose.covariance[14] = 999.0
        odom_msg.pose.covariance[21] = 999.0
        odom_msg.pose.covariance[28] = 999.0

        # Twist covariance 6x6:
        # vx, vy, vz, wx, wy, wz
        odom_msg.twist.covariance[0] = self.twist_covariance_v
        odom_msg.twist.covariance[35] = self.twist_covariance_w

        # High covariance for unused dimensions
        odom_msg.twist.covariance[7] = 999.0
        odom_msg.twist.covariance[14] = 999.0
        odom_msg.twist.covariance[21] = 999.0
        odom_msg.twist.covariance[28] = 999.0

    def publish_transform(
        self,
        now,
        x: float,
        y: float,
        theta: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> None:
        del theta

        transform = TransformStamped()

        transform.header.stamp = now
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame

        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = OdomBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()