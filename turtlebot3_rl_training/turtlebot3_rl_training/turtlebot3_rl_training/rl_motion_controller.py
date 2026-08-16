#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RLMotionController(Node):
    """
    Controlador de movimiento discreto para RL.

    Espacio de acciones:
        0 = avanzar recto
        1 = curva suave izquierda
        2 = curva suave derecha
        3 = curva fuerte izquierda
        4 = curva fuerte derecha

    Cada acción se ejecuta como un comando de velocidad (v, w)
    durante un tiempo fijo. Esto evita que el robot se quede girando
    en sitio sin generar progreso hacia la meta.
    """

    def __init__(self):
        super().__init__('rl_motion_controller')

        # ------------------------------------------------------------
        # Parámetros generales
        # ------------------------------------------------------------
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('action_duration', 0.60)
        self.declare_parameter('action_timeout', 2.0)

        # ------------------------------------------------------------
        # Velocidades para acciones tipo arco
        # ------------------------------------------------------------
        self.declare_parameter('linear_speed_forward', 0.18)

        self.declare_parameter('linear_speed_soft_turn', 0.14)
        self.declare_parameter('angular_speed_soft_turn', 0.60)

        self.declare_parameter('linear_speed_hard_turn', 0.10)
        self.declare_parameter('angular_speed_hard_turn', 1.20)

        self.control_rate = float(
            self.get_parameter('control_rate').value
        )

        self.action_duration = float(
            self.get_parameter('action_duration').value
        )

        self.action_timeout = float(
            self.get_parameter('action_timeout').value
        )

        self.linear_speed_forward = float(
            self.get_parameter('linear_speed_forward').value
        )

        self.linear_speed_soft_turn = float(
            self.get_parameter('linear_speed_soft_turn').value
        )

        self.angular_speed_soft_turn = float(
            self.get_parameter('angular_speed_soft_turn').value
        )

        self.linear_speed_hard_turn = float(
            self.get_parameter('linear_speed_hard_turn').value
        )

        self.angular_speed_hard_turn = float(
            self.get_parameter('angular_speed_hard_turn').value
        )

        # ------------------------------------------------------------
        # Publicadores y suscriptores
        # ------------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.action_done_pub = self.create_publisher(
            Bool,
            '/rl_action_done',
            10
        )

        self.create_subscription(
            Int32,
            '/rl_action',
            self.action_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/rl_goal_reached',
            self.goal_reached_callback,
            10
        )

        # ------------------------------------------------------------
        # Estado interno
        # ------------------------------------------------------------
        self.odom_ready: bool = False
        self.goal_reached: bool = False

        self.active_action: Optional[int] = None
        self.action_start_time = None

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        self.publish_action_done(True)

        self.get_logger().info('RL Motion Controller started.')
        self.get_logger().info('Action space:')
        self.get_logger().info('  0 = forward')
        self.get_logger().info('  1 = soft left arc')
        self.get_logger().info('  2 = soft right arc')
        self.get_logger().info('  3 = hard left arc')
        self.get_logger().info('  4 = hard right arc')
        self.get_logger().info(
            f'action_duration={self.action_duration:.2f} s, '
            f'action_timeout={self.action_timeout:.2f} s, '
            f'control_rate={self.control_rate:.1f} Hz'
        )
        self.get_logger().info(
            f'forward: v={self.linear_speed_forward:.3f}, w=0.000 | '
            f'soft: v={self.linear_speed_soft_turn:.3f}, '
            f'w=±{self.angular_speed_soft_turn:.3f} | '
            f'hard: v={self.linear_speed_hard_turn:.3f}, '
            f'w=±{self.angular_speed_hard_turn:.3f}'
        )

    def odom_callback(self, msg: Odometry):
        self.odom_ready = True

    def goal_reached_callback(self, msg: Bool):
        self.goal_reached = bool(msg.data)

        if self.goal_reached and self.active_action is not None:
            self.get_logger().info(
                'Goal reached during active action. Stopping robot.'
            )
            self.finish_action(reason='goal reached')

    def publish_action_done(self, done: bool):
        msg = Bool()
        msg.data = bool(done)
        self.action_done_pub.publish(msg)

    def action_callback(self, msg: Int32):
        if self.goal_reached:
            self.get_logger().info(
                'Ignoring action because goal is already reached.'
            )
            self.stop_robot()
            self.publish_action_done(True)
            return

        if not self.odom_ready:
            self.get_logger().warn(
                'Ignoring action because odometry is not ready.'
            )
            self.stop_robot()
            self.publish_action_done(True)
            return

        if self.active_action is not None:
            self.get_logger().debug(
                'Ignoring action because another action is active.'
            )
            return

        action = int(msg.data)

        if action not in [0, 1, 2, 3, 4]:
            self.get_logger().warn(f'Invalid RL action: {action}')
            self.stop_robot()
            self.publish_action_done(True)
            return

        self.active_action = action
        self.action_start_time = self.get_clock().now()

        self.publish_action_done(False)

        self.get_logger().debug(f'Received action: {action}')

    def stop_robot(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def finish_action(self, reason: str = 'completed'):
        self.stop_robot()

        finished_action = self.active_action

        self.active_action = None
        self.action_start_time = None

        self.publish_action_done(True)

        if finished_action is not None:
            self.get_logger().debug(
                f'Action {finished_action} finished: {reason}'
            )

    def elapsed_action_time(self) -> float:
        if self.action_start_time is None:
            return 0.0

        elapsed = self.get_clock().now() - self.action_start_time
        return elapsed.nanoseconds / 1e9

    def get_cmd_for_action(self, action: int) -> Twist:
        cmd = Twist()

        if action == 0:
            # Avanzar recto
            cmd.linear.x = self.linear_speed_forward
            cmd.angular.z = 0.0

        elif action == 1:
            # Curva suave izquierda
            cmd.linear.x = self.linear_speed_soft_turn
            cmd.angular.z = self.angular_speed_soft_turn

        elif action == 2:
            # Curva suave derecha
            cmd.linear.x = self.linear_speed_soft_turn
            cmd.angular.z = -self.angular_speed_soft_turn

        elif action == 3:
            # Curva fuerte izquierda
            cmd.linear.x = self.linear_speed_hard_turn
            cmd.angular.z = self.angular_speed_hard_turn

        elif action == 4:
            # Curva fuerte derecha
            cmd.linear.x = self.linear_speed_hard_turn
            cmd.angular.z = -self.angular_speed_hard_turn

        return cmd

    def control_loop(self):
        if self.active_action is None:
            return

        if self.goal_reached:
            self.finish_action(reason='goal reached')
            return

        elapsed = self.elapsed_action_time()

        if elapsed >= self.action_duration:
            self.finish_action(
                reason=f'action duration reached ({elapsed:.2f} s)'
            )
            return

        if elapsed >= self.action_timeout:
            self.get_logger().warn(
                f'Action {self.active_action} timeout. Stopping robot.'
            )
            self.finish_action(reason='timeout')
            return

        cmd = self.get_cmd_for_action(self.active_action)
        self.cmd_vel_pub.publish(cmd)


def main():
    rclpy.init()

    node = RLMotionController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.publish_action_done(True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
