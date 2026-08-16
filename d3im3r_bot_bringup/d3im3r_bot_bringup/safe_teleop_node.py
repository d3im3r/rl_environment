#!/usr/bin/env python3

from threading import Lock
from typing import Optional, Set, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from pynput import keyboard


HELP_TEXT = """
d3im3r_bot safe teleop
----------------------

Modo:
    Mantener tecla presionada para mover.
    Soltar tecla para detener.
    No acumula comandos.
    Publica /cmd_vel a rata fija.

Movimiento:
    w : avanzar
    s : retroceder
    a : girar izquierda
    d : girar derecha

Combinaciones:
    w + a : avanzar girando izquierda
    w + d : avanzar girando derecha
    s + a : retroceder girando izquierda
    s + d : retroceder girando derecha

Otros:
    espacio : parada total
    h       : ayuda
    q       : salir
"""


class SafeTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('d3im3r_safe_teleop')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('tof_topic', '/tof_distances_m')
        self.declare_parameter('safety_status_topic', '/safety_status')

        self.declare_parameter('publish_rate_hz', 30.0)

        self.declare_parameter('linear_speed_m_s', 0.5)
        self.declare_parameter('reverse_speed_m_s', 0.5)
        self.declare_parameter('angular_speed_rad_s', 0.5)
        self.declare_parameter('angular_speed_turning_rad_s', 0.5)

        self.declare_parameter('max_linear_m_s', 0.5)
        self.declare_parameter('max_angular_rad_s', 1.0)

        self.declare_parameter('enable_pc_safety_filter', False)

        self.declare_parameter('front_stop_m', 0.12)
        self.declare_parameter('front_slow_m', 0.30)
        self.declare_parameter('side_stop_m', 0.10)
        self.declare_parameter('side_slow_m', 0.18)

        self.declare_parameter('suppress_keys', False)
        self.declare_parameter('publish_zero_on_exit', True)

        self.cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic'
        ).get_parameter_value().string_value

        self.tof_topic = self.get_parameter(
            'tof_topic'
        ).get_parameter_value().string_value

        self.safety_status_topic = self.get_parameter(
            'safety_status_topic'
        ).get_parameter_value().string_value

        self.publish_rate_hz = self.get_parameter(
            'publish_rate_hz'
        ).get_parameter_value().double_value

        self.linear_speed = self.get_parameter(
            'linear_speed_m_s'
        ).get_parameter_value().double_value

        self.reverse_speed = self.get_parameter(
            'reverse_speed_m_s'
        ).get_parameter_value().double_value

        self.angular_speed = self.get_parameter(
            'angular_speed_rad_s'
        ).get_parameter_value().double_value

        self.angular_speed_turning = self.get_parameter(
            'angular_speed_turning_rad_s'
        ).get_parameter_value().double_value

        self.max_linear = self.get_parameter(
            'max_linear_m_s'
        ).get_parameter_value().double_value

        self.max_angular = self.get_parameter(
            'max_angular_rad_s'
        ).get_parameter_value().double_value

        self.enable_pc_safety_filter = self.get_parameter(
            'enable_pc_safety_filter'
        ).get_parameter_value().bool_value

        self.front_stop_m = self.get_parameter(
            'front_stop_m'
        ).get_parameter_value().double_value

        self.front_slow_m = self.get_parameter(
            'front_slow_m'
        ).get_parameter_value().double_value

        self.side_stop_m = self.get_parameter(
            'side_stop_m'
        ).get_parameter_value().double_value

        self.side_slow_m = self.get_parameter(
            'side_slow_m'
        ).get_parameter_value().double_value

        self.suppress_keys = self.get_parameter(
            'suppress_keys'
        ).get_parameter_value().bool_value

        self.publish_zero_on_exit = self.get_parameter(
            'publish_zero_on_exit'
        ).get_parameter_value().bool_value

        self.lock = Lock()
        self.active_keys: Set[str] = set()

        self.last_tof: Optional[Vector3] = None
        self.last_safety_status: Optional[Vector3] = None

        self.shutdown_requested = False

        self.last_v: Optional[float] = None
        self.last_w: Optional[float] = None

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self.tof_sub = self.create_subscription(
            Vector3,
            self.tof_topic,
            self.tof_callback,
            10,
        )

        self.safety_sub = self.create_subscription(
            Vector3,
            self.safety_status_topic,
            self.safety_status_callback,
            10,
        )

        timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release,
            suppress=self.suppress_keys,
        )
        self.keyboard_listener.start()

        self.get_logger().info('d3im3r safe teleop started.')
        self.get_logger().info(HELP_TEXT)
        self.get_logger().info(f'publish_rate_hz={self.publish_rate_hz}')
        self.get_logger().info(f'suppress_keys={self.suppress_keys}')
        self.get_logger().info(
            f'linear={self.linear_speed}, angular={self.angular_speed}'
        )
        self.get_logger().info(
            f'enable_pc_safety_filter={self.enable_pc_safety_filter}'
        )

    def tof_callback(self, msg: Vector3) -> None:
        self.last_tof = msg

    def safety_status_callback(self, msg: Vector3) -> None:
        self.last_safety_status = msg

    def key_to_char(self, key) -> Optional[str]:
        try:
            return key.char
        except AttributeError:
            if key == keyboard.Key.space:
                return 'space'
            return None

    def on_key_press(self, key) -> None:
        key_char = self.key_to_char(key)

        if key_char is None:
            return

        with self.lock:
            if key_char in ['w', 's', 'a', 'd']:
                self.active_keys.add(key_char)

            elif key_char == 'space':
                self.active_keys.clear()
                self.publish_stop()
                self.get_logger().info('Full stop.')

            elif key_char == 'h':
                self.get_logger().info(HELP_TEXT)

            elif key_char == 'q':
                self.active_keys.clear()
                self.shutdown_requested = True
                self.get_logger().info('Shutdown requested.')

    def on_key_release(self, key) -> None:
        key_char = self.key_to_char(key)

        if key_char is None:
            return

        with self.lock:
            if key_char in self.active_keys:
                self.active_keys.remove(key_char)

        if key_char in ['w', 's', 'a', 'd']:
            self.publish_stop()

    def get_keys_snapshot(self) -> Set[str]:
        with self.lock:
            return set(self.active_keys)

    def compute_command(self) -> Tuple[float, float, str]:
        keys = self.get_keys_snapshot()

        forward = 'w' in keys
        backward = 's' in keys
        left = 'a' in keys
        right = 'd' in keys

        v = 0.0
        w = 0.0

        if forward and not backward:
            v = self.linear_speed
        elif backward and not forward:
            v = -self.reverse_speed

        moving_linear = abs(v) > 0.001

        if moving_linear:
            angular_value = self.angular_speed_turning
        else:
            angular_value = self.angular_speed

        if left and not right:
            w = angular_value
        elif right and not left:
            w = -angular_value

        v = self.clamp(v, -self.max_linear, self.max_linear)
        w = self.clamp(w, -self.max_angular, self.max_angular)

        pressed = ''.join(key for key in ['w', 's', 'a', 'd'] if key in keys)

        return v, w, pressed

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    @staticmethod
    def valid_distance(value: float) -> bool:
        return value > 0.0

    def apply_pc_safety_filter(self, v: float, w: float) -> Tuple[float, float]:
        if not self.enable_pc_safety_filter:
            return v, w

        if self.last_tof is None:
            return v, w

        front = float(self.last_tof.x)
        left = float(self.last_tof.y)
        right = float(self.last_tof.z)

        if v > 0.0 and self.valid_distance(front):
            if front <= self.front_stop_m:
                v = 0.0
            elif front <= self.front_slow_m:
                span = self.front_slow_m - self.front_stop_m
                if span > 0.0:
                    factor = (front - self.front_stop_m) / span
                    factor = self.clamp(factor, 0.20, 1.0)
                    v *= factor

        if w > 0.0 and self.valid_distance(left):
            if left <= self.side_stop_m:
                w = 0.0
            elif left <= self.side_slow_m:
                span = self.side_slow_m - self.side_stop_m
                if span > 0.0:
                    factor = (left - self.side_stop_m) / span
                    factor = self.clamp(factor, 0.20, 1.0)
                    w *= factor

        if w < 0.0 and self.valid_distance(right):
            if right <= self.side_stop_m:
                w = 0.0
            elif right <= self.side_slow_m:
                span = self.side_slow_m - self.side_stop_m
                if span > 0.0:
                    factor = (right - self.side_stop_m) / span
                    factor = self.clamp(factor, 0.20, 1.0)
                    w *= factor

        return v, w

    def timer_callback(self) -> None:
        if self.shutdown_requested:
            self.publish_stop()

            if self.keyboard_listener is not None:
                self.keyboard_listener.stop()

            rclpy.shutdown()
            return

        v, w, pressed = self.compute_command()
        v, w = self.apply_pc_safety_filter(v, w)

        self.publish_cmd_vel(v, w)

        if self.command_changed(v, w):
            self.print_status(v, w, pressed)

        self.last_v = v
        self.last_w = w

    def command_changed(self, v: float, w: float) -> bool:
        if self.last_v is None or self.last_w is None:
            return True

        return abs(v - self.last_v) > 1e-4 or abs(w - self.last_w) > 1e-4

    def publish_cmd_vel(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)

    def publish_stop(self) -> None:
        self.publish_cmd_vel(0.0, 0.0)

    def print_status(self, v: float, w: float, pressed: str) -> None:
        safety_info = ''

        if self.last_safety_status is not None:
            limited = int(self.last_safety_status.x)
            emergency = int(self.last_safety_status.y)
            reason = int(self.last_safety_status.z)

            safety_info = (
                f' | safety limited={limited} '
                f'emergency={emergency} reason={reason}'
            )

        self.get_logger().info(
            f'keys=[{pressed}] cmd: v={v:.3f} m/s, '
            f'w={w:.3f} rad/s{safety_info}'
        )

    def destroy_node(self) -> bool:
        if self.publish_zero_on_exit:
            self.publish_stop()

        if hasattr(self, 'keyboard_listener') and self.keyboard_listener is not None:
            self.keyboard_listener.stop()

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = SafeTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()