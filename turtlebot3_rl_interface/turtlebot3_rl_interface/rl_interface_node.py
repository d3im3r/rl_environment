import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32MultiArray, Bool


class TurtleBot3RLInterface(Node):

    def __init__(self):
        super().__init__('turtlebot3_rl_interface')

        # ============================================================
        # Parámetros ROS 2
        # ============================================================

        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 1.0)

        self.declare_parameter('max_sensor_range', 3.5)
        self.declare_parameter('max_goal_distance', 5.0)
        self.declare_parameter('goal_tolerance', 0.10)

        self.declare_parameter('forward_distance', 0.20)
        self.declare_parameter('rotation_angle', math.pi / 12.0)

        self.declare_parameter('linear_speed', 0.08)
        self.declare_parameter('angular_speed', 0.25)

        self.declare_parameter('front_angle', 0.0)
        self.declare_parameter('left_angle', math.pi / 4.0)
        self.declare_parameter('right_angle', -math.pi / 4.0)

        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('state_publish_period', 0.10)

        # ============================================================
        # Lectura de parámetros
        # ============================================================

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        self.max_sensor_range = self.get_parameter('max_sensor_range').value
        self.max_goal_distance = self.get_parameter('max_goal_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        self.forward_distance = self.get_parameter('forward_distance').value
        self.rotation_angle = self.get_parameter('rotation_angle').value

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        self.front_angle = self.get_parameter('front_angle').value
        self.left_angle = self.get_parameter('left_angle').value
        self.right_angle = self.get_parameter('right_angle').value

        self.control_period = self.get_parameter('control_period').value
        self.state_publish_period = self.get_parameter('state_publish_period').value

        # ============================================================
        # Variables internas
        # ============================================================

        self.latest_scan = None
        self.odom_received = False
        self.scan_received = False

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.executing_action = False
        self.current_action = None

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0

        self.goal_reached = False

        # ============================================================
        # Subscriptores
        # ============================================================

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.action_sub = self.create_subscription(
            Int32,
            '/rl_action',
            self.action_callback,
            10
        )

        # ============================================================
        # Publicadores
        # ============================================================

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Estado completo:
        # [d_front_norm, d_left_norm, d_right_norm, theta_goal_norm, d_goal_norm]
        self.state_pub = self.create_publisher(
            Float32MultiArray,
            '/rl_state',
            10
        )

        # Distancias reales:
        # [d_front, d_left, d_right]
        self.distances_pub = self.create_publisher(
            Float32MultiArray,
            '/rl_distances',
            10
        )

        # Distancias normalizadas:
        # [d_front_norm, d_left_norm, d_right_norm]
        self.norm_distances_pub = self.create_publisher(
            Float32MultiArray,
            '/rl_distances_norm',
            10
        )

        # Indica que una acción discreta terminó
        self.done_pub = self.create_publisher(
            Bool,
            '/rl_action_done',
            10
        )

        # Indica si el robot alcanzó la meta
        self.goal_reached_pub = self.create_publisher(
            Bool,
            '/rl_goal_reached',
            10
        )

        # ============================================================
        # Timers
        # ============================================================

        self.control_timer = self.create_timer(
            self.control_period,
            self.control_loop
        )

        # Publicación continua de estado, distancias y bandera de meta
        self.state_timer = self.create_timer(
            self.state_publish_period,
            self.publish_observations
        )

        self.get_logger().info('Nodo turtlebot3_rl_interface iniciado correctamente.')
        self.print_parameters()

    # ============================================================
    # Impresión de parámetros
    # ============================================================

    def print_parameters(self):
        self.get_logger().info('Parámetros cargados:')
        self.get_logger().info(f'goal_x: {self.goal_x}')
        self.get_logger().info(f'goal_y: {self.goal_y}')
        self.get_logger().info(f'max_sensor_range: {self.max_sensor_range}')
        self.get_logger().info(f'max_goal_distance: {self.max_goal_distance}')
        self.get_logger().info(f'goal_tolerance: {self.goal_tolerance}')
        self.get_logger().info(f'forward_distance: {self.forward_distance}')
        self.get_logger().info(f'rotation_angle: {self.rotation_angle}')
        self.get_logger().info(f'linear_speed: {self.linear_speed}')
        self.get_logger().info(f'angular_speed: {self.angular_speed}')
        self.get_logger().info(f'front_angle: {self.front_angle}')
        self.get_logger().info(f'left_angle: {self.left_angle}')
        self.get_logger().info(f'right_angle: {self.right_angle}')
        self.get_logger().info(f'control_period: {self.control_period}')
        self.get_logger().info(f'state_publish_period: {self.state_publish_period}')

    # ============================================================
    # Callbacks
    # ============================================================

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_received = True

    def odom_callback(self, msg):
        self.odom_received = True

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.theta = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def action_callback(self, msg):
        if self.check_goal_reached():
            self.stop_robot()
            self.get_logger().info(
                'La meta ya fue alcanzada. No se aceptan más acciones.'
            )
            return

        if self.executing_action:
            self.get_logger().warn(
                'Ya hay una acción en ejecución. Acción ignorada.'
            )
            return

        action = msg.data

        if action not in [0, 1, 2]:
            self.get_logger().warn(f'Acción inválida recibida: {action}')
            return

        self.current_action = action
        self.executing_action = True

        self.start_x = self.x
        self.start_y = self.y
        self.start_theta = self.theta

        self.get_logger().info(f'Iniciando acción discreta: {action}')

    # ============================================================
    # Bucle de control de acciones discretas
    # ============================================================

    def control_loop(self):
        if self.check_goal_reached():
            self.stop_robot()
            self.executing_action = False
            self.current_action = None
            return

        if not self.executing_action:
            return

        cmd = Twist()

        if self.current_action == 0:
            self.execute_forward_action(cmd)

        elif self.current_action == 1:
            self.execute_ccw_rotation_action(cmd)

        elif self.current_action == 2:
            self.execute_cw_rotation_action(cmd)

    # ============================================================
    # Acciones discretas
    # ============================================================

    def execute_forward_action(self, cmd):
        distance_traveled = math.sqrt(
            (self.x - self.start_x) ** 2 +
            (self.y - self.start_y) ** 2
        )

        if distance_traveled < self.forward_distance:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
        else:
            self.finish_action()

    def execute_ccw_rotation_action(self, cmd):
        delta_theta = self.angle_difference(self.theta, self.start_theta)

        if delta_theta < self.rotation_angle:
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(cmd)
        else:
            self.finish_action()

    def execute_cw_rotation_action(self, cmd):
        delta_theta = self.angle_difference(self.theta, self.start_theta)

        if abs(delta_theta) < self.rotation_angle:
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed
            self.cmd_vel_pub.publish(cmd)
        else:
            self.finish_action()

    # ============================================================
    # Finalización de acción
    # ============================================================

    def finish_action(self):
        self.stop_robot()

        self.executing_action = False
        self.current_action = None

        self.check_goal_reached()

        done_msg = Bool()
        done_msg.data = True
        self.done_pub.publish(done_msg)

        state = self.compute_state()
        self.get_logger().info(f'Acción finalizada. Estado actual: {state}')

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    # ============================================================
    # Publicación continua de observaciones
    # ============================================================

    def publish_observations(self):
        """
        Publica continuamente:
        - /rl_state
        - /rl_distances
        - /rl_distances_norm
        - /rl_goal_reached
        """

        self.check_goal_reached()

        d_front, d_left, d_right = self.compute_laser_distances()

        d_front_norm = self.normalize_distance(d_front, self.max_sensor_range)
        d_left_norm = self.normalize_distance(d_left, self.max_sensor_range)
        d_right_norm = self.normalize_distance(d_right, self.max_sensor_range)

        distances_msg = Float32MultiArray()
        distances_msg.data = [
            float(d_front),
            float(d_left),
            float(d_right)
        ]
        self.distances_pub.publish(distances_msg)

        norm_distances_msg = Float32MultiArray()
        norm_distances_msg.data = [
            float(d_front_norm),
            float(d_left_norm),
            float(d_right_norm)
        ]
        self.norm_distances_pub.publish(norm_distances_msg)

        state = self.compute_state()

        state_msg = Float32MultiArray()
        state_msg.data = state
        self.state_pub.publish(state_msg)

        self.publish_goal_reached_flag()

    # ============================================================
    # Construcción del estado
    # ============================================================

    def compute_state(self):
        d_front, d_left, d_right = self.compute_laser_distances()

        d_front_norm = self.normalize_distance(d_front, self.max_sensor_range)
        d_left_norm = self.normalize_distance(d_left, self.max_sensor_range)
        d_right_norm = self.normalize_distance(d_right, self.max_sensor_range)

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        d_goal = self.compute_goal_distance()

        theta_goal = math.atan2(dy, dx)
        theta_error = self.angle_difference(theta_goal, self.theta)

        theta_goal_norm = theta_error / math.pi
        d_goal_norm = self.normalize_distance(
            d_goal,
            self.max_goal_distance
        )

        state = [
            float(d_front_norm),
            float(d_left_norm),
            float(d_right_norm),
            float(theta_goal_norm),
            float(d_goal_norm)
        ]

        return state

    # ============================================================
    # Distancia y bandera de llegada a la meta
    # ============================================================

    def compute_goal_distance(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        return math.sqrt(dx ** 2 + dy ** 2)

    def check_goal_reached(self):
        if not self.odom_received:
            return False

        d_goal = self.compute_goal_distance()

        if d_goal <= self.goal_tolerance:
            if not self.goal_reached:
                self.get_logger().info(
                    f'Meta alcanzada. Distancia a la meta: {d_goal:.3f} m'
                )

            self.goal_reached = True
        else:
            self.goal_reached = False

        return self.goal_reached

    def publish_goal_reached_flag(self):
        msg = Bool()
        msg.data = self.goal_reached
        self.goal_reached_pub.publish(msg)

    # ============================================================
    # Lecturas puntuales del LiDAR
    # ============================================================

    def compute_laser_distances(self):
        if self.latest_scan is None:
            return (
                self.max_sensor_range,
                self.max_sensor_range,
                self.max_sensor_range
            )

        d_front = self.get_distance_at_angle(self.front_angle)
        d_left = self.get_distance_at_angle(self.left_angle)
        d_right = self.get_distance_at_angle(self.right_angle)

        return d_front, d_left, d_right

    def get_distance_at_angle(self, target_angle):
        scan = self.latest_scan

        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment

        target_angle = self.normalize_angle_to_scan_range(
            target_angle,
            angle_min,
            angle_max
        )

        if target_angle < angle_min or target_angle > angle_max:
            self.get_logger().warn(
                f'Ángulo {target_angle:.3f} fuera del rango del LiDAR '
                f'[{angle_min:.3f}, {angle_max:.3f}]'
            )
            return self.max_sensor_range

        index = int(round((target_angle - angle_min) / angle_increment))
        index = max(0, min(index, len(scan.ranges) - 1))

        distance = scan.ranges[index]

        if math.isinf(distance) or math.isnan(distance):
            distance = self.max_sensor_range

        if distance <= 0.0:
            distance = self.max_sensor_range

        if distance > self.max_sensor_range:
            distance = self.max_sensor_range

        return float(distance)

    def normalize_angle_to_scan_range(self, angle, angle_min, angle_max):
        """
        Ajusta el ángulo objetivo al rango angular real del LaserScan.

        Algunos LiDAR publican en:
        - [-pi, pi]
        - [0, 2*pi]

        Esta función permite que right_angle = -pi/4 funcione aunque
        el sensor esté publicado en [0, 2*pi].
        """

        # Caso común: LaserScan en [0, 2*pi]
        if angle_min >= 0.0 and angle_max > math.pi:
            while angle < 0.0:
                angle += 2.0 * math.pi

            while angle >= 2.0 * math.pi:
                angle -= 2.0 * math.pi

            return angle

        # Caso común: LaserScan en [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # ============================================================
    # Funciones auxiliares
    # ============================================================

    def normalize_distance(self, distance, max_distance):
        if max_distance <= 0.0:
            return 0.0

        value = distance / max_distance
        value = max(0.0, min(value, 1.0))

        return value

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

    def angle_difference(self, angle_a, angle_b):
        """
        Retorna angle_a - angle_b normalizado en [-pi, pi].
        """

        diff = angle_a - angle_b

        while diff > math.pi:
            diff -= 2.0 * math.pi

        while diff < -math.pi:
            diff += 2.0 * math.pi

        return diff


def main(args=None):
    rclpy.init(args=args)

    node = TurtleBot3RLInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()