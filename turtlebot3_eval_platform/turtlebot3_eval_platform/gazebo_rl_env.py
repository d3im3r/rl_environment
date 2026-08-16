#!/usr/bin/env python3

import math
import time
from typing import Tuple, Optional, List, Dict, Any

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GazeboTurtleBot3Env(Node):
    """
    Entorno de simulación autocontenido para TurtleBot3 Burger en ROS 2 / Gazebo Classic.
    
    Implementa un bucle de control continuo y directo a '/cmd_vel'.
    El estado es un vector de 5 elementos normalizados:
    [d_front_norm, d_left_norm, d_right_norm, theta_goal_norm, d_goal_norm]
    """

    def __init__(
        self,
        stage: int = 1,
        max_steps: int = 100,
        step_dt: float = 0.1,  # Duración del paso físico (10 Hz)
        goal_list: Optional[List[Tuple[float, float]]] = None,
        robot_start_list: Optional[List[Tuple[float, float, float]]] = None,
        max_sensor_range: float = 3.5,
        max_goal_distance: float = 5.0,
        collision_distance_norm: float = 0.10,  # ~35 cm
        goal_tolerance_m: float = 0.15,
    ):
        super().__init__('gazebo_turtlebot3_continuous_env')

        # Configuración
        self.stage = stage
        self.max_steps = max_steps
        self.step_dt = step_dt
        self.max_sensor_range = max_sensor_range
        self.max_goal_distance = max_goal_distance
        self.collision_distance_norm = collision_distance_norm
        self.goal_tolerance_m = goal_tolerance_m

        # Listas de metas e inicios del robot
        self.goal_list = goal_list if goal_list is not None else [(1.5, 0.0)]
        self.robot_start_list = robot_start_list if robot_start_list is not None else [(-1.5, 0.0, 0.0)]
        
        self.current_goal = self.goal_list[0]
        self.robot_start = self.robot_start_list[0]

        # Estado del sensorizado
        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.step_count = 0

        # Publicador de velocidad
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Suscriptores
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Clientes de servicios de Gazebo
        self.reset_sim_client = self.create_client(Empty, '/reset_simulation')
        self.set_entity_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        self.get_logger().info('Esperando servicios de Gazebo...')
        self.reset_sim_client.wait_for_service()
        self.set_entity_client.wait_for_service()
        self.get_logger().info('Entorno Gazebo TurtleBot3 listo para control continuo.')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = yaw_from_quaternion(q)

    def spin_some(self, duration: float):
        """Ejecuta los callbacks de ROS 2 durante un tiempo determinado."""
        end_time = time.time() + duration
        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def get_distance_at_angle(self, target_angle: float) -> float:
        if self.latest_scan is None:
            return self.max_sensor_range

        scan = self.latest_scan
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment

        # Normalizar el ángulo target_angle al rango de lectura del LiDAR
        if angle_min >= 0.0 and angle_max > math.pi:
            while target_angle < 0.0:
                target_angle += 2.0 * math.pi
            while target_angle >= 2.0 * math.pi:
                target_angle -= 2.0 * math.pi
        else:
            while target_angle > math.pi:
                target_angle -= 2.0 * math.pi
            while target_angle < -math.pi:
                target_angle += 2.0 * math.pi

        if target_angle < angle_min or target_angle > angle_max:
            return self.max_sensor_range

        index = int(round((target_angle - angle_min) / angle_increment))
        index = max(0, min(index, len(scan.ranges) - 1))

        distance = scan.ranges[index]
        if math.isinf(distance) or math.isnan(distance) or distance <= 0.0:
            return self.max_sensor_range

        return float(min(distance, self.max_sensor_range))

    def compute_state(self) -> np.ndarray:
        # 1. Distancias del LiDAR a 0 (frente), 45 (izq) y -45 (der) grados
        d_front = self.get_distance_at_angle(0.0)
        d_left = self.get_distance_at_angle(math.pi / 4.0)
        d_right = self.get_distance_at_angle(-math.pi / 4.0)

        d_front_norm = d_front / self.max_sensor_range
        d_left_norm = d_left / self.max_sensor_range
        d_right_norm = d_right / self.max_sensor_range

        # 2. Información del objetivo
        dx = self.current_goal[0] - self.robot_x
        dy = self.current_goal[1] - self.robot_y
        d_goal = math.hypot(dx, dy)

        theta_goal = math.atan2(dy, dx)
        theta_error = normalize_angle(theta_goal - self.robot_yaw)

        theta_goal_norm = theta_error / math.pi
        d_goal_norm = min(d_goal / self.max_goal_distance, 1.0)

        return np.array([
            float(d_front_norm),
            float(d_left_norm),
            float(d_right_norm),
            float(theta_goal_norm),
            float(d_goal_norm)
        ], dtype=np.float32)

    def set_entity_pose(self, name: str, x: float, y: float, z: float, yaw: float):
        state = EntityState()
        state.name = name
        state.reference_frame = 'world'
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z)
        state.pose.orientation = yaw_to_quaternion(yaw)

        # Forzar detención física al teletransportar
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        req = SetEntityState.Request()
        req.state = state

        future = self.set_entity_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None:
            self.get_logger().error(f"Fallo crítico al llamar /gazebo/set_entity_state para {name}")
        elif not result.success:
            self.get_logger().warn(f"No se pudo mover la entidad {name}: {result.status_message}")
        else:
            self.get_logger().info(f"Entidad {name} reposicionada con éxito a ({x:.2f}, {y:.2f})")

    def reset_simulation(self):
        req = Empty.Request()
        future = self.reset_sim_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def reset(self, episode_index: Optional[int] = None) -> np.ndarray:
        # Detener robot
        self.stop_robot()
        self.step_count = 0

        # Selección aleatoria o indexada de inicio y meta
        if episode_index is not None:
            self.current_goal = self.goal_list[episode_index % len(self.goal_list)]
            self.robot_start = self.robot_start_list[episode_index % len(self.robot_start_list)]
        else:
            self.current_goal = self.goal_list[np.random.randint(len(self.goal_list))]
            self.robot_start = self.robot_start_list[np.random.randint(len(self.robot_start_list))]

        # Llamar servicios de Gazebo
        self.reset_simulation()
        self.spin_some(0.1)

        # Ubicar robot y meta
        self.set_entity_pose('turtlebot3', self.robot_start[0], self.robot_start[1], 0.01, self.robot_start[2])
        self.set_entity_pose('goal_marker', self.current_goal[0], self.current_goal[1], 0.03, 0.0)

        # Esperar estabilización física y sensorización
        self.spin_some(0.5)

        # Asegurar que se leen datos actualizados de odometría y scan
        while self.latest_scan is None or self.latest_odom is None:
            self.spin_some(0.05)

        return self.compute_state()

    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """
        Aplica la acción de control continuo [linear_vel, angular_vel],
        espera step_dt, y retorna el nuevo estado y la recompensa.
        """
        self.step_count += 1
        
        # 1. Aplicar la acción directa
        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.angular.z = float(action[1])
        self.cmd_vel_pub.publish(cmd)

        # 2. Esperar duración del paso físico
        self.spin_some(self.step_dt)

        # 3. Leer nuevo estado
        next_state = self.compute_state()

        # 4. Verificar condiciones de parada
        d_goal_actual = math.hypot(self.current_goal[0] - self.robot_x, self.current_goal[1] - self.robot_y)
        goal_reached = d_goal_actual <= self.goal_tolerance_m
        
        min_lidar = min(float(next_state[0]), float(next_state[1]), float(next_state[2]))
        collision = min_lidar <= self.collision_distance_norm

        if goal_reached:
            collision = False  # Prioridad a la meta en caso de cercanía

        timeout = self.step_count >= self.max_steps
        done = goal_reached or collision or timeout

        # 5. Calcular recompensa
        reward = self.compute_reward(next_state, action, done, collision, goal_reached, timeout)

        info = {
            "step": self.step_count,
            "robot_x": self.robot_x,
            "robot_y": self.robot_y,
            "robot_yaw": self.robot_yaw,
            "goal_x": self.current_goal[0],
            "goal_y": self.current_goal[1],
            "collision": collision,
            "goal_reached": goal_reached,
            "timeout": timeout,
            "distance_to_goal": d_goal_actual
        }

        if done:
            self.stop_robot()

        return next_state, reward, done, info

    def compute_reward(
        self,
        state: np.ndarray,
        action: np.ndarray,
        done: bool,
        collision: bool,
        goal_reached: bool,
        timeout: bool
    ) -> float:
        # Recompensa adaptada para pasos cortos continuos
        d_goal_norm = float(state[4])
        theta_goal_norm = float(state[3])

        # Penalización base por paso
        reward = -0.01  # Pequeño costo para motivar rapidez

        # Penalización angular por mala orientación
        reward -= 0.05 * abs(theta_goal_norm)

        # Recompensa por orientarse y avanzar
        # Premia velocidades lineales altas si está alineado
        is_aligned = abs(theta_goal_norm) < 0.15
        if is_aligned and action[0] > 0.0:
            reward += 0.08 * (action[0] / 0.18)

        # Penalización por giros innecesarios o bruscos
        reward -= 0.02 * (abs(action[1]) / 1.0)

        # Recompensas de fin de episodio
        if goal_reached:
            reward += 100.0
        elif collision:
            reward -= 50.0
        elif timeout:
            reward -= 20.0

        return float(reward)
