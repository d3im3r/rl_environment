#!/usr/bin/env python3

import math
import time
from typing import Optional, Tuple, Dict, Any, List

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


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


def point_to_segment_distance(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float
) -> float:
    abx = bx - ax
    aby = by - ay

    apx = px - ax
    apy = py - ay

    ab_len_sq = abx * abx + aby * aby

    if ab_len_sq == 0.0:
        return math.hypot(px - ax, py - ay)

    t = (apx * abx + apy * aby) / ab_len_sq
    t = max(0.0, min(1.0, t))

    closest_x = ax + t * abx
    closest_y = ay + t * aby

    return math.hypot(px - closest_x, py - closest_y)


class GazeboTurtleBot3Env(Node):
    def __init__(
        self,
        stage: int = 1,
        max_steps: int = 80,
        robot_start: Tuple[float, float, float] = (-1.5, 0.0, 0.0),
        robot_start_list: Optional[List[Tuple[float, float, float]]] = None,
        goal_list: Optional[List[Tuple[float, float]]] = None,
        action_execution_time: float = 5.0,
        state_timeout: float = 5.0,
        max_goal_distance: float = 5.0,
        collision_distance_norm: float = 0.10,
        goal_tolerance_norm: float = 0.05,
        reward_progress_gain: float = 8.0,
        reward_step_penalty: float = 0.05,
        reward_heading_penalty: float = 0.10,
        reward_collision: float = -100.0,
        reward_timeout: float = -80.0,
        reward_goal: float = 150.0,
        reward_backward_penalty: float = 0.50,
        reward_turn_penalty: float = 0.03,
        reward_unnecessary_turn_penalty: float = 0.20,
        reward_forward_aligned_bonus: float = 0.30,
        reward_alignment_gain: float = 0.50,
        reward_oscillation_penalty: float = 0.30,
        aligned_threshold: float = 0.08,
        bad_heading_threshold: float = 0.25,
        reward_correct_turn_bonus: float = 0.35,
        reward_wrong_turn_penalty: float = 0.45,
        reward_forward_misaligned_penalty: float = 0.60,
        reward_heading_improvement_gain: float = 1.00
    ):
        super().__init__('gazebo_turtlebot3_env')

        self.stage = int(stage)
        self.max_steps = int(max_steps)

        self.robot_start = robot_start

        if robot_start_list is None:
            robot_start_list = [robot_start]

        self.robot_start_list = robot_start_list

        if goal_list is None:
            goal_list = [(1.5, 0.0)]

        self.goal_list = goal_list
        self.current_goal = goal_list[0]

        self.action_execution_time = float(action_execution_time)
        self.state_timeout = float(state_timeout)
        self.max_goal_distance = float(max_goal_distance)

        self.collision_distance_norm = float(collision_distance_norm)
        self.goal_tolerance_norm = float(goal_tolerance_norm)
        self.goal_tolerance_m = self.goal_tolerance_norm * self.max_goal_distance

        self.reward_progress_gain = float(reward_progress_gain)
        self.reward_step_penalty = float(reward_step_penalty)
        self.reward_heading_penalty = float(reward_heading_penalty)
        self.reward_collision = float(reward_collision)
        self.reward_timeout = float(reward_timeout)
        self.reward_goal = float(reward_goal)
        self.reward_backward_penalty = float(reward_backward_penalty)

        self.reward_turn_penalty = float(reward_turn_penalty)
        self.reward_unnecessary_turn_penalty = float(reward_unnecessary_turn_penalty)
        self.reward_forward_aligned_bonus = float(reward_forward_aligned_bonus)
        self.reward_alignment_gain = float(reward_alignment_gain)
        self.reward_oscillation_penalty = float(reward_oscillation_penalty)

        self.aligned_threshold = float(aligned_threshold)
        self.bad_heading_threshold = float(bad_heading_threshold)

        # Recompensa direccional adicional.
        # Estos valores se dejan como parámetros internos con defaults para no
        # romper compatibilidad con train_dqn_ros.py.
        self.reward_correct_turn_bonus = float(reward_correct_turn_bonus)
        self.reward_wrong_turn_penalty = float(reward_wrong_turn_penalty)
        self.reward_forward_misaligned_penalty = float(
            reward_forward_misaligned_penalty
        )
        self.reward_heading_improvement_gain = float(
            reward_heading_improvement_gain
        )

        self.raw_state: Optional[np.ndarray] = None
        self.current_state: Optional[np.ndarray] = None

        self.goal_reached_topic: bool = False
        self.action_done: bool = True
        self.odom_msg: Optional[Odometry] = None

        self.step_count = 0
        self.previous_d_goal_norm: Optional[float] = None
        self.last_action: Optional[int] = None

        # El entorno solo publica acciones discretas.
        # NO publica /cmd_vel para evitar conflicto con rl_motion_controller.
        self.rl_action_pub = self.create_publisher(
            Int32,
            '/rl_action',
            10
        )

        self.create_subscription(
            Float32MultiArray,
            '/rl_state',
            self.rl_state_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/rl_goal_reached',
            self.goal_reached_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/rl_action_done',
            self.action_done_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.reset_sim_client = self.create_client(
            Empty,
            '/reset_simulation'
        )

        self.set_entity_client = self.create_client(
            SetEntityState,
            '/gazebo/set_entity_state'
        )

        self.get_logger().info('Waiting for /reset_simulation service...')
        self.reset_sim_client.wait_for_service()

        self.get_logger().info('Waiting for /gazebo/set_entity_state service...')
        self.set_entity_client.wait_for_service()

        self.get_logger().info('GazeboTurtleBot3Env ready.')
        self.get_logger().info(
            f'stage={self.stage}, max_steps={self.max_steps}, '
            f'action_timeout={self.action_execution_time:.2f} s, '
            f'goal_tolerance_m={self.goal_tolerance_m:.3f} m, '
            f'collision_distance_norm={self.collision_distance_norm:.3f}'
        )

    def rl_state_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 5:
            return

        self.raw_state = np.array(msg.data[:5], dtype=np.float32)

    def goal_reached_callback(self, msg: Bool):
        self.goal_reached_topic = bool(msg.data)

    def action_done_callback(self, msg: Bool):
        self.action_done = bool(msg.data)

    def odom_callback(self, msg: Odometry):
        self.odom_msg = msg

    def spin_some(self, duration: float = 0.1):
        end_time = time.time() + duration

        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_raw_state(self) -> np.ndarray:
        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.raw_state is not None:
                return self.raw_state.copy()

            if time.time() - start_time > self.state_timeout:
                raise TimeoutError('Timeout waiting for /rl_state.')

        raise RuntimeError('ROS interrupted while waiting for /rl_state.')

    def wait_for_odom(self) -> Odometry:
        self.odom_msg = None
        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.odom_msg is not None:
                return self.odom_msg

            if time.time() - start_time > self.state_timeout:
                raise TimeoutError('Timeout waiting for /odom.')

        raise RuntimeError('ROS interrupted while waiting for /odom.')

    def get_robot_pose(self) -> Tuple[float, float, float]:
        odom = self.wait_for_odom()
        pose = odom.pose.pose

        x = pose.position.x
        y = pose.position.y
        yaw = yaw_from_quaternion(pose.orientation)

        return float(x), float(y), float(yaw)

    def compute_goal_features_from_pose(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float
    ) -> Tuple[float, float, float]:
        goal_x, goal_y = self.current_goal

        dx = goal_x - robot_x
        dy = goal_y - robot_y

        d_goal_m = math.hypot(dx, dy)
        theta_goal = math.atan2(dy, dx)
        theta_error = normalize_angle(theta_goal - robot_yaw)

        theta_goal_norm = theta_error / math.pi
        d_goal_norm = max(
            0.0,
            min(d_goal_m / self.max_goal_distance, 1.0)
        )

        return theta_goal_norm, d_goal_norm, d_goal_m

    def build_corrected_state(self, raw_state: np.ndarray) -> np.ndarray:
        robot_x, robot_y, robot_yaw = self.get_robot_pose()

        theta_goal_norm, d_goal_norm, _ = self.compute_goal_features_from_pose(
            robot_x=robot_x,
            robot_y=robot_y,
            robot_yaw=robot_yaw
        )

        corrected_state = np.array(
            [
                float(raw_state[0]),
                float(raw_state[1]),
                float(raw_state[2]),
                float(theta_goal_norm),
                float(d_goal_norm),
            ],
            dtype=np.float32
        )

        return corrected_state

    def wait_for_state(self) -> np.ndarray:
        raw_state = self.wait_for_raw_state()
        corrected_state = self.build_corrected_state(raw_state)
        self.current_state = corrected_state.copy()

        return corrected_state

    def wait_for_action_done(self, timeout: float = 5.0) -> bool:
        """
        Espera a que rl_motion_controller reporte que terminó la acción actual.

        El flujo esperado es:
            publish_action()
            /rl_action_done = False
            /rl_action_done = True
        """
        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

            if self.action_done:
                return True

            if time.time() - start_time > timeout:
                self.get_logger().warn(
                    'Timeout waiting for /rl_action_done.'
                )
                return False

        return False

    def stop_robot(self):
        """
        El entorno NO publica /cmd_vel.

        El movimiento y la parada del robot son responsabilidad exclusiva de
        rl_motion_controller.py. Esto evita que existan dos publishers
        compitiendo sobre /cmd_vel.
        """
        return

    def reset_simulation(self):
        request = Empty.Request()
        future = self.reset_sim_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            raise RuntimeError('Failed to call /reset_simulation.')

        self.spin_some(0.3)

    def set_model_pose(
        self,
        model_name: str,
        x: float,
        y: float,
        z: float,
        yaw: float
    ):
        state = EntityState()
        state.name = model_name
        state.reference_frame = 'world'

        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z)
        state.pose.orientation = yaw_to_quaternion(yaw)

        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        request = SetEntityState.Request()
        request.state = state

        future = self.set_entity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()

        if result is None:
            raise RuntimeError(f'Failed to move entity: {model_name}')

        if not result.success:
            msg = getattr(result, 'status_message', 'SetEntityState service returned success=False')
            raise RuntimeError(
                f'Failed to move entity {model_name}: {msg}'
            )

        self.spin_some(0.1)

    def sample_goal(self, episode_index: Optional[int] = None) -> Tuple[float, float]:
        if episode_index is None:
            index = np.random.randint(0, len(self.goal_list))
        else:
            index = int(episode_index) % len(self.goal_list)

        return self.goal_list[index]

    def sample_robot_start(
        self,
        episode_index: Optional[int] = None
    ) -> Tuple[float, float, float]:
        if episode_index is None:
            index = np.random.randint(0, len(self.robot_start_list))
        else:
            index = int(episode_index) % len(self.robot_start_list)

        return self.robot_start_list[index]

    def reset(
        self,
        episode_index: Optional[int] = None,
        goal: Optional[Tuple[float, float]] = None
    ) -> np.ndarray:
        self.stop_robot()

        self.raw_state = None
        self.current_state = None
        self.goal_reached_topic = False
        self.action_done = True
        self.step_count = 0
        self.previous_d_goal_norm = None
        self.last_action = None

        if goal is None:
            goal = self.sample_goal(episode_index)

        self.current_goal = goal
        self.robot_start = self.sample_robot_start(episode_index)

        robot_x, robot_y, robot_yaw = self.robot_start
        goal_x, goal_y = self.current_goal

        self.reset_simulation()

        self.set_model_pose(
            model_name='turtlebot3',
            x=robot_x,
            y=robot_y,
            z=0.01,
            yaw=robot_yaw
        )

        self.set_model_pose(
            model_name='goal_marker',
            x=goal_x,
            y=goal_y,
            z=0.03,
            yaw=0.0
        )

        self.spin_some(0.8)

        state = self.wait_for_state()
        self.previous_d_goal_norm = float(state[4])

        return state

    def publish_action(self, action: int):
        """
        Publica una acción discreta.

        Importante:
        - El entorno marca localmente action_done = False.
        - El controlador debe publicar /rl_action_done = True al finalizar.
        """
        self.action_done = False

        msg = Int32()
        msg.data = int(action)
        self.rl_action_pub.publish(msg)

    def check_collision(self, state: np.ndarray) -> bool:
        d_front_norm = float(state[0])
        d_left_norm = float(state[1])
        d_right_norm = float(state[2])

        min_distance_norm = min(
            d_front_norm,
            d_left_norm,
            d_right_norm
        )

        return min_distance_norm <= self.collision_distance_norm

    def compute_reward(
        self,
        state: np.ndarray,
        next_state: np.ndarray,
        action: int,
        done: bool,
        collision: bool,
        goal_reached: bool,
        timeout: bool = False,
        previous_action: Optional[int] = None
    ) -> float:
        d_goal_prev = float(state[4])
        d_goal_next = float(next_state[4])

        theta_prev_signed = float(state[3])
        theta_next_signed = float(next_state[3])

        theta_prev = abs(theta_prev_signed)
        theta_next = abs(theta_next_signed)

        progress = d_goal_prev - d_goal_next
        heading_improvement = theta_prev - theta_next

        reward = 0.0

        # ------------------------------------------------------------
        # 1. Progreso hacia la meta
        # ------------------------------------------------------------
        reward += self.reward_progress_gain * progress

        if progress < 0.0:
            reward -= self.reward_backward_penalty

        # ------------------------------------------------------------
        # 2. Penalización por paso y por mala orientación
        # ------------------------------------------------------------
        reward -= self.reward_step_penalty
        reward -= self.reward_heading_penalty * theta_next

        # ------------------------------------------------------------
        # 3. Mejora angular explícita
        # Si reduce |theta_goal|, se premia.
        # Si aumenta |theta_goal|, se penaliza.
        # ------------------------------------------------------------
        reward += self.reward_heading_improvement_gain * heading_improvement

        # ------------------------------------------------------------
        # 4. Recompensa direccional según acción
        #
        # action 0 -> avanzar
        # action 1 -> girar izquierda
        # action 2 -> girar derecha
        #
        # theta > 0 -> meta hacia la izquierda
        # theta < 0 -> meta hacia la derecha
        # ------------------------------------------------------------
        goal_left = theta_next_signed > self.aligned_threshold
        goal_right = theta_next_signed < -self.aligned_threshold
        aligned = theta_next <= self.aligned_threshold
        badly_misaligned = theta_next >= self.bad_heading_threshold

        d_front_next = float(next_state[0])
        obstacle_shaping_threshold = 0.35 if self.stage >= 2 else 0.20  # 1.22m en Stage 2+, 0.70m en Stage 1

        if action == 0:
            # Solo otorgar bono de alineación si NO hay obstáculo cercano al frente (< 0.70m / norm 0.20)
            if aligned and d_front_next >= obstacle_shaping_threshold:
                reward += self.reward_forward_aligned_bonus
            else:
                reward -= self.reward_forward_misaligned_penalty * theta_next

            if badly_misaligned:
                reward -= self.reward_forward_misaligned_penalty

            # Penalización por insistir en avanzar recto hacia un obstáculo cercano (< 0.70m / norm 0.20)
            if d_front_next < obstacle_shaping_threshold:
                reward -= 1.5 * (obstacle_shaping_threshold - d_front_next)

        elif action in (1, 3):
            # Si hay obstáculo al frente (< 0.70m), premiar el giro de evitación y eximir penalización innecesaria
            if d_front_next < obstacle_shaping_threshold:
                reward += 0.30
            else:
                reward -= self.reward_turn_penalty
                if aligned:
                    reward -= self.reward_unnecessary_turn_penalty

            if goal_left:
                reward += self.reward_correct_turn_bonus * theta_next
            elif goal_right:
                reward -= self.reward_wrong_turn_penalty * theta_next

        elif action in (2, 4):
            # Si hay obstáculo al frente (< 0.70m), premiar el giro de evitación y eximir penalización innecesaria
            if d_front_next < obstacle_shaping_threshold:
                reward += 0.30
            else:
                reward -= self.reward_turn_penalty
                if aligned:
                    reward -= self.reward_unnecessary_turn_penalty

            if goal_right:
                reward += self.reward_correct_turn_bonus * theta_next
            elif goal_left:
                reward -= self.reward_wrong_turn_penalty * theta_next

        # ------------------------------------------------------------
        # 5. Mejora de alineación heredada
        # Mantiene compatibilidad con tus parámetros previos.
        # ------------------------------------------------------------
        reward += self.reward_alignment_gain * heading_improvement

        # ------------------------------------------------------------
        # 6. Penalización por oscilación
        # ------------------------------------------------------------
        if previous_action is not None:
            if (
                (previous_action in (1, 3) and action in (2, 4))
                or (previous_action in (2, 4) and action in (1, 3))
            ):
                reward -= self.reward_oscillation_penalty

        # ------------------------------------------------------------
        # 7. Recompensas terminales
        # ------------------------------------------------------------
        if goal_reached:
            reward += self.reward_goal

        elif collision:
            reward += self.reward_collision

        elif timeout:
            reward += self.reward_timeout

        return float(reward)

    def check_done(
        self,
        collision: bool,
        goal_reached: bool
    ) -> bool:
        if goal_reached:
            return True

        if collision:
            return True

        if self.step_count >= self.max_steps:
            return True

        return False

    def step(self, action: int):
        if self.current_state is None:
            state = self.wait_for_state()
        else:
            state = self.current_state.copy()

        self.step_count += 1
        previous_action = self.last_action

        robot_x_before, robot_y_before, _ = self.get_robot_pose()

        self.publish_action(action)

        action_completed = self.wait_for_action_done(
            timeout=self.action_execution_time
        )

        if not action_completed:
            self.get_logger().warn(
                'Action did not complete before timeout.'
            )

        raw_next_state = self.wait_for_raw_state()

        robot_x, robot_y, robot_yaw = self.get_robot_pose()

        next_state = self.build_corrected_state(raw_next_state)

        goal_x, goal_y = self.current_goal

        final_distance_to_goal = math.hypot(
            goal_x - robot_x,
            goal_y - robot_y
        )

        segment_distance_to_goal = point_to_segment_distance(
            px=goal_x,
            py=goal_y,
            ax=robot_x_before,
            ay=robot_y_before,
            bx=robot_x,
            by=robot_y
        )

        goal_reached_by_final_pose = (
            final_distance_to_goal <= self.goal_tolerance_m
        )

        goal_reached_by_segment = (
            segment_distance_to_goal <= self.goal_tolerance_m
        )

        goal_reached = (
            goal_reached_by_final_pose
            or goal_reached_by_segment
        )

        collision = self.check_collision(next_state)

        if goal_reached:
            collision = False

        timeout = (
            self.step_count >= self.max_steps
            and not goal_reached
            and not collision
        )

        done = self.check_done(
            collision=collision,
            goal_reached=goal_reached
        )

        reward = self.compute_reward(
            state=state,
            next_state=next_state,
            action=action,
            done=done,
            collision=collision,
            goal_reached=goal_reached,
            timeout=timeout,
            previous_action=previous_action
        )

        info: Dict[str, Any] = {
            "step": self.step_count,
            "robot_x": robot_x,
            "robot_y": robot_y,
            "robot_yaw": robot_yaw,
            "goal_x": goal_x,
            "goal_y": goal_y,
            "collision": collision,
            "goal_reached": goal_reached,
            "goal_reached_by_final_pose": goal_reached_by_final_pose,
            "goal_reached_by_segment": goal_reached_by_segment,
            "final_distance_to_goal": final_distance_to_goal,
            "segment_distance_to_goal": segment_distance_to_goal,
            "goal_reached_topic": self.goal_reached_topic,
            "timeout": timeout,
            "action_completed": action_completed,
            "action": int(action),
            "theta_goal_norm": float(next_state[3]),
            "d_goal_norm": float(next_state[4])
        }

        self.last_action = int(action)
        self.current_state = next_state.copy()

        return next_state, reward, done, info