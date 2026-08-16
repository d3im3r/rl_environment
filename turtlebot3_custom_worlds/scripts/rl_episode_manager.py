#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class RLEpisodeManager(Node):
    def __init__(self):
        super().__init__('rl_episode_manager')

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
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

        self.get_logger().info('Waiting for /reset_simulation...')
        self.reset_sim_client.wait_for_service()

        self.get_logger().info('Waiting for /gazebo/set_entity_state...')
        self.set_entity_client.wait_for_service()

        self.get_logger().info('RL Episode Manager ready.')

    def stop_robot(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        time.sleep(0.1)

    def reset_simulation(self):
        request = Empty.Request()
        future = self.reset_sim_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.2)

    def set_model_pose(self, model_name: str, x: float, y: float, z: float, yaw: float):
        state = EntityState()
        state.name = model_name
        state.reference_frame = 'world'

        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
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

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(
                    f'{model_name} moved to x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
                )
            else:
                msg = getattr(future.result(), 'status_message', 'SetEntityState returned success=False')
                self.get_logger().warn(
                    f'Failed to move {model_name}: {msg}'
                )
        else:
            self.get_logger().error(f'Service call failed for {model_name}')

    def reset_episode(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        goal_x: float,
        goal_y: float
    ):
        self.get_logger().info('Resetting RL episode...')

        self.stop_robot()
        self.reset_simulation()
        time.sleep(0.5)

        self.set_model_pose(
            model_name='turtlebot3',
            x=robot_x,
            y=robot_y,
            z=0.01,
            yaw=robot_yaw
        )
        
        time.sleep(0.2)

        self.set_model_pose(
            model_name='goal_marker',
            x=goal_x,
            y=goal_y,
            z=0.03,
            yaw=0.0
        )

        self.stop_robot()
        time.sleep(0.2)

        self.get_logger().info('Episode reset complete.')


def main():
    rclpy.init()

    manager = RLEpisodeManager()

    test_goals = [
        (1.5, 0.0),
        (1.5, 2.0),
        (1.0, -2.0),
        (0.8, 1.2),
        (1.8, -1.0),
    ]

    robot_start = (-1.5, 0.0, 0.0)

    for i, goal in enumerate(test_goals):
        manager.get_logger().info(f'Test episode {i + 1}')

        manager.reset_episode(
            robot_x=robot_start[0],
            robot_y=robot_start[1],
            robot_yaw=robot_start[2],
            goal_x=goal[0],
            goal_y=goal[1]
        )

        time.sleep(3.0)

    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()