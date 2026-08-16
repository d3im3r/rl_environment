#!/usr/bin/env python3

import rclpy

from turtlebot3_rl_training.gazebo_rl_env import GazeboTurtleBot3Env


def main():
    rclpy.init()

    env = GazeboTurtleBot3Env(
        stage=1,
        max_steps=20,
        goal_list=[
            (1.5, 0.0),
            (1.5, 0.5),
            (1.5, -0.5)
        ],
        action_execution_time=5.0
    )

    state = env.reset(episode_index=0)

    print("Initial state:", state)

    actions = [0, 0, 1, 0, 2, 0]

    for action in actions:
        next_state, reward, done, info = env.step(action)

        print()
        print("Action:", action)
        print("Next state:", next_state)
        print("Reward:", reward)
        print("Done:", done)
        print("Info:", info)

        if done:
            break

    env.stop_robot()
    env.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
