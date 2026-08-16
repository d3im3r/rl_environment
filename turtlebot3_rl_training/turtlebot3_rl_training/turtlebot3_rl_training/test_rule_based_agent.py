#!/usr/bin/env python3

import argparse
import math
import time

import rclpy

from turtlebot3_rl_training.gazebo_rl_env import GazeboTurtleBot3Env


def get_goals_for_stage(stage: int):
    if stage == 1:
        return [
            (1.5, 0.0),
            (1.5, 0.4),
            (1.5, -0.4),
        ]

    if stage == 2:
        return [
            (1.5, 0.0),
            (1.5, 0.6),
            (1.5, -0.6),
        ]

    if stage == 3:
        return [
            (1.5, 0.0),
            (1.6, 0.4),
            (1.6, -0.4),
        ]

    return [(1.5, 0.0)]


def select_rule_based_action(state, aligned_threshold: float):
    """
    State:
        [d_front_norm, d_left_norm, d_right_norm, theta_goal_norm, d_goal_norm]

    Actions:
        0 -> forward
        1 -> turn left
        2 -> turn right
    """

    theta_goal_norm = float(state[3])

    if theta_goal_norm > aligned_threshold:
        return 1

    if theta_goal_norm < -aligned_threshold:
        return 2

    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Rule-based test agent for TurtleBot3 RL environment."
    )

    parser.add_argument("--stage", type=int, default=1)
    parser.add_argument("--max-steps", type=int, default=40)
    parser.add_argument("--episodes-per-goal", type=int, default=1)

    parser.add_argument("--aligned-threshold", type=float, default=0.06)
    parser.add_argument("--goal-tolerance-norm", type=float, default=0.05)
    parser.add_argument("--collision-distance-norm", type=float, default=0.10)
    parser.add_argument("--action-timeout", type=float, default=5.0)

    args = parser.parse_args()

    rclpy.init()

    goals = get_goals_for_stage(args.stage)

    env = GazeboTurtleBot3Env(
        stage=args.stage,
        max_steps=args.max_steps,
        goal_list=goals,
        action_execution_time=args.action_timeout,
        collision_distance_norm=args.collision_distance_norm,
        goal_tolerance_norm=args.goal_tolerance_norm,
    )

    print()
    print("===============================================")
    print(" Rule-Based Agent Test")
    print("===============================================")
    print(f"Stage: {args.stage}")
    print(f"Goals: {goals}")
    print(f"Max steps: {args.max_steps}")
    print(f"Aligned threshold: {args.aligned_threshold}")
    print(f"Goal tolerance norm: {args.goal_tolerance_norm}")
    print("===============================================")
    print()

    total_tests = 0
    total_success = 0
    total_collision = 0
    total_timeout = 0

    try:
        test_index = 0

        for goal_id, goal in enumerate(goals, start=1):
            for repeat in range(1, args.episodes_per_goal + 1):
                test_index += 1
                total_tests += 1

                print()
                print("-----------------------------------------------")
                print(f"Test {test_index}")
                print(f"Goal {goal_id}: x={goal[0]:.2f}, y={goal[1]:.2f}")
                print("-----------------------------------------------")

                state = env.reset(
                    episode_index=test_index,
                    goal=goal
                )

                print(
                    "Initial state: "
                    f"front={state[0]:.3f}, "
                    f"left={state[1]:.3f}, "
                    f"right={state[2]:.3f}, "
                    f"theta={state[3]:.3f}, "
                    f"d_goal={state[4]:.3f}"
                )

                total_reward = 0.0
                success = False
                collision = False
                timeout = False

                for step in range(args.max_steps):
                    action = select_rule_based_action(
                        state=state,
                        aligned_threshold=args.aligned_threshold
                    )

                    next_state, reward, done, info = env.step(action)

                    total_reward += reward

                    print(
                        f"Step {step + 1:02d} | "
                        f"action={action} | "
                        f"theta={next_state[3]: .3f} | "
                        f"d_goal={next_state[4]:.3f} | "
                        f"reward={reward: .2f} | "
                        f"goal={info['goal_reached']} | "
                        f"collision={info['collision']}"
                    )

                    state = next_state

                    if done:
                        success = bool(info["goal_reached"])
                        collision = bool(info["collision"])
                        timeout = bool(info["timeout"])
                        break

                if success:
                    total_success += 1

                if collision:
                    total_collision += 1

                if timeout:
                    total_timeout += 1

                print()
                print("Result:")
                print(f"Success: {success}")
                print(f"Collision: {collision}")
                print(f"Timeout: {timeout}")
                print(f"Total reward: {total_reward:.2f}")
                print(f"Final robot pose: x={info['robot_x']:.3f}, y={info['robot_y']:.3f}")
                print(f"Goal pose: x={info['goal_x']:.3f}, y={info['goal_y']:.3f}")

                time.sleep(0.5)

    except KeyboardInterrupt:
        print()
        print("Rule-based test interrupted by user.")

    finally:
        env.stop_robot()
        env.destroy_node()
        rclpy.shutdown()

        print()
        print("===============================================")
        print(" Rule-Based Test Summary")
        print("===============================================")
        print(f"Total tests: {total_tests}")
        print(f"Successes: {total_success}")
        print(f"Collisions: {total_collision}")
        print(f"Timeouts: {total_timeout}")

        if total_tests > 0:
            print(f"Success rate: {total_success / total_tests:.3f}")

        print("===============================================")
        print()


if __name__ == "__main__":
    main()