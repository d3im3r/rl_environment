#!/usr/bin/env python3

import argparse
import csv
import json
import random
import shutil
from pathlib import Path
from datetime import datetime

import numpy as np
import torch
import torch.optim as optim

import rclpy

try:
    import yaml
except ImportError:
    yaml = None

from turtlebot3_rl_training.dqn_core import (
    QNetwork,
    ReplayBuffer,
    select_action,
    train_step,
    save_checkpoint
)

from turtlebot3_rl_training.gazebo_rl_env import GazeboTurtleBot3Env
from turtlebot3_rl_training.episode_logger import EpisodeLogger
from turtlebot3_rl_training.video_renderer import render_episode
from turtlebot3_rl_training.postprocess_training_run import postprocess_run
from turtlebot3_rl_training.rosbag_tools import RosbagRecorder


METRICS_FIELDNAMES = [
    "episode",
    "reward",
    "steps",
    "success",
    "collision",
    "timeout",
    "epsilon",
    "avg_loss",
    "buffer_size",
    "eval_avg_reward",
    "eval_success_rate",
    "eval_collision_rate",
    "eval_avg_steps",
]


def str2bool(value):
    if isinstance(value, bool):
        return value

    value = value.lower()

    if value in ("true", "1", "yes", "y", "si", "sí"):
        return True

    if value in ("false", "0", "no", "n"):
        return False

    raise argparse.ArgumentTypeError("Boolean value expected: true/false")


def parse_args(args=None):
    if args is None:
        import sys
        from rclpy.utilities import remove_ros_args
        args = remove_ros_args(sys.argv)[1:]

    parser = argparse.ArgumentParser(
        description="Train a DQN agent for TurtleBot3 in ROS 2 / Gazebo."
    )

    parser.add_argument("--stage", type=int, default=1)

    parser.add_argument(
        "--goal-mode",
        type=str,
        default="single",
        choices=["single", "soft", "medium", "separated"],
        help=(
            "Fallback goal mode if --scenario-config is not used. "
            "single: one frontal goal. "
            "soft: frontal and small lateral goals. "
            "medium: frontal and medium lateral goals. "
            "separated: multiple separated goals."
        )
    )

    parser.add_argument(
        "--scenario-config",
        type=str,
        default="",
        help=(
            "Path to a YAML scenario file. If provided, stage, goal_mode, "
            "goals, robot_starts, reward parameters and environment parameters "
            "are loaded from the YAML file."
        )
    )

    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--max-steps", type=int, default=40)
    parser.add_argument("--seed", type=int, default=42)

    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--buffer-capacity", type=int, default=20000)
    parser.add_argument("--min-buffer-size", type=int, default=500)
    parser.add_argument("--target-update-every", type=int, default=25)

    parser.add_argument("--epsilon-start", type=float, default=1.0)
    parser.add_argument("--epsilon-end", type=float, default=0.05)
    parser.add_argument("--epsilon-decay", type=float, default=0.995)

    parser.add_argument("--eval-every", type=int, default=25)
    parser.add_argument("--eval-episodes", type=int, default=3)

    parser.add_argument("--save-videos", type=str2bool, default=True)
    parser.add_argument("--save-gif", type=str2bool, default=False)
    parser.add_argument("--video-fps", type=int, default=8)
    parser.add_argument("--record-bags", type=str2bool, default=False)

    parser.add_argument("--action-timeout", type=float, default=5.0)
    parser.add_argument("--max-goal-distance", type=float, default=5.0)
    parser.add_argument("--collision-distance-norm", type=float, default=0.10)
    parser.add_argument("--goal-tolerance-norm", type=float, default=0.05)

    parser.add_argument("--reward-progress-gain", type=float, default=8.0)
    parser.add_argument("--reward-step-penalty", type=float, default=0.05)
    parser.add_argument("--reward-heading-penalty", type=float, default=0.10)
    parser.add_argument("--reward-collision", type=float, default=-100.0)
    parser.add_argument("--reward-timeout", type=float, default=-80.0)
    parser.add_argument("--reward-goal", type=float, default=150.0)
    parser.add_argument("--reward-backward-penalty", type=float, default=0.50)

    parser.add_argument("--reward-turn-penalty", type=float, default=0.03)
    parser.add_argument("--reward-unnecessary-turn-penalty", type=float, default=0.20)
    parser.add_argument("--reward-forward-aligned-bonus", type=float, default=0.30)
    parser.add_argument("--reward-alignment-gain", type=float, default=0.50)
    parser.add_argument("--reward-oscillation-penalty", type=float, default=0.30)

    parser.add_argument("--aligned-threshold", type=float, default=0.08)
    parser.add_argument("--bad-heading-threshold", type=float, default=0.25)

    parser.add_argument(
        "--resume-checkpoint",
        type=str,
        default="",
        help="Path to checkpoint .pth file to continue training from."
    )

    parser.add_argument(
        "--load-optimizer",
        type=str2bool,
        default=False,
        help=(
            "Load optimizer state from checkpoint. "
            "Use false for fine-tuning from best_model.pth. "
            "Use true only if you want to continue the exact same training run."
        )
    )

    parser.add_argument(
        "--sync-target-on-resume",
        type=str2bool,
        default=True,
        help=(
            "If true, after loading the Q network, copy Q network weights "
            "into the target network. Recommended for fine-tuning."
        )
    )

    parser.add_argument(
        "--eval-only",
        action="store_true",
        help=(
            "Run episodes without training updates. "
            "Useful to test a checkpoint deterministically."
        )
    )

    parser.add_argument(
        "--base-dir",
        type=str,
        default=str(
            Path.home()
            / "ros2_ws"
            / "src"
            / "train_runs"
        )
    )

    parser.add_argument(
        "--run-prefix",
        type=str,
        default=None,
        help="Custom prefix for training_id. If empty, uses scenario_name or stage_<stage>_<goal_mode>."
    )

    return parser.parse_args(args)


def create_training_id(prefix: str = "run") -> str:
    timestamp = datetime.now().strftime("%Y_%m_%d_%H%M%S")
    return f"{prefix}_{timestamp}"


def create_run_dirs(base_dir: Path, training_id: str):
    root = base_dir / training_id

    dirs = {
        "root": root,
        "checkpoints": root / "checkpoints",
        "episodes": root / "episodes",
        "plots": root / "plots",
        "bags": root / "bags",
    }

    for path in dirs.values():
        path.mkdir(parents=True, exist_ok=True)

    return dirs


def save_json(path: Path, data: dict):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4)


def append_metrics_row(metrics_path: Path, row: dict):
    file_exists = metrics_path.exists()

    safe_row = {}

    for field in METRICS_FIELDNAMES:
        safe_row[field] = row.get(field, "")

    with open(metrics_path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=METRICS_FIELDNAMES
        )

        if not file_exists:
            writer.writeheader()

        writer.writerow(safe_row)


def load_scenario_config(path: str):
    if not path:
        return {}

    if yaml is None:
        raise ImportError(
            "PyYAML is required to use --scenario-config. "
            "Install it with: pip install pyyaml"
        )

    scenario_path = Path(path).expanduser().resolve()

    if not scenario_path.exists():
        raise FileNotFoundError(f"Scenario config not found: {scenario_path}")

    with open(scenario_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if data is None:
        data = {}

    if not isinstance(data, dict):
        raise ValueError(f"Scenario config must be a YAML dictionary: {scenario_path}")

    data["_scenario_path"] = str(scenario_path)

    return data


def parse_goals_from_scenario(scenario: dict):
    goals_data = scenario.get("goals", [])

    goals = []

    for item in goals_data:
        goals.append(
            (
                float(item["x"]),
                float(item["y"]),
            )
        )

    return goals


def parse_robot_starts_from_scenario(scenario: dict):
    starts_data = scenario.get("robot_starts", [])

    starts = []

    for item in starts_data:
        starts.append(
            (
                float(item["x"]),
                float(item["y"]),
                float(item.get("yaw", 0.0)),
            )
        )

    return starts


def get_nested_value(data: dict, keys, default):
    current = data

    for key in keys:
        if not isinstance(current, dict):
            return default

        if key not in current:
            return default

        current = current[key]

    return current


def apply_scenario_to_args(args, scenario: dict):
    if not scenario:
        return args

    args.stage = int(scenario.get("stage", args.stage))
    args.goal_mode = str(scenario.get("goal_mode", args.goal_mode))

    args.max_steps = int(scenario.get("max_steps", args.max_steps))
    args.action_timeout = float(scenario.get("action_timeout", args.action_timeout))
    args.max_goal_distance = float(
        scenario.get("max_goal_distance", args.max_goal_distance)
    )
    args.goal_tolerance_norm = float(
        scenario.get("goal_tolerance_norm", args.goal_tolerance_norm)
    )
    args.collision_distance_norm = float(
        scenario.get("collision_distance_norm", args.collision_distance_norm)
    )

    reward = scenario.get("reward", {})

    if isinstance(reward, dict):
        args.reward_progress_gain = float(
            reward.get("progress_gain", args.reward_progress_gain)
        )
        args.reward_step_penalty = float(
            reward.get("step_penalty", args.reward_step_penalty)
        )
        args.reward_heading_penalty = float(
            reward.get("heading_penalty", args.reward_heading_penalty)
        )
        args.reward_turn_penalty = float(
            reward.get("turn_penalty", args.reward_turn_penalty)
        )
        args.reward_unnecessary_turn_penalty = float(
            reward.get(
                "unnecessary_turn_penalty",
                args.reward_unnecessary_turn_penalty
            )
        )
        args.reward_forward_aligned_bonus = float(
            reward.get(
                "forward_aligned_bonus",
                args.reward_forward_aligned_bonus
            )
        )
        args.reward_alignment_gain = float(
            reward.get("alignment_gain", args.reward_alignment_gain)
        )
        args.reward_oscillation_penalty = float(
            reward.get("oscillation_penalty", args.reward_oscillation_penalty)
        )
        args.reward_backward_penalty = float(
            reward.get("backward_penalty", args.reward_backward_penalty)
        )
        args.reward_goal = float(
            reward.get("goal", args.reward_goal)
        )
        args.reward_collision = float(
            reward.get("collision", args.reward_collision)
        )
        args.reward_timeout = float(
            reward.get("timeout", args.reward_timeout)
        )

    thresholds = scenario.get("thresholds", {})

    if isinstance(thresholds, dict):
        args.aligned_threshold = float(
            thresholds.get("aligned", args.aligned_threshold)
        )
        args.bad_heading_threshold = float(
            thresholds.get("bad_heading", args.bad_heading_threshold)
        )

    return args


def get_goals_for_stage(stage: int, goal_mode: str):
    if stage == 1:
        if goal_mode == "single":
            return [
                (1.5, 0.0),
            ]

        if goal_mode == "soft":
            return [
                (1.5, 0.0),
                (1.5, 0.25),
                (1.5, -0.25),
            ]

        if goal_mode == "medium":
            return [
                (1.5, 0.0),
                (1.4, 0.45),
                (1.4, -0.45),
            ]

        if goal_mode == "separated":
            return [
                (1.5, 0.0),
                (1.2, 0.8),
                (1.2, -0.8),
            ]

    if stage == 2:
        if goal_mode == "single":
            return [
                (1.5, 0.0),
            ]

        return [
            (1.5, 0.0),
            (1.2, 0.8),
            (1.2, -0.8),
        ]

    if stage == 3:
        if goal_mode == "single":
            return [
                (1.5, 0.0),
            ]

        return [
            (1.5, 0.0),
            (1.6, 0.8),
            (1.6, -0.8),
        ]

    if stage == 4:
        return [
            (1.5, 0.0),
        ]

    if stage == 5:
        if goal_mode == "single":
            return [
                (1.5, 0.0),
            ]

        return [
            (1.5, 0.0),
            (1.6, 0.8),
            (1.6, -0.8),
        ]

    if stage == 6:
        return [
            (1.6, 1.2),
            (1.6, 0.8),
            (1.4, -0.8),
        ]

    if stage == 7:
        return [
            (1.6, 1.3),
        ]

    return [(1.5, 0.0)]


def get_robot_starts_for_stage(stage: int, goal_mode: str):
    if goal_mode in ["single", "soft", "medium"]:
        return [
            (-1.5, 0.0, 0.0),
        ]

    if stage == 1 and goal_mode == "separated":
        return [
            (-1.5, 0.0, 0.0),
            (-1.5, 0.5, 0.0),
            (-1.5, -0.5, 0.0),
        ]

    if stage == 2 and goal_mode == "separated":
        return [
            (-1.5, 0.0, 0.0),
            (-1.5, 0.5, 0.0),
            (-1.5, -0.5, 0.0),
        ]

    return [
        (-1.5, 0.0, 0.0),
    ]


def copy_scenario_to_run(scenario: dict, run_root: Path):
    scenario_path = scenario.get("_scenario_path", "")

    if not scenario_path:
        return ""

    source = Path(scenario_path).expanduser().resolve()
    destination = run_root / "scenario.yaml"

    shutil.copy2(source, destination)

    return str(destination)


def load_training_checkpoint(
    checkpoint_path: str,
    q_network,
    target_network,
    optimizer,
    device,
    load_optimizer=False,
    sync_target_on_resume=True,
    learning_rate=None
):
    checkpoint_path = Path(checkpoint_path).expanduser().resolve()

    if not checkpoint_path.exists():
        raise FileNotFoundError(
            f"Checkpoint not found: {checkpoint_path}"
        )

    print()
    print("===============================================")
    print(" Loading checkpoint")
    print("===============================================")
    print(f"Checkpoint: {checkpoint_path}")

    checkpoint = torch.load(
        checkpoint_path,
        map_location=device
    )

    q_keys = [
        "q_network_state_dict",
        "q_network",
        "model_state_dict",
        "policy_net_state_dict",
    ]

    loaded_q = False

    if isinstance(checkpoint, dict):
        for key in q_keys:
            if key in checkpoint:
                q_network.load_state_dict(checkpoint[key])
                loaded_q = True
                print(f"Loaded Q network from key: {key}")
                break

    if not loaded_q:
        try:
            q_network.load_state_dict(checkpoint)
            loaded_q = True
            print("Loaded Q network from raw checkpoint/state_dict.")
        except Exception as exc:
            raise KeyError(
                "No valid Q network state dict found in checkpoint."
            ) from exc

    if sync_target_on_resume:
        target_network.load_state_dict(q_network.state_dict())
        print("Target network synchronized from Q network.")
    else:
        target_keys = [
            "target_network_state_dict",
            "target_network",
            "target_net_state_dict",
        ]

        loaded_target = False

        if isinstance(checkpoint, dict):
            for key in target_keys:
                if key in checkpoint:
                    target_network.load_state_dict(checkpoint[key])
                    loaded_target = True
                    print(f"Loaded target network from key: {key}")
                    break

        if not loaded_target:
            target_network.load_state_dict(q_network.state_dict())
            print("Target network key not found. Copied from Q network.")

    target_network.eval()

    if load_optimizer:
        optimizer_keys = [
            "optimizer_state_dict",
            "optimizer",
        ]

        loaded_optimizer = False

        if isinstance(checkpoint, dict):
            for key in optimizer_keys:
                if key in checkpoint:
                    try:
                        optimizer.load_state_dict(checkpoint[key])
                        loaded_optimizer = True
                        print(f"Loaded optimizer from key: {key}")
                    except Exception as exc:
                        print(f"[WARN] Could not load optimizer state: {exc}")
                    break

        if not loaded_optimizer:
            print("Optimizer key not found. Using fresh optimizer.")
    else:
        print("Optimizer state NOT loaded. Using fresh optimizer.")

    if learning_rate is not None:
        for param_group in optimizer.param_groups:
            param_group["lr"] = learning_rate

        print(f"Optimizer learning rate forced to: {learning_rate}")

    metadata = {}

    if isinstance(checkpoint, dict):
        metadata = checkpoint.get("metadata", {})

    print("Checkpoint loaded successfully.")
    print(f"Metadata: {metadata}")
    print("===============================================")
    print()

    return metadata


def evaluate_policy(
    env,
    q_network,
    device,
    training_id,
    episode_id,
    stage,
    run_dirs,
    action_dim=5,
    eval_episodes=3,
    max_steps=40,
    save_videos=True,
    save_gif=False,
    fps=8,
    record_bags=False
):
    q_network.eval()

    rewards = []
    successes = []
    collisions = []
    steps_list = []

    for eval_id in range(1, eval_episodes + 1):
        bag_name = f"episode_{episode_id:04d}_eval_{eval_id:02d}"
        bag_dir = run_dirs["bags"] / bag_name

        bag_recorder = RosbagRecorder(
            output_dir=str(bag_dir),
            enabled=record_bags
        )

        try:
            bag_recorder.start()

            state = env.reset(episode_index=eval_id)

            logger = EpisodeLogger(output_dir=str(run_dirs["episodes"]))

            logger.start_episode({
                "training_id": training_id,
                "episode_id": episode_id,
                "stage": stage,
                "eval_id": eval_id,
                "goal": {
                    "x": env.current_goal[0],
                    "y": env.current_goal[1]
                },
                "robot_start": {
                    "x": env.robot_start[0],
                    "y": env.robot_start[1],
                    "yaw": env.robot_start[2]
                }
            })

            total_reward = 0.0
            success = False
            collision = False
            timeout = False
            steps = 0

            for step in range(max_steps):
                action = select_action(
                    q_network=q_network,
                    state=state,
                    epsilon=0.0,
                    action_dim=action_dim,
                    device=device
                )

                next_state, reward, done, info = env.step(action)

                total_reward += reward
                steps = step + 1

                logger.log_step(
                    step=step,
                    x=info["robot_x"],
                    y=info["robot_y"],
                    yaw=info["robot_yaw"],
                    goal_x=info["goal_x"],
                    goal_y=info["goal_y"],
                    action=action,
                    reward=reward,
                    total_reward=total_reward,
                    done=done,
                    goal_reached=info["goal_reached"],
                    collision=info["collision"],
                    state=next_state
                )

                state = next_state

                if done:
                    success = bool(info["goal_reached"])
                    collision = bool(info["collision"])
                    timeout = bool(info["timeout"])
                    break

            rewards.append(total_reward)
            successes.append(1.0 if success else 0.0)
            collisions.append(1.0 if collision else 0.0)
            steps_list.append(steps)

            json_filename = logger.make_episode_filename(
                episode_id=episode_id,
                eval_id=eval_id
            )

            json_path = logger.save_episode(
                filename=json_filename,
                success=success,
                collision=collision,
                timeout=timeout,
                total_reward=total_reward,
                steps=steps
            )

            if save_videos:
                base_name = Path(json_path).with_suffix("")

                try:
                    render_episode(
                        json_path=json_path,
                        output_path=str(base_name) + ".mp4",
                        fps=fps,
                        save_format="mp4"
                    )

                    if save_gif:
                        render_episode(
                            json_path=json_path,
                            output_path=str(base_name) + ".gif",
                            fps=fps,
                            save_format="gif"
                        )

                except Exception as exc:
                    print(f"[WARN] Could not render video for {json_path}: {exc}")

        finally:
            bag_recorder.stop()

    q_network.train()

    return {
        "eval_avg_reward": float(np.mean(rewards)) if rewards else 0.0,
        "eval_success_rate": float(np.mean(successes)) if successes else 0.0,
        "eval_collision_rate": float(np.mean(collisions)) if collisions else 0.0,
        "eval_avg_steps": float(np.mean(steps_list)) if steps_list else 0.0
    }


def main():
    args = parse_args()

    scenario = load_scenario_config(args.scenario_config)
    args = apply_scenario_to_args(args, scenario)

    rclpy.init()

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(args.seed)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    stage = args.stage
    episodes = args.episodes
    max_steps = args.max_steps

    state_dim = 5
    action_dim = 3

    epsilon = args.epsilon_start

    scenario_name = scenario.get("scenario_name", "") if scenario else ""

    run_prefix = args.run_prefix

    if run_prefix is None:
        if scenario_name:
            run_prefix = scenario_name
        else:
            run_prefix = f"stage_{stage}_{args.goal_mode}"

    training_id = create_training_id(prefix=run_prefix)

    base_dir = Path(args.base_dir).expanduser().resolve()

    run_dirs = create_run_dirs(
        base_dir=base_dir,
        training_id=training_id
    )

    copied_scenario_path = copy_scenario_to_run(scenario, run_dirs["root"])

    if scenario:
        goals = parse_goals_from_scenario(scenario)
        robot_starts = parse_robot_starts_from_scenario(scenario)

        if not goals:
            goals = get_goals_for_stage(stage, args.goal_mode)

        if not robot_starts:
            robot_starts = get_robot_starts_for_stage(stage, args.goal_mode)
    else:
        goals = get_goals_for_stage(stage, args.goal_mode)
        robot_starts = get_robot_starts_for_stage(stage, args.goal_mode)

    config = {
        "training_id": training_id,
        "scenario_name": scenario_name,
        "scenario_config": args.scenario_config,
        "scenario_copy": copied_scenario_path,
        "stage": stage,
        "goal_mode": args.goal_mode,
        "episodes": episodes,
        "max_steps": max_steps,
        "state_dim": state_dim,
        "action_dim": action_dim,
        "seed": args.seed,
        "batch_size": args.batch_size,
        "gamma": args.gamma,
        "learning_rate": args.learning_rate,
        "buffer_capacity": args.buffer_capacity,
        "min_buffer_size": args.min_buffer_size,
        "target_update_every": args.target_update_every,
        "epsilon_start": args.epsilon_start,
        "epsilon_end": args.epsilon_end,
        "epsilon_decay": args.epsilon_decay,
        "eval_every": args.eval_every,
        "eval_episodes": args.eval_episodes,
        "save_videos": args.save_videos,
        "save_gif": args.save_gif,
        "record_bags": args.record_bags,
        "video_fps": args.video_fps,
        "action_timeout": args.action_timeout,
        "max_goal_distance": args.max_goal_distance,
        "collision_distance_norm": args.collision_distance_norm,
        "goal_tolerance_norm": args.goal_tolerance_norm,
        "reward_progress_gain": args.reward_progress_gain,
        "reward_step_penalty": args.reward_step_penalty,
        "reward_heading_penalty": args.reward_heading_penalty,
        "reward_collision": args.reward_collision,
        "reward_timeout": args.reward_timeout,
        "reward_goal": args.reward_goal,
        "reward_backward_penalty": args.reward_backward_penalty,
        "reward_turn_penalty": args.reward_turn_penalty,
        "reward_unnecessary_turn_penalty": args.reward_unnecessary_turn_penalty,
        "reward_forward_aligned_bonus": args.reward_forward_aligned_bonus,
        "reward_alignment_gain": args.reward_alignment_gain,
        "reward_oscillation_penalty": args.reward_oscillation_penalty,
        "aligned_threshold": args.aligned_threshold,
        "bad_heading_threshold": args.bad_heading_threshold,
        "device": str(device),
        "goals": goals,
        "robot_starts": robot_starts,
        "resume_checkpoint": args.resume_checkpoint,
        "load_optimizer": args.load_optimizer,
        "sync_target_on_resume": args.sync_target_on_resume,
        "eval_only": args.eval_only,
    }

    save_json(run_dirs["root"] / "config.json", config)

    metrics_path = run_dirs["root"] / "metrics.csv"

    q_network = QNetwork(
        state_dim=state_dim,
        action_dim=action_dim
    ).to(device)

    target_network = QNetwork(
        state_dim=state_dim,
        action_dim=action_dim
    ).to(device)

    target_network.load_state_dict(q_network.state_dict())
    target_network.eval()

    optimizer = optim.Adam(
        q_network.parameters(),
        lr=args.learning_rate
    )

    replay_buffer = ReplayBuffer(
        capacity=args.buffer_capacity
    )

    resume_metadata = {}

    if args.resume_checkpoint:
        resume_metadata = load_training_checkpoint(
            checkpoint_path=args.resume_checkpoint,
            q_network=q_network,
            target_network=target_network,
            optimizer=optimizer,
            device=device,
            load_optimizer=args.load_optimizer,
            sync_target_on_resume=args.sync_target_on_resume,
            learning_rate=args.learning_rate
        )

    env = GazeboTurtleBot3Env(
        stage=stage,
        max_steps=max_steps,
        goal_list=goals,
        robot_start_list=robot_starts,
        action_execution_time=args.action_timeout,
        max_goal_distance=args.max_goal_distance,
        collision_distance_norm=args.collision_distance_norm,
        goal_tolerance_norm=args.goal_tolerance_norm,
        reward_progress_gain=args.reward_progress_gain,
        reward_step_penalty=args.reward_step_penalty,
        reward_heading_penalty=args.reward_heading_penalty,
        reward_collision=args.reward_collision,
        reward_timeout=args.reward_timeout,
        reward_goal=args.reward_goal,
        reward_backward_penalty=args.reward_backward_penalty,
        reward_turn_penalty=args.reward_turn_penalty,
        reward_unnecessary_turn_penalty=args.reward_unnecessary_turn_penalty,
        reward_forward_aligned_bonus=args.reward_forward_aligned_bonus,
        reward_alignment_gain=args.reward_alignment_gain,
        reward_oscillation_penalty=args.reward_oscillation_penalty,
        aligned_threshold=args.aligned_threshold,
        bad_heading_threshold=args.bad_heading_threshold,
    )

    best_score = -float("inf")

    if resume_metadata:
        checkpoint_stage = resume_metadata.get("stage", None)
        checkpoint_goal_mode = resume_metadata.get("goal_mode", None)
        checkpoint_score = resume_metadata.get("score", None)

        if checkpoint_score is not None:
            try:
                same_stage = checkpoint_stage is not None and int(checkpoint_stage) == int(stage)
                same_goal_mode = checkpoint_goal_mode is None or str(checkpoint_goal_mode) == str(args.goal_mode)

                # Solo conservar el best_score previo si la etapa Y el modo de metas coinciden
                if same_stage and same_goal_mode:
                    best_score = float(checkpoint_score)
                    print(
                        f"[RESUME] Misma etapa ({stage}) y mismo goal_mode ({args.goal_mode}). "
                        f"Initial best_score cargado del checkpoint: {best_score:.2f}"
                    )
                else:
                    reason = f"Etapa: {checkpoint_stage}->{stage}" if not same_stage else f"Goal Mode: {checkpoint_goal_mode}->{args.goal_mode}"
                    print(
                        f"[TRANSFER] Cambio de configuración ({reason}). "
                        f"Reiniciando best_score = -inf para rastrear el mejor modelo del nuevo entorno."
                    )
            except (TypeError, ValueError):
                print(
                    "[WARN] Checkpoint metadata contains an invalid score/stage/goal_mode. "
                    "Using best_score = -inf."
                )

    completed_episodes = 0

    print()
    print("===============================================")
    print(" TurtleBot3 DQN Training")
    print("===============================================")
    print(f"Training ID: {training_id}")
    print(f"Scenario name: {scenario_name}")
    print(f"Scenario config: {args.scenario_config}")
    print(f"Scenario copy: {copied_scenario_path}")
    print(f"Stage: {stage}")
    print(f"Goal mode: {args.goal_mode}")
    print(f"Goals: {goals}")
    print(f"Robot starts: {robot_starts}")
    print(f"Run dir: {run_dirs['root']}")
    print(f"Device: {device}")
    print(f"Episodes: {episodes}")
    print(f"Max steps: {max_steps}")
    print(f"Epsilon start: {args.epsilon_start}")
    print(f"Epsilon decay: {args.epsilon_decay}")
    print(f"Epsilon end: {args.epsilon_end}")
    print(f"Learning rate: {args.learning_rate}")
    print(f"Min buffer size: {args.min_buffer_size}")
    print(f"Batch size: {args.batch_size}")
    print(f"Target update every: {args.target_update_every}")
    print(f"Reward timeout: {args.reward_timeout}")
    print(f"Resume checkpoint: {args.resume_checkpoint}")
    print(f"Load optimizer: {args.load_optimizer}")
    print(f"Sync target on resume: {args.sync_target_on_resume}")
    print(f"Eval only: {args.eval_only}")
    print(f"Record bags: {args.record_bags}")
    print("===============================================")
    print()

    if resume_metadata:
        print("Resume metadata:")
        print(json.dumps(resume_metadata, indent=4))
        print()

    try:
        from collections import deque
        recent_rewards = deque(maxlen=20)
        recent_successes = deque(maxlen=20)
        recent_collisions = deque(maxlen=20)

        for episode in range(1, episodes + 1):
            completed_episodes = episode

            state = env.reset(episode_index=episode)

            total_reward = 0.0
            total_loss = 0.0
            loss_count = 0

            success = False
            collision = False
            timeout = False
            steps = 0

            q_network.train()

            for step in range(max_steps):
                action = select_action(
                    q_network=q_network,
                    state=state,
                    epsilon=epsilon,
                    action_dim=action_dim,
                    device=device
                )

                next_state, reward, done, info = env.step(action)

                replay_buffer.push(
                    state=state,
                    action=action,
                    reward=reward,
                    next_state=next_state,
                    done=done
                )

                if (
                    not args.eval_only
                    and len(replay_buffer) >= args.min_buffer_size
                ):
                    loss = train_step(
                        q_network=q_network,
                        target_network=target_network,
                        replay_buffer=replay_buffer,
                        optimizer=optimizer,
                        batch_size=args.batch_size,
                        gamma=args.gamma,
                        device=device
                    )

                    total_loss += loss
                    loss_count += 1

                state = next_state
                total_reward += reward
                steps = step + 1

                if done:
                    success = bool(info["goal_reached"])
                    collision = bool(info["collision"])
                    timeout = bool(info["timeout"])
                    break

            if (
                not args.eval_only
                and episode % args.target_update_every == 0
            ):
                target_network.load_state_dict(q_network.state_dict())
                target_network.eval()

            epsilon = max(args.epsilon_end, epsilon * args.epsilon_decay)

            avg_loss = total_loss / loss_count if loss_count > 0 else 0.0

            recent_rewards.append(total_reward)
            recent_successes.append(1.0 if success else 0.0)
            recent_collisions.append(1.0 if collision else 0.0)

            avg_recent_reward = np.mean(recent_rewards)
            avg_recent_success = np.mean(recent_successes) * 100.0
            avg_recent_collision = np.mean(recent_collisions) * 100.0

            row = {
                "episode": episode,
                "reward": total_reward,
                "steps": steps,
                "success": int(success),
                "collision": int(collision),
                "timeout": int(timeout),
                "epsilon": epsilon,
                "avg_loss": avg_loss,
                "buffer_size": len(replay_buffer)
            }

            if success:
                result_str = "ÉXITO"
            elif collision:
                result_str = "COLISIÓN"
            else:
                result_str = "TIMEOUT"

            print(
                f"Episode {episode:04d} | "
                f"resultado={result_str:8s} | "
                f"reward={total_reward:7.1f} (avg20={avg_recent_reward:7.1f}) | "
                f"éxito20={avg_recent_success:5.1f}% | "
                f"colisión20={avg_recent_collision:5.1f}% | "
                f"pasos={steps:3d} | "
                f"epsilon={epsilon:.3f} | "
                f"loss={avg_loss:.5f}",
                flush=True
            )

            if episode % args.eval_every == 0:
                eval_metrics = evaluate_policy(
                    env=env,
                    q_network=q_network,
                    device=device,
                    training_id=training_id,
                    episode_id=episode,
                    stage=stage,
                    run_dirs=run_dirs,
                    action_dim=action_dim,
                    eval_episodes=args.eval_episodes,
                    max_steps=max_steps,
                    save_videos=args.save_videos,
                    save_gif=args.save_gif,
                    fps=args.video_fps,
                    record_bags=args.record_bags
                )

                row.update(eval_metrics)

                score = (
                    eval_metrics["eval_avg_reward"]
                    + 100.0 * eval_metrics["eval_success_rate"]
                    - 100.0 * eval_metrics["eval_collision_rate"]
                )

                is_new_best = score > best_score
                print("\n" + "="*60)
                print(f"📊 EVALUACIÓN DE POLÍTICA (Episodio {episode})")
                print("-"*60)
                print(f"• Tasa de Éxito (Eval): {eval_metrics['eval_success_rate']*100:.1f}% ({int(eval_metrics['eval_success_rate']*args.eval_episodes)}/{args.eval_episodes} episodios)")
                print(f"• Recompensa Promedio (Eval): {eval_metrics['eval_avg_reward']:.2f}")
                print(f"• Tasa de Colisión (Eval): {eval_metrics['eval_collision_rate']*100:.1f}%")
                print(f"• Pasos Promedios (Eval): {eval_metrics['eval_avg_steps']:.1f}")
                print(f"• Puntaje de Desempeño: {score:.2f}")
                print(f"• ¿Nuevo mejor modelo?: {'¡SÍ! 🎉' if is_new_best else 'NO ❌'} (Mejor anterior: {best_score:.2f})")
                print("="*60 + "\n")

                metadata = {
                    "training_id": training_id,
                    "scenario_name": scenario_name,
                    "scenario_config": args.scenario_config,
                    "scenario_copy": copied_scenario_path,
                    "episode": episode,
                    "stage": stage,
                    "goal_mode": args.goal_mode,
                    "epsilon": epsilon,
                    "score": score,
                    "eval_metrics": eval_metrics,
                    "eval_only": args.eval_only,
                    "resume_checkpoint": args.resume_checkpoint,
                    "load_optimizer": args.load_optimizer,
                    "sync_target_on_resume": args.sync_target_on_resume,
                    "learning_rate": args.learning_rate,
                }

                save_checkpoint(
                    path=str(run_dirs["checkpoints"] / "last_model.pth"),
                    q_network=q_network,
                    target_network=target_network,
                    optimizer=optimizer,
                    metadata=metadata
                )

                if is_new_best:
                    best_score = score

                    save_checkpoint(
                        path=str(run_dirs["checkpoints"] / "best_model.pth"),
                        q_network=q_network,
                        target_network=target_network,
                        optimizer=optimizer,
                        metadata=metadata
                    )

                    print(
                        f"[BEST] New best model saved | "
                        f"score={best_score:.2f}"
                    )

            append_metrics_row(metrics_path, row)

    except KeyboardInterrupt:
        print()
        print("Training interrupted by user.")

    finally:
        try:
            env.stop_robot()
        except Exception as exc:
            print(f"[WARN] Could not stop robot safely: {exc}")

        try:
            metadata = {
                "training_id": training_id,
                "scenario_name": scenario_name,
                "scenario_config": args.scenario_config,
                "scenario_copy": copied_scenario_path,
                "episode": completed_episodes,
                "stage": stage,
                "goal_mode": args.goal_mode,
                "epsilon": epsilon,
                "score": best_score,
                "reason": "final_checkpoint",
                "eval_only": args.eval_only,
                "resume_checkpoint": args.resume_checkpoint,
                "load_optimizer": args.load_optimizer,
                "sync_target_on_resume": args.sync_target_on_resume,
                "learning_rate": args.learning_rate,
            }

            save_checkpoint(
                path=str(run_dirs["checkpoints"] / "final_model.pth"),
                q_network=q_network,
                target_network=target_network,
                optimizer=optimizer,
                metadata=metadata
            )

        except Exception as exc:
            print(f"[WARN] Could not save final checkpoint: {exc}")

        summary = {
            "training_id": training_id,
            "scenario_name": scenario_name,
            "scenario_config": args.scenario_config,
            "scenario_copy": copied_scenario_path,
            "stage": stage,
            "goal_mode": args.goal_mode,
            "best_score": best_score,
            "episodes_requested": episodes,
            "episodes_completed": completed_episodes,
            "run_dir": str(run_dirs["root"]),
            "record_bags": args.record_bags,
            "save_videos": args.save_videos,
            "save_gif": args.save_gif,
            "resume_checkpoint": args.resume_checkpoint,
            "load_optimizer": args.load_optimizer,
            "sync_target_on_resume": args.sync_target_on_resume,
            "eval_only": args.eval_only,
        }

        save_json(run_dirs["root"] / "summary.json", summary)

        env.destroy_node()
        rclpy.shutdown()

        print()
        print("Training finished.")
        print(f"Run directory: {run_dirs['root']}")
        print()

        try:
            postprocess_run(str(run_dirs["root"]))
        except Exception as exc:
            print(f"[WARN] Postprocess failed: {exc}")


if __name__ == "__main__":
    main()