import argparse
import json
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch
import numpy as np


ACTION_LABELS = {
    0: "Forward",
    1: "Soft left arc",
    2: "Soft right arc",
    3: "Hard left arc",
    4: "Hard right arc"
}


def load_episode_json(json_path: str):
    with open(json_path, "r", encoding="utf-8") as f:
        return json.load(f)


def get_obstacles_for_stage(stage: int):
    """
    Simple obstacle definitions matching the custom Gazebo worlds approximately.
    Each obstacle is represented as:
    {"type": "box", "x": ..., "y": ..., "sx": ..., "sy": ...}
    """

    if stage == 2:
        return [
            {"type": "box", "x": 0.0, "y": 0.0, "sx": 0.45, "sy": 0.45}
        ]

    if stage == 3:
        return [
            {"type": "box", "x": 0.0, "y": 0.55, "sx": 0.45, "sy": 0.45},
            {"type": "box", "x": 0.0, "y": -0.55, "sx": 0.45, "sy": 0.45},
        ]

    if stage == 4:
        return [
            {"type": "box", "x": 0.0, "y": 0.75, "sx": 3.6, "sy": 0.10},
            {"type": "box", "x": 0.0, "y": -0.75, "sx": 3.6, "sy": 0.10},
        ]

    if stage == 5:
        return [
            {"type": "box", "x": -0.35, "y": 0.35, "sx": 1.30, "sy": 0.12},
            {"type": "box", "x": 0.55, "y": -0.35, "sx": 1.30, "sy": 0.12},
            {"type": "box", "x": 0.15, "y": 0.95, "sx": 0.35, "sy": 0.35},
        ]

    if stage == 6:
        return [
            {"type": "box", "x": -0.6, "y": 0.7, "sx": 0.40, "sy": 0.40},
            {"type": "box", "x": 0.2, "y": -0.6, "sx": 0.40, "sy": 0.40},
            {"type": "box", "x": 0.8, "y": 0.45, "sx": 0.40, "sy": 0.40},
            {"type": "box", "x": 1.1, "y": -0.25, "sx": 0.35, "sy": 0.35},
        ]

    if stage == 7:
        return [
            {"type": "box", "x": -0.6, "y": 0.45, "sx": 1.80, "sy": 0.10},
            {"type": "box", "x": 0.35, "y": -0.35, "sx": 1.80, "sy": 0.10},
            {"type": "box", "x": 0.9, "y": 0.45, "sx": 0.10, "sy": 1.30},
            {"type": "box", "x": -1.35, "y": -0.75, "sx": 0.10, "sy": 1.10},
        ]

    return []


def get_status(summary):
    success = summary.get("success", False)
    collision = summary.get("collision", False)
    timeout = summary.get("timeout", False)

    if success:
        return "SUCCESS"
    if collision:
        return "COLLISION"
    if timeout:
        return "TIMEOUT"

    return "FINISHED"


def point_to_rect_distance(
    px: float,
    py: float,
    rx_min: float,
    ry_min: float,
    rx_max: float,
    ry_max: float
):
    """
    Distance from a point to an axis-aligned rectangle.
    Returns 0 if the point is inside the rectangle.
    """
    dx = max(rx_min - px, 0.0, px - rx_max)
    dy = max(ry_min - py, 0.0, py - ry_max)
    return np.hypot(dx, dy)


def choose_textbox_position(
    points_xy: List[Tuple[float, float]],
    goal_xy: Tuple[float, float],
    robot_xy: Tuple[float, float],
    xlim: Tuple[float, float],
    ylim: Tuple[float, float]
):
    """
    Choose the least occupied corner for the text box.

    Returns:
        (x_axes, y_axes, ha, va)
    in axes coordinates.
    """

    candidates = [
        {"name": "top_left",     "x": 0.02, "y": 0.98, "ha": "left",  "va": "top"},
        {"name": "top_right",    "x": 0.98, "y": 0.98, "ha": "right", "va": "top"},
        {"name": "bottom_left",  "x": 0.02, "y": 0.02, "ha": "left",  "va": "bottom"},
        {"name": "bottom_right", "x": 0.98, "y": 0.02, "ha": "right", "va": "bottom"},
    ]

    # Approximate text box size in axes fraction.
    box_w_frac = 0.38
    box_h_frac = 0.30

    x_min, x_max = xlim
    y_min, y_max = ylim

    def axes_rect_to_data_rect(candidate):
        if candidate["ha"] == "left":
            ax_x0 = candidate["x"]
            ax_x1 = candidate["x"] + box_w_frac
        else:
            ax_x0 = candidate["x"] - box_w_frac
            ax_x1 = candidate["x"]

        if candidate["va"] == "bottom":
            ax_y0 = candidate["y"]
            ax_y1 = candidate["y"] + box_h_frac
        else:
            ax_y0 = candidate["y"] - box_h_frac
            ax_y1 = candidate["y"]

        rx_min = x_min + ax_x0 * (x_max - x_min)
        rx_max = x_min + ax_x1 * (x_max - x_min)
        ry_min = y_min + ax_y0 * (y_max - y_min)
        ry_max = y_min + ax_y1 * (y_max - y_min)

        return rx_min, ry_min, rx_max, ry_max

    best_candidate = candidates[0]
    best_score = -1.0

    protected_points = list(points_xy)
    protected_points.append(goal_xy)
    protected_points.append(robot_xy)

    for candidate in candidates:
        rx_min, ry_min, rx_max, ry_max = axes_rect_to_data_rect(candidate)

        distances = [
            point_to_rect_distance(px, py, rx_min, ry_min, rx_max, ry_max)
            for px, py in protected_points
        ]

        min_dist = min(distances) if distances else float("inf")

        # Slight preference for top corners because they usually look cleaner.
        bonus = 0.05 if "top" in candidate["name"] else 0.0
        score = min_dist + bonus

        if score > best_score:
            best_score = score
            best_candidate = candidate

    return (
        best_candidate["x"],
        best_candidate["y"],
        best_candidate["ha"],
        best_candidate["va"]
    )


def render_episode(
    json_path: str,
    output_path: str,
    fps: int = 8,
    save_format: str = "mp4"
):
    data = load_episode_json(json_path)

    metadata = data.get("metadata", {})
    summary = data.get("summary", {})
    trajectory = data.get("trajectory", [])

    if not trajectory:
        raise ValueError("The episode JSON does not contain trajectory data.")

    stage = int(metadata.get("stage", 0))
    episode_id = metadata.get("episode_id", "unknown")
    run_id = metadata.get("run_id", metadata.get("training_id", "run"))

    xs = [p["x"] for p in trajectory]
    ys = [p["y"] for p in trajectory]

    goal_x = trajectory[0]["goal_x"]
    goal_y = trajectory[0]["goal_y"]

    obstacles = get_obstacles_for_stage(stage)
    final_status = get_status(summary)

    fig, ax = plt.subplots(figsize=(7.4, 7.4))

    xlim = (-2.2, 2.2)
    ylim = (-2.2, 2.2)

    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.30)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    ax.set_title(
        f"Stage {stage} | Episode {episode_id} | {run_id}",
        fontsize=13,
        fontweight="bold"
    )

    # Obstacles
    for obs in obstacles:
        rect = Rectangle(
            (obs["x"] - obs["sx"] / 2.0, obs["y"] - obs["sy"] / 2.0),
            obs["sx"],
            obs["sy"],
            facecolor="dimgray",
            edgecolor="black",
            linewidth=1.2,
            alpha=0.95
        )
        ax.add_patch(rect)

    # Goal as star
    ax.plot(
        [goal_x],
        [goal_y],
        marker="*",
        markersize=18,
        linestyle="None",
        color="limegreen",
        markeredgecolor="darkgreen",
        markeredgewidth=1.2
    )

    ax.text(
        goal_x + 0.06,
        goal_y + 0.06,
        "Goal",
        fontsize=9,
        color="darkgreen"
    )

    # Start marker
    ax.plot(
        xs[0],
        ys[0],
        marker="o",
        markersize=5,
        linestyle="None",
        color="black"
    )

    ax.text(
        xs[0] + 0.06,
        ys[0] + 0.06,
        "Start",
        fontsize=9,
        color="black"
    )

    # Trajectory
    trajectory_line, = ax.plot(
        [],
        [],
        linewidth=2.2,
        color="royalblue",
        alpha=0.80
    )

    # Robot as circle
    robot_patch = Circle(
        (xs[0], ys[0]),
        0.13,
        facecolor="deepskyblue",
        edgecolor="navy",
        linewidth=1.5,
        alpha=0.95
    )
    ax.add_patch(robot_patch)

    # Direction arrow
    arrow_length = 0.30
    arrow = FancyArrowPatch(
        (xs[0], ys[0]),
        (xs[0] + arrow_length, ys[0]),
        arrowstyle="-|>",
        mutation_scale=18,
        linewidth=2.0,
        color="black"
    )
    ax.add_patch(arrow)

    # Current position marker
    current_point, = ax.plot(
        [xs[0]],
        [ys[0]],
        marker="o",
        markersize=3,
        linestyle="None",
        color="navy"
    )

    # Auto-positioned text box
    text_box = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        verticalalignment="top",
        horizontalalignment="left",
        fontsize=10,
        linespacing=1.20,
        bbox=dict(
            facecolor="white",
            alpha=0.88,
            edgecolor="gray"
        )
    )

    total_reward_summary = summary.get("total_reward", 0.0)
    steps_summary = summary.get("steps", len(trajectory))

    def update(frame):
        point = trajectory[frame]

        x = point["x"]
        y = point["y"]
        yaw = point["yaw"]
        action = point["action"]
        total_reward = point["total_reward"]

        trajectory_line.set_data(xs[:frame + 1], ys[:frame + 1])

        robot_patch.center = (x, y)
        current_point.set_data([x], [y])

        hx = x + arrow_length * np.cos(yaw)
        hy = y + arrow_length * np.sin(yaw)
        arrow.set_positions((x, y), (hx, hy))

        action_label = ACTION_LABELS.get(action, str(action))

        main_text = (
            f"Step: {point['step']}\n"
            f"Action: {action_label}\n"
            f"Reward total: {total_reward:.2f}\n"
            f"Goal reached: {point['goal_reached']}\n"
            f"Collision: {point['collision']}\n"
            f"Steps: {steps_summary}\n"
            f"Summary reward: {total_reward_summary:.2f}\n"
            f"Status: {final_status}"
        )

        text_box.set_text(main_text)

        used_points = list(zip(xs[:frame + 1], ys[:frame + 1]))

        tx, ty, ha, va = choose_textbox_position(
            points_xy=used_points,
            goal_xy=(goal_x, goal_y),
            robot_xy=(x, y),
            xlim=xlim,
            ylim=ylim
        )

        text_box.set_position((tx, ty))
        text_box.set_ha(ha)
        text_box.set_va(va)

        return trajectory_line, robot_patch, arrow, current_point, text_box

    animation = FuncAnimation(
        fig,
        update,
        frames=len(trajectory),
        interval=1000 / fps,
        blit=False
    )

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if save_format == "gif":
        writer = PillowWriter(fps=fps)
        animation.save(str(output_path), writer=writer)
    else:
        writer = FFMpegWriter(fps=fps)
        animation.save(str(output_path), writer=writer)

    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description="Render a TurtleBot3 RL episode JSON as MP4 or GIF."
    )

    parser.add_argument(
        "--input",
        required=True,
        help="Path to episode JSON file."
    )

    parser.add_argument(
        "--output",
        required=True,
        help="Output video path."
    )

    parser.add_argument(
        "--format",
        choices=["mp4", "gif"],
        default="mp4",
        help="Output format."
    )

    parser.add_argument(
        "--fps",
        type=int,
        default=8,
        help="Frames per second."
    )

    args = parser.parse_args()

    render_episode(
        json_path=args.input,
        output_path=args.output,
        fps=args.fps,
        save_format=args.format
    )


if __name__ == "__main__":
    main()