#!/usr/bin/env python3

import argparse
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt


def get_column(df, column_name):
    """
    Returns a pandas column as a NumPy array.

    This avoids compatibility issues between recent pandas versions
    and matplotlib when matplotlib tries to index Series internally.
    """
    return df[column_name].to_numpy()


def save_figure(fig, output_path):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def load_metrics(metrics_path: Path):
    if not metrics_path.exists():
        raise FileNotFoundError(f"metrics.csv not found: {metrics_path}")

    df = pd.read_csv(metrics_path)

    required_columns = [
        "episode",
        "reward",
        "steps",
        "success",
        "collision",
        "timeout",
        "epsilon",
        "avg_loss",
        "buffer_size",
    ]

    missing_columns = [
        column for column in required_columns
        if column not in df.columns
    ]

    if missing_columns:
        raise ValueError(
            "metrics.csv is missing required columns: "
            + ", ".join(missing_columns)
        )

    numeric_columns = [
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

    for column in numeric_columns:
        if column in df.columns:
            df[column] = pd.to_numeric(df[column], errors="coerce")

    df = df.dropna(subset=["episode"])

    return df


def plot_evaluation_rates(df, plots_dir: Path):
    required_columns = [
        "eval_success_rate",
        "eval_collision_rate",
    ]

    for column in required_columns:
        if column not in df.columns:
            return

    eval_df = df.dropna(subset=["eval_success_rate", "eval_collision_rate"])

    if eval_df.empty:
        print("[WARN] No evaluation rate data available yet.")
        return

    fig, ax = plt.subplots(figsize=(10, 5))

    ax.plot(
        get_column(eval_df, "episode"),
        get_column(eval_df, "eval_success_rate"),
        marker="o",
        label="Evaluation success rate"
    )

    ax.plot(
        get_column(eval_df, "episode"),
        get_column(eval_df, "eval_collision_rate"),
        marker="o",
        label="Evaluation collision rate"
    )

    ax.set_title("Evaluation rates")
    ax.set_xlabel("Episode")
    ax.set_ylabel("Rate")
    ax.set_ylim(-0.1, 1.1)
    ax.grid(True)
    ax.legend()

    save_figure(fig, plots_dir / "01_eval_rates.png")


def plot_evaluation_reward(df, plots_dir: Path):
    if "eval_avg_reward" not in df.columns:
        return

    eval_df = df.dropna(subset=["eval_avg_reward"])

    if eval_df.empty:
        print("[WARN] No evaluation reward data available yet.")
        return

    fig, ax = plt.subplots(figsize=(10, 5))

    ax.plot(
        get_column(eval_df, "episode"),
        get_column(eval_df, "eval_avg_reward"),
        marker="o",
        label="Evaluation average reward"
    )

    ax.set_title("Evaluation average reward")
    ax.set_xlabel("Episode")
    ax.set_ylabel("Average reward")
    ax.grid(True)
    ax.legend()

    save_figure(fig, plots_dir / "02_eval_avg_reward.png")


def plot_success_collision_timeout_rates(df, plots_dir: Path):
    fig, ax = plt.subplots(figsize=(10, 5))

    window = 10

    success_rate = df["success"].rolling(window=window, min_periods=1).mean()
    collision_rate = df["collision"].rolling(window=window, min_periods=1).mean()
    timeout_rate = df["timeout"].rolling(window=window, min_periods=1).mean()

    ax.plot(
        get_column(df, "episode"),
        success_rate.to_numpy(),
        label=f"Success rate rolling {window}"
    )

    ax.plot(
        get_column(df, "episode"),
        collision_rate.to_numpy(),
        label=f"Collision rate rolling {window}"
    )

    ax.plot(
        get_column(df, "episode"),
        timeout_rate.to_numpy(),
        label=f"Timeout rate rolling {window}"
    )

    ax.set_title("Rolling termination rates")
    ax.set_xlabel("Episode")
    ax.set_ylabel("Rate")
    ax.set_ylim(-0.1, 1.1)
    ax.grid(True)
    ax.legend()

    save_figure(fig, plots_dir / "03_termination_rates.png")


def plot_rewards(df, plots_dir: Path):
    fig, ax = plt.subplots(figsize=(10, 5))

    ax.plot(
        get_column(df, "episode"),
        get_column(df, "reward"),
        label="Episode reward"
    )

    if len(df) >= 5:
        rolling_reward = df["reward"].rolling(window=5, min_periods=1).mean()

        ax.plot(
            get_column(df, "episode"),
            rolling_reward.to_numpy(),
            label="Reward moving average"
        )

    ax.set_title("Training reward")
    ax.set_xlabel("Episode")
    ax.set_ylabel("Reward")
    ax.grid(True)
    ax.legend()

    save_figure(fig, plots_dir / "04_reward.png")


def plot_steps(df, plots_dir: Path):
    fig, ax = plt.subplots(figsize=(10, 5))

    ax.plot(
        get_column(df, "episode"),
        get_column(df, "steps"),
        label="Steps per episode"
    )

    ax.set_title("Steps per episode")
    ax.set_xlabel("Episode")
    ax.set_ylabel("Steps")
    ax.grid(True)
    ax.legend()

    save_figure(fig, plots_dir / "05_steps.png")


def plot_loss(df, plots_dir: Path):
    fig, ax = plt.subplots(figsize=(10, 5))

    ax.plot(
        get_column(df, "episode"),
        get_column(df, "avg_loss"),
        label="Average loss"
    )

    ax.set_title("Average training loss")
    ax.set_xlabel("Episode")
    ax.set_ylabel("Loss")
    ax.grid(True)
    ax.legend()

    save_figure(fig, plots_dir / "06_loss.png")


def plot_epsilon(df, plots_dir: Path):
    fig, ax = plt.subplots(figsize=(10, 5))

    ax.plot(
        get_column(df, "episode"),
        get_column(df, "epsilon"),
        label="Epsilon"
    )

    ax.set_title("Exploration rate")
    ax.set_xlabel("Episode")
    ax.set_ylabel("Epsilon")
    ax.grid(True)
    ax.legend()

    save_figure(fig, plots_dir / "07_epsilon.png")


def plot_buffer_size(df, plots_dir: Path):
    fig, ax = plt.subplots(figsize=(10, 5))

    ax.plot(
        get_column(df, "episode"),
        get_column(df, "buffer_size"),
        label="Replay buffer size"
    )

    ax.set_title("Replay buffer size")
    ax.set_xlabel("Episode")
    ax.set_ylabel("Transitions")
    ax.grid(True)
    ax.legend()

    save_figure(fig, plots_dir / "08_buffer_size.png")


def plot_success_collision_timeout(df, plots_dir: Path):
    fig, ax = plt.subplots(figsize=(10, 5))

    ax.plot(
        get_column(df, "episode"),
        get_column(df, "success"),
        label="Success"
    )

    ax.plot(
        get_column(df, "episode"),
        get_column(df, "collision"),
        label="Collision"
    )

    ax.plot(
        get_column(df, "episode"),
        get_column(df, "timeout"),
        label="Timeout"
    )

    ax.set_title("Episode termination flags")
    ax.set_xlabel("Episode")
    ax.set_ylabel("Flag value")
    ax.set_ylim(-0.1, 1.1)
    ax.grid(True)
    ax.legend()

    save_figure(fig, plots_dir / "09_termination_flags.png")


def plot_evaluation_steps(df, plots_dir: Path):
    if "eval_avg_steps" not in df.columns:
        return

    eval_df = df.dropna(subset=["eval_avg_steps"])

    if eval_df.empty:
        print("[WARN] No evaluation step data available yet.")
        return

    fig, ax = plt.subplots(figsize=(10, 5))

    ax.plot(
        get_column(eval_df, "episode"),
        get_column(eval_df, "eval_avg_steps"),
        marker="o",
        label="Evaluation average steps"
    )

    ax.set_title("Evaluation average steps")
    ax.set_xlabel("Episode")
    ax.set_ylabel("Average steps")
    ax.grid(True)
    ax.legend()

    save_figure(fig, plots_dir / "10_eval_avg_steps.png")


def plot_summary_dashboard(df, plots_dir: Path):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    axes[0, 0].plot(
        get_column(df, "episode"),
        get_column(df, "reward"),
        label="Reward"
    )
    axes[0, 0].set_title("Reward")
    axes[0, 0].set_xlabel("Episode")
    axes[0, 0].set_ylabel("Reward")
    axes[0, 0].grid(True)

    axes[0, 1].plot(
        get_column(df, "episode"),
        get_column(df, "epsilon"),
        label="Epsilon"
    )
    axes[0, 1].set_title("Epsilon")
    axes[0, 1].set_xlabel("Episode")
    axes[0, 1].set_ylabel("Epsilon")
    axes[0, 1].grid(True)

    axes[1, 0].plot(
        get_column(df, "episode"),
        get_column(df, "avg_loss"),
        label="Average loss"
    )
    axes[1, 0].set_title("Average loss")
    axes[1, 0].set_xlabel("Episode")
    axes[1, 0].set_ylabel("Loss")
    axes[1, 0].grid(True)

    axes[1, 1].plot(
        get_column(df, "episode"),
        get_column(df, "success"),
        label="Success"
    )
    axes[1, 1].plot(
        get_column(df, "episode"),
        get_column(df, "collision"),
        label="Collision"
    )
    axes[1, 1].plot(
        get_column(df, "episode"),
        get_column(df, "timeout"),
        label="Timeout"
    )
    axes[1, 1].set_title("Termination flags")
    axes[1, 1].set_xlabel("Episode")
    axes[1, 1].set_ylabel("Flag")
    axes[1, 1].set_ylim(-0.1, 1.1)
    axes[1, 1].grid(True)
    axes[1, 1].legend()

    fig.tight_layout()
    fig.savefig(plots_dir / "11_summary_dashboard.png", dpi=150)
    plt.close(fig)


def generate_metrics_summary(df, plots_dir: Path):
    summary_path = plots_dir / "12_metrics_summary.txt"

    total_episodes = int(df["episode"].max())
    last_row = df.iloc[-1]

    mean_reward_last_10 = df["reward"].tail(10).mean()
    success_rate_last_10 = df["success"].tail(10).mean()
    collision_rate_last_10 = df["collision"].tail(10).mean()
    timeout_rate_last_10 = df["timeout"].tail(10).mean()

    lines = [
        "Training Metrics Summary",
        "========================",
        "",
        f"Total episodes: {total_episodes}",
        f"Last episode reward: {last_row['reward']:.4f}",
        f"Last episode steps: {int(last_row['steps'])}",
        f"Last epsilon: {last_row['epsilon']:.4f}",
        f"Last average loss: {last_row['avg_loss']:.6f}",
        "",
        "Last 10 episodes:",
        f"Mean reward: {mean_reward_last_10:.4f}",
        f"Success rate: {success_rate_last_10:.4f}",
        f"Collision rate: {collision_rate_last_10:.4f}",
        f"Timeout rate: {timeout_rate_last_10:.4f}",
    ]

    if "eval_success_rate" in df.columns:
        eval_df = df.dropna(subset=["eval_success_rate"])

        if not eval_df.empty:
            last_eval = eval_df.iloc[-1]

            lines.extend([
                "",
                "Last evaluation:",
                f"Episode: {int(last_eval['episode'])}",
                f"Eval average reward: {last_eval.get('eval_avg_reward', 0.0):.4f}",
                f"Eval success rate: {last_eval.get('eval_success_rate', 0.0):.4f}",
                f"Eval collision rate: {last_eval.get('eval_collision_rate', 0.0):.4f}",
                f"Eval average steps: {last_eval.get('eval_avg_steps', 0.0):.4f}",
            ])

    summary_path.write_text("\n".join(lines), encoding="utf-8")


def generate_all_plots(run_dir: Path):
    metrics_path = run_dir / "metrics.csv"
    plots_dir = run_dir / "plots"

    plots_dir.mkdir(parents=True, exist_ok=True)

    df = load_metrics(metrics_path)

    if df.empty:
        print("[WARN] metrics.csv is empty. No plots generated.")
        return

    plot_evaluation_rates(df, plots_dir)
    plot_evaluation_reward(df, plots_dir)
    plot_success_collision_timeout_rates(df, plots_dir)
    plot_rewards(df, plots_dir)
    plot_steps(df, plots_dir)
    plot_loss(df, plots_dir)
    plot_epsilon(df, plots_dir)
    plot_buffer_size(df, plots_dir)
    plot_success_collision_timeout(df, plots_dir)
    plot_evaluation_steps(df, plots_dir)
    plot_summary_dashboard(df, plots_dir)
    generate_metrics_summary(df, plots_dir)

    print(f"[PLOT] Plots generated in: {plots_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="Plot TurtleBot3 DQN training metrics."
    )

    parser.add_argument(
        "--run-dir",
        required=True,
        help="Path to the training run directory."
    )

    args = parser.parse_args()

    run_dir = Path(args.run_dir).expanduser().resolve()

    generate_all_plots(run_dir)


if __name__ == "__main__":
    main()