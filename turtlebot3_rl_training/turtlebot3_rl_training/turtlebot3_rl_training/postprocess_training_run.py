#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

from turtlebot3_rl_training.plot_training_metrics import main as plot_main
from turtlebot3_rl_training.video_renderer import render_episode


def load_config(run_dir: Path) -> dict:
    config_path = run_dir / "config.json"

    if not config_path.exists():
        return {}

    with open(config_path, "r", encoding="utf-8") as f:
        return json.load(f)


def update_summary(run_dir: Path, extra_data: dict):
    summary_path = run_dir / "summary.json"

    if summary_path.exists():
        with open(summary_path, "r", encoding="utf-8") as f:
            summary = json.load(f)
    else:
        summary = {}

    summary.update(extra_data)

    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=4)


def generate_plots(run_dir: Path):
    metrics_path = run_dir / "metrics.csv"

    if not metrics_path.exists():
        print(f"[WARN] metrics.csv not found: {metrics_path}")
        return

    import sys

    old_argv = sys.argv
    sys.argv = [
        "plot_training_metrics",
        "--run-dir",
        str(run_dir)
    ]

    try:
        plot_main()
    finally:
        sys.argv = old_argv


def render_missing_videos(run_dir: Path, save_mp4: bool, save_gif: bool, fps: int):
    episodes_dir = run_dir / "episodes"

    if not episodes_dir.exists():
        print(f"[WARN] episodes directory not found: {episodes_dir}")
        return

    json_files = sorted(episodes_dir.glob("*.json"))

    for json_path in json_files:
        base_path = json_path.with_suffix("")

        if save_mp4:
            mp4_path = base_path.with_suffix(".mp4")

            if not mp4_path.exists():
                print(f"[POST] Rendering MP4: {mp4_path.name}")

                try:
                    render_episode(
                        json_path=str(json_path),
                        output_path=str(mp4_path),
                        fps=fps,
                        save_format="mp4"
                    )
                except Exception as exc:
                    print(f"[WARN] Could not render MP4 for {json_path.name}: {exc}")

        if save_gif:
            gif_path = base_path.with_suffix(".gif")

            if not gif_path.exists():
                print(f"[POST] Rendering GIF: {gif_path.name}")

                try:
                    render_episode(
                        json_path=str(json_path),
                        output_path=str(gif_path),
                        fps=fps,
                        save_format="gif"
                    )
                except Exception as exc:
                    print(f"[WARN] Could not render GIF for {json_path.name}: {exc}")


def postprocess_run(run_dir: str):
    run_dir = Path(run_dir).expanduser().resolve()

    if not run_dir.exists():
        raise FileNotFoundError(f"Run directory not found: {run_dir}")

    print()
    print("===============================================")
    print(" Postprocessing training run")
    print("===============================================")
    print(f"Run directory: {run_dir}")
    print("===============================================")

    config = load_config(run_dir)

    save_mp4 = bool(config.get("save_videos", True))
    save_gif = bool(config.get("save_gif", False))
    fps = int(config.get("video_fps", 8))

    generate_plots(run_dir)

    render_missing_videos(
        run_dir=run_dir,
        save_mp4=save_mp4,
        save_gif=save_gif,
        fps=fps
    )

    update_summary(
        run_dir=run_dir,
        extra_data={
            "postprocess_completed": True,
            "plots_generated": True,
            "videos_checked": True
        }
    )

    print()
    print("[POST] Postprocess completed.")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Postprocess a TurtleBot3 DQN training run."
    )

    parser.add_argument(
        "--run-dir",
        required=True,
        help="Path to the training run directory."
    )

    args = parser.parse_args()

    postprocess_run(args.run_dir)


if __name__ == "__main__":
    main()
