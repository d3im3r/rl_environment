import json
from pathlib import Path
from typing import Dict, Any, List, Optional


class EpisodeLogger:
    def __init__(self, output_dir: str):
        """
        Logger for one or multiple RL episodes.

        Parameters
        ----------
        output_dir:
            Directory where episode JSON files will be stored.
            Recommended:
                training_runs/<training_id>/episodes/
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.trajectory: List[Dict[str, Any]] = []
        self.metadata: Dict[str, Any] = {}

    def start_episode(self, metadata: Dict[str, Any]):
        """
        Start a new episode log.

        Recommended metadata example:
            {
                "training_id": "run_2026_05_22_143512",
                "episode_id": 50,
                "stage": 5,
                "eval_id": 1,
                "goal": {
                    "x": 1.5,
                    "y": 0.0
                }
            }
        """
        self.metadata = metadata
        self.trajectory = []

    def log_step(
        self,
        step: int,
        x: float,
        y: float,
        yaw: float,
        goal_x: float,
        goal_y: float,
        action: int,
        reward: float,
        total_reward: float,
        done: bool,
        goal_reached: bool,
        collision: bool,
        state=None
    ):
        """
        Save one step of the episode trajectory.
        """
        row = {
            "step": int(step),
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
            "goal_x": float(goal_x),
            "goal_y": float(goal_y),
            "action": int(action),
            "reward": float(reward),
            "total_reward": float(total_reward),
            "done": bool(done),
            "goal_reached": bool(goal_reached),
            "collision": bool(collision)
        }

        if state is not None:
            row["state"] = [float(v) for v in state]

        self.trajectory.append(row)

    @staticmethod
    def make_episode_filename(
        episode_id: int,
        eval_id: Optional[int] = None,
        prefix: str = "episode"
    ) -> str:
        """
        Create a standard JSON filename for an episode.

        Examples:
            episode_0050.json
            episode_0050_eval_01.json
        """
        if eval_id is None:
            return f"{prefix}_{episode_id:04d}.json"

        return f"{prefix}_{episode_id:04d}_eval_{eval_id:02d}.json"

    def save_episode(
        self,
        filename: str,
        success: bool,
        collision: bool,
        total_reward: float,
        steps: int,
        timeout: bool = False,
        extra_summary: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Save the episode as a JSON file.
        """
        summary = {
            "success": bool(success),
            "collision": bool(collision),
            "timeout": bool(timeout),
            "total_reward": float(total_reward),
            "steps": int(steps)
        }

        if extra_summary is not None:
            summary.update(extra_summary)

        data = {
            "metadata": self.metadata,
            "summary": summary,
            "trajectory": self.trajectory
        }

        path = self.output_dir / filename

        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4)

        return str(path)

    def save_episode_auto(
        self,
        episode_id: int,
        success: bool,
        collision: bool,
        total_reward: float,
        steps: int,
        eval_id: Optional[int] = None,
        timeout: bool = False,
        extra_summary: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Save the episode using the standard filename convention.
        """
        filename = self.make_episode_filename(
            episode_id=episode_id,
            eval_id=eval_id
        )

        return self.save_episode(
            filename=filename,
            success=success,
            collision=collision,
            timeout=timeout,
            total_reward=total_reward,
            steps=steps,
            extra_summary=extra_summary
        )