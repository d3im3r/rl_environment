from datetime import datetime
from pathlib import Path
import json
from typing import Dict, Any


def create_training_id(prefix: str = "run") -> str:
    """
    Create a unique training id using current date and time.

    Example:
        run_2026_05_22_143512
    """
    timestamp = datetime.now().strftime("%Y_%m_%d_%H%M%S")
    return f"{prefix}_{timestamp}"


def create_training_run_dirs(
    base_dir: str = "training_runs",
    training_id: str = None
) -> Dict[str, Path]:
    """
    Create the folder structure for one training run.
    """

    if training_id is None:
        training_id = create_training_id()

    root_dir = Path(base_dir) / training_id

    dirs = {
        "root": root_dir,
        "checkpoints": root_dir / "checkpoints",
        "episodes": root_dir / "episodes",
        "plots": root_dir / "plots",
        "bags": root_dir / "bags",
    }

    for path in dirs.values():
        path.mkdir(parents=True, exist_ok=True)

    return dirs


def save_run_config(
    run_dir: Path,
    config: Dict[str, Any],
    training_id: str
) -> str:
    """
    Save the full training configuration used in the run.
    """

    config_to_save = dict(config)
    config_to_save["training_id"] = training_id

    output_path = run_dir / "config.json"

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(config_to_save, f, indent=4)

    return str(output_path)


def save_run_summary(
    run_dir: Path,
    summary: Dict[str, Any]
) -> str:
    """
    Save a final summary of the training run.
    """

    output_path = run_dir / "summary.json"

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=4)

    return str(output_path)