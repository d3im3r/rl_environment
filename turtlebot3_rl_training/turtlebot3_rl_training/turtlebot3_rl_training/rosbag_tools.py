#!/usr/bin/env python3

import os
import signal
import subprocess
import time
from pathlib import Path
from typing import List, Optional


DEFAULT_BAG_TOPICS = [
    "/odom",
    "/scan",
    "/cmd_vel",
    "/rl_state",
    "/rl_action",
    "/rl_action_done",
    "/rl_goal_reached",
    "/tf",
    "/tf_static",
]


class RosbagRecorder:
    def __init__(
        self,
        output_dir: str,
        topics: Optional[List[str]] = None,
        enabled: bool = True
    ):
        self.output_dir = Path(output_dir)
        self.topics = topics if topics is not None else DEFAULT_BAG_TOPICS
        self.enabled = enabled
        self.process = None

    def start(self):
        if not self.enabled:
            return

        self.output_dir.parent.mkdir(parents=True, exist_ok=True)

        if self.output_dir.exists():
            self.output_dir = self._make_unique_path(self.output_dir)

        command = [
            "ros2",
            "bag",
            "record",
            "-o",
            str(self.output_dir),
            *self.topics
        ]

        print(f"[BAG] Starting rosbag2: {self.output_dir}")

        self.process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )

        time.sleep(0.5)

    def stop(self):
        if not self.enabled:
            return

        if self.process is None:
            return

        print(f"[BAG] Stopping rosbag2: {self.output_dir}")

        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            self.process.wait(timeout=5.0)

        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=3.0)

        finally:
            self.process = None

    def _make_unique_path(self, path: Path) -> Path:
        index = 1

        while True:
            candidate = Path(f"{path}_{index:02d}")

            if not candidate.exists():
                return candidate

            index += 1