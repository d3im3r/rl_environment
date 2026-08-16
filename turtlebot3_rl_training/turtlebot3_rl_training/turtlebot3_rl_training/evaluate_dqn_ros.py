#!/usr/bin/env python3

import sys
from turtlebot3_rl_training.train_dqn_ros import main as train_main


def main():
    """
    Punto de entrada para evaluación de políticas DQN pre-entrenadas.
    Delega a train_dqn_ros con la bandera --eval-only activada por defecto.
    """
    if "--eval-only" not in sys.argv:
        sys.argv.append("--eval-only")
        sys.argv.append("true")

    train_main()


if __name__ == "__main__":
    main()