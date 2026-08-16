from turtlebot3_rl_training.run_manager import (
    create_training_id,
    create_training_run_dirs,
    save_run_config
)


def main():
    training_id = create_training_id()
    run_dirs = create_training_run_dirs(
        base_dir="training_runs",
        training_id=training_id
    )

    config = {
        "training": {
            "episodes": 10
        },
        "environment": {
            "stage": 5
        }
    }

    save_run_config(
        run_dir=run_dirs["root"],
        config=config,
        training_id=training_id
    )

    print(f"Training ID: {training_id}")
    print(f"Root: {run_dirs['root']}")
    print(f"Episodes: {run_dirs['episodes']}")


if __name__ == "__main__":
    main()