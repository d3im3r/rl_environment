#!/usr/bin/env python3

import argparse
import csv
import math
import os
import time
from datetime import datetime
from pathlib import Path
import numpy as np

import rclpy
from turtlebot3_eval_platform.gazebo_rl_env import GazeboTurtleBot3Env
from turtlebot3_eval_platform.agents.dqn_agent import DQNAgent
from turtlebot3_eval_platform.agents.fuzzy_agent import FuzzyAgent
from turtlebot3_eval_platform.agents.rule_based_agent import RuleBasedAgent


def get_goals_for_stage(stage: int) -> list:
    """Retorna la lista de metas para la etapa especificada."""
    if stage == 1:
        return [(1.5, 0.0), (1.5, 0.3), (1.5, -0.3)]
    elif stage == 2:
        return [(1.5, 0.0), (1.3, 0.5), (1.3, -0.5)]
    elif stage == 3:
        return [(1.6, 0.0), (1.6, 0.5), (1.6, -0.5)]
    elif stage in [4, 5, 6, 7]:
        return [(1.6, 0.0), (1.5, 0.8), (1.5, -0.8)]
    return [(1.5, 0.0)]


def get_starts_for_stage(stage: int) -> list:
    """Retorna las poses de inicio del robot [x, y, yaw]."""
    return [(-1.5, 0.0, 0.0)]


def get_obstacle_count_for_stage(stage: int) -> int:
    """Infiere la cantidad de obstáculos fijos según la etapa."""
    if stage == 1:
        return 0
    elif stage == 2:
        return 1
    elif stage == 3:
        return 2
    return 4  # Etapas avanzadas


def append_to_eval_history(csv_path: str, row_data: dict):
    """Guarda los resultados agregados en el CSV central en el formato exacto original."""
    path = Path(csv_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    file_exists = path.exists()
    
    fieldnames = [
        "timestamp",
        "agent",
        "checkpoint",
        "world",
        "n_obstacles",
        "init_pos",
        "goal_pos",
        "episodes",
        "avg_reward",
        "success_rate",
        "collision_rate",
        "min_steps",
        "avg_steps",
        "max_steps",
        "std_steps",
        "run_dir"
    ]
    
    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()
        writer.writerow(row_data)


def main(args=None):
    if args is None:
        import sys
        from rclpy.utilities import remove_ros_args
        args = remove_ros_args(sys.argv)[1:]

    parser = argparse.ArgumentParser(description="Runner de Benchmarking para TurtleBot3 en Gazebo Classic.")
    parser.add_argument("--agent", type=str, default="rule-based", choices=["dqn", "fuzzy", "rule-based"],
                        help="Tipo de controlador a evaluar.")
    parser.add_argument("--model-path", type=str, default=None,
                        help="Ruta al checkpoint .pth (requerido para dqn).")
    parser.add_argument("--stage", type=int, default=1,
                        help="Etapa del mundo de Gazebo (1 a 7).")
    parser.add_argument("--episodes", type=int, default=3,
                        help="Cantidad de episodios a evaluar por meta.")
    parser.add_argument("--max-steps", type=int, default=250,  # Aumentado a 250 para evitar timeouts prematuros
                        help="Máximo de pasos por episodio.")
    parser.add_argument("--csv-path", type=str, default="/home/d3im3r/ros2_ws/src/eval_history.csv",
                        help="Ruta para guardar el historial de evaluación.")
    
    parsed_args = parser.parse_args(args)

    rclpy.init()

    goals = get_goals_for_stage(parsed_args.stage)
    starts = get_starts_for_stage(parsed_args.stage)

    print(f"\n[Benchmarking] Inicializando entorno para Stage {parsed_args.stage}...")
    env = GazeboTurtleBot3Env(
        stage=parsed_args.stage,
        max_steps=parsed_args.max_steps,
        step_dt=0.1,  # Control continuo a 10 Hz
        goal_list=goals,
        robot_start_list=starts
    )

    agent = None
    checkpoint_str = "N/A"
    if parsed_args.agent == "dqn":
        print(f"[Benchmarking] Cargando agente DQN...")
        agent = DQNAgent()
        agent.load(parsed_args.model_path)
        checkpoint_str = parsed_args.model_path if parsed_args.model_path else "random_weights"
    elif parsed_args.agent == "fuzzy":
        print(f"[Benchmarking] Cargando agente Fuzzy...")
        agent = FuzzyAgent()
        agent.load()
        checkpoint_str = "fuzzy_rules"
    elif parsed_args.agent == "rule-based":
        print(f"[Benchmarking] Cargando agente Rule-Based...")
        agent = RuleBasedAgent()
        agent.load()
        checkpoint_str = "heuristic_rules"

    # Generar directorio de corrida de evaluación único para logs detallados
    timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run_dir_name = f"eval_runs/eval_{agent.name.lower()}_stage_{parsed_args.stage}_{timestamp_str}"
    run_dir_full = Path("/home/d3im3r/ros2_ws/src") / run_dir_name
    run_dir_full.mkdir(parents=True, exist_ok=True)

    print("\n================================================")
    print(f" Iniciando evaluación de agente: {agent.name}")
    print(f" Stage: {parsed_args.stage} | Episodios por meta: {parsed_args.episodes}")
    print(f" Metas a evaluar: {goals}")
    print(f" Directorio de logs detallados: {run_dir_full}")
    print("================================================\n")

    detailed_episodes_log = []
    
    # Acumuladores para el resumen agregado
    rewards_all = []
    steps_all = []
    successes_all = []
    collisions_all = []

    try:
        episode_count = 0
        for goal_idx, goal in enumerate(goals):
            for ep in range(parsed_args.episodes):
                episode_count += 1
                print(f"\n--- Episodio {episode_count} | Evaluando Meta {goal_idx + 1}: x={goal[0]:.2f}, y={goal[1]:.2f} ---")
                
                state = env.reset(episode_index=goal_idx)
                
                total_reward = 0.0
                steps = 0
                done = False
                
                last_x, last_y = env.robot_x, env.robot_y
                path_length = 0.0
                
                while not done and rclpy.ok():
                    action = agent.select_action(state)
                    next_state, reward, done, info = env.step(action)
                    
                    curr_x, curr_y = env.robot_x, env.robot_y
                    path_length += math.hypot(curr_x - last_x, curr_y - last_y)
                    last_x, last_y = curr_x, curr_y
                    
                    total_reward += reward
                    steps += 1
                    state = next_state

                # Resultados del episodio
                success = int(info["goal_reached"])
                collision = int(info["collision"])
                timeout = int(info["timeout"])
                avg_speed = path_length / (steps * 0.1) if steps > 0 else 0.0

                print(f"Resultado: {'ÉXITO' if success else 'COLISIÓN' if collision else 'TIMEOUT'}")
                print(f"Pasos: {steps} | Distancia: {path_length:.2f} m | Recompensa: {total_reward:.2f}")

                # Guardar métricas del episodio para análisis detallado
                detailed_episodes_log.append({
                    "episode": episode_count,
                    "goal_x": goal[0],
                    "goal_y": goal[1],
                    "success": success,
                    "collision": collision,
                    "timeout": timeout,
                    "reward": round(total_reward, 2),
                    "steps": steps,
                    "path_length": round(path_length, 2),
                    "avg_speed": round(avg_speed, 3)
                })

                rewards_all.append(total_reward)
                steps_all.append(steps)
                successes_all.append(success)
                collisions_all.append(collision)
                
                time.sleep(0.5)

        # 4. Guardar archivo con los logs detallados del episodio en el run_dir
        detailed_csv_path = run_dir_full / "episode_history.csv"
        detailed_fieldnames = ["episode", "goal_x", "goal_y", "success", "collision", "timeout", "reward", "steps", "path_length", "avg_speed"]
        with open(detailed_csv_path, "w", newline="", encoding="utf-8") as df:
            writer = csv.DictWriter(df, fieldnames=detailed_fieldnames)
            writer.writeheader()
            writer.writerows(detailed_episodes_log)
        print(f"\n[Benchmarking] Logs de episodios detallados guardados en: {detailed_csv_path}")

        # 5. Calcular métricas agregadas globales
        total_episodes = len(rewards_all)
        if total_episodes > 0:
            avg_reward = np.mean(rewards_all)
            success_rate = np.mean(successes_all)
            collision_rate = np.mean(collisions_all)
            min_steps = np.min(steps_all)
            avg_steps = np.mean(steps_all)
            max_steps = np.max(steps_all)
            std_steps = np.std(steps_all)
        else:
            avg_reward = 0.0
            success_rate = 0.0
            collision_rate = 0.0
            min_steps = 0
            avg_steps = 0.0
            max_steps = 0
            std_steps = 0.0

        # Formato de inicio e meta
        init_pos_str = f"[{starts[0][0]}, {starts[0][1]}]"
        # Usamos la primera meta evaluada como representativa para el log
        goal_pos_str = f"[{goals[0][0]}, {goals[0][1]}]"

        # 6. Guardar fila agregada única en el CSV central
        aggregated_row = {
            "timestamp": datetime.now().strftime("%Y-%m-%d_%H-%M-%S"),
            "agent": agent.name,
            "checkpoint": checkpoint_str,
            "world": f"stage_{parsed_args.stage}",
            "n_obstacles": get_obstacle_count_for_stage(parsed_args.stage),
            "init_pos": init_pos_str,
            "goal_pos": goal_pos_str,
            "episodes": total_episodes,
            "avg_reward": round(float(avg_reward), 3),
            "success_rate": round(float(success_rate), 2),
            "collision_rate": round(float(collision_rate), 2),
            "min_steps": int(min_steps) if not np.isnan(min_steps) else "nan",
            "avg_steps": round(float(avg_steps), 2) if not np.isnan(avg_steps) else "nan",
            "max_steps": int(max_steps) if not np.isnan(max_steps) else "nan",
            "std_steps": round(float(std_steps), 2) if not np.isnan(std_steps) else "nan",
            "run_dir": run_dir_name
        }
        append_to_eval_history(parsed_args.csv_path, aggregated_row)
        print(f"[Benchmarking] Resumen agregado guardado en el log central: {parsed_args.csv_path}")

    except KeyboardInterrupt:
        print("\nEvaluación interrumpida por el usuario.")
    finally:
        env.stop_robot()
        env.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
