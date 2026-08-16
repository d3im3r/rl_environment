#!/usr/bin/env python3

from typing import Optional
import numpy as np

from turtlebot3_eval_platform.agents.base_agent import BaseAgent


class RuleBasedAgent(BaseAgent):
    """
    Agente reactivo clásico basado en reglas y umbrales de seguridad.
    Evita obstáculos cercanos y se orienta hacia la meta.
    """

    def __init__(self, collision_threshold: float = 0.10, aligned_threshold: float = 0.10):
        super().__init__(name="Rule-Based")
        self.collision_threshold = collision_threshold
        self.aligned_threshold = aligned_threshold

        self.actions = {
            "recto": np.array([0.18, 0.0]),
            "giro_leve_izq": np.array([0.14, 0.4]),
            "giro_leve_der": np.array([0.14, -0.4]),
            "giro_fuerte_izq": np.array([0.08, 1.0]),
            "giro_fuerte_der": np.array([0.08, -1.0])
        }

    def load(self, model_path: Optional[str] = None) -> bool:
        print("[Rule-Based] Cargado con parámetros de umbral estáticos.")
        return True

    def select_action(self, state: np.ndarray) -> np.ndarray:
        # state: [d_front, d_left, d_right, theta_goal, d_goal]
        d_front, d_left, d_right, theta_goal, _ = state

        # 1. Evasión de emergencia (Obstáculo muy cercano al frente)
        if d_front < self.collision_threshold:
            # Girar hacia donde haya más espacio libre
            if d_left > d_right:
                return self.actions["giro_fuerte_izq"]
            else:
                return self.actions["giro_fuerte_der"]

        # 2. Navegación hacia la meta
        # Si la meta está a la izquierda
        if theta_goal > self.aligned_threshold:
            # Si el camino está medianamente libre
            if d_left > self.collision_threshold:
                return self.actions["giro_leve_izq"]
            else:
                return self.actions["giro_fuerte_der"]  # Contra-giro evasivo

        # Si la meta está a la derecha
        elif theta_goal < -self.aligned_threshold:
            if d_right > self.collision_threshold:
                return self.actions["giro_leve_der"]
            else:
                return self.actions["giro_fuerte_izq"]  # Contra-giro evasivo

        # 3. Camino libre y meta al frente -> Avanzar recto
        return self.actions["recto"]
