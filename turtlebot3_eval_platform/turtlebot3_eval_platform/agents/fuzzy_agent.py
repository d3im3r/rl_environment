#!/usr/bin/env python3

from typing import Optional, Dict
import numpy as np

from turtlebot3_eval_platform.agents.base_agent import BaseAgent


class FuzzyAgent(BaseAgent):
    """
    Agente de Lógica Difusa (Fuzzy Logic Controller) usando el modelo Takagi-Sugeno.
    Calcula velocidades continuas (v, w) basándose en las entradas:
    - d_front (obstáculo al frente)
    - d_left (obstáculo a la izquierda)
    - d_right (obstáculo a la derecha)
    - theta_goal (ángulo hacia la meta)
    """

    def __init__(self):
        super().__init__(name="Fuzzy")
        
        # Mapeo de velocidades para el modelo Sugeno (Singletons de salida)
        self.actions = {
            "recto": np.array([0.18, 0.0]),
            "giro_leve_izq": np.array([0.14, 0.4]),
            "giro_leve_der": np.array([0.14, -0.4]),
            "giro_fuerte_izq": np.array([0.08, 1.0]),
            "giro_fuerte_der": np.array([0.08, -1.0])
        }

    def load(self, model_path: Optional[str] = None) -> bool:
        print("[Fuzzy] Controlador cargado con reglas integradas.")
        return True

    # --- Funciones de Membresía (Fuzzificación) ---

    def _fuzzy_cercano(self, d: float) -> float:
        # Zona de peligro inminente: menor a ~35cm (0.10). Cae a 0 en ~56cm (0.16)
        if d <= 0.10:
            return 1.0
        elif d >= 0.16:
            return 0.0
        return (0.16 - d) / 0.06

    def _fuzzy_medio(self, d: float) -> float:
        # Zona de evitación intermedia: entre 0.10 y 0.35 (~1.22m)
        if d <= 0.10 or d >= 0.35:
            return 0.0
        elif 0.10 < d <= 0.20:
            return (d - 0.10) / 0.10
        else:
            return (0.35 - d) / 0.15

    def _fuzzy_lejano(self, d: float) -> float:
        # Zona libre de obstáculos: mayor a ~70cm (0.20), sube a 1 en ~1.22m (0.35)
        if d <= 0.20:
            return 0.0
        elif d >= 0.35:
            return 1.0
        return (d - 0.20) / 0.15

    def _fuzzy_meta_izquierda(self, theta: float) -> float:
        # Sube a 1 para theta >= 0.08 (~14.4 grados)
        if theta <= 0.0:
            return 0.0
        elif theta >= 0.08:
            return 1.0
        return theta / 0.08

    def _fuzzy_meta_derecha(self, theta: float) -> float:
        # Sube a 1 para theta <= -0.08 (~ -14.4 grados)
        if theta >= 0.0:
            return 0.0
        elif theta <= -0.08:
            return 1.0
        return -theta / 0.08

    def _fuzzy_meta_centro(self, theta: float) -> float:
        # Caída rápida a 0 en -0.08 y 0.08 para mantener el robot bien centrado
        if theta <= -0.08 or theta >= 0.08:
            return 0.0
        elif -0.08 < theta <= 0.0:
            return (theta + 0.08) / 0.08
        else:
            return (0.08 - theta) / 0.08

    def select_action(self, state: np.ndarray) -> np.ndarray:
        # Desempaquetar estado
        d_front, d_left, d_right, theta_goal, _ = state

        # Fuzzificar entradas
        obs_cercano = self._fuzzy_cercano(d_front)
        obs_medio = self._fuzzy_medio(d_front)
        obs_lejano = self._fuzzy_lejano(d_front)

        meta_izq = self._fuzzy_meta_izquierda(theta_goal)
        meta_der = self._fuzzy_meta_derecha(theta_goal)
        meta_centro = self._fuzzy_meta_centro(theta_goal)

        # Reglas difusas (Fuzzy Rules) y activación por método Min/AND
        reglas = []

        # 1. Sin obstáculos (Obstáculo lejano) -> Seguir la meta directamente
        r1 = min(obs_lejano, meta_centro)
        reglas.append((r1, self.actions["recto"]))
        
        r2 = min(obs_lejano, meta_izq)
        reglas.append((r2, self.actions["giro_leve_izq"]))
        
        r3 = min(obs_lejano, meta_der)
        reglas.append((r3, self.actions["giro_leve_der"]))

        # 2. Obstáculo medio -> Combinar avance y evasión
        # Si la meta está al centro pero hay obstáculo medio, preferimos esquivar
        # eligiendo el lado con mayor espacio libre
        if d_left > d_right:
            r4 = min(obs_medio, meta_centro)
            reglas.append((r4, self.actions["giro_leve_izq"]))
        else:
            r4 = min(obs_medio, meta_centro)
            reglas.append((r4, self.actions["giro_leve_der"]))

        r5 = min(obs_medio, meta_izq)
        reglas.append((r5, self.actions["giro_fuerte_izq"]))

        r6 = min(obs_medio, meta_der)
        reglas.append((r6, self.actions["giro_fuerte_der"]))

        # 3. Obstáculo cercano (¡Emergencia!) -> Evasión fuerte obligatoria
        # Se ignora la dirección de la meta temporalmente para no chocar
        if d_left > d_right:
            r7 = obs_cercano
            reglas.append((r7, self.actions["giro_fuerte_izq"]))
        else:
            r7 = obs_cercano
            reglas.append((r7, self.actions["giro_fuerte_der"]))

        # Defuzzificación: Promedio ponderado Sugeno
        numerador = np.zeros(2)
        denominador = 0.0

        for peso, salida in reglas:
            if peso > 0.0:
                numerador += peso * salida
                denominador += peso

        if denominador == 0.0:
            return self.actions["recto"]

        velocidad_calculada = numerador / denominador
        return velocidad_calculada
