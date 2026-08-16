#!/usr/bin/env python3

from typing import Optional
import numpy as np
import torch
import torch.nn as nn

from turtlebot3_eval_platform.agents.base_agent import BaseAgent


class QNetwork(nn.Module):
    def __init__(self, state_dim: int = 5, action_dim: int = 5, hidden_dim: int = 128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim)
        )

    def forward(self, x):
        return self.net(x)


class DQNAgent(BaseAgent):
    """
    Agente DQN que carga un archivo de checkpoint (.pth)
    y mapea sus 5 salidas discretas a las 5 velocidades continuas del robot.
    """

    def __init__(self, state_dim: int = 5, action_dim: int = 5, hidden_dim: int = 128):
        super().__init__(name="DQN")
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.hidden_dim = hidden_dim
        
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.q_network = QNetwork(state_dim, action_dim, hidden_dim).to(self.device)
        self.q_network.eval()

        # Diccionario de mapeo: 5 acciones que siempre se desplazan hacia adelante
        # 0 = Avanzar recto (5ta acción en la descripción del usuario)
        # 1 = Giro leve izquierda
        # 2 = Giro leve derecha
        # 3 = Giro pronunciado izquierda
        # 4 = Giro pronunciado derecha
        self.action_mapping = {
            0: np.array([0.18,  0.0]),   # Recto
            1: np.array([0.14,  0.4]),   # Leve Izquierda
            2: np.array([0.14, -0.4]),   # Leve Derecha
            3: np.array([0.08,  1.0]),   # Pronunciado Izquierda
            4: np.array([0.08, -1.0])    # Pronunciado Derecha
        }

    def load(self, model_path: Optional[str] = None) -> bool:
        if model_path is None or model_path == "":
            print("[DQN] No se especificó ruta del modelo. Utilizando pesos aleatorios con estructura de 5 acciones.")
            return False

        try:
            checkpoint = torch.load(model_path, map_location=self.device)
            state_dict = None
            
            # Extraer el state_dict correcto
            if isinstance(checkpoint, dict):
                state_dict_keys = ["q_net_state_dict", "q_network_state_dict", "model_state_dict", "state_dict"]
                for key in state_dict_keys:
                    if key in checkpoint:
                        state_dict = checkpoint[key]
                        break
                if state_dict is None:
                    # Si no hay llave conocida, podría ser el state_dict directamente
                    state_dict = checkpoint
            else:
                state_dict = checkpoint

            # Inspeccionar dimensiones del state_dict
            if state_dict is not None and "net.0.weight" in state_dict and "net.4.weight" in state_dict:
                state_dim = state_dict["net.0.weight"].shape[1]
                hidden_dim = state_dict["net.0.weight"].shape[0]
                action_dim = state_dict["net.4.weight"].shape[0]
                
                print(f"[DQN] Dimensiones detectadas en checkpoint: state_dim={state_dim}, action_dim={action_dim}, hidden_dim={hidden_dim}")
                
                # Reconfigurar red y mapeo de acciones dinámicamente
                self.state_dim = state_dim
                self.action_dim = action_dim
                self.hidden_dim = hidden_dim
                
                self.q_network = QNetwork(state_dim, action_dim, hidden_dim).to(self.device)
                self.q_network.load_state_dict(state_dict)
                
                # Mapeo según cantidad de acciones
                if action_dim == 3:
                    self.action_mapping = {
                        0: np.array([0.18,  0.0]),   # Avanzar recto
                        1: np.array([0.0,   1.0]),   # Rotar izquierda (in situ)
                        2: np.array([0.0,  -1.0])    # Rotar derecha (in situ)
                    }
                    print("[DQN] Mapeo de 3 acciones cargado.")
                else:
                    self.action_mapping = {
                        0: np.array([0.18,  0.0]),   # Recto
                        1: np.array([0.14,  0.4]),   # Leve Izquierda
                        2: np.array([0.14, -0.4]),   # Leve Derecha
                        3: np.array([0.08,  1.0]),   # Pronunciado Izquierda
                        4: np.array([0.08, -1.0])    # Pronunciado Derecha
                    }
                    print("[DQN] Mapeo de 5 acciones cargado.")
            else:
                raise KeyError("El state_dict no contiene las llaves net.0.weight y net.4.weight esperadas.")

            self.q_network.eval()
            print("[DQN] Modelo y parámetros cargados con éxito.")
            return True
        except Exception as e:
            print(f"[DQN] Error al cargar los pesos del modelo: {e}")
            return False

    def select_action(self, state: np.ndarray) -> np.ndarray:
        with torch.no_grad():
            state_t = torch.tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)
            q_values = self.q_network(state_t)
            action_idx = int(torch.argmax(q_values, dim=1).item())

        return self.action_mapping[action_idx]
