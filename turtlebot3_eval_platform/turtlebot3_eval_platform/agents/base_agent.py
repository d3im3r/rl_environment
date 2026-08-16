#!/usr/bin/env python3

from abc import ABC, abstractmethod
from typing import Optional
import numpy as np

class BaseAgent(ABC):
    """
    Clase abstracta base para todos los controladores del área de pruebas.
    """

    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def load(self, model_path: Optional[str] = None) -> bool:
        """Carga pesos del modelo o parámetros de calibración."""
        pass

    @abstractmethod
    def select_action(self, state: np.ndarray) -> np.ndarray:
        """
        Dada una observación normalizada:
        [d_front, d_left, d_right, theta_goal, d_goal]
        
        Debe retornar una acción continua:
        np.array([velocidad_lineal, velocidad_angular])
        """
        pass
