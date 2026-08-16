import os
import torch
import torch.nn as nn
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Bool


class QNetwork(nn.Module):
    """
    Red neuronal Q usada por el agente DQN.

    IMPORTANTE:
    Esta arquitectura debe coincidir exactamente con la arquitectura usada
    durante el entrenamiento en el notebook.
    """

    def __init__(self, state_dim=5, action_dim=3, hidden_dim=128):
        super().__init__()

        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
        )

    def forward(self, x):
        return self.net(x)


class DQNPolicyNode(Node):
    """
    Nodo ROS 2 para inferencia de una política DQN.

    Entradas:
        /rl_state
            std_msgs/msg/Float32MultiArray

        /rl_goal_reached
            std_msgs/msg/Bool

    Salida:
        /rl_action
            std_msgs/msg/Int32

    Acciones:
        0 -> avanzar
        1 -> girar izquierda
        2 -> girar derecha
    """

    def __init__(self):
        super().__init__("dqn_policy_node")

        # ----------------------------------------------------
        # Parámetros ROS 2
        # ----------------------------------------------------
        self.declare_parameter("checkpoint_path", "")
        self.declare_parameter("state_dim", 5)
        self.declare_parameter("action_dim", 3)
        self.declare_parameter("hidden_dim", 128)

        self.declare_parameter("state_topic", "/rl_state")
        self.declare_parameter("action_topic", "/rl_action")
        self.declare_parameter("goal_reached_topic", "/rl_goal_reached")

        self.declare_parameter("min_publish_period", 0.2)
        self.declare_parameter("print_debug", True)

        self.checkpoint_path = (
            self.get_parameter("checkpoint_path")
            .get_parameter_value()
            .string_value
        )

        self.state_dim = (
            self.get_parameter("state_dim")
            .get_parameter_value()
            .integer_value
        )

        self.action_dim = (
            self.get_parameter("action_dim")
            .get_parameter_value()
            .integer_value
        )

        self.hidden_dim = (
            self.get_parameter("hidden_dim")
            .get_parameter_value()
            .integer_value
        )

        self.state_topic = (
            self.get_parameter("state_topic")
            .get_parameter_value()
            .string_value
        )

        self.action_topic = (
            self.get_parameter("action_topic")
            .get_parameter_value()
            .string_value
        )

        self.goal_reached_topic = (
            self.get_parameter("goal_reached_topic")
            .get_parameter_value()
            .string_value
        )

        self.min_publish_period = (
            self.get_parameter("min_publish_period")
            .get_parameter_value()
            .double_value
        )

        self.print_debug = (
            self.get_parameter("print_debug")
            .get_parameter_value()
            .bool_value
        )

        # ----------------------------------------------------
        # Estado interno del nodo
        # ----------------------------------------------------
        self.device = torch.device("cpu")
        self.goal_reached = False
        self.last_publish_time = self.get_clock().now()

        # ----------------------------------------------------
        # Crear y cargar red DQN
        # ----------------------------------------------------
        self.q_net = QNetwork(
            state_dim=self.state_dim,
            action_dim=self.action_dim,
            hidden_dim=self.hidden_dim,
        ).to(self.device)

        self.load_checkpoint(self.checkpoint_path)

        self.q_net.eval()

        # ----------------------------------------------------
        # Publicador de acciones
        # ----------------------------------------------------
        self.action_pub = self.create_publisher(
            Int32,
            self.action_topic,
            10,
        )

        # ----------------------------------------------------
        # Suscriptor al estado RL
        # ----------------------------------------------------
        self.state_sub = self.create_subscription(
            Float32MultiArray,
            self.state_topic,
            self.state_callback,
            10,
        )

        # ----------------------------------------------------
        # Suscriptor a meta alcanzada
        # ----------------------------------------------------
        self.goal_reached_sub = self.create_subscription(
            Bool,
            self.goal_reached_topic,
            self.goal_reached_callback,
            10,
        )

        self.get_logger().info("Nodo DQN Policy iniciado correctamente.")
        self.get_logger().info(f"Escuchando estado en: {self.state_topic}")
        self.get_logger().info(
            f"Escuchando meta alcanzada en: {self.goal_reached_topic}"
        )
        self.get_logger().info(f"Publicando acción en: {self.action_topic}")
        self.get_logger().info(f"Checkpoint: {self.checkpoint_path}")
        self.get_logger().info(
            f"Arquitectura: state_dim={self.state_dim}, "
            f"hidden_dim={self.hidden_dim}, action_dim={self.action_dim}"
        )

    def load_checkpoint(self, checkpoint_path):
        """
        Carga los pesos del modelo entrenado.

        El método intenta ser flexible con el formato del checkpoint.
        Soporta checkpoints con claves:
            - model_state_dict
            - q_net_state_dict
            - state_dict

        También soporta el caso en que el archivo sea directamente
        el state_dict de PyTorch.
        """

        if checkpoint_path == "":
            raise ValueError(
                "El parámetro 'checkpoint_path' está vacío. "
                "Debes indicar la ruta completa al archivo best_model.pth."
            )

        if not os.path.exists(checkpoint_path):
            raise FileNotFoundError(
                f"No se encontró el checkpoint: {checkpoint_path}"
            )

        checkpoint = torch.load(
            checkpoint_path,
            map_location=self.device,
        )

        if not isinstance(checkpoint, dict):
            raise ValueError(
                "Formato de checkpoint no reconocido. "
                "Se esperaba un diccionario de PyTorch."
            )

        if "model_state_dict" in checkpoint:
            state_dict = checkpoint["model_state_dict"]
        elif "q_net_state_dict" in checkpoint:
            state_dict = checkpoint["q_net_state_dict"]
        elif "state_dict" in checkpoint:
            state_dict = checkpoint["state_dict"]
        else:
            # Caso en que el checkpoint ya sea directamente el state_dict.
            state_dict = checkpoint

        self.q_net.load_state_dict(state_dict)

        self.get_logger().info("Modelo DQN cargado correctamente.")

        if isinstance(checkpoint, dict):
            if "episode" in checkpoint:
                self.get_logger().info(
                    f"Checkpoint correspondiente al episodio: {checkpoint['episode']}"
                )

            if "best_score" in checkpoint:
                self.get_logger().info(
                    f"Best score guardado: {checkpoint['best_score']}"
                )

            if "eval_metrics" in checkpoint:
                self.get_logger().info(
                    f"Métricas de evaluación: {checkpoint['eval_metrics']}"
                )

    def goal_reached_callback(self, msg):
        """
        Callback que recibe si la meta fue alcanzada.

        Si goal_reached es True, este nodo deja de publicar /rl_action.
        Como el controlador se detiene cuando deja de recibir acciones,
        no se necesita una acción adicional de parada.
        """

        previous_state = self.goal_reached
        self.goal_reached = bool(msg.data)

        if self.goal_reached and not previous_state:
            self.get_logger().info(
                "Meta alcanzada. Se detiene la publicación de /rl_action."
            )

        if not self.goal_reached and previous_state:
            self.get_logger().info(
                "Meta restablecida. Se reanuda la política DQN."
            )

    def state_callback(self, msg):
        """
        Recibe el estado normalizado, calcula la acción con la red DQN
        y publica la acción discreta.

        Si la meta ya fue alcanzada, no publica acción.
        """

        # ----------------------------------------------------
        # Si la meta fue alcanzada, no se publican más acciones.
        # ----------------------------------------------------
        if self.goal_reached:
            return

        # ----------------------------------------------------
        # Control de frecuencia de publicación
        # ----------------------------------------------------
        now = self.get_clock().now()
        elapsed = (now - self.last_publish_time).nanoseconds / 1e9

        if elapsed < self.min_publish_period:
            return

        self.last_publish_time = now

        # ----------------------------------------------------
        # Validar estado recibido
        # ----------------------------------------------------
        state = np.array(msg.data, dtype=np.float32)

        if state.shape[0] != self.state_dim:
            self.get_logger().warn(
                f"Estado inválido. Se esperaban {self.state_dim} valores, "
                f"pero llegaron {state.shape[0]}."
            )
            return

        if not np.all(np.isfinite(state)):
            self.get_logger().warn(
                "Estado inválido: contiene NaN o valores infinitos."
            )
            return

        # ----------------------------------------------------
        # Inferencia DQN
        # ----------------------------------------------------
        state_tensor = torch.tensor(
            state,
            dtype=torch.float32,
            device=self.device,
        ).unsqueeze(0)

        with torch.no_grad():
            q_values = self.q_net(state_tensor)

        action = int(torch.argmax(q_values, dim=1).item())

        # ----------------------------------------------------
        # Publicar acción
        # ----------------------------------------------------
        action_msg = Int32()
        action_msg.data = action

        self.action_pub.publish(action_msg)

        # ----------------------------------------------------
        # Debug
        # ----------------------------------------------------
        if self.print_debug:
            q_values_np = q_values.cpu().numpy().flatten()

            self.get_logger().info(
                f"state={np.round(state, 3).tolist()} | "
                f"q={np.round(q_values_np, 3).tolist()} | "
                f"action={action} ({self.action_name(action)})"
            )

    def action_name(self, action):
        """
        Traduce la acción discreta a texto.
        """

        if action == 0:
            return "avanzar"

        if action == 1:
            return "girar_izquierda"

        if action == 2:
            return "girar_derecha"

        return "desconocida"


def main(args=None):
    rclpy.init(args=args)

    node = DQNPolicyNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Nodo DQN detenido por el usuario.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()