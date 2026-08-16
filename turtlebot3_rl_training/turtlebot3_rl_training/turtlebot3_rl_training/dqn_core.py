import random
from collections import deque
from dataclasses import dataclass
from typing import Deque, Tuple, List, Dict, Any

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim


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


@dataclass
class Transition:
    state: np.ndarray
    action: int
    reward: float
    next_state: np.ndarray
    done: bool


class ReplayBuffer:
    def __init__(self, capacity: int):
        self.buffer: Deque[Transition] = deque(maxlen=capacity)

    def push(
        self,
        state: np.ndarray,
        action: int,
        reward: float,
        next_state: np.ndarray,
        done: bool
    ):
        self.buffer.append(
            Transition(
                state=np.array(state, dtype=np.float32),
                action=int(action),
                reward=float(reward),
                next_state=np.array(next_state, dtype=np.float32),
                done=bool(done)
            )
        )

    def sample(self, batch_size: int) -> Tuple[torch.Tensor, ...]:
        batch = random.sample(self.buffer, batch_size)

        states = torch.tensor(
            np.array([t.state for t in batch]),
            dtype=torch.float32
        )

        actions = torch.tensor(
            [t.action for t in batch],
            dtype=torch.long
        ).unsqueeze(1)

        rewards = torch.tensor(
            [t.reward for t in batch],
            dtype=torch.float32
        ).unsqueeze(1)

        next_states = torch.tensor(
            np.array([t.next_state for t in batch]),
            dtype=torch.float32
        )

        dones = torch.tensor(
            [t.done for t in batch],
            dtype=torch.float32
        ).unsqueeze(1)

        return states, actions, rewards, next_states, dones

    def __len__(self):
        return len(self.buffer)


def select_action(
    q_network: QNetwork,
    state: np.ndarray,
    epsilon: float,
    action_dim: int,
    device: torch.device
) -> int:
    if random.random() < epsilon:
        return random.randrange(action_dim)

    with torch.no_grad():
        state_tensor = torch.tensor(
            state,
            dtype=torch.float32,
            device=device
        ).unsqueeze(0)

        q_values = q_network(state_tensor)
        return int(torch.argmax(q_values, dim=1).item())


def train_step(
    q_network: QNetwork,
    target_network: QNetwork,
    replay_buffer: ReplayBuffer,
    optimizer: optim.Optimizer,
    batch_size: int,
    gamma: float,
    device: torch.device
) -> float:
    if len(replay_buffer) < batch_size:
        return 0.0

    states, actions, rewards, next_states, dones = replay_buffer.sample(batch_size)

    states = states.to(device)
    actions = actions.to(device)
    rewards = rewards.to(device)
    next_states = next_states.to(device)
    dones = dones.to(device)

    q_values = q_network(states).gather(1, actions)

    with torch.no_grad():
        next_q_values = target_network(next_states).max(dim=1, keepdim=True)[0]
        target_q_values = rewards + gamma * next_q_values * (1.0 - dones)

    # Huber Loss (Smooth L1) is more stable than MSE when TD errors are large.
    # This helps avoid destructive updates when continuing from a good checkpoint.
    loss = nn.functional.smooth_l1_loss(q_values, target_q_values)

    optimizer.zero_grad()
    loss.backward()

    # Gradient clipping prevents very large updates from destabilizing the policy.
    torch.nn.utils.clip_grad_norm_(
        q_network.parameters(),
        max_norm=1.0
    )

    optimizer.step()

    return float(loss.item())


def save_checkpoint(
    path: str,
    q_network: QNetwork,
    target_network: QNetwork,
    optimizer: optim.Optimizer,
    metadata: Dict[str, Any]
):
    checkpoint = {
        "q_network_state_dict": q_network.state_dict(),
        "target_network_state_dict": target_network.state_dict(),
        "optimizer_state_dict": optimizer.state_dict(),
        "metadata": metadata
    }

    torch.save(checkpoint, path)


def load_q_network_from_checkpoint(
    path: str,
    state_dim: int,
    action_dim: int,
    device: torch.device
) -> QNetwork:
    checkpoint = torch.load(path, map_location=device)

    q_network = QNetwork(
        state_dim=state_dim,
        action_dim=action_dim
    ).to(device)

    q_network.load_state_dict(checkpoint["q_network_state_dict"])
    q_network.eval()

    return q_network