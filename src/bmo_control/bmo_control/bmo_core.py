"""
BMO Core Algorithm — Pure Python (NO ROS dependency)

Implements the 4-phase Butterfly Mating Optimization algorithm:
  Phase 1: UV Update       — each agent updates UV from sensor fitness
  Phase 2: UV Distribution — agents share UV weighted by inverse distance
  Phase 3: l-mate Selection — each agent picks a mate with higher UV
  Phase 4: Movement        — each agent moves toward its l-mate

This module is intentionally ROS-free so it can be:
  - Unit tested without ROS installed (pytest)
  - Reused in other contexts (embedded, different simulators)
  - Understood independently from the ROS wiring

Reference: "Butterfly Mating Optimization" paper
See ~/guides/Butterfly_Mating_Optimization.pdf for full details.
"""

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional


# Minimum distance to prevent division by zero in UV distribution and movement.
# 0.01m = 1cm — smaller than any meaningful separation between drones.
EPSILON = 0.01

# Collision avoidance parameters.
# When two drones are closer than SAFE_DISTANCE, a repulsive velocity is added
# pushing them apart. This prevents physical collisions in simulation and
# real hardware. The repulsive strength increases as distance decreases.
SAFE_DISTANCE = 0.35   # meters — ~3.5x Crazyflie body diameter (0.1m)
REPULSION_GAIN = 0.3   # m/s at exactly SAFE_DISTANCE boundary


@dataclass
class BMOAgent:
    """State of a single BMO agent (one Crazyflie drone).

    Attributes:
        agent_id:        Unique identifier (0, 1, 2, 3)
        position:        Current [x, y] position in meters
        uv:              Current UV value (attractiveness/fitness)
        fitness:         Latest sensor reading (light intensity in lux)
        lmate_id:        ID of selected local mate (None if at peak)
        velocity:        Computed [vx, vy] velocity command in m/s
    """
    agent_id: int
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    uv: float = 0.0
    fitness: float = 0.0
    lmate_id: Optional[int] = None
    velocity: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


class BMOAlgorithm:
    """Implements the 4-phase BMO algorithm for N agents.

    Parameters:
        n_agents:   Number of agents (drones)
        b1:         Weight for previous UV value (memory). 0 = no memory.
        b2:         Weight for current fitness. Higher = more reactive.
        step_size:  Movement speed in m/s (B_s in the paper)

    Usage:
        bmo = BMOAlgorithm(n_agents=4, b1=0.0, b2=3.0, step_size=0.1)
        bmo.update_fitness(agent_id=0, fitness=42.5, position=[1.0, 2.0])
        bmo.run_iteration()
        velocity = bmo.agents[0].velocity
    """

    def __init__(self, n_agents: int, b1: float, b2: float, step_size: float):
        self.n_agents = n_agents
        self.b1 = b1
        self.b2 = b2
        self.step_size = step_size
        self.agents = [BMOAgent(agent_id=i) for i in range(n_agents)]

    def update_fitness(self, agent_id: int, fitness: float,
                       position: List[float]):
        """Update an agent's sensor reading and position.

        Called by the ROS coordinator node each time new data arrives
        from /cfN/light_intensity and /cfN/odom.

        Args:
            agent_id: Which agent (0-3)
            fitness:  Light intensity reading (lux)
            position: [x, y] position from odometry (meters)
        """
        agent = self.agents[agent_id]
        agent.fitness = fitness
        agent.position = list(position)

    def run_iteration(self):
        """Execute one full BMO iteration (all 4 phases + collision avoidance).

        Call this at the BMO iteration rate (e.g. 10 Hz).
        After calling, each agent's .velocity field contains the
        commanded [vx, vy] to publish on /cfN/cmd_vel.
        """
        self._phase1_uv_update()
        self._phase2_uv_distribution()
        self._phase3_lmate_selection()
        self._phase4_movement()
        self._apply_collision_avoidance()

    # ------------------------------------------------------------------
    # PHASE 1: UV UPDATE (BMO Paper, Equation 1)
    #
    # Each agent updates its UV value based on current sensor reading.
    # b1 controls "memory" of past UV (0 = no memory, pure reactive).
    # b2 scales the current fitness — higher means stronger reaction
    # to what the drone currently senses.
    #
    # Equation: UV_i(t) = max(0, b1 * UV_i(t-1) + b2 * f(i))
    # ------------------------------------------------------------------
    def _phase1_uv_update(self):
        for agent in self.agents:
            agent.uv = max(0.0, self.b1 * agent.uv + self.b2 * agent.fitness)

    # ------------------------------------------------------------------
    # PHASE 2: UV DISTRIBUTION (BMO Paper, Equation 2)
    #
    # Each agent distributes its UV to all OTHER agents, weighted by
    # inverse distance. Closer agents receive more UV.
    #
    # UV absorbed by j from i:
    #   UV_{i->j} = UV_i * (1/d_ij) / sum_k(1/d_ik)  for k != i
    #
    # After distribution, each agent's UV is the SUM of UV received
    # from all other agents.
    #
    # IMPORTANT: sum is over k != i (never include self).
    # If d_ij < EPSILON, clamp to EPSILON to avoid division by zero.
    # ------------------------------------------------------------------
    def _phase2_uv_distribution(self):
        n = self.n_agents
        # Matrix of UV received: received[j] = total UV agent j absorbs
        received = [0.0] * n

        for i in range(n):
            # Compute inverse distances from agent i to all others
            inv_distances = []
            for k in range(n):
                if k == i:
                    inv_distances.append(0.0)  # skip self
                    continue
                d = self._distance(self.agents[i], self.agents[k])
                d = max(d, EPSILON)  # clamp to avoid div by zero
                inv_distances.append(1.0 / d)

            # Sum of inverse distances (excluding self)
            inv_sum = sum(inv_distances)
            if inv_sum < 1e-12:
                # All agents co-located — distribute UV equally
                continue

            # Distribute UV from agent i to all others
            for j in range(n):
                if j == i:
                    continue
                weight = inv_distances[j] / inv_sum
                received[j] += self.agents[i].uv * weight

        # Add distributed UV to each agent's own UV.
        # The agent keeps its UV from Phase 1 (based on its own fitness)
        # and ADDS the UV received from neighbors. This ensures that an
        # agent near the lamp (high own UV) stays attractive even if
        # far from other agents.
        for j in range(n):
            self.agents[j].uv += received[j]

    # ------------------------------------------------------------------
    # PHASE 3: l-MATE SELECTION (BMO Paper, Equation 3)
    #
    # Each agent selects a "local mate" — the best nearby agent to
    # move toward.
    #
    # Procedure for agent i:
    #   1. Sort all OTHER agents by UV in descending order
    #   2. Pick the first agent j where UV_j > UV_i
    #   3. If no such j exists, agent i is at a peak → no l-mate
    #
    # This adaptive selection is what gives BMO its multi-peak
    # capturing ability. Agents with high UV attract others; agents
    # already at peaks stop moving.
    # ------------------------------------------------------------------
    def _phase3_lmate_selection(self):
        for agent in self.agents:
            # Get all other agents sorted by UV (highest first)
            others = sorted(
                [a for a in self.agents if a.agent_id != agent.agent_id],
                key=lambda a: a.uv,
                reverse=True
            )

            # Pick first agent with UV higher than ours
            agent.lmate_id = None
            for candidate in others:
                if candidate.uv > agent.uv:
                    agent.lmate_id = candidate.agent_id
                    break

    # ------------------------------------------------------------------
    # PHASE 4: MOVEMENT (BMO Paper, Equation 4)
    #
    # Each agent moves toward its l-mate at fixed speed (step_size).
    #
    # Equation:
    #   direction = (pos_lmate - pos_i) / ||pos_lmate - pos_i||
    #   velocity  = step_size * direction
    #
    # Edge cases:
    #   - No l-mate (at peak): velocity = [0, 0]
    #   - ||pos_lmate - pos_i|| < EPSILON: velocity = [0, 0]
    #   - All UV = 0: random walk (explore until fitness is nonzero)
    # ------------------------------------------------------------------
    def _phase4_movement(self):
        # Check if all agents have zero UV — if so, do random walk
        all_zero = all(agent.uv < 1e-12 for agent in self.agents)

        for agent in self.agents:
            if all_zero:
                # ---- Random walk: explore in a random direction (3D) ----
                theta = random.uniform(0, 2 * math.pi)
                agent.velocity = [
                    self.step_size * math.cos(theta),
                    self.step_size * math.sin(theta),
                    0.0  # no random vertical movement
                ]
                continue

            if agent.lmate_id is None:
                # At a peak — hold position
                agent.velocity = [0.0, 0.0, 0.0]
                continue

            lmate = self.agents[agent.lmate_id]
            dx = lmate.position[0] - agent.position[0]
            dy = lmate.position[1] - agent.position[1]
            dz = lmate.position[2] - agent.position[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            if dist < EPSILON:
                # Already at l-mate position — hold
                agent.velocity = [0.0, 0.0, 0.0]
                continue

            # Normalized direction * step_size (3D)
            agent.velocity = [
                self.step_size * dx / dist,
                self.step_size * dy / dist,
                self.step_size * dz / dist
            ]

    # ------------------------------------------------------------------
    # COLLISION AVOIDANCE (not part of BMO paper — engineering addition)
    #
    # When two drones are closer than SAFE_DISTANCE, add a repulsive
    # velocity pushing them apart. The repulsion is inversely proportional
    # to distance — stronger when closer.
    #
    # For each agent, sum repulsive vectors from all neighbors within
    # SAFE_DISTANCE, then add to the BMO velocity. If repulsion is
    # stronger than BMO attraction, the drone moves away (correct —
    # collision avoidance takes priority over convergence).
    #
    # This is essential for the single-source case where all drones
    # converge to the same lamp position.
    # ------------------------------------------------------------------
    def _apply_collision_avoidance(self):
        for i, agent in enumerate(self.agents):
            repulsion = [0.0, 0.0, 0.0]

            for j, other in enumerate(self.agents):
                if i == j:
                    continue

                dx = agent.position[0] - other.position[0]
                dy = agent.position[1] - other.position[1]
                dz = agent.position[2] - other.position[2]
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)

                if dist < SAFE_DISTANCE and dist > EPSILON:
                    strength = REPULSION_GAIN * (SAFE_DISTANCE / dist - 1.0)
                    repulsion[0] += strength * dx / dist
                    repulsion[1] += strength * dy / dist
                    repulsion[2] += strength * dz / dist

            agent.velocity[0] += repulsion[0]
            agent.velocity[1] += repulsion[1]
            agent.velocity[2] += repulsion[2]

    @staticmethod
    def _distance(a: BMOAgent, b: BMOAgent) -> float:
        """Euclidean distance between two agents (3D)."""
        dx = a.position[0] - b.position[0]
        dy = a.position[1] - b.position[1]
        dz = a.position[2] - b.position[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)
