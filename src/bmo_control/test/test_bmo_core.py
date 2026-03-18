"""
Unit tests for BMO core algorithm.

Tests each phase independently and then full iteration.
Run with: python3 -m pytest test/test_bmo_core.py -v
No ROS required.
"""

import math
import sys
import os

# Add parent directory to path so we can import bmo_control
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bmo_control.bmo_core import BMOAlgorithm, EPSILON


def test_phase1_uv_update_basic():
    """UV should be b2 * fitness when b1=0 (no memory)."""
    bmo = BMOAlgorithm(n_agents=2, b1=0.0, b2=3.0, step_size=0.1)
    bmo.update_fitness(0, fitness=10.0, position=[0.0, 0.0, 0.0])
    bmo.update_fitness(1, fitness=20.0, position=[1.0, 0.0, 0.0])

    bmo._phase1_uv_update()

    assert bmo.agents[0].uv == 30.0   # 0 * 0 + 3 * 10
    assert bmo.agents[1].uv == 60.0   # 0 * 0 + 3 * 20


def test_phase1_uv_update_with_memory():
    """UV should include previous UV when b1 > 0."""
    bmo = BMOAlgorithm(n_agents=1, b1=0.5, b2=2.0, step_size=0.1)
    bmo.agents[0].uv = 10.0
    bmo.update_fitness(0, fitness=5.0, position=[0.0, 0.0, 0.0])

    bmo._phase1_uv_update()

    assert bmo.agents[0].uv == 15.0   # 0.5 * 10 + 2 * 5


def test_phase1_uv_never_negative():
    """UV should be clamped to 0, never go negative."""
    bmo = BMOAlgorithm(n_agents=1, b1=-1.0, b2=0.0, step_size=0.1)
    bmo.agents[0].uv = 10.0
    bmo.update_fitness(0, fitness=0.0, position=[0.0, 0.0, 0.0])

    bmo._phase1_uv_update()

    assert bmo.agents[0].uv == 0.0


def test_phase2_uv_distribution_closer_gets_more():
    """Agent closer to the source should receive more UV."""
    bmo = BMOAlgorithm(n_agents=3, b1=0.0, b2=1.0, step_size=0.1)

    # Agent 0 has high UV, agents 1 and 2 have none
    bmo.agents[0].uv = 100.0
    bmo.agents[0].position = [0.0, 0.0, 0.0]
    bmo.agents[1].uv = 0.0
    bmo.agents[1].position = [1.0, 0.0, 0.0]   # close
    bmo.agents[2].uv = 0.0
    bmo.agents[2].position = [10.0, 0.0, 0.0]  # far

    bmo._phase2_uv_distribution()

    # Agent 1 (closer) should have higher UV than agent 2 (farther)
    assert bmo.agents[1].uv > bmo.agents[2].uv


def test_phase2_uv_distribution_no_self():
    """Agent should not distribute UV to itself."""
    bmo = BMOAlgorithm(n_agents=2, b1=0.0, b2=1.0, step_size=0.1)
    bmo.agents[0].uv = 100.0
    bmo.agents[0].position = [0.0, 0.0, 0.0]
    bmo.agents[1].uv = 0.0
    bmo.agents[1].position = [1.0, 0.0, 0.0]

    bmo._phase2_uv_distribution()

    # Agent 1 should receive all of agent 0's UV (only 2 agents)
    # Agent 0's UV comes only from agent 1 (which was 0)
    assert bmo.agents[1].uv > 0.0


def test_phase3_lmate_selection():
    """Agent with lower UV should select agent with higher UV as l-mate."""
    bmo = BMOAlgorithm(n_agents=3, b1=0.0, b2=1.0, step_size=0.1)
    bmo.agents[0].uv = 10.0
    bmo.agents[1].uv = 50.0
    bmo.agents[2].uv = 30.0

    bmo._phase3_lmate_selection()

    # Agent 0 (UV=10) should pick agent 1 (UV=50, highest > 10)
    assert bmo.agents[0].lmate_id == 1
    # Agent 2 (UV=30) should pick agent 1 (UV=50, highest > 30)
    assert bmo.agents[2].lmate_id == 1
    # Agent 1 (UV=50) has highest UV — no l-mate (at peak)
    assert bmo.agents[1].lmate_id is None


def test_phase3_no_lmate_at_peak():
    """Agent with highest UV should have no l-mate."""
    bmo = BMOAlgorithm(n_agents=2, b1=0.0, b2=1.0, step_size=0.1)
    bmo.agents[0].uv = 100.0
    bmo.agents[1].uv = 50.0

    bmo._phase3_lmate_selection()

    assert bmo.agents[0].lmate_id is None  # at peak
    assert bmo.agents[1].lmate_id == 0     # moves toward 0


def test_phase4_movement_toward_lmate():
    """Agent should move toward its l-mate."""
    bmo = BMOAlgorithm(n_agents=2, b1=0.0, b2=1.0, step_size=0.1)
    bmo.agents[0].position = [0.0, 0.0, 0.0]
    bmo.agents[0].uv = 10.0    # nonzero UV so random walk doesn't trigger
    bmo.agents[0].lmate_id = 1
    bmo.agents[1].position = [3.0, 4.0, 0.0]
    bmo.agents[1].uv = 50.0
    bmo.agents[1].lmate_id = None

    bmo._phase4_movement()

    # Agent 0 should move toward agent 1 at (3, 4, 0)
    # Direction is (3/5, 4/5, 0), velocity is 0.1 * (0.6, 0.8, 0)
    vx, vy, vz = bmo.agents[0].velocity
    assert abs(vx - 0.06) < 0.001
    assert abs(vy - 0.08) < 0.001
    assert abs(vz) < 0.001

    # Agent 1 has no l-mate — should hold position
    assert bmo.agents[1].velocity == [0.0, 0.0, 0.0]


def test_phase4_colocated_agents():
    """Agents at same position should have zero velocity."""
    bmo = BMOAlgorithm(n_agents=2, b1=0.0, b2=1.0, step_size=0.1)
    bmo.agents[0].position = [1.0, 1.0, 0.0]
    bmo.agents[0].uv = 10.0    # nonzero UV so random walk doesn't trigger
    bmo.agents[0].lmate_id = 1
    bmo.agents[1].position = [1.0, 1.0, 0.0]
    bmo.agents[1].uv = 50.0
    bmo.agents[1].lmate_id = None

    bmo._phase4_movement()

    assert bmo.agents[0].velocity == [0.0, 0.0, 0.0]


def test_phase4_random_walk_when_all_uv_zero():
    """When all UV=0, agents should random walk (nonzero velocity)."""
    bmo = BMOAlgorithm(n_agents=4, b1=0.0, b2=1.0, step_size=0.1)
    for a in bmo.agents:
        a.uv = 0.0
        a.position = [0.0, 0.0, 0.0]

    bmo._phase4_movement()

    # All agents should have nonzero velocity (random direction)
    for a in bmo.agents:
        speed = math.sqrt(a.velocity[0]**2 + a.velocity[1]**2)
        assert abs(speed - 0.1) < 0.001  # magnitude = step_size


def test_full_iteration_convergence_direction():
    """After one iteration, agent with low fitness moves toward high fitness."""
    bmo = BMOAlgorithm(n_agents=3, b1=0.0, b2=3.0, step_size=0.1)

    # Agent 0 is far from lamp, low fitness
    bmo.update_fitness(0, fitness=1.0, position=[-2.0, 0.0, 0.0])
    # Agent 1 is close to lamp, high fitness
    bmo.update_fitness(1, fitness=100.0, position=[2.0, 0.0, 0.0])
    # Agent 2 is medium
    bmo.update_fitness(2, fitness=10.0, position=[0.0, 1.0, 0.0])

    bmo.run_iteration()

    # Agent 0 should have l-mate pointing toward agent 1 (highest UV after dist)
    # and should be moving in positive x direction
    assert bmo.agents[0].velocity[0] > 0


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
