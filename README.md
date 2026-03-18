# BMO Swarm — Butterfly Mating Optimization on Crazyflie Drones

A swarm robotics system that uses **Butterfly Mating Optimization (BMO)** — a nature-inspired algorithm — to guide a fleet of [Crazyflie 2.1+](https://www.bitcraze.io/products/crazyflie-2-1/) nano-quadrotors toward signal sources (tungsten lamps) autonomously.

> **In plain English:** Imagine releasing four tiny drones into a room with a hidden heat lamp. Without any pre-programmed map, the drones "sniff out" the lamp's light, share clues with their neighbors, and collectively swarm toward the source — much like butterflies find mates by following pheromone trails.

---

## Table of Contents

1. [How It Works — The Big Picture](#how-it-works--the-big-picture)
2. [System Architecture](#system-architecture)
3. [Package Overview](#package-overview)
4. [Node Responsibilities](#node-responsibilities)
5. [The BMO Algorithm](#the-bmo-algorithm)
6. [Data Flow](#data-flow)
7. [World Setup](#world-setup)
8. [Configuration Reference](#configuration-reference)
9. [Getting Started](#getting-started)
10. [Running Tests](#running-tests)
11. [Project Structure](#project-structure)

---

## How It Works — The Big Picture

```
  Tungsten Lamp (signal source)
         *
        /|\         ← light radiates outward
       / | \
      /  |  \
   cf2  cf3  ...    ← drones sense light intensity
    \   |   /
     \  |  /
      SWARM         ← drones communicate & converge on the source
```

1. **Four drones** start at spread-out positions in a simulated arena.
2. Each drone carries a **light sensor** that measures how bright the lamp appears from its current position (brighter = closer).
3. A **coordinator** collects every drone's position and sensor reading, then runs the BMO algorithm to decide where each drone should fly next.
4. Drones that sense stronger light attract their neighbors — just like butterflies releasing UV pheromones attract mates.
5. Over 30–60 seconds, **all four drones converge** on the lamp's location without any drone knowing the lamp's position in advance.

---

## System Architecture

The system is composed of three layers that communicate through ROS 2 topics:

```
┌─────────────────────────────────────────────────────────────────────┐
│                        DECISION LAYER (ROS 2)                       │
│                                                                     │
│   ┌───────────────────────────────────────────────────────────┐     │
│   │              BMO Coordinator Node (Python)                │     │
│   │                                                           │     │
│   │  Subscribes to:           Publishes:                      │     │
│   │    /cf{0-3}/odom            /cf{0-3}/cmd_vel              │     │
│   │    /cf{0-3}/light_intensity /cf{0-3}/enable               │     │
│   │                                                           │     │
│   │  Runs the BMO algorithm every 100 ms (10 Hz)             │     │
│   └───────────────────────────────────────────────────────────┘     │
│                          ▲              │                            │
│                          │              ▼                            │
├─────────────────────────────────────────────────────────────────────┤
│                      BRIDGE LAYER (ros_gz_bridge)                   │
│                                                                     │
│   Translates topics between ROS 2 ↔ Gazebo Harmonic                │
│   17 bridge entries: 4 topics × 4 drones + 1 clock                 │
│                          ▲              │                            │
│                          │              ▼                            │
├─────────────────────────────────────────────────────────────────────┤
│                    SIMULATION LAYER (Gazebo Harmonic)                │
│                                                                     │
│   ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐      │
│   │  cf0  cf1    │  │ Tungsten     │  │ Light Intensity     │      │
│   │  cf2  cf3    │  │ Lamp         │  │ Sensor Plugin (C++) │      │
│   │  (Crazyflie  │  │ (signal      │  │                     │      │
│   │   models)    │  │  source)     │  │ Computes lux at     │      │
│   │              │  │              │  │ each drone using    │      │
│   │  Physics,    │  │  60W bulb,   │  │ inverse-square law  │      │
│   │  motors,     │  │  at (3,0,0.3)│  │ + Gaussian noise    │      │
│   │  velocity    │  │              │  │                     │      │
│   │  controllers │  │              │  │ Publishes per-drone │      │
│   │              │  │              │  │ light_intensity     │      │
│   └──────────────┘  └──────────────┘  └─────────────────────┘      │
│                                                                     │
│   World: single_source_world.sdf                                    │
│   Physics: 1 ms timestep, real-time factor 1.0                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Package Overview

The workspace contains **three packages**, each with a distinct responsibility:

| Package | Language | Purpose |
|---------|----------|---------|
| **bmo_control** | Python | The "brain" — runs the BMO algorithm and sends velocity commands to each drone |
| **bmo_sim** | SDF / YAML | The "world" — defines the simulation environment, drone models, lamp, and topic bridges |
| **bmo_gz_plugins** | C++ | The "sensor" — a Gazebo plugin that simulates how much light each drone sees |

### Why three packages?

- **Separation of concerns:** The algorithm knows nothing about physics. The physics simulation knows nothing about the algorithm. The bridge layer connects them.
- **Testability:** The algorithm can be tested with pure unit tests — no simulator required.
- **Reusability:** Swap the simulator for real hardware by replacing only the bridge and plugin layers.

---

## Node Responsibilities

### 1. BMO Coordinator Node

**What it is:** The single decision-maker for the entire swarm. It is a ROS 2 node written in Python.

**What it does:**

| Responsibility | Details |
|----------------|---------|
| **Collects sensor data** | Subscribes to each drone's position (odometry) and light intensity reading |
| **Runs the BMO algorithm** | Every 100 ms, computes a new velocity for each drone based on the algorithm's four phases |
| **Sends flight commands** | Publishes velocity commands (`cmd_vel`) telling each drone which direction to fly and how fast |
| **Manages the flight state machine** | Handles the lifecycle: wait for data → take off → run algorithm → stop when converged |
| **Ensures safety** | Collision avoidance between drones; altitude recovery if a drone drops too low |

**State machine:**

```
  WAITING ──► TAKEOFF ──► RUNNING ──► STOPPED
  (collect     (climb to    (execute     (converged
   initial      0.5m)        BMO loop)    or manual
   data)                                  stop)
```

**Source file:** `src/bmo_control/bmo_control/bmo_coordinator_node.py`

---

### 2. Light Intensity Sensor Plugin

**What it is:** A C++ plugin loaded into the Gazebo simulator that acts as a virtual light sensor on each drone.

**What it does:**

| Responsibility | Details |
|----------------|---------|
| **Simulates light physics** | Computes illuminance (in lux) at each drone's position using the inverse-square law |
| **Models sensor noise** | Adds realistic Gaussian noise (default: 0.5 lux standard deviation) to simulate real-world sensor imperfections |
| **Publishes readings** | Outputs a light intensity value for each drone at 10 Hz on Gazebo transport topics |
| **Supports multiple lamps** | Sums contributions from all lamps in the world (currently one, extensible to many) |

**Physics formula:**

```
Illuminance = (Power × Efficacy) / (4π × distance²) + noise

Where:
  Power    = 60 watts (tungsten lamp)
  Efficacy = 15 lumens/watt (tungsten at ~2700K)
  distance = straight-line distance from drone to lamp (meters)
  noise    = random value from Normal(0, 0.5²)
```

**Source file:** `src/bmo_gz_plugins/src/light_intensity_sensor.cpp`

---

### 3. ROS-Gazebo Bridge

**What it is:** Not a custom node, but a configuration of the standard `ros_gz_bridge` package that translates messages between ROS 2 and Gazebo.

**What it does:**

| Direction | Topic | Message Type | Purpose |
|-----------|-------|--------------|---------|
| Gazebo → ROS | `/cf{N}/odom` | Odometry | Drone position and orientation |
| Gazebo → ROS | `/cf{N}/light_intensity` | Float64 | Light sensor reading (lux) |
| ROS → Gazebo | `/cf{N}/cmd_vel` | Twist | Velocity command (m/s) |
| ROS → Gazebo | `/cf{N}/enable` | Bool | Activate the drone's velocity controller |
| Gazebo → ROS | `/clock` | Clock | Simulation time synchronization |

*Where `{N}` = 0, 1, 2, 3 — one set per drone.*

**Config file:** `src/bmo_sim/config/bmo_bridge.yaml`

---

## The BMO Algorithm

The Butterfly Mating Optimization algorithm mimics how butterflies find mates using ultraviolet (UV) pheromones. In our system, "UV" represents how attractive a drone's current location is — higher light intensity means higher UV.

The algorithm runs **four phases** every iteration (10 times per second):

### Phase 1 — UV Update

Each drone computes its attractiveness based on its current sensor reading:

```
UV = max(0,  b1 × previous_UV  +  b2 × current_light_reading)
```

- `b1 = 0.0` (memory weight — currently no memory, fully reactive)
- `b2 = 3.0` (fitness weight — amplifies current sensor reading)

**In plain English:** A drone near the lamp gets a high UV score. A drone far away gets a low one.

### Phase 2 — UV Distribution

Drones share UV with their neighbors, weighted by proximity:

```
UV sent from drone A to drone B = UV_A × (1/distance_AB) / (sum of 1/distance to all others)
```

**In plain English:** If you're close to a high-UV drone, you receive more of its UV — like smelling a stronger scent when you're near the source.

### Phase 3 — Mate Selection

Each drone picks a **"mate"** — the first drone it finds with a higher UV score:

- If a drone has the highest UV in the swarm, it has no mate (it's already at the best spot — it stays put).
- If all UV scores are zero, no mates are selected.

**In plain English:** Each drone asks, "Who nearby is doing better than me?" and picks that drone as its guide.

### Phase 4 — Movement

| Situation | Action |
|-----------|--------|
| All UV = 0 (no signal detected yet) | Random walk — explore the environment |
| Drone has a mate | Fly toward the mate at `step_size` speed (0.1 m/s) |
| Drone has no mate (it's the best) | Hold position |

**Collision avoidance** is applied on top: if two drones get closer than 0.35 m, a repulsive force pushes them apart to prevent collisions.

**Algorithm source file:** `src/bmo_control/bmo_control/bmo_core.py`

---

## Data Flow

Here is the complete path of data through the system in one iteration cycle:

```
Step 1: PHYSICS (Gazebo, 1000 Hz)
   │  Simulates motor forces, gravity, drag → updates drone positions
   │
   ▼
Step 2: SENSING (Light Intensity Plugin, 10 Hz)
   │  Reads each drone's position from Gazebo
   │  Computes lux = lamp_power × efficacy / (4π × distance²) + noise
   │  Publishes: /model/cf{N}/light_intensity
   │
   ▼
Step 3: BRIDGING (ros_gz_bridge)
   │  Translates Gazebo topics → ROS 2 topics
   │  /model/cf{N}/light_intensity  →  /cf{N}/light_intensity
   │  /model/cf{N}/odometry         →  /cf{N}/odom
   │
   ▼
Step 4: DECISION (BMO Coordinator, 10 Hz)
   │  Reads latest sensor data + positions
   │  Runs BMO 4-phase algorithm
   │  Computes velocity command for each drone
   │  Publishes: /cf{N}/cmd_vel
   │
   ▼
Step 5: BRIDGING (ros_gz_bridge)
   │  Translates ROS 2 topics → Gazebo topics
   │  /cf{N}/cmd_vel  →  /cf{N}/gazebo/command/twist
   │
   ▼
Step 6: ACTUATION (Gazebo Velocity Controller)
      Converts velocity command → individual motor thrusts
      Applied at next physics step → back to Step 1
```

---

## World Setup

The simulation world (`single_source_world.sdf`) contains:

```
         y
         ▲
    +1   │  cf2              cf3
         │  (-1, +1)         (+1, +1)
         │
    0  ──┼──────────────────────── lamp_0
         │                         (3.0, 0.0, 0.3)
         │
   -1    │  cf0              cf1
         │  (-1, -1)         (+1, -1)
         │
         └──────────────────────►  x
        -1    0    +1   +2   +3
```

| Entity | Position | Role |
|--------|----------|------|
| cf0 | (-1.0, -1.0, 0.0) | Drone — bottom-left corner |
| cf1 | (+1.0, -1.0, 0.0) | Drone — bottom-right corner |
| cf2 | (-1.0, +1.0, 0.0) | Drone — top-left corner |
| cf3 | (+1.0, +1.0, 0.0) | Drone — top-right corner |
| lamp_0 | (+3.0, 0.0, 0.3) | Tungsten lamp — 60W signal source |

**Expected outcome:** All four drones converge near (3.0, 0.0) within 30–60 seconds.

---

## Configuration Reference

### Algorithm Parameters (`src/bmo_control/config/bmo_params.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `n_agents` | 4 | Number of drones in the swarm |
| `b1` | 0.0 | UV memory weight (0 = no memory, fully reactive) |
| `b2` | 3.0 | Current fitness weight (amplifies sensor reading) |
| `step_size` | 0.1 | Drone movement speed in meters per second |
| `iteration_rate` | 10.0 | Algorithm execution frequency in Hz |
| `hover_height` | 0.5 | Target flight altitude in meters |
| `drone_prefix` | "cf" | Namespace prefix for drone topics |

### Sensor Plugin Parameters (configured in SDF world file)

| Parameter | Default | Description |
|-----------|---------|-------------|
| Lamp power | 60 W | Tungsten lamp wattage |
| Luminous efficacy | 15 lm/W | Light output efficiency (tungsten at ~2700K) |
| Noise std dev | 0.5 lux | Gaussian noise on sensor readings |
| Max reading | 10,000 lux | Sensor saturation limit |
| Update rate | 10 Hz | Sensor publish frequency |

### Safety Parameters (hardcoded in algorithm)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `SAFE_DISTANCE` | 0.35 m | Minimum distance before collision avoidance activates |
| `REPULSION_GAIN` | 0.3 | Strength of collision avoidance force |
| `EPSILON` | 0.01 m | Minimum distance to prevent division by zero |

---

## Getting Started

### Prerequisites

- **ROS 2 Humble** (Ubuntu 22.04)
- **Gazebo Harmonic** (gz-sim8)
- **ros_gz_bridge** (for ROS ↔ Gazebo communication)
- **Bitcraze Crazyflie simulation models** (in `~/cf_ws/simulation_ws/crazyflie-simulation/`)

### Build

```bash
cd ~/bmo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Set Environment Variables

```bash
export GZ_SIM_RESOURCE_PATH="\
$HOME/bmo_ws/install/bmo_sim/share/bmo_sim/models:\
$HOME/cf_ws/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"

export GZ_SIM_SYSTEM_PLUGIN_PATH="\
$HOME/bmo_ws/install/bmo_gz_plugins/lib"
```

### Launch

Open **two terminals**:

**Terminal 1 — Start the simulation:**
```bash
source ~/bmo_ws/install/setup.bash
ros2 launch bmo_sim bmo_simulation.launch.py
```

**Terminal 2 — Start the algorithm:**
```bash
source ~/bmo_ws/install/setup.bash
ros2 launch bmo_control bmo_control.launch.py log_level:=debug
```

### Verify Convergence

Watch the drones in the Gazebo GUI. Within 30–60 seconds, all four should cluster near the lamp at position (3.0, 0.0).

You can also monitor topics live:
```bash
# Check light intensity readings
ros2 topic echo /cf0/light_intensity

# Check drone positions
ros2 topic echo /cf0/odom

# Check velocity commands
ros2 topic echo /cf0/cmd_vel
```

---

## Running Tests

Unit tests for the BMO algorithm run without any simulator or ROS infrastructure:

```bash
cd ~/bmo_ws/src/bmo_control
python3 -m pytest test/test_bmo_core.py -v
```

Tests cover all four algorithm phases, edge cases (zero UV, co-located drones, no valid mate), and collision avoidance behavior.

---

## Project Structure

```
bmo_ws/
├── src/
│   ├── bmo_control/                     # DECISION LAYER
│   │   ├── bmo_control/
│   │   │   ├── bmo_core.py              # Pure BMO algorithm (no ROS)
│   │   │   └── bmo_coordinator_node.py  # ROS 2 coordinator node
│   │   ├── config/
│   │   │   └── bmo_params.yaml          # Algorithm parameters
│   │   ├── launch/
│   │   │   └── bmo_control.launch.py    # Launches the coordinator
│   │   └── test/
│   │       └── test_bmo_core.py         # Unit tests (pytest)
│   │
│   ├── bmo_sim/                         # SIMULATION LAYER
│   │   ├── worlds/
│   │   │   └── single_source_world.sdf  # Gazebo world definition
│   │   ├── models/
│   │   │   ├── crazyflie_bmo/           # Modified Crazyflie model
│   │   │   ├── cf0/ cf1/ cf2/ cf3/      # Per-drone namespace variants
│   │   │   └── tungsten_lamp/           # Visual lamp model
│   │   ├── config/
│   │   │   └── bmo_bridge.yaml          # ROS ↔ Gazebo bridge config
│   │   ├── launch/
│   │   │   └── bmo_simulation.launch.py # Launches Gazebo + bridge
│   │   └── rviz/
│   │       └── bmo_swarm.rviz           # RViz visualization config
│   │
│   └── bmo_gz_plugins/                  # SENSOR LAYER
│       ├── src/
│       │   └── light_intensity_sensor.cpp  # Gazebo world plugin
│       └── include/bmo_gz_plugins/
│           └── light_intensity_sensor.hpp  # Plugin header
│
├── CLAUDE.md                            # Detailed architecture document
├── VALIDATION.md                        # End-to-end validation checklist
└── README.md                            # This file
```

---

## License

This project is developed for academic research purposes.
