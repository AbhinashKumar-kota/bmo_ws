# BMO Swarm — Crazyflie Implementation

## Project Overview

Implementation of the **Butterfly Mating Optimization (BMO)** algorithm on a swarm of
Crazyflie 2.1+ nano-quadrotors. The drones locate multiple signal sources (tungsten lamps)
autonomously using decentralized swarm intelligence. Simulation in Gazebo Harmonic validates
the algorithm before real-world deployment.

This project is based on:
- "Butterfly Mating Optimization" (BMO paper) — swarm algorithm
- "Bflybots: A Novel Robotic Swarm" — ground robot implementation
- "Bfly Rotor: Aerial Robotic Platform" — aerial adaptation

Reference documents: `~/guides/Butterfly_Mating_Optimization.pdf`, `~/guides/BMO.odt`,
`~/UW_robot/BflybotsANovelRoboticSwarminPursuitofDynamicSignalSources.pdf`

---

## Architecture

### Workspace Layout

```
~/bmo_ws/src/
├── bmo_gz_plugins/     # C++ Gazebo Harmonic system plugin (light intensity sensor)
├── bmo_sim/            # Gazebo worlds, signal source models, launch files, bridge config
└── bmo_control/        # Python — BMO algorithm coordinator node
```

### Simulation World

**single_source_world.sdf** — 4 Crazyflies (cf0, cf1, cf2, cf3) + 1 tungsten lamp.
Validates that all 4 drones converge to the single signal source using BMO.

### Current Scope (strict)

**One world. 4 drones. 1 lamp. BMO algorithm that works.**

- Single source world ONLY — no multi-source world until single source is validated end-to-end
- 4 Crazyflies (cf0, cf1, cf2, cf3)
- 1 tungsten lamp
- Success criteria: all 4 drones converge to the lamp
- Priority: algorithm correctness over everything else

Do NOT build multi-source world, decentralized mode, or extra features until this works.

### Design Strategy

Centralized implementation is used to validate algorithm correctness before transitioning
to decentralized execution. This is deliberate:

| Stage         | Purpose                                         |
|---------------|------------------------------------------------|
| Centralized   | Debug algorithm correctness in simulation       |
| Hybrid        | Add communication constraints, test robustness  |
| Decentralized | Real swarm behavior on physical Crazyflies      |

All drones publish state → coordinator aggregates → computes BMO → sends velocity commands.
No direct drone-to-drone communication in current implementation.

### System Diagram

```
                        Gazebo Harmonic
┌──────────────────────────────────────────────────────────┐
│                                                          │
│  World: single_source_world.sdf                          │
│                                                          │
│  tungsten_lamp_0 (x=3.0, y=0.0, P=60W)                  │
│                                                          │
│  cf0 ─┬─ MulticopterVelocityControl (Bitcraze)           │
│       ├─ OdometryPublisher (Bitcraze)                    │
│       └─ LightIntensitySensor (our world plugin)         │
│  cf1 ─┬─ (same stack)                                    │
│  cf2 ─┬─ (same stack)                                    │
│  cf3 ─┬─ (same stack)                                    │
│                                                          │
│  NOTE: We use a modified crazyflie model (crazyflie_bmo) │
│  with <robotNamespace> REMOVED from all plugins so that  │
│  each <include> with a unique <name> (cf0, cf1, cf2)     │
│  gets its own namespaced topics automatically.            │
│                                                          │
└───────────────┬──────────────────────────────────────────┘
                │ ros_gz_bridge (one per drone)
                │
     ┌──────────▼──────────────────────┐
     │          ROS 2 Humble           │
     │                                 │
     │  /cf0/odom ─────────────────┐   │
     │  /cf0/light_intensity ──────┤   │
     │  /cf1/odom ─────────────────┤   │
     │  /cf1/light_intensity ──────┤   │
     │  /cf2/odom ─────────────────┼──→ bmo_coordinator_node
     │  /cf2/light_intensity ──────┤        │
     │  /cf3/odom ─────────────────┤        │ BMO 4-phase algorithm
     │  /cf3/light_intensity ──────┘        ▼
     │                                 velocity commands
     │  /cf0/cmd_vel  ◄──────────────
     │  /cf1/cmd_vel  ◄──────────────
     │  /cf2/cmd_vel  ◄──────────────
     │  /cf3/cmd_vel  ◄──────────────
     └─────────────────────────────────┘
```

### Multi-Drone Namespacing (Important)

The original Bitcraze `model.sdf` hardcodes `<robotNamespace>crazyflie</robotNamespace>`
in all motor and velocity control plugins. This means multiple `<include>` instances would
ALL listen on the same Gazebo topics — broken for multi-drone.

**Our fix**: We maintain a modified model `crazyflie_bmo` in `bmo_sim/models/` that is
identical to Bitcraze's model except `<robotNamespace>` is removed. When gz-sim loads
a model without explicit robotNamespace, it uses the model's scoped name (set by `<name>`
in `<include>`) as the namespace. So:

```xml
<include>
  <uri>model://crazyflie_bmo</uri>
  <name>cf0</name>           <!-- topics become /cf0/gazebo/command/twist etc. -->
  <pose>-1 -1 0.0 0 0 0</pose>
</include>
```

This model references Bitcraze's original mesh files via relative path. Both our models
directory AND the Bitcraze gazebo directory must be on GZ_SIM_RESOURCE_PATH:

```bash
export GZ_SIM_RESOURCE_PATH="$HOME/bmo_ws/install/bmo_sim/share/bmo_sim/models:$HOME/cf_ws/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
```

### Execution Order

```
1. Launch Gazebo world          → spawns drones + loads LightIntensitySensor plugin
2. Start ros_gz_bridge          → bridges Gazebo topics to ROS 2 (per drone)
3. Start control_services       → per-drone takeoff/height-hold state machine
4. Start bmo_coordinator_node   → subscribes to sensors, runs BMO, publishes cmd_vel
```

Steps 1-3 are in the bmo_sim launch file. Step 4 is in the bmo_control launch file.

### Data Flow (per iteration at 10 Hz)

```
1. Gazebo: LightIntensitySensor plugin reads drone pose, computes intensity from all lamps
2. Bridge: /cfN/light_intensity (Float64) and /cfN/odom (Odometry) flow to ROS 2
3. bmo_coordinator_node:
   a. UV Update:       UV_i(t) = max(0, b1 * UV_i(t-1) + b2 * intensity_i)
   b. UV Distribution: UV_{i->j} = UV_i * d_ij^{-1} / sum(d_k^{-1})
   c. l-mate Selection: pick j where UV_j > UV_i (descending UV order)
   d. Movement:        v_i = B_s * (x_lmate - x_i) / ||x_lmate - x_i||
4. Coordinator publishes /cfN/cmd_vel (Twist)
5. Bridge forwards to Gazebo MulticopterVelocityControl
```

### Failure Handling

```
If a drone's odometry or intensity is unavailable:
- Skip UV update for that agent in current iteration
- Maintain previous UV value
- Log a warning (not an error — transient gaps are expected at startup)

If all data is stale for >2 seconds:
- Coordinator publishes zero velocity (hover in place)
- Logs error
```

### Assumptions

```
- All agent states are updated synchronously at fixed rate (10 Hz)
- No communication delay is modeled (to be relaxed in future decentralized work)
- Drones fly at fixed altitude (2D problem — z is held by control_services)
- Signal sources are static (no moving lamps in current implementation)
```

---

## Tungsten Lamp Model (Signal Source)

The real hardware uses tungsten filament lamps as signal sources, sensed by LDR
(Light Dependent Resistor) sensors on the robots. The Gazebo plugin replicates this physics.

### Emission Model

Tungsten lamps are modeled as **isotropic point sources** (omnidirectional).

Luminous intensity from a single lamp at distance r:

```
I(r) = (P * eta) / (4 * pi * r^2)
```

Where:
- P     = electrical power (watts), e.g. 40W, 60W, 100W
- eta   = luminous efficacy of tungsten at ~2700K = 15 lm/W (typical)
- r     = Euclidean distance from lamp to drone (meters)
- I(r)  = illuminance at drone position (lux)

For multiple lamps, total illuminance is the superposition:

```
I_total = sum_k [ (P_k * eta) / (4 * pi * ||p_lamp_k - p_drone||^2) ]
```

### Sensor Model (LDR on drone)

The LDR produces a reading proportional to illuminance. We model:

```
reading = I_total + N(0, sigma^2)
```

Where:
- sigma  = sensor noise standard deviation (configurable, default 0.5 lux)
- Reading is clamped to [0, max_reading] (default max_reading = 10000 lux)

### Why Tungsten Specifically

- Color temperature ~2700K (warm white, peak emission ~1050nm IR)
- LDRs (CdS cells) have peak sensitivity at ~550nm (green) but respond to broadband
- The luminous efficacy of 15 lm/W accounts for the spectral mismatch between
  tungsten emission and LDR sensitivity
- In hardware, lamp wattage directly controls peak height — same in simulation

### Initial Drone Positions (defined in world SDF)

```
cf0: (-1.0, -1.0, 0.0)    # bottom-left
cf1: ( 1.0, -1.0, 0.0)    # bottom-right
cf2: (-1.0,  1.0, 0.0)    # top-left
cf3: ( 1.0,  1.0, 0.0)    # top-right
lamp_0: (3.0,  0.0, 0.3)  # single source, offset from all drones, P=60W
```

Lamp height 0.3m = slightly below drone hover height (0.5m), matching real
tabletop tungsten lamp placement.

### Configurable Parameters (in world SDF per lamp)

```xml
<plugin filename="LightIntensitySensor" name="bmo_gz_plugins::LightIntensitySensor">
  <source_positions>2.0 3.0 0.3, -1.0 4.0 0.3, 3.0 -2.0 0.3</source_positions>
  <source_powers>60.0, 40.0, 100.0</source_powers>
  <luminous_efficacy>15.0</luminous_efficacy>
  <noise_stddev>0.5</noise_stddev>
  <max_reading>10000.0</max_reading>
  <update_rate>10.0</update_rate>
</plugin>
```

---

## BMO Algorithm Parameters

From the original paper, tuned for 3-source scenario:

```yaml
# bmo_params.yaml
bmo:
  n_agents: 4                 # number of Crazyflies
  b1: 0.0                     # previous UV weight (0 = no memory)
  b2: 3.0                     # current fitness weight
  step_size: 0.1              # B_s in m/s (scaled for drone velocity)
  uv_initial: 0.0             # starting UV for all agents
  iteration_rate: 10.0        # Hz — BMO iteration frequency
  hover_height: 0.5           # meters — fixed flight altitude
```

### BMO Equations Reference

```
Phase 1 — UV Update:
  UV_i(t) = max(0, b1 * UV_i(t-1) + b2 * f(i))
  where f(i) = light_intensity reading at drone i's position

Phase 2 — UV Distribution:
  UV_{i->j} = UV_i * (1/d_ij) / sum_k(1/d_ik)  where k ∈ {all agents} and k != i
  where d_ij = ||pos_i - pos_j|| (Euclidean distance)
  NOTE: sum is strictly over k != i (never include self)

Phase 3 — l-mate Selection:
  Sort other agents by UV in descending order
  Select first agent j where UV_j > UV_i
  If none found, agent stays in place (it is at a peak)

Phase 4 — Movement:
  direction = (pos_lmate - pos_i) / ||pos_lmate - pos_i||
  velocity  = B_s * direction
  Publish as Twist: linear.x = vx, linear.y = vy, linear.z = 0
```

### Edge Cases (must be enforced in code)

```
- d_ij = 0:        Clamp to epsilon = 0.01m (avoid division by zero in UV distribution)
- d_ij < epsilon:  Clamp to epsilon = 0.01m (agents are nearly co-located)
- No valid l-mate: Agent holds position (zero velocity) — it is at a local peak
- ||pos_lmate - pos_i|| < epsilon: velocity = 0 (already at target, no movement needed)
- UV_i = 0 for all: All agents do random walk until fitness is nonzero

Random walk (when UV = 0 for all agents):
  Sample random direction theta ∈ [0, 2*pi]
  velocity = step_size * [cos(theta), sin(theta), 0]
  Each agent gets an independent random direction per iteration
```

---

## Crazyflie Model

### Original (Bitcraze)

The original Crazyflie Gazebo model lives in:
```
~/cf_ws/simulation_ws/crazyflie-simulation/simulator_files/gazebo/crazyflie/
```

Meshes live in:
```
~/cf_ws/simulation_ws/crazyflie-simulation/meshes/collada_files/
  cf2_assembly.dae, ccw_prop.dae, cw_prop.dae
```

### Our Modified Model (crazyflie_bmo)

Lives in `bmo_sim/models/crazyflie_bmo/`. Changes from original:
1. **Removed** `<robotNamespace>crazyflie</robotNamespace>` from all plugins
   (MulticopterMotorModel x4, MulticopterVelocityControl) so each instance gets
   unique namespace from its `<name>` tag
2. Mesh URIs updated to work with our GZ_SIM_RESOURCE_PATH setup

Key properties (unchanged from Bitcraze):
- Mass: 0.025 kg (25g)
- Inertia: Ixx=16.57e-6, Iyy=16.66e-6, Izz=29.26e-6
- 4 motors with force constant 1.28192e-8, moment constant 0.005964552
- Velocity control gains: [1.25, 1.25, 0.2425] (linear), [0.02, 0.02, 0.02] (attitude)
- Control input: Twist message on `/{model_name}/gazebo/command/twist`
- Output: Odometry on `/model/{model_name}/odometry`, Pose on `/model/{model_name}/pose`

---

## Package Details

### bmo_gz_plugins (C++)

Gazebo Harmonic system plugin. Compiled against gz-sim8 headers.

```
bmo_gz_plugins/
├── CMakeLists.txt          # find gz-sim8, gz-msgs10, gz-transport13
├── package.xml             # build_depend on gz-sim8-vendor or gz-harmonic
├── include/
│   └── light_intensity_sensor.hpp
└── src/
    └── light_intensity_sensor.cpp
```

The plugin:
- Implements gz::sim::System, ISystemConfigure, ISystemPostUpdate
- On Configure: reads source positions, powers, efficacy, noise params from SDF
- On PostUpdate: for each entity with this plugin attached:
  1. Get entity world pose from gz::sim::WorldPose component
  2. Compute I_total from all lamp positions (inverse-square + noise)
  3. Publish on gz topic: /model/{model_name}/light_intensity (gz.msgs.Double)

### bmo_sim (CMake + data files)

```
bmo_sim/
├── CMakeLists.txt
├── package.xml
├── worlds/
│   └── single_source_world.sdf  # 4 CFs + 1 tungsten lamp
├── models/
│   ├── crazyflie_bmo/            # Modified Bitcraze model (no hardcoded namespace)
│   │   ├── model.config
│   │   └── model.sdf
│   └── tungsten_lamp/            # Visual marker: small cylinder + orange point light
│       ├── model.config
│       └── model.sdf
├── config/
│   └── bmo_bridge.yaml           # Per-drone topic bridges (same for both worlds)
├── rviz/
│   └── bmo_swarm.rviz
└── launch/
    └── bmo_simulation.launch.py  # arg: world:=single_source (default) or multi_source
```

World SDF spawns:
- 4 Crazyflies via `<include uri="model://crazyflie_bmo">` with unique `<name>` and `<pose>`
- 1 tungsten_lamp model as visual marker (static)
- LightIntensitySensor as a **world-level plugin** (one instance reads all drone poses)

Bridge config (per drone):
```yaml
# For cf0 (repeated for cf1, cf2 with namespace swap)
- ros_topic_name: "/cf0/cmd_vel"
  gz_topic_name: "/cf0/gazebo/command/twist"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "ignition.msgs.Twist"
  direction: ROS_TO_GZ
- ros_topic_name: "/cf0/odom"
  gz_topic_name: "/model/cf0/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
- ros_topic_name: "/cf0/light_intensity"
  gz_topic_name: "/model/cf0/light_intensity"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: GZ_TO_ROS
```

### bmo_control (Python)

```
bmo_control/
├── package.xml
├── setup.py
├── setup.cfg
├── bmo_control/
│   ├── __init__.py
│   ├── bmo_coordinator_node.py   # ROS node: subscribes sensors, runs BMO, publishes cmd_vel
│   ├── bmo_core.py               # Pure algorithm (no ROS imports): UV, l-mate, movement
│   └── control_services.py       # Per-drone takeoff/land/height-hold (adapted from Bitcraze)
├── config/
│   └── bmo_params.yaml
├── launch/
│   └── bmo_control.launch.py
└── test/
    └── test_bmo_core.py          # Unit tests for algorithm (pytest, no ROS needed)
```

bmo_core.py is **ROS-free** — pure Python + numpy. This means:
- Teammates can unit test the algorithm without ROS installed
- Algorithm can be ported to other platforms later
- Clear separation: bmo_core = math, bmo_coordinator_node = ROS wiring

---

## Build & Run

### Prerequisites

```bash
# Gazebo Harmonic + ROS bridge (already installed in system)
# Crazyflie model path (already in ~/cf_ws/simulation_ws/)
```

### Build

```bash
cd ~/bmo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$HOME/bmo_ws/install/bmo_sim/share/bmo_sim/models:$HOME/cf_ws/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
```

### Run Simulation

```bash
# Terminal 1: Gazebo + bridges + control services
ros2 launch bmo_sim bmo_simulation.launch.py

# Terminal 2: BMO coordinator
ros2 launch bmo_control bmo_control.launch.py
```

### Verify System is Alive

After launching bmo_sim, confirm all topics are active:

```bash
# Check all drone topics are bridged
ros2 topic list | grep cf0
# Expected output:
#   /cf0/cmd_vel
#   /cf0/light_intensity
#   /cf0/odom

# Check intensity is being published
ros2 topic echo /cf0/light_intensity --once

# Check odometry is flowing
ros2 topic echo /cf0/odom --once
```

### Run Tests

```bash
# Algorithm unit tests (no ROS needed)
cd ~/bmo_ws/src/bmo_control
python3 -m pytest test/

# Full colcon test
cd ~/bmo_ws
colcon test
colcon test-result --verbose
```

---

## Expected Behavior (Correctness Criteria)

```
1. All 4 drones take off and hold at hover_height (0.5m)
2. Drones sense light intensity — closer to lamp = higher reading
3. UV values increase as drones approach the source
4. All 4 drones converge toward the single lamp
5. Drones cluster near the lamp position
6. UV values stabilize at high values
7. Velocities approach zero

Convergence criteria (formal):
- distance_to_lamp < 0.3m for all 4 agents
- velocity magnitude < 0.02 m/s for all 4 agents
```

### Failure Indicators (something is wrong if...)
```
- Drones oscillate without converging → step_size too large or b2 too high
- Drones fly away from lamp → sensor reading inverted or movement sign error
- Drones don't move at all → cmd_vel not bridged or control_services not running
- UV values are always 0 → plugin not loaded or bridge not configured
```

---

## Coding Conventions

### General
- **Python**: snake_case for files, functions, variables. CamelCase for classes.
- **C++**: snake_case for files and variables. CamelCase for classes. UPPER_SNAKE for constants.
- Every file has a module-level docstring (Python) or file-level comment (C++) explaining
  what it does and how it fits in the system.
- Every function/method has a docstring explaining: purpose, parameters, return value.
- Comments explain **why**, not what. The code should be readable on its own.
- Use inline comments for non-obvious physics equations — reference the paper equation number.

### ROS 2 Specific
- Node names: descriptive, snake_case (e.g. `bmo_coordinator`, `control_services_cf0`)
- Topics: `/{drone_namespace}/{topic_name}` (e.g. `/cf0/light_intensity`)
- Parameters: loaded from YAML, declared in __init__ with defaults
- Timers: prefer create_timer over spin_once loops
- Logging: use self.get_logger().info/warn/error, not print()
- Debug logging: coordinator MUST log per iteration (at DEBUG level):
    agent positions, UV values, selected l-mate IDs, velocity commands
  Enable with: `ros2 launch bmo_control bmo_control.launch.py log_level:=debug`

### Gazebo Plugin (C++)
- Follow gz-sim coding style
- Use sdf::Element to parse plugin parameters
- Use gz::sim::EntityComponentManager for pose queries
- Use gz::transport::Node for publishing gz topics

### Comments Style (for junior team members)

```python
# --------------------------------------------------------------------------
# UV UPDATE PHASE (BMO Paper, Equation 1)
# Each drone updates its UV value based on current sensor reading.
# b1 controls how much "memory" of past UV is retained (0 = none).
# b2 scales the current fitness (light intensity) — higher b2 means
# the drone reacts more strongly to what it currently senses.
# --------------------------------------------------------------------------
def uv_update(self, agents, b1, b2):
```

```cpp
/**
 * Compute illuminance at a point from a tungsten lamp.
 *
 * Uses the inverse-square law for isotropic point sources:
 *   I = (P * eta) / (4 * pi * r^2)
 *
 * @param lamp_pos   World position of the tungsten lamp (meters)
 * @param drone_pos  World position of the drone (meters)
 * @param power_w    Electrical power of the lamp (watts)
 * @param efficacy   Luminous efficacy (lumens/watt), 15.0 for tungsten at 2700K
 * @return           Illuminance at drone position (lux)
 */
double ComputeIlluminance(/* ... */);
```

---

## Git Workflow (Team)

```
main            <- protected, merge via PR only
├── develop     <- integration branch, PRs merge here first
│   ├── feature/gz-light-sensor-plugin
│   ├── feature/bmo-core-algorithm
│   ├── feature/multi-drone-world
│   ├── fix/uv-distribution-edge-case
│   └── experiment/5-drone-benchmark
```

- Branch from `develop`, PR back to `develop`
- `develop` -> `main` only when simulation runs end-to-end
- PR requires: builds clean, tests pass, 1 review
- Commit messages: `type: short description` (e.g. `feat: add UV distribution phase`)
  Types: feat, fix, refactor, test, docs, chore

---

## Milestones

1. **M1**: bmo_gz_plugins builds, plugin loads in Gazebo, publishes intensity for 1 drone
2. **M2**: bmo_sim launches 4 drones + 1 lamp, all topics bridged to ROS
3. **M3**: bmo_control coordinator runs BMO, drones move toward the lamp
4. **M4**: All 4 drones converge to the lamp — algorithm correctness validated
