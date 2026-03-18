# BMO Workspace Validation Checklist

Run these checks in order after any code change.

---

## 1. Build Validation

```bash
cd ~/bmo_ws
source /opt/ros/humble/setup.bash
colcon build
```

Expected: `3 packages finished` with 0 failures.

---

## 2. Plugin Library Check

```bash
ls -la ~/bmo_ws/install/bmo_gz_plugins/lib/libLightIntensitySensor.so
```

Expected: file exists, size > 0.

---

## 3. Installed Files Check

```bash
# World file
ls ~/bmo_ws/install/bmo_sim/share/bmo_sim/worlds/single_source_world.sdf

# Models
ls ~/bmo_ws/install/bmo_sim/share/bmo_sim/models/crazyflie_bmo/model.sdf
ls ~/bmo_ws/install/bmo_sim/share/bmo_sim/models/crazyflie_bmo/meshes/cf2_assembly.dae
ls ~/bmo_ws/install/bmo_sim/share/bmo_sim/models/tungsten_lamp/model.sdf

# Bridge config
ls ~/bmo_ws/install/bmo_sim/share/bmo_sim/config/bmo_bridge.yaml

# Launch files
ls ~/bmo_ws/install/bmo_sim/share/bmo_sim/launch/bmo_simulation.launch.py
ls ~/bmo_ws/install/bmo_control/share/bmo_control/launch/bmo_control.launch.py

# Params
ls ~/bmo_ws/install/bmo_control/share/bmo_control/config/bmo_params.yaml
```

Expected: all files exist.

---

## 4. BMO Algorithm Unit Tests (no ROS needed)

```bash
cd ~/bmo_ws/src/bmo_control
python3 -m pytest test/test_bmo_core.py -v
```

Expected: 11 passed, 0 failed.

---

## 5. ROS Node Import Check (no simulation needed)

```bash
source ~/bmo_ws/install/setup.bash
python3 -c "from bmo_control.bmo_core import BMOAlgorithm; print('bmo_core OK')"
python3 -c "from bmo_control.bmo_coordinator_node import BMOCoordinator; print('coordinator OK')"
```

Expected: both print OK.

---

## 6. Gazebo World Load Test (visual only, no ROS)

```bash
export GZ_SIM_RESOURCE_PATH="$HOME/bmo_ws/install/bmo_sim/share/bmo_sim/models:$HOME/cf_ws/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/bmo_ws/install/bmo_gz_plugins/lib"
gz sim ~/bmo_ws/install/bmo_sim/share/bmo_sim/worlds/single_source_world.sdf -r
```

Expected:
- Gazebo opens with 4 Crazyflies and 1 glowing lamp
- Console shows: `[LightIntensitySensor] Tracking 4 drones: cf0 cf1 cf2 cf3`
- Console shows: `[LightIntensitySensor] Lamp 0: pos=(3, 0, 0.3), power=60W`
- Console shows: `[LightIntensitySensor] Resolved entity for drone 'cf0'` (x4)

Check Gazebo topics:
```bash
# In another terminal
gz topic -l | grep light_intensity
```
Expected: `/model/cf0/light_intensity`, `/model/cf1/light_intensity`, etc.

---

## 7. Full Simulation Launch Test

### Terminal 1: Simulation
```bash
source ~/bmo_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="$HOME/bmo_ws/install/bmo_sim/share/bmo_sim/models:$HOME/cf_ws/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
ros2 launch bmo_sim bmo_simulation.launch.py
```

### Terminal 2: Verify topics
```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep cf0
```

Expected topics:
```
/cf0/cmd_vel
/cf0/light_intensity
/cf0/odom
```

Check sensor data is flowing:
```bash
ros2 topic echo /cf0/light_intensity --once
ros2 topic echo /cf0/odom --once
```

### Terminal 3: BMO coordinator
```bash
source ~/bmo_ws/install/setup.bash
ros2 launch bmo_control bmo_control.launch.py log_level:=debug
```

Expected:
- `BMO Coordinator started: 4 agents, b1=0.0, b2=3.0, step_size=0.1, rate=10.0Hz`
- `All drone data received — starting takeoff`
- `All drones at hover height (0.5m) — starting BMO`
- Debug log shows positions, UV values, l-mate selections

### Convergence Check

After ~30-60 seconds, all drones should cluster near lamp at (3, 0):
```bash
ros2 topic echo /cf0/odom --once | grep -A2 position
ros2 topic echo /cf1/odom --once | grep -A2 position
ros2 topic echo /cf2/odom --once | grep -A2 position
ros2 topic echo /cf3/odom --once | grep -A2 position
```

Expected: all x values approaching 3.0, all y values approaching 0.0.

---

## 8. Quick Smoke Test (single command)

```bash
cd ~/bmo_ws && source /opt/ros/humble/setup.bash && \
colcon build && \
source install/setup.bash && \
cd src/bmo_control && python3 -m pytest test/test_bmo_core.py -v && \
python3 -c "from bmo_control.bmo_core import BMOAlgorithm; print('ALL CHECKS PASS')"
```
