"""
BMO Coordinator Node — Centralized BMO controller for 4 Crazyflie drones

This ROS 2 node:
  1. Subscribes to /cfN/odom and /cfN/light_intensity for each drone
  2. Runs the BMO algorithm (bmo_core.py) at a fixed rate (10 Hz)
  3. Publishes /cfN/cmd_vel (Twist) velocity commands to drive each drone

The node implements a simple state machine:
  WAITING  → collecting initial data from all drones
  TAKEOFF  → sending upward velocity until hover height reached
  RUNNING  → executing BMO iterations
  STOPPED  → zero velocity (converged or manually stopped)

Parameters (from bmo_params.yaml):
  n_agents:       Number of drones (default: 4)
  b1:             UV memory weight (default: 0.0)
  b2:             UV fitness weight (default: 3.0)
  step_size:      Movement speed in m/s (default: 0.1)
  iteration_rate: BMO iteration frequency in Hz (default: 10.0)
  hover_height:   Target flight altitude in meters (default: 0.5)
  drone_prefix:   Namespace prefix for drones (default: "cf")
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool

from bmo_control.bmo_core import BMOAlgorithm


class BMOCoordinator(Node):

    def __init__(self):
        super().__init__('bmo_coordinator')

        # ----- Declare and read parameters -----
        self.declare_parameter('n_agents', 4)
        self.declare_parameter('b1', 0.0)
        self.declare_parameter('b2', 3.0)
        self.declare_parameter('step_size', 0.1)
        self.declare_parameter('iteration_rate', 10.0)
        self.declare_parameter('hover_height', 0.5)
        self.declare_parameter('drone_prefix', 'cf')

        self.n_agents = self.get_parameter('n_agents').value
        b1 = self.get_parameter('b1').value
        b2 = self.get_parameter('b2').value
        step_size = self.get_parameter('step_size').value
        rate = self.get_parameter('iteration_rate').value
        self.hover_height = self.get_parameter('hover_height').value
        prefix = self.get_parameter('drone_prefix').value

        # ----- Initialize BMO algorithm (pure math, no ROS) -----
        self.bmo = BMOAlgorithm(
            n_agents=self.n_agents,
            b1=b1,
            b2=b2,
            step_size=step_size
        )

        # ----- Per-drone state tracking -----
        # Track whether we've received data from each drone
        self.odom_received = [False] * self.n_agents
        self.intensity_received = [False] * self.n_agents
        self.current_heights = [0.0] * self.n_agents

        # ----- State machine -----
        # WAITING: wait for data from all drones
        # TAKEOFF: fly up to hover_height
        # RUNNING: execute BMO
        self.state = 'WAITING'

        # ----- Per-drone height hold state -----
        # Stores the desired height once hover is reached, used to maintain
        # altitude during BMO (since BMO only controls x,y).
        self.desired_heights = [self.hover_height] * self.n_agents

        # ----- Create subscribers and publishers for each drone -----
        self.cmd_publishers = []
        self.enable_publishers = []
        self.controllers_enabled = False
        for i in range(self.n_agents):
            name = f'{prefix}{i}'

            # Publisher: velocity command
            pub = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            self.cmd_publishers.append(pub)

            # Publisher: enable velocity controller
            enable_pub = self.create_publisher(Bool, f'/{name}/enable', 10)
            self.enable_publishers.append(enable_pub)

            # Subscriber: odometry (position and velocity)
            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, idx=i: self._odom_callback(msg, idx),
                10
            )

            # Subscriber: light intensity (sensor reading)
            self.create_subscription(
                Float64,
                f'/{name}/light_intensity',
                lambda msg, idx=i: self._intensity_callback(msg, idx),
                10
            )

        # ----- Timer: runs BMO at fixed rate -----
        period = 1.0 / rate
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'BMO Coordinator started: {self.n_agents} agents, '
            f'b1={b1}, b2={b2}, step_size={step_size}, rate={rate}Hz'
        )

    def _odom_callback(self, msg: Odometry, agent_id: int):
        """Called when odometry data arrives for a drone.

        Extracts [x, y] position and current height, then updates
        the BMO agent's position.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        self.bmo.update_fitness(
            agent_id=agent_id,
            fitness=self.bmo.agents[agent_id].fitness,  # keep current fitness
            position=[x, y, z]
        )
        self.current_heights[agent_id] = z
        self.odom_received[agent_id] = True

    def _intensity_callback(self, msg: Float64, agent_id: int):
        """Called when light intensity reading arrives for a drone.

        Updates the BMO agent's fitness value.
        """
        agent = self.bmo.agents[agent_id]
        self.bmo.update_fitness(
            agent_id=agent_id,
            fitness=msg.data,
            position=agent.position  # keep current position
        )
        self.intensity_received[agent_id] = True

    def _timer_callback(self):
        """Main loop — runs at iteration_rate Hz.

        Implements the state machine:
          WAITING → TAKEOFF → RUNNING
        """

        if self.state == 'WAITING':
            # Wait until we have data from all drones
            if all(self.odom_received) and all(self.intensity_received):
                self.get_logger().info('All drone data received — enabling controllers')
                # Enable the MulticopterVelocityControl on each drone.
                # Without this, the controller ignores all cmd_vel commands.
                self._enable_controllers()
                self.state = 'TAKEOFF'
            else:
                return

        elif self.state == 'TAKEOFF':
            # Keep publishing enable (in case bridge wasn't ready on first send)
            if not self.controllers_enabled:
                self._enable_controllers()

            # Send upward velocity until all drones reach hover_height
            all_at_height = True
            for i in range(self.n_agents):
                cmd = Twist()
                if self.current_heights[i] < self.hover_height * 0.9:
                    cmd.linear.z = 0.5  # climb rate
                    all_at_height = False
                else:
                    cmd.linear.z = 0.0
                self.cmd_publishers[i].publish(cmd)

            if all_at_height:
                # Record desired heights for height hold during BMO
                for i in range(self.n_agents):
                    self.desired_heights[i] = self.current_heights[i]
                self.get_logger().info(
                    f'All drones at hover height ({self.hover_height}m) — starting BMO'
                )
                self.state = 'RUNNING'

        elif self.state == 'RUNNING':
            # ----- Run one BMO iteration -----
            self.bmo.run_iteration()

            # ----- Publish velocity commands (3D from BMO + height safety) -----
            # BMO now controls x, y, z. We add a minimum height floor so
            # drones never descend below hover_height during BMO.
            for i in range(self.n_agents):
                agent = self.bmo.agents[i]
                cmd = Twist()
                cmd.linear.x = agent.velocity[0]
                cmd.linear.y = agent.velocity[1]
                cmd.linear.z = agent.velocity[2]

                # Safety: if drone is below hover_height, override z with climb
                if self.current_heights[i] < self.hover_height * 0.85:
                    height_error = self.hover_height - self.current_heights[i]
                    cmd.linear.z = max(cmd.linear.z, height_error * 2.0)

                self.cmd_publishers[i].publish(cmd)

            # ----- Debug logging -----
            self.get_logger().debug(self._format_debug_log())

    def _enable_controllers(self):
        """Publish 'true' on each drone's enable topic.

        The MulticopterVelocityControl plugin in Gazebo requires an enable
        signal before it accepts cmd_vel commands. Without this, all
        velocity commands are silently ignored and the drones don't move.
        """
        msg = Bool()
        msg.data = True
        for i in range(self.n_agents):
            self.enable_publishers[i].publish(msg)
        self.controllers_enabled = True
        self.get_logger().info('Velocity controllers enabled for all drones')

    def _format_debug_log(self) -> str:
        """Format a debug log line with all agent states.

        Output format per agent:
          [id] pos=(x,y) uv=V fitness=F lmate=M vel=(vx,vy)

        This is invaluable for debugging convergence issues.
        """
        lines = ['BMO iteration:']
        for a in self.bmo.agents:
            lmate_str = str(a.lmate_id) if a.lmate_id is not None else 'NONE'
            lines.append(
                f'  [{a.agent_id}] pos=({a.position[0]:.2f},{a.position[1]:.2f},{a.position[2]:.2f}) '
                f'uv={a.uv:.2f} fitness={a.fitness:.2f} '
                f'lmate={lmate_str} vel=({a.velocity[0]:.3f},{a.velocity[1]:.3f},{a.velocity[2]:.3f})'
            )
        return '\n'.join(lines)


def main(args=None):
    rclpy.init(args=args)
    node = BMOCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
