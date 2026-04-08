"""Backend that bridges Crazyswarm2 to Gazebo Harmonic via cmd_vel / odom topics."""

from __future__ import annotations

from functools import partial

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool

from ..sim_data_types import Action, State


class Backend:
    """Gazebo backend: sends velocity commands, reads odometry."""

    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.t = 0
        self.dt = 0.01  # 100 Hz control loop

        # Per-drone publishers and subscribers
        self.cmd_pubs = {}
        self.enable_pubs = {}
        self.odom_states = {}

        for i, name in enumerate(names):
            # Publisher for Gazebo velocity commands
            self.cmd_pubs[name] = node.create_publisher(
                Twist, f'/{name}/cmd_vel', 10)

            # Publisher to enable the Gazebo MulticopterVelocityControl plugin
            self.enable_pubs[name] = node.create_publisher(
                Bool, f'/{name}/enable', 10)

            # Store initial state from config
            self.odom_states[name] = State(
                pos=np.array(states[i].pos, dtype=float))

            # Subscribe to Gazebo odometry
            node.create_subscription(
                Odometry,
                f'/{name}/odom',
                partial(self._odom_cb, name=name),
                10)

        # Track which drones have received odom (confirming Gazebo is ready)
        self._odom_received = {name: False for name in names}

        # PD gains for converting desired state to velocity
        self.kp_pos = np.array([0.8, 0.8, 1.0])   # Position P gain (softer XY, slightly higher Z)
        self.kd_vel = np.array([0.5, 0.5, 0.5])   # Velocity damping (D term via velocity error)
        self.kff_vel = np.array([0.3, 0.3, 0.3])  # Feedforward on desired velocity
        self.max_vel = 1.0                          # Max velocity clamp (m/s)

        # Periodically enable all velocity controllers (ensures late-starting ones work)
        self._enable_timer = node.create_timer(0.5, self._enable_all)

        node.get_logger().info(
            f'[Gazebo backend] Controlling {len(names)} drones: {names}')

    def time(self) -> float:
        return self.t

    def step(self, states_desired: list[State], actions: list[Action]) -> list[State]:
        self.t += self.dt

        next_states = []
        for i, name in enumerate(self.names):
            desired = states_desired[i]
            current = self.odom_states[name]

            # Only send commands once Gazebo confirms the drone is ready (odom received)
            if self._odom_received[name]:
                # PD controller: position error + velocity damping -> velocity command
                pos_err = desired.pos - current.pos
                vel_err = desired.vel - current.vel
                vel_cmd = (self.kp_pos * pos_err
                           + self.kd_vel * vel_err
                           + self.kff_vel * desired.vel)

                # Clamp velocity to prevent flips from large position errors
                speed = np.linalg.norm(vel_cmd)
                if speed > self.max_vel:
                    vel_cmd = vel_cmd * (self.max_vel / speed)

                msg = Twist()
                msg.linear.x = float(vel_cmd[0])
                msg.linear.y = float(vel_cmd[1])
                msg.linear.z = float(vel_cmd[2])
                self.cmd_pubs[name].publish(msg)

            # Return actual state from Gazebo odometry
            next_states.append(State(
                pos=np.array(current.pos),
                vel=np.array(current.vel),
                quat=np.array(current.quat),
                omega=np.array(current.omega),
            ))

        # Use Gazebo's clock (via /clock bridge) instead of our own
        self.t = self.node.get_clock().now().nanoseconds / 1e9

        return next_states

    def _enable_all(self):
        """Publish enable=True to all drones' velocity controllers."""
        msg = Bool()
        msg.data = True
        for name in self.names:
            self.enable_pubs[name].publish(msg)

    def shutdown(self):
        # Stop all drones
        for name in self.names:
            self.cmd_pubs[name].publish(Twist())

    def _odom_cb(self, msg: Odometry, name: str):
        """Update stored state from Gazebo odometry."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular

        if not self._odom_received.get(name, False):
            self._odom_received[name] = True
            self.node.get_logger().info(f'[Gazebo backend] {name} odom ready')

        self.odom_states[name] = State(
            pos=np.array([p.x, p.y, p.z]),
            vel=np.array([v.x, v.y, v.z]),
            quat=np.array([q.w, q.x, q.y, q.z]),
            omega=np.array([w.x, w.y, w.z]),
        )
