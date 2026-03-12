#!/usr/bin/env python3
"""
velocity_integrator.py

Integrates joint velocity commands into position states for use with
topic_based_ros2_control in fake hardware mode.

Subscribes to:  /fake_joint_commands  (sensor_msgs/JointState)
Publishes to:   /fake_joint_states    (sensor_msgs/JointState)

Usage:
  ros2 run <your_pkg> velocity_integrator --ros-args
    -p joint_names:="['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7']"
    -p initial_positions:="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
    -p publish_rate:=100.0
    -p command_topic:=/fake_joint_commands
    -p state_topic:=/fake_joint_states
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState


class VelocityIntegrator(Node):
    def __init__(self):
        super().__init__("velocity_integrator")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter(
            "joint_names", ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
        )
        self.declare_parameter("initial_positions", [0.2, -0.18, 2.16, -1.57, -0.6, -1.34])
        self.declare_parameter("publish_rate", 100.0)  # Hz
        self.declare_parameter("command_topic", "/fake_joint_commands")
        self.declare_parameter("state_topic", "/fake_joint_states")
        self.declare_parameter("position_limits_min", [-6.28, -2.41, -6.28, -2.66, -6.28, -2.23, -6.28])
        self.declare_parameter("position_limits_max", [6.28, 2.41, 6.28, 2.66, 6.28, 2.23, 6.28])

        self.joint_names = self.get_parameter("joint_names").value
        initial_positions = self.get_parameter("initial_positions").value
        publish_rate = self.get_parameter("publish_rate").value
        command_topic = self.get_parameter("command_topic").value
        state_topic = self.get_parameter("state_topic").value
        self.pos_min = np.array(self.get_parameter("position_limits_min").value)
        self.pos_max = np.array(self.get_parameter("position_limits_max").value)

        n = len(self.joint_names)

        # ── State ──────────────────────────────────────────────────────────────
        self.positions = np.array(initial_positions, dtype=float)
        self.velocities = np.zeros(n, dtype=float)
        self.efforts = np.zeros(n, dtype=float)

        # Time tracking for integration
        self.last_cmd_time: Time | None = None
        self.cmd_received = False

        # ── ROS interfaces ─────────────────────────────────────────────────────
        self.cmd_sub = self.create_subscription(JointState, command_topic, self._command_callback, 10)

        self.state_pub = self.create_publisher(JointState, state_topic, 10)

        dt = 1.0 / publish_rate
        self.timer = self.create_timer(dt, self._publish_states)

        self.get_logger().info(
            f"VelocityIntegrator started\n"
            f"  Joints        : {self.joint_names}\n"
            f"  Initial pos   : {self.positions.tolist()}\n"
            f"  Command topic : {command_topic}\n"
            f"  State topic   : {state_topic}\n"
            f"  Publish rate  : {publish_rate} Hz"
        )

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _command_callback(self, msg: JointState):
        """Receive velocity (or position) commands and integrate."""

        now = self.get_clock().now()

        # Build a name→index map so we handle partial / reordered messages
        name_to_idx = {name: i for i, name in enumerate(self.joint_names)}

        has_velocity = len(msg.velocity) == len(msg.name)
        has_position = len(msg.position) == len(msg.name)

        if has_velocity:
            # ── Velocity command: integrate → position ─────────────────────
            if self.last_cmd_time is not None:
                dt = (now - self.last_cmd_time).nanoseconds * 1e-9
                dt = min(dt, 0.1)  # clamp dt to avoid large jumps on resume
            else:
                dt = 0.0

            for msg_idx, name in enumerate(msg.name):
                if name not in name_to_idx:
                    continue
                joint_idx = name_to_idx[name]
                vel = msg.velocity[msg_idx]
                self.velocities[joint_idx] = vel
                self.positions[joint_idx] += vel * dt

            # Enforce joint position limits
            self.positions = np.clip(self.positions, self.pos_min, self.pos_max)

        elif has_position:
            # ── Position command: pass through directly ────────────────────
            for msg_idx, name in enumerate(msg.name):
                if name not in name_to_idx:
                    continue
                joint_idx = name_to_idx[name]
                self.positions[joint_idx] = msg.position[msg_idx]
            self.velocities[:] = 0.0

        self.last_cmd_time = now
        self.cmd_received = True

    def _publish_states(self):
        """Publish current joint states at fixed rate."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions.tolist()
        msg.velocity = self.velocities.tolist()
        msg.effort = self.efforts.tolist()
        self.state_pub.publish(msg)


# ── Entry point ────────────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    node = VelocityIntegrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
