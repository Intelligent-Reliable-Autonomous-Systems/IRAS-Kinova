"""
gen3_reach.py
--------------------

Thin wrapper around a pre-trained reach policy for the Kinova Gen3 arm with the RoboticIQ 2F85 gripper.
Extends `PolicyController` with:

* State update via `update_joint_state()`
* Forward pass (`forward`) that returns a target joint-position command
  every call, computing a new action every ``decimation`` steps.

Change the default paths below or pass them explicitly in the constructor.

Written by Will Solow, 2025. IRAS Lab.
"""

import numpy as np
import os
import rclpy
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import Marker
import tf_transformations

from gen3_controllers.policy_controller import PolicyController


class Gen3ReachPolicy(PolicyController):
    """Policy controller for Gen3 Reach using a pre-trained policy model."""

    targ_cmd_len = 7  # XYZ RPY + gripper open/close
    isaac_oc = np.array([-1.0, 1.0])  # min/max for gripper open close in IsaacSim
    gen3_oc = np.array([0, 0.8])  # min/max for gripper open/close in Gen3 hardware
    _action_scale = 0.5

    def __init__(self) -> None:
        """Initialize the URReachPolicy instance."""
        super().__init__("Gen3ReachPolicy")

        self.declare_parameter("model_path", f"{os.getcwd()}/sim2real/policies/reach2")
        #self.declare_parameter("target_pos", [0.5, 0.0, 0.2, 0.7071, 0.0, 0.7071, 0.0])
        self.declare_parameter("target_pos", [0.6, 0.1, 0.45, 0.7071, 0.0, 0.7071, 0.0])
        self.model_path = self.get_parameter("model_path").value
        target_pos = self.get_parameter("target_pos").value

        self.load_policy(f"{self.model_path}/policy.pt", f"{self.model_path}/env.yaml")

        self.target_pos = np.array(list(target_pos))
        self._previous_action = np.zeros(self.num_actions)

        self.add_on_set_parameters_callback(self.param_callback)

        self.timer = self.create_timer(self.step_size, self.target_pub_callback)
        self.marker_pub = self.create_publisher(Marker, "target_point", 10)

    def _compute_observation(self, command: np.ndarray) -> np.ndarray:
        """
        Compute the observation vector for the policy network.

        Args:
            command: The target command vector.

        Returns:
            An observation vector if joint data is available, otherwise None.
            Split into [joint_positions, joint_velocities, target_location, previous action]
        """
        if not self.has_joint_data:
            return None
        if not self.has_default_pos:
            return None

        obs = np.zeros(2 * self.num_joints + self.targ_cmd_len + self.num_actions)
        print(self.current_joint_positions)
        obs[: self.num_joints] = self.current_joint_positions - self.default_pos

        obs[self.num_joints : 2 * self.num_joints] = self.current_joint_velocities

        obs[2 * self.num_joints : 2 * self.num_joints + self.targ_cmd_len] = command
        obs[2 * self.num_joints + self.targ_cmd_len :] = self._previous_action

        return obs

    def forward(self, dt: float, command: np.ndarray) -> np.ndarray:
        """
        Compute the next joint positions based on the policy.

        Args:
            dt: Time step for the forward pass.
            command: The target command vector.

        Returns:
            The computed joint positions if joint data is available, otherwise None.
        """
        joint_pos = np.zeros(self.num_actions)
        if not self.has_joint_data:
            return None

        obs = self._compute_observation(command)
        if obs is None:
            return None
        self.action = self._compute_action(obs)
        self._previous_action = self.action.copy()

        joint_pos[: len(self.arm_actions)] = self.default_pos[: len(self.arm_actions)] + (
            self.action[: len(self.arm_actions)] * self._action_scale
        )

        joint_pos[-1] = ((self.action[-1] - self.isaac_oc[0]) / (self.isaac_oc[1] - self.isaac_oc[0])) * (
            self.gen3_oc[1] - self.gen3_oc[0]
        ) + self.gen3_oc[0]

        return joint_pos

    def target_pub_callback(self) -> None:
        """
        Publish the target position to rviz
        """
        q = tf_transformations.quaternion_from_euler(self.target_pos[3], self.target_pos[4], self.target_pos[5])
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = self.target_pos[0]
        marker.pose.position.y = self.target_pos[1]
        marker.pose.position.z = self.target_pos[2]

        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        self.marker_pub.publish(marker)

    def param_callback(self, params):
        for param in params:
            if param.name == "target_pos":
                if len(list(param.value)) != self.targ_cmd_len:
                    self.get_logger().warn(f"Target cmd length must be {self.targ_cmd_len}")
                    return SetParametersResult(successful=False)
                self.get_logger().info(f"Updated target_pos to: {param.value}")
                self.target_pos = np.array(list(param.value))

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = Gen3ReachPolicy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
