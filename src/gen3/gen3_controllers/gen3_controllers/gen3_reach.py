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

from gen3_controllers.policy_controller import PolicyController


class Gen3ReachPolicy(PolicyController):
    """Policy controller for Gen3 Reach using a pre-trained policy model."""

    def __init__(self) -> None:
        """Initialize the URReachPolicy instance."""
        super().__init__("Gen3ReachPolicy")

        self.declare_parameter("model_path", f"{os.getcwd()}/sim2real/pretrained_models/reach4")
        self.declare_parameter("action_scale", 0.5)
        self.declare_parameter("target_pos", [0.5, 0.0, 0.2, 0.7071, 0.0, 0.7071, 0.0])
        self.model_path = self.get_parameter("model_path").value
        self._action_scale = self.get_parameter("action_scale").value
        target_pos = self.get_parameter("target_pos").value

        self.load_policy(f"{self.model_path}/policy.pt", f"{self.model_path}/env.yaml")

        self.target_pos = np.array(list(target_pos))
        self.targ_cmd_len = 7  # XYZ RPY + gripper open/close
        self.num_arm_joints = 7  # TODO fix this
        self.num_gripper_joints = 6
        self.has_joint_data = False

        self._previous_action = np.zeros(self.num_actions)
        self.current_arm_joint_positions = np.zeros(self.num_arm_joints)
        self.current_arm_joint_velocities = np.zeros(self.num_arm_joints)
        self.current_gripper_joint_positions = np.zeros(self.num_gripper_joints)
        self.current_gripper_joint_velocities = np.zeros(self.num_gripper_joints)

        self.add_on_set_parameters_callback(self.param_callback)

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

        obs = np.zeros(2 * self.num_joints + self.targ_cmd_len + self.num_actions)
        obs[: self.num_joints] = (
            np.concatenate((self.current_arm_joint_positions, self.current_gripper_joint_positions)) - self.default_pos
        )
        obs[self.num_joints : 2 * self.num_joints] = np.concatenate(
            (self.current_arm_joint_velocities, self.current_gripper_joint_velocities)
        )
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
        if not self.has_joint_data:
            return None

        obs = self._compute_observation(command)
        if obs is None:
            return None
        self.action = self._compute_action(obs)
        self._previous_action = self.action.copy()

        # Debug Logging (commented out)
        """print("\n=== Policy Step ===")
        print(f"{'Command:':<20} {np.round(command, 4)}\n")
        print("--- Observation ---")
        print(f"{'Δ Joint Positions:':<20} {np.round(obs[:6], 4)}")
        print(f"{'Joint Velocities:':<20} {np.round(obs[6:12], 4)}")
        print(f"{'Command:':<20} {np.round(obs[12:19], 4)}")
        print(f"{'Previous Action:':<20} {np.round(obs[19:25], 4)}\n")
        print("--- Action ---")
        print(f"{'Raw Action:':<20} {np.round(self.action, 4)}")"""
        processed_action = self.default_pos[: self.num_actions] + (self.action * self._action_scale)  # TODO fix this
        # print(f"{'Processed Action:':<20} {np.round(processed_action, 4)}")

        joint_positions = self.default_pos[: self.num_actions] + (self.action * self._action_scale)  # TODO fix this

        return joint_positions

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
