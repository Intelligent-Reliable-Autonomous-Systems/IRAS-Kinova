"""
policy_controller.py

Minimal wrapper to load a TorchScript policy and expose helpers to:

* build observations   implemented in subclasses
* compute actions      via `_compute_action()`

The class also extracts physics and robot-joint parameters from the
`env.yaml` that accompanies each policy.

Sub-classes must set `self.dof_names` before calling `load_policy`
(in `__init__`) and implement:

* `_compute_observation()`
* `forward()`

Author: Will Solow
"""

import io
import numpy as np
import torch
import fnmatch
import re
import io
from typing import List, Tuple
import sys
import yaml
import itertools

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from rcl_interfaces.msg import SetParametersResult


class PolicyController(Node):
    """A controller that loads and executes a policy from a file as a ROS2 node"""

    current_joint_positions = None
    current_joint_velocities = None
    ISAAC_OC = np.array([-1.0, 1.0])  # min/max for gripper open close in IsaacSim
    GEN3_OC = np.array([0, 0.8])  # min/max for gripper open/close in Gen3 hardware
    
    def __init__(self, name) -> None:
        super().__init__(name)

        self.declare_parameter("state_topic", "/joint_states")
        self.declare_parameter("cmd_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("min_traj_dur", 1.0)
        self.declare_parameter("step_size", 0.02)
        self.declare_parameter("isaac", False) # If running in isaacsim
        self.state_topic = self.get_parameter("state_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.min_traj_dur = self.get_parameter("min_traj_dur").value
        self.step_size = self.get_parameter("step_size").value
        self.isaac = self.get_parameter("isaac").value

        self.TRAJ_TOPIC_TYPE = JointTrajectory if not self.isaac else JointState

        self.traj_sub = self.create_subscription(JointState, self.state_topic, self.robot_state_callback, 10)
        self.add_on_set_parameters_callback(self.param_callback)

        self.traj_pub = self.create_publisher(self.TRAJ_TOPIC_TYPE, self.cmd_topic, 10)
        self.timer = self.create_timer(self.step_size, self.robot_cmd_callback)
        self.gripper_action_client = ActionClient(self, GripperCommand, "/robotiq_gripper_controller/gripper_cmd")

        self.get_logger().info(f"Initialized {name} policy controller")
        self.has_joint_data = False
        self.has_default_pos = False

    def robot_state_callback(self, msg: JointState):
        """
        Callback for receiving controller state messages.
        Updates the current joint positions and passes the state to the robot model.
        """
        self.update_joint_state(msg.position, msg.velocity)

    def update_joint_state(self, position: np.ndarray, velocity: np.ndarray) -> None:
        """
        Update the current joint state.

        Args:
            position: A list or array of joint positions.
            velocity: A list or array of joint velocities.
        """
        if not self.has_default_pos:
            self.default_pos = np.array(position[: self.num_joints], dtype=np.float32)
            self.has_default_pos = True
        self.current_joint_positions = np.array(position[: self.num_joints], dtype=np.float32)
        
        self.current_joint_velocities = np.array(velocity[: self.num_joints], dtype=np.float32)
        self.has_joint_data = True

    def robot_cmd_callback(self) -> None:
        """
        Timer callback to compute and publish the next joint trajectory command
        and send action to gripper
        """

        # Get simulation joint positions from the robot's forward model
        joint_pos = self.forward(self.step_size, self.target_pos)

        if joint_pos is not None:
            if len(joint_pos) != self.num_actions:
                self.get_logger().error(f"Expected {self.num_actions} joint positions, got {len(joint_pos)}!")
            else:
                if self.isaac:
                    traj = JointState()
                    traj.name = self.arm_joints + self.gripper_joints
                    traj.position = joint_pos.tolist()
                else: 
                    traj = JointTrajectory()
                    traj.joint_names = self.arm_joints

                    point = JointTrajectoryPoint()
                    point.positions = joint_pos.tolist()[: len(self.arm_actions)]
                    point.time_from_start = Duration(sec=1, nanosec=0)

                    traj.points.append(point)
                    self.send_gripper_goal(position=joint_pos[-1])
                self.traj_pub.publish(traj)
        else:
            pass
            # self.get_logger().info("Joint positions are `None`")

    def send_gripper_goal(self, position: float = 0.0, max_effort: float = 100.0) -> None:
        """
        Send position goal to the gripper
        """
        self.gripper_action_client.wait_for_server()

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        send_goal_future = self.gripper_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

    def load_policy(self, policy_file_path: str, policy_env_path: str) -> bool:
        """
        Load a TorchScript *policy* plus its environment metadata.

        Parameters
        ----------
        model_path : str | Path
            Path to a ``.pt`` / ``.pth`` TorchScript file.
        env_path   : str | Path
            Path to the corresponding metadata ``env.yaml``.
        """

        self.get_logger().info(f"Loading policy: `{policy_file_path}`")
        try:
            with open(policy_file_path, "rb") as f:
                file = io.BytesIO(f.read())
            self.policy = torch.jit.load(file)
        except FileNotFoundError as e:
            self.get_logger().error(f"Unable to load Torch Jit file: `{policy_file_path}`")

        self.get_logger().info(f"Loading env file: `{policy_env_path}`")
        try:
            with open(policy_env_path, "rb") as f:
                file = io.BytesIO(f.read())
            self.policy_env_params = yaml.safe_load(file)
        except FileNotFoundError as e:
            self.get_logger().error(f"Unable to load Policy Env file: `{policy_env_path}`")

        self.arm_joints, self.gripper_joints, self.arm_actions, self.gripper_actions, self.default_pos = (
            self.get_joint_properties(self.policy_env_params)
        )

        self.num_joints = len(self.arm_joints) + len(self.gripper_joints)
        self.num_actions = len(self.arm_actions) + len(self.gripper_actions)

        self.get_logger().info(f"{'Number of joints:':<18} {self.num_joints}")
        self.get_logger().info(f"{'Number of actions:':<18} {self.num_actions}")

    def get_joint_properties(self, data: dict) -> Tuple[dict, dict]:
        """
        Handle metadata passed in `env.yaml` file

        Args:
            data (dict): Dictionary of metadata for policy
        Returns:
            dict: Joint observations
            dict: Actions
        """
        joints = data["actuators"]
        actions = data["actions"]

        arm_joints = joints["arm"]["joint_names"]
        gripper_joints = joints["gripper"]["joint_names"]

        arm_actions = actions["arm_action"]["joint_names"]
        gripper_actions = actions["gripper_action"]["joint_names"]

        if "default_pos" in data.keys():
            default_pos = np.array(list(data["default_pos"].values()))
            self.has_default_pos = True

        return arm_joints, gripper_joints, arm_actions, gripper_actions, default_pos

    def _compute_action(self, obs: np.ndarray) -> np.ndarray:
        """
        Computes the action from the observation using the loaded policy.

        Args:
            obs (np.ndarray): The observation.

        Returns:
            np.ndarray: The action.
        """
        with torch.no_grad():
            obs = torch.from_numpy(obs).view(1, -1).float()
            action = self.policy(obs).detach().view(-1).numpy()
        return action
    
    def param_callback(self, params):
        for param in params:
            if param.name == "isaac":
                if not isinstance(param.value, bool):
                    self.get_logger().warn(f"Isaac param must be of type `bool`")
                    return SetParametersResult(successful=False)
                self.get_logger().info(f"Updated `isaac` to: {param.value}")
                self.isaac = param.value
                self.gen3_oc = self.GEN3_OC if not self.isaac else self.ISAAC_OC
                self.TRAJ_TOPIC_TYPE = JointTrajectory if not self.isaac else JointState
                self.destroy_publisher(self.traj_pub)
                self.traj_pub = self.create_publisher(self.TRAJ_TOPIC_TYPE, self.cmd_topic, 10)
            if param.name == "state_topic":
                if not isinstance(param.value, str):
                    self.get_logger().warn(f"`state_topic` param must be of type `str`")
                    return SetParametersResult(successful=False)
                self.get_logger().info(f"Updated `state_topic` to: {param.value}")
                self.state_topic = param.value
                self.destroy_subscription(self.traj_sub)
                self.traj_sub = self.create_subscription(JointState, self.state_topic, self.robot_state_callback, 10)
            if param.name == "cmd_topic":
                if not isinstance(param.value, str):
                    self.get_logger().warn(f"`cmd_topic` param must be of type `str`")
                    return SetParametersResult(successful=False)
                self.get_logger().info(f"Updated `cmd_topic` to: {param.value}")
                self.cmd_topic = param.value
                self.destroy_publisher(self.traj_pub)
                self.traj_pub = self.create_publisher(self.TRAJ_TOPIC_TYPE, self.cmd_topic, 10)

        return SetParametersResult(successful=True)

    def _compute_observation(self) -> NotImplementedError:
        """Build an observation, must be overridden."""

        raise NotImplementedError(
            "Compute observation need to be implemented, expects np.ndarray in the structure specified by env yaml"
        )

    def forward(self) -> NotImplementedError:
        """Return the next command, must be overridden."""

        raise NotImplementedError(
            "Forward needs to be implemented to compute and apply robot control from observations"
        )
