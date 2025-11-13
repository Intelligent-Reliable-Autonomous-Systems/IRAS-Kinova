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
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PolicyController(Node):
    """A controller that loads and executes a policy from a file as a ROS2 node"""

    def __init__(self, name) -> None:
        super().__init__(name)

        self.declare_parameter("state_topic", "/joint_trajectory_controller/controller_state")
        self.declare_parameter("cmd_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("min_traj_dur", 1.0)
        self.declare_parameter("step_size", 0.02)
        self.state_topic = self.get_parameter("state_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.min_traj_dur = self.get_parameter("min_traj_dur").value
        self.step_size = self.get_parameter("step_size").value

        self.create_subscription(JointTrajectoryControllerState, self.state_topic, self.arm_state_callback, 10)

        self.pub = self.create_publisher(JointTrajectory, self.cmd_topic, 10)
        self.timer = self.create_timer(self.step_size, self.robot_cmd_callback)

        self.get_logger().info(f"Initialized {name} policy controller")

    def arm_state_callback(self, msg: JointTrajectoryControllerState):
        """
        Callback for receiving controller state messages.
        Updates the current joint positions and passes the state to the robot model.
        """
        self.update_joint_state(msg.reference.positions, msg.reference.velocities)

    def robot_cmd_callback(self) -> None:
        """
        Timer callback to compute and publish the next joint trajectory command.
        """

        # Get simulation joint positions from the robot's forward model
        joint_pos = self.forward(self.step_size, self.target_pos)

        if joint_pos is not None:
            if len(joint_pos) != self.num_actions:
                self.get_logger().error(f"Expected {self.num_actions} joint positions, got {len(joint_pos)}!")
            else:
                traj = JointTrajectory()
                traj.joint_names = self.joint_names[:7]  # TODO fix this

                point = JointTrajectoryPoint()
                point.positions = joint_pos.tolist()[:7]  # TODO fix this
                point.time_from_start = Duration(sec=1, nanosec=0)

                traj.points.append(point)

                self.pub.publish(traj)
        else:
            pass
            # self.get_logger().info("Joint positions are `None`")

    def load_policy(self, policy_file_path: str, policy_env_path: str) -> bool:
        """
        Load a TorchScript *policy* plus its environment metadata.

        Parameters
        ----------
        model_path : str | Path
            Path to a ``.pt`` / ``.pth`` TorchScript file.
        env_path   : str | Path
            Path to the corresponding ``env.yaml``.
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
            self.policy_env_params = self.parse_env_config(policy_env_path)
        except FileNotFoundError as e:
            self.get_logger().error(f"Unable to load Policy Env file: `{policy_env_path}`")

        (
            self._max_effort,
            self._max_vel,
            self._stiffness,
            self._damping,
            self.default_pos,
            self.default_vel,
            self.joint_names,
            self.actions,
        ) = self.get_robot_joint_properties(self.policy_env_params)
        self.default_pos = self.default_pos[:-2]  # TODO fix this
        self.default_pos[:7] = np.zeros(7)  # TODO fix this
        self.num_joints = len(self.joint_names) - 2  # TODO fix this
        self.num_actions = len(self.actions)

        self.get_logger().info(f"{'Number of joints:':<18} {self.num_joints}")
        self.get_logger().info(f"{'Max effort:':<18} {self._max_effort}")
        self.get_logger().info(f"{'Max velocity:':<18} {self._max_vel}")
        self.get_logger().info(f"{'Stifness:':<18} {self._stiffness}")
        self.get_logger().info(f"{'Damping:':<18} {self._damping}")
        self.get_logger().info(f"{'Default position:':<18} {self.default_pos}")
        self.get_logger().info(f"{'Default velocity:':<18} {self.default_vel}")

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

    def update_joint_state(self, position: np.ndarray, velocity: np.ndarray) -> None:
        """
        Update the current joint state.

        Args:
            position: A list or array of joint positions.
            velocity: A list or array of joint velocities.
        """
        self.current_arm_joint_positions = np.array(
            position[: self.num_joints][: self.num_arm_joints], dtype=np.float32
        )
        self.current_arm_joint_velocities = np.array(
            velocity[: self.num_joints][: self.num_arm_joints], dtype=np.float32
        )
        self.has_joint_data = True

    def parse_env_config(self, env_config_path: str = "env.yaml") -> dict:
        """
        Parses the environment configuration file from a local path or an Omniverse URL.

        Args:
            env_config_path (str, optional): The path to the environment configuration file. Can be local or an Omniverse URL.

        Returns:
            dict: The parsed environment configuration data.
        """

        class SafeLoaderIgnoreUnknown(yaml.SafeLoader):
            def ignore_unknown(self, node) -> None:
                return None

            def tuple_constructor(loader, node) -> tuple:
                return tuple(loader.construct_sequence(node))

        SafeLoaderIgnoreUnknown.add_constructor(
            "tag:yaml.org,2002:python/tuple", SafeLoaderIgnoreUnknown.tuple_constructor
        )
        SafeLoaderIgnoreUnknown.add_constructor(None, SafeLoaderIgnoreUnknown.ignore_unknown)

        with open(env_config_path, "rb") as f:
            file = io.BytesIO(f.read())

        data = yaml.load(file, Loader=SafeLoaderIgnoreUnknown)
        return data

    def get_robot_joint_properties(
        self, data: dict
    ) -> Tuple[List[float], List[float], List[float], List[float], List[float], List[float]]:
        """
        Gets the robot joint properties from the environment configuration data.

        Args:
            data (dict): The environment configuration data.

        Returns:
            tuple: A tuple containing the effort limits, velocity limits, stiffness, damping, default positions, and default velocities.
        """
        actuator_data = data.get("scene").get("robot").get("actuators")
        stiffness = {}
        damping = {}
        effort_limits = {}
        velocity_limits = {}
        default_pos = {}
        default_vel = {}
        joint_names_expr_list = []
        joint_names = []
        actions = []

        for actuator in actuator_data:
            actuator_config = actuator_data.get(actuator)
            joint_names_expr = actuator_config.get("joint_names_expr")
            joint_names_expr_list.extend(joint_names_expr)

            effort_limit = actuator_config.get("effort_limit")
            velocity_limit = actuator_config.get("velocity_limit")
            joint_stiffness = actuator_config.get("stiffness")
            joint_damping = actuator_config.get("damping")

            [[joint_names.append(jn) for jn in self.expand_fnmatch(j)] for j in joint_names_expr]

            if isinstance(effort_limit, (float, int)) or effort_limit is None:
                if effort_limit is None or effort_limit == float("inf"):
                    effort_limit = float(sys.maxsize)
                for names in joint_names_expr:
                    effort_limits[names] = float(effort_limit)
            elif isinstance(effort_limit, dict):
                effort_limits.update(effort_limit)
            else:
                self.get_logger().error(
                    f"Failed to parse effort limit, expected float, int, or dict, got: {type(effort_limit)}"
                )

            if isinstance(velocity_limit, (float, int)) or velocity_limit is None:
                if velocity_limit is None or velocity_limit == float("inf"):
                    velocity_limit = float(sys.maxsize)
                for names in joint_names_expr:
                    velocity_limits[names] = float(velocity_limit)
            elif isinstance(velocity_limit, dict):
                velocity_limits.update(velocity_limit)
            else:
                self.get_logger().error(
                    f"Failed to parse velocity limit, expected float, int, or dict, got: {type(velocity_limit)}"
                )

            if isinstance(joint_stiffness, (float, int)) or joint_stiffness is None:
                if joint_stiffness is None:
                    joint_stiffness = 0
                for names in joint_names_expr:
                    stiffness[names] = float(joint_stiffness)
            elif isinstance(joint_stiffness, dict):
                stiffness.update(joint_stiffness)
            else:
                self.get_logger().error(
                    f"Failed to parse stiffness, expected float, int, or dict, got: {type(joint_stiffness)}"
                )

            if isinstance(joint_damping, (float, int)) or joint_damping is None:
                if joint_damping is None:
                    joint_damping = 0
                for names in joint_names_expr:
                    damping[names] = float(joint_damping)
            elif isinstance(joint_damping, dict):
                damping.update(joint_damping)
            else:
                self.get_logger().error(
                    f"Failed to parse damping, expected float, int, or dict, got: {type(joint_damping)}"
                )

        # parse default joint position
        init_joint_pos = data.get("scene").get("robot").get("init_state").get("joint_pos")
        if isinstance(init_joint_pos, (float, int)):
            for names in joint_names_expr:
                default_pos[names] = float(init_joint_pos)
        elif isinstance(init_joint_pos, dict):
            default_pos.update(init_joint_pos)
        else:
            self.get_logger().error(
                f"Failed to parse init state joint position, expected float, int, or dict, got: {type(init_joint_pos)}"
            )

        # parse default joint velocity
        init_joint_vel = data.get("scene").get("robot").get("init_state").get("joint_vel")
        if isinstance(init_joint_vel, (float, int)):
            for names in joint_names_expr:
                default_vel[names] = float(init_joint_vel)
        elif isinstance(init_joint_vel, dict):
            default_vel.update(init_joint_vel)
        else:
            self.get_logger().error(
                f"Failed to parse init state vel position, expected float, int, or dict, got: {type(init_joint_vel)}"
            )

        stiffness_inorder = []
        damping_inorder = []
        effort_limits_inorder = []
        velocity_limits_inorder = []
        default_pos_inorder = []
        default_vel_inorder = []

        for joint in joint_names:
            if isinstance(stiffness, float):
                stiffness_inorder.append(stiffness)
            elif isinstance(stiffness, dict):
                for pattern in list(stiffness.keys()):
                    if fnmatch.fnmatch(joint, pattern.replace(".", "*") + "*"):
                        if pattern in stiffness:
                            stiffness_inorder.append(stiffness[pattern])
                        else:
                            stiffness_inorder.append(0)
                            print(f"{joint} stiffness not found, setting to 0")
                        break
            if isinstance(damping, float):
                damping_inorder.append(damping)
            elif isinstance(damping, dict):
                for pattern in list(damping.keys()):
                    if fnmatch.fnmatch(joint, pattern.replace(".", "*") + "*"):
                        if pattern in damping:
                            damping_inorder.append(damping[pattern])
                        else:
                            damping_inorder.append(0)
                            print(f"{joint} damping not found, setting to 0")
                        break
            if isinstance(effort_limit, float):
                effort_limits_inorder.append(effort_limit)
            elif isinstance(effort_limit, dict):
                for pattern in list(effort_limit.keys()):
                    if fnmatch.fnmatch(joint, pattern.replace(".", "*") + "*"):
                        if pattern in effort_limits:
                            effort_limits_inorder.append(effort_limits[pattern])
                        else:
                            effort_limits_inorder.append(0)
                            print(f"{joint} effort limit not found, setting to 0")
                        break
            if isinstance(velocity_limit, float):
                velocity_limits_inorder.append(velocity_limit)
            elif isinstance(velocity_limit, dict):
                for pattern in list(velocity_limit.keys()):
                    if fnmatch.fnmatch(joint, pattern.replace(".", "*") + "*"):
                        if pattern in velocity_limits:
                            velocity_limits_inorder.append(velocity_limits[pattern])
                        else:
                            velocity_limits_inorder.append(0)
                            print(f"{joint} velocity limit not found, setting to 0")
                        break

            if isinstance(default_pos, float):
                default_pos_inorder.append(default_pos)
            elif isinstance(default_pos, dict):
                for pattern in list(default_pos.keys()):
                    if fnmatch.fnmatch(joint, pattern.replace(".", "*") + "*"):
                        if pattern in default_pos:
                            default_pos_inorder.append(default_pos[pattern])
                        else:
                            default_pos_inorder.append(0)
                            self.get_logger().warn(f"{joint} default position not found, setting to 0")
                        break

            if isinstance(default_vel, float):
                default_vel_inorder.append(default_vel)
            elif isinstance(default_vel, dict):
                for pattern in list(default_vel.keys()):
                    if fnmatch.fnmatch(joint, pattern.replace(".", "*") + "*"):
                        if pattern in default_vel:
                            default_vel_inorder.append(default_vel[pattern])
                        else:
                            default_vel_inorder.append(0)
                            self.get_logger().warn(f"{joint} default position not found, setting to 0")
                        break

        act_list = data.get("actions")
        [[actions.append(a) for a in act_list.get(act).get("joint_names")] for act in act_list]

        self.get_logger().info(f"Num Joints: {len(joint_names)}")
        self.get_logger().info(f"Num Actions: {len(actions)}")
        return (
            effort_limits_inorder,
            velocity_limits_inorder,
            stiffness_inorder,
            damping_inorder,
            default_pos_inorder,
            default_vel_inorder,
            joint_names,
            actions,
        )

    def expand_fnmatch(self, pattern: str) -> list[str]:
        # Find character ranges like [1-7]
        ranges = re.findall(r"\[([^\]]+)\]", pattern)
        if not ranges:
            return [pattern]

        # Convert each range to a list of characters/numbers
        options = []
        for r in ranges:
            chars = []
            i = 0
            while i < len(r):
                if i + 2 < len(r) and r[i + 1] == "-":
                    chars.extend([str(x) for x in range(int(r[i]), int(r[i + 2]) + 1)])
                    i += 3
                else:
                    chars.append(r[i])
                    i += 1
            options.append(chars)

        # Replace ranges with placeholders to combine later
        parts = re.split(r"\[[^\]]+\]", pattern)
        all_combinations = []
        for combo in itertools.product(*options):
            s = "".join(p + c for p, c in zip(parts, list(combo) + [""]))
            all_combinations.append(s)

        return all_combinations

    def map_joint_angle(self, pos: float, index: int) -> float:
        """
        Map a simulation joint angle (in radians) to the real-world servo angle (in radians).

        Args:
            pos: Joint angle from simulation (in radians).
            index: Index of the joint.

        Returns:
            Mapped joint angle within the servo limits.
        """
        L, U, inversed = self.SIM_DOF_ANGLE_LIMITS[index]
        A, B = self.SERVO_ANGLE_LIMITS[index]
        angle_deg = np.rad2deg(float(pos))
        # Check if the simulation angle is within limits
        if not L <= angle_deg <= U:
            self.get_logger().warn(f"Simulation joint {index} angle ({angle_deg}) out of range [{L}, {U}]. Clipping.")
            angle_deg = np.clip(angle_deg, L, U)
        # Map the angle from the simulation range to the servo range
        mapped = (angle_deg - L) * ((B - A) / (U - L)) + A
        if inversed:
            mapped = (B - A) - (mapped - A) + A
        # Verify the mapped angle is within servo limits
        if not A <= mapped <= B:
            raise Exception(f"Mapped joint {index} angle ({mapped}) out of servo range [{A}, {B}].")
        return mapped
