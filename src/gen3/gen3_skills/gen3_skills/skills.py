"""
gen3_skills.skills.skills.py

Class of available skills for gen3 manipulation

Written by Will Solow, 2025
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import SetParametersResult
from moveit_msgs.srv import GetPositionIK
from gen3_cpp.srv import ParamSkill
from moveit_msgs.msg import PositionIKRequest, RobotState
from builtin_interfaces.msg import Duration
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup


import numpy as np

from gen3_skills.utils import (
    ARM_JOINTS,
    GRIPPER_JOINTS,
    GRIPPER_CTRL_JOINT,
    GRIPPER_CTRL_JOINT_ID,
    GRIPPER_OPEN,
    GRIPPER_CLOSE,
)


class Skills(Node):

    SKILLS = [
        "go_to_xyz",
        "go_to_xy",
        "top_grasp",
        "side_grasp",
        "lift",
        "twist",
        "rotate",
        "no_op",
        "place",
        "reset",
        "open",
        "close",
    ]

    def __init__(self) -> None:

        super().__init__("gen3_skills")

        self.declare_parameter("state_topic", "/joint_states")
        self.declare_parameter("isaac", False)  # If running in isaacsim
        self.declare_parameter("ee_topic", "ee_pose")
        self.state_topic = self.get_parameter("state_topic").value
        self.isaac = self.get_parameter("isaac").value
        self.ee_topic = self.get_parameter("ee_topic").value

        self.arm_group_name = "manipulator"
        self.gripper_group_name = "gripper"

        self._reentrant_cb_group = ReentrantCallbackGroup()
        self._mu_cb_group = MutuallyExclusiveCallbackGroup()

        self.TRAJ_TOPIC_TYPE = JointTrajectory if not self.isaac else JointState

        self.traj_sub = self.create_subscription(JointState, self.state_topic, self.robot_state_callback, 10)
        self.ee_sub = self.create_subscription(PoseStamped, self.ee_topic, self.ee_pose_callback, 10)

        self.srv = self.create_service(ParamSkill, "/get_skill_trajectory", self.skill_trajectory_callback)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=self._reentrant_cb_group)

        self.add_on_set_parameters_callback(self.param_callback)

        self.has_joint_data = False
        self.has_ee_pose = False

        self.available_skills = {
            name: getattr(self, name) for name in dir(self) if callable(getattr(self, name)) and name in self.SKILLS
        }

        self.last_ik_solution = None
        self.new_ik_solution = False

    def robot_state_callback(self, msg: JointState) -> None:
        """
        Callback for receiving controller state messages.
        Updates the current joint positions and passes the state to the robot model.
        """

        self.current_joint_positions = np.array(msg.position, dtype=np.float32)
        self.current_joint_names = np.array(msg.name, dtype=str)
        self.current_joint_velocities = np.array(msg.velocity, dtype=np.float32)
        self.has_joint_data = True

    def ee_pose_callback(self, msg: PoseStamped) -> None:
        """
        Get the end effector pose in XYZ RPY
        """
        self.ee_pose = msg.pose
        self.has_ee_pose = True

    async def skill_trajectory_callback(self, request, response):
        """
        Callaback for skill trajectories
        """
        if request.skill.skill_name not in self.available_skills:
            response.success = False
            return response

        skill = self.available_skills[request.skill.skill_name]

        param_dict = dict(zip(request.skill.param_names, request.skill.param_values))

        joint_state, gripper_pos, arm, gripper = await skill(**param_dict)

        self.get_logger().info(f"{joint_state}, {gripper_pos}, {arm}, {gripper}")
        response.success = True
        response.joint_state = joint_state if joint_state is not None else JointState()
        response.gripper_position = gripper_pos if gripper_pos is not None else float()
        response.gripper = gripper
        response.arm = arm

        return response

    async def go_to_xyz(self, x: float, y: float, z: float, **kwargs) -> JointState:
        """
        Go to a location in xyz space
        """
        req = GetPositionIK.Request()
        ik = PositionIKRequest()

        # Current Pose
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = self.ee_pose.orientation.x
        pose.pose.orientation.y = self.ee_pose.orientation.y
        pose.pose.orientation.z = self.ee_pose.orientation.z
        pose.pose.orientation.w = self.ee_pose.orientation.w

        ik.group_name = "manipulator"
        ik.ik_link_name = "end_effector_link"
        ik.pose_stamped = pose
        ik.timeout = Duration(sec=5)
        ik.avoid_collisions = False

        # Build a RobotState with zeros
        js = JointState()
        js.name = ARM_JOINTS
        js.position = [0.0] * len(ARM_JOINTS)
        rs = RobotState(joint_state=js)

        ik.robot_state = rs
        req.ik_request = ik

        compute_ik_future: Future = self.ik_client.call_async(req)
        await compute_ik_future

        result = compute_ik_future.result()

        if result is None:
            self.get_logger().error("Received None result (timeout or internal error).")
            return None, None, True, False

        if result.error_code.val != 1:
            self.get_logger().error(f"IK failed: error={result.error_code.val}\nFull message:\n{result}")
            return None, None, True, False

        js = result.solution.joint_state
        gripper_pos = js.position[GRIPPER_CTRL_JOINT_ID]
        js.name = js.name[: len(ARM_JOINTS)]
        js.position = js.position[: len(ARM_JOINTS)]

        return js, gripper_pos, True, False

    async def go_to_xy(self, x: float = None, y: float = None, **kwargs) -> JointTrajectory:
        """
        Go to a location specified by xy and mantain end effector orientation
        """
        """
        Go to a location in xyz space
        """
        req = GetPositionIK.Request()
        ik = PositionIKRequest()

        # Current Pose
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = self.ee_pose.position.z
        pose.pose.orientation.x = self.ee_pose.orientation.x
        pose.pose.orientation.y = self.ee_pose.orientation.y
        pose.pose.orientation.z = self.ee_pose.orientation.z
        pose.pose.orientation.w = self.ee_pose.orientation.w

        ik.group_name = "manipulator"
        ik.ik_link_name = "end_effector_link"
        ik.pose_stamped = pose
        ik.timeout = Duration(sec=5)
        ik.avoid_collisions = False

        # Build a RobotState with zeros
        js = JointState()
        js.name = ARM_JOINTS
        js.position = [0.0] * len(ARM_JOINTS)
        rs = RobotState(joint_state=js)

        ik.robot_state = rs
        req.ik_request = ik

        compute_ik_future: Future = self.ik_client.call_async(req)
        await compute_ik_future

        result = compute_ik_future.result()

        if result is None:
            self.get_logger().error("Received None result (timeout or internal error).")
            return None, None, True, False

        self.get_logger().info(f"IK response error_code: {result.error_code.val}")

        if result.error_code.val != 1:
            self.get_logger().error(f"IK failed: error={result.error_code.val}\nFull message:\n{result}")
            return None, None, True, False

        js = result.solution.joint_state
        gripper_pos = js.position[GRIPPER_CTRL_JOINT_ID]
        js.name = js.name[: len(ARM_JOINTS)]
        js.position = js.position[: len(ARM_JOINTS)]

        return result.solution.joint_state, gripper_pos, True, False

    async def no_op(self, **kwargs) -> None:
        """
        No operation
        """

        return None, None, False, False

    async def open(self, **kwargs) -> None:
        """
        No operation
        """

        return None, GRIPPER_OPEN, False, True

    async def close(self, **kwargs) -> None:
        """
        No operation
        """

        return None, GRIPPER_CLOSE, False, True

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

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = Skills()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
