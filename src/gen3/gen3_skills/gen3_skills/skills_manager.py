"""
gen3_skills.skills_manager.py

Node for managing all skills for the Kinova Gen3 Arm

Written by Will Solow, 2025
"""

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from gen3_cpp.msg import Params
from rcl_interfaces.msg import SetParametersResult
from gen3_cpp.srv import ParamSkill
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.task import Future
from rclpy.action import ActionClient
import time

from gen3_skills.utils import ARM_JOINTS, GRIPPER_JOINTS


class SkillsManager(Node):

    def __init__(self) -> None:

        super().__init__("gen3_skills_manager")

        self.declare_parameter("cmd_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("isaac", False)  # If running in isaacsim
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.isaac = self.get_parameter("isaac").value

        self._reentrant_cb_group = ReentrantCallbackGroup()
        self._mu_cb_group = MutuallyExclusiveCallbackGroup()

        self.TRAJ_TOPIC_TYPE = JointTrajectory if not self.isaac else JointState

        self.traj_pub = self.create_publisher(self.TRAJ_TOPIC_TYPE, self.cmd_topic, 10)
        self.gripper_action_client = ActionClient(self, GripperCommand, "/robotiq_gripper_controller/gripper_cmd")

        self.skills_sub = self.create_subscription(
            Params, "/run_skill", self.skill_callback, 10, callback_group=self._mu_cb_group
        )

        self.skills_client = self.create_client(
            ParamSkill, "/get_skill_trajectory", callback_group=self._reentrant_cb_group
        )
        while not self.skills_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Skills service `/get_skill_trajectory` not available, waiting...")

    async def skill_callback(self, msg: Params) -> None:
        """
        Send position goal to the gripper
        """
        self.get_logger().info(f"{msg}")

        req = ParamSkill.Request()
        req.skill = msg

        # Call the service asynchronously
        skills_future: Future = self.skills_client.call_async(req)
        await skills_future

        resp = skills_future.result()

        TRAJ_TIME = 1
        if resp.success:
            if resp.arm:
                traj = JointTrajectory()
                traj.joint_names = resp.joint_state.name
                point = JointTrajectoryPoint()
                point.positions = resp.joint_state.position
                point.time_from_start.sec = TRAJ_TIME
                traj.points.append(point)
                self.traj_pub.publish(traj)
                if resp.gripper:
                    time.sleep(TRAJ_TIME)
            if resp.gripper:
                goal_msg = GripperCommand.Goal()
                goal_msg.command.position = resp.gripper_position
                goal_msg.command.max_effort = 100.0

                send_goal_future = self.gripper_action_client.send_goal_async(goal_msg)
        else:
            self.get_logger().info("Unable to create joint trajectory ")

    def get_result_callback(self, future):
        result = future.result().result

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
            if param.name == "cmd_topic":
                if not isinstance(param.value, str):
                    self.get_logger().warn(f"`cmd_topic` param must be of type `str`")
                    return SetParametersResult(successful=False)
                self.get_logger().info(f"Updated `cmd_topic` to: {param.value}")
                self.cmd_topic = param.value
                self.destroy_publisher(self.traj_pub)
                self.traj_pub = self.create_publisher(self.TRAJ_TOPIC_TYPE, self.cmd_topic, 10)

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = SkillsManager()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
