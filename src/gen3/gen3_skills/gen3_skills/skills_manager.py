"""
gen3_skills.skills_manager.py

Node for managing all skills for the Kinova Gen3 Arm

Written by Will Solow, 2025
"""

import rclpy 
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from gen3_cpp.msgs import ParamDict
from rcl_interfaces.msg import SetParametersResult

from gen3_skills.skills import skills
import inspect

class SkillsManager(Node):

    def __init__(self) -> None:

        super().__init__("gen3_skills_manager")

        self.declare_parameter("cmd_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("isaac", False) # If running in isaacsim
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.isaac = self.get_parameter("isaac").value

        self.TRAJ_TOPIC_TYPE = JointTrajectory if not self.isaac else JointState

        self.traj_pub = self.create_publisher(self.TRAJ_TOPIC_TYPE, self.cmd_topic, 10)

        self.avail_skills = inspect.getmembers(skills, inspect.isfunction)

        self.skills_client = ActionClient(self, ParamDict, "/skills")

    def send_skill(self, position: float = 0.0, max_effort: float = 100.0) -> None:
        """
        Send position goal to the gripper
        """
        self.skills_client.wait_for_server()

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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
