"""
gen3_skills.skills_manager.py

Node for managing all skills for the Kinova Gen3 Arm

Written by Will Solow, 2025
"""

import rclpy 
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from gen3_cpp.msg import Params
from rcl_interfaces.msg import SetParametersResult
from gen3_cpp.srv import ParamSkill

class SkillsManager(Node):

    def __init__(self) -> None:

        super().__init__("gen3_skills_manager")

        self.declare_parameter("cmd_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("isaac", False) # If running in isaacsim
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.isaac = self.get_parameter("isaac").value

        self.TRAJ_TOPIC_TYPE = JointTrajectory if not self.isaac else JointState

        self.traj_pub = self.create_publisher(self.TRAJ_TOPIC_TYPE, self.cmd_topic, 10)

        self.skills_sub = self.create_subscription(Params, '/run_skill', self.skill_callback, 10)

        self.skills_client = self.create_client(ParamSkill, '/get_skill_trajectory')
        while not self.skills_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Skills service `/get_skill_trajectory` not available, waiting...')

    def skill_callback(self, msg: Params) -> None:
        """
        Send position goal to the gripper
        """
        self.get_logger().info(f"{msg}")

        req = ParamSkill.Request()
        req.skill = msg

        # Call the service asynchronously
        future = self.skills_client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    
    def service_response_callback(self, future):
        self.get_logger().info("Callaing response callback...")
        try:
            response = future.result()
            self.get_logger().info(f"Service success: {response.success}, message: {response.message}")
            self.get_logger().info(f"{response.trajectory}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")




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
