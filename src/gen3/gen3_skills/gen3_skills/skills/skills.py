"""
gen3_skills.skills.skills.py

Class of available skills for gen3 manipulation

Written by Will Solow, 2025
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import SetParametersResult
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from moveit_msgs.srv import GetPositionIK
from gen3_cpp.srv import ParamSkill


import numpy as np 

class Skills(Node):

    SKILLS = ["go_to_xyz",
              "go_to_xy",
              "top_grasp",
              "side_grasp",
              "lift",
              "twist",
              "rotate",
              "no_op",
              "place",
              "reset"]
    
    def __init__(self) -> None:

        super().__init__("gen3_skills")

        self.declare_parameter("state_topic", "/joint_states")
        self.declare_parameter("isaac", False) # If running in isaacsim
        self.declare_parameter("ee_topic", "ee_pose")
        self.state_topic = self.get_parameter("state_topic").value
        self.isaac = self.get_parameter("isaac").value
        self.ee_topic = self.get_parameter("ee_topic").value

        self.arm_group_name = "manipulator"
        self.gripper_group_name = "gripper"

        self.TRAJ_TOPIC_TYPE = JointTrajectory if not self.isaac else JointState

        self.traj_sub = self.create_subscription(JointState, self.state_topic, self.robot_state_callback, 10)
        self.ee_sub = self.create_subscription(PoseStamped, self.ee_topic, self.ee_pose_callback, 10)

        self.srv = self.create_service(ParamSkill, '/get_skill_trajectory',self.skill_trajectory_callback)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        self.add_on_set_parameters_callback(self.param_callback)

        self.has_joint_data = False
        self.has_ee_pose = False

        self.available_skills = {
            name: getattr(self, name)
            for name in dir(self)
            if callable(getattr(self, name)) and name in self.SKILLS
        }

    def skill_trajectory_callback(self, request, response):
        """
        Callaback for skill trajectories
        """
        if request.skill.skill_name not in self.available_skills:
            response.success = False
            return response 
        
        skill = self.available_skills[request.skill.skill_name]

        param_dict = dict(zip(request.skill.param_names, request.skill.param_values))

        traj = skill(**param_dict)
        response.success = traj is not None
        response.trajectory = traj

        return response

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
        q = [msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w]

        r, p, y = euler_from_quaternion(q)
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z

        self.ee_pose = np.array([x,y,z,r,p,y],dtype=np.float32)
        self.has_ee_pose = True

    def go_to_xyz(self, x: float=None, y: float=None, z: float=None, **kwargs) -> JointTrajectory:
        """
        Go to a location specified by xyz and mantain end effector orientation
        """

        self.get_logger().info(f"x: {x}, y: {y}, z: {z}")
        if not self.has_ee_pose: return None

        q = quaternion_from_euler(self.ee_pose[3], self.ee_pose[4], self.ee_pose[5])

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.arm_group_name
        req.ik_request.pose_stamped = pose
        req.ik_request.timeout.sec = 1
        '''req.ik_request.robot_state.joint_state.name = self.current_joint_names
        req.ik_request.robot_state.joint_state.position = self.current_joint_positions 
        req.ik_request.robot_state.joint_state.velocity = self.current_joint_velocities'''
        req.ik_request.robot_state.joint_state.name = [
            "joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7"
        ]
        req.ik_request.robot_state.joint_state.position = [0.0]*7
        req.ik_request.ik_link_name = "end_effector_link"
        req.ik_request.avoid_collisions = True

        self.get_logger().info("STARTING SPIN...")
        self.get_logger().info(f"IK Request: {req}")
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result().error_code.val == 1: 
            joints = future.result().solution.joint_state
            self.get_logger().info("NOT RETURNING NONE...")

            return dict(zip(joints.name, joints.position))
        else:
            self.get_logger().info("RETURNING NONE...")
            return None

    def go_to_xy(x: float=None, y: float=None, curr_state: JointState=None, **kwargs) -> JointTrajectory:
        """
        Go to a location specified by xy and mantain end effector orientation
        """
        pass

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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()