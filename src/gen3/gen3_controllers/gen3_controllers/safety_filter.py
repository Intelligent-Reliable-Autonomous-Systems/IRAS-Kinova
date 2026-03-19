"""
safety_filter.py

The filter layer to check the safety of the command before publishing it to the controller.

The class extracts joint limits from the`urdf` file.

Author: Natalia Zaitseva
"""

import numpy as np
import rclpy
from gen3_skills.utils import ARM_JOINTS
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from urdf_parser_py.urdf import URDF


class SafetyFilter(Node):
    def __init__(self):
        super().__init__("safety_filter")
        self.in_topic = "/cmd_safety"
        self.out_topic = "/joint_trajectory_controller/joint_trajectory"
        self.state_topic = "/joint_states"
        self.traj_dt = 1.0
        self.current_pos = None
        self.current_vel = None
        self.state_sub = self.create_subscription(JointState, self.state_topic, self.state_cb, 10)
        self.cmd_sub = self.create_subscription(JointTrajectory, self.in_topic, self.cmd_cb, 10)
        self.cmd_pub = self.create_publisher(JointTrajectory, self.out_topic, 10)

        self.get_logger().info(f"SafetyFilter: {self.in_topic} -> {self.out_topic}")

        self.joint_limits = self.read_joint_limits(
            "./src/third_party/ros2_kortex/ros2_kortex/kortex_description/robots/gen3_2f85.urdf"
        )

        self.get_logger().info(f"{self.joint_limits}")

    def state_cb(self, msg: JointState):
        """Callback for states
        Runs automatically when receives the joint states"""
        self.current_pos = np.array(msg.position[:7], dtype=np.float32)
        self.current_vel = np.array(msg.velocity[:7], dtype=np.float32)
        self.current_torq = np.array(msg.effort[:7], dtype=np.float32)

    def cmd_cb(self, msg: JointTrajectory):
        """Publishing callback
        Runs when the message is published to cmd_safety"""
        if self.current_pos is None or self.current_vel is None:
            self.get_logger().warn("No joint state yet; dropping command.")
            return

        if not msg.points or len(msg.points[0].positions) < 7:
            self.get_logger().warn("Bad trajectory message; dropping.")
            return

        target = np.array(msg.points[0].positions[:7], dtype=np.float32)
        safe = self.safety_arm_check(self.current_pos, target, self.current_vel, self.traj_dt)
        if safe:
            self.get_logger().info("SAFE: forwarding")
            self.cmd_pub.publish(msg)
        else:
            self.get_logger().warn("BLOCKED: unsafe command")

    def read_joint_limits(self, filepath):
        """Read joint limits from the URDF file and normalizes the names
        from gen3_joint_1 to joint_1 that we use."""
        robot = URDF.from_xml_file(filepath)
        joint_limits = {}

        for joint in robot.joints:
            if joint.type in ["revolute", "continuous", "prismatic"] and joint.limit:
                name = self.normalize_urdf_name(joint.name)
                joint_limits[name] = {
                    "lower": joint.limit.lower,
                    "upper": joint.limit.upper,
                    "effort": joint.limit.effort,
                    "velocity": joint.limit.velocity,
                }
        return joint_limits

    # to avoid joint names mismatch
    def normalize_urdf_name(self, urdf_name):
        if urdf_name.startswith("gen3_"):
            return urdf_name.replace("gen3_", "")
        else:
            return urdf_name

    def safety_arm_check(self, current_joint_positions, joint_pos, current_joint_velocities, step_size):
        """Checks if the joint implied velocity adn acceleration are within a safe range,
        which implies that the torque is in the safe range and prevents the hardware shoutdown."""

        for i, joint_name in enumerate(ARM_JOINTS):
            velocity_lim = self.joint_limits[joint_name]["velocity"]
            lower_lim = self.joint_limits[joint_name]["lower"]
            upper_lim = self.joint_limits[joint_name]["upper"]
            effort_lim = self.joint_limits[joint_name]["effort"]
            print(f"Upper limit: {upper_lim}")
            print(f"Lower limit: {lower_lim}")
            print(f"Velocity limit: {velocity_lim}")
            print(f"Torque limit: {effort_lim}")

            # check for continuous joints
            if lower_lim == upper_lim:
                continue

            # dt = step_size in that case
            implied_velocity = (joint_pos[i] - current_joint_positions[i]) / step_size
            if abs(implied_velocity) > velocity_lim:
                self.get_logger().info(f"{joint_name} exceeds allowable velocity")
                return False
            if joint_pos[i] < lower_lim or joint_pos[i] > upper_lim:
                self.get_logger().info(f"{joint_name} exceeds allowable joint limits")
                return False
            if self.current_torq[i] > effort_lim:
                self.get_logger().info(f"{joint_name} exceeds the torque limits")
                return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SafetyFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
