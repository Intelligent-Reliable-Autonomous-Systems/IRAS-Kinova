"""
Docstring for gen3.gen3_py.gen3_py.ee_publisher

Publishes the end effector position relative to the base link

Written by Will Solow, 2025
"""

import rclpy
from gen3_cpp.msg import RobotInfo
from rclpy.node import Node
from urdf_parser_py.urdf import URDF


class RobotInfoPublisher(Node):
    def __init__(self):
        super().__init__("robot_info")
        self.pub = self.create_publisher(RobotInfo, "/robot_info", 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.declare_parameter("robot_description", "")
        urdf_xml = (
            self.get_parameter("robot_description").get_parameter_value().string_value
        )
        self.robot = URDF.from_xml_string(urdf_xml)

    def timer_callback(self):
        upper_limit = []
        lower_limit = []
        joints = []
        links = []
        msg_pub = RobotInfo()
        for joint in self.robot.joints:
            if joint.limit:
                joints.append(joint.name)
                upper_limit.append(joint.limit.upper)
                lower_limit.append(joint.limit.lower)
        for link in self.robot.links:
            links.append(link.name)

        msg_pub.links = links
        msg_pub.joints = joints
        msg_pub.lower_limits = lower_limit
        msg_pub.upper_limits = upper_limit
        self.pub.publish(msg_pub)


def main(args=None):
    rclpy.init(args=args)
    node = RobotInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
