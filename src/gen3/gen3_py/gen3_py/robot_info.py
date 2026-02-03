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

    def timer_callback(self):
        robot = URDF.from_parameter_server()
        upper_limit = []
        lower_limit = []
        joints = []
        links = []
        msg_pub = RobotInfo()
        self.get_logger().info("Recieved robot description...")
        for joint in robot.joints:
            if joint.limit:
                joints.append(joint.name)
                upper_limit.append(joint.limit.upper)
                lower_limit.append(joint.limit.lower)
                self.get_logger().info(joint.name, joint.limit.lower, joint.limit.upper)
        for link in robot.link:
            links.append(link)

        msg_pub.links = links
        msg_pub.joints = joints
        msg_pub.lower_limit = lower_limit
        msg_pub.upper_limit = upper_limit
        self.pub.publish(msg_pub)


def main(args=None):
    rclpy.init(args=args)
    node = RobotInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
