import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class TestServo(Node):
    def __init__(self):
        super().__init__("test_servo")

        self.pub = self.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            10,
        )

        self.timer = self.create_timer(0.02, self.publish_twist)

    def publish_twist(self):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "end_effector_link"

        msg.twist.linear.z = -0.04

        self.pub.publish(msg)


rclpy.init()
node = TestServo()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
