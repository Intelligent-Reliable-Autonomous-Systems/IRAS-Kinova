"""
TF Relay node to republish everything on /tf and /tf_static

Written by Will Solow, 2025
"""

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TfRelayNode(Node):
    def __init__(self):
        super().__init__("tf_relay_node")

        self.declare_parameter("namespace", "")

        ns = self.get_parameter("namespace").get_parameter_value().string_value.strip("/")
        if ns != "":
            ns = ns + "/"

        self.input_tf_topic = ns + "tf"
        self.input_tf_static_topic = ns + "tf_static"

        self.get_logger().info(
            f"Relaying TF topics: {self.input_tf_topic} -> /tf and {self.input_tf_static_topic} -> /tf_static"
        )

        self.tf_sub = self.create_subscription(TFMessage, self.input_tf_topic, self.tf_callback, 10)
        self.tf_static_sub = self.create_subscription(
            TFMessage, self.input_tf_static_topic, self.tf_static_callback, 10
        )

        static_qos = QoSProfile(depth=10)
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        # Publishers on standard TF topics
        self.tf_pub = self.create_publisher(TFMessage, "tf", 10)
        self.tf_static_pub = self.create_publisher(TFMessage, "tf_static", 10)

    def tf_callback(self, msg):
        self.tf_pub.publish(msg)

    def tf_static_callback(self, msg):
        self.tf_static_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfRelayNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
