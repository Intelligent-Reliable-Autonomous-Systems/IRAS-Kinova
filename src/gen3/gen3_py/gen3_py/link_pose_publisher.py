import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node


class LinkPoseListener(Node):
    def __init__(self):
        super().__init__("link_pose_listener")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_link_pose(self, link_name: str, reference_frame: str = "world"):
        try:
            # Look up the latest transform
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                reference_frame,  # target frame
                link_name,  # source frame
                rclpy.time.Time(),
            )
            return trans
        except Exception as e:
            self.get_logger().warn(f"Could not get transform for {link_name}: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = LinkPoseListener()

    # Wait a little for TF buffer to fill
    rclpy.spin_once(node, timeout_sec=0.5)

    link_names = ["base_link", "wrist_3_link", "tool0"]
    for link in link_names:
        pose = node.get_link_pose(link)
        if pose:
            print(
                f"{link}: translation={pose.transform.translation}, rotation={pose.transform.rotation}"
            )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
