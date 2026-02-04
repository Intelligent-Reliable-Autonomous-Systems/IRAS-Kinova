import rclpy
import tf2_ros
from gen3_cpp.msg import BodyPose
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from urdf_parser_py.urdf import URDF


class LinkPosePublisher(Node):
    def __init__(self):
        super().__init__("link_pose_publisher")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(BodyPose, "/robot_body_pose_w", 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.declare_parameter("robot_description", "")
        urdf_xml = (
            self.get_parameter("robot_description").get_parameter_value().string_value
        )
        self.robot = URDF.from_xml_string(urdf_xml)

    def timer_callback(self):
        pose_msg = BodyPose()
        all_pose_w = []
        i = 0
        for link in self.robot.links:
            if link.name == "world":
                continue
            t = self.get_link_pose(link.name)
            if t is None:
                continue
            pos = t.transform.translation
            quat = t.transform.rotation
            for p in [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w]:
                all_pose_w.append(p)
            i += 1

        t = self.get_link_pose("base_link")
        if t is not None:
            pos = t.transform.translation
            quat = t.transform.rotation
            root_pose_w = [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w]

            pose_msg.num_links = i
            pose_msg.root_pose_w = root_pose_w
            pose_msg.body_pose_w = all_pose_w

            self.pub.publish(pose_msg)
        else:
            self.get_logger().warn("TRANSLATION FROM BASE LINK IS NONE")

    def get_link_pose(self, link_name: str, reference_frame: str = "world"):
        try:
            # Look up the latest transform
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                reference_frame,
                link_name,
                rclpy.time.Time(),
            )
            return trans
        except Exception:
            # self.get_logger().warn(f"Could not get transform for {link_name}: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = LinkPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
