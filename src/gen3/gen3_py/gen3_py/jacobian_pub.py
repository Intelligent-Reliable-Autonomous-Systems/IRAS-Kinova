import tempfile

import numpy as np
import pinocchio as pin
import rclpy
from gen3_cpp.srv import Jacobian
from rclpy.node import Node
from urdf_parser_py.urdf import URDF

robot = URDF.from_parameter_server()
links = [link.name for link in robot.links]


class JacobianServer(Node):
    def __init__(self):
        super().__init__("jacobian_publisher")

        self.declare_parameter("robot_description", "")
        self.declare_parameter("link1", "base_link")
        self.declare_parameter("link2", "end_effector_link")
        urdf_str = (
            self.get_parameter("robot_description").get_parameter_value().string_value
        )

        self.base_link = self.get_parameter("link1").get_parameter_value().string_value
        self.ee_link = self.get_parameter("link2").get_parameter_value().string_value

        if not urdf_str:
            raise RuntimeError("robot_description is empty")

        # Pinocchio wants a file, not a stringiras2025

        with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as f:
            f.write(urdf_str.encode("utf-8"))
            self.urdf_path = f.name

        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        self.frame_cache = {}

        self.pub = self.create_publisher(Jacobian, "jacobian", 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Pinocchio Jacobian server ready.")

    def get_frame_id(self, name):
        if name not in self.frame_cache:
            self.frame_cache[name] = self.model.getFrameId(name)
        return self.frame_cache[name]

    def timer_callback(self):
        if not self.ready:
            if self.tf_buffer.can_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            ):
                self.ready = True
                self.get_logger().info("TF tree ready. Starting EE publisher.")
            else:
                self.get_logger().warn("Waiting for TF tree to connect...")
                return
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time()
            )

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame

            msg.pose.position = Point(
                x=t.transform.translation.x,
                y=t.transform.translation.y,
                z=t.transform.translation.z,
            )

            msg.pose.orientation = Quaternion(
                x=t.transform.rotation.x,
                y=t.transform.rotation.y,
                z=t.transform.rotation.z,
                w=t.transform.rotation.w,
            )

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")

    def compute(self, req, res):
        q = np.array(req.q)

        if q.shape[0] != self.model.nq:
            raise RuntimeError(f"Expected {self.model.nq} joints, got {q.shape[0]}")

        ee_id = self.get_frame_id(req.ee_link)

        pin.forwardKinematics(self.model, self.data, q)
        pin.computeJointJacobians(self.model, self.data, q)

        J = pin.getFrameJacobian(
            self.model,
            self.data,
            ee_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )

        res.rows = J.shape[0]
        res.cols = J.shape[1]
        res.data = J.reshape(-1).tolist()

        return res


def main():
    rclpy.init()
    node = JacobianServer()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
