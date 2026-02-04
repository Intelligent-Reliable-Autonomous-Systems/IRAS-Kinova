import PyKDL
import rclpy
from gen3_cpp.msg import Jacobian
from kdl_parser_py.urdf import treeFromUrdfModel
from rclpy.node import Node
from urdf_parser_py.urdf import URDF


class JacobianPublisher(Node):
    def __init__(self):
        super().__init__("jacobian_publisher")

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.pub = self.create_publisher(Jacobian, "jacobian", 10)

        self.declare_parameter("robot_description", "")
        urdf_xml = (
            self.get_parameter("robot_description").get_parameter_value().string_value
        )

        self.robot = URDF.from_xml_string(urdf_xml)
        self.ok, self.tree = treeFromUrdfModel(self.robot)
        self.articulated_joints = [j for j in self.robot.joints if j.limit]

    def timer_callback(self) -> None:
        jacobian_msg = Jacobian()
        jacobians = []
        k = 0
        for link in self.robot.links:
            if link.name == "world":
                continue
            try:
                chain = self.tree.getChain("base_link", link.name)
                jac_solver = PyKDL.ChainJntToJacSolver(chain)
                q = PyKDL.JntArray(chain.getNrOfJoints())
                J = PyKDL.Jacobian(chain.getNrOfJoints())
                jac_solver.JntToJac(q, J)
                for i in range(6):
                    for j in range(len(self.articulated_joints)):
                        if j < J.columns():
                            jacobians.append(J[i, j])
                        else:
                            jacobians.append(0)
            except:
                # self.get_logger().info(
                #    f"Unable to compute Jacobian Transform for {link.name}"
                # )
                continue
            k += 1
        jacobian_msg.num_links = k
        jacobian_msg.jac_matrix = jacobians
        jacobian_msg.rows = 6
        jacobian_msg.cols = len(self.articulated_joints)
        # self.get_logger().info(f"Jacobian length {len(jacobians)}")
        self.pub.publish(jacobian_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JacobianPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
