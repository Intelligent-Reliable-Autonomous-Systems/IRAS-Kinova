"""
jacobian_pub.py

Handles publishing of jacobian topic based on current joint positions

Written by Will Solow, 2026. IRAS Lab.
"""

import numpy as np
import PyKDL
import rclpy
from gen3_cpp.msg import Jacobian
from kdl_parser_py.urdf import treeFromUrdfModel
from rclpy.node import Node
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF


class JacobianPublisher(Node):
    def __init__(self):
        super().__init__("jacobian_publisher")
        self.declare_parameter("state_topic", "/joint_states")
        self.state_topic = self.get_parameter("state_topic").value

        self.joint_pos = None
        self.pub = self.create_publisher(Jacobian, "jacobian", 10)
        self.joint_sub = self.create_subscription(JointState, self.state_topic, self.joint_callback, 10)

        self.declare_parameter("robot_description", "")
        urdf_xml = self.get_parameter("robot_description").get_parameter_value().string_value

        self.robot = URDF.from_xml_string(urdf_xml)
        self.ok, self.tree = treeFromUrdfModel(self.robot)
        self.articulated_joints = [j.name for j in self.robot.joints if j.limit]
        self.num_joints = len(self.articulated_joints)

        self.has_all_joint_data = False

    def joint_callback(self, msg: JointState) -> None:
        """Recieve a JointState message and publish the Jacobian Transform between all joints.

        Args:
            msg: a JointState mesage

        """
        self.joint_pos = {}
        for i, j in enumerate(msg.name):
            self.joint_pos[j] = msg.position[i]

        if not self.has_all_joint_data:
            for j in self.articulated_joints:
                if j not in self.joint_pos.keys():
                    self.get_logger().info(f"Waiting for joint info from `{j}`...")
                    return
            self.has_all_joint_data = True

        jacobians = []
        for link in self.robot.links:
            if link.name == "world":
                continue
            jac = self.jacobian(link_name=link.name, positions=self.joint_pos)
            jacobians.append(jac)
        jacobians = np.stack(jacobians, axis=0)

        jacobian_msg = Jacobian()
        jacobian_msg.num_links = jacobians.shape[0]
        jacobian_msg.jac_matrix = jacobians.flatten()
        jacobian_msg.rows = jacobians.shape[1]
        jacobian_msg.cols = jacobians.shape[2]
        self.pub.publish(jacobian_msg)

    def kdl_to_np(self, jac: PyKDL.Jacobian) -> np.ndarray:
        """Convert a PyKDL Jacobian object to a numpy ndarray.

        Input:
            jac: A PyKDL Jacobian data type of shape (num_links, num_joints)

        Returns:
            A numpy.ndarray of shape (num_links, num_joints)

        """
        arr = np.zeros(shape=(6, self.num_joints))
        for i in range(jac.rows()):
            for j in range(jac.columns()):
                arr[i, j] = jac[i, j]
        np.set_printoptions(suppress=True, precision=3)
        return arr

    def jacobian(self, link_name: str = None, positions: dict = None) -> np.ndarray:
        """Computes the Jacobian matrices of all the joints.

        Args:
            link_name: a string of the link name to compute the Jacobian for
            positions: a dictionary of joint names and joint positions

        Returns:
            A numpy ndarray of shape (something, num_joints)

        """
        # Create a chain from the base link to the current link
        chain = self.filter_fixed_joints_from_chain(self.tree.getChain("base_link", link_name))

        jacobian = PyKDL.Jacobian(chain.getNrOfJoints())
        jac_solver = PyKDL.ChainJntToJacSolver(chain)
        jac_solver.JntToJac(self.joints_to_kdl(chain, positions=positions), jacobian)

        return self.kdl_to_np(jacobian)

    def filter_fixed_joints_from_chain(self, chain: PyKDL.Chain) -> PyKDL.Chain:
        """Create a kinematic chain excluding fixed joints.

        Args:
            base_link: Name of the base link
            end_link: Name of the end effector link

        Returns:
            A PyKDL.Chain with only non-fixed joints
        """
        filtered_chain = PyKDL.Chain()

        for i in range(chain.getNrOfSegments()):
            segment = chain.getSegment(i)
            joint = segment.getJoint()

            # Only add segments with non-fixed joints
            if joint.getType() != PyKDL.Joint.Fixed:
                filtered_chain.addSegment(segment)

        return filtered_chain

    def joints_to_kdl(self, chain: PyKDL.Chain, positions: dict = None) -> PyKDL.JntArray:
        """Returns a KDL array of joint positions.

        Args:
            positions: a dictionary of joint names and joint positions

        Returns:
            A KDL Joint Array containing those positions"""
        kdl_array = PyKDL.JntArray(chain.getNrOfJoints())
        for i in range(chain.getNrOfJoints()):
            kdl_array[i] = positions[chain.getSegment(i).getJoint().getName()]
        return kdl_array


def main(args=None):
    rclpy.init(args=args)
    node = JacobianPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
