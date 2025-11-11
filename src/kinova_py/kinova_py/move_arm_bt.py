#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import py_trees

# === Behavior for sending a trajectory ===
class MoveArm(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node, name: str, joint_positions, move_time: float):
        super().__init__(name)
        self.node = node
        self.joint_positions = joint_positions
        self.move_time = move_time
        self.sent = False

    def initialise(self):
        self.node.get_logger().info(f"Initialising {self.name} behavior")

    def update(self):
        if not self.sent:
            msg = JointTrajectory()
            msg.joint_names = [
                'joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7'
            ]
            point = JointTrajectoryPoint()
            point.positions = self.joint_positions
            point.time_from_start.sec = int(self.move_time)
            msg.points.append(point)

            self.node.publisher.publish(msg)
            self.node.get_logger().info(f"{self.name}: sent {self.joint_positions}")
            self.sent = True
            return py_trees.common.Status.RUNNING

        # You could add feedback or wait logic here.
        # For simplicity, we'll assume it succeeds after one tick.
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.SUCCESS:
            self.node.get_logger().info(f"{self.name} completed successfully.")


# === Node that runs the BT ===
class KinovaBTNode(Node):
    def __init__(self):
        super().__init__('kinova_bt_node')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Define behaviors
        home_positions   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        target_positions = [1.0, 0.5, 0.0, 0.0, 0.5, 0.0, 0.0]

        move_home   = MoveArm(self, "MoveToHome",   home_positions,   3.0)
        move_target = MoveArm(self, "MoveToTarget", target_positions, 4.0)

        # Build a simple sequence tree
        self.root = py_trees.composites.Sequence("KinovaSequence", memory=False)
        self.root.add_children([move_home, move_target])
        self.tree = py_trees.trees.BehaviourTree(self.root)

        # Tick the tree periodically
        self.timer = self.create_timer(1.0, self.tick_tree)
        self.get_logger().info("Behavior Tree started")

    def tick_tree(self):
        self.tree.tick()
        self.get_logger().info(f"Tree status: {self.root.status}")


def main(args=None):
    rclpy.init(args=args)
    node = KinovaBTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
