import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

ARM_JOINTS = [
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6", "joint_7"
]

class DramaticDance(Node):

    def __init__(self):
        super().__init__("dramatic_dance")

        self.pub = self.create_publisher(
            JointTrajectory,
            "/collision_policy",
            10
        )

        self.t = 0.0
        self.dt = 0.1   # slow and smooth
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info("🌸 Dramatic dance started...")

    def update(self):

        # slow time scale
        speed = 0.5
        self.t += self.dt * speed

        # graceful base sway
        base = 0.5 * math.sin(self.t)

        # shoulder lift wave
        shoulder = 0.45 + 0.15 * math.sin(self.t * 0.5)

        # elbow breathing motion
        elbow = 0.4 * math.sin(self.t * 0.7)

        # forearm stable but soft motion
        forearm = 1.1 + 0.1 * math.sin(self.t * 0.3)

        # fan arc (wrist bend)
        wrist_bend = 0.5 * math.sin(self.t)

        # elegant wrist rotation
        wrist_roll = 0.7 * math.sin(self.t * 0.8)

        tool = 0.0

        positions = [
            base,
            shoulder,
            elbow,
            forearm,
            wrist_bend,
            wrist_roll,
            tool
        ]

        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=1)

        msg.points.append(point)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DramaticDance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()