import time
import rclpy
from rclpy.logging import get_logger
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand
from moveit.planning import MoveItPy

BASE_FRAME = "base_link"
END_EFFECTOR_LINK = "end_effector_link"
ARM_GROUP = "manipulator"

TABLE_SIZE = [0.8, 0.8, 0.05]
TABLE_POS  = [0.45, 0.0, -0.03]
CUBE_SIZE  = [0.05, 0.05, 0.05]
CUBE_POS   = [0.45, 0.0, 0.20]
PLACE_DROP_POS = [0.45, -0.25, 0.20]
GRIPPER_OPEN   = 0.0
GRIPPER_CLOSED = 0.8
PRE_GRASP_POSE = [0.45,  0.0,   0.38,  0.0, 1.0, 0.0, 0.0]
GRASP_POSE = [0.45,  0.0,   0.22,  0.0, 1.0, 0.0, 0.0]
LIFT_POSE = [0.45,  0.0,   0.45,  0.0, 1.0, 0.0, 0.0]
PRE_PLACE_POSE = [0.45, -0.25,  0.38,  0.0, 1.0, 0.0, 0.0]
PLACE_POSE = [0.45, -0.25,  0.22,  0.0, 1.0, 0.0, 0.0]
RETREAT_POSE = [0.45, -0.25,  0.45,  0.0, 1.0, 0.0, 0.0]


def make_pose_stamped(frame_id, xyzq):
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x = xyzq[0]
    ps.pose.position.y = xyzq[1]
    ps.pose.position.z = xyzq[2]
    ps.pose.orientation.x = xyzq[3]
    ps.pose.orientation.y = xyzq[4]
    ps.pose.orientation.z = xyzq[5]
    ps.pose.orientation.w = xyzq[6]
    return ps


def make_box_collision_object(obj_id, frame_id, dimensions, position):
    co = CollisionObject()
    co.header.frame_id = frame_id
    co.id = obj_id
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = dimensions
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.w = 1.0
    co.primitives.append(box)
    co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD
    return co


def plan_and_execute(robot, planning_component, logger, sleep_time=2.0):
    logger.info("Planning ...")
    plan_result = planning_component.plan()
    if plan_result:
        logger.info("Plan found, executing ...")
        robot.execute(plan_result.trajectory, controllers=[])
        time.sleep(sleep_time)
        return True
    logger.error("Planning FAILED")
    return False


def add_scene_objects(logger):
    logger.info("Adding table and cube to planning scene ...")
    temp_node = rclpy.create_node("scene_publisher")
    pub = temp_node.create_publisher(CollisionObject, "/collision_object", 10)
    time.sleep(1.5)
    pub.publish(make_box_collision_object("table", BASE_FRAME, TABLE_SIZE, TABLE_POS))
    time.sleep(0.5)
    pub.publish(make_box_collision_object("cube", BASE_FRAME, CUBE_SIZE, CUBE_POS))
    time.sleep(0.5)
    temp_node.destroy_node()
    logger.info("Scene objects added")


def control_gripper(logger, position):
    label = "Opening" if position < 0.1 else "Closing"
    logger.info(f"{label} gripper ...")
    temp_node = rclpy.create_node("gripper_controller")
    executor = SingleThreadedExecutor()
    executor.add_node(temp_node)
    client = ActionClient(temp_node, GripperCommand, "/robotiq_gripper_controller/gripper_cmd")
    timeout = 5.0
    start = time.time()
    while not client.server_is_ready():
        executor.spin_once(timeout_sec=0.1)
        if time.time() - start > timeout:
            logger.error("Gripper action server not available!")
            temp_node.destroy_node()
            return False
    goal = GripperCommand.Goal()
    goal.command.position   = position
    goal.command.max_effort = 50.0
    send_future = client.send_goal_async(goal)
    while not send_future.done():
        executor.spin_once(timeout_sec=0.1)
    goal_handle = send_future.result()
    if not goal_handle.accepted:
        logger.error("Gripper goal rejected!")
        temp_node.destroy_node()
        return False
    result_future = goal_handle.get_result_async()
    while not result_future.done():
        executor.spin_once(timeout_sec=0.1)
    temp_node.destroy_node()
    time.sleep(0.5)
    logger.info("Gripper command done")
    return True


def attach_cube(logger):
    logger.info("Attaching cube to end-effector ...")
    temp_node = rclpy.create_node("attach_publisher")
    pub = temp_node.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
    time.sleep(0.5)
    aco = AttachedCollisionObject()
    aco.link_name = END_EFFECTOR_LINK
    aco.object = make_box_collision_object("cube", BASE_FRAME, CUBE_SIZE, CUBE_POS)
    aco.object.operation = CollisionObject.ADD
    aco.touch_links = [END_EFFECTOR_LINK]
    pub.publish(aco)
    time.sleep(0.5)
    temp_node.destroy_node()
    logger.info("Cube attached")


def detach_and_place_cube(logger):
    logger.info("Detaching and placing cube ...")
    temp_node = rclpy.create_node("detach_publisher")
    attach_pub = temp_node.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
    scene_pub  = temp_node.create_publisher(CollisionObject, "/collision_object", 10)
    time.sleep(0.5)
    aco = AttachedCollisionObject()
    aco.object.id = "cube"
    aco.link_name = END_EFFECTOR_LINK
    aco.object.operation = CollisionObject.REMOVE
    attach_pub.publish(aco)
    time.sleep(0.3)
    scene_pub.publish(make_box_collision_object("cube", BASE_FRAME, CUBE_SIZE, PLACE_DROP_POS))
    time.sleep(0.3)
    temp_node.destroy_node()
    logger.info("Cube placed")


def move_to(robot, arm, logger, xyzq, label="waypoint"):
    logger.info(f"Moving to {label}")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=make_pose_stamped(BASE_FRAME, xyzq), pose_link=END_EFFECTOR_LINK)
    return plan_and_execute(robot, arm, logger)


def main():
    rclpy.init()
    logger = get_logger("pick_and_place")
    add_scene_objects(logger)
    logger.info("Initialising MoveItPy ...")
    robot = MoveItPy(node_name="pick_and_place")
    arm = robot.get_planning_component(ARM_GROUP)
    logger.info("MoveItPy ready")
    logger.info("Waiting for joint states to stabilise ...")
    time.sleep(5.0)
    logger.info("=== Starting pick-and-place sequence ===")
    #open gripper
    control_gripper(logger, GRIPPER_OPEN)
    #move above cube
    if not move_to(robot, arm, logger, PRE_GRASP_POSE, "pre-grasp"):
        rclpy.shutdown(); return
    #descend to cube
    if not move_to(robot, arm, logger, GRASP_POSE, "grasp"):
        rclpy.shutdown(); return
    #close gripper
    control_gripper(logger, GRIPPER_CLOSED)
    #attach cube in planning scene
    attach_cube(logger)
    #lift up
    if not move_to(robot, arm, logger, LIFT_POSE, "lift"):
        rclpy.shutdown(); return
    #move above place location
    if not move_to(robot, arm, logger, PRE_PLACE_POSE, "pre-place"):
        rclpy.shutdown(); return
    #descend to place
    if not move_to(robot, arm, logger, PLACE_POSE, "place"):
        rclpy.shutdown(); return
    #open gripper to release
    control_gripper(logger, GRIPPER_OPEN)
    #detach cube and show at new location
    detach_and_place_cube(logger)
    #back up
    if not move_to(robot, arm, logger, RETREAT_POSE, "retreat"):
        rclpy.shutdown(); return
    time.sleep(2.0)
    robot.__del__()
    time.sleep(1.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()