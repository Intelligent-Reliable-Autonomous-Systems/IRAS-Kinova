"""
utils.py

Contains extra code for Gen3 skills

Written by Will Solow, 2025"""

ARM_JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
]

GRIPPER_JOINTS = [
    "robotiq_85_left_finger_tip_joint",
    "robotiq_85_left_inner_knuckle_joint",
    "robotiq_85_left_knuckle_joint",
    "robotiq_85_right_finger_tip_joint",
    "robotiq_85_right_inner_knuckle_joint",
    "robotiq_85_right_knuckle_joint",
]

GRIPPER_CTRL_JOINT = "robotiq_85_left_knuckle_joint"
GRIPPER_CTRL_JOINT_ID = 9

GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 0.8

#Temporarily here: load the limits for joints and their velocity from URDF file
from urdf_parser_py.urdf import URDF 

def read_joint_limits(filepath):
    robot = URDF.from_xml_file(filepath)
    joint_limits = {}

    for joint in robot.joints:
        if joint.type in ["revolute", "continuous", "prismatic"] and joint.limit:
            name = normalize_urdf_name(joint.name)
            joint_limits[name] = {
                "lower": joint.limit.lower,
                "upper": joint.limit.upper,
                "effort": joint.limit.effort,
                "velocity": joint.limit.velocity,
            }
    return joint_limits

#to avoid joint names mismatch
def normalize_urdf_name(urdf_name):
    if urdf_name.startswith("gen3_"):
        return urdf_name.replace("gen3_", "")
    else:
        return urdf_name


def safety_arm_check(current_joint_positions, joint_pos, current_joint_velocities, step_size):
    """Checks if the joint implied velocity adn acceleration are within a safe range,
    which implies that the torque is in the safe range and prevents the hardware shoutdown."""

    joint_limits = read_joint_limits("/home/iras/IRAS-Kinova/src/third_party/ros2_kortex/ros2_kortex/kortex_description/robots/gen3_2f85.urdf")
    

    for i, joint_name in enumerate(ARM_JOINTS):
        
        velocity_lim = joint_limits[joint_name]["velocity"]
        lower_lim = joint_limits[joint_name]["lower"]
        upper_lim = joint_limits[joint_name]["upper"]
        print(f"Upper limit: {upper_lim}")
        print(f"Lower limit: {lower_lim}")
        print(f"Velocity limit: {velocity_lim}")
        #dt = step_size in that case
        implied_velocity = (joint_pos[i] - current_joint_positions[i]) / step_size
        if abs(implied_velocity) > velocity_lim or joint_pos[i] < lower_lim or joint_pos[i] > upper_lim:
            return False
        
    return True

# if __name__ == "__main__":
#     import numpy as np

#     print("Running safety_arm_check test...")

#     current_pos = np.zeros(7)
#     current_vel = np.zeros(7)
#     step = 0.02

#     joint_pos = np.array([0, 0.3, 500, 0.3, 5, 0.3, 0])
#     result = safety_arm_check(current_pos, joint_pos, current_vel, step)

#     print("Test result:", result)