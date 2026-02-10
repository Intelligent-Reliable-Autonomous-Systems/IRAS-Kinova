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

#define the limits for the arm joints
MAX_VELOCITY = 2.0 #rad/s
MAX_ACCELERATION = 5.0 #rad/s^2

def safety_arm_check(current_joint_positions, joint_pos, current_joint_velocities, step_size):
    """Checks if the joint implied velocity adn acceleration are within a safe range,
    which implies that the torque is in the safe range and prevents the hardware shoutdown."""

    for i in range(len(joint_pos)-1):
        implied_velocity = (joint_pos[i] - current_joint_positions[i]) / step_size
        implied_acceleration = (implied_velocity - current_joint_velocities[i]) / step_size
        if abs(implied_velocity) > MAX_VELOCITY or abs(implied_acceleration) > MAX_ACCELERATION:
            return False
    return True
