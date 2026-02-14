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
