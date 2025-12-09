"""
gen3_skills.skills.skills.py

Available skills for gen3 manipulation

Written by Will Solow, 2025
"""

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState



def go_to_xyz(x: float=None, y: float=None, z: float=None, curr_state: JointState=None, **kwargs) -> JointTrajectory:
    """
    Go to a location specified by xyz and mantain end effector orientation
    """
    pass


def go_to_xy(x: float=None, y: float=None, curr_state: JointState=None, **kwargs) -> JointTrajectory:
    """
    Go to a location specified by xy and mantain end effector orientation
    """
    pass