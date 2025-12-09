"""
gen3_skills.launch.py

Main launch file for all skills

Written by Will Solow, 2025. IRAS Lab.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)

from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):


    skills_manager = Node(
        package="gen3_skills",
        executable="skills_manager",
    )

    skills = Node(
        package="gen3_skills",
        executable="skills"
    )

    nodes_to_launch = [
        skills_manager,
        skills
    ]

    return nodes_to_launch


def generate_launch_description():

    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
