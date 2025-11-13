"""
kinova.launch.py

Main launch file for Kinova Gen3 Arm with Robotiq 2F 85 gripper

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


def launch_setup(context, *args, **kwargs):

    # Packages to load
    pkg_kortex_bringup = get_package_share_directory("kortex_bringup")
    pkg_gen3_py = get_package_share_directory("gen3_py")

    # Variables
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")

    # Kinova Arm Launch Description
    kinova_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_kortex_bringup, "launch", "gen3.launch.py"])),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
            "robot_ip": robot_ip,
            "gripper": "robotiq_2f_85",
        }.items(),
    )

    ee_publisher = Node(
        package="gen3_py",
        executable="ee_pub",
    )

    nodes_to_launch = [
        kinova_arm_launch,
        # ee_publisher
    ]

    return nodes_to_launch


def generate_launch_description():

    declared_arguments = []

    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use RViz2 for simulation",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="yyy.yyy.yyy.yyy",
            description="ip of robot",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
