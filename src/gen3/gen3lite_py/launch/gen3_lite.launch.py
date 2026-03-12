"""
gen3.launch.py

Main launch file for Kinova Gen3 Arm with Robotiq 2F 85 gripper

Written by Will Solow, 2025. IRAS Lab.
"""

import os
from pathlib import Path
from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


def launch_setup(context, *args, **kwargs):
    # Packages to load
    pkg_realsense = get_package_share_directory("iras_realsense")
    pkg_gen3litepy = get_package_share_directory("gen3lite_py")

    # Variables
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    default_joint_pos = LaunchConfiguration("default_joint_pos")
    use_table_camera = LaunchConfiguration("use_table_camera")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file_name = LaunchConfiguration("rviz_config_file")
    robot_controller = LaunchConfiguration("robot_controller")

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("gen3lite_py"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    moveit_controllers = Path(get_package_share_directory("gen3lite_py")) / "config" / "moveit_controllers.yaml"

    robot_path = Path(get_package_share_directory("gen3lite_py")) / "robot" / "gen3_lite_gen3_lite_2f.xacro"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("gen3lite_py"), "robot", "kinova.urdf.xacro"]),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "name:=gen3_lite",
            " ",
            "arm:=gen3_lite",
            " ",
            "dof:=6",
            " ",
            "prefix:=''",
            " ",
            "sim_gazebo:=false",
            " ",
            "simulation_controllers:=",
            robot_controllers,
            " ",
            "gripper:=gen3_lite_2f",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Kinova Arm Launch Description
    kinova_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_gen3litepy, "launch", "kortex_gen3_lite.launch.py"])),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
            "robot_ip": robot_ip,
            "vision": "true",
            "robot_controller": robot_controller,
        }.items(),
    )

    table_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_realsense, "launch", "rgbd_april.py"])),
        condition=IfCondition(use_table_camera),
    )

    table_scene_node = Node(
        package="iras_viz",
        executable="table_scene",
        name="table_scene_visualizer",
        output="screen",
        # condition=IfCondition(rviz2),
    )

    ee_publisher = Node(
        package="gen3_py",
        executable="ee_pub",
        parameters=[
            {
                "base_frame": "base_link",
                "ee_frame": "end_effector_link",
            }
        ],
    )

    robot_info_publisher = Node(
        package="gen3_py",
        executable="robot_info",
        parameters=[
            {
                "robot_description": robot_description_content,
                "default_joint_pos": default_joint_pos,
                "traj_duration_sec": 0 if use_fake_hardware else 8,
                "joint_names": [
                    "joint_1",
                    "joint_2",
                    "joint_3",
                    "joint_4",
                    "joint_5",
                    "joint_6",
                ],
            }
        ],
    )

    body_pose_publisher = Node(package="gen3_py", executable="body_pose", parameters=[robot_description])

    jacobian_publisher = Node(package="gen3_py", executable="jacobian_pub", parameters=[robot_description])

    moveit_config = (
        MoveItConfigsBuilder("gen3lite", package_name="kinova_gen3_lite_moveit_config")
        .robot_description(
            file_path=robot_path,
            mappings={
                "use_fake_hardware": use_fake_hardware,
                "robot_ip": robot_ip,
                "gripper": "gen3_lite_2f",
                "gripper_joint_name": "right_finger_bottom_joint",
                "dof": "6",
                "gripper_max_velocity": "100",
                "gripper_max_force": "100",
                "use_internal_bus_gripper_comm": "true",
            },
        )
        .trajectory_execution(file_path=moveit_controllers)
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    servo_params = {"moveit_servo": ParameterBuilder("gen3lite_py").yaml("config/servo.yaml").to_dict()}

    # Add a print to verify the path is correct

    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"moveit_servo.planning_group_name": "arm"}
    planning_group_name = {"moveit_servo.move_group_name": "arm"}

    print("######### SERVO PARAMS##############")
    print(servo_params)
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    rviz_config_file = PathJoinSubstitution([pkg_gen3litepy, "rviz", rviz_config_file_name])

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    delay_rviz_after_servo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=servo_node,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    nodes_to_launch = [
        kinova_arm_launch,
        table_camera_launch,
        table_scene_node,
        move_group_node,
        ee_publisher,
        robot_info_publisher,
        body_pose_publisher,
        jacobian_publisher,
        rviz_node,
        # servo_node
    ]

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []

    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="If to only launch the fake hardware",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.10",
            description="ip of robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "default_joint_pos",
            default_value="[0.12, -0.18, 2.16, -1.57, -0.6, -1.34]",
            description="Default joint positions of the robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            description="Name of robot controller to start",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_table_camera",
            default_value="false",
            description="If to launch table camera and RGB-D snapshot service",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch Kortex RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file", default_value="view_robot.rviz", description="Name of RViz file in pkg_gen3litepy/rviz/"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
