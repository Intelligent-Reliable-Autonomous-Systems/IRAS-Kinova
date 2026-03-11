"""
gen3.launch.py

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
from launch.conditions import IfCondition, UnlessCondition
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


def launch_setup(context, *args, **kwargs):
    # Packages to load
    pkg_realsense = get_package_share_directory("iras_realsense")
    pkg_gen3litepy = get_package_share_directory("gen3lite_py")

    # Variables
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    default_joint_pos = LaunchConfiguration("default_joint_pos")
    use_table_camera = LaunchConfiguration("use_table_camera")
    table_camera_world_frame = LaunchConfiguration("table_camera_world_frame")
    table_camera_frame = LaunchConfiguration("table_camera_frame")
    table_camera_x = LaunchConfiguration("table_camera_x")
    table_camera_y = LaunchConfiguration("table_camera_y")
    table_camera_z = LaunchConfiguration("table_camera_z")
    table_camera_qx = LaunchConfiguration("table_camera_qx")
    table_camera_qy = LaunchConfiguration("table_camera_qy")
    table_camera_qz = LaunchConfiguration("table_camera_qz")
    table_camera_qw = LaunchConfiguration("table_camera_qw")
    launch_kortex_rviz = LaunchConfiguration("launch_kortex_rviz")
    rviz_config_file_name = LaunchConfiguration("rviz_config_file")
    robot_controller = LaunchConfiguration("robot_controller")
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("kortex_description"),
            "arms/gen3_lite/config",
            "ros2_controllers.yaml",
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("kortex_description"), "robots", "kinova.urdf.xacro"]),
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
            "gripper": "gen3_lite_2f",
            "vision": "true",
            "launch_rviz": launch_kortex_rviz,
            "robot_controller": robot_controller,
        }.items(),
    )

    table_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_realsense, "launch", "rgbd_april.py"])),
        condition=IfCondition(use_table_camera),
    )

    table_scene_node = Node(
        package='iras_viz',
        executable='table_scene',
        name='table_scene_visualizer',
        output='screen',
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
            mappings={
                "use_fake_hardware": use_fake_hardware,
                "robot_ip": robot_ip,
                "gripper": "gen3_lite_2f",
                "gripper_joint_name": "right_finger_bottom_joint",
                "dof": "6",
                "gripper_max_velocity": "100",
                "gripper_max_force": "100",
                "use_internal_bus_gripper_comm": "true",
            }
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
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

    rviz_config_file = PathJoinSubstitution([pkg_gen3litepy, "rviz", rviz_config_file_name])

    rviz_node = Node(
        package="rviz2",
        condition=UnlessCondition(launch_kortex_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # rosbridge_node = Node(package="rosbridge_server", executable="rosbridge_websocket")

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
        # rosbridge_node,
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
    declared_arguments.append(DeclareLaunchArgument("table_camera_world_frame", default_value="world"))
    declared_arguments.append(DeclareLaunchArgument("table_camera_frame", default_value="table_camera_link"))
    declared_arguments.append(DeclareLaunchArgument("table_camera_x", default_value="0.0"))
    declared_arguments.append(DeclareLaunchArgument("table_camera_y", default_value="0.0"))
    declared_arguments.append(DeclareLaunchArgument("table_camera_z", default_value="0.0"))
    declared_arguments.append(DeclareLaunchArgument("table_camera_qx", default_value="0.0"))
    declared_arguments.append(DeclareLaunchArgument("table_camera_qy", default_value="0.0"))
    declared_arguments.append(DeclareLaunchArgument("table_camera_qz", default_value="0.0"))
    declared_arguments.append(DeclareLaunchArgument("table_camera_qw", default_value="1.0"))
    declared_arguments.append(
        DeclareLaunchArgument("launch_kortex_rviz", default_value="false", description="Launch Kortex RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file", default_value="view_robot.rviz", description="Name of RViz file in pkg_gen3litepy/rviz/"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
