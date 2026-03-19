"""
gen3.launch.py

Main launch file for Kinova Gen3 Arm with Robotiq 2F 85 gripper

Written by Will Solow, 2025. IRAS Lab.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Packages to load
    pkg_kortex_vision = get_package_share_directory("kinova_vision")
    pkg_realsense = get_package_share_directory("iras_realsense")
    pkg_gen3py = get_package_share_directory("gen3_py")
    pkg_description = get_package_share_directory("kortex_description")

    # Variables
    use_fake_hardware = LaunchConfiguration("use_fake_hardware", default="true")
    robot_ip = LaunchConfiguration("robot_ip", default="192.168.8.10")
    default_joint_pos = LaunchConfiguration("default_joint_pos")
    use_table_camera = LaunchConfiguration("use_table_camera")
    rviz2 = LaunchConfiguration("rviz2", default="true")
    robot_controller = LaunchConfiguration("robot_controller")
    rviz_config_file_name = LaunchConfiguration("rviz_config_file")
    launch_rviz = LaunchConfiguration("launch_rviz")

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("gen3_py"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    robot_urdf = PathJoinSubstitution([pkg_gen3py, "robot", "gen3_2f85.urdf"])

    moveit_controllers = Path(get_package_share_directory("gen3_py")) / "config" / "moveit_controllers.yaml"

    robot_path = Path(get_package_share_directory("gen3_py")) / "robot" / "gen3.xacro"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("gen3_py"), "robot", "kinova.urdf.xacro"]),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "name:=gen3",
            " ",
            "arm:=gen3",
            " ",
            "dof:=7",
            " ",
            "vision:=true",
            " ",
            "prefix:=''",
            " ",
            "sim_gazebo:=false",
            " ",
            "simulation_controllers:=",
            robot_controllers,
            " ",
            "gripper:=robotiq_2f_85",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Kinova Arm Launch Description
    kinova_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_gen3py, "launch", "kortex_gen3.launch.py"])),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
            "robot_ip": robot_ip,
            "gripper": "robotiq_2f_85",
            "vision": "true",
            "robot_controller": robot_controller,
        }.items(),
    )

    kinova_vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_kortex_vision, "launch", "kinova_vision.launch.py"])),
        launch_arguments={
            "device": robot_ip,
        }.items(),
        condition=IfCondition(LaunchConfiguration("vision")),
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

    robot_info_publisher = Node(
        package="gen3_py",
        executable="robot_info",
        parameters=[
            {
                "robot_description": robot_description_content,
                "default_joint_pos": default_joint_pos,
                "traj_duration_sec": 0 if use_fake_hardware else 8,
            }
        ],
    )

    body_pose_publisher = Node(package="gen3_py", executable="body_pose", parameters=[robot_description])

    jacobian_publisher = Node(package="gen3_py", executable="jacobian_pub", parameters=[robot_description])

    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config")
        .robot_description(
            file_path=robot_path,
            mappings={
                "use_fake_hardware": use_fake_hardware,
                "robot_ip": robot_ip,
                "gripper": "robotiq_2f_85",
                "gripper_joint_name": "robotiq_85_left_knuckle_joint",
                "dof": "7",
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

    servo_params = {"moveit_servo": ParameterBuilder("gen3_py").yaml("config/servo.yaml").to_dict()}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    rviz_config_file = PathJoinSubstitution([pkg_gen3py, "rviz", rviz_config_file_name])

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    twist_integrator = Node(
        package="gen3_py",
        executable="twist_integrator",
        parameters=[
            {
                "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"],
                "initial_positions": [0.0, 0.523599, 0.0, 1.5708, 0.0, 0.785398, 0.0],
                "robot_description": robot_description_content,
            }
        ],
        condition=IfCondition(use_fake_hardware),
    )

    twist_pause = Node(
        package="gen3_py",
        executable="twist_watch",
    )
    safety_filter = Node(
        package="gen3_py",
        executable="safety_filter",
        parameters=[{"urdf_filename": robot_urdf}],
    )

    nodes_to_launch = [
        kinova_arm_launch,
        twist_integrator,
        kinova_vision_launch,
        table_camera_launch,
        servo_node,
        move_group_node,
        robot_info_publisher,
        body_pose_publisher,
        jacobian_publisher,
        table_scene_node,
        twist_pause,
        safety_filter,
        rviz_node,
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
            default_value="192.168.18.10",
            description="ip of robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "vision",
            default_value="false",
            description="If to load vision topics",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "default_joint_pos",
            default_value="[0.0, 0.523599, 0.0, 1.5708, 0.0, 0.785398, 0.0]",
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
            "rviz_config_file", default_value="view_robot_camera.rviz", description="Name of RViz file in gen3_py/rviz/"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
