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


def launch_setup(context, *args, **kwargs):
    # Packages to load
    pkg_kortex_bringup = get_package_share_directory("kortex_bringup")
    pkg_kortex_vision = get_package_share_directory("kinova_vision")
    pkg_realsense = get_package_share_directory("iras_realsense")

    # Variables
    use_fake_hardware = LaunchConfiguration("use_fake_hardware", default="true")
    robot_ip = LaunchConfiguration("robot_ip", default="192.168.8.10")
    default_joint_pos = LaunchConfiguration("default_joint_pos")
    use_table_camera = LaunchConfiguration("use_table_camera")
    rviz2 = LaunchConfiguration("rviz2", default="true")

    robot_controllers = PathJoinSubstitution(
        # https://answers.ros.org/question/397123/how-to-access-the-runtime-value-of-a-launchconfiguration-instance-within-custom-launch-code-injected-via-an-opaquefunction-in-ros2/
        [
            FindPackageShare("kortex_description"),
            "arms/gen3/7dof/config",
            "ros2_controllers.yaml",
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("kortex_description"), "robots", "kinova.urdf.xacro"]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
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
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_kortex_bringup, "launch", "gen3.launch.py"])),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
            "robot_ip": robot_ip,
            "gripper": "robotiq_2f_85",
            "vision": "true",
        }.items(),
    )

    kinova_vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_kortex_vision, "launch", "kinova_vision.launch.py"])),
        launch_arguments={
            "device": robot_ip,
        }.items(),
        condition=IfCondition(LaunchConfiguration("vision")),
    )

    # table_camera_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_realsense, "launch", "static_table_depth.py"])),
    #     launch_arguments={
    #         "world_frame": table_camera_world_frame,
    #         "camera_frame": table_camera_frame,
    #         "camera_x": table_camera_x,
    #         "camera_y": table_camera_y,
    #         "camera_z": table_camera_z,
    #         "camera_qx": table_camera_qx,
    #         "camera_qy": table_camera_qy,
    #         "camera_qz": table_camera_qz,
    #         "camera_qw": table_camera_qw,
    #     }.items(),
    #     condition=IfCondition(use_table_camera),
    # )

    table_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [pkg_realsense, "launch", "rgbd_april.py"])),
        condition=IfCondition(use_table_camera),
    )

    ee_publisher = Node(
        package="gen3_py",
        executable="ee_pub",
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
            mappings={
                "use_fake_hardware": use_fake_hardware,
                "robot_ip": robot_ip,
                "gripper": "robotiq_2f_85",
                "gripper_joint_name": "robotiq_85_left_knuckle_joint",
                "dof": "7",
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

    table_scene_node = Node(
        package='iras_viz',
        executable='table_scene',
        name='table_scene_visualizer',
        output='screen',
        condition=IfCondition(rviz2),
    )

    # rosbridge_node = Node(package="rosbridge_server", executable="rosbridge_websocket")

    nodes_to_launch = [
        kinova_arm_launch,
        kinova_vision_launch,
        table_camera_launch,
        move_group_node,
        ee_publisher,
        robot_info_publisher,
        body_pose_publisher,
        jacobian_publisher,
        table_scene_node,
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
