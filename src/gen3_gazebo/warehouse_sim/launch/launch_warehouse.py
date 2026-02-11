from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    sim_gazebo = LaunchConfiguration("sim_gazebo")
    spawn_arm = LaunchConfiguration("spawn_arm")
    worlds_file = LaunchConfiguration("worlds_file")

    pkg_warehouseworld = get_package_share_directory("warehouse_world")
    pkg_warehouse_sim = get_package_share_directory("warehouse_sim")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    warehouse_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([pkg_warehouseworld, "models"])
    )

    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])),
        launch_arguments={
            "gz_args": [
                " -r -v 4 ",
                PathJoinSubstitution(
                    [
                        pkg_warehouseworld,
                        "worlds",
                        worlds_file,
                    ]
                ),
            ],
            "on_exit_shutdown": "True",
        }.items(),
        condition=IfCondition(sim_gazebo),
    )

    # Kinova Arm Launch Description
    kinova_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_warehouse_sim, "launch", "spawn_kinova.launch.py"])),
        launch_arguments={}.items(),
        condition=IfCondition(spawn_arm),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    table_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="table_camera_bridge",
        output="screen",
        arguments=[
            "/world/warehouse/model/table_camera/link/camera_link/sensor/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/world/warehouse/model/table_camera/link/camera_link/sensor/rgb_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/world/warehouse/model/table_camera/link/camera_link/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/world/warehouse/model/table_camera/link/camera_link/sensor/rgbd_camera/depth_image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        remappings=[
            (
                "/world/warehouse/model/table_camera/link/camera_link/sensor/rgbd_camera/depth_image",
                "/table_camera/depth/image_raw",
            ),
            (
                "/world/warehouse/model/table_camera/link/camera_link/sensor/rgbd_camera/camera_info",
                "/table_camera/camera_info",
            ),
            (
                "/world/warehouse/model/table_camera/link/camera_link/sensor/rgb_camera/image",
                "/table_camera/image/image_raw",
            ),
            (
                "/world/warehouse/model/table_camera/link/camera_link/sensor/rgbd_camera/depth_image/points",
                "/table_camera/depth/points",
            ),
        ],
    )

    world_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="world_camera_bridge",
        output="screen",
        arguments=[
            "/world/warehouse/model/world_camera/link/camera_link/sensor/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/world/warehouse/model/world_camera/link/camera_link/sensor/rgb_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/world/warehouse/model/world_camera/link/camera_link/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/world/warehouse/model/world_camera/link/camera_link/sensor/rgbd_camera/depth_image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        remappings=[
            (
                "/world/warehouse/model/world_camera/link/camera_link/sensor/rgbd_camera/depth_image",
                "/world_camera/depth/image_raw",
            ),
            (
                "/world/warehouse/model/world_camera/link/camera_link/sensor/rgbd_camera/camera_info",
                "/world_camera/camera_info",
            ),
            (
                "/world/warehouse/model/world_camera/link/camera_link/sensor/rgb_camera/image",
                "/world_camera/image/image_raw",
            ),
            (
                "/world/warehouse/model/world_camera/link/camera_link/sensor/rgbd_camera/depth_image/points",
                "/world_camera/depth/points",
            ),
        ],
    )

    nodes_to_launch = [
        warehouse_resource_path,
        gz_launch_description,
        kinova_arm_launch,
        clock_bridge,
        table_camera_bridge,
        world_camera_bridge,
    ]

    return nodes_to_launch


def generate_launch_description():

    declared_arguments = []

    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="true",
            description="Use Gazebo for simulation",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated clock",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "worlds_file",
            default_value="table.sdf",
            description="World to launch",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_arm",
            default_value="true",
            description="Spawn the Kinova Arm",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
