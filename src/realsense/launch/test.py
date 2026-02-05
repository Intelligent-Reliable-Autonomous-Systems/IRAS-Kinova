from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="realsense",
            namespace="table_camera",
            parameters=[{
                # Device selection (optional but helpful if multiple cameras)
                # "serial_no": "YOUR_SERIAL",

                # Streams
                "enable_color": True,
                "enable_depth": True,
                "enable_infra1": False,
                "enable_infra2": False,

                # Profiles (examples; pick what you need)
                "rgb_camera.profile": "640x480x30",
                "depth_module.profile": "640x480x30",

                # Useful outputs
                "align_depth.enable": True,
                "pointcloud.enable": True,

                # D435i IMU (enable if you need it)
                "enable_gyro": False,
                "enable_accel": False,

                # Frame naming (optional; wrapper provides a full TF tree)
                # "base_frame_id": "table_camera_link",
            }],
            output="screen",
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # arguments=['-d', 'default.rviz'],
        ),
    ])