# IRAS-Kinova
Kinova Kortex Gen3 arm package for IRAS lab

## Setting up robot with new laptop

1. Connect robot ethernet cable to computer ethernet port
2. Open Settings -> Network -> Wired Connection -> Properties 
3. Select IPv4. Set IP address to 192.168.8.XX where XX > 10. Set Netmask to 255.255.255.0. Hit apply.
4. Unplug/replug ethernet cable from computer.
5. Open browser to 192.168.8.10 (Gen3 IPv4 address). This opens the Kinova WebApp. Username/password is admin/admin.
6. See [Kinova Reference Guide](https://www.kinovarobotics.com/uploads/User-Guide-Gen3-R07.pdf) for more information.

# IRAS Alienware laptop

1. Plug in Ethernet cable directly to laptop using port on left
2. Robot is on 192.168.8.10, wired IP address must be set 192.168.8.11 (or greater than 10). This is sometimes finicky
3. Check that control can be done in Gen3 Web App (Open FireFox and go to 192.168.8.10). Username/password is admin/admin
4. If all this works, we can launch the robot: `ros2 launch gen3_py gen3.launch.py robot_ip:=192.168.8.10 use_fake_hardware:=false gripper:=robotiq_2f_85`


# Gazebo Installation
Gazebo can be really finicky. It does not support mimic joints, so the Kinova Gen3 arm is brought up without the Robotiq 2F85 gripper. 
Ensure that Ogre2 is installed (no Ogre 1.9). The language models can help with debugging this.


## Installation Instructions

1. Install ROS2 Jazzy
    Latest LTS Release: [Install ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
    Install rosdep and colcon 
    Recommend installing to global (not in venv)

2. Clone this repository: 
    ```git@github.com:Intelligent-Reliable-Autonomous-Systems/IRAS-Kinova.git```

3. Clone the ros2_kortex package and install dependencies:

    ```
    git clone https://github.com/Kinovarobotics/ros2_kortex.git src/third_party/ros2_kortex/ros2_kortex
    vcs import src/third_party/ros2_kortex --skip-existing --input src/third_party/ros2_kortex/ros2_kortex/ros2_kortex.jazzy.repos
    vcs import src/third_party/ros2_kortex --skip-existing --input src/third_party/ros2_kortex/ros2_kortex/ros2_kortex-not-released.jazzy.repos
    ```

4. Clone the ros2_kortex_vision package and kdl_parser_py package:
    ```
    git clone https://github.com/Kinovarobotics/ros2_kortex_vision.git src/third_party/ros2_kortex_vision
    git clone https://github.com/ros/kdl_parser_py.git src/third_party/kdl_parser_py
    ```

5. Source the ROS2 installation from the home directory

    ```
    source opt/ros/jazzy/setup.bash
    ```

6. Install dependencies and build workspace:

    ```
    rosdep install --ignore-src --from-paths src -y -r
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    ```

7. Source the workspace

    ```
    source install/setup.bash
    ```

## Testing the Robot with fake hardware

1. Launch the robot in rviz

    ```
    ros2 launch gen3_py gen3.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true gripper:=robotiq_2f_85 vision:=false
    ```

2. Send a trajectory command to the robot

    ```
    ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
    points: [
        { positions: [0, 0.523599, 0, 1.5708, 0, .785398, 0], time_from_start: { sec: 5 } },
    ]
    }" -1
    ```

    ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
    points: [
        { positions: [0, 0.0, 0, 0.0, 0, .0, 0], time_from_start: { sec: 5 } },
    ]
    }" -1

    ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [
        { positions: [0.2, -0.18, 2.16, -1.57, -0.6, -1.34], time_from_start: { sec: 5 } },
    ]
    }" -1

    

3. Try resetting the robot

    ```
    ros2 run gen3_cpp gen3_reset --ros-args -p move_time:=5.0
    ```


## Testing Sim to Sim with a Reach Policy


1. Launch the robot in rviz

    ```
    ros2 launch gen3_py gen3.launch.py robot_ip:=192.168.10.yyy use_fake_hardware:=true gripper:=robotiq_2f_85
    ```

2. OR launch the robot in Gazebo

    ```
    ros2 launch warehouse_sim launch_warehouse.py
    ```

3. Launch the policy

    ```
    ros2 run gen3_controllers gen3_reach
    ```

## Testing Sim to Real with a Reach Policy

1. Launch the robot hardware

    ```
    ros2 launch gen3_py gen3.launch.py robot_ip:=192.168.8.10 use_fake_hardware:=false gripper:=robotiq_2f_85
    ```


## Testing Sim to IsaacSim

1. Launch the robot from the Isaac-Kinova Repository (see instructions)

2. Launch the policy

    ```
    ros2 run gen3_controllers gen3_reach_isaac
    ```



ros2 topic pub -r 30 servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{twist: {linear: {z: 0.01}}}"
ros2 control switch_controllers --activate forward_position_controller --deactivate joint_trajectory_controller
ros2 control switch_controllers --activate joint_trajectory_controller --deactivate forward_position_controller
ros2 control switch_controllers --activate forward_velocity_controller --deactivate joint_trajectory_controller


ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"

