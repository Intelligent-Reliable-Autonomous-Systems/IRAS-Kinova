# IRAS-Kinova
Kinova Kortex Gen3 arm package for IRAS lab

## Setting up robot with new laptop

1. Connect robot ethernet cable to computer ethernet port
2. Open Settings -> Network -> Wired Connection -> Properties 
3. Select IPv4. Set IP address to 192.168.8.XX where XX > 10. Set Netmask to 255.255.255.0. Hit apply.
4. Unplug/replug ethernet cable from computer.
5. Open browser to 192.168.8.10 (Gen3 IPv4 address). This opens the Kinova WebApp. Username/password is admin/admin.
6. See [Kinova Reference Guide](https://www.kinovarobotics.com/uploads/User-Guide-Gen3-R07.pdf) for more information.

## Installation Instructions

1. Install ROS2 Jazzy
    Latest LTS Release: [Install ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
    Install rosdep and colcon 
    Recommend installing to global (not in venv)

2. Clone this repository: 
    ```git@github.com:Intelligent-Reliable-Autonomous-Systems/IRAS-Kinova.git```

3. Navigate to `src/` and clone the ros2_kortex package:

    ```
    git clone https://github.com/Kinovarobotics/ros2_kortex.git
    vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.jazzy.repos
    vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.jazzy.repos
    ```

4. Clone the ros2_kortex_vision package:
    ```
    git clone https://github.com/Kinovarobotics/ros2_kortex_vision.git
    ```

5. Navigate to IRAS-Kinova/ folder and Source the ROS2 installation 

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
    ros2 launch gen3_py gen3.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true gripper:=robotiq_2f_85
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
        { positions: [0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 2 } },
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

2. Launch the policy

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