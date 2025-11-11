# IRAS-Kinova
Kinova Kortex Gen3 arm package for IRAS lab

## Installation Instructions

1. Install ROS2 Jazzy
    Latest LTS Release: [Install ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

2. Clone this repository: 
    ```git@github.com:Intelligent-Reliable-Autonomous-Systems/IRAS-Kinova.git```

3. Navigate to `src/` and clone the ros2_kortex package:

    ```
    git clone https://github.com/Kinovarobotics/ros2_kortex.git src/ros2_kortex
    vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.jazzy.repos
    vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.jazzy.repos
    ```

    Note: may need to install gazebo-sim manually:

    ```
    sudo apt update && sudo apt update
    sudo apt install ros-jazzy-ros-gz
    ```

4. Rename folder that was just created to ros2_kortex:

    ```
    mv src/ ros2_kortex
    ```

5. Navigate to IRAS-Kinova/ folder and Source the ROS2 installation 

    ```
    source opt/ros/jazzy/setup.bash
    ```

6. Install dependencies and build workspace:

    ```
    rosdep install --ignore-src --from-paths src -y -r
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

7. Source the workspace

    ```
    source install/setup.bash
    ```

## Testing the Robot with fake hardware

1. Launch the robot in rviz

    ```
    ros2 launch kinova kinova.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true gripper:=robotiq_2f_85
    ```

2. Send a trajectory command to the robot

    ```
    ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
    points: [
        { positions: [0, 0, 0, 0, 2, 0, 0], time_from_start: { sec: 1 } },
    ]
    }" -1
    ```
