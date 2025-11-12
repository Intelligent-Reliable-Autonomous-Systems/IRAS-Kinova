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
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    ```

7. Source the workspace

    ```
    source install/setup.bash
    ```

## Testing the Robot with fake hardware

1. Launch the robot in rviz

    ```
    ros2 launch kinova_py gen3.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true gripper:=robotiq_2f_85
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

3. Try resetting the robot

    ```
    ros2 run kinova_cpp kinova_reset --ros-args -p move_time:=5


## Testing Sim to Real with a Reach Policy


1. Launch the robot in rviz

    ```
    ros2 launch kinova kinova.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true gripper:=robotiq_2f_85
    ```

2. Launch the policy

    ```
    python3 sim2real/scripts/sim2real/run_task_reach.py
    ```
