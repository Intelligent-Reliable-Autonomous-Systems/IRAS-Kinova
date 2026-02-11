### Installation Guide

1. `mkdir ros2_ws/src -p`
2. `cd ros2_ws/src`
3. `git clone git@github.com:Intelligent-Reliable-Autonomous-Systems/ROS-Warehouse-Demo.git`
4. `vcs import < ros2_ws.repos`
5. `cd ..`
6. `rosdep install --from-paths src --ignore-src -r -y`
7. `sudo apt-get install ros-jazzy-ros-gz && sudo apt-get install ros-jazzy-clearpath-simulator`
8. `colcon build --executor sequential --allow-overriding kortex_description`
9. `source install/local_setup.bash `

### Launch simulation (without apriltag)
- `ros2 launch warehouse_sim launch_warehouse.py`
   
### Launch simulation (with apriltag)
- `ros2 launch warehouse_sim launch_warehouse_with_apriltags.py     # Terminal 1`
- `python3 ~/ros2_ws/src/scripts/timed_twist_sequence.py     # Terminal 2`

### To test movement of Jackal

```
ros2 topic pub /j100_0000/cmd_vel geometry_msgs/msg/TwistStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
    twist: {linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}
    }
}"
```

### To run the Pick and Place task on the Kinova Arm

`ros2 run kinova_pick_place block_pick_place`

### To test movement of Kinova Arm

```
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [kinova/joint_1, kinova/joint_2, kinova/joint_3, kinova/joint_4, kinova/joint_5, kinova/joint_6],
  points: [
    { positions: [2.23, 1.67, -0.0, .9, -1.2, 1], time_from_start: { sec: 2 } },
  ]
}"
```

### To test movement of Kinova Gripper

`ros2 action send_goal gen3_lite_2f_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.0, max_effort: 100.0}}"`


### To view data in Rviz2 with Kinova/Jackal

`ros2 run rviz2 rviz2 --ros-args -r /tf:=/kinova/tf`

`ros2 run rviz2 rviz2 --ros-args -r /tf:=/j100_0000/tf`

### To echo tf frames
`ros2 run tf2_tools view_frames --ros-args -r /tf:=/kinova/tf`

`ros2 run tf2_tools view_frames --ros-args -r /tf:=/j100_0000/tf`

