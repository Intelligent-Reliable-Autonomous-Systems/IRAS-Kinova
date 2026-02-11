### Launch simulation 
- `ros2 launch warehouse_sim launch_warehouse.py`

```
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
  points: [
    { positions: [2.23, 1.67, -0.0, .9, -1.2, 1], time_from_start: { sec: 2 } },
  ]
}"
```
