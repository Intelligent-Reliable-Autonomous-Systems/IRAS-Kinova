safety_filter.py

The new layer node between the publisher and the policy_controller.
NOw policy_controller is subscribing to the cmd_safety which is safety_filter node.
IT checks if the command is safety to be executed or not based on the implied velocity
compared to the velocity limit and on the joint upper and lower limits. 
All the limits were taken from the URDF file:
```
./src/third_party/ros2_kortex/ros2_kortex/kortex_description/robots/gen3_2f85.urdf
```

To run it and see the print statements:
```
ros2 run gen3_controllers safety_filter
```

Can be ran in the background together with Gazebo or the arm itself.

Safe command to test:
ros2 topic pub /cmd_safety trajectory_msgs/msg/JointTrajectory "{
joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
points: [
  {
    positions: [0.0, 0.5, 0.0, 0.5, 0.0, 0.3, 0.0],
    time_from_start: { sec: 5 }
  }
]
}" -1


Unsafe command to test:
ros2 topic pub /cmd_safety trajectory_msgs/msg/JointTrajectory "{
joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
points: [
  {
    positions: [0.0, 3.0, 0.0, 0.5, 0.0, 0.3, 0.0],
    time_from_start: { sec: 5 }
  }
]
}" -1
