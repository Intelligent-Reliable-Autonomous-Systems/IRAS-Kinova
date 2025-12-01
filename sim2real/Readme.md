### Template for env.yaml configuration

This template uses the arm and gripper in the actuator label for observations while the actions label is used for actions. The default position is useful if the robot is operating from an offset.


```
actuators:
  arm:
    joint_names:
      - joint 
  gripper:
    joint_names:
      - joint
actions:
  arm_action:
    joint_names:
      -joint 
  gripper_action:
    joint_names:
      - joint
default_pos:
  joint_1: 0.0
```

