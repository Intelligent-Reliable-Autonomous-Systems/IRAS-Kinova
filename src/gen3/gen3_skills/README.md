# Gen3Skills Package
A package containing behavior primitives, ie. "skills" for use.

# Installation
See the IRAS-Kinova README for installation

# Running
1. Run the launch file to bring up the skills manager and skills nodes 
2. Publish a skills topic: 

```
ros2 topic pub /run_skill gen3_cpp/Params "{skill_name: go_to_xyz, param_names: [x, y, z], param_values: [0.4, 0.0, 0.3]}" -1
```

ros2 topic pub /run_skill gen3_cpp/Params "{skill_name: go_to_xy, param_names: [x, y], param_values: [0.4, 0.0]}" -1
