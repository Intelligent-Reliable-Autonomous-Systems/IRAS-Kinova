ros2 topic pub /run_skill gen3_cpp/Params "{skill_name: go_to_xyz, param_names: [x, y, z], param_values: [0.4, 0.0, 0.3]}" -1


ros2 topic pub /run_skill gen3_cpp/Params "{skill_name: go_to_xyz, param_names: [x, y, z], param_values: [0.52, -0.02, 0.33]}" -1


joint_1: -2.4555
[skills-2] [INFO] [1765560786.660195210] [gen3_skills]:   joint_2: -1.5764
[skills-2] [INFO] [1765560786.660357283] [gen3_skills]:   joint_3: -2.2197
[skills-2] [INFO] [1765560786.660522416] [gen3_skills]:   joint_4: -2.0519
[skills-2] [INFO] [1765560786.660691437] [gen3_skills]:   joint_5: 1.5396
[skills-2] [INFO] [1765560786.660853613] [gen3_skills]:   joint_6: -2.0900
[skills-2] [INFO] [1765560786.661018930] [gen3_skills]:   joint_7: 1.8364




ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
    points: [
        { positions: [-2.4555, -1.5764, -2.2197, -2.0519, 1.5396, -2.0900,1.8364], time_from_start: { sec: 5 } },
    ]
    }" -1