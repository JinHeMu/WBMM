# tracer_jaka_description

本包是 Tracer + JAKA ZU5 的唯一机器人描述源。

- 规划/碰撞/TF：`urdf/tracer_jaka_zu5.urdf`
- MuJoCo ros2_control：展开 `tracer_jaka_zu5.controlled.urdf.xacro control_backend:=mujoco`
- 实机 ros2_control：展开 `... control_backend:=real robot_ip:=... local_ip:=... jaka_read_only:=true`
- MoveIt mock：展开 `... control_backend:=mock`
- 统一控制器：`config/ros2_controllers.yaml`

`arm_controller` 和 `arm_trajectory_controller` 会声明同一组关节，运行时只能按任务
启动其中一个：OCS2 使用前者，MoveIt 使用后者。
