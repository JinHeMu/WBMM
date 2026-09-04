# ADR-0001：统一机器人描述与 ros2_control 配置

- 状态：已接受
- 日期：2026-09-01
- 范围：Tracer + JAKA ZU5 的仿真、规划、MoveIt 与实机启动链

## 背景

仓库曾在 `tracer_jaka_description`、`tracer_jaka_mujoco`、
`tracer_jaka_ocs2`、`tracer_jaka_moveit_config` 和 `jaka_driver` 中维护多份
URDF/xacro 与 controller YAML。它们的关节、传感器框架、碰撞体和控制器名称已经
发生漂移；其中旧 description URDF 还包含非法关节名，不能作为运行基线。

## 决策

1. `tracer_jaka_description/urdf/tracer_jaka_zu5.urdf` 是唯一的几何、运动学、
   关节限位、碰撞体和传感器 TF 权威源。
2. `tracer_jaka_zu5.controlled.urdf.xacro` 只负责注入 ros2_control；
   `control_backend` 可取 `mujoco`、`real` 或 `mock`，不得改变几何树。
3. JAKA IP、EDG 本机 IP、只读开关和 F/T 标定仅存在于 real backend。
4. `config/ros2_controllers.yaml` 是唯一 controller inventory：
   - `arm_controller`：位置 forward controller，供 OCS2/Servo；
   - `arm_trajectory_controller`：JointTrajectoryController，供 MoveIt；
   - `base_controller`：仅在 MuJoCo 后端启动；
   - `fts_broadcaster`：仅在实机后端启动；
   - `joint_state_broadcaster`：两端共用。
5. 仿真保持 200 Hz，实机保持 125 Hz；launch 只覆盖更新率和 `use_sim_time`，
   不再复制 controller 定义。
6. 实机默认 `jaka_read_only:=true`，上层 `command_output_enabled:=false` 的双门禁不变。

## 兼容与验证

- 所有仓库内消费者已迁移到 description 包；旧重复文件删除。
- 自动测试比较 canonical、MuJoCo、real 和 mock 展开结果的 link、joint、origin、
  limit 与 collision，防止后端切换改变机器人模型。
- 自动测试同时检查六球底盘碰撞包络、关键传感器框架、mesh 所有权、插件隔离和
  controller 名称。
- MuJoCo 的 MJCF 仍由 `tracer_jaka_mujoco/models` 管理；本 ADR 不把 MJCF 与 URDF
  合并，因为两者面向不同解析器，但 URDF/MJCF 的标定一致性仍需测试保障。

## 后果

新增机器人几何或标定时只修改 canonical URDF；新增硬件后端只扩展 ros2_control
xacro。旧路径不再是兼容 API，外部脚本必须改为通过
`get_package_share_directory('tracer_jaka_description')` 获取模型。
