# WBMM 项目与启动地图

> CURRENT（2026-09-17）：launch 目录已扁平化，不再使用 `launch/common`、`launch/real`、
> `launch/sim`，也不再用 `_real` / `_sim` 文件名。硬件/仿真只能由
> `wbmm_hardware_interface.launch.py` 与 `mujoco_hardware_interface.launch.py` 启动；
> 其余核心 launch 只启动算法并消费 `config/common/interface.yaml` 定义的接口。
> `wbmm.launch.py` 是可选总入口，默认 `hardware_backend:=none`。
> 下文部分是旧版部署记录，具体路径以当前 launch 目录和接口文档为准。


> Status: DRAFT  
> Author: Agent  
> Reviewer: TBD  
> Reviewed at: TBD  
> Review Level: L1  
> Warning: 本文档尚未经过人工完整审阅，不能作为实现或实机执行依据。

## 1. CURRENT：目录职责

| 目录 | 当前职责 |
|---|---|
| `src/core/` | `wbmm_core` 公共数据与契约实现 |
| `src/planning/` | `search/` 中的 `wbmm_search` 提供离线差速底盘 Kino A*、测试和绘图；`optimize/` 尚为空，运行控制链接入 TBD |
| `src/control/` | OCS2 模型、ROS 跟踪接口和全身力控 |
| `src/robotics/` | 机器人描述、Pinocchio、ROS 接口、MoveIt 配置 |
| `src/map/` | 定位、地图、ESDF 与地图可视化 |
| `src/metrics/` | `wbmm_robot_metrics` 机械臂构型评价骨架，指标计算 TBD |
| `src/drivers/` | 底盘、机械臂、传感器与夹爪驱动 |
| `src/sim/` | MuJoCo 模型与状态/命令接口 |
| `src/vendor/` | REMANI、OCS2 等外部后端，本次未修改 |
| `src/bringup/` | 启动编排、部署参数与就绪检查 |

离线搜索入口：`wbmm_search/kino_astar_demo` → 底盘搜索 → CSV →
`plot_kino_astar.py`。仅生成搜索初值，不发布 ROS 参考或机器人命令。
默认要求底盘碰撞回调，示例显式关闭碰撞检查；详情见
[差速底盘 Kino A*](kino_astar.md)。

新增骨架（2026-09-18，DRAFT）：`src/map/wbmm_environment` 提供 ESDF 数据与加载/查询入口，
`src/robotics/wbmm_collision` 提供碰撞球与环境检测入口，`src/metrics/wbmm_robot_metrics`
提供机械臂及关节限位评价入口。算法当前均为 `NotImplemented`，尚未接入规划器或控制器。
目录、输入输出及待审阅约定见 [环境、碰撞与评价骨架](environment_collision_metrics.md)。

## 2. CURRENT：common 只启动算法

依据：`src/bringup/launch/common/`、`src/bringup/CMakeLists.txt`。

源启动文件按 `common/`、`sim/`、`real/` 分类；安装时仍平铺到包的
`share/tracer_jaka_bringup/launch/`，因此命令中的启动文件名不带分类目录。

| common 文件 | 输入 | 启动内容 / 输出 |
|---|---|---|
| `localization.launch.py` | EKF/SLAM 参数文件，轮速里程计、IMU、激光 topic | EKF 与可选 SLAM；滤波里程计及对应 TF |
| `ocs2.launch.py` | 参数 YAML、task、URDF、里程计/关节反馈、MPC 参考 | MPC/MRT、可选目标与 RViz；由显式输出门控制底盘/机械臂命令 |
| `remani.launch.py` | URDF、静态 ESDF、里程计/关节状态、规划/控制 frame | REMANI 与可选参考桥；规划轨迹与 OCS2 名义参考 |
| `moveit.launch.py` | 外部机器人状态、MoveIt 包内配置 | MoveIt 与 RViz；默认关闭轨迹执行 |

`common` 不选择 backend，不启动 MuJoCo、硬件驱动、
`robot_state_publisher`、`controller_manager` 或夹爪驱动。
`use_sim_time` 是显式 ROS 时钟接口，而不是部署环境选择开关。

参数精简后：OCS2 的 topic/数值调参由传入的 YAML 承担，launch 保留文件路径、
必要运行开关、里程计覆盖项与命令输出门。EKF/SLAM 和 OCS2 的参数文件由外层明确传入；
不传入有效文件时会报错，不会自动猜测部署环境。

## 3. CURRENT：部署文件负责组件组合

```text
sim/remani_mpc_sim.launch.py
  -> sim/ocs2_sim.launch.py
     -> sim/sim.launch.py                    状态源 / 命令消费者
     -> common/localization.launch.py       EKF + 可选 SLAM
     -> common/ocs2.launch.py               MPC/MRT
     -> common/remani.launch.py             REMANI + 参考桥

real/remani_mpc_real.launch.py
  -> real/real_slam.launch.py                硬件反馈 + EKF/SLAM
  -> real/ocs2_real.launch.py                JAKA 控制接口 + common/ocs2
  -> common/remani.launch.py                 REMANI + 参考桥

real/remani_mpc_localized_real.launch.py
  -> real/real_slam.launch.py start_slam=false
  -> tracer_jaka_localization/amcl_localization.launch.py
  -> odom_to_map_relay.py
  -> real/ocs2_real.launch.py
  -> common/remani.launch.py
```

`real/moveit_real.launch.py` 负责 JAKA ros2_control、控制器与可选夹爪，
然后调用 `common/moveit.launch.py`。`common/moveit.launch.py` 本身不再自动启动仿真；
使用者应由外层提供状态源和命令消费者。

统一 `common/bringup.launch.py` 已删除，原有直接调用均已改为上述组件组合。
不新增 manager、抽象层或第三方依赖。

本次文件变更（不包含用户已有的其他未提交改动）：

- 删除：`src/bringup/launch/common/bringup.launch.py`；安装目录的旧副本与源目录缓存
  已移到 `/tmp/wbmm_removed_bringup.s0xs1H/`，可用旧安装副本恢复。
- 精简算法端：`common/localization.launch.py`、`common/ocs2.launch.py`、
  `common/remani.launch.py`、`common/moveit.launch.py`。
- 更新直接调用：`sim/ocs2_sim.launch.py`、`sim/remani_mpc_sim.launch.py`、
  `sim/slam_sim.launch.py`、`sim/ocs2_esdf_validation.launch.py`；
  `real/ocs2_real.launch.py`、`real/remani_mpc_real.launch.py`、
  `real/remani_mpc_localized_real.launch.py`、`real/real_slam.launch.py`、
  `real/moveit_real.launch.py`。以上相对路径均位于 `src/bringup/launch/`。
- 新增 `src/bringup/test/test_common_launch.py`：不启动节点，检查算法端边界、
  已删除参数、TF 设置、单一命令所有权及组件引用/参数传递。
- `src/bringup/CMakeLists.txt`：仅在用户已有分类安装修改之上注册上述新测试。
- 文档阶段：新增本文件，并在 `docs/README.md` 中登记；未再修改源码。

## 4. CURRENT：偏移参数的清理边界

bringup 启动文件中不再声明或传递 `static_esdf_offset_x/y/z` 和
`planner_to_ocs2_x/y/yaw`。

静态 ESDF 必须事先与 `planner_frame` 对齐；不能靠 launch 中的平移量修补地图原点、
旋转或坐标系错误。规划与控制 frame 相同时使用同一坐标；不同时必须启用 TF 变换。
保存地图的定位入口仍保留 `map` 规划、`odom` 跟踪及里程计转换，不把两者静默合并。

注意：本次只移除 bringup 启动接口，没有删除外部 REMANI launch、地图实现或
`remani_to_ocs2_reference_bridge.cpp` 的兼容参数。
该桥在 TF 查询失败时仍具有原来的固定变换回退。此行为不是本次修复，
必须另行人工审查，不能将“参数已删除”理解为“TF 故障已安全处理”。

状态维度仍为 9D，输入仍为 8D；本次未修改规划/MPC 求解器、代价、约束、
力控算法或驱动源码。

## 5. 运行与验证

工作目录：WBMM 仓库根目录；依赖已构建并加载的 ROS 2 Humble 工作空间。
查看算法入口参数，不启动节点：

```bash
ros2 launch tracer_jaka_bringup remani.launch.py --show-args
ros2 launch tracer_jaka_bringup ocs2.launch.py --show-args
```

仿真入口示例（不自动发送目标）：

```bash
ros2 launch tracer_jaka_bringup remani_mpc_sim.launch.py publish_demo_goal:=false
```

CURRENT：本次实际执行了 bringup 单包构建、启动编排解析测试、原有安全门测试、
格式检查与 diff 检查：19 项用例通过，3 组 CTest 通过，15 个相关 Python 文件
格式检查通过。编排解析不执行 ROS Node 或外部进程，未连接机器人、
未发送命令，也未证明真实轨迹跟踪、接触力控或硬件安全。

验证依据：`src/bringup/test/test_common_launch.py`、
`test_localized_real_safety_gate.py`、`test_force_control_real_safety_gate.py`。

## 6. TBD：人工确认

- 实机安全门保持默认只读、命令关闭；部署编排已经调整，实机行为尚未重新验证。
- 急停、看门狗、反馈新鲜度、通信/命令超时、碰撞/限幅、SAFE_HOLD/FAULT 与恢复条件，
  本次未新增或验证；实机相关改动需逐条人工确认，不提供执行放行结论。
- 删除偏移后，具体 ESDF 与机器人/规划坐标是否实际对齐，需要数据与可视化验收。
- `src/bringup/README.md` 中旧 `backend` 用法和偏移校准说明尚未同步，不能据此启动新入口；
  当前接口以本次启动文件和参数展示为准。
- PROPOSED：如后续需要移除节点内部的固定变换回退，应单独定义 TF 丢失行为，
  再作为独立代码任务实现与测试。
