# REMANI → WBMM 第二轮完善与人工审核清单

日期：2026-09-28。状态：DRAFT，已实现，等待人工审核。本文记录本轮增量，不把工作区已有未提交迁移归为本轮成果。

## 本轮实现

保留既有 `wbmm_core → wbmm_search → wbmm_planner → wbmm_planner_ros → wbmm_reference_bridge → wbmm_ocs2` 分层。未修改 vendor、OCS2 代价函数、MRT 或 MuJoCo 驱动；这些文件在本轮开始前已有修改。

本轮默认流程：

1. **Kino A\***：继续使用现有差速 `(v, omega)` 搜索。
2. **Sample Arm RRT**：在 `(底盘路径索引, 关节配置)` 上保留树与父指针，允许固定底盘调整手臂，也允许底盘与手臂同时推进。一个分支走不通后可以从其他已保留分支扩展。起始关节严格来自实测，不再被第一个采样点偷偷替换。
3. **Whole-body RRT**：A* 搜索预算耗尽/无路径，或路径约束臂搜索失败时，联合搜索 `(x,y,yaw,q)`。局部连接使用差速原地转向、前进/倒退直线、终端转向；检查每段扫掠，而不是把底盘当作可横移质点。非法请求、碰撞回调异常不以换后端掩盖。
4. **轨迹生成**：优先保留 MINCO 整形。对于原地旋转、固定底盘调臂、混合挡位或整形失败，采用精确运动原语的五次时间缩放，每段首尾速度和加速度为零。最终仍执行关节限位、碰撞、时间预算和终端目标检查。
5. **执行接线**：后台规划，主 ROS 执行器持续接收状态。新目标先取消旧导航并产生实测状态保持参考，再等待静止后规划；取消或更新目标会使正在计算的旧结果失效。发布前复核新鲜度、静止条件和起点偏移。
6. **完成与模式切换**：轨迹时长结束后，通过实测位置、航向、关节目标（如提供）及连续静止确认 `/planning/finish`。EE 目标取消旧导航，并通过现有 OCS2 服务申请 Execution；服务未就绪/拒绝时重试，过时回复不覆盖更新的意图。

这是可行 RRT，不是上游 RRT* 的逐行复刻；没有最优性或完备性声明。全身 RRT 的局部连接更保守，分段停止也可能较慢。

## 新增和保留的接口

| 接口 | 行为 |
|---|---|
| `/goal_pose`，`geometry_msgs/PoseStamped` | 保留原 2D 导航目标；终端手臂自由 |
| `/wbmm/whole_body_goal`，`wbmm_planning_msgs/WholeBodyGoal` | `header + base_pose + joint_names + joint_positions`；关节数组可空；非空时按名称重排到控制顺序并约束终端关节 |
| `/wbmm/whole_body_trajectory` | 继续使用原采样轨迹契约；无需 vendor 多项式消息 |
| `/wbmm/planning/status`，`std_msgs/String` | `WAITING_FOR_GOAL / WAITING_FOR_STOP / PLANNING / EXECUTING / SUCCEEDED / FAILED / CANCELLED / FAULT`，带后端/失败原因，保留最后状态 |
| `/wbmm/planning/cancel`，`std_msgs/Header` | 带当前 ROS 时间的取消事件；撤销规划结果并请求桥发布实测状态的零前馈保持参考；延迟到达的旧轨迹会被拒绝 |
| `/planning/finish`，`std_msgs/Bool` | 新目标发布 false；实测完成后发布 true |
| OCS2 双参考与 SetTaskPhase | 继续复用既有实现，未改代价权重或模式语义 |

保持参考是对控制器的请求，不是瞬时制动保证。观测不新鲜时参考桥不能伪造当前状态；控制器反馈超时保护仍承担其原有职责。

新增搜索参数可通过 `wbmm_planning.launch.py` 设置：`sample_rrt_max_time`（2 s）、`whole_body_rrt_max_time`（3 s）、`rrt_max_nodes`（12000）、`rrt_random_seed`（1）、`enable_whole_body_rrt`、`enable_primitive_fallback`。搜索区域从加载的 ESDF 元数据读取。完整跟踪入口采用这些默认值。

节点仍使用已有 `state_timeout=0.5 s`，允许反馈相对 `/clock` 最多提前 0.05 s，接收与执行监测采用同一判据。默认实测到达门槛为 0.20 m / 0.30 rad，规划使用不超过其一半的终点容差，为控制跟踪误差预留空间。

## 启动与复现

保留用户指定的入口，同时支持 WBMM 命名入口：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tracer_jaka_bringup remani_tracking.launch.py \
  static_esdf_file:=/absolute/path/to/site_remani.npz
# 等价的 WBMM 入口：
ros2 launch tracer_jaka_bringup wbmm_tracking.launch.py \
  esdf_file:=/absolute/path/to/site_remani.npz
```

默认演示假设地图 frame 为 `map`，并发布单位 `map → odom`。真实定位或不同地图坐标系必须由部署提供正确 TF；不能用单位 TF 冒充定位对齐。地图由规划器与 OCS2 加载，MuJoCo 的 `esdf_validation` 场景本身只提供机器人和地面，ESDF 障碍不等于 MuJoCo 物理接触几何。

离线构建/测试：

```bash
colcon build --packages-select wbmm_planning_msgs wbmm_search wbmm_planner \
  wbmm_planner_ros wbmm_reference_bridge tracer_jaka_bringup --parallel-workers 2
colcon test --packages-select wbmm_search wbmm_planner wbmm_reference_bridge \
  wbmm_traj_opt wbmm_collision wbmm_environment
python3 -m pytest src/bringup/test/ -q -p no:anyio -p no:cacheprovider
```

带真实 ESDF/URDF 的隔离消息链回归，以及包含 MuJoCo/OCS2 的闭环回归：

```bash
mkdir -p /tmp/wbmm_migration_check
cd /tmp/wbmm_migration_check
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/wbmm_migration_check/roslog
python3 /home/a/WBMM/src/bringup/test/planning_migration_regression.py \
  /home/a/WBMM/src/robotics/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf \
  /home/a/WBMM/maps/map1/site_remani.npz

# 选择未使用的独立 ROS 域；脚本仅启动 MuJoCo，并自动回收测试进程。
export ROS_DOMAIN_ID=93
python3 /home/a/WBMM/src/bringup/test/planning_tracking_check.py \
  --output /tmp/wbmm_tracking_check
# --entrypoint remani_tracking.launch.py 可复核兼容入口。
```

闭环脚本为自身和子进程设置 CycloneDDS 的参与者预算（100）；不改系统配置。首次试跑暴露了本机默认参与者上限不足。后续试跑暴露了执行新鲜度判据与接收判据不一致，以及规划终点恰在到达容差边界的问题；两者均在本轮修正。

## 必须人工审核的设计与未完成边界

| 优先级 | 项目 | 当前边界 / 审核内容 |
|---|---|---|
| P1 | 碰撞几何覆盖与自碰撞 | 新搜索复用当前环境碰撞检查器。尚未补齐独立自碰撞硬检查、完整 mesh/box/cylinder 覆盖与允许碰撞对配置；OCS2 的自碰撞代价不等于搜索硬约束 |
| P1 | 未知区域策略 | 沿用既有地图及 launch 的开放策略，未擅自修改。现场需要审核 NPZ 的 unknown 策略、边界和模型球覆盖 |
| P1 | 停止与取消 | 新目标和 EE 接管采用停止/保持后切换；需要验收实体制动距离、观测超时及通信故障，不是机器人安全认证 |
| P1 | 参考发布与阶段切换 | 仍复用 OCS2 既有双参考、阶段服务与控制策略。服务请求接受不等于控制策略即时切换；真机需验证切换瞬态与所有权 |
| P2 | 全身 RRT 质量 | 固定种子便于同环境复现，超时仍受机器负载影响。无 rewiring、无 RRT* 最优性保证；狭窄通道和长距离需要场景统计 |
| P2 | 分段停止轨迹 | 每条原语边都停下，可能影响效率；速度满足配置，但尚未实施全局加速度/jerk 限制验收。桥使用线性采样插值，不能宣称连续碰撞或连续动力学证明 |
| P2 | 优化器 | `WholeBodyOptimizer` 仍未接入运行主链；既有横向梯度与时长优化问题未在本轮解决。MINCO 整形不等于上游完整联合优化 |
| P2 | 在线重规划 | 实现的是新目标触发的停止后重规划、取消、反馈故障与到达超时。没有迁移保持非零边界速度的连续重规划、动态地图刷新或移动障碍预测 |
| P2 | 整机验收 | 单一 map1 小场景不能代替用户地图上的障碍绕行、倒车、狭窄通道、长距离、频繁模式切换统计；未执行真机试验 |

## 验证结果

- 六个 C++ 包共 **130 项测试通过**，0 failures/errors；包含 5 项新增 RRT 测试和 5 项新增编排测试。
- bringup / phase bridge Python 测试 **39 项通过**。
- 真实 ESDF/URDF 的隔离 ROS 回归通过：TF、非法输入、状态/观测超时、所有权失效、取消保持、过时计划拒绝、取消乱序、计算中取消、全身目标及轻微时钟偏差。
- `wbmm_tracking.launch.py`：0.8 m 导航、原地转向、Navigation → Execution 与 4 cm EE 位移闭环通过。
- **用户指定的 `remani_tracking.launch.py`**：上述流程及 Execution → Navigation 回切通过。该入口实测导航位置误差约 **0.995 mm**，转向航向误差约 **0.00276 rad**；EE 位置误差在 **2 cm**、姿态误差在 **0.1 rad** 内持续至少 1 s，结束时位置误差约 **12.7 mm**。回切导航后的保持位置误差约 **2.32 mm**。
- 以上是一次小场景回归的观测值，不是性能统计。未执行实际 ESDF 障碍绕行的完整 RRT 闭环、整张用户地图验收、物理障碍接触或真机试验。
- 最后一轮代码审查还修正了跨话题乱序：旧取消事件晚于新轨迹到达时，不再覆盖新轨迹；该改动通过隔离 ROS 回归，未额外重复整机仿真。

机器可读记录：[remani_migration_round2_verification.json](remani_migration_round2_verification.json)。原始构建、测试和仿真日志保存在 `/tmp/wbmm_migration_0928/`，临时目录可能被系统清理。未执行全工作区构建，未提交或推送。

## 本轮准确文件清单

相对于本轮开始时保存的工作区快照，**19 个既有文件修改、6 个源码/测试文件新增**；另新增本文、验证 JSON 和增量补丁。这里的“既有”包括用户上一轮已经创建但尚未 git add 的文件。

[增量补丁](remani_migration_round2.patch) 只包含以下 25 个源码/配置/测试文件的本轮改动，不包含用户已有迁移，也不包含三份交付材料本身。不要将整份当前 `git diff` 误认为本轮增量。

| 类型 | 文件 |
|---|---|
| 修改 | `src/bringup/launch/remani_tracking.launch.py` |
| 修改 | `src/bringup/launch/wbmm_planning.launch.py` |
| 修改 | `src/bringup/launch/wbmm_tracking.launch.py` |
| 修改 | `src/bringup/package.xml` |
| 修改 | `src/bringup/scripts/remani_phase_bridge.py` |
| 修改 | `src/bringup/test/planning_migration_regression.py` |
| 新增 | `src/bringup/test/planning_tracking_check.py` |
| 修改 | `src/bringup/test/test_common_launch.py` |
| 新增 | `src/bringup/test/test_phase_bridge.py` |
| 修改 | `src/control/wbmm_reference_bridge/CMakeLists.txt` |
| 修改 | `src/control/wbmm_reference_bridge/package.xml` |
| 修改 | `src/control/wbmm_reference_bridge/src/wbmm_reference_bridge_node.cpp` |
| 修改 | `src/planning/search/CMakeLists.txt` |
| 新增 | `src/planning/search/include/wbmm_search/whole_body_rrt.hpp` |
| 新增 | `src/planning/search/src/whole_body_rrt.cpp` |
| 新增 | `src/planning/search/test/test_whole_body_rrt.cpp` |
| 修改 | `src/planning/wbmm_planner/include/wbmm_planner/whole_body_planner.hpp` |
| 修改 | `src/planning/wbmm_planner/src/whole_body_planner.cpp` |
| 修改 | `src/planning/wbmm_planner/test/test_whole_body_planner.cpp` |
| 修改 | `src/planning/wbmm_planner_ros/CMakeLists.txt` |
| 修改 | `src/planning/wbmm_planner_ros/package.xml` |
| 修改 | `src/planning/wbmm_planner_ros/src/wbmm_planner_node.cpp` |
| 修改 | `src/planning/wbmm_planning_msgs/CMakeLists.txt` |
| 新增 | `src/planning/wbmm_planning_msgs/msg/WholeBodyGoal.msg` |
| 修改 | `src/planning/wbmm_planning_msgs/package.xml` |
