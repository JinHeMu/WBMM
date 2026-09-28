# WBMM 仓库重读方案

> Status: DRAFT
> Author: Agent
> Reviewer: TBD
> Reviewed at: TBD
> Warning: 本文档尚未经过人工审查，不能作为实现依据。

> 面向「自 `wbmm_collision` 开始编写后就没有再看过代码」的读者。
> 目标不是逐行读完，而是用最短路径重建心智模型，并知道哪里需要质疑。

---

## 0. 先建立时间感（5 分钟）

你的阅读起点是 `0a0046d`。之后仓库发生了两次大变化：

```bash
git log --oneline --date=short --pretty="%h %ad %s" -12
```

| 提交 | 日期 | 意义 |
| --- | --- | --- |
| `e010cbf` | 09-17 | 加入 `planning/search`，whole-body force control 改进 |
| `0a0046d` | 09-21 | **加入 `wbmm_collision` + `wbmm_environment` + `wbmm_robot_metrics`** ← 你的起点 |
| `16962b0` | 09-21 | 双参考 OCS2 模式切换 |
| `f320b03` | 09-22 | ft_payload |
| `db4c79e` | 09-22 | force control 拆分传感器处理与导纳 |
| `3d3c5ae` | 09-23 | ESDF map1 导航 + 双参考跟踪（**最后一次提交**） |
| — | 09-24 | **本次 REMANI 迁移，全部未提交** |

**关键状态：工作区有 51 个未提交改动。** 你现在读到的东西没有任何一次提交作为检查点。

```bash
git status --short | wc -l     # 51（含本方案文档自身）
git diff --stat | tail -5      # 27 个已跟踪文件，+543 / -1276
```

改动分三类：

1. **纯新增**（本次迁移的核心）：5 个新包 + 3 个新文件
2. **修改**：`wbmm_collision` / `wbmm_environment` / `wbmm_ocs2_ros` / bringup / mujoco 桥
3. **删除**：`remani.launch.py`、`remani_to_ocs2_reference_bridge.cpp`（1047 行）

---

## 1. 契约层（约 30 分钟）

先看契约，后面所有代码都是它的实现。

| 文件 | 看什么 |
| --- | --- |
| `docs/math_contract.md` | 已有文档。状态 9 维 `[x, y, ψ, q₁..q₆]`、输入 8 维 `[v, ω, q̇₁..q̇₆]`、坐标系约定 |
| `src/core/wbmm_core/` | `WholeBodyState` / `WholeBodyInput` / `WholeBodyTrajectory` / `RobotModel` / `RobotLimits` |
| `src/planning/wbmm_planning_msgs/msg/WholeBodyTrajectory.msg` | **新增**。62 行，**采样轨迹**而非多项式系数 |

**要质疑的点**：新消息传采样点（`float64[] base_x` 等），REMANI 传多项式系数。
理由是解耦参数化，代价是消息体积。这是有意的设计变更，不是等价迁移。

---

## 2. ROS-free 算法层（主体，约 3 小时）

**按依赖顺序读**，每个包都是「ROS-free 核心 + 薄节点」的结构。建议边读边跑对应单测。

### 2.1 `wbmm_collision` 的新增部分（约 40 分钟）

你熟悉这个包的旧内容（`environment_collision_checker`），**新增了三个文件**：

| 文件 | 作用 |
| --- | --- |
| `include/wbmm_collision/urdf_collision_model.hpp` + `src/*.cpp` | 用 **tinyxml2**（不是 `urdf::Model`）从 URDF 提取碰撞球与关节链 |
| `include/wbmm_collision/whole_body_kinematics.hpp` + `src/*.cpp` | **解析梯度**运动学：球心位置 + 3×(4+n) 雅可比 |

**要重点看的**：`whole_body_kinematics.hpp` 里的变量布局 ——

```
[0] x  [1] y  [2] vx  [3] vy  [4..] 关节角
```

**yaw 不是变量**，而是由速度方向重建 `yaw = atan2(gear·vy, gear·vx)`。
整个求值器存在的理由就是让求导穿过这个重建。

**验证**：`test_whole_body_kinematics.cpp` 用中心差分对照 ~9000 个雅可比元素。
这个测试**有鉴别力**——注入符号错误能定位到具体元素（我实测过）。

**要质疑的点**：`kMinSpeedForHeading` 的处理。近停时航向病态，这个限制一路影响到优化器（见 2.4）。

### 2.2 `wbmm_environment` 的改动（约 20 分钟）

只改了三处，但**语义重要**：

| 文件 | 改动 |
| --- | --- |
| `include/wbmm_environment/types.hpp` | `MapInfo::unknown_is_occupied`、`DistanceQuery::fully_observed` |
| `src/esdf_grid.cpp` | 未观测角点不再返回 `kUnknown`，改返回有效距离 + `fully_observed=false` |
| `src/npz_esdf_loader.cpp` | 读可选的 `unknown_is_occupied.npy` |

**背景（实测数据）**：`maps/map1/site_remani.npz` 的 `observed` 只有 **12.34%**，
八角全观测 **10.55%**。按 `observed` 硬拒绝会破坏约 **89%** 的查询。
且 100% 的未观测体素 `esdf == +2.0`、零个是占据的。

**要质疑的点**：这是对部署地图的数据驱动妥协，不是通用语义。换地图要重新确认。

### 2.3 `wbmm_search` 的新增（约 30 分钟）

| 文件 | 作用 |
| --- | --- |
| `include/wbmm_search/arm_seed_search.hpp` + `src/*.cpp` | 全身臂种子：沿底盘路径逐点用**确定性 Halton 序列**采样 |

**要质疑的点（重要）**：REMANI 用 RRT* + `sample_mani_RRT`（2484 行）。
这里换成确定性 Halton，好处是可复现、无随机失败，**代价是完备性弱**——
狭窄通道可能找不到种子。这是取舍不是等价。

### 2.4 `wbmm_traj_opt`（最大的一块，约 1.5 小时）

5739 行，分四个部分：

| 文件 | 来源 | 看什么 |
| --- | --- | --- |
| `minco.hpp` (1626 行) | 从 REMANI `poly_traj_utils.hpp` 移植 | **系数是真实时间多项式系数**，`getTraj` 里有 `.rowwise().reverse()` |
| `root_finder.hpp` / `lbfgs.hpp` | 逐字复制 | 可跳过 |
| `whole_body_trajectory_builder.{hpp,cpp}` | 新写 | MINCO → 可发布参考 |
| `whole_body_optimizer.{hpp,cpp}` | 新写 | LBFGS 轨迹优化 |

**`whole_body_trajectory_builder` 里最值得看的一点**：航向基线用**弧长**而不是样本数。

| 航向估计方式 | 实测峰值 ω |
| --- | --- |
| 瞬时速度方向（REMANI 做法） | 14.06 rad/s |
| 固定样本数窗口 | 4.52 rad/s |
| **固定弧长弦** | **0.112 rad/s** |

原因：MINCO 在每个内部路径点处 x 速度会下陷，固定样本窗口在那里只跨过极短距离。

**`whole_body_optimizer` 要质疑的点**（都写在头文件 scope note 里）：
- 决策向量**只有内点**，时长优化被移出（未验证正确）
- 横向（y）分量梯度病态，`TransverseGradientIsIllConditioned` 测试记录了这个限制
- 有限差分测试用**收敛性**判据而非绝对容差

### 2.5 `wbmm_planner`（约 30 分钟）

860 行，把上面几块串起来：KinoAstar → 臂种子 → MINCO 整形 → 时间缩放。

**要看的关键点**：
- `PlannerConfig::minco_waypoint_stride`（默认 3）—— MINCO **隔点穿过**路径点，
  否则会忠实复现 KinoAstar 粗搜索的高频抖动。代价是碰撞保证变弱
- 时间缩放循环：七次多项式峰值因子 ≈ 2.19×平均速度，按平均速度分配时间必然超包络

---

## 3. ROS 薄封装层（约 1 小时）

| 包 | 行数 | 看什么 |
| --- | --- | --- |
| `src/planning/wbmm_planner_ros/` | 534 | ESDF/URDF 加载、TF、`state_timeout` 新鲜度检查、静止性检查 |
| `src/control/wbmm_reference_bridge/` | 978 | **从 `wbmm_ocs2_ros` 迁出并重命名**的旧桥 |

**`wbmm_reference_bridge` 值得对比读**：旧文件 `remani_to_ocs2_reference_bridge.cpp`
（1047 行，已删除）要解码 REMANI 的多项式；新桥只做采样插值。用
`git show HEAD:src/control/wbmm_ocs2_ros/src/remani_to_ocs2_reference_bridge.cpp` 对照。

---

## 4. 集成层（约 40 分钟）

### 4.1 launch

| 文件 | 状态 |
| --- | --- |
| `src/bringup/launch/wbmm_planning.launch.py` | **新增**，算法侧（planner + bridge） |
| `src/bringup/launch/wbmm_tracking.launch.py` | **新增**，完整 demo（替代 `remani_tracking`） |
| `src/bringup/launch/remani.launch.py` | **已删除**（vendor 包装） |
| `wbmm.launch.py` / `remani_mpc` / `remani_mpc_localized` | remani 分支改为 include 新 launch |

**已知隐患**：`wbmm.launch.py` 把 `bridge_world_frame` 默认成 `"odom"`，
而 planner 采纳 ESDF 的 `"map"` —— 直接跑 `wbmm.launch.py` 会让桥拒绝所有轨迹。
`wbmm_tracking.launch.py` 里把两者钉到同一个值绕过了，但**默认值组合仍然不一致**。

### 4.2 config 改动

```bash
git diff src/bringup/config/
```

| 文件 | 改动 | 目的 |
| --- | --- | --- |
| `common/ocs2.yaml` | 底盘限速 0.1 / 0.4 | 真机安全 |
| `sim/ocs2.yaml` | 底盘限速 0.5 / 1.0，`arm_max_delta_per_step` 收紧 | 修「乱飞」 |
| `sim/task_esdf_tracking.info` | `solutionTimeWindow` 0.2 → 0.5 | 修 MPC plan expired |
| `sim/remani_tracking.yaml` | 轮速限幅 | vendor 用，现已失效 |

### 4.3 硬件接口改动

`src/sim/tracer_jaka_mujoco/.../mujoco_bridge_node.py` 加了 cmd_vel 限幅 + 0.5 s 看门狗。
`src/control/wbmm_ocs2_ros/src/WbmmMrtNode.cpp` 加了底盘硬限速和策略窗口告警。

---

## 5. 必读：两份复核文档

**这两份比任何代码都优先。**

| 文档 | 作者 | 内容 |
| --- | --- | --- |
| `docs/remani_migration_review.md` | 迁移执行方（我） | 交付物、验证命令、设计判断、已知限制 |
| `docs/remani_migration_independent_review.md` | **独立复核方** | 源码复核 + 本轮修复 |

**独立复核的核心结论（你应该先看这段）**：

> 分包方向合理：保留 ROS-free 搜索、轨迹整形、规划编排，ROS 节点负责消息、
> 地图/模型加载与 TF，参考桥独立于 OCS2 求解器。这一结构值得保留。
>
> 但当前实现是一个重新设计的、静态地图下的静止到静止导航规划器，
> **不能称为 REMANI 功能等价迁移**。主链没有调用 `WholeBodyOptimizer`；
> 现有单向逐点 Halton 臂种子、固定时长 MINCO 整形，
> 也不等价于原来的机械臂 RRT* 与联合轨迹优化。

**这句话决定了你读代码时的心态**：这是一次**重新设计**，不是等价移植。
如果你要的是 REMANI 的功能等价，还有工作要做。

---

## 6. 边读边验证（约 30 分钟）

### 6.1 跑测试

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=/home/a/WBMM/.roslog && mkdir -p .roslog
colcon build --packages-select \
  wbmm_planning_msgs wbmm_environment wbmm_collision wbmm_search \
  wbmm_traj_opt wbmm_planner wbmm_planner_ros wbmm_reference_bridge \
  wbmm_ocs2_ros tracer_jaka_bringup
# 期望：10 packages finished
```

单测（期望 113 通过 / 0 失败）：

```bash
for t in build/wbmm_*/test_*; do
  [ -x "$t" ] && printf "%-46s %s\n" "$(basename $t)" \
    "$(./$t 2>&1 | grep -oE '\[  PASSED  \] [0-9]+' | grep -oE '[0-9]+')"
done
```

**注意**：全量 `colcon build` 会在 `cgal5_colcon` 失败（CGAL ExternalProject 下载步骤，
沙箱无网络），与本次改动无关。

### 6.2 跑端到端

```bash
source install/setup.bash
ros2 launch tracer_jaka_bringup wbmm_tracking.launch.py \
  viewer:=false use_rviz:=false auto_goal:=1.0,0.0
```

期望：规划器打印 `Planned N samples ...`，底盘从 x≈0 移动到 x≈0.9。

**已知现象**：前几次目标可能被拒（`Ignoring goal: waiting for odom and joint states`），
自动目标会重试。这是**未查清的间歇性问题**，不是设计如此。

### 6.3 ROS-free 核验

```bash
for p in wbmm_planner search wbmm_traj_opt wbmm_collision wbmm_environment; do
  printf "%-20s %s\n" "$p" \
    "$(grep -rn 'rclcpp\|ROS_\|#include <ros' src/**/$p/{include,src} 2>/dev/null | wc -l)"
done
# 期望：全部 0
```

---

## 7. 建议的阅读节奏

如果只有**半天**：

1. 第 5 节两份复核文档（30 分钟）—— 先建立「这是什么、不是什么」
2. 第 1 节契约（20 分钟）
3. 2.4 的 `whole_body_trajectory_builder.cpp` + 2.5 `whole_body_planner.cpp`（1 小时）
   —— 这两块是**实际在跑**的主链
4. 第 6 节验证（40 分钟）
5. 2.4 的 `whole_body_optimizer.hpp` 只看 scope note（15 分钟）—— 它**没接入主链**

如果有**两天**：按第 2 节完整顺序读，每读完一个包跑它的单测。

**不要先读**：`minco.hpp`（1626 行，移植自上游，只在需要理解系数约定时查）、
`lbfgs.hpp` / `root_finder.hpp`（逐字复制）。

---

## 8. 读完应该能回答的问题

用来自查是否真的重建了心智模型：

1. 从 `/goal_pose` 到 `MpcTargetTrajectories`，中间经过哪几个进程、哪几个包？
2. 为什么优化器的决策变量里没有 yaw？
3. 为什么航向基线用弧长而不是样本数？不用会怎样？
4. 为什么 ESDF 的未观测空间不直接拒绝？
5. `WholeBodyOptimizer` 为什么不在主链里？它在等什么？
6. 如果换一张新地图，哪些假设需要重新确认？
7. `wbmm.launch.py` 的 `bridge_world_frame` 默认值有什么问题？

---

## 9. 未提交状态的处理建议

51 个未提交改动、没有任何检查点，这对后续阅读和回退都不利。建议：

1. **先清理临时产物**：`rm -rf .roslog MUJOCO_LOG.TXT`
2. **决定 vendor 的去留**：`src/vendor/remani_planner/COLCON_IGNORE` 已加，
   但目录仍在（含 09-24 对 `remani_mpc_tracking.launch.py` 的改动，现已失效）
3. **分几次提交**，建议按层拆：
   - `wbmm_planning_msgs` + `wbmm_reference_bridge`（契约与桥迁出）
   - `wbmm_collision` + `wbmm_environment` 新增（算法基础）
   - `wbmm_search` + `wbmm_traj_opt`（搜索与整形）
   - `wbmm_planner` + `wbmm_planner_ros`（编排与节点）
   - launch/config/vendor 清理（集成）
4. **提交信息里写清「这是重新设计而非等价迁移」**，避免后来者误判
