# REMANI 上游迁移 —— 审阅与复核材料

> Status: DRAFT
> Author: Agent
> Reviewer: Codex（源码独立复核；非人工/硬件验收）
> Reviewed at: 2026-09-24
> Warning: 本文档尚未经过人工审查，不能作为实现依据。

> 本文档是 REMANI → WBMM 迁移任务的交接材料，供独立审阅与复核使用。
> 文档区分三类内容：**已验证事实**（附可复现命令）、**设计判断**（需要审阅者质疑）、
> **已知限制**（明确未验证或未完成的部分）。审阅者应重点质疑第二类和第三类。

---

## 0. 独立复核更新（2026-09-24）

下文保留原迁移交接记录。当前结论与本轮修复见
[独立复核与修复记录](remani_migration_independent_review.md)。
重点纠正：第 4.2 节弧长航向不保证差速运动学；第 5.5 节关于上游只采样到 1 秒的推断被源码中的 `s1 += T/K` 否定；第 5.4 节遗漏了当前测试的 `fine < 2e-2` 判据。
原第 3.4 节的纯消息测试现在需加 `bridge_world_frame:=map`，否则须提供 map→odom TF。
本轮没有宣称完整 REMANI 功能等价、MuJoCo 闭环或真机验收通过。

---

## 1. 任务目标

原始要求（用户原话）：

1. 把 `vendor/remani_planner` 的核心算法迁移进 WBMM，使其不再依赖 REMANI 上游
2. 核心算法包必须 ROS-free，只依赖 `wbmm_core` / `wbmm_environment` / `wbmm_collision`
   等通用契约，风格对齐已迁移的 `wbmm_search::KinoAstar`
3. ROS 部分只做薄节点封装
4. 把 `remani_to_ocs2_reference_bridge.cpp` 从 `wbmm_ocs2_ros` 迁出到合适的包并重命名
5. 重接 launch/config
6. 移除对 `vendor/remani_planner` 的构建依赖

用户在执行过程中给过两条重要放宽：

- 「有些必要的时刻，比如说方便打开 urdf 读取数据什么的，你还是可以使用 ros 相关功能包，只不过是尽量不要使用」
- 「没必要完全照搬 remain 的设计，我们只是参考和学习，结合目前仓库里已有的代码和接口，
  如果有更好的设计，那我们也可以重新设计一下」

---

## 2. 交付物清单

### 2.1 新建包

| 路径 | 文件数 | 行数 | 性质 |
| --- | --- | --- | --- |
| `src/planning/wbmm_planning_msgs/` | 3 | — | 消息契约（ROS） |
| `src/planning/optimization/wbmm_traj_opt/` | 11 | 5732 | **ROS-free** |
| `src/planning/wbmm_planner/` | 3 | 660 | **ROS-free** |
| `src/planning/wbmm_planner_ros/` | 2 | 488 | ROS 薄封装 |
| `src/control/wbmm_reference_bridge/` | 4 | 912 | ROS 薄封装（迁出重命名） |

行数由 `find <pkg> \( -name "*.cpp" -o -name "*.hpp" \) -exec cat {} + | wc -l` 统计。

### 2.2 扩展的既有包

| 路径 | 新增内容 |
| --- | --- |
| `src/robotics/wbmm_collision/` | `urdf_collision_model.{hpp,cpp}`、`whole_body_kinematics.{hpp,cpp}` + 测试 |
| `src/planning/search/` | `arm_seed_search.{hpp,cpp}` + 测试 |
| `src/map/wbmm_environment/` | `MapInfo::unknown_is_occupied`、`DistanceQuery::fully_observed` |

### 2.3 消息契约

`src/planning/wbmm_planning_msgs/msg/WholeBodyTrajectory.msg` 定义**采样轨迹**（非多项式系数）：

```text
std_msgs/Header header
string trajectory_id
uint64 environment_revision
uint64 collision_model_revision
float64[] time_from_start
string[] joint_names
float64[] base_x
float64[] base_y
float64[] base_yaw
float64[] base_linear_velocity
float64[] base_yaw_rate
float64[] joint_positions          # index = sample * joint_count + joint
float64[] joint_velocities
uint8[] phase
```

**设计判断（需审阅）**：REMANI 的 `quadrotor_msgs/PolynomialTraj` 传多项式系数，
这里改为传采样点。理由是新契约对控制器更直接、且不绑定 MINCO 这一种参数化。
代价是消息体积更大（76 个采样点 × 8 维）。

### 2.4 launch 改动

- 新增 `src/bringup/launch/wbmm_planning.launch.py`（启动 planner + bridge）
- 删除 `src/bringup/launch/remani.launch.py`（vendor 包装，vendor 被排除后必然运行时报错）
- `wbmm.launch.py` / `remani_mpc.launch.py` / `remani_mpc_localized.launch.py`
  的 remani 分支改为 include `wbmm_planning.launch.py`
- `start_remani` 保留为废弃别名，与新的 `start_planning` 等效

---

## 3. 独立验证方法

以下命令可直接复现本文档的验证结论。前置：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=/home/a/WBMM/.roslog    # 沙箱内 ~/.ros 只读，必须覆盖
mkdir -p .roslog
```

### 3.1 构建

```bash
colcon build --packages-select \
  wbmm_planning_msgs wbmm_environment wbmm_collision wbmm_search \
  wbmm_traj_opt wbmm_planner wbmm_planner_ros wbmm_reference_bridge \
  wbmm_ocs2_ros tracer_jaka_bringup
# 期望：Summary: 10 packages finished
```

**注意**：全量 `colcon build` 会在 `cgal5_colcon` 失败。那是
`src/vendor/ocs2_ros2/submodules/plane_segmentation_ros2/cgal5_colcon` 里 CGAL 的
`ExternalProject` 卡在 `_ep_add_download_command`（下载步骤），属沙箱无网络导致的既有环境问题，
与本次改动无关。审阅者若在有网络环境复核，此项应自行消失。

### 3.2 单元测试

```bash
for t in build/wbmm_collision/test_whole_body_kinematics \
         build/wbmm_collision/test_urdf_collision_model \
         build/wbmm_collision/test_environment_collision_checker \
         build/wbmm_collision/test_search_collision_integration \
         build/wbmm_search/test_kino_astar \
         build/wbmm_search/test_arm_seed_search \
         build/wbmm_traj_opt/test_minco \
         build/wbmm_traj_opt/test_whole_body_trajectory_builder \
         build/wbmm_traj_opt/test_whole_body_optimizer \
         build/wbmm_planner/test_whole_body_planner \
         build/wbmm_environment/test_environment_skeleton \
         build/wbmm_reference_bridge/test_trajectory_sampler; do
  echo "== $(basename $t)"; ./$t 2>&1 | tail -1
done
```

**期望合计 113 通过、0 失败**，分布为：

| 测试 | 数量 |
| --- | --- |
| `test_kino_astar` | 34 |
| `test_trajectory_sampler` | 12 |
| `test_environment_collision_checker` | 11 |
| `test_arm_seed_search` | 10 |
| `test_whole_body_planner` | 8 |
| `test_whole_body_trajectory_builder` | 7 |
| `test_whole_body_optimizer` | 7 |
| `test_environment_skeleton` | 7 |
| `test_urdf_collision_model` | 6 |
| `test_whole_body_kinematics` | 5 |
| `test_minco` | 5 |
| `test_search_collision_integration` | 1 |

### 3.3 launch 测试

```bash
source install/setup.bash
python3 -m pytest src/bringup/test/ -q -p no:anyio -p no:cacheprovider
# 期望：32 passed
```

`-p no:anyio` 是必需的：本机 `anyio` 的 pytest 插件与 `_pytest.scope` 不兼容，
不禁用会直接 ImportError，与本次改动无关。

### 3.4 端到端验证（核心验收）

```bash
source install/setup.bash
ros2 launch tracer_jaka_bringup wbmm_planning.launch.py \
  urdf_file:=$PWD/src/robotics/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf \
  esdf_file:=$PWD/maps/map1/site_remani.npz &
sleep 10
python3 src/bringup/test/planning_pipeline_check.py 1.0 0.0
```

**期望输出**：

```text
reference: 76 samples, t=[...], start x=-0.000, yaw=0.000, v=0.000, w=0.000
PASS: planner -> bridge produced a valid OCS2 reference window
```

该脚本自己发布 odom / joint_states / `MpcObservation`，发布 `/goal_pose`，
然后校验 `mobile_manipulator_whole_body_target` 上的 `MpcTargetTrajectories`
（数组长度一致、时间严格递增、9 维状态、8 维输入）。76 = 3.0 s / 0.04 s。

### 3.5 ROS-free 核验

```bash
for p in src/planning/wbmm_planner src/planning/search \
         src/planning/optimization/wbmm_traj_opt \
         src/robotics/wbmm_collision src/map/wbmm_environment; do
  printf "%-24s %s\n" "$(basename $p)" \
    "$(grep -rn 'rclcpp\|ROS_\|#include <ros' $p/include/ $p/src/ 2>/dev/null | wc -l)"
done
# 期望：5 行全部为 0
```

### 3.6 vendor 依赖已移除

```bash
test -f src/vendor/remani_planner/COLCON_IGNORE && echo "COLCON_IGNORE 存在"
colcon list | grep -cE "remani_planner|quadrotor_msgs|traj_utils|plan_manage|plan_env|mm_config|path_searching"
# 期望：0
grep -c remani_planner src/bringup/package.xml
# 期望：0
test -f src/control/wbmm_ocs2_ros/src/remani_to_ocs2_reference_bridge.cpp && echo "旧桥仍在" || echo "旧桥已删除"
```

---

## 4. 需要审阅者重点质疑的设计判断

以下是我在迁移中做的**重新设计**（而非照搬 REMANI），每一项都值得质疑。

### 4.1 航向由速度方向重建，而非作为优化变量

底盘非完整，`yaw = atan2(gear·vy, gear·vx)`。优化器的变量布局因此是
`[x, y, vx, vy, q_1..q_6]`，**yaw 不是变量**。好处是优化器可以通过改变速度来转向；
代价是近停时航向病态（见 5.1）。

### 4.2 轨迹构建器的航向基线用**弧长**，不是样本数

这是迁移中最重要的发现。三种做法的实测角速度：

| 航向估计方式 | 峰值 ω |
| --- | --- |
| 瞬时速度方向（REMANI 做法） | **14.06 rad/s** |
| 固定样本数窗口的弦向 | 4.52 rad/s |
| **固定弧长的弦向** | **0.112 rad/s** |

原因：MINCO 在每个内部路径点处 x 速度会下陷，固定样本窗口在那里只跨过极短距离
（弦长从 0.041 缩到 0.011 m），方向估计被噪声主导。

### 4.3 全身臂种子搜索用确定性 Halton 序列，不是 RRT*

REMANI 用 RRT* + `sample_mani_RRT`（2484 行）。我改成按路径点推进的确定性 Halton 采样
（`arm_seed_search.cpp`），理由是：可复现、无随机失败、且与 WBMM 的
`SearchResult{base_path, arm_seed}` 契约天然契合。**代价是完备性弱于 RRT\***，
狭窄通道场景可能找不到种子。这是需要审阅者判断的取舍。

### 4.4 MINCO 隔点穿过路径点

`PlannerConfig::minco_waypoint_stride`（默认 3）。逐个穿过会让 MINCO 忠实复现
KinoAstar 粗搜索的高频抖动。**代价是碰撞保证变弱** —— 臂种子在全分辨率计算，
但 MINCO 只在抽稀后的控制点上受约束。

### 4.5 时间缩放循环

七次多项式的峰值因子 ≈ 2.19×平均速度，所以按平均速度分配时间必然超包络。
`whole_body_planner.cpp` 里有 8 次尝试的时间缩放循环，作用在**实际发布的参考**上，
并受 `max_trajectory_duration` 约束。

### 4.6 安全扫描的职责划分

`wbmm_traj_opt` 的 `safetySweep` 只对**碰撞**判不安全。包络超出只记录在 `message` 里，
因为包络项是软惩罚，且硬性包络由编排层的时间缩放负责。**审阅者应质疑这个划分是否合理。**

### 4.7 `world_frame` 自动采纳

planner 在未指定时采纳 ESDF 自己的 `frame_id`，bridge 采纳 planner 的 frame。
理由：这两者本来就必须一致，去掉一个配错的机会。bridge 仍强制 frame 不得中途改变。

---

## 5. 已知限制（明确未完成或未验证）

### 5.1 优化器横向（y）分量梯度病态 —— **最重要的一项**

`wbmm_traj_opt/test_whole_body_optimizer.cpp` 的
`TransverseGradientIsIllConditioned` 测试记录了这个限制。

现象：测试轨迹沿 +x 直线运动时 `vy ≈ 0`，航向灵敏度 `∂yaw/∂vy = vx/|v|²` 爆炸。
实测横向分量相对误差约 **2.3**（回归护栏设在 4.0）。

我尝试过的修法及其**失败证据**：

- 给 `evaluateWholeBodyKinematics` 加 `held_yaw` 参数，在低速时保持上一次可信航向。
  实测让主有限差分误差从 **0.21 变差到 0.48**，因为保持值来自同样近停的相邻样本。
  该 API 保留（是有用的能力）但默认关闭，`heading_hold_speed = -1.0`。
- 扫描保持阈值 0.05 / 0.01 / 0.001 / 0.0001，全部比不保持更差。

**真正的修法**（已写入 `whole_body_optimizer.hpp` 的 scope note，未实现）：
用弧长弦估计航向，也就是 4.2 中轨迹构建器已经采用的方法。

### 5.2 优化器**尚未接入编排层**

`wbmm_planner` 目前走的是 KinoAstar + 臂种子 + MINCO 整形 + 时间缩放，
**没有调用 `WholeBodyOptimizer`**。优化器本身 7/7 测试通过，但不在控制回路里。
这是我在原始要求之外追加的第三块，是否接入由用户决定。

### 5.3 时长优化被移出决策向量

决策向量只有内点。原因：MINCO 的显式时长导数与 KKT 伴随传播之间的交互我没能验证正确，
而未经证实的时序梯度比没有更糟。已写在头文件 scope note 里，不是静默省略。

### 5.4 有限差分测试用收敛性而非绝对容差

`MatchesFiniteDifferences` 断言「步长缩小 4 倍，误差必须下降」而不是「误差 < 某阈值」。
理由是目标函数在近停处高阶导数很大，任何固定容差都不现实。实测：

```text
[converge] step 4h -> 1, step h -> 0.006409 (index 0)
```

**审阅者应质疑这个判据是否足够强。** 我的理由是符号错误或漏项不会收敛。

### 5.5 REMANI 上游疑似 bug（未上报）

`poly_traj_optimizer.cpp::addPVAJGradCost2CT` 用 `s1 ∈ [0,1]` 配合 `step = T/K` 采样，
而 `Piece::getPos` 表明系数是**真实时间**多项式系数。这意味着每段多项式只在
`[0, min(T,1)]` 秒内被采样：T>1 s 的段会漏掉大部分，T<1 s 的段会采样到段外。
**这是我在阅读上游代码时的推断，没有实证。** 我自己的实现按 `τ = s·T` 采样。

### 5.6 未验证项

- 未在真机或 MuJoCo 全闭环中运行新的 `wbmm_planner_ros`（只验证到发布参考为止）
- 未验证优化器在真实障碍场景下的规划质量（只在合成算例上测过）
- `remani_mpc.launch.py` / `remani_mpc_localized.launch.py` 只做了静态解析测试
  （`test_common_launch.py`），未实际启动

---

## 6. 过程中发现并修复的缺陷

按发现顺序，附证据。

### 6.1 原机器人「乱飞」的根因链

这是任务开始时的原始问题。已确认的因果链：

1. REMANI 参考峰值 0.60 m/s > OCS2 限制 0.5 m/s
2. 参考 ω 高达 ~4.4 rad/s（`ω ∝ 1/t`，t=0.05 s 时），控制器限制 1.0 rad/s
3. 跟踪误差累积到 0.907 m（模型复现 `3.31 s ⇒ L = 0.908 m`，与日志 0.907 m 吻合）
4. REMANI 目标重规划在 0.40 m 触发后失败（`KinoAstar: start (0) is not free` 等）
5. `planEnd = mpcInitObservation.time + solutionTimeWindow`，所以
   `[SAFETY] MPC plan expired` 等价于 `plan_age > 0.2 s`，而 MPC 求解时间尖峰到 224–500 ms
6. 全链路无底盘限速；MuJoCo `exact_base=true` 直接把 (v,ω) 积分进 qpos，无看门狗
7. `arm_max_delta_per_step: 0.50` rad/step @125 Hz ≈ 62.5 rad/s ≈ 无限速

已施加的修复：`WbmmMrtNode.cpp` 底盘硬限速 + `publishHoldArmCommand()` 保持**实测**臂位；
`mujoco_bridge_node.py` 限速 + 0.5 s 看门狗；`ocs2.yaml` / `remani_tracking.yaml` 参数收紧。

### 6.2 ESDF 未观测空间会阻塞 ~89% 查询

实测部署地图 `maps/map1/site_remani.npz`：shape `(201,169,73)`、voxel_size **0.1**、
origin `(-11.2,-8.8,-3.2)`、frame `map`。`observed` 仅 **12.34%**，
八角全观测比例 **10.55%**，占据率 3.84%，`unknown_is_occupied = False`。
**100% 的未观测体素 `esdf == +2.0`，且零个未观测体素是占据的。**

因此按 `observed` 硬拒绝会破坏约 89% 的查询。改为 `query()` 返回 `fully_observed`
标志而非 `kUnknown`。

### 6.3 航向保持分支误清零速度（真 bug）

`whole_body_trajectory_builder.cpp` 的航向保持分支里我原本同时把 `forward_velocity` 清零。
轨迹一变慢，弦长就低于保持阈值 → 速度被全部清零 → 调用方的包络检查**平凡通过** →
时间缩放循环「假收敛」到一条静止轨迹（314 秒）。MINCO 本身在两端已经自然趋零，
速度不该被航向逻辑影响。

### 6.4 首个样本的航向未与种子比较

原来只在样本之间比较航向步长，导致**档位翻转的 π 跳变恰好藏在第一个样本**
（它相对种子而非相对前一个样本）。已修。

### 6.5 自碰撞项梯度两个符号都写反

`d(cost)/d(a.pos) = 3w·shortfall²·(−direction)` 写成了 `+`，`d/d(b.pos)` 写成了 `−`。
修正后 `ReducesTheCost` 通过。

### 6.6 优化器求积权重丢失（最隐蔽的一个）

```cpp
sample_cost += config_.feasibility_weight * term;   // 漏了 weight * step
...
cost += sample_cost;
```

**代价是未加权的逐样本求和，梯度却用辛普森加权积分**，两者相差 4.9 倍。

定位方法（可复现）：把梯度链切开，绕过 `getGrad2TP`，直接用 `gdC · (dc/dx)`
（`dc/dx` 用数值法求）：

```text
[split] idx= 2  gdC.dcdx=  651.284784  cost_fd=  911.226223  adjoint=  651.284784
```

`gdC·dc/dx` 与伴随结果逐位相同 → 伴随求解没问题；但按链式法则它**必须**等于 `cost_fd`。
矛盾只有一种解释：代价本身的结构与预期不符。修复后梯度吻合到 **9 位有效数字**。

### 6.7 `wbmm_planning_msgs` 的 Python 绑定被静默跳过

本工作区其他包会预置空的 `PYTHON_EXECUTABLE`，导致 `rosidl_generator_py` 的
`FindPythonInterp` 失败但不报错。加了与 `wbmm_ocs2_ros` 相同的显式解释器发现。

### 6.8 ROS 节点只填了关节位置

`jointCallback` 只填 `positions`，未填 `names`/`velocities`，
而规划器契约要求三者尺寸一致，缺一个就整体被拒。已修。

---

## 7. 需要清理的项

本次会话产生了以下未跟踪的临时产物，**建议删除**：

```bash
rm -rf /home/a/WBMM/.roslog /home/a/WBMM/MUJOCO_LOG.TXT
```

`src/vendor/remani_planner/plan_manage/launch/remani_mpc_tracking.launch.py`
在早期轮次被修改过（加入参考速度限幅）。vendor 现已 `COLCON_IGNORE`，
该改动不再生效，但仍是工作区里的未提交修改。审阅者需决定是保留、还原还是随 vendor 一并移除。

---

## 8. 审阅建议的优先级

如果时间有限，建议按此顺序：

1. **5.2** —— 优化器未接入，确认这是否符合预期
2. **4.3 / 4.4** —— 臂种子搜索与 MINCO 抽稀的取舍是否可接受
3. **5.1** —— 横向梯度病态是否会实际影响规划质量
4. **5.4** —— 收敛性判据是否足够强
5. **4.6** —— 安全扫描的职责划分
6. 其余设计判断
