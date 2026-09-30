# wbmm_collision

## 自碰撞检测

`trajectory_spheres.cpp` 计算共享碰撞球模型在轨迹变量
`z = [x, y, vx, vy, q_1..q_n]` 下的球心和解析 Jacobian。
`self_collision_checker.cpp` 消费这些结果，负责球对筛选、净间距、
碰撞判定及净间距对 `z` 的解析梯度，不重复执行正运动学。

```cpp
#include <wbmm_collision/self_collision_checker.hpp>

const auto spheres = wbmm::collision::evaluateTrajectorySpheres(
  description, sphere_model, base_position_xy, base_velocity_xy,
  gear, joint_angles, held_yaw);
const auto result = wbmm::collision::checkSelfCollision(spheres, safety_margin);
```

- 沿用现有规划器规则：检查全部臂球/底盘球，以及组序号相差至少 2 的
  臂球/臂球；排除底盘内部、同一臂组和相邻臂组球对。
- `clearance = |p_a - p_b| - r_a - r_b - safety_margin`；
  `clearance <= 0` 返回 `kCollision`，包含接触安全裕量边界的情况。
- 梯度为 `(J_a - J_b)^T (p_a - p_b) / |p_a - p_b|`。
  球心重合时仍报告碰撞，梯度取零，因为此处距离没有唯一导数。
- `pairs` 包含所有参与检测的球对；`min_clearance` 和最近球对 ID 用于诊断。
  无可检查球对时返回 `kFree`、空球对列表和正无穷最小净间距；
  输入错误时返回 `kInvalidInput`、空球对列表和 NaN。

`wbmm_planner` 的 `WholeBodyOptimizer` 已使用此接口计算自碰撞代价，
保留原有权重和三次惩罚函数。此接口检查单个采样配置；轨迹采样策略由
调用者管理，目前未更改规划器的最终 `safetySweep()` 检查范围。
