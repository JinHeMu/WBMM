# WBMM OCS2 改造设计：双参考阶段权重、可操作度、模式切换与 ESDF 碰撞后端

> Status: DRAFT  
> Scope: `wbmm_ocs2` / `wbmm_ocs2_ros` / `wbmm_environment`  
> 目标：在不修改 OCS2 vendor 核心的前提下，实现四项需求  
> 说明：本文只给设计方案和修改方法，未修改源码

---

## 核心设计修订：双参考 + 阶段权重 + 外部模式消息

> 本节是当前推荐的核心设计，优先于后文旧版“单一 `_mpc_target` + 模式硬切”的描述。后文第 2、3、5 节仍可作为具体实现细节参考；第 4 节已按本节思想重写。

### 核心思想

1. **target 分为两路，同时维护**
   - `/mobile_manipulator_whole_body_target`：9D 全身参考，来自 REMANI / 轨迹规划器。
   - `/mobile_manipulator_ee_target`：7D 末端参考，来自交互 marker / 视觉 / 任务规划器。
   - 两个参考可以同时存在，不需要互相停发。

2. **外部消息切换阶段**
   - `/mobile_manipulator_task_phase`：`std_msgs/msg/Int32`。
   - `0 = Navigation`，`1 = Transition`，`2 = Execution`，`3 = Retract`。
   - `WbmmMpcNode` 收到后调用 `WbmmReferenceManager::setTaskPhase()`。
   - `WbmmMrtNode` 也订阅同一消息，用于 reset 和 policy 阶段检查。

3. **不同阶段，两路参考的代价权重不同**
   - 不采用简单的“二选一硬切”，而是加权和：

$$
L = w_{wb}(s)L_{wb} + w_{ee}(s)L_{ee} + L_{reg}
$$

其中 `s` 是当前阶段。

| 阶段 | `w_wb` | `w_ee` | 说明 |
|---|---:|---:|---|
| Navigation | 1.0 | 0.0 | 全身参考主导 |
| Transition | 1.0 → 0.0 | 0.0 → 1.0 | 平滑过渡，两个代价同时存在 |
| Execution | 低/仅底盘与姿态 | 1.0 | 末端参考主导，全身只做正则 |
| Retract | 0.0 → 1.0 | 1.0 → 0.0 | 切回全身或回撤 |

4. **始终开启的公共代价/约束**
   - 输入代价
   - 关节位置/速度限位
   - 自碰撞
   - 环境碰撞（ESDF backend）
   - 可操作度/奇异值代价
   - 底盘/姿态正则

### 推荐数据流

```text
/mobile_manipulator_whole_body_target  9D
                    \
                     \
/mobile_manipulator_ee_target         7D
                     \
                      +--> WbmmReferenceManager
/mobile_manipulator_task_phase  Int32 --> |  whole_body_target
                                           |  ee_target
                                           |  task_phase
                                           |
                                  PhaseWeightedStateCost
                                           |
                    +----------------------+----------------------+
                    |                                             |
        w_wb(phase) * WholeBodyTrackingCost       w_ee(phase) * EndEffectorTrackingCost
                    |                                             |
                    +----------------------+----------------------+
                                           |
                                 always-on costs:
                                 input / limits / collision / manipulability
                                           |
                                        OCS2 OCP
```

### 与旧版方案的关系

- 旧版：一个 `_mpc_target`，导航模式发 9D，执行模式发 7D，通过 `isActive` 二选一。
- 新版：两个独立 target 话题，同时维护；外部 phase 消息切换阶段；两个 cost 都注册，按阶段权重加权。
- 新版更符合“导航跟踪用全身参考、执行用末端参考”的任务语义，也避免目标维度混用。

---

## 0. 阅读结论（CURRENT）

### 0.1 当前 OCP 组装位置

`src/control/wbmm_ocs2/src/WbmmInterface.cpp`：

- `WbmmInterface::WbmmInterface()`：`76-307`
  - 输入代价：`193`
  - 末端软约束：`209-222`
  - 全身跟踪代价：`233-253`
  - 自碰撞：`255-264`
  - body-relative：`266-274`
  - 环境碰撞：`276-285`
  - dynamics / precomputation / rollout / initializer：`287-305`
- `getWholeBodyTrajectoryCost()`：`339-389`
- `getEndEffectorConstraint()`：`391-518`
- `getSelfCollisionConstraint()`：`520-610`
- `getEnvironmentCollisionConstraint()`：`797-863`
- `loadInitialObstacles()`：`865-955`

当前模式逻辑：

```cpp
endEffector.activate        // 7D EE pose soft constraint
wholeBodyTracking.activate  // 9D whole-body state cost
```

二者只能开一个，否则 `WbmmInterface.cpp:237-243` 直接抛异常。

### 0.2 当前末端跟踪

`src/control/wbmm_ocs2/src/constraint/EndEffectorConstraint.cpp`：

- 单臂 6 维约束：`getNumConstraints()` 返回 6。
- `interpolateEndEffectorPose()` 直接假设目标为 7D：

```cpp
position = stateTrajectory.front().head<3>();
orientation = quaternion_t(stateTrajectory.front().tail<4>());
```

如果模式切换时目标还是 9D，语义错误；如果目标维度不足 7，可能越界。

### 0.3 当前环境碰撞后端

`src/control/wbmm_ocs2/src/collision/EnvironmentGeometryInterface.cpp`：

- 使用 `ocs2::PinocchioGeometryInterface` + coal/FCL。
- `computeDistances()`：`190-252`
- 返回最近点、机器人几何索引、joint index。
- `EnvironmentCollisionConstraint.cpp:65-138` 用最近点方向构造距离梯度。

当前是 **coal geometry backend**，不是 ESDF backend。

### 0.4 ESDF 现状

`src/map/wbmm_environment` 目前是骨架：

- `EsdfGrid::query()`：`src/esdf_grid.cpp:12`，返回 `kNotImplemented`。
- `NpzEsdfLoader::load()`：`src/npz_esdf_loader.cpp:6`，返回 `kNotImplemented`。

实际 nvblox 导出格式已经在：

- `src/map/my_nvblox_bringup/my_nvblox_bringup/nvblox_map_exporter.py:364-398`
- REMANI `GridMap` 的静态 ESDF 加载和查询实现可参考：
  - `src/vendor/remani_planner/plan_env/src/grid_map.cpp:1-425`
  - `src/vendor/remani_planner/plan_env/src/grid_map.cpp:2088-2186`

导出 NPZ 字段包括：

```text
esdf.npy
occupancy.npy
observed.npy
origin.npy
voxel_size.npy
bounds_max.npy
frame_id.npy
```

---

## 1. 总体设计方案

> 修订：推荐采用“双参考 + 阶段权重 + 外部 phase 消息”，见文首“核心设计修订”。本节下面的图已按新版更新。

```text
/mobile_manipulator_whole_body_target  9D
                    \
                     \
/mobile_manipulator_ee_target         7D
                     \
                      +--> WbmmReferenceManager
/mobile_manipulator_task_phase  Int32 --> |  whole_body_target
                                           |  ee_target
                                           |  task_phase
                                           |
                                  PhaseWeightedStateCost
                                           |
                    +----------------------+----------------------+
                    |                                             |
        w_wb(phase) * WholeBodyTrackingCost       w_ee(phase) * EndEffectorTrackingCost
                    |                                             |
                    +----------------------+----------------------+
                                           |
                                 always-on costs:
                                 input / limits / collision / manipulability
                                           |
                                        OCS2 OCP

环境：
NPZ ESDF -> NpzEsdfLoader -> EsdfGrid::query
          -> EsdfEnvironmentInterface
          -> EsdfEnvironmentCollisionConstraint
```

建议新增：



```text
src/control/wbmm_ocs2/include/wbmm_ocs2/WbmmReferenceManager.h
src/control/wbmm_ocs2/include/wbmm_ocs2/cost/PhaseWeightedStateCost.h
src/control/wbmm_ocs2/include/wbmm_ocs2/cost/ArmManipulabilityCost.h
src/control/wbmm_ocs2/src/cost/ArmManipulabilityCost.cpp
src/control/wbmm_ocs2/include/wbmm_ocs2/cost/EndEffectorTrackingCost.h
src/control/wbmm_ocs2/src/cost/EndEffectorTrackingCost.cpp
src/control/wbmm_ocs2/include/wbmm_ocs2/collision/EsdfEnvironmentInterface.h
src/control/wbmm_ocs2/src/collision/EsdfEnvironmentInterface.cpp
src/control/wbmm_ocs2/include/wbmm_ocs2/constraint/EsdfEnvironmentCollisionConstraint.h
src/control/wbmm_ocs2/src/constraint/EsdfEnvironmentCollisionConstraint.cpp
```

并修改：

```text
src/control/wbmm_ocs2/include/wbmm_ocs2/WbmmInterface.h
src/control/wbmm_ocs2/src/WbmmInterface.cpp
src/control/wbmm_ocs2/CMakeLists.txt
src/control/wbmm_ocs2/package.xml

src/map/wbmm_environment/src/esdf_grid.cpp
src/map/wbmm_environment/src/npz_esdf_loader.cpp
src/map/wbmm_environment/CMakeLists.txt
src/map/wbmm_environment/package.xml

src/control/wbmm_ocs2_ros/src/WbmmMpcNode.cpp
src/control/wbmm_ocs2_ros/src/WbmmMrtNode.cpp
src/control/wbmm_ocs2_ros/src/WbmmTargetNode.cpp
src/control/wbmm_ocs2_ros/src/remani_to_ocs2_reference_bridge.cpp

src/bringup/config/sim/task.info
src/bringup/config/real/task.info
src/bringup/config/common/ocs2.yaml
src/bringup/config/sim/ocs2.yaml
src/bringup/config/real/ocs2.yaml
```

---

## 2. 需求 1：加入机械臂操作度、奇异值等代价

### 2.1 指标定义

设末端任务 Jacobian 为 `J`。

- 机械臂模式：

$$
J_{\rm arm}=J_{\rm ee}[:,3:9] \in \mathbb{R}^{6\times 6}
$$

- 全身模式：

$$
J_{\rm whole}=J_{\rm ee}[:,0:3]B(\psi)+J_{\rm ee}[:,3:9] \in \mathbb{R}^{6\times 8}
$$

其中

$$
B(\psi)=
\begin{bmatrix}
\cos\psi & 0\\
\sin\psi & 0\\
0 & 1
\end{bmatrix}
$$

建议使用的指标：

- Yoshikawa 操作度：

$$
w=\sqrt{\det(JJ^T)}=\prod_i \sigma_i
$$

- 最小奇异值：

$$
\sigma_{\min}=\min_i \sigma_i
$$

- 最大奇异值 / 条件数：

$$
\sigma_{\max}=\max_i \sigma_i,\qquad
\kappa=\frac{\sigma_{\max}}{\sigma_{\min}}
$$

- 任务方向可操作度：

$$
\rho_d=\sqrt{d^TJJ^Td}
$$

- 逆操作度/近奇异代理：

$$
m_{\rm inv}=\operatorname{tr}\left((JJ^T+\epsilon I)^{-1}\right)
$$

### 2.2 代价形式

建议不要直接写线性代价 `-σ_min`，因为可能带来负曲率和数值问题。推荐一侧二次 hinge：

$$
L_{\sigma}=\frac12 w_{\sigma}\max(0,\sigma_{\rm ref}-\sigma_{\min})^2
$$

$$
L_w=\frac12 w_w\max(0,w_{\rm ref}-w)^2
$$

$$
L_{\kappa}=\frac12 w_{\kappa}\max(0,\kappa-\kappa_{\max})^2
$$

$$
L_{\rm inv}=w_{\rm inv}m_{\rm inv}
$$

总代价：

$$
L_{\rm manip}=L_{\sigma}+L_w+L_{\kappa}+L_{\rm inv}
$$

### 2.3 新增 `ArmManipulabilityCost`

建议接口：

```cpp
struct ArmManipulabilitySettings {
  bool activate = false;
  enum class Scope { kArmOnly, kWholeBody };
  Scope scope = Scope::kArmOnly;

  std::string frameName;      // 通常 modelInfo.eeFrame
  int armStartIndex = 3;
  int armDim = 6;

  bool useMinSingularValue = true;
  scalar_t minSingularWeight = 1.0;
  scalar_t minSingularRef = 0.05;

  bool useYoshikawa = false;
  scalar_t yoshikawaWeight = 0.1;
  scalar_t yoshikawaRef = 0.1;

  bool useTaskDirection = false;
  vector_t taskDirection = vector_t::Zero(6);
  scalar_t taskDirectionWeight = 1.0;
  scalar_t taskDirectionRef = 0.1;

  bool useInverseManipulability = false;
  scalar_t inverseManipulabilityWeight = 1e-3;

  bool useConditionNumber = false;
  scalar_t conditionWeight = 1e-3;
  scalar_t conditionMax = 50.0;

  scalar_t regularization = 1e-6;
  scalar_t finiteDiffStep = 1e-6;
  scalar_t hessianRegularization = 1e-6;
};

class ArmManipulabilityCost final : public ocs2::StateCost {
 public:
  ArmManipulabilityCost(const ocs2::PinocchioInterface& pinocchioInterface,
                        WbmmModelInfo modelInfo,
                        ArmManipulabilitySettings settings);

  ArmManipulabilityCost* clone() const override;

  scalar_t getValue(scalar_t time, const vector_t& state,
                    const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComputation) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      scalar_t time, const vector_t& state,
      const TargetTrajectories& targetTrajectories,
      const PreComputation& preComputation) const override;

 private:
  struct Metrics {
    scalar_t yoshikawa{0};
    scalar_t sigmaMin{0};
    scalar_t sigmaMax{0};
    scalar_t taskDirection{0};
    scalar_t inverseManipulability{0};
    scalar_t condition{0};
    bool valid{false};
  };

  Metrics computeMetrics(const vector_t& state) const;
  scalar_t costFromMetrics(const Metrics& metrics) const;

  ocs2::PinocchioInterface pinocchioInterface_;
  WbmmModelInfo modelInfo_;
  ArmManipulabilitySettings settings_;
  pinocchio::FrameIndex eeFrameId_;
};
```

`computeMetrics()` 核心实现建议：

```cpp
auto& model = pinocchioInterface_.getModel();
auto data = pinocchioInterface_.getData();  // 拷贝一份，保证线程安全

pinocchio::forwardKinematics(model, data, state);
pinocchio::updateFramePlacements(model, data);
pinocchio::computeJointJacobians(model, data, state);

matrix_t J = matrix_t::Zero(6, model.nv);
pinocchio::getFrameJacobian(
    model, data, eeFrameId_,
    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

matrix_t Juse;
if (settings_.scope == ArmManipulabilitySettings::Scope::kArmOnly) {
  Juse = J.middleCols(settings_.armStartIndex, settings_.armDim);
} else {
  Juse.resize(6, modelInfo_.inputDim);
  Eigen::Matrix<scalar_t, 3, 2> B;
  B << std::cos(state(2)), 0.0,
       std::sin(state(2)), 0.0,
       0.0,                1.0;
  Juse.leftCols<2>() = J.leftCols<3>() * B;
  Juse.rightCols(settings_.armDim) =
      J.middleCols(settings_.armStartIndex, settings_.armDim);
}

Eigen::JacobiSVD<matrix_t> svd(
    Juse, Eigen::ComputeThinU | Eigen::ComputeThinV);
const vector_t s = svd.singularValues();

Metrics m;
m.sigmaMin = s.minCoeff();
m.sigmaMax = s.maxCoeff();
m.yoshikawa = s.prod();
m.condition = m.sigmaMax / std::max(m.sigmaMin, 1e-9);

const matrix_t JJt =
    Juse * Juse.transpose() +
    settings_.regularization * matrix_t::Identity(6, 6);
m.inverseManipulability = JJt.inverse().trace();

if (settings_.taskDirection.size() == 6) {
  m.taskDirection = std::sqrt(std::max(
      0.0, settings_.taskDirection.dot(JJt * settings_.taskDirection)));
}
m.valid = s.allFinite();
```

`getQuadraticApproximation()` 建议：

- `getValue()` 用 SVD 精确算指标。
- `getQuadraticApproximation()` 对活跃状态变量做中心差分：

```cpp
for (int k : activeIndices) {
  auto xp = state; xp(k) += h;
  auto xm = state; xm(k) -= h;
  const Metrics mp = computeMetrics(xp);
  const Metrics mm = computeMetrics(xm);

  // 对每个 hinge 指标
  const scalar_t g = (metric(mp) - metric(mm)) / (2.0 * h);
  if (metric(state) < ref) {
    L.dfdx(k) += -weight * (ref - metric(state)) * g;
    L.dfdxx += weight * g * g.transpose();   // Gauss-Newton
  }
}
```

- 逆操作度是线性项，可以只用一阶梯度，二阶加小的 `hessianRegularization * I`。
- 对 arm-only 模式，活跃变量只需 `q1..q6`，即 state 的 `3..8`；对 whole-body 模式，活跃变量为 `yaw, q1..q6`，即 `2..8`。

### 2.4 在 `WbmmInterface` 中接入

在 `WbmmInterface.cpp` 的 cost 组装区新增：

```cpp
bool armManipulabilityEnabled = false;
loadData::loadPtreeValue(
    pt, armManipulabilityEnabled, "armManipulability.activate", false);
if (armManipulabilityEnabled) {
  problem_.stateCostPtr->add(
      "armManipulability",
      getArmManipulabilityCost(taskFile, *pinocchioInterfacePtr_, false));

  problem_.finalCostPtr->add(
      "finalArmManipulability",
      getArmManipulabilityCost(taskFile, *pinocchioInterfacePtr_, true));
}
```

建议新增：

```cpp
std::unique_ptr<ocs2::StateCost> getArmManipulabilityCost(
    const std::string& taskFile,
    const ocs2::PinocchioInterface& pinocchioInterface,
    bool isFinal);
```

`task.info` 配置示例：

```ini
armManipulability
{
  activate true
  scope "arm"        ; "arm" 或 "whole_body"
  frame "tool0"

  ; 可选：6 维任务缩放，避免平移/旋转量纲直接混在一起
  taskScale { [0] 1.0 [1] 1.0 [2] 1.0 [3] 0.2 [4] 0.2 [5] 0.2 }

  minSingularValue
  {
    activate true
    weight   0.5
    ref      0.05
  }

  yoshikawa
  {
    activate true
    weight   0.1
    ref      0.05
  }

  inverseManipulability
  {
    activate false
    weight   1e-3
  }

  conditionNumber
  {
    activate false
    weight   1e-3
    max      50.0
  }

  finiteDiffStep 1e-6
  regularization 1e-6
  hessianRegularization 1e-6
}
```

---

## 3. 需求 2：从全身跟踪改成末端跟踪

> 修订：最终设计采用“双参考 + 阶段权重”。本节方案 B 的 `EndEffectorTrackingCost` 是执行阶段末端代价的推荐实现；它和 `WholeBodyTrackingCost` 同时注册，由第 4 节的 `PhaseWeightedStateCost` 按阶段加权。

这里有两种落地方案。

### 方案 A：最小改动，复用现有 `EndEffectorConstraint`

当前代码本来就支持“只开末端约束、关全身代价”：

```ini
endEffector.activate true
wholeBodyTracking.activate false
```

如果只是要在某个模式下切到末端跟踪，复用现有 `EndEffectorConstraint` 即可。

但必须做维度保护。在 `EndEffectorConstraint.cpp` 的 `getValue()` 和 `getLinearApproximation()` 开头增加：

```cpp
const auto& tt = referenceManagerPtr_->getTargetTrajectories();
const size_t expectedDim = dualArmMode_ ? 14 : 7;
const size_t numConstraints = dualArmMode_ ? 12 : 6;

if (tt.stateTrajectory.empty() ||
    tt.stateTrajectory.front().size() != expectedDim) {
  // 模式切换瞬态：目标维度还不匹配，本项不产生代价和梯度
  if (getValue) return vector_t::Zero(numConstraints);
  if (getLinearApproximation) {
    VectorFunctionLinearApproximation approx(numConstraints, state.rows(), 0);
    approx.f.setZero();
    approx.dfdx.setZero();
    return approx;
  }
}
```

这样即使模式切到 EE，但 `_mpc_target` 还是 9D 全身目标，也不会错误解释四元数或崩溃。

### 方案 B：推荐，新增 `EndEffectorTrackingCost`

更干净的做法是新增一个真正的 `StateCost`，而不是软约束。原因：

- `StateCost::getValue()` 直接拿到 `TargetTrajectories`，不需要 `ReferenceManager` 裸指针。
- 可以和 `WholeBodyTrajectoryCost` 对称地由 `PhaseWeightedStateCost` 按阶段加权。
- 维度校验和失效回退更容易写。

建议接口：

```cpp
class EndEffectorTrackingCost final : public ocs2::StateCost {
 public:
  EndEffectorTrackingCost(
      const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics,
      ocs2::matrix_t QPosition,
      ocs2::matrix_t QOrientation);

  EndEffectorTrackingCost* clone() const override;

  ocs2::scalar_t getValue(
      ocs2::scalar_t time, const ocs2::vector_t& state,
      const ocs2::TargetTrajectories& targetTrajectories,
      const ocs2::PreComputation& preComputation) const override;

  ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      ocs2::scalar_t time, const ocs2::vector_t& state,
      const ocs2::TargetTrajectories& targetTrajectories,
      const ocs2::PreComputation& preComputation) const override;

 private:
  struct TargetPose {
    ocs2::vector_t position;
    ocs2::quaternion_t orientation;
    bool valid{false};
  };

  TargetPose getTargetPose(
      ocs2::scalar_t time,
      const ocs2::TargetTrajectories& targetTrajectories) const;

  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> kinematicsPtr_;
  ocs2::PinocchioEndEffectorKinematics* pinocchioPtr_{nullptr};
  ocs2::matrix_t Qp_;
  ocs2::matrix_t Qo_;
};
```

目标插值：

```cpp
const auto& ts = targetTrajectories.timeTrajectory;
const auto& xs = targetTrajectories.stateTrajectory;

if (xs.empty()) return {};

// 关键：EE 模式必须是 7D
if (xs.front().size() != 7) {
  // 非 ROS 包内用 std::cerr 一次性警告
  return {};
}

if (xs.size() == 1) {
  TargetPose out;
  out.position = xs.front().head<3>();
  out.orientation = quaternion_t(xs.front().tail<4>());
  out.valid = true;
  return out;
}

// 越界 clamp
if (time <= ts.front()) { /* xs.front() */ }
if (time >= ts.back())  { /* xs.back()  */ }

int index;
scalar_t alpha;
std::tie(index, alpha) =
    ocs2::LinearInterpolation::timeSegment(time, ts);

const auto& lhs = xs[index];
const auto& rhs = xs[index + 1];

out.position =
    alpha * lhs.head<3>() + (1.0 - alpha) * rhs.head<3>();
out.orientation =
    quaternion_t(lhs.tail<4>()).slerp(
        1.0 - alpha, quaternion_t(rhs.tail<4>()));
out.valid = true;
```

注意：OCS2 的 `LinearInterpolation::timeSegment` 约定 `alpha=1` 在区间起点，`alpha=0` 在区间终点，加权顺序不要写反。

代价与近似：

```cpp
// getValue
const vector_t ep = eePosition - target.position;
const vector_t eo = eeOrientationError;
return 0.5 * ep.dot(Qp_ * ep) + 0.5 * eo.dot(Qo_ * eo);

// getQuadraticApproximation
const auto p = kinematicsPtr_->getPositionLinearApproximation(state).front();
const auto o = kinematicsPtr_->getOrientationErrorLinearApproximation(
    state, {target.orientation}).front();

const vector_t ep = p.f - target.position;
const vector_t eo = o.f;

L.f = 0.5 * ep.dot(Qp_ * ep) + 0.5 * eo.dot(Qo_ * eo);
L.dfdx = ep.transpose() * Qp_ * p.dfdx +
         eo.transpose() * Qo_ * o.dfdx;
L.dfdxx = p.dfdx.transpose() * Qp_ * p.dfdx +
          o.dfdx.transpose() * Qo_ * o.dfdx;  // Gauss-Newton
```

接入 `WbmmInterface`：

```cpp
std::unique_ptr<ocs2::StateCost> getEndEffectorTrackingCost(
    const ocs2::PinocchioInterface& pinocchioInterface,
    const std::string& taskFile,
    const std::string& prefix,
    bool usePreComputation,
    const std::string& libraryFolder,
    bool recompileLibraries);
```

配置示例：

```ini
endEffectorTracking
{
  activate true

  Q
  {
    position
    {
      (0,0) 100.0
      (1,1) 100.0
      (2,2) 100.0
    }
    orientation
    {
      (0,0) 25.0
      (1,1) 25.0
      (2,2) 25.0
    }
  }
}
```

如果保留旧 `EndEffectorConstraint`，建议它只用于 legacy 或双末端场景；模式切换统一用 `EndEffectorTrackingCost`。

---

## 4. 需求 3：双参考管理与模式切换

> 本节已按“target 分为全身参考和末端参考、不同阶段不同权重、外部消息切换模式”重写。

### 4.1 阶段定义

建议定义：

```cpp
enum class TaskPhase : size_t {
  kNavigation = 0,  // 导航/跟踪：全身参考主导
  kTransition = 1,  // 过渡：两个参考同时加权
  kExecution  = 2,  // 执行：末端参考主导
  kRetract    = 3,  // 回撤/切回全身
};
```

阶段由外部消息给出，不建议由 MPC 内部用时间或误差硬判断。上层任务状态机/规划器可以综合：

- REMANI/规划器当前段类型；
- 距 pre-grasp / pre-insert 位姿的距离；
- 任务触发信号；
- 力/接触状态；
- 超时和失败恢复。

### 4.2 外部消息定义

推荐三个 ROS 话题：

```text
/mobile_manipulator_whole_body_target   ocs2_msgs/msg/MpcTargetTrajectories   9D
/mobile_manipulator_ee_target           ocs2_msgs/msg/MpcTargetTrajectories   7D
/mobile_manipulator_task_phase          std_msgs/msg/Int32
```

`task_phase` 取值：

```text
0 = Navigation
1 = Transition
2 = Execution
3 = Retract
```

可选再提供一个服务 `set_task_phase`，但核心切换入口建议保持为 latched topic，便于多个节点同时订阅。

### 4.3 新增 `WbmmReferenceManager`

它同时维护两个参考和当前阶段，并在 `preSolverRun()` 时锁存，保证一次 MPC 求解过程中阶段和参考不变。

```cpp
// WbmmReferenceManager.h
#pragma once

#include <atomic>
#include <cstddef>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_core/thread_support/BufferedValue.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

namespace wbmm_ocs2 {

enum class TaskPhase : size_t {
  kNavigation = 0,
  kTransition = 1,
  kExecution  = 2,
  kRetract    = 3,
};

class WbmmReferenceManager final : public ocs2::ReferenceManager {
 public:
  explicit WbmmReferenceManager(
      TaskPhase initialPhase = TaskPhase::kNavigation)
      : phase_(static_cast<size_t>(initialPhase)),
        requestedPhase_(static_cast<size_t>(initialPhase)) {
    // 让 OCS2 policy 的 mode 与 task phase 一致，便于 MRT 检查
    setModeSchedule(ocs2::ModeSchedule(
        {}, {static_cast<size_t>(initialPhase)}));
  }

  // ---------- target setters ----------
  void setWholeBodyTarget(const ocs2::TargetTrajectories& target) {
    wholeBodyTarget_.setBuffer(target);
  }

  void setEndEffectorTarget(const ocs2::TargetTrajectories& target) {
    eeTarget_.setBuffer(target);
  }

  // ---------- phase setter ----------
  void setTaskPhase(TaskPhase phase) {
    const auto value = static_cast<size_t>(phase);
    requestedPhase_.store(value, std::memory_order_relaxed);
    phase_.setBuffer(value);

    // policy mode = task phase，便于 MRT 做阶段一致性检查
    setModeSchedule(ocs2::ModeSchedule({}, {value}));
  }

  // ---------- getters ----------
  TaskPhase getTaskPhase() const {
    return static_cast<TaskPhase>(phase_.get());
  }

  TaskPhase getRequestedTaskPhase() const {
    return static_cast<TaskPhase>(
        requestedPhase_.load(std::memory_order_relaxed));
  }

  const ocs2::TargetTrajectories& getWholeBodyTarget() const {
    return wholeBodyTarget_.get();
  }

  const ocs2::TargetTrajectories& getEndEffectorTarget() const {
    return eeTarget_.get();
  }

  // OCS2 内部某些模块仍会调用 getTargetTrajectories()。
  // 这里返回当前阶段主导的参考。
  const ocs2::TargetTrajectories& getTargetTrajectories() const override {
    return (getTaskPhase() == TaskPhase::kExecution)
        ? eeTarget_.get()
        : wholeBodyTarget_.get();
  }

  // reset service / 旧接口的兼容入口：按当前阶段路由到对应 target。
  void setTargetTrajectories(
      const ocs2::TargetTrajectories& target) override {
    if (getTaskPhase() == TaskPhase::kExecution) {
      eeTarget_.setBuffer(target);
    } else {
      wholeBodyTarget_.setBuffer(target);
    }
  }

  void preSolverRun(
      ocs2::scalar_t initTime,
      ocs2::scalar_t finalTime,
      const ocs2::vector_t& initState) override {
    wholeBodyTarget_.updateFromBuffer();
    eeTarget_.updateFromBuffer();
    phase_.updateFromBuffer();

    ocs2::ReferenceManager::preSolverRun(initTime, finalTime, initState);
  }

 private:
  ocs2::BufferedValue<ocs2::TargetTrajectories>
      wholeBodyTarget_{ocs2::TargetTrajectories()};
  ocs2::BufferedValue<ocs2::TargetTrajectories>
      eeTarget_{ocs2::TargetTrajectories()};
  ocs2::BufferedValue<size_t> phase_;
  std::atomic<size_t> requestedPhase_;
};

}  // namespace wbmm_ocs2
```

### 4.4 新增 `PhaseWeightedStateCost`

它不直接二选一，而是按阶段对两个 cost 加权：

```cpp
enum class TargetKind {
  kWholeBody,
  kEndEffector,
};

class PhaseWeightedStateCost final : public ocs2::StateCost {
 public:
  PhaseWeightedStateCost(
      std::unique_ptr<ocs2::StateCost> cost,
      std::shared_ptr<const WbmmReferenceManager> refManager,
      TargetKind targetKind,
      std::array<ocs2::scalar_t, 4> phaseWeights)
      : cost_(std::move(cost)),
        refManager_(std::move(refManager)),
        targetKind_(targetKind),
        phaseWeights_(phaseWeights) {}

  PhaseWeightedStateCost* clone() const override {
    return new PhaseWeightedStateCost(
        std::unique_ptr<ocs2::StateCost>(cost_->clone()),
        refManager_, targetKind_, phaseWeights_);
  }

  bool isActive(ocs2::scalar_t time) const override {
    return weight() > 0.0 && cost_->isActive(time);
  }

  ocs2::scalar_t getValue(
      ocs2::scalar_t time, const ocs2::vector_t& state,
      const ocs2::TargetTrajectories& /*target*/,
      const ocs2::PreComputation& preComp) const override {
    const auto& target = targetForPhase();
    return weight() * cost_->getValue(time, state, target, preComp);
  }

  ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      ocs2::scalar_t time, const ocs2::vector_t& state,
      const ocs2::TargetTrajectories& /*target*/,
      const ocs2::PreComputation& preComp) const override {
    const auto& target = targetForPhase();
    auto approx = cost_->getQuadraticApproximation(
        time, state, target, preComp);
    const ocs2::scalar_t w = weight();
    approx.f *= w;
    approx.dfdx *= w;
    approx.dfdxx *= w;
    return approx;
  }

 private:
  PhaseWeightedStateCost(const PhaseWeightedStateCost& other)
      : ocs2::StateCost(other),
        cost_(other.cost_->clone()),
        refManager_(other.refManager_),
        targetKind_(other.targetKind_),
        phaseWeights_(other.phaseWeights_) {}

  const ocs2::TargetTrajectories& targetForPhase() const {
    return (targetKind_ == TargetKind::kWholeBody)
        ? refManager_->getWholeBodyTarget()
        : refManager_->getEndEffectorTarget();
  }

  ocs2::scalar_t weight() const {
    return phaseWeights_[static_cast<size_t>(refManager_->getTaskPhase())];
  }

  std::unique_ptr<ocs2::StateCost> cost_;
  std::shared_ptr<const WbmmReferenceManager> refManager_;
  TargetKind targetKind_;
  std::array<ocs2::scalar_t, 4> phaseWeights_;
};
```

说明：

- `WholeBodyTrackingCost` 用 `TargetKind::kWholeBody`，读取 `wholeBodyTarget_`。
- `EndEffectorTrackingCost` 用 `TargetKind::kEndEffector`，读取 `eeTarget_`。
- 权重为 0 时该 cost 不参与优化。
- `kTransition` 阶段两个 cost 可以同时非零，实现平滑过渡。

### 4.5 修改 `WbmmInterface`

头文件：

```cpp
std::shared_ptr<WbmmReferenceManager> getWbmmReferenceManagerPtr() const {
  return wbmmRefManagerPtr_;
}

void setTaskPhase(TaskPhase phase) {
  wbmmRefManagerPtr_->setTaskPhase(phase);
}

void setWholeBodyTarget(const ocs2::TargetTrajectories& target) {
  wbmmRefManagerPtr_->setWholeBodyTarget(target);
}

void setEndEffectorTarget(const ocs2::TargetTrajectories& target) {
  wbmmRefManagerPtr_->setEndEffectorTarget(target);
}

TaskPhase getTaskPhase() const {
  return wbmmRefManagerPtr_->getTaskPhase();
}
```

构造函数里：

```cpp
wbmmRefManagerPtr_ =
    std::make_shared<WbmmReferenceManager>(TaskPhase::kNavigation);
referenceManagerPtr_ = wbmmRefManagerPtr_;
```

然后在 `WbmmInterface.cpp:233-253` 附近改成：

```cpp
// 全身参考代价
auto wholeBodyCost =
    getWholeBodyTrajectoryCost(taskFile, "wholeBodyTracking", false);
problem_.stateCostPtr->add(
    "wholeBodyTracking",
    std::make_unique<PhaseWeightedStateCost>(
        std::move(wholeBodyCost),
        wbmmRefManagerPtr_,
        TargetKind::kWholeBody,
        std::array<scalar_t, 4>{1.0, 0.5, 0.1, 0.5}));

// 末端参考代价
auto eeCost = getEndEffectorTrackingCost(
    *pinocchioInterfacePtr_, taskFile,
    "endEffectorTracking", usePreComputation,
    libraryFolder, recompileLibraries);
problem_.stateCostPtr->add(
    "endEffectorTracking",
    std::make_unique<PhaseWeightedStateCost>(
        std::move(eeCost),
        wbmmRefManagerPtr_,
        TargetKind::kEndEffector,
        std::array<scalar_t, 4>{0.0, 0.5, 1.0, 0.0}));

// 终端代价同样用 PhaseWeightedStateCost 包装
problem_.finalCostPtr->add(
    "finalWholeBodyTracking",
    std::make_unique<PhaseWeightedStateCost>(
        getWholeBodyTrajectoryCost(taskFile, "wholeBodyTracking", true),
        wbmmRefManagerPtr_,
        TargetKind::kWholeBody,
        std::array<scalar_t, 4>{1.0, 0.5, 0.1, 0.5}));

problem_.finalCostPtr->add(
    "finalEndEffectorTracking",
    std::make_unique<PhaseWeightedStateCost>(
        getEndEffectorTrackingCost(
            *pinocchioInterfacePtr_, taskFile,
            "endEffectorTracking", usePreComputation,
            libraryFolder, recompileLibraries),
        wbmmRefManagerPtr_,
        TargetKind::kEndEffector,
        std::array<scalar_t, 4>{0.0, 0.5, 1.0, 0.0}));
```

权重建议从 `task.info` 读取，而不是写死在代码里：

```ini
wholeBodyTracking
{
  phaseWeights { [0] 1.0 [1] 0.5 [2] 0.1 [3] 0.5 }
}

endEffectorTracking
{
  phaseWeights { [0] 0.0 [1] 0.5 [2] 1.0 [3] 0.0 }
}
```

始终开启的代价/约束：

```text
inputCost
jointPositionLimits
jointVelocityLimits
selfCollision
environmentCollision
armManipulability
base/posture regularization
```

### 4.6 ROS 层实现

#### `WbmmMpcNode.cpp`

订阅三个话题：

```cpp
// whole body target
auto wholeBodyTargetSub =
    nodeHandle->create_subscription<ocs2_msgs::msg::MpcTargetTrajectories>(
        wholeBodyTargetTopic, 1,
        [&interface](const ocs2_msgs::msg::MpcTargetTrajectories::SharedPtr msg) {
          interface.setWholeBodyTarget(
              ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(*msg));
        });

// ee target
auto eeTargetSub =
    nodeHandle->create_subscription<ocs2_msgs::msg::MpcTargetTrajectories>(
        eeTargetTopic, 1,
        [&interface](const ocs2_msgs::msg::MpcTargetTrajectories::SharedPtr msg) {
          interface.setEndEffectorTarget(
              ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(*msg));
        });

// task phase
auto phaseSub =
    nodeHandle->create_subscription<std_msgs::msg::Int32>(
        taskPhaseTopic, rclcpp::QoS(1).transient_local(),
        [&interface](const std_msgs::msg::Int32::SharedPtr msg) {
          interface.setTaskPhase(
              static_cast<wbmm_ocs2::TaskPhase>(msg->data));
        });
```

注意：

- `RosReferenceManager` 不再负责 target；target 由上面两个订阅直接写入 `WbmmReferenceManager`。
- `MPC_ROS_Interface` 的 reset service 仍会调用 `setTargetTrajectories`，由 `WbmmReferenceManager` 按当前阶段路由到对应 buffer。
- 外部 phase 消息建议使用 latched QoS，保证新启动节点也能拿到当前阶段。

#### `WbmmMrtNode.cpp`

- 订阅 `task_phase` 话题，保存 `std::atomic<int> currentPhase_`。
- `resetMpc()` 根据当前 phase 选择 reset target：

```cpp
if (currentPhase_ == static_cast<int>(wbmm_ocs2::TaskPhase::kExecution)) {
  target = lookupCurrentEePose();   // 7D
} else {
  target = observation.state;       // 9D
}
```

- 安全增强：`evaluateCurrentPolicy()` 返回的 policy mode 由 `WbmmReferenceManager` 的 `ModeSchedule` 决定。因为 `setTaskPhase()` 会把 mode schedule 设为当前 phase，所以可以直接比较：

```cpp
if (policyMode != static_cast<size_t>(currentPhase_)) {
  RCLCPP_WARN_THROTTLE(... "phase mismatch, holding");
  publishZeroBaseCommand();
  holdArmCommand();
  return;
}
```

#### target publishers

- `remani_to_ocs2_reference_bridge.cpp`：发布 9D 全身参考到 `/mobile_manipulator_whole_body_target`。
- `WbmmTargetNode.cpp`：发布 7D 末端参考到 `/mobile_manipulator_ee_target`。
- 两个 publisher 可以同时运行，不需要按模式 gate。
- 阶段切换时，外部任务节点同时切换 phase 消息即可。

### 4.7 阶段切换时序建议

```text
1. 外部任务节点发布 /mobile_manipulator_task_phase
2. WbmmMpcNode:
   - interface.setTaskPhase(phase)
   - WbmmReferenceManager 更新 phase buffer 和 ModeSchedule
3. WbmmMrtNode:
   - 收到 phase，currentPhase_ 更新
   - 检查 policy mode，不匹配时先 hold
4. 两个 target 话题持续更新:
   - whole_body_target 来自 REMANI
   - ee_target 来自任务/视觉节点
5. MPC 下一轮 preSolverRun:
   - WbmmReferenceManager 锁存 phase 和两个 target
   - PhaseWeightedStateCost 按阶段权重生效
6. MRT 收到新 policy 后恢复执行
```

### 4.8 过渡与安全建议

- 不要直接从 `Navigation` 硬切到 `Execution`；建议经过 `Transition`。
- `Transition` 阶段两个 cost 同时非零，权重平滑变化。
- 切到 `Execution` 前，EE target 建议先发“当前 EE pose”作为 hold target，再逐渐过渡到任务目标。
- 执行阶段不要完全关闭全身约束；保留底盘位置/航向、机械臂姿态低权重正则，避免底盘漂移和机械臂奇异。
- 接触/力控任务仍需导纳/力控参与，OCS2 位置跟踪不能替代力控。
- 实机切换时必须有人工确认的 hold/急停/看门狗策略。

---

## 5. 需求 4：环境/碰撞后端改为 ESDF

建议分两层实现：

1. `wbmm_environment` 实现真正的 NPZ 加载和 ESDF 查询。
2. `wbmm_ocs2` 新增基于 ESDF 的环境碰撞约束，并与现有 coal backend 可配置切换。

### 5.1 实现 `NpzEsdfLoader::load()`

直接参考 REMANI：

- `src/vendor/remani_planner/plan_env/src/grid_map.cpp:1-425`
  - `NpyArray`
  - `readNpyMember`
  - `copyFloatArray`
  - `copyScalarString`
  - `loadStaticEsdfArchive`

把其中与 ROS 无关的部分搬到：

```text
src/map/wbmm_environment/src/npz_esdf_loader.cpp
```

读取字段：

```text
esdf.npy
occupancy.npy
observed.npy       // 可选；如果没有则全部视为 observed
origin.npy
voxel_size.npy
bounds_max.npy
frame_id.npy
```

校验：

- `esdf`、`occupancy` 必须 3D 且 shape 相同。
- `origin`、`bounds_max` 必须 3 维。
- `voxel_size` 必须为正标量。
- `bounds_max ≈ origin + voxel_size * shape`。
- `frame_id` 必须 ASCII 非空。

返回：

```cpp
EsdfLoadResult{
  LoadStatus::kSuccess,
  std::make_shared<EsdfGrid>(std::move(data)),
  "loaded"
};
```

CMake / package 需要增加 `libzip`：

```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBZIP REQUIRED libzip)

target_include_directories(wbmm_environment PRIVATE ${LIBZIP_INCLUDE_DIRS})
target_link_libraries(wbmm_environment PUBLIC ${LIBZIP_LIBRARIES})
```

```xml
<depend>libzip-dev</depend>
```

### 5.2 实现 `EsdfGrid::query()`

查询约定参考 REMANI：

- 体素中心：

$$
p_{\rm center}(i,j,k)=o + \left([i,j,k]^T+\frac12\mathbf{1}\right)\Delta
$$

- 三线性插值：

```cpp
DistanceQuery EsdfGrid::query(
    const std::string& frame_id,
    const Eigen::Vector3d& p) const {

  DistanceQuery result;

  if (frame_id != data_.info.frame_id) {
    result.status = QueryStatus::kFrameMismatch;
    result.message = "frame mismatch";
    return result;
  }

  const auto& info = data_.info;
  const Eigen::Vector3d pos_m =
      p - 0.5 * info.voxel_size * Eigen::Vector3d::Ones();

  Eigen::Vector3i idx;
  for (int i = 0; i < 3; ++i) {
    idx(i) = static_cast<int>(std::floor(
        (pos_m(i) - info.origin(i)) / info.voxel_size));
  }

  // 检查 idx 和 idx+1 是否都在网格内
  for (int i = 0; i < 3; ++i) {
    if (idx(i) < 0 || idx(i) + 1 >= info.shape(i)) {
      result.status = QueryStatus::kOutOfBounds;
      result.message = "query point outside ESDF grid";
      return result;
    }
  }

  const Eigen::Vector3d idx_pos =
      info.origin +
      (idx.cast<double>() + 0.5 * Eigen::Vector3d::Ones()) *
          info.voxel_size;

  const Eigen::Vector3d diff =
      (p - idx_pos) / info.voxel_size;

  double values[2][2][2];
  bool observed = true;

  auto address = [&](int x, int y, int z) {
    return (x * info.shape.y() + y) * info.shape.z() + z;
  };

  for (int x = 0; x < 2; ++x)
    for (int y = 0; y < 2; ++y)
      for (int z = 0; z < 2; ++z) {
        const int ix = idx.x() + x;
        const int iy = idx.y() + y;
        const int iz = idx.z() + z;
        const size_t id = address(ix, iy, iz);
        values[x][y][z] =
            static_cast<double>(data_.esdf[id]);
        observed = observed && data_.observed[id];
      }

  // 三线性插值 value
  const double v00 =
      (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];
  const double v01 =
      (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];
  const double v10 =
      (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];
  const double v11 =
      (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];

  const double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
  const double v1 = (1 - diff(1)) * v01 + diff(1) * v11;
  result.distance = (1 - diff(2)) * v0 + diff(2) * v1;

  // 梯度
  const double inv = 1.0 / info.voxel_size;
  result.gradient(2) = (v1 - v0) * inv;
  result.gradient(1) =
      ((1 - diff(2)) * (v10 - v00) +
       diff(2) * (v11 - v01)) * inv;

  result.gradient(0) =
      (1 - diff(2)) * (1 - diff(1)) *
          (values[1][0][0] - values[0][0][0]) +
      (1 - diff(2)) * diff(1) *
          (values[1][1][0] - values[0][1][0]) +
      diff(2) * (1 - diff(1)) *
          (values[1][0][1] - values[0][0][1]) +
      diff(2) * diff(1) *
          (values[1][1][1] - values[0][1][1]);
  result.gradient(0) *= inv;

  result.gradient_valid = observed;
  result.status = observed ? QueryStatus::kSuccess
                           : QueryStatus::kUnknown;
  return result;
}
```

关键点：

- `frame_id` 不一致必须返回 `kFrameMismatch`，不能自动把 `map` 和 `odom` 当同一个 frame。
- `observed` 中任一邻域体素未观测，`gradient_valid=false`。
- 越界返回 `kOutOfBounds`。
- 未知区域的距离沿用导出器写入的保守值，但梯度不可用。

### 5.3 新增 `EsdfEnvironmentInterface`

它把 Pinocchio 球近似和 ESDF 查询连接起来。

```cpp
class EsdfEnvironmentInterface {
 public:
  struct DistanceResult {
    double distance{0.0};        // ESDF at sphere center
    double radius{0.0};          // sphere radius
    double minimumDistance{0.0}; // safety margin
    Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
    bool gradientValid{false};
    std::string linkName;
  };

  EsdfEnvironmentInterface(
      const ocs2::PinocchioInterface& pinocchioInterface,
      std::shared_ptr<const wbmm::environment::EsdfGrid> grid,
      const std::vector<std::string>& collisionLinks,
      const std::vector<ocs2::scalar_t>& maxExcesses,
      ocs2::scalar_t shrinkRatio,
      ocs2::scalar_t defaultMinimumDistance);

  std::vector<DistanceResult> computeDistances(
      const ocs2::PinocchioInterface& pinocchioInterface) const;

  std::size_t getNumSpheres() const;
  const ocs2::PinocchioSphereInterface& getSphereInterface() const;
  std::string getFrameId() const;

 private:
  std::shared_ptr<const wbmm::environment::EsdfGrid> grid_;
  ocs2::PinocchioSphereInterface sphereInterface_;
  std::vector<double> sphereRadii_;
  std::vector<std::string> sphereLinks_;
  double defaultMinimumDistance_;
  std::string frameId_;
};
```

构造函数中：

```cpp
sphereInterface_ = ocs2::PinocchioSphereInterface(
    pinocchioInterface, collisionLinks, maxExcesses, shrinkRatio);

sphereRadii_ = sphereInterface_.getSphereRadii();
frameId_ = grid_->info().frame_id;
```

`computeDistances()`：

```cpp
const auto centers =
    sphereInterface_.computeSphereCentersInWorldFrame(pinocchioInterface);

std::vector<DistanceResult> results(centers.size());

for (size_t i = 0; i < centers.size(); ++i) {
  const auto q = grid_->query(frameId_, centers[i]);

  results[i].distance =
      std::isfinite(q.distance) ? q.distance : -defaultMinimumDistance_;
  results[i].gradient =
      q.gradient_valid ? q.gradient : Eigen::Vector3d::Zero();
  results[i].gradientValid =
      q.gradient_valid && q.status == QueryStatus::kSuccess;
  results[i].radius = sphereRadii_[i];
  results[i].minimumDistance = defaultMinimumDistance_;
  results[i].linkName = sphereLinks_[i];
}
```

注意：

- 当前 URDF 的碰撞几何大量是 sphere，非常适合 `PinocchioSphereInterface`。
- `PinocchioSphereInterface` 支持 box / cylinder / sphere；如果某些 link 的碰撞是 mesh，会抛异常，需要配置中排除或改成球近似。
- 如果 `EsdfGrid::query` 返回未知/越界，调用方必须保守处理，不能当自由空间。

### 5.4 新增 `EsdfEnvironmentCollisionConstraint`

约束值：

$$
h_i = d_i - r_i - m_i
$$

其中：

- `d_i`：第 i 个碰撞球中心的 ESDF 距离；
- `r_i`：球半径；
- `m_i`：最小安全距离。

梯度：

$$
\frac{\partial h_i}{\partial x}
=
J_{p_i}(x)^T \nabla_p d(p_i)
$$

`J_{p_i}` 是球心相对于 OCS2 state 的 Jacobian。

实现建议：

```cpp
class EsdfEnvironmentCollisionConstraint final
    : public ocs2::StateConstraint {
 public:
  EsdfEnvironmentCollisionConstraint(
      const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
      std::shared_ptr<const EsdfEnvironmentInterface> env,
      ocs2::scalar_t defaultMinimumDistance);

  EsdfEnvironmentCollisionConstraint* clone() const override;

  size_t getNumConstraints(ocs2::scalar_t time) const override {
    return env_->getNumSpheres();
  }

  ocs2::vector_t getValue(
      ocs2::scalar_t time, const ocs2::vector_t& state,
      const ocs2::PreComputation& preComp) const override;

  ocs2::VectorFunctionLinearApproximation getLinearApproximation(
      ocs2::scalar_t time, const ocs2::vector_t& state,
      const ocs2::PreComputation& preComp) const override;

 private:
  const ocs2::PinocchioInterface& getPinocchioInterface(
      const ocs2::PreComputation& preComp) const;

  std::shared_ptr<const EsdfEnvironmentInterface> env_;
  std::unique_ptr<ocs2::PinocchioSphereKinematics> sphereKinematics_;
  ocs2::scalar_t defaultMinimumDistance_;
};
```

`getValue()`：

```cpp
const auto& pinocchioInterface = getPinocchioInterface(preComp);
const auto distances = env_->computeDistances(pinocchioInterface);

vector_t h(distances.size());
for (size_t i = 0; i < distances.size(); ++i) {
  h(i) = distances[i].distance
       - distances[i].radius
       - distances[i].minimumDistance;
}
return h;
```

`getLinearApproximation()`：

```cpp
const auto& pinocchioInterface = getPinocchioInterface(preComp);
sphereKinematics_->setPinocchioInterface(pinocchioInterface);

const auto centers =
    sphereKinematics_->getPositionLinearApproximation(state);
const auto distances =
    env_->computeDistances(pinocchioInterface);

VectorFunctionLinearApproximation approx(
    distances.size(), state.rows(), 0);

for (size_t i = 0; i < distances.size(); ++i) {
  approx.f(i) = distances[i].distance
              - distances[i].radius
              - distances[i].minimumDistance;

  if (distances[i].gradientValid) {
    approx.dfdx.row(i) =
        distances[i].gradient.transpose() * centers[i].dfdx;
  } else {
    approx.dfdx.row(i).setZero();
  }
}
return approx;
```

`getPinocchioInterface()`：

```cpp
return cast<WbmmPreComputation>(preComp).getPinocchioInterface();
```

### 5.5 修改 `WbmmInterface::getEnvironmentCollisionConstraint()`

当前 `797-863` 行可以改成 backend 分支：

```ini
environmentCollision
{
  activate true
  backend "esdf"       ; "esdf" 或 "geometry"

  esdf
  {
    file  "/absolute/path/to/remani_esdf.npz"
    frame "odom"
  }

  collisionLinks
  {
    [0] "base_link"
    [1] "Link_0"
    [2] "Link_1"
    [3] "Link_2"
    [4] "Link_3"
    [5] "Link_4"
    [6] "Link_5"
    [7] "Link_6"
    [8] "tool0_and_camera_link"
    [9] "d435i_link"
  }

  maxExcess      0.01
  shrinkRatio    0.8
  minimumDistance    0.05
  activationDistance 0.20
  mu   1e-3
  delta 1e-3
}
```

逻辑：

```cpp
std::string backend = "geometry";
loadData::loadPtreeValue(pt, backend, prefix + ".backend", false);

if (backend == "esdf") {
  std::string esdfFile;
  loadData::loadPtreeValue(pt, esdfFile, prefix + ".esdf.file", true);

  auto loadResult = wbmm::environment::NpzEsdfLoader::load(esdfFile);
  if (loadResult.status != wbmm::environment::LoadStatus::kSuccess) {
    throw std::runtime_error("[EnvironmentCollision] ESDF load failed: " +
                             loadResult.message);
  }

  esdfGrid_ = loadResult.grid;

  // collisionLinks / maxExcesses / shrinkRatio
  // 构造 EsdfEnvironmentInterface
  esdfEnvInterfacePtr_ = std::make_shared<EsdfEnvironmentInterface>(
      pinocchioInterface, esdfGrid_, collisionLinks,
      maxExcesses, shrinkRatio, minimumDistance);

  auto constraint = std::make_unique<EsdfEnvironmentCollisionConstraint>(
      WbmmPinocchioMapping(modelInfo_),
      esdfEnvInterfacePtr_, minimumDistance);

  const scalar_t activationThreshold =
      activationDistance - minimumDistance;

  auto penalty = std::make_unique<ThresholdRelaxedBarrierPenalty>(
      ThresholdRelaxedBarrierPenalty::Config{
          mu, delta, activationThreshold});

  return std::make_unique<StateSoftConstraint>(
      std::move(constraint), std::move(penalty));
} else {
  // 保留原来的 coal/geometry backend
  // 即当前 797-863 行逻辑
}
```

注意：

- ESDF backend 不再要求 `selfCollision` 先创建 `PinocchioGeometryInterface`。
- 但 `selfCollision` 仍可独立保留。
- `loadInitialObstacles()` 只对 `geometry` backend 有意义；ESDF 不需要静态障碍物列表。

### 5.6 CMake / package 修改

`src/control/wbmm_ocs2/CMakeLists.txt`：

```cmake
find_package(wbmm_environment REQUIRED)
find_package(ocs2_sphere_approximation REQUIRED)

add_library(${PROJECT_NAME}
  ...
  src/cost/ArmManipulabilityCost.cpp
  src/cost/EndEffectorTrackingCost.cpp
  src/collision/EsdfEnvironmentInterface.cpp
  src/constraint/EsdfEnvironmentCollisionConstraint.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${ocs2_ddp_TARGETS}
  ${ocs2_sqp_TARGETS}
  ${ocs2_self_collision_TARGETS}
  ${ocs2_sphere_approximation_TARGETS}
  wbmm_environment::wbmm_environment
)

ament_export_dependencies(
  ocs2_ddp
  ocs2_sqp
  ocs2_self_collision
  ocs2_sphere_approximation
  wbmm_environment
)
```

`src/control/wbmm_ocs2/package.xml`：

```xml
<depend>wbmm_environment</depend>
<depend>ocs2_sphere_approximation</depend>
```

`src/map/wbmm_environment/CMakeLists.txt` 和 `package.xml` 增加 `libzip-dev`。

---

## 6. 配置示例汇总

### 6.1 `task.info`

```ini
; task phase:
;   0 = Navigation
;   1 = Transition
;   2 = Execution
;   3 = Retract
;
; phaseWeights 顺序对应 [Navigation, Transition, Execution, Retract]

wholeBodyTracking
{
  activate true
  finalWeightScale 1.0

  phaseWeights { [0] 1.0 [1] 0.5 [2] 0.1 [3] 0.5 }

  Q
  {
    base { (0,0) 5.0 (1,1) 5.0 (2,2) 2.0 }
    arm  { (0,0) 2.0 (1,1) 2.0 (2,2) 2.0
           (3,3) 2.0 (4,4) 2.0 (5,5) 2.0 }
  }
}

endEffectorTracking
{
  activate true

  phaseWeights { [0] 0.0 [1] 0.5 [2] 1.0 [3] 0.0 }

  Q
  {
    position    { (0,0) 100.0 (1,1) 100.0 (2,2) 100.0 }
    orientation { (0,0) 25.0  (1,1) 25.0  (2,2) 25.0 }
  }
}

armManipulability
{
  activate true
  scope "arm"
  frame "tool0"

  minSingularValue
  {
    activate true
    weight   0.5
    ref      0.05
  }

  yoshikawa
  {
    activate true
    weight   0.1
    ref      0.05
  }

  inverseManipulability
  {
    activate false
    weight   1e-3
  }
}

environmentCollision
{
  activate true
  backend "esdf"

  esdf
  {
    file  "/absolute/path/to/remani_esdf.npz"
    frame "odom"
  }

  collisionLinks
  {
    [0] "base_link"
    [1] "Link_0"
    [2] "Link_1"
    [3] "Link_2"
    [4] "Link_3"
    [5] "Link_4"
    [6] "Link_5"
    [7] "Link_6"
    [8] "tool0_and_camera_link"
    [9] "d435i_link"
  }

  maxExcess      0.01
  shrinkRatio    0.8
  minimumDistance    0.05
  activationDistance 0.20
  mu   1e-3
  delta 1e-3
}
```

### 6.2 `ocs2.yaml`

```yaml
wbmm_mpc_node:
  ros__parameters:
    whole_body_target_topic: mobile_manipulator_whole_body_target
    ee_target_topic: mobile_manipulator_ee_target
    task_phase_topic: mobile_manipulator_task_phase

wbmm_mrt_node:
  ros__parameters:
    task_phase_topic: mobile_manipulator_task_phase
    initial_phase: 0
```

---

## 7. 建议实施顺序

1. **ESDF 基础设施**
   - 实现 `NpzEsdfLoader::load()`。
   - 实现 `EsdfGrid::query()`。
   - 加单元测试：解析 NPZ、三线性距离、梯度、frame mismatch、out-of-bounds、unknown。

2. **末端跟踪 cost**
   - 新增 `EndEffectorTrackingCost`。
   - 加单元测试：7D 目标、9D 目标时安全返回零、位置/姿态梯度与数值差分一致。

3. **可操作度 cost**
   - 新增 `ArmManipulabilityCost`。
   - 用 Pinocchio 对一个固定 q 计算 `J`，与 Eigen SVD 对比 `σ_min`、`w`。
   - 用数值差分验证 `dfdx`。

4. **阶段/模式切换**
   - 新增 `WbmmReferenceManager` 和 `PhaseWeightedStateCost`。
   - 修改 `WbmmInterface` 支持双参考 target 和 `task_phase`。
   - 单元测试：`setTaskPhase` + `preSolverRun` 后对应阶段的 cost 权重正确，且两个 target 不会混用。

5. **ESDF 碰撞约束**
   - 新增 `EsdfEnvironmentInterface` 和 `EsdfEnvironmentCollisionConstraint`。
   - 在 `WbmmInterface` 中按 `environmentCollision.backend` 选择。
   - 测试：给定简单 ESDF 场 `d=x`，验证 `h` 和 `dh/dx`。

6. **ROS 模式接口**
   - `WbmmMpcNode` 加 mode service + latched topic。
   - `WbmmMrtNode` 订阅模式并做 policy mode mismatch hold。
   - `WbmmTargetNode` / `remani_to_ocs2_reference_bridge` 按模式 gate 发布。

7. **仿真验证**
   - 用 `mujoco_mapping_export.launch.py` 或 `d455_bag_esdf.launch.py` 导出 NPZ。
   - 设置 `environmentCollision.backend="esdf"`。
   - 先不开可操作度代价，只验证 ESDF 避障；再逐步加 cost。
   - 模式切换测试：whole-body -> EE -> whole-body，确认：
     - `_mpc_target` 维度正确；
     - MRT 在模式不匹配时不执行旧 policy；
     - 没有 NaN / 约束维度变化 / solver 异常。

---

## 8. 主要风险与 TBD

- **ESDF frame 与 OCS2 state frame 必须一致**。当前 state 没有显式 frame 字段，建议在 `environmentCollision.esdf.frame` 中声明，并在 `WbmmInterface` 构造时校验。
- **未知区域和越界策略**：不能当作自由空间。建议 `gradient_valid=false` 时约束值保守、梯度置零；但梯度置零可能让 solver 无法脱离未知区域，后续可考虑边界最近点梯度。
- **动态 ESDF**：当前 NPZ 是静态文件。若要在线更新，需要增加 `EsdfGrid` 的线程安全替换接口，或订阅 nvblox 服务/点云重新生成 `EsdfGrid`。
- **可操作度代价的 Hessian**：SVD 指标非凸，建议使用一侧 hinge + Gauss-Newton，不要直接最小化 `-σ_min`。
- **量纲问题**：6D Jacobian 平移和旋转量纲不同，建议使用 `taskScale` 或特征长度归一化，和 `wbmm_robot_metrics` 的 `JacobianScaling` 约定保持一致。
- **模式切换瞬态安全**：必须做 MRT policy mode mismatch hold；否则会继续执行旧模式 policy。
- **`EndEffectorConstraint` 兼容**：如果继续保留，必须加 7D/14D 维度保护。
- **`PinocchioSphereInterface` 几何限制**：只支持 box / cylinder / sphere；mesh 碰撞会抛异常。当前 URDF 的碰撞几何大多是 sphere，较适合，但配置前要逐 link 检查。
- **实机安全**：以上方案不改变现有 `command_output_enabled` 安全门。实机测试前仍需人工确认模式切换时的 hold、急停、看门狗和超时策略。

---

## 9. 文件修改清单

| 文件 | 改动 |
|---|---|
| `src/map/wbmm_environment/src/npz_esdf_loader.cpp` | 实现 NPZ 加载 |
| `src/map/wbmm_environment/src/esdf_grid.cpp` | 实现三线性距离/梯度查询 |
| `src/map/wbmm_environment/CMakeLists.txt` | 增加 `libzip` |
| `src/map/wbmm_environment/package.xml` | 增加 `libzip-dev` |
| `src/control/wbmm_ocs2/include/wbmm_ocs2/WbmmReferenceManager.h` | 新增双参考 + 阶段管理 |
| `src/control/wbmm_ocs2/include/wbmm_ocs2/cost/PhaseWeightedStateCost.h` | 新增阶段权重包装器 |
| `src/control/wbmm_ocs2/include/wbmm_ocs2/cost/ArmManipulabilityCost.h` | 新增可操作度代价 |
| `src/control/wbmm_ocs2/src/cost/ArmManipulabilityCost.cpp` | 实现 SVD/梯度/Hessian |
| `src/control/wbmm_ocs2/include/wbmm_ocs2/cost/EndEffectorTrackingCost.h` | 新增末端跟踪代价 |
| `src/control/wbmm_ocs2/src/cost/EndEffectorTrackingCost.cpp` | 实现末端跟踪 |
| `src/control/wbmm_ocs2/include/wbmm_ocs2/collision/EsdfEnvironmentInterface.h` | 新增 ESDF + 球近似 |
| `src/control/wbmm_ocs2/src/collision/EsdfEnvironmentInterface.cpp` | 实现距离查询 |
| `src/control/wbmm_ocs2/include/wbmm_ocs2/constraint/EsdfEnvironmentCollisionConstraint.h` | 新增 ESDF 约束 |
| `src/control/wbmm_ocs2/src/constraint/EsdfEnvironmentCollisionConstraint.cpp` | 实现 h 和 dh/dx |
| `src/control/wbmm_ocs2/include/wbmm_ocs2/WbmmInterface.h` | 增加双参考 / 阶段 / backend 接口 |
| `src/control/wbmm_ocs2/src/WbmmInterface.cpp` | 组装双参考阶段权重、可操作度、ESDF backend |
| `src/control/wbmm_ocs2/src/constraint/EndEffectorConstraint.cpp` | 增加目标维度保护 |
| `src/control/wbmm_ocs2/src/cost/WholeBodyTrajectoryCost.cpp` | 增加目标维度保护 |
| `src/control/wbmm_ocs2/CMakeLists.txt` | 增加新源文件/依赖 |
| `src/control/wbmm_ocs2/package.xml` | 增加 `wbmm_environment`、`ocs2_sphere_approximation` |
| `src/control/wbmm_ocs2_ros/src/WbmmMpcNode.cpp` | 订阅 whole-body/EE target、task phase |
| `src/control/wbmm_ocs2_ros/src/WbmmMrtNode.cpp` | phase 订阅、reset 按阶段、policy phase hold |
| `src/control/wbmm_ocs2_ros/src/WbmmTargetNode.cpp` | 发布 7D 目标到 ee_target 话题 |
| `src/control/wbmm_ocs2_ros/src/remani_to_ocs2_reference_bridge.cpp` | 发布 9D 参考到 whole_body_target 话题 |
| `src/bringup/config/sim/task.info` | 增加双参考阶段权重、`endEffectorTracking`、`armManipulability`、ESDF 配置 |
| `src/bringup/config/real/task.info` | 同上，按实机调整 |
| `src/bringup/config/*/ocs2.yaml` | whole-body/EE target 与 task phase 话题参数 |

---

## 10. 当前仍需继续优化/修改的地方（REVIEW）

> 本节根据当前仓库代码状态，以及《WBMM 导航—操作联合规划与全身协调控制架构设计》整理。  
> 状态说明：CURRENT = 已在代码中实现；PROPOSED = 建议后续实现；TBD = 待确认。

### 10.1 当前实现进度对照

| 方向 | 当前状态 | 主要缺口 / 下一步 |
|---|---|---|
| 双参考 Reference | CURRENT：`WbmmReferenceManager` 已支持 whole-body / EE 两路 target 和 `TaskPhase` | ROS 侧仍未接双 target 和 `task_phase` |
| 阶段权重 Cost | CURRENT：`PhaseWeightedStateCost` 已支持按 phase 加权 | 生产 `task.info` 尚未迁移；ROS phase 闭环未完成 |
| 全身跟踪 Cost | CURRENT：`WholeBodyTrajectoryCost` | 建议拆成 `BaseTrackingCost` + `ArmPostureCost`，弱化全 9D 强跟踪 |
| 末端跟踪 Cost | CURRENT：`EndEffectorTrackingCost` | 尚未接入导纳/力控的 EE 修正 |
| 可操作度 Cost | CURRENT：`ArmManipulabilityCost` | 权重、量纲缩放、与 `wbmm_robot_metrics` 的统一仍需确认 |
| ESDF 数据与查询 | CURRENT：`wbmm_environment` 已实现 NPZ 加载和三线性距离/梯度 | 动态更新、frame 对齐、unknown 策略仍需工程化 |
| ESDF 控制约束 | CURRENT：`EsdfEnvironmentInterface` / `EsdfEnvironmentCollisionConstraint` | 规划侧尚未使用同一 ESDF 数据源 |
| 规划 Search | CURRENT：Kino A* 只有 base 3D 路径 | 缺 TaskEntryRegion、whole-body seed、TaskPlan |
| 规划 Optimization | PROPOSED：`planning/optimization` 仍为空 | 需实现固定 dt 的 whole-body 优化 |
| Collision / Metrics | PROPOSED：`wbmm_collision`、`wbmm_robot_metrics` 仍为骨架 | 与 OCS2 内部实现存在重复，需要统一或明确边界 |
| 力控 | CURRENT：导纳 + `WholeBodyKinematics` + `base_share` + IK | 需改为“导纳 → EE correction → OCS2” |
| ROS / Launch / Config | PROPOSED：仍以单 `_mpc_target` 和旧配置为主 | 需接双 target、phase、ESDF backend 和生产配置 |

---

### 10.2 P0：Reference / ROS 接口闭环

**问题**

- `wbmm_ocs2` 内部已经支持双参考，但 `wbmm_ocs2_ros` 尚未闭环：
  - `WbmmMpcNode.cpp` 仍只有一个 `_mpc_target` 订阅；
  - `WbmmMrtNode.cpp` 仍使用 `use_whole_body_target` 参数；
  - `WbmmTargetNode.cpp` 仍只发 7D 目标；
  - `remani_to_ocs2_reference_bridge.cpp` 仍只发 9D 参考；
  - 没有 `task_phase` 话题；
  - 没有 MRT policy phase mismatch hold。

**PROPOSED**

- 固定三个 ROS 话题：

```text
/mobile_manipulator_whole_body_target
/mobile_manipulator_ee_target
/mobile_manipulator_task_phase
```

- `WbmmMpcNode`：
  - 分别订阅两个 target 话题；
  - 订阅 `task_phase`；
  - 调用 `WbmmInterface::setWholeBodyTarget()`、`setEndEffectorTarget()`、`setTaskPhase()`。
- `WbmmMrtNode`：
  - 订阅 `task_phase`；
  - `resetMpc()` 按当前 phase 选择 9D 或 7D 初始 target；
  - `evaluatePolicy()` 的 mode 与当前 phase 不一致时 hold。
- `WbmmTargetNode`：只发布 EE target。
- `remani_to_ocs2_reference_bridge`：只发布 whole-body target。
- 不再让单一 `_mpc_target` 同时承载 7D/9D 两种语义。

---

### 10.3 P0：去除核心 `base_share`

**问题**

当前力控链路仍是：

```text
Force Sensor
  -> Force Processor
  -> Admittance
  -> WholeBodyKinematics::correctedStateWorld6D(base_share + IK)
  -> 9D reference
```

相关位置：

- `src/control/whole_body_force_control/src/node.cpp`
- `src/robotics/wbmm_pinocchio/src/whole_body_kinematics.cpp`
- `src/bringup/config/*/force_control.yaml`

`base_share` 与目标框架冲突：

```text
Base/Arm 比例应成为优化结果，而不是人工控制参数。
```

**PROPOSED**

- 短期：
  - 保留 `base_share` 作为 fallback；
  - 增加 `whole_body_allocation_enabled` 开关；
  - 开启后导纳不再生成 9D 参考。
- 长期：
  - 导纳只输出 EE pose correction；
  - 通过 EE target 进入 OCS2；
  - 由 OCS2 决定 `[v, omega, qdot]`；
  - `WholeBodyKinematics` 降级为调试、初值生成或安全回退工具。

---

### 10.4 P1：拆分 Tracking Cost

**问题**

当前主要跟踪项：

```text
WholeBodyTrajectoryCost
EndEffectorTrackingCost
ArmManipulabilityCost
```

仍缺少目标框架中的：

```text
BaseTrackingCost
ArmPostureCost
```

**PROPOSED**

- 新增 `BaseTrackingCost`：
  - 只跟踪 `x_b, y_b, yaw`。
- 新增 `ArmPostureCost`：
  - 跟踪 `q_nom` 或导航舒适构型。
- `WholeBodyTrajectoryCost` 降级为弱 nominal 参考，避免 Planner 锁死全部 9D 状态。
- 推荐 Mode-Cost 关系：

| Mode | Base Tracking | Arm Posture | EE Tracking | Singularity | Collision |
|---|---:|---:|---:|---:|---:|
| Navigation | 强 | 中 | 关闭 | 弱 | 开 |
| Approach | 弱 | 弱 | 强 | 开 | 开 |
| Execute | 关闭/很弱 | 很弱 | 强 | 强 | 开 |
| Retreat | 弱 | 中 | 强 | 开 | 开 |

---

### 10.5 P1：补齐 Planning / Optimization 链路

**问题**

当前：

```text
planning/search        -> 只有 base Kino A*
planning/optimization  -> 空
planning/task_planner  -> 不存在
```

无法输出目标框架中的：

```text
TaskPlan
  ├── ModeSchedule
  ├── BaseReference
  ├── ArmPostureReference
  ├── EndEffectorReference
  └── ForceReference
```

**PROPOSED**

- `wbmm_search` 输出：
  - base path；
  - task entry region；
  - phase hint。
- 新增 `TaskPlan` 数据结构。
- `planning/optimization` 先实现：
  - 固定 dt；
  - 差速非完整约束；
  - base + arm 联合初值；
  - 代价包含 base tracking、posture、EE、collision、singularity、joint limit、input smoothness。
- Planner 只输出“任务参考 + 弱 nominal”，不要锁死全部 9D 状态。

---

### 10.6 P1：统一 Collision / Metrics

**问题**

当前存在重复：

```text
map/wbmm_environment        -> ESDF 距离场（已实现）
robotics/wbmm_collision     -> 环境碰撞骨架（未实现）
metrics/wbmm_robot_metrics  -> 构型评价骨架（未实现）

control/wbmm_ocs2
  -> EsdfEnvironmentInterface
  -> EsdfEnvironmentCollisionConstraint
  -> ArmManipulabilityCost
```

控制层已经直接使用 `wbmm_environment` 和 Pinocchio sphere，但规划层 `wbmm_collision`、`wbmm_robot_metrics` 尚未接上。

**PROPOSED**

二选一：

1. 统一到 `wbmm_collision` / `wbmm_robot_metrics`：
   - OCS2 的 ESDF constraint 和 manipulability cost 改为调用公共模块。
2. 明确 OCS2 内部实现为控制专用：
   - 规划侧使用 `wbmm_collision`；
   - 控制侧使用 OCS2 版本；
   - 但必须保证碰撞球、余量、frame、unknown 策略一致。

否则会出现：

```text
规划认为无碰撞，控制认为碰撞
规划 sigma_min 与控制 sigma_min 不一致
```

---

### 10.7 P2：ESDF 工程化

**CURRENT**

- `wbmm_environment` 已支持 NPZ 加载和 ESDF 查询。
- OCS2 已支持 `environmentCollision.backend = "esdf"`。

**仍需优化**

- 动态 ESDF 在线更新；
- ESDF frame 与 OCS2 state frame 的显式校验；
- unknown / out-of-bounds 策略统一；
- map revision / stamp 与 `WholeBodyTrajectory` 关联；
- 规划与控制使用同一个 `EsdfGrid` 快照；
- nvblox 在线服务或点云到 `EsdfGrid` 的实时转换。

---

### 10.8 P2：TaskPhase / ExecutionPhase 统一

当前有两套阶段定义：

```text
wbmm::core::ExecutionPhase
  kIdle / kNavigate / kPreExecution / kExecution / kTracking / kFinish / kFault

wbmm_ocs2::TaskPhase
  kNavigation / kTransition / kExecution / kRetract
```

**PROPOSED**

- 统一一套 TaskMode / TaskPhase：

```text
NAVIGATION
APPROACH
EXECUTE
RETREAT
```

- `ExecutionPhase` 作为系统级状态；
- `TaskPhase` 作为 MPC 代价权重阶段；
- 建立映射：

```text
kNavigate       -> Navigation
kPreExecution   -> Approach
kExecution      -> Execute
kTracking       -> Execute
kFinish         -> Retreat
kFault          -> Hold / Recovery
```

- 外部 `task_phase` 消息与 OCS2 `ModeSchedule` 使用同一套编号。

---

### 10.9 P2：Input Smoothing / Joint Limit / Contribution Metrics

**PROPOSED**

- 增加 input rate / delta-u 代价：

$$
w_{\Delta u}J_{\Delta u}
$$

- 实现 `wbmm_robot_metrics::JointLimitMetrics`；
- 在日志或 MRT 中计算 Base/Arm contribution：

$$
lpha_b=
rac{\|V_{base}\|}
{\|V_{base}\|+\|V_{arm}\|+\epsilon}
$$

- 用于实验 A/B/C 对比，不参与控制律。

---

### 10.10 P3：力控重新接入

目标框架：

```text
Force Sensor
  -> Force Processor
  -> Admittance
  -> EE Reference Correction
  -> Whole-Body NMPC
```

当前：

```text
Force Sensor
  -> Force Processor
  -> Admittance
  -> 9D correctedStateWorld6D(base_share + IK)
  -> OCS2 reference
```

**PROPOSED**

- 导纳输出从 9D state 改成 EE pose correction；
- 通过 EE target 进入 OCS2；
- OCS2 负责决定 base/arm 如何实现；
- 力控安全门、限幅、watchdog、SAFE_HOLD 保持不变；
- 实机切换必须人工逐条确认。

---

### 10.11 推荐优先级

```text
P0:
  1. ROS 双 target + task_phase 闭环
  2. 去除 base_share 核心链路

P1:
  3. BaseTrackingCost / ArmPostureCost
  4. planning/optimization + TaskPlan
  5. 统一 collision / metrics

P2:
  6. ESDF 工程化与统一地图
  7. Mode/Phase 统一
  8. input smoothing / contribution metrics

P3:
  9. 力控重新接入
  10. contact-aware NMPC
```

---

## 11. 任务输出格式

```text
任务类型：MIXED / DESIGN + REVIEW
修改文件：docs/wbmm_ocs2_mode_manipulability_esdf_design.md
是否修改源码：否
验证方式：
  - 阅读并交叉核对 wbmm_ocs2 / wbmm_ocs2_ros / wbmm_environment / REMANI GridMap 源码
  - 设计层验证：指标公式、ESDF 梯度、模式切换时序、目标维度保护
  - 后续实现后需补：NPZ 加载测试、ESDF 插值测试、可操作度数值差分测试、双参考阶段切换单测、MuJoCo ESDF 闭环仿真
不确定项：
  - ESDF frame 与 OCS2 state frame 的最终统一方式
  - 动态 ESDF 在线更新方式
  - 可操作度代价权重量纲和缩放
  - 实机模式切换 hold/急停策略
  - URDF 中所有 collisionLinks 是否都能被 PinocchioSphereInterface 处理
人工审查结论：DRAFT
```
