# wbmm_math

`wbmm_math` 现在是 `wbmm_core/math/` 的兼容转发包。新代码请直接使用：

```text
wbmm_core/math/conversions.hpp
wbmm_core/math/linear_algebra.hpp
wbmm_core/math/math.hpp
wbmm_core/math/wbmm_math.hpp
```

原 `wbmm_math` 头文件只做转发，目标库链接 `wbmm_core`。所有实现已经迁入 `wbmm_core`；删除本包前保留它给尚未切换的旧调用方过渡。

```text
wbmm_core <- wbmm_math (compatibility only)
```

## 职责

- `wbmm_core::Vector3/Quaternion/Pose/Wrench` 与 Eigen 类型转换；
- `wbmm_core::Matrix` 与 `Eigen::MatrixXd` 的显式行主序转换；
- 按 joint name 映射关节位置和速度；
- 差速底盘状态和输入的 adapter-facing Eigen 向量转换；
- skew-symmetric matrix、阻尼伪逆和阻尼最小二乘。

不负责：

- Pinocchio、OCS2、REMANI 或其他后端调用；
- ROS 消息转换；
- 规划、控制和碰撞策略；
- 为缺失关节补零或猜测数组顺序；
- 硬实时内存管理。

## 显式约定

```text
Wrench Eigen order = [Fx, Fy, Fz, Tx, Ty, Tz]

Differential-drive state = [base_x, base_y, base_yaw, q...]
Differential-drive input = [base_v, base_yaw_rate, qdot...]
```

关节部分始终根据调用方给出的 `expected_joint_order` 按名称重排。缺失、重复、额外关节、错误维度和 NaN/Inf 都返回失败。

阻尼伪逆采用奇异值滤波：

```text
sigma_inverse = sigma / (sigma^2 + damping^2)
```

当 `damping == 0` 时，使用带显式奇异值阈值的 Moore-Penrose 伪逆。

## 使用示例

```cpp
#include <wbmm_math/wbmm_math.hpp>

auto state_vector = wbmm::math::differentialDriveStateVector(
  state, expected_joint_order);
if (!state_vector.ok()) {
  // 拒绝进入后端，并记录 status code/message。
}
```

## 实时性说明

当前接口使用 `Eigen::VectorXd`、`Eigen::MatrixXd` 和 `std::vector`，可能产生动态内存分配，因此只声明为非实时数学/adapter 层。未来硬实时控制应增加固定维度、预分配且经过单独 WCET/分配检查的接口，不能直接把本包测试结果当作实时证据。

## 构建测试

```bash
colcon build --packages-select wbmm_core wbmm_math
colcon test --packages-select wbmm_core wbmm_math
colcon test-result --verbose
```

在线下构建中只需要构建 `wbmm_core`，无需 ament/ROS：

```bash
cmake -S src/core/wbmm_core -B build/wbmm_core_offline -DWBMM_BUILD_OFFLINE=ON
cmake --build build/wbmm_core_offline
ctest --test-dir build/wbmm_core_offline --output-on-failure
```
