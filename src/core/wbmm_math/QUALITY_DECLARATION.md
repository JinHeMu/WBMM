# wbmm_math 质量声明

## 当前状态

- 版本：`0.1.0`；
- 成熟度：实验性数学和转换层；
- 架构状态：临时独立包，目标合入 `wbmm_core/math/`；
- ROS 质量等级：暂不声明；
- 目标证据：L1 纯 C++ 单元测试；
- 不声明硬实时、规划正确性、闭环稳定性、仿真或实机安全。

## 公共 API

- `conversions.hpp`：领域类型、具名关节和 Eigen 类型转换；
- `linear_algebra.hpp`：机器人无关的线性代数工具；
- `wbmm_math.hpp`：聚合头文件。

## 依赖规则

- 允许：`wbmm_core`、Eigen3、C++17 标准库；
- 禁止：ROS 消息/节点、具体机器人描述、硬件 SDK、Pinocchio、OCS2、REMANI、MuJoCo；
- `wbmm_core` 禁止反向依赖 `wbmm_math`；
- 只有需要 Eigen 表达的 algorithms/adapters 才依赖本包。

## 变更门禁

公共转换顺序、坐标约定、伪逆公式或容差发生变化时，必须：

1. 更新 README 中的数学合同；
2. 增加正常、奇异、维度错误和非有限值测试；
3. 独立构建 `wbmm_core` 与 `wbmm_math`；
4. 若影响具体后端，再由对应 adapter 提供集成回归证据。
