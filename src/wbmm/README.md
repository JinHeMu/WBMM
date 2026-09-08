# wbmm 主程序包（目标骨架）

> 当前仅为目录骨架，用于把后续 task/model/environment/planning/execution/contact/adapters/cli
> 按职责收敛到同一个包内，同时继续保留 `tracer_jaka_ocs2`、`whole_body_force_control`、
> `tracer_jaka_bringup` 等重依赖/集成包。

## 目录职责

| 路径 | 职责 | 依赖边界 |
|---|---|---|
| `src/node.cpp` | 收消息、读配置、调用主程序、发状态 | ROS 外壳 |
| `src/runtime.cpp` | 组装对象、快照、规划工作线程、取消与结果交付 | 不写具体搜索/力控公式 |
| `src/task/` | 任务解析、轨迹生成、切入构型与评分 | 纯算法 target |
| `src/model/` | Pinocchio FK、Jacobian、关节映射与几何 | 纯模型 target |
| `src/environment/` | 地图读取、距离查询、碰撞检查 | 不在算法内创建 ROS 订阅者 |
| `src/planning/` | pipeline、search、seed_builder、optimization、validator | 搜索与优化可独立运行 |
| `src/execution/` | FSM、进度管理、参考选择与采样 | 管阶段和参考，不直接驱动电机 |
| `src/contact/` | 力数据处理、接触监督、修正策略 | 复用已有导纳数学库 |
| `src/adapters/` | legacy REMANI 通信、ROS 类型、OCS2 目标消息转换 | vendor/ROS 类型集中在这里 |
| `src/cli/` | 离线任务、搜索、优化与对比入口 | 不启动机器人 |
| `config/` | robot、planner、execution、任务与场景索引 | 原生 OCS2 `.info` 用引用方式接入 |

## 状态

- 当前尚未迁移实际代码；
- 现有 TA-WBMP / Coordinator / wipe_planner 仍作为回归基线；
- 新代码按本骨架落位，避免继续向 `ta_wbmp`、`wipe_planner` 等旧包堆积职责。
