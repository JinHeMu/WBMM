# wbmm/src

本目录是 `wbmm` 主程序包的内部源码目录。每个子目录对应一个职责边界；同一个包内可拆多个 CMake target。

| 子目录 / 文件 | 职责 |
|---|---|
| `node.cpp` | ROS 外壳：收消息、读配置、调 runtime、发状态 |
| `runtime.cpp` | 装配与调度：组装对象、规划线程、取消与结果交付 |
| `task/` | 任务解析、轨迹生成、切入构型与评分 |
| `model/` | Pinocchio FK、Jacobian、关节映射与几何 |
| `environment/` | 地图读取、距离查询、碰撞检查 |
| `planning/` | pipeline、search、seed_builder、optimization、validator |
| `execution/` | FSM、进度管理、参考选择与采样 |
| `contact/` | 力数据处理、接触监督、修正策略 |
| `adapters/` | legacy REMANI/OCS2 通信与 ROS 类型转换 |
| `cli/` | 离线任务、搜索、优化与对比入口 |
