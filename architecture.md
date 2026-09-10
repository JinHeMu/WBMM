# WBMM Architecture

> 技术合同：所有新功能、重构、Bug 修复都必须先对照本文档，再修改代码。
> 状态：持续维护；当前仓库处于“旧 ROS 主链保留 + 新核心逐步收敛”的重构阶段。
> 若实现与本文档不一致，以本文档为基准更新代码或文档；涉及稳定接口变化时先写 ADR。

## 1. 当前阶段定位

WBMM 当前不是“单一新架构已替换旧链”的状态，而是：

- 保留现有可运行的 ROS 2 / MuJoCo / OCS2 / REMANI 链路作为回归基线；
- 逐步把通用状态、结果、数学、模型、环境、任务和执行能力收敛到轻量核心；
- 先建立 `wbmm_core`，再按真实调用接入，避免一次大规模 Port 化。

因此本文件同时描述：

1. **目标架构**：未来希望形成的轻量主链；
2. **当前架构**：现在实际运行的 ROS 主链；
3. **迁移路径**：从当前到目标的小步迁移顺序。

## 2. 目标架构

```text
WbmmNode（ROS 输入输出）
      ↓ 转成 wbmm_core 类型
WbmmFSM（阶段与故障）
      ↓
WbmmManager（唯一业务入口）
      ├── RobotModel         # FK、Jacobian、关节限制、碰撞几何
      ├── Environment        # 距离、占据、环境版本
      ├── PlannerBackend     # 状态/任务/环境 → 全身轨迹
      └── ControllerBackend  # 参考/反馈 → 全身命令
      ↓
轨迹 / 命令 / 状态
```

依赖方向：

```text
ROS 接口 / 应用 / bringup
        ↓
WbmmManager / execution
        ↓
core 类型 + math + RobotModel / Environment / backend 接口
        ↓
vendor / 具体求解器 / 驱动
```

当前目录与目标职责：

```text
src/
├── core/                      # wbmm_core：类型、Result、校验、math、少量稳定接口
├── planning/                  # 规划容器：当前为 ta_wbmp
├── control/                   # OCS2 与力控集成
├── visual/                    # wbmm_visualization 统一显示
├── map/                       # 定位、建图、ESDF 与地图工具
├── sim/                       # 仿真后端容器
├── applications/              # 具体任务与回归基线
├── robot/                     # 机器人描述与模型资源
├── drivers/                   # 硬件驱动
├── bringup/                   # tracer_jaka_bringup：唯一顶层组合
└── vendor/                    # 上游工程与重依赖
```

`core`、`visual`、`bringup` 是系统内唯一能力，目录本身即 ROS 包根。
`planning`、`control`、`map`、`sim` 是可容纳多个实现的包容器。不再保留
空的 `src/wbmm` 骨架；只在开始实现 WbmmNode/FSM/Manager 主链时创建真实主程序包。

## 3. 当前实际主链（保留基线）

当前仍可运行的主链：

```text
统一任务 YAML
  → TA-WBMP 任务轨迹与候选规划
  → ExecutionCoordinator / REMANI 导航到 q_pre
  → 显式参考所有权交接
  → OCS2 MPC/MRT 跟踪全身参考
  → 可选 whole_body_force_control
  → MuJoCo / 实机
```

`wipe_planner` 不再作为新主链规划器，但仍作为接触安全监督和任务行为回归基线保留。

## 4. 核心数据合同

### 4.1 领域类型与数学

- `wbmm_core` 存放统一领域类型：
  - `WholeBodyState`
  - `WholeBodyInput`
  - `TaskTrajectory`
  - `WholeBodyTrajectory`
  - `EnvironmentSnapshot`
  - `PlanningResult`
  - `Status / Result<T>`
- 数学与转换已迁入 `wbmm_core/math/`：
  - `conversions.hpp`
  - `linear_algebra.hpp`
  - `math.hpp`
  - `wbmm_math.hpp`
- 旧 `wbmm_math` 兼容包已删除，数学 API 的唯一位置为 `src/core/include/wbmm_core/math/`。
- 算法内部允许 Eigen，ROS/适配层在边界完成转换，领域结构保持普通 C++ 字段。

### 4.2 状态与控制维度

全身状态固定为 9 维：

```text
x = [x_b, y_b, yaw_b, q1, q2, q3, q4, q5, q6]^T
```

控制输入固定为 8 维：

```text
u = [v_b, omega_b, qdot1, qdot2, qdot3, qdot4, qdot5, qdot6]^T
```

- 关节顺序显式、可审计；禁止猜测、补零或静默重排。
- 差速底盘不允许横向速度。
- 所有空间量必须声明 frame；时间量必须声明 clock。

### 4.3 参考所有权

同一时刻只能有一个 MPC/执行参考所有者：

- `NAVIGATING`：REMANI bridge / 对应导航后端。
- `TASK_EXEC`：ExecutionCoordinator / 新 WbmmManager。
- 所有权切换必须有明确请求、确认与超时检查。

## 5. 模块职责

| 模块 | 职责 | 当前状态 |
|---|---|---|
| `src/core` | `wbmm_core`：统一类型、Result、校验、数学、少量稳定接口 | 数学已合并，可独立构建 |
| `src/planning/ta_wbmp` | 任务轨迹、候选规划、执行协调 | 保留，后续按真实主链收敛 |
| `src/control/tracer_jaka_ocs2` | OCS2 MPC/MRT 集成 | 保留独立集成 |
| `src/control/whole_body_force_control` | 导纳/恒力/力跟随 | 保留，迁移接触监督 |
| `src/visual` | `wbmm_visualization` 统一显示 | 保留 |
| `src/applications/wiping/wipe_planner` | 旧接触执行基线 | 冻结/回归基线 |
| `src/robot` | 机器人描述/模型 | 保留，收敛唯一模型源 |
| `src/drivers` | 硬件驱动 | 保留 |
| `src/map` | ESDF、地图、定位 | 保留，统一查询语义 |
| `src/sim/tracer_jaka_mujoco` | MuJoCo 仿真 I/O 与场景 | 保留为仿真后端 |
| `src/bringup` | `tracer_jaka_bringup`：顶层 launch/部署 | 唯一完整系统组合入口 |

## 6. 接口与转换边界

### 6.1 稳定接口（目标最小集合）

- `RobotModel`
- `Environment`
- `PlannerBackend`
- `ControllerBackend`

当前 `wbmm_core` 内仍保留较多 Port 原型；在没有真实调用者前冻结，不继续扩展。

### 6.2 边界规则

- `wbmm_core` 不依赖 ROS、具体机器人、vendor、求解器。
- 核心不依赖 `ament_cmake`：可用 `WBMM_BUILD_OFFLINE=ON` 进行纯 CMake/CTest 构建。
- ROS 消息、TF、ros2_control 只在 adapter/application/bringup 边界出现。
- 算法包可使用 Eigen，但对外输出应是核心领域类型或由边界转换后的 ROS 类型。

## 7. 迁移顺序

1. **M01 基础类型与数学**：已完成 math 迁入 `wbmm_core`、离线构建、关节映射与 round-trip。
2. **M02 机器人模型与碰撞几何**：收敛 URDF、FK/IK/Jacobian、碰撞几何。
3. **M03 环境/ESDF 与碰撞检查**：统一地图查询与碰撞语义。
4. **M04+ 任务/规划/控制拆分**：把 TA-WBMP、力控、协调器按职责拆入新主链。
5. 按真实调用者裁剪旧 Ports 和已迁移旧模块。

## 8. 文档与验证

- 坐标系详细合同见 [frames.md](frames.md)。
- 仓库级修改与历史记录见 [CHANGELOG.md](CHANGELOG.md)。
- 验证分级沿用 L0–L6；每个改动必须记录实际达到的等级，不得用“能启动”代替“已验证”。
