# WBMM 模块职责与修改记录

> 本文件用于长期记录 WBMM 的模块职责、工作目录约定，以及每一次代码/架构修改。
> 规则：**只追加、不删除历史**；每次修改完成后，在文末“修改记录”增加一条新记录。

---

## 1. 模块职责（当前基线）

下表是当前工作空间模块职责和重构处理意见的整理。后续迁移时以“当前状态”列逐步推进。

| 模块 / 目录 | 实际作用 | 重构处理 | 当前状态 |
|---|---|---|---|
| `src/core/wbmm_core` | 状态、结果、Port 原型、数学转换、校验 | 作为唯一核心库；数学已迁入 `math/`，Port 后续按真实调用裁剪 | 已开始收敛 |
| `src/core/wbmm_math` | 旧 Eigen 数学包 | 保留为兼容转发层；所有使用者切换后删除 | 兼容转发中 |
| `src/algorithms/planning/ta_wbmp` | 候选枚举、IK、评分、导航预览、接近/任务拼接、验证 | 按职责拆分到 planning/task 等子目录 | 待拆分 |
| `src/algorithms/control/tracer_jaka_ocs2` | MPC 装配、MRT、目标输入、轨迹 bridge | 保留独立集成包，内部重构 | 保留 |
| `src/algorithms/control/whole_body_force_control` | 导纳、恒力/力跟随与位姿修正 | 复用数值库；应用监督迁入 `contact` | 待迁移 |
| `src/algorithms/visualization/wbmm_visualization` | 统一轨迹/机器人显示 | 保留，消费算法输出数据 | 保留 |
| `src/applications/wiping/wipe_planner` | 旧规划与接触执行基线 | 暂保留；按功能迁移，不整包扩展 | 冻结基线 |
| `src/robot/` | 机器人模型、描述、网格 | 保留主要结构，修正边界 | 保留 |
| `src/drivers/` | 硬件驱动：底盘、机械臂、传感器 | 保留主要结构，修正边界 | 保留 |
| `src/perception/` | ESDF、地图、定位、感知 | 保留主要结构，先统一查询语义 | 保留 |
| `src/bringup/` | 系统组合、launch、部署 | 保留主要结构，修正部署配置 | 保留 |
| `docs/` | 架构、实验、迁移记录 | 与根目录记录文件配合维护 | 保留 |

---

## 2. 工作目录约定

当前源码仍以 ROS 2 包形式组织；随着重构推进，应逐步收敛到以下职责边界：

```text
src/
├── core/
│   ├── wbmm_core/                 # 唯一基础库（领域类型 + math + 校验 + 必要接口）
│   └── wbmm_math/                 # 兼容转发层，目标删除
├── algorithms/
│   ├── control/                   # 控制集成（OCS2、力控等）
│   ├── planning/                  # 规划集成与算法（TA-WBMP 等）
│   └── visualization/             # 显示/可视化
├── applications/                  # 具体应用任务（如 wiping/wipe_planner）
├── robot/                         # 机器人描述与模型资源
├── drivers/                       # 硬件驱动
├── perception/                    # 环境、地图、ESDF、定位
└── bringup/                       # 顶层系统组合
```

核心原则：

1. `wbmm_core` 不依赖 ROS、具体机器人、后端求解器；
2. Eigen 只用于 `wbmm_core/math/` 的转换与算法内部，不进入领域数据结构；
3. 算法包内部可使用 Eigen，但 ROS 消息/TF 只在边界转换；
4. 旧模块在迁移完成前保持可运行，作为回归基线；
5. 删除旧模块前，先确认所有真实调用者已切换。

---

## 3. 修改记录

### 2026-09-05 前（历史基线）：现有 WBMM 工作区与核心原型

- 仓库已形成 ROS 2 / MuJoCo / OCS2 / REMANI / 机械臂与底盘驱动的工作区结构。
- `wbmm_core` 已建立领域类型、状态/结果、校验和 Port 原型。
- `wbmm_math` 已建立 Eigen 转换、关节映射和线性代数工具。
- 当时架构文档已提出：`wbmm_math` 应合入 `wbmm_core/math/`，Ports 应裁剪为四个稳定接口。
- 该阶段作为后续重构的功能回归基线，相关旧入口在未替换前继续保留。

---

### 2026-09-08：M01 基础类型与数学优化（本次）

**改动目标**  
让基础类型与数学先收敛到 `wbmm_core`，同时支持无 ROS/ament 的离线构建。

**实际改动**

- 将 `wbmm_math` 的实现迁入 `wbmm_core/math/`：
  - 新增 `include/wbmm_core/math/conversions.hpp`
  - 新增 `include/wbmm_core/math/linear_algebra.hpp`
  - 新增 `include/wbmm_core/math/math.hpp`
  - 新增 `include/wbmm_core/math/wbmm_math.hpp`
  - 新增 `src/math/conversions.cpp` 与 `src/math/linear_algebra.cpp`
- `wbmm_math` 改为兼容转发包：
  - 头文件转发到 `wbmm_core/math/`
  - 旧源码不再参与编译，新增 `src/compat.cpp` 保持 target 存在
- 补齐差速状态反向转换：
  - 新增 `differentialDriveStateFromVector()`
  - 状态和输入都具备领域类型 <-> Eigen 定长向量的 round-trip
- 为 `wbmm_core` 增加离线构建选项：
  - `WBMM_BUILD_OFFLINE=ON`
  - 别名：`WBMM_OFFLINE=ON`、`WBMM_ENABLE_AMENT=OFF`
  - 离线时不再 `find_package(ament_cmake)`，使用普通 CMake + CTest + GTest
- 更新 `package.xml`：`wbmm_core` 增加 Eigen 依赖
- 更新 README、QUALITY_DECLARATION、架构现状文档
- 增加核心数学测试，覆盖：
  - 乱序关节按名称映射
  - 缺失关节
  - 重复关节名
  - 状态/输入 round-trip
  - 错误维度与非有限值

**涉及文件（主要）**

```text
src/core/wbmm_core/CMakeLists.txt
src/core/wbmm_core/package.xml
src/core/wbmm_core/include/wbmm_core/wbmm_core.hpp
src/core/wbmm_core/include/wbmm_core/math/*
src/core/wbmm_core/src/math/*
src/core/wbmm_core/test/test_math_*.cpp
src/core/wbmm_math/CMakeLists.txt
src/core/wbmm_math/include/wbmm_math/*
src/core/wbmm_math/src/*
```

**验证**

```text
colcon test --packages-select wbmm_core wbmm_math
# 全部通过

cmake -S src/core/wbmm_core -B /tmp/wbmm_core_offline \
  -DWBMM_BUILD_OFFLINE=ON -DBUILD_TESTING=ON
cmake --build /tmp/wbmm_core_offline
ctest --test-dir /tmp/wbmm_core_offline --output-on-failure
# 6/6 passed
```

---

### 2026-09-08：文档同步与旧目录设计删除

**改动范围**

- 根据 `WBMM架构评估与重构执行方案.md` 重新更新根目录工程治理与架构文档：
  - `仓库开发与维护标准.md`
  - `architecture.md`
  - `frames.md`
  - `README.md`
- 删除旧的 `工程目录重构设计.md`，其有效内容已合并到以上文档与 `CHANGELOG.md`。
- 修正 README 与 `docs/QUICKSTART.md` 中文档链接，改为指向根目录的 `architecture.md`、`frames.md`、`仓库开发与维护标准.md`。
- 在 README 中补充 `wbmm_core` 离线构建说明与重构状态说明。

**验证**

- Markdown 文档链接与内容人工核对；
- 不涉及 C++ 代码逻辑，无需重新跑构建；原有 M01 代码测试保持通过。

---

### 2026-09-08：建立 `src/wbmm` 主程序包目录骨架

**改动范围**

- 根据重构方案 5.1 的目录职责，新增 `src/wbmm/` 骨架：
  - `src/wbmm/src/node.cpp`、`src/wbmm/src/runtime.cpp` 对应入口位置暂未创建文件，先建立目录说明；
  - `src/wbmm/src/task/`
  - `src/wbmm/src/model/`
  - `src/wbmm/src/environment/`
  - `src/wbmm/src/planning/`
  - `src/wbmm/src/execution/`
  - `src/wbmm/src/contact/`
  - `src/wbmm/src/adapters/`
  - `src/wbmm/src/cli/`
  - `src/wbmm/config/`
  - `src/wbmm/launch/`、`src/wbmm/rviz/`、`src/wbmm/test/`
- 新增 `src/wbmm/README.md` 记录目录职责与当前状态。
- 同步更新 `architecture.md`、`README.md`、`仓库开发与维护标准.md` 的目标目录树。

**当前状态**

- 只是目录骨架，尚未加入 ROS package.xml / CMake target；
- 旧 TA-WBMP / wipe_planner / 力控 / OCS2 仍作为现有基线保留；
- 后续迁移实际代码时，再为 `src/wbmm` 建立 ROS 包并编译多个内部 target。

**验证**

- 不影响现有 colcon 包发现与构建；
- 纯目录与文档变更。

---

### 后续计划（待执行）

- **M02：机器人模型与碰撞几何**
  - canonical URDF、tool0、关节名、底盘参考点
  - FK/IK/Jacobian 模型查询
  - 与 TA / REMANI / 力控 / 可视化做交叉对照
- **M03：环境、ESDF 与碰撞检查**
  - 统一地图查询语义
  - 碰撞检查与距离查询收敛
- **M04+：任务轨迹、规划器拆分、控制/监督迁移**
  - `ta_wbmp` 的 task/planner/execution 拆分
  - `whole_body_force_control` 的 contact 能力迁移
  - `wbmm_math` 删除条件：所有使用方切到 `wbmm_core/math/`
  - Ports 按真实调用裁剪

---

## 4. 以后如何维护本文件

每次修改完成后，请：

1. 在 `## 3. 修改记录` 下追加一条新记录；
2. 记录日期、改动范围、关键文件、验证结果；
3. 若模块职责或目录发生变化，同步更新第 1、2 节；
4. 不删除历史记录，便于回溯“以前 / 现在 / 以后”。
