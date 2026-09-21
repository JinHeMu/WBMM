# WBMM 数学与接口契约

> Status: ACTIVE
> Author: Agent  
> Reviewer: Jinhemu
> Reviewed at: TBD  
> Warning: 本文档尚未经过人工审查，不能作为实现依据。

> 本文档定义 WBMM 的输入输出、坐标系、轨迹、任务和力控的数学与接口语义。  
> 实现代码必须与本文档保持一致；如果实现与契约冲突，应先修正契约或实现，不能静默改变语义。

---

## 1. 符号与单位约定

### 1.1 基本符号

| 符号 | 含义 |
|---|---|
| $x$ | 全身状态向量 |
| $u$ | 全身输入向量 |
| $t$ | 时间，单位 s |
| ${}^{A}T_B$ | 从坐标系 B 到坐标系 A 的齐次变换 |
| ${}^{A}R_B$ | 从坐标系 B 到坐标系 A 的旋转矩阵 |
| ${}^{A}p_B$ | 坐标系 B 原点在坐标系 A 中的位置 |
| $\psi$ | 偏航角 yaw，单位 rad |
| $q$ | 机械臂关节角向量 |
| $\mathcal{F}$ | 六维力/力矩向量 |

### 1.2 单位约定

| 物理量 | 单位 |
|---|---|
| 长度 | m |
| 角度 | rad |
| 线速度 | m/s |
| 角速度 | rad/s |
| 线加速度 | m/s² |
| 角加速度 | rad/s² |
| 力 | N |
| 力矩 | N·m |
| 时间 | s |

### 1.3 旋转表示

- 矩阵表示使用 ${}^{A}R_B \in SO(3)$。
- 四元数内部顺序固定为：

$$
q = [w, x, y, z]
$$

- ROS 消息中的四元数顺序为 `xyzw`，进入内部后必须转换为 `wxyz`。
- 角度误差计算必须做最短路径 wrap：

$$
\operatorname{wrapToPi}(\theta) = \operatorname{atan2}(\sin\theta, \cos\theta)
$$

## 2. 坐标系定义

### 2.1 坐标系树

![WBMM coordinate frames](img/frame_tree.svg)

主要坐标系：

| 坐标系 | 语义 |
|---|---|
| `world` / `map` | 全局参考系，长期稳定，可能存在地图更新 |
| `odom` | 连续里程计参考系，局部平滑，长期可能漂移 |
| `base_footprint` | 底盘在地面上的投影系 |
| `base_link` | 底盘本体坐标系 |
| `jaka_base_link` | 机械臂基座坐标系 |
| `joint_1 ... joint_6` | 各旋转关节坐标系 |
| `tool0` / `ee_link` | 末端工具或末端执行器坐标系 |
| `task_frame` | 任务/表面坐标系，通常定义法向、切向和接触面 |
| `sensor_*` | 相机、力传感器、IMU 等传感器坐标系 |

### 2.2 齐次变换

齐次变换定义为：

$$
{}^{A}T_B =
\begin{bmatrix}
{}^{A}R_B & {}^{A}p_B \\
0_{1\times 3} & 1
\end{bmatrix}
$$

点变换关系：

$$
{}^{A}p = {}^{A}T_B \, {}^{B}p
$$

变换链：

$$
{}^{A}T_C = {}^{A}T_B \, {}^{B}T_C
$$

底盘到全局的变换：

$$
{}^{w}T_b(x_b) =
\begin{bmatrix}
\cos\psi_b & -\sin\psi_b & 0 & x_b \\
\sin\psi_b & \cos\psi_b & 0 & y_b \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

末端到全局的变换：

$$
{}^{w}T_{ee}(x) = {}^{w}T_b(x_b) \, {}^{b}T_{ee}(q)
$$

### 2.3 坐标系使用规则

- 所有规划、控制和日志必须显式携带 `frame_id`。
- 不允许在未知坐标系之间直接做加减。
- `odom` 与 `map` 不一致时，必须通过 TF 或显式变换处理。
- 速度、加速度、力/力矩在跨坐标系变换时不能只改 `frame_id`。
- 任务法向、切向、接触力必须明确表达在哪个坐标系下。

---

## 3. 输入与输出定义

### 3.1 全身状态

全身状态定义为：

$$
x = [x_b, y_b, \psi_b, q_1, q_2, q_3, q_4, q_5, q_6]^T \in \mathbb{R}^{9}
$$

其中：

- $x_b, y_b$：底盘在规划坐标系下的位置；
- $\psi_b$：底盘 yaw；
- $q_1, \dots, q_6$：六轴机械臂关节角。

结构对应：

```text
WholeBodyState
├── Header
│   ├── frame_id
│   └── stamp
├── BaseState
│   ├── x
│   ├── y
│   ├── yaw
│   ├── linear_velocity
│   ├── lateral_velocity
│   └── yaw_rate
└── JointState
    ├── names
    ├── positions
    ├── velocities
    └── efforts
```

### 3.2 字段命名与单位约定

`wbmm_core` 中的字段名默认不携带单位后缀，例如：

```text
x
y
yaw
linear_velocity
lateral_velocity
yaw_rate
positions
velocities
efforts
```

单位由契约统一约定：

| 物理量 | 单位 |
|---|---|
| 长度 | m |
| 角度 | rad |
| 线速度 | m/s |
| 角速度 | rad/s |
| 时间 | s |
| 力 | N |
| 力矩 | N·m |

这样做的原因是：

- 字段名保持简短；
- 避免出现 `x_m`、`yaw_rad`、`velocities_radps` 这类重复信息；
- 单位由 `docs/math_contract.md` 和 `wbmm_core` 统一约束；
- ROS 边界再做显式单位转换和消息字段转换。

通用 core 只检查“差速底盘 + 六关节”的结构契约，不硬编码具体 JAKA 关节名称。  
具体关节名称、顺序和限位由 `RobotModel::validate()` 检查。  
当前 WBMM 平台使用：

```text
joint_1 ... joint_6
```

### 3.3 全身输入

全身输入定义为：

$$
u = [v, \omega, \dot q_1, \dot q_2, \dot q_3, \dot q_4, \dot q_5, \dot q_6]^T \in \mathbb{R}^{8}
$$

其中：

- $v$：底盘前向速度；
- $\omega$：底盘 yaw 角速度；
- $\dot q_i$：第 $i$ 个机械臂关节角速度。

结构对应：

```text
WholeBodyInput
├── stamp
├── base_model
├── base_command
├── joint_names
└── joint_velocities
```

### 3.4 末端位姿与速度

末端位姿：

$$
{}^{w}T_{ee} = FK(x)
$$

末端空间速度：

$$
{}^{w}V_{ee} = J_{wb}(x) \, u
$$

其中 $J_{wb}(x)$ 是包含底盘运动和机械臂关节运动的全身雅可比。

第一版约定：

$$
{}^{w}V_{ee} =
\begin{bmatrix}
{}^{w}v_{ee} \\
{}^{w}\omega_{ee}
\end{bmatrix}
\in \mathbb{R}^{6}
$$

其中：

- 前三行 ${}^{w}v_{ee}$：末端参考点的线速度；
- 后三行 ${}^{w}\omega_{ee}$：末端坐标系的角速度；
- 参考点：`link_name` 坐标系原点，不是质心；
- 表达坐标系：`state.header.frame_id`；
- 行顺序固定为 `[linear; angular]`；
- 雅可比尺寸：$6 \times n_u$。

> 不允许混用 `[angular; linear]`、body-frame twist、质心速度或其他参考点。  
> 如果底层库使用不同约定，必须在 Adapter 中显式转换。

### 3.5 输入输出维度契约

对于当前差速底盘 + 六轴机械臂：

$$
n_x = 3 + 6 = 9
$$

$$
n_u = 2 + 6 = 8
$$

对应：

```text
WholeBodyState: 9D
WholeBodyInput: 8D
Task pose: 6D
Wrench: 6D
```

---

## 4. 运动学与轨迹定义

### 4.1 差速底盘运动学

连续时间差速底盘模型：

$$
\begin{aligned}
\dot{x}_b &= v \cos\psi_b \\
\dot{y}_b &= v \sin\psi_b \\
\dot{\psi}_b &= \omega \\
\dot{q} &= u_q
\end{aligned}
$$

非完整约束：

$$
-\sin\psi_b \, \dot{x}_b + \cos\psi_b \, \dot{y}_b = 0
$$

### 4.2 离散状态转移

离散模型：

$$
x_{k+1} = f_d(x_k, u_k, \Delta t_k)
$$

如果优化变量同时包含状态和输入，必须显式加入缺陷约束：

$$
x_{k+1} - f_d(x_k, u_k, \Delta t_k) = 0
$$

如果使用状态参数化，至少必须约束横向滑移。

定义：

$$
\Delta x_k = x_{k+1} - x_k
$$

$$
\Delta y_k = y_{k+1} - y_k
$$

$$
\Delta \psi_k = \operatorname{wrapToPi}(\psi_{k+1} - \psi_k)
$$

横向滑移残差：

$$
r_{\text{lat},k} =
-\sin\psi_k \, \Delta x_k
+ \cos\psi_k \, \Delta y_k
$$

要求：

$$
r_{\text{lat},k} = 0
$$

第一版非完整残差可写为：

$$
r_{\text{nh},k} =
\begin{bmatrix}
-\sin\psi_k \, \Delta x_k + \cos\psi_k \, \Delta y_k \\
\Delta \psi_k - \omega_k \Delta t_k \\
\Delta q_k - u_{q,k} \Delta t_k
\end{bmatrix}
= 0
$$

### 4.3 轨迹定义

全身轨迹定义为按时间排列的序列：

$$
\mathcal{X} =
\left\{
(t_i, x_i, u_i, \phi_i, \mathcal{T}_{\text{task},i})
\right\}_{i=0}^{N}
$$

其中：

- $t_i$：相对轨迹起点的时间；
- $x_i$：全身状态；
- $u_i$：可选前馈输入；
- $\phi_i$：执行相位；
- $\mathcal{T}_{\text{task},i}$：可选任务参考。

时间必须单调：

$$
0 \le t_0 < t_1 < \dots < t_N = T
$$

### 4.4 轨迹光滑性

第 $r$ 阶光滑性代价：

$$
J_{\text{smooth}} = \int_0^T \left\| x^{(r)}(t) \right\|_Q^2 \, dt
$$

常用：

- $r = 2$：加速度平滑；
- $r = 3$：jerk 平滑；
- $r = 4$：snap 平滑。

### 4.5 OCS2 参考轨迹

OCS2 参考状态：

$$
x_{\text{ref}} = [x_b, y_b, \psi_b, q_1, \dots, q_6]^T
$$

OCS2 参考输入：

$$
u_{\text{ref}} = [v, \omega, \dot q_1, \dots, \dot q_6]^T
$$

参考轨迹必须与 `SystemObservation` 使用同一时间轴。

### 4.6 SearchResult 约束

`SearchResult` 只提供拓扑和初值，不提供最终执行轨迹：

$$
\mathcal{S} =
\left\{
(base_i, q_i, \phi_i)
\right\}_{i=0}^{N}
$$

其中：

- $base_i$：第 $i$ 个底盘状态；
- $q_i$：第 $i$ 个机械臂关节种子；
- $\phi_i$：第 $i$ 个节点对应的执行阶段。

必须满足：

$$
\text{size}(base\_path) =
\text{size}(arm\_seed) =
\text{size}(phases)
$$

也就是说：

- `base_path[i]` 和 `arm_seed[i]` 必须是同一个全身节点；
- `arm_seed[i]` 是建立在 `base_path[i]` 基础上的机械臂构型；
- 三者长度不一致时，SearchResult 视为无效；
- `path_length` 和 `solve_time` 必须有限且非负；
- 每个 `base_path[i].yaw` 必须 wrap 到 $(-\pi, \pi]$；
- 每个 `arm_seed[i]` 必须使用完全相同、且与 `RobotModel::jointNames()` 一致的六关节名称和顺序。

---

## 5. 任务定义

### 5.1 任务轨迹

任务轨迹描述“末端要做什么”，不是机器人全身如何运动。

任务轨迹集合定义为：

$$
\mathcal{T}_{\text{task}} =
\left\{
(t_i, {}^{w}T_{ee,i}^{des}, \mathbf{n}_i, \mathbf{t}_i)
\right\}_{i=0}^{N}
$$

其中：

- $t_i$：任务点相对轨迹起点的时间；
- ${}^{w}T_{ee,i}^{des}$：期望末端位姿；
- $\mathbf{n}_i$：任务表面法向；
- $\mathbf{t}_i$：任务切向。

> `TaskTrajectory` 只表示单纯的任务轨迹：时间、期望末端位姿、表面法向和切向；当前版本不包含接触标志，也不包含执行相位。
> 第一版任务轨迹只给位置和姿态，不给末端速度。  
> 末端速度由后续规划/优化或执行层根据时间参数自行得到。  
> 力控不作为规划轨迹的一部分，而是在执行层对已经规划好的 `WholeBodyTrajectory` 生成修正量。

### 5.2 任务相位（用于后续轨迹评判）

`TaskTrajectory` 本身不携带相位。相位是在后续把导航轨迹和任务轨迹结合起来做统一评判时，用来标识长序列不同功能段的独立结构。

任务相位定义为：

$$
\phi \in \{
\text{IDLE},
\text{NAVIGATE},
\text{PRE\_EXECUTION},
\text{EXECUTION},
\text{TRACKING},
\text{FINISH},
\text{FAULT}
\}
$$

相位时间表定义为：

$$
\mathcal{P} =
\{(t_i^{\text{start}}, t_i^{\text{end}}, \phi_i, \text{task\_id}_i)\}
$$

相位不仅表示权重调度，还可以改变约束结构：

| 相位 | 含义 | 规划重点 | 执行重点 |
|---|---|---|---|
| `IDLE` | 空闲 | 不规划 | 不执行 |
| `NAVIGATE` | 导航到任务入口 | 底盘路径、避障 | 底盘跟踪 |
| `PRE_EXECUTION` | 预执行 | 末端对齐、接近准备 | 位姿对准 |
| `EXECUTION` | 任务执行 | 末端任务约束、全身协同 | 名义轨迹跟踪 |
| `TRACKING` | 纯轨迹跟踪 | 全身轨迹连续性 | OCS2 / 控制器跟踪 |
| `FINISH` | 结束 | 撤退、停止 | 安全收尾 |
| `FAULT` | 故障 | 不规划 | 进入安全状态 |

### 5.3 任务误差

SE(3) 任务误差定义为：

$$
e_{\text{task}}(t) =
\log\left(
{}^{w}T_{ee}^{des}(t)^{-1}
\, {}^{w}T_{ee}(x(t))
\right)^{\vee}
\in \mathbb{R}^{6}
$$

任务代价：

$$
J_{\text{task}} =
\int_0^T
e_{\text{task}}(t)^T
Q_{\text{task}}
e_{\text{task}}(t)
\, dt
$$

### 5.4 任务入口区域

任务入口不是一个固定点，而是一个集合：

$$
\mathcal{G}_{\text{task}} =
\left\{
x_b \mid
\exists q:
\text{IK}(T_{\text{task}}^{\text{entry}}, x_b, q) \ \text{feasible},
m(q) > m_{\min},
d(x) > d_{\text{safe}}
\right\}
$$

其中：

- $m(q)$：操作度或可操作度指标；
- $d(x)$：全身最小安全距离；
- $T_{\text{task}}^{\text{entry}}$：任务起始末端位姿。

搜索目标可以写为：

$$
\min_{x_b} \; J_{\text{navigate}}(x_b)
\quad \text{s.t.} \quad
x_b \in \mathcal{G}_{\text{task}}
$$

---

## 6. 力控定义

> 力控不参与 `TaskTrajectory` 和 `WholeBodyTrajectory` 的名义轨迹生成。  
> 力控属于执行层：它接收已经规划好的名义轨迹，并根据接触力误差生成位置/速度修正量。

### 6.1 力/力矩向量

六维力/力矩 wrench 定义为：

$$
\mathcal{F} =
\begin{bmatrix}
f \\
\tau
\end{bmatrix}
\in \mathbb{R}^{6}
$$

其中：

- $f = [f_x, f_y, f_z]^T$：力；
- $\tau = [\tau_x, \tau_y, \tau_z]^T$：力矩。

### 6.2 测量力与期望力

测量力：

$$
\mathcal{F}_{\text{meas}}
$$

期望力：

$$
\mathcal{F}_{\text{des}}
$$

力误差：

$$
e_{\mathcal{F}} = \mathcal{F}_{\text{des}} - \mathcal{F}_{\text{meas}}
$$

### 6.3 法向力

任务法向力为：

$$
f_n = \mathbf{n}^T \, {}^{\text{task}}f_{\text{meas}}
$$

其中 $\mathbf{n}$ 是任务表面法向。

### 6.4 导纳控制

导纳控制将力误差映射为位置修正：

$$
M_d \Delta \ddot{p}
+ D_d \Delta \dot{p}
+ K_d \Delta p
= e_{\mathcal{F}}
$$

其中：

- $M_d$：期望惯性矩阵；
- $D_d$：期望阻尼矩阵；
- $K_d$：期望刚度矩阵；
- $\Delta p$：位置修正量。

离散实现：

$$
\Delta p_{k+1} =
\Delta p_k
+ \Delta \dot{p}_k \, \Delta t
+ \frac{1}{2} \Delta \ddot{p}_k \, \Delta t^2
$$

### 6.5 阻抗控制

笛卡尔阻抗控制可写为：

$$
\mathcal{F}_{\text{cmd}} =
K_p(p_d - p)
+ K_d(v_d - v)
+ \mathcal{F}_{\text{ff}}
$$

其中：

- $p_d$：期望位置；
- $v_d$：期望速度；
- $\mathcal{F}_{\text{ff}}$：前馈力/力矩；
- $K_p$：位置刚度；
- $K_d$：速度阻尼。

### 6.6 力/位混合控制

使用轴选择矩阵：

$$
\Lambda = \operatorname{diag}(\lambda_1, \lambda_2, \lambda_3, \lambda_4, \lambda_5, \lambda_6)
$$

其中：

$$
\lambda_i = 1
$$

表示该轴执行力控；

$$
\lambda_i = 0
$$

表示该轴执行位置/速度控制。

### 6.7 力安全约束

力安全约束：

$$
\|f_{\text{meas}}\| \le f_{\max}
$$

$$
\|\tau_{\text{meas}}\| \le \tau_{\max}
$$

接触丢失判断：

$$
\|f_{\text{meas}}\| < f_{\text{contact,min}}
$$

超过安全阈值时必须进入：

```text
SAFE_HOLD
```

或：

```text
FAULT
```

---

## 7. 数据契约总表

| 名称 | 维度 | 含义 | 内部类型 |
|---|---:|---|---|
| $x$ | 9 | 全身状态 | `WholeBodyState` |
| $u$ | 8 | 全身输入 | `WholeBodyInput` |
| ${}^{w}T_{ee}$ | 6 | 末端位姿 | `Pose` |
| ${}^{w}V_{ee}$ | 6 | 末端空间速度 | `Twist` |
| $\mathcal{F}$ | 6 | 力/力矩 | `Wrench` |
| $\mathcal{T}_{\text{task}}$ | variable | 纯任务轨迹参考序列 | `TaskTrajectory` |
| $\mathcal{X}$ | variable | 全身名义轨迹 | `WholeBodyTrajectory` |
| $\mathcal{P}$ | variable | 后续轨迹评判用的相位时间表 | `PhaseSchedule` |
| $x_{\text{ref}}$ | 9 | OCS2 参考状态 | `MpcTargetTrajectories` |
| $u_{\text{ref}}$ | 8 | OCS2 参考输入 | `MpcTargetTrajectories` |

---

## 8. 校验规则

所有输入输出必须满足：

1. 所有数值有限，不允许 `NaN` 或 `Inf`；
2. 四元数必须归一化；
3. yaw 必须 wrap 到 $(-\pi, \pi]$；
4. 轨迹时间必须严格递增；
5. 关节名称必须显式给出并按名称映射；
6. 不允许静默截断、补零或猜测关节顺序；
7. frame_id 不允许为空；
8. 环境和碰撞模型的 revision 必须一致；
9. 跨坐标系速度必须做正确变换；
10. 力控模式必须有明确的参考坐标系和作用轴。

统一校验入口位于：

```text
src/core/wbmm_core/include/wbmm_core/validation.hpp
```

调用方必须 fail-closed：只要统一校验返回失败，就不得继续规划、控制或发送命令。

`RobotModel::validate()` 只负责模型相关的语义校验，例如关节限位、frame 是否存在等；通用结构校验必须先由 `wbmm_core/validation.hpp` 完成。

---

## 9. 与代码的对应关系

| 契约概念 | 代码位置 |
|---|---|
| 全身状态、输入 | `src/core/wbmm_core/include/wbmm_core/types.hpp` |
| 基础校验 | `src/core/wbmm_core/include/wbmm_core/validation.hpp` |
| 机器人模型接口 | `src/core/wbmm_core/include/wbmm_core/robot_model.hpp` |
| 可视化 9D 契约 | `src/visual/include/wbmm_visualization/contract.hpp` |
| OCS2 9D/8D 模型合同 | `src/control/wbmm_ocs2/include/wbmm_ocs2/WbmmModelInfo.h`、`src/control/wbmm_ocs2/src/FactoryFunctions.cpp` |
| OCS2 差速动力学 | `src/control/wbmm_ocs2/include/wbmm_ocs2/Dynamics.h`、`src/control/wbmm_ocs2/src/Dynamics.cpp` |
| OCS2 状态/输入与 Pinocchio 映射 | `src/control/wbmm_ocs2/include/wbmm_ocs2/PinocchioMapping.h`、`src/control/wbmm_ocs2/src/PinocchioMapping.cpp` |
| OCS2 问题装配 | `src/control/wbmm_ocs2/include/wbmm_ocs2/WbmmInterface.h`、`src/control/wbmm_ocs2/src/WbmmInterface.cpp` |
| OCS2 全身参考代价 | `src/control/wbmm_ocs2/include/wbmm_ocs2/cost/WholeBodyTrajectoryCost.h`、`src/control/wbmm_ocs2/src/cost/WholeBodyTrajectoryCost.cpp` |
| OCS2 ROS 求解与执行适配 | `src/control/wbmm_ocs2_ros/src/WbmmMpcNode.cpp`、`src/control/wbmm_ocs2_ros/src/WbmmMrtNode.cpp` |
| 任务轨迹与相位数据结构 | `src/core/wbmm_core/include/wbmm_core/trajectory.hpp` |
| REMANI 算法 | `src/vendor/remani_planner` |
| OCS2 算法 | `src/vendor/ocs2_ros2` |

---

## 10. 总结

WBMM 的核心数学主干是：

$$
\text{TaskTrajectory}
\rightarrow
\text{SearchResult}
\rightarrow
\text{WholeBodyTrajectory}
\rightarrow
\text{OCS2 Reference}
\rightarrow
\text{WholeBodyCommand}
$$

其中：

- `TaskTrajectory` 描述末端要做什么；
- `SearchResult` 提供拓扑和初值；
- `WholeBodyTrajectory` 是规划完成后唯一的机器人名义运动轨迹；
- `OCS2 Reference` 是 OCS2 可跟踪的滚动参考；
- `WholeBodyCommand` 是最终发给机器人/仿真器的控制命令；
- 执行相位不在 `TaskTrajectory` 内，而是在后续把导航轨迹和任务轨迹合并评判时使用。

所有后续代码、实验和可视化都必须以本文档定义的坐标系、维度、单位和语义为准。
