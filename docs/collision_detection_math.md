# 碰撞检测数学原理 — mm_config.cpp

## 1. 总体架构：球体代理模型 (Sphere Proxy Model)

碰撞检测将移动操作机器人（底盘 + 机械臂）**近似为一组球体集合**。每个球体由一个球心点和一个半径定义。碰撞检测转换为：判断球体集合与障碍物（ESDF 地图）之间、或球体集合之间是否发生干涉。

系统实现了 **4 层碰撞检测**：

| 层级 | 函数 | 检查对象 |
|------|------|----------|
| Layer 1 | `checkCarObsCollision` | 底盘 ↔ 环境障碍物 |
| Layer 2 | `checkManiObsCollision` | 机械臂 ↔ 环境障碍物 |
| Layer 3 | `checkCarManiCollision` | 底盘 ↔ 机械臂（自碰撞） |
| Layer 4 | `checkManiManiCollision` | 机械臂 ↔ 机械臂（自碰撞） |

---

## 2. 正运动学链 (Forward Kinematics Chain)

机械臂连杆上每个碰撞球心 $\mathbf{p}_j^{(i)}$ 在关节 $i$ 的局部坐标系下定义（齐次坐标 $4 \times 1$）。

通过正运动学递推，计算其在世界坐标系下的位置：

$$
T_{\text{world}} = T_{\text{car}} \cdot T_{q_0} \cdot \prod_{k=0}^{i} T_k(\theta_k)
$$

其中：

- $T_{\text{car}}$ 为底盘位姿的齐次变换矩阵：
  $$
  T_{\text{car}} = \begin{bmatrix}
  \cos\psi & -\sin\psi & 0 & x \\
  \sin\psi & \cos\psi  & 0 & y \\
  0        & 0         & 1 & 0 \\
  0        & 0         & 0 & 1
  \end{bmatrix}
  $$

- $T_{q_0}$ 为机械臂基座相对于底盘中心的安装偏移（常量）
- $T_k(\theta_k)$ 为关节 $k$ 的齐次变换矩阵，由 `getAJointTran(k, θ_k)` 给定

连杆 $i$ 上第 $j$ 个碰撞球心在世界坐标系中的位置：

$$
\mathbf{c}_j^{(i)} = \left( T_{\text{world}} \cdot \mathbf{p}_j^{(i)} \right)_{\text{head}(3)}
$$

其中 $\mathbf{p}_j^{(i)} \in \mathbb{R}^4$ 为齐次坐标 $[p_x, p_y, p_z, 1]^T$，$\text{head}(3)$ 取前三个分量。

---

## 3. 底盘碰撞球心生成

底盘近似为一个长方体（$L \times W \times H$），用半径为 $r_{\text{car}}$ 的球体覆盖。

### 3.1 XY 平面分布

长方体在 XY 平面的截面为矩形。碰撞球心分布在：

- **4 个倒角角点**：位于矩形四角，向内侧偏移以形成圆角效果
- **边长插值点**：沿四条边以间距 $\approx r_{\text{car}}$ 均匀分布

### 3.2 Z 方向分层

碰撞球沿 Z 轴分层堆放，层数约为 $H / r_{\text{car}}$。最终世界坐标下的球心为：

$$
\mathbf{c}_{\text{car},k} = \begin{bmatrix} x \\ y \\ 0 \end{bmatrix} + R_z(\psi) \cdot \mathbf{p}_{\text{car},k}^{\text{local}}
$$

其中 $\mathbf{p}_{\text{car},k}^{\text{local}}$ 为底盘局部坐标系下的球心位置，$R_z(\psi)$ 为 yaw 旋转矩阵。

URDF 模式下，球心直接从 URDF 的 `<collision>` 元素中提取，半径也由 URDF 定义。

---

## 4. 障碍物碰撞检测：ESDF 查询

环境障碍物由 **ESDF（Euclidean Signed Distance Field，欧几里得有向距离场）** 表示，存储在 `GridMap` 中。

### 4.1 有向距离场

对于空间中的任意点 $\mathbf{p} \in \mathbb{R}^3$，ESDF 返回该点到最近障碍物表面的**有向距离**：

$$
d(\mathbf{p}) = \begin{cases}
\text{距离最近障碍物的欧氏距离} & \mathbf{p} \text{ 在自由空间中} \\
-(\text{距离最近穿透深度})      & \mathbf{p} \text{ 在障碍物内部}
\end{cases}
$$

### 4.2 碰撞判定

对于球心 $\mathbf{c}$ 和半径 $r$，到障碍物表面的距离为：

$$
d_{\text{surf}} = d_{\text{ESDF}}(\mathbf{c}) - r - d_{\text{safe}} - d_{\text{res}}
$$

其中：
- $d_{\text{ESDF}}(\mathbf{c})$ 通过三线性插值 (`getPreciseDistance`) 或最近格点查询 (`getDistance`) 获得
- $d_{\text{safe}}$ 为安全裕度（`car_safe_margin_` 或 `mani_safe_margin_`）
- $d_{\text{res}}$ 为 ESDF 地图分辨率，补偿离散化误差

**碰撞条件**：

$$
d_{\text{surf}} < 0 \quad \Longrightarrow \quad \text{碰撞!}
$$

### 4.3 地面穿透检测（仅机械臂）

额外判断球心 Z 坐标是否低于地面安全高度：

$$
c_z^{(i,j)} < h_{\text{ground\_safe}} \quad \Longrightarrow \quad \text{碰撞（穿透地面）!}
$$

---

## 5. 自碰撞检测

### 5.1 底盘-机械臂自碰撞 (`checkCarManiCollision`)

在底盘局部坐标系中（设底盘 $(x,y,\psi) = (0,0,0)$）：

$$
\text{for } i = 1 \dots N-1,\; \forall (a, b):
$$

$$
\|\mathbf{c}_{\text{car},a} - \mathbf{c}_{\text{arm},b}^{(i)}\|_2 \;<\; r_{\text{car},a} + r_{\text{arm},b} + d_{\text{self\_safe}}
$$

跳过 link 0（基座关节，与底盘邻接，不检查）。

### 5.2 机械臂自碰撞 (`checkManiManiCollision`)

沿正运动学链累积球心。对每个新连杆 $i$，检查与之前所有连杆 $k \leq i-2$ 的球心距离（跳过直接相邻连杆 $i-1$）：

$$
\text{for } i = 0 \dots N-1,\; k = 0 \dots i-2,\; \forall (j, m):
$$

$$
\|\mathbf{c}_j^{(i)} - \mathbf{c}_m^{(k)}\|_2 \;<\; r_j^{(i)} + r_m^{(k)} + d_{\text{self\_safe}}
$$

跳过相邻连杆 $i-1$ 的原因是：相邻连杆在关节处始终邻接，它们的碰撞球必然相交，无需检测。

---

## 6. 连杆碰撞球心配置 (setLinkPoint)

碰撞球心点集 $\{ \mathbf{p}_j^{(i)} \}$ 在 `setLinkPoint()` 中硬编码定义。设计原则：

- 沿连杆**轴向**和**径向**分布球心
- 球心间距 $\approx$ 连杆直径 / 2，保证无死角覆盖
- 短连杆（如 J2、J4）仅用 1 个原点球心

**FastArmer 配置**：

| 关节 | 球数 | 分布描述 |
|------|------|----------|
| J0   | 4    | 基座垂直轴：中心 + ±Z + -Y |
| J1   | 6    | 大臂沿 X 轴 0→0.35m 线性分布 |
| J2   | 1    | 原点（短连杆） |
| J3   | 5    | 前臂沿 Y 轴 0.07→0.35m 线性分布 |
| J4   | 1    | 原点（短连杆） |
| J5   | 7    | 手腕 + 夹爪 3D 分布 |

---

## 7. 碰撞检测在轨迹优化中的应用

碰撞检测函数被 `PolyTrajOptimizer::obstacleGradCostforMM()` 调用，用于计算轨迹碰撞代价及其梯度。

底盘球心相对于 yaw 和速度的雅可比由 `getCarPtsGrad()` / `getCarPtsGradNew()` 提供，使优化器能够对轨迹进行梯度下降以避开障碍物。

---

## 8. 参数汇总

| 参数 | 含义 | 默认来源 |
|------|------|----------|
| `car_safe_margin_` | 底盘-障碍物安全裕度 | `optimization.safe_margin_car` |
| `mani_safe_margin_` | 机械臂-障碍物安全裕度 | `optimization.safe_margin_mani` |
| `self_safe_margin_` | 自碰撞安全裕度 | `optimization.safe_margin_self` |
| `ground_safe_dis_` | 地面安全高度阈值 | `optimization.ground_safe_dis` |
| `mobile_base_check_radius_` | 底盘碰撞球半径 | 由底盘尺寸计算 |
| `manipulator_thickness_` | 机械臂碰撞球半径（非URDF模式） | `optimization.manipulator_thickness` |
| `manipulator_link_radii_` | 各球体独立半径（URDF模式） | 从URDF提取 |
