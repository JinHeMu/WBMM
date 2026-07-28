# 轨迹优化数学原理 — poly_traj_optimizer.cpp

## 1. 总体架构

`PolyTrajOptimizer` 是 REMANI 全局规划器的**后端轨迹优化引擎**，核心流程：

```
前端 A* 搜索 → MINCO 初值 → L-BFGS 优化 → 最优多项式轨迹 → 碰撞+可行性验证
```

优化变量通过 MINCO (Minimum Control) 参数化，代价函数包含平滑性、避障、可行性、时间等项，梯度通过解析链式法则计算。

---

## 2. 轨迹参数化：MINCO (Minimum Control Polynomial)

### 2.1 多项式表示

轨迹按段 (piece) 组织，每段用 8 阶多项式表示（最高次项为 $s^7$），轨迹维度为：

$$
\text{traj\_dim} = \text{mobile\_base\_dof} + \text{manipulator\_dof}
$$

一段多项式的位置表达式：

$$
\mathbf{p}(s) = \mathbf{c}^T \boldsymbol{\beta}(s), \quad s \in [0, 1]
$$

其中 $\mathbf{c} \in \mathbb{R}^{8 \times \text{traj\_dim}}$ 是多项式系数矩阵，基函数向量：

$$
\boldsymbol{\beta}(s) = \begin{bmatrix} 1 & s & s^2 & s^3 & s^4 & s^5 & s^6 & s^7 \end{bmatrix}^T
$$

速度、加速度、jerk、snap 通过对基函数求导获得：

$$
\begin{aligned}
\boldsymbol{\beta}_1(s) &= \begin{bmatrix} 0 & 1 & 2s & 3s^2 & 4s^3 & 5s^4 & 6s^5 & 7s^6 \end{bmatrix}^T \quad &\text{(速度)} \\
\boldsymbol{\beta}_2(s) &= \begin{bmatrix} 0 & 0 & 2 & 6s & 12s^2 & 20s^3 & 30s^4 & 42s^5 \end{bmatrix}^T \quad &\text{(加速度)} \\
\boldsymbol{\beta}_3(s) &= \begin{bmatrix} 0 & 0 & 0 & 6 & 24s & 60s^2 & 120s^3 & 210s^4 \end{bmatrix}^T \quad &\text{(jerk)} \\
\boldsymbol{\beta}_4(s) &= \begin{bmatrix} 0 & 0 & 0 & 0 & 24 & 120s & 360s^2 & 840s^3 \end{bmatrix}^T \quad &\text{(snap)}
\end{aligned}
$$

### 2.2 优化变量

L-BFGS 优化变量 $x$ 按以下顺序拼接：

| 块 | 内容 | 维度 |
|----|------|------|
| $[1]$ | 各段中间点 $\mathbf{P}^{(i)}$ | $\text{traj\_dim} \times (\text{piece\_num}_i - 1)$ |
| $[2]$ | 各段虚拟时间 $\boldsymbol{\tau}^{(i)}$ | $\text{piece\_num}_i$ |
| $[3]$ | 换向点位置 (Gear) | $\text{traj\_dim} \times (\text{traj\_num} - 1)$ |
| $[4]$ | 换向点角度 (Angle) | $1 \times (\text{traj\_num} - 1)$ |

总变量数：

$$
N_{\text{var}} = \sum_i \big[ \text{traj\_dim} \cdot (N_i - 1) + N_i \big] + (2 \cdot \text{traj\_dim} + 1) \cdot (N_{\text{traj}} - 1)
$$

其中 $N_i$ 为第 $i$ 段的段数，$N_{\text{traj}}$ 为总轨迹段数（按前进/后退方向分割）。

---

## 3. 虚拟时间映射

真实时间 $T$ 必须严格为正 ($T > 0$)，但 L-BFGS 是无约束优化。通过虚拟时间映射将约束优化转化为无约束优化：

### 3.1 正向映射：$T \to \tau$

$$
\tau(T) = \begin{cases}
\sqrt{2T - 1} - 1 & T > 1 \\
1 - \sqrt{\frac{2}{T} - 1} & T \leq 1
\end{cases}
$$

### 3.2 逆向映射：$\tau \to T$

$$
T(\tau) = \begin{cases}
\frac{1}{2}\tau^2 + \tau + 1 & \tau > 0 \\
\frac{1}{\frac{1}{2}\tau^2 - \tau + 1} & \tau \leq 0
\end{cases}
$$

### 3.3 梯度传播

$$
\frac{dT}{d\tau} = \begin{cases}
\tau + 1 & \tau > 0 \\
\frac{1 - \tau}{(0.5\tau^2 - \tau + 1)^2} & \tau \leq 0
\end{cases}
$$

时间代价和梯度：

$$
J_{\text{time}} = w_{\text{time}} \sum_i T_i
$$

$$
\frac{\partial J}{\partial \tau_i} = \left( \frac{\partial J}{\partial T_i} + w_{\text{time}} \right) \cdot \frac{dT_i}{d\tau_i}
$$

---

## 4. 代价函数总览

总代价：

$$
J = \underbrace{J_{\text{smooth}}}_{\text{平滑性}} + \underbrace{J_{\text{obs}} + J_{\text{feas}}}_{\text{避障 + 可行性}} + \underbrace{J_{\text{time}}}_{\text{时间}}
$$

具体展开：

$$
\begin{aligned}
J = &\quad w_{\text{snap}} \cdot \int \|\ddot{\mathbf{p}}\|^2 \, dt \quad \text{(min-snap)} \\
   &+ J_{\text{obs\_car}} + J_{\text{obs\_mani}} + J_{\text{self}} \quad \text{(碰撞)} \\
   &+ J_{\text{feas\_car}} + J_{\text{feas\_joint}} \quad \text{(可行性)} \\
   &+ J_{\text{time}} \quad \text{(时间)}
\end{aligned}
$$

---

## 5. 平滑性代价 (Min-Snap)

Min-snap 代价最小化 snap（加加加速度）的平方积分：

$$
J_{\text{smooth}} = \int_0^{T_f} \left\|\frac{d^4\mathbf{p}}{dt^4}\right\|^2 dt
$$

由 MINCO 的 `initGradCost` 通过闭式解析解计算代价和梯度（利用多项式系数可解析积分）。

---

## 6. 避障代价

### 6.1 ESDF 距离惩罚

对每个碰撞球中心 $\mathbf{p}$，查询 ESDF 获得最近距离 $d(\mathbf{p})$，定义：

$$
\delta_{\text{err}} = r_{\text{sphere}} + d_{\text{safe}} - d(\mathbf{p})
$$

惩罚函数使用 **三次幂惩罚**（max(0, x)³）：

$$
J_{\text{obs}} = w_{\text{obs}} \cdot \max(0, \delta_{\text{err}})^3
$$

梯度：

$$
\frac{\partial J_{\text{obs}}}{\partial \mathbf{p}} = -3 w_{\text{obs}} \cdot \max(0, \delta_{\text{err}})^2 \cdot \nabla d(\mathbf{p})
$$

### 6.2 地面碰撞

对每个机械臂碰撞球，检查 Z 坐标：

$$
\delta_{\text{ground}} = r_{\text{sphere}} + h_{\text{safe}} - p_z
$$

使用 **smoothed L1** 惩罚（而非三次幂），使梯度连续可导。

---

## 7. 底盘可行性代价

### 7.1 差速轮运动学

Tracer 底盘采用差速驱动。关键运动学关系：

**航向角速度**：

$$
\omega = \frac{\mathbf{a}^T \mathbf{B} \mathbf{v}}{\mathbf{v}^T \mathbf{v}}, \quad \mathbf{B} = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}
$$

**左右轮转速**（$s = \pm 1$ 为前进/后退标志，$b$ 为轮距，$r$ 为轮半径）：

$$
\begin{aligned}
\omega_L &= \frac{2s\|\mathbf{v}\| - b\omega}{2r} \\
\omega_R &= \frac{2s\|\mathbf{v}\| + b\omega}{2r}
\end{aligned}
$$

**航向角加速度**：

$$
\alpha = \frac{\mathbf{j}^T \mathbf{B} \mathbf{v}}{\mathbf{v}^T \mathbf{v}} - \frac{2(\mathbf{a}^T \mathbf{B} \mathbf{v})(\mathbf{a}^T \mathbf{v})}{(\mathbf{v}^T \mathbf{v})^2}
$$

**左右轮角加速度**：

$$
\begin{aligned}
\alpha_L &= \frac{2s \frac{\mathbf{a}^T\mathbf{v}}{\|\mathbf{v}\|} - b\alpha}{2r} \\
\alpha_R &= \frac{2s \frac{\mathbf{a}^T\mathbf{v}}{\|\mathbf{v}\|} + b\alpha}{2r}
\end{aligned}
$$

### 7.2 约束惩罚

对以下约束使用 smoothed L1 惩罚：

| 约束 | 表达式 |
|------|--------|
| 最大线速度 | $\|\mathbf{v}\|^2 \leq v_{\max}^2$ |
| 最小线速度 (密集采样) | $\|\mathbf{v}\|^2 \geq v_{\min}^2$ |
| 左轮最大转速 | $\omega_L^2 \leq \omega_{\text{wheel,max}}^2$ |
| 右轮最大转速 | $\omega_R^2 \leq \omega_{\text{wheel,max}}^2$ |
| 左轮最大角加速度 | $\alpha_L^2 \leq \alpha_{\text{wheel,max}}^2$ |
| 右轮最大角加速度 | $\alpha_R^2 \leq \alpha_{\text{wheel,max}}^2$ |

### 7.3 解析梯度示例

以 $\omega$ 为例，其对 $\mathbf{v}$ 和 $\mathbf{a}$ 的梯度：

$$
\frac{\partial \omega}{\partial \mathbf{v}} = \frac{\mathbf{B}^T\mathbf{a}}{\mathbf{v}^T\mathbf{v}} - \frac{(\mathbf{a}^T\mathbf{B}\mathbf{v})\mathbf{v} + \mathbf{v}^T\mathbf{B}^T\mathbf{a}\mathbf{v}}{(\mathbf{v}^T\mathbf{v})^2}
$$

$$
\frac{\partial \omega}{\partial \mathbf{a}} = \frac{\mathbf{B}\mathbf{v}}{\mathbf{v}^T\mathbf{v}}
$$

$\alpha$ 对 $\mathbf{v}, \mathbf{a}, \mathbf{j}$ 的梯度：

$$
\begin{aligned}
\frac{\partial \alpha}{\partial \mathbf{v}} &= \frac{\mathbf{B}^T\mathbf{j} \cdot \mathbf{v}^T\mathbf{v} - 2 (\mathbf{j}^T\mathbf{B}\mathbf{v}) \mathbf{v}}{(\mathbf{v}^T\mathbf{v})^2} - \frac{2(\mathbf{a}^T\mathbf{B}\mathbf{v})\mathbf{a} + 2(\mathbf{a}^T\mathbf{v})\mathbf{B}^T\mathbf{a}}{(\mathbf{v}^T\mathbf{v})^2} + \frac{8(\mathbf{a}^T\mathbf{B}\mathbf{v})(\mathbf{a}^T\mathbf{v})\mathbf{v}}{(\mathbf{v}^T\mathbf{v})^3} \\
\frac{\partial \alpha}{\partial \mathbf{a}} &= -\frac{2}{(\mathbf{v}^T\mathbf{v})^2} \left[ (\mathbf{a}^T\mathbf{v})\mathbf{B}\mathbf{v} + (\mathbf{a}^T\mathbf{B}\mathbf{v})\mathbf{v} \right] \\
\frac{\partial \alpha}{\partial \mathbf{j}} &= \frac{\mathbf{B}\mathbf{v}}{\mathbf{v}^T\mathbf{v}}
\end{aligned}
$$

---

## 8. 关节可行性代价

对每个关节 $i$：

**位置限位**：

$$
\begin{aligned}
\delta_{\text{pos\_max}} &= q_i - q_i^{\max} \\
\delta_{\text{pos\_min}} &= q_i^{\min} - q_i
\end{aligned}
$$

**速度限位**：

$$
\delta_{\text{vel}} = \dot{q}_i^2 - (\dot{q}_i^{\max})^2
$$

**加速度限位**：

$$
\delta_{\text{acc}} = \ddot{q}_i^2 - (\ddot{q}_i^{\max})^2
$$

全部使用 smoothed L1 惩罚。

---

## 9. 平滑惩罚函数

### 9.1 Smoothed L1

替代不可导的 $\max(0, x)$，定义为：

$$
f(x; \mu) = \begin{cases}
0 & x \leq 0 \\
(3\mu - 0.5x) \cdot \frac{x^3}{\mu^3} & 0 < x \leq \mu \\
x - \frac{\mu}{2} & x > \mu
\end{cases}
$$

及其导数：

$$
f'(x; \mu) = \begin{cases}
0 & x \leq 0 \\
3 \cdot \frac{x^2}{\mu^2} \cdot \frac{\mu - 0.5x}{\mu + x} & 0 < x \leq \mu \\
1 & x > \mu
\end{cases}
$$

$\mu$ 为平滑过渡区宽度（典型值 0.005）。

### 9.2 Smoothed Max³

$$
f(x) = \begin{cases}
0 & x \leq 0 \\
x^3 & x > 0
\end{cases}, \quad f'(x) = \begin{cases}
0 & x \leq 0 \\
3x^2 & x > 0
\end{cases}
$$

### 9.3 Smoothed Log (平滑开关函数)

在 $[-\mu, \mu]$ 区间上从 0 平滑过渡到 1，用于密集采样的激活开关：

$$
\text{smoothlog}(x; \mu) \approx \mathbf{1}_{\{x > 0\}}
$$

---

## 10. 积分方案：Simpson 法则

沿归一化时间 $s \in [0, 1]$ 等距采样 $K+1$ 个点，使用 Simpson 权重：

$$
\omega_j = \begin{cases}
0.5 & j = 0 \text{ 或 } j = K \quad \text{(端点权重减半)} \\
1.0 & \text{否则}
\end{cases}
$$

采样步长：$\Delta s = 1 / K$，实际时间步长：$\Delta t = T_i \cdot \Delta s$

积分近似为：

$$
\int_0^{T_i} f\big(\mathbf{p}(t), \dot{\mathbf{p}}(t), \dots\big) dt \approx T_i \sum_{j=0}^K \omega_j \cdot f\big(\mathbf{p}(s_j), \dot{\mathbf{p}}(s_j), \dots\big) \cdot \Delta s
$$

---

## 11. 梯度传播：链式法则

### 11.1 最优控制中的伴随方法

代价函数对多项式系数 $\mathbf{c}$ 和段时间 $T$ 的梯度通过 MINCO 的伴随方法计算。

代价对多项式系数的梯度由各采样点的梯度累积：

$$
\frac{\partial J}{\partial \mathbf{c}} = \sum_{j} \omega_j \cdot \Delta t \cdot \boldsymbol{\beta}(s_j) \cdot \left(\frac{\partial J}{\partial \mathbf{p}}\right)^T
$$

代价对段时间的梯度：

$$
\frac{\partial J}{\partial T_i} = \sum_{j} \omega_j \cdot \left[ \frac{J_{\text{sample}}}{K} + \Delta t \cdot \alpha \cdot \left(\frac{\partial J}{\partial \mathbf{p}}\right)^T \dot{\mathbf{p}} \right]
$$

其中 $\alpha = j/K$ 为归一化时间的位置因子。

### 11.2 MINCO 伴随：$\nabla_{c,T} J \to \nabla_{P,\tau,\text{ini},\text{fin}} J$

`SnapOpt::getGrad2TP` 将多项式系数梯度传播回：

- 中间点 $\mathbf{P}$ 的梯度
- 真实时间 $\mathbf{T}$ 的梯度
- 起始/终止状态梯度

### 11.3 换向点梯度

起始/终止状态的梯度进一步传播到换向点变量：

对起始状态（非首段，速度 = $-v \cdot [\cos\theta, \sin\theta]^T$）：

$$
\frac{\partial J}{\partial \mathbf{p}_{\text{gear}}} += \frac{\partial J}{\partial \mathbf{p}_{\text{ini}}}, \quad \frac{\partial J}{\partial \theta} += \frac{\partial J}{\partial \dot{\mathbf{p}}_{\text{ini}}}^T \begin{bmatrix} v\sin\theta \\ -v\cos\theta \end{bmatrix}
$$

对终止状态（非尾段，速度 = $+v \cdot [\cos\theta, \sin\theta]^T$）：

$$
\frac{\partial J}{\partial \mathbf{p}_{\text{gear}}} += \frac{\partial J}{\partial \mathbf{p}_{\text{fin}}}, \quad \frac{\partial J}{\partial \theta} += \frac{\partial J}{\partial \dot{\mathbf{p}}_{\text{fin}}}^T \begin{bmatrix} -v\sin\theta \\ v\cos\theta \end{bmatrix}
$$

---

## 12. 密集采样策略

在低速度区（$\|\mathbf{v}\|^2 < v_{\min}^2$），在每个主采样区间内额外进行子采样（分辨率由 `dense_sample_resolution_` 控制），通过 `smoothedLog` 平滑激活额外的可行性约束惩罚。

子采样在 $[s - 0.5\Delta s, s + 0.5\Delta s]$ 区间内均匀插值，子步长为：

$$
\Delta s_{\text{dense}} = \frac{\Delta s}{\text{dense\_resolution}}
$$

密集采样的惩罚权重乘以平滑开关函数 `smoothedLog(pen_min_vel)`，在速度降低时逐步激活，实现平滑过渡。

---

## 13. 优化流程总结

```
OptimizeTrajectory_lbfgs:
  ├─ Step 1: 解析段数、总变量维度
  ├─ Step 2: 组装初始变量 x = [P, T, Gear, Angles]
  │    └─ 虚拟时间映射 T → τ
  ├─ Step 3: 配置 L-BFGS (mem=256, max_iter=1000)
  ├─ Step 4: 执行 L-BFGS 优化
  │    └─ costFunctionCallback 每次迭代:
  │         ├─ 解析 x → [P, τ, Gear, Angles]
  │         ├─ τ → T (虚拟时间逆映射)
  │         ├─ 对每段:
  │         │   ├─ MINCO.generate(P, T) → 多项式系数
  │         │   ├─ J_smooth + ∇  (min-snap)
  │         │   └─ addPVAJGradCost2CT:
  │         │       ├─ 主采样：障碍物 + 底盘可行性 + 关节可行性
  │         │       └─ 密集采样：低速度区额外可行性
  │         ├─ 链式法则: ∇_c → ∇_P, ∇_T
  │         ├─ 传播到 Gear/Angle 变量
  │         └─ 虚拟时间梯度 + 时间代价
  ├─ Step 5: 结果检查 (收敛/提前退出)
  ├─ Step 6: 整条轨迹安全验证 (密集采样 + 碰撞 + 可行性)
  └─ Step 7: 输出优化后的控制点、路径点、时间分配
```
