# 移动机械臂接触修正中的全身运动学分配与奇异性规避

> **技术报告 / 小论文草稿**
> 主题：Whole-Body Mobile Manipulation（WBMM）中的末端在线修正、底盘—机械臂协同与奇异性规避
> 状态：当前方案总结 + 后续改进设计
> 说明：本文明确区分“**当前已实现方案**”与“**未来拟实现方案**”，未实现内容均以“拟 / 建议 / 可进一步”表述。

---

## 摘要

在移动机械臂执行擦拭、插入、推压等接触任务时，末端通常需要根据力传感器、导纳控制器或其他在线反馈产生小范围位姿修正。对于固定机械臂，这类修正通常可以通过局部逆运动学直接映射为关节增量；但对于移动机械臂，底盘与机械臂之间存在冗余自由度，因此需要进一步解决“**末端修正应该由底盘承担多少、由机械臂承担多少**”的问题。

当前实现采用一种较为直接的分配策略：对于差速底盘，仅将期望末端平移在底盘当前航向方向上的投影按固定比例分配给底盘，底盘保持航向不变；其余误差由机械臂通过阻尼最小二乘（Damped Least Squares, DLS）逆运动学补偿。该方法实现简单、稳定性较好，适合小范围在线修正，但在“期望侧向位移较大”或“机械臂接近奇异位形”时存在明显不足：底盘无法主动转向形成新的可行运动方向，而机械臂被迫承担剩余侧移，可能导致关节增量快速增大、可操作性下降甚至修正失败。

针对该问题，本文提出一条渐进式改进路线。第一阶段将当前“底盘固定比例 + 机械臂 IK”升级为基于**全身 Jacobian 的加权阻尼最小二乘**，将底盘前进速度、底盘角速度和机械臂关节速度统一作为优化变量；第二阶段引入机械臂最小奇异值、条件数或 manipulability 作为奇异性指标，根据机械臂状态在线调整底盘与机械臂的运动权重；第三阶段进一步加入零空间 manipulability 优化、关节限位与障碍约束，并将问题写成小规模 QP；最终可由“单点状态修正”扩展为“短时域全身参考轨迹修正”，与 OCS2 MPC 自然衔接。

本文同时给出与现有代码的对应关系、推荐的软件模块重构方式，以及未来实验中的基线、消融实验、指标和预期现象。

---

# 1. 研究背景

## 1.1 移动机械臂接触任务中的在线修正

对于白板擦拭、硬盘插入、桌面擦拭、推门、插销等任务，仅依赖离线规划得到的名义轨迹通常不足以保证最终执行质量。实际系统中存在：

- RGB-D 感知误差；
- 手眼标定误差；
- SLAM / 里程计误差；
- 接触表面位置偏差；
- 被操作物轻微移动或形变；
- 机械臂关节跟踪误差；
- 末端力传感器噪声；
- 接触过程中真实几何与规划模型不一致。

因此，更合理的执行链路通常是：

```text
名义全身轨迹
      ↓
   OCS2 / MPC
      ↓
机器人实际运动
      ↓
力 / 位姿反馈
      ↓
导纳控制或其他反馈控制
      ↓
末端小范围 correction
      ↓
全身修正器
      ↓
新的全身参考
```

其中，本文关注的是“**末端 correction → 全身状态 / 全身轨迹 correction**”这一层。

---

## 1.2 为什么不能永远只让机械臂补偿

对于固定机械臂，末端误差通常只能依赖机械臂关节运动消除；但移动机械臂拥有额外的底盘自由度，例如差速底盘可控制：

\[
u_b =
\begin{bmatrix}
v \\
\omega
\end{bmatrix}
\]

其中：

- \(v\)：底盘沿自身航向的线速度；
- \(\omega\)：底盘绕世界 \(z\) 轴的角速度。

机械臂关节速度记作：

\[
\dot q =
\begin{bmatrix}
\dot q_1 & \cdots & \dot q_n
\end{bmatrix}^T
\]

则全身输入可写为：

\[
\boxed{
u =
\begin{bmatrix}
v \\
\omega \\
\dot q
\end{bmatrix}
}
\]

因此，移动机械臂实际上拥有“**底盘与机械臂共同完成同一个末端任务**”的冗余能力。

如果机械臂接近奇异位形、关节限位或不舒适姿态，继续强迫机械臂独立承担末端修正通常是不合理的。更合适的方式是：

> 机械臂状态健康时，以机械臂进行小范围快速修正；机械臂接近奇异或关节限位时，主动让底盘转向并移动，改变机械臂与目标之间的几何关系。

---

# 2. 系统状态与输入模型

本文当前针对：

> **差速底盘 + 6 自由度机械臂**

进行讨论。

## 2.1 全身状态

当前工程中的状态可写为：

\[
\boxed{
x =
\begin{bmatrix}
x_b &
y_b &
\theta_b &
q_1 &
q_2 &
\cdots &
q_6
\end{bmatrix}^T
}
\]

其中：

- \(x_b, y_b\)：底盘在世界 / odom 坐标系中的位置；
- \(\theta_b\)：底盘 yaw；
- \(q_i\)：机械臂关节角。

因此当前系统状态维度为：

\[
n_x = 3 + 6 = 9
\]

---

## 2.2 全身控制输入

差速底盘与机械臂联合输入为：

\[
\boxed{
u =
\begin{bmatrix}
v &
\omega &
\dot q_1 &
\cdots &
\dot q_6
\end{bmatrix}^T
}
\]

因此：

\[
n_u = 2 + 6 = 8
\]

---

## 2.3 全身 Jacobian

末端空间速度定义为：

\[
V_e =
\begin{bmatrix}
v_e \\
\omega_e
\end{bmatrix}
\in \mathbb{R}^6
\]

全身 Jacobian 满足：

\[
\boxed{
V_e = J_{\text{whole}}(x) u
}
\]

并可写成：

\[
\boxed{
J_{\text{whole}}
=
\begin{bmatrix}
J_{\text{base}} & J_{\text{arm}}
\end{bmatrix}
}
\]

对当前差速底盘：

\[
J_{\text{base}}
\in \mathbb{R}^{6\times 2}
\]

机械臂：

\[
J_{\text{arm}}
\in \mathbb{R}^{6\times 6}
\]

当前 `RobotModel::frameJacobian()` 已经完成这一层抽象，因此后续奇异性规避和全身分配并不需要修改底层 Pinocchio 模型接口。

---

# 3. 当前已实现方案

## 3.1 当前设计目标

当前 `WholeBodyKinematics` 的主要任务是：

> 根据末端期望修正，生成一个新的全身状态参考。

当前主要包含两种修正：

1. `correctedState()`：只处理 3D 平移；
2. `correctedState6D()`：处理 6D 位姿修正。

当前思想可以概括为：

```text
末端期望 correction
        ↓
先让底盘承担“当前航向方向上”的一部分
        ↓
目标末端位姿保持为完整 correction 后的目标
        ↓
机械臂通过 DLS IK 补偿剩余误差
```

---

# 4. 当前 3D 修正方法

## 4.1 期望末端位移

设末端期望沿世界坐标系单位方向：

\[
\hat d
\]

修正距离：

\[
s
\]

则完整期望位移为：

\[
\boxed{
\Delta p_d = s\hat d
}
\]

名义末端位置为：

\[
p_0
\]

因此最终目标末端位置始终定义为：

\[
\boxed{
p_d = p_0 + \Delta p_d
}
\]

---

## 4.2 当前底盘分配方式

差速底盘当前航向单位向量为：

\[
\boxed{
h =
\begin{bmatrix}
\cos\theta_b \\
\sin\theta_b
\end{bmatrix}
}
\]

由于差速底盘无法瞬时侧移，因此当前代码仅取期望位移在航向方向上的投影：

\[
h^T \Delta p_d^{xy}
\]

再乘固定的底盘分担比例：

\[
\alpha_b \in [0,1]
\]

得到：

\[
\boxed{
d_b =
\alpha_b
h^T\Delta p_d^{xy}
}
\]

并限制：

\[
|d_b| \le d_{b,\max}
\]

最终底盘位移为：

\[
\boxed{
\Delta p_b = h d_b
}
\]

当前算法中：

\[
\Delta \theta_b = 0
\]

即底盘只沿当前航向前后移动，不主动改变 yaw。

---

## 4.3 当前机械臂补偿方式

底盘完成部分位移后，机械臂继续尝试使末端到达完整目标：

\[
p_d = p_0 + \Delta p_d
\]

当前误差：

\[
\boxed{
e_p = p_d - p(q)
}
\]

只截取全身 Jacobian 中机械臂部分的前三行：

\[
J_p = J_{\text{arm}}^{position}
\]

然后采用阻尼最小二乘：

\[
\boxed{
\Delta q
=
J_p^T
\left(
J_pJ_p^T+\lambda I
\right)^{-1}
e_p
}
\]

迭代更新：

\[
q_{k+1}
=
q_k+\Delta q_k
\]

同时限制：

\[
|q-q_{\text{nominal}}|
\le
\Delta q_{\max}
\]

并满足真实关节限位：

\[
q_{\min}\le q\le q_{\max}
\]

---

# 5. 当前 6D 修正方法

当前 `correctedState6D()` 将修正定义在末端自身局部坐标系中：

\[
\delta x_e =
\begin{bmatrix}
\delta p_e \\
\delta \phi_e
\end{bmatrix}
\]

其中：

\[
\delta p_e\in\mathbb R^3
\]

为局部平移修正，

\[
\delta \phi_e\in\mathbb R^3
\]

为局部旋转向量。

---

## 5.1 局部平移到世界系

若当前末端姿态为：

\[
R_0
\]

则：

\[
\boxed{
\Delta p_d^W
=
R_0\delta p_e
}
\]

---

## 5.2 姿态目标

局部旋转向量通过指数映射转为旋转矩阵：

\[
R_\delta = \exp(\delta\phi_e)
\]

最终目标姿态：

\[
\boxed{
R_d = R_0R_\delta
}
\]

---

## 5.3 姿态误差

当前位置误差：

\[
e_p=p_d-p
\]

姿态误差：

\[
\boxed{
e_R
=
\log
\left(
R_dR^T
\right)
}
\]

最终 6D 误差：

\[
\boxed{
e=
\begin{bmatrix}
e_p\\
e_R
\end{bmatrix}
}
\]

当前同样只使用机械臂 Jacobian：

\[
J_{\text{arm}}\in\mathbb R^{6\times6}
\]

并采用：

\[
\boxed{
\Delta q
=
J_{\text{arm}}^T
\left(
J_{\text{arm}}J_{\text{arm}}^T+\lambda I
\right)^{-1}
e
}
\]

---

# 6. 当前方案的优点

当前方法虽然简单，但具有明显工程优势。

## 6.1 逻辑清晰

底盘与机械臂之间的职责非常直观：

```text
底盘：
只承担“当前航向可实现”的那部分

机械臂：
补偿其余误差
```

适合早期工程验证。

---

## 6.2 与差速底盘运动约束一致

当前算法没有直接给差速底盘虚构横向自由度，因此不会出现：

\[
v_y\neq0
\]

这种不可实现的瞬时命令。

---

## 6.3 接触修正不会无限破坏名义轨迹

`max_base_delta` 与 `max_joint_delta` 对修正量进行了限制，因此 correction 仍然局限在名义轨迹附近。

---

## 6.4 DLS 对轻度奇异具有一定鲁棒性

与普通伪逆相比：

\[
J^T(JJ^T+\lambda I)^{-1}
\]

在 Jacobian 接近奇异时不会直接数值爆炸，因此作为第一版在线 IK 是合理的。

---

# 7. 当前方案的主要问题

## 7.1 固定航向导致底盘无法主动参与侧移修正

考虑：

\[
\theta_b=0
\]

底盘沿世界 \(x\) 方向。

但末端需要：

\[
\Delta p_d=
\begin{bmatrix}
0\\
0.05\\
0
\end{bmatrix}
\]

即世界 \(y\) 方向移动 5 cm。

则：

\[
h=
\begin{bmatrix}
1\\
0
\end{bmatrix}
\]

因此：

\[
h^T\Delta p_d^{xy}=0
\]

得到：

\[
d_b=0
\]

当前算法会认为：

> 底盘完全无法帮助。

但实际上底盘可以通过：

```text
先旋转
   ↓
改变 heading
   ↓
再前进
```

在有限时间内产生侧向位移。

因此，当前算法使用的是“**瞬时可实现性**”，没有利用“**短时域可实现性**”。

---

## 7.2 机械臂承担所有剩余误差

当前 Jacobian 求解显式丢弃了底盘列：

\[
J_{\text{whole}}
=
\begin{bmatrix}
J_b & J_a
\end{bmatrix}
\]

但实际 IK 只使用：

\[
J_a
\]

因此即使底盘旋转可以显著改善机械臂姿态，也不会参与剩余误差的求解。

---

## 7.3 固定 `base_share` 无法反映机械臂状态

当前：

\[
\alpha_b = \text{constant}
\]

例如可能一直使用：

\[
\alpha_b=0.3
\]

但合理行为实际上应该依赖当前状态：

```text
机械臂可操作性高
→ 少动底盘

机械臂接近奇异
→ 多动底盘

机械臂接近关节限位
→ 多动底盘

底盘附近存在障碍
→ 少动底盘
```

因此未来更合理的是：

\[
\boxed{
\alpha_b = f(x, J_a, q, environment)
}
\]

而不是常数。

---

# 8. 机械臂奇异性

## 8.1 SVD

机械臂 Jacobian：

\[
J_a
\]

进行奇异值分解：

\[
\boxed{
J_a=U\Sigma V^T
}
\]

其中：

\[
\Sigma=
\operatorname{diag}
(\sigma_1,\sigma_2,\dots,\sigma_m)
\]

---

## 8.2 最小奇异值

最直接的奇异性指标是：

\[
\boxed{
\sigma_{\min}
}
\]

当：

\[
\sigma_{\min}\rightarrow0
\]

说明至少存在一个任务空间方向，机械臂很难产生运动。

因此：

```text
σ_min 较大
→ 机械臂状态较健康

σ_min 较小
→ 接近奇异
```

---

## 8.3 条件数

也可使用：

\[
\boxed{
\kappa(J)
=
\frac{\sigma_{\max}}
{\sigma_{\min}}
}
\]

越大表示 Jacobian 数值条件越差。

---

## 8.4 Manipulability

Yoshikawa manipulability：

\[
\boxed{
w(q)
=
\sqrt{
\det(JJ^T)
}
}
\]

它可以粗略描述末端速度椭球的总体体积。

当：

\[
w(q)\rightarrow0
\]

机器人接近奇异位形。

---

# 9. 改进方案一：全身加权阻尼最小二乘

这是最推荐的第一步。

## 9.1 核心思想

不再只求：

\[
\Delta q
\]

而统一求：

\[
\boxed{
\Delta z=
\begin{bmatrix}
\Delta s\\
\Delta\theta_b\\
\Delta q
\end{bmatrix}
}
\]

其中：

- \(\Delta s\)：底盘沿自身航向的小位移；
- \(\Delta\theta_b\)：底盘 yaw 修正；
- \(\Delta q\)：机械臂关节修正。

使用完整全身 Jacobian：

\[
\boxed{
J_{\text{whole}}
=
\begin{bmatrix}
J_s & J_\theta & J_a
\end{bmatrix}
}
\]

---

## 9.2 加权 DLS

如果直接使用：

\[
\Delta z=J^+e
\]

求解器并不知道“底盘不应该为 1 mm 的误差频繁转动”。

因此引入权重矩阵：

\[
W=
\operatorname{diag}
(
w_s,
w_\theta,
w_{q1},\dots,w_{qn}
)
\]

采用：

\[
\boxed{
\Delta z
=
W^{-1}J^T
\left(
JW^{-1}J^T+\lambda I
\right)^{-1}
e
}
\]

含义是：

- 权重大：不希望该自由度运动；
- 权重小：允许该自由度承担更多 correction。

---

# 10. 改进方案二：奇异性感知的动态权重

定义机械臂奇异程度：

\[
s_{\text{sing}}
=
f(\sigma_{\min})
\]

例如归一化：

\[
\boxed{
\alpha_{\text{sing}}
=
\operatorname{clip}
\left(
\frac{
\sigma_{\text{soft}}-\sigma_{\min}
}{
\sigma_{\text{soft}}-\sigma_{\text{hard}}
},
0,1
\right)
}
\]

其中：

- \(\sigma_{\text{soft}}\)：开始认为机械臂不够舒服；
- \(\sigma_{\text{hard}}\)：认为机械臂已经非常接近奇异。

则：

```text
α_sing ≈ 0
→ 机械臂健康

α_sing ≈ 1
→ 机械臂接近奇异
```

---

## 10.1 权重插值

正常情况下：

\[
W_{\text{normal}}
=
\operatorname{diag}
(
20,\,
30,\,
1,\dots,1
)
\]

表示：

```text
尽量少动底盘
优先使用机械臂
```

接近奇异时：

\[
W_{\text{sing}}
=
\operatorname{diag}
(
2,\,
1,\,
10,\dots,10
)
\]

表示：

```text
允许底盘转动和前进
抑制机械臂继续逼近奇异
```

最终：

\[
\boxed{
W =
(1-\alpha_{\text{sing}})W_{\text{normal}}
+
\alpha_{\text{sing}}W_{\text{sing}}
}
\]

这样底盘—机械臂分配会连续变化，而不是突然切换。

---

# 11. 差速底盘状态更新

全身 DLS 得到：

\[
\Delta z=
[
\Delta s,
\Delta\theta,
\Delta q
]
\]

不能将 \(\Delta s\) 简单理解为世界 \(x\) 位移。

可采用中点积分近似：

\[
\boxed{
x_{k+1}
=
x_k+
\cos
\left(
\theta_k+\frac{\Delta\theta}{2}
\right)\Delta s
}
\]

\[
\boxed{
y_{k+1}
=
y_k+
\sin
\left(
\theta_k+\frac{\Delta\theta}{2}
\right)\Delta s
}
\]

\[
\boxed{
\theta_{k+1}
=
\theta_k+\Delta\theta
}
\]

因此通过多次小步迭代，可以产生：

```text
转向
+
前进
+
继续转向
+
继续前进
```

最终在宏观上形成差速底盘的侧向位移。

---

# 12. 改进方案三：零空间 manipulability 优化

全身系统具有冗余自由度：

\[
n_u > 6
\]

因此完成末端任务后仍然存在零空间自由度。

可采用：

\[
\boxed{
\Delta z
=
J^\# e
+
\left(
I-J^\#J
\right)z_0
}
\]

第一项：

\[
J^\#e
\]

负责完成末端 correction。

第二项：

\[
(I-J^\#J)z_0
\]

负责优化次级目标。

---

## 12.1 以 manipulability 为次级目标

令：

\[
z_0
=
k_m\nabla w
\]

则：

\[
\boxed{
\Delta z
=
J^\# e
+
\left(
I-J^\#J
\right)
k_m\nabla w
}
\]

含义：

> 在尽量不影响末端任务的前提下，让全身状态主动朝机械臂 manipulability 更高的方向运动。

这允许机器人出现如下行为：

```text
末端任务基本保持不变
        ↓
底盘轻微转向
机械臂重新配置
        ↓
σ_min 增大
        ↓
为后续接触运动留下更好的姿态余量
```

---

# 13. 改进方案四：QP 全身修正

进一步可将问题写成小规模 QP。

定义：

\[
\Delta z=
[
\Delta s,
\Delta\theta,
\Delta q
]
\]

目标：

\[
\boxed{
\min_{\Delta z}
\;
\|J\Delta z-e\|_{Q_e}^2
+
\|\Delta z_b\|_{Q_b}^2
+
\|\Delta q\|_{Q_q}^2
+
\lambda_s C_{\text{sing}}(q+\Delta q)
}
\]

并加入：

### 底盘步长约束

\[
|\Delta s|
\le
\Delta s_{\max}
\]

\[
|\Delta\theta|
\le
\Delta\theta_{\max}
\]

### 关节增量约束

\[
|\Delta q_i|
\le
\Delta q_{i,\max}
\]

### 关节限位

\[
q_{\min}
\le
q+\Delta q
\le
q_{\max}
\]

后续还可以加入：

- 障碍距离约束；
- 碰撞约束；
- 底盘可行区域；
- 末端接触方向约束；
- 法向 / 切向 correction 权重；
- 关节速度和加速度平滑项。

---

# 14. 与当前代码的具体对应关系

## 14.1 不建议修改的部分

以下模块可以保持稳定：

```text
RobotModel
PinocchioRobotModel
wbmm_conversions
wbmm_ros_conversions
```

原因是：

- `RobotModel::frameJacobian()` 已经提供完整全身 Jacobian；
- Pinocchio 层已经完成 FK / Jacobian；
- conversions 只是数据格式桥梁。

---

## 14.2 当前最关键的修改位置

当前 `correctedState()` / `correctedState6D()` 中类似：

```cpp
jacobian.middleCols(base_input_columns, arm_dimension)
```

这一操作本质是在做：

```text
完整 J_whole
[v | ω | arm]
         ↑
只保留 arm
```

第一阶段改进应该变成：

```text
完整 J_whole
[v | ω | arm]
 ↑   ↑    ↑
全部参与求解
```

即：

\[
J_{\text{solver}}
=
J_{\text{whole}}
\]

---

# 15. 推荐的代码模块重构

不建议未来把所有逻辑继续堆进 `WholeBodyKinematics`。

建议结构：

```text
whole_body/
├── whole_body_kinematics.hpp
├── whole_body_kinematics.cpp
│
├── singularity_metrics.hpp
├── singularity_metrics.cpp
│
├── correction_policy.hpp
├── correction_policy.cpp
│
├── weighted_dls_solver.hpp
├── weighted_dls_solver.cpp
│
├── whole_body_integrator.hpp
├── whole_body_integrator.cpp
│
└── qp_correction_solver.cpp        # 后续
```

---

## 15.1 `WholeBodyKinematics`

只负责：

```text
FK
Jacobian
状态 / Pose 查询
```

---

## 15.2 `SingularityMetrics`

负责：

```text
sigma_min
condition number
manipulability
joint-limit margin
```

接口示例：

```cpp
struct SingularityInfo
{
  double sigma_min;
  double condition_number;
  double manipulability;
};

SingularityInfo evaluateArmSingularity(
    const wbmm::core::RobotModel& model,
    const wbmm::core::WholeBodyState& state,
    const std::string& ee_frame);
```

---

## 15.3 `CorrectionPolicy`

负责回答：

> 现在应该更相信底盘还是机械臂？

输入：

```text
σ_min
joint-limit margin
base obstacle margin
correction direction
```

输出：

```text
base translation weight
base yaw weight
arm weight
damping
```

---

## 15.4 `WeightedDlsSolver`

只负责：

\[
e,J,W
\rightarrow
\Delta z
\]

这样以后：

```text
普通 DLS
加权 DLS
自适应 DLS
QP
```

都可以替换，而不需要修改外层逻辑。

---

## 15.5 `WholeBodyIntegrator`

负责将：

\[
[\Delta s,\Delta\theta,\Delta q]
\]

积分为：

\[
[x',y',\theta',q']
\]

这样差速底盘运动学不会混进求解器。

---

# 16. 推荐配置结构

例如：

```cpp
struct WholeBodyCorrectionOptions
{
  double max_base_translation{0.02};
  double max_base_yaw{0.05};
  double max_joint_delta{0.05};

  double singularity_soft_threshold{0.08};
  double singularity_hard_threshold{0.03};

  double normal_base_translation_weight{20.0};
  double normal_base_yaw_weight{30.0};
  double normal_arm_weight{1.0};

  double singular_base_translation_weight{2.0};
  double singular_base_yaw_weight{1.0};
  double singular_arm_weight{10.0};

  double damping{1.0e-5};
};
```

这些数值只是初始实验参数，不应直接作为最终系统参数，需要根据具体机器人标定和实验调节。

---

# 17. 推荐的第一版改进算法

伪代码：

```cpp
for iteration in 0 ... max_iterations
{
    pose = FK(state);

    error = target_pose - pose;

    J_whole = wholeBodyJacobian(state);

    J_arm = armColumns(J_whole);

    sigma_min = minimumSingularValue(J_arm);

    weights = correctionPolicy(sigma_min);

    delta = weightedDampedLeastSquares(
        J_whole,
        error,
        weights);

    delta_base_translation =
        clamp(delta[0]);

    delta_base_yaw =
        clamp(delta[1]);

    delta_q =
        clamp(delta.tail());

    state = integrateDifferentialBase(
        state,
        delta_base_translation,
        delta_base_yaw,
        delta_q);
}
```

这一步就可以实现：

```text
机械臂健康
→ 主要使用机械臂

机械臂接近奇异
→ 底盘开始旋转 + 前进

机械臂严重奇异
→ 大幅提高机械臂代价
→ 底盘承担更多调整
```

---

# 18. 从“单点修正”升级为“短轨迹修正”

当前输出：

\[
x'
\]

只是一个最终 correction 状态。

但一旦引入：

```text
旋转
+
移动
+
机械臂协同
```

过程本身就具有路径意义。

因此后续推荐接口：

```cpp
wbmm::core::WholeBodyTrajectory
correctTrajectory(
    const wbmm::core::WholeBodyTrajectory& nominal,
    const EndEffectorCorrection& correction);
```

输出：

```text
t0 : x0
t1 : x1
t2 : x2
...
tN : xN
```

再通过现有：

```text
WholeBodyTrajectory
        ↓
toMpcTargetTrajectories()
        ↓
OCS2
```

交给 MPC。

这比“突然把参考点从 \(x\) 跳到 \(x'\)”更加平滑，也更加适合底盘旋转—移动这种非完整运动。

---

# 19. 与接触 / 导纳控制的关系

整条执行链可以设计为：

```text
力传感器
    ↓
重力补偿 / 滤波
    ↓
接触状态估计
    ↓
导纳控制
    ↓
局部 EE correction
    ↓
Whole-Body Correction Solver
    ↓
短时域全身参考轨迹
    ↓
OCS2 MPC
    ↓
底盘 + 机械臂
```

其中：

- 导纳控制器只回答“末端应该怎么修”；
- Whole-Body Solver 回答“底盘和机械臂分别怎么动”；
- MPC 回答“如何稳定、连续地执行这些参考”。

三者职责应保持独立。

---

# 20. 实验设计

## 20.1 实验目标

验证：

> 奇异性感知的全身修正是否能在完成相同末端 correction 的同时，减少机械臂奇异风险、降低过大关节运动，并提高任务成功率。

---

# 21. Baseline 设计

建议至少设计 4 组方法。

## Baseline A：Arm-only DLS

完全固定底盘：

\[
\Delta s=0
\]

\[
\Delta\theta=0
\]

只使用：

\[
J_{\text{arm}}
\]

这是最纯粹基线。

---

## Baseline B：当前方法

即当前工程方案：

```text
底盘固定 yaw
+
航向投影 × base_share
+
Arm DLS
```

记作：

> **Heading-Projection + Arm-DLS**

---

## Method C：Whole-Body Weighted DLS

使用：

\[
[v,\omega,\dot q]
\]

完整全身 Jacobian。

但权重固定：

\[
W=\text{constant}
\]

用于验证：

> “允许底盘旋转参与”本身带来的提升。

---

## Method D：Singularity-Aware Whole-Body DLS

本文推荐主要方法：

\[
W = W(\sigma_{\min})
\]

根据奇异程度动态调整权重。

---

## Method E：Whole-Body DLS + Null-Space Manipulability

进一步加入：

\[
(I-J^\#J)\nabla w
\]

用于验证 proactive singularity avoidance。

---

## Method F：QP Whole-Body Correction

加入：

```text
joint limits
base step constraints
collision
singularity cost
```

作为后续完整方法。

---

# 22. 推荐实验场景

## 场景 1：正常姿态，小范围前向 correction

目的：

> 验证改进方法不会在简单场景中不必要地移动底盘。

期望：

```text
Arm-only
≈ Current
≈ Adaptive Whole-Body
```

但自适应方法底盘运动应很小。

---

## 场景 2：纯侧向 correction

例如：

\[
\Delta p=
[0,\;0.05,\;0]^T
\]

底盘初始朝 \(x\)。

目的：

> 测试差速底盘“转向 + 前进”能力是否真正被利用。

---

## 场景 3：机械臂接近奇异 + 侧向 correction

这是最重要实验。

选择一个：

\[
\sigma_{\min}
\ll 1
\]

的初始姿态。

再要求末端沿机械臂弱方向移动。

预期：

```text
Arm-only:
Δq 大 / 失败

Current:
底盘帮助有限
机械臂仍然困难

Weighted Whole-Body:
底盘开始明显参与

Adaptive Whole-Body:
底盘主动旋转，σ_min 回升
```

---

## 场景 4：接近关节限位

测试：

> 即使 Jacobian 尚未奇异，是否可以通过 joint-limit cost 让底盘提前参与。

---

## 场景 5：连续接触任务

例如：

```text
沿白板横向擦拭
或
沿插槽连续插入
```

让 correction 连续存在若干秒。

目的：

> 验证方法在连续运行时是否产生底盘抖动、频繁转向或参考不连续。

---

# 23. 评价指标

## 23.1 末端跟踪误差

位置 RMSE：

\[
\boxed{
E_p
=
\sqrt{
\frac{1}{N}
\sum_{k=1}^{N}
\|p_k-p_{d,k}\|^2
}
}
\]

姿态误差：

\[
E_R
=
\frac{1}{N}
\sum_{k=1}^{N}
\|
\log(R_{d,k}R_k^T)
\|
\]

---

## 23.2 最小奇异值

记录整个任务中的：

\[
\boxed{
\sigma_{\min}^{\text{min}}
=
\min_t \sigma_{\min}(J_a(t))
}
\]

越大说明越不容易陷入奇异。

---

## 23.3 Manipulability

统计：

\[
w_{\min}
\]

与：

\[
\bar w
\]

评价整个任务期间机械臂姿态质量。

---

## 23.4 关节运动量

\[
\boxed{
C_q
=
\sum_k
\|\Delta q_k\|^2
}
\]

用于评价是否存在过度机械臂补偿。

---

## 23.5 底盘运动量

\[
C_b
=
\sum_k
\left(
\alpha_s|\Delta s_k|
+
\alpha_\theta|\Delta\theta_k|
\right)
\]

用于评价方法是否为了躲避奇异而过度移动底盘。

---

## 23.6 最大关节速度

\[
\max |\dot q_i|
\]

奇异附近通常会显著上升，因此这是非常直观的指标。

---

## 23.7 任务成功率

例如：

```text
末端达到目标
且
无 joint-limit violation
且
无 solver failure
且
无 collision
```

统计：

\[
\text{Success Rate}
\]

---

## 23.8 求解时间

实时性必须记录：

\[
t_{\text{solve}}
\]

至少给出：

```text
mean
P95
max
```

---

# 24. 推荐实验表格

| 方法 | EE RMSE | 最小 \(\sigma_{\min}\) | 平均 manipulability | 关节运动量 | 底盘运动量 | 成功率 | 平均求解时间 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Arm-only DLS |  |  |  |  | 0 |  |  |
| Current Heading Projection |  |  |  |  |  |  |  |
| Whole-Body Weighted DLS |  |  |  |  |  |  |  |
| Singularity-Aware DLS |  |  |  |  |  |  |  |
| + Null-Space |  |  |  |  |  |  |  |
| QP |  |  |  |  |  |  |  |

---

# 25. 消融实验

## Ablation 1：去掉底盘 yaw

只允许：

\[
\Delta s
\]

不允许：

\[
\Delta\theta
\]

与完整方法比较。

目的：

> 验证“主动转向”是不是解决侧向 correction 的关键。

---

## Ablation 2：固定权重 vs 自适应权重

固定：

\[
W=W_0
\]

对比：

\[
W=W(\sigma_{\min})
\]

目的：

> 验证奇异性反馈是否真正产生价值。

---

## Ablation 3：只使用 \(\sigma_{\min}\) vs manipulability

比较：

```text
sigma_min
condition number
manipulability
```

哪个更稳定、更适合作为 policy 输入。

---

## Ablation 4：无 Null-space vs 有 Null-space

验证：

> proactive singularity avoidance 是否能在任务误差几乎相同的情况下显著提高姿态余量。

---

# 26. 预期现象

预计：

### Arm-only DLS

- 小 correction 性能好；
- 奇异附近 \(\Delta q\) 增大；
- 侧向 correction 容易失败；
- 底盘能力完全浪费。

### 当前 Heading-Projection 方法

- 对沿底盘航向 correction 有改善；
- 简单稳定；
- 侧向 correction 仍然主要依赖机械臂；
- 无法主动通过旋转改变几何关系。

### Whole-Body Weighted DLS

- 底盘 yaw 可参与；
- 侧向 correction 成功率提高；
- 机械臂关节运动减少；
- 但固定权重可能导致底盘在某些健康状态下运动过多。

### Singularity-Aware Whole-Body DLS

- 健康状态主要由机械臂工作；
- 奇异附近底盘参与度自动增加；
- \(\sigma_{\min}\) 最低值明显改善；
- 最大关节速度下降；
- 成功率提高。

### Null-Space / QP

- 能进一步改善机械臂姿态；
- 更容易处理 joint limit / obstacle；
- 但求解复杂度增加。

---

# 27. 实施路线图

## Stage 0：保留当前方案作为 Baseline

不要删除当前：

```text
Heading Projection
+
Arm DLS
```

它应该保留下来作为后续论文实验基线。

---

## Stage 1：增加奇异性监控

先实现：

```cpp
double minimumSingularValue(...);
double manipulability(...);
double conditionNumber(...);
```

先只打印 / 记录，不改变控制行为。

目标：

> 观察真实任务中哪些状态开始进入危险区。

---

## Stage 2：完整全身 Jacobian DLS

把：

```text
Arm Jacobian
```

改成：

```text
Whole-Body Jacobian
```

求：

\[
[\Delta s,\Delta\theta,\Delta q]
\]

先使用固定权重。

---

## Stage 3：自适应权重

根据：

\[
\sigma_{\min}
\]

动态改变底盘 / 机械臂权重。

---

## Stage 4：短时域 trajectory correction

不再只输出一个：

\[
x'
\]

而输出：

\[
x_0,x_1,\dots,x_N
\]

交给 OCS2 MPC 跟踪。

---

## Stage 5：Null-space manipulability

加入：

\[
(I-J^\#J)\nabla w
\]

形成主动奇异规避。

---

## Stage 6：QP

加入：

```text
joint limits
collision constraints
base constraints
smoothness
singularity cost
```

形成完整全身在线 correction optimizer。

---

# 28. 推荐论文中的方法演进表达

如果未来要写论文，可以把方法演进描述为：

### Baseline

> Fixed-ratio heading-projected base correction with arm-only damped IK.

### Proposed Method V1

> Whole-body weighted damped least-squares correction using the complete mobile-manipulator Jacobian.

### Proposed Method V2

> Singularity-aware adaptive whole-body correction with online base-arm weight scheduling.

### Proposed Method V3

> Constraint-aware whole-body correction with manipulability maximization and trajectory-level MPC integration.

---

# 29. 可能形成的论文贡献点

如果实验结果足够好，可以进一步整理为以下贡献：

### Contribution 1

提出一种面向接触任务的移动机械臂全身在线 correction 框架，将导纳 / 接触反馈产生的末端修正统一映射为底盘与机械臂联合参考。

### Contribution 2

提出一种基于机械臂奇异性指标的动态底盘—机械臂运动分配方法，使底盘在机械臂接近奇异或不舒适状态时主动参与。

### Contribution 3

在差速底盘非完整约束下，通过小步全身修正与轨迹化输出实现“旋转 + 前进 + 机械臂协同”的短时域侧向 correction。

### Contribution 4

通过 OCS2 MPC 将在线 correction 与全身轨迹执行闭环结合，并在侧移、奇异、关节限位和连续接触场景中验证其有效性。

---

# 30. 需要特别注意的问题

## 30.1 奇异性阈值不能直接拍脑袋

例如：

```text
σ_min < 0.03
```

只能作为初始实验值。

不同机器人、不同单位、不同 Jacobian scaling 会影响奇异值。

最终应该基于：

```text
任务统计
+
实验曲线
+
关节速度增长趋势
```

确定阈值。

---

## 30.2 位置和姿态单位需要加权

6D error 中：

```text
m
+
rad
```

不能默认完全同权。

推荐：

\[
Q_e
=
\operatorname{diag}
(
w_p,w_p,w_p,
w_R,w_R,w_R
)
\]

否则 1 cm 和 1 rad 在数值上可能被不合理比较。

---

## 30.3 Jacobian 列也可能需要尺度归一化

底盘：

```text
m
rad
```

机械臂：

```text
rad
```

映射到 EE 后虽然 Jacobian 已统一到 twist，但优化变量本身仍需要合理权重。

因此 `W` 不只是“偏好”，也承担一定尺度归一化作用。

---

## 30.4 底盘和机械臂的动态响应不同

底盘通常：

```text
更慢
更重
惯性更明显
```

机械臂：

```text
局部小动作更快
```

所以即使几何上底盘可以参与，也不能让它对高频力噪声频繁响应。

建议：

```text
高频小 correction
→ arm

低频持续 correction
→ base + arm
```

未来可在 correction policy 中加入频率 / 时间尺度分配。

---

# 31. 总结

当前系统已经具备一个完整的第一版全身 correction 框架：

\[
\boxed{
\text{EE correction}
\rightarrow
\text{heading-projected base motion}
+
\text{arm DLS}
}
\]

它的核心优点是简单、稳定、容易集成，但主要问题在于：

\[
\boxed{
\text{底盘 yaw 没有进入 correction 求解}
}
\]

因此遇到：

```text
侧向 correction
+
机械臂接近奇异
```

时，机械臂会承担过多误差。

推荐的第一阶段改进是：

\[
\boxed{
\text{Arm-only DLS}
\rightarrow
\text{Whole-Body Weighted DLS}
}
\]

进一步：

\[
\boxed{
W
=
W(\sigma_{\min})
}
\]

形成：

> **Singularity-Aware Adaptive Whole-Body Correction**

再向后可以加入：

\[
\boxed{
\text{Null-Space Manipulability Optimization}
}
\]

以及：

\[
\boxed{
\text{QP-based Constraint-Aware Correction}
}
\]

最终形成：

```text
接触反馈
   ↓
末端 correction
   ↓
奇异性 / 约束感知
   ↓
底盘 + 机械臂联合修正
   ↓
短时域 WholeBodyTrajectory
   ↓
OCS2 MPC
   ↓
真实机器人
```

从研究角度看，最值得重点验证的问题并不是“能不能让底盘动”，而是：

> **能否根据机械臂当前可操作性和任务方向，在线决定底盘何时参与、参与多少，并在保证末端任务精度的同时提高整体运动可行性和鲁棒性。**

这个问题既有明确的工程意义，也适合设计系统性的 baseline、消融与实机实验。
