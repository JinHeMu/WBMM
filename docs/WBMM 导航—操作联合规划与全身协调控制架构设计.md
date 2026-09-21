# WBMM 导航—操作联合规划与全身协调控制架构设计

> 项目：WBMM（Whole-Body Mobile Manipulation）
> 平台：差速移动底盘 + 6 自由度机械臂
> 目标：实现从导航、任务接近、接触执行到撤离的统一移动操作系统，并支持底盘与机械臂的在线动态协调。

# 1. 总体研究目标

WBMM 最终不应采用传统的串行结构：

```
导航
↓
停车
↓
机械臂规划
↓
机械臂执行
```

而应实现：

```
长时域导航
↓
任务区域接近
↓
全身协调
↓
接触任务执行
↓
安全撤离
```

整个系统需要根据以下因素在线决定底盘与机械臂的运动分配：

- 末端任务要求；
- 环境障碍；
- 底盘差速运动学约束；
- 机械臂奇异性；
- 机械臂关节限位；
- 自碰撞与环境碰撞；
- 接触力反馈；
- 控制输入平滑性。

最终核心问题可以概括为：

$$
\boxed{
\text{Task-aware}
+
\text{Environment-aware}
+
\text{Configuration-aware}
+
\text{Contact-aware}
}
$$

即：

$$
\boxed{
\text{Whole-Body Mobile Manipulation}
}
$$

# 2. 总体系统架构

推荐最终形成以下数据流：

```
                    Task Planner
                         │
                         │
                         ▼
                  Nominal Task Plan
                         │
             ┌───────────┴───────────┐
             │                       │
             ▼                       ▼
       Mode / Phase              References
       NAVIGATION                Base Ref
       APPROACH                  EE Ref
       EXECUTE                   Posture Ref
       RETREAT                   Force Ref
             │                       │
             └───────────┬───────────┘
                         ▼
                Whole-Body NMPC
                    （OCS2）
                         │
          ┌──────────────┼──────────────┐
          │              │              │
          ▼              ▼              ▼
     EE Tracking     ESDF Collision   Metrics
                                    Singularity
                                    Joint Limits
          │              │              │
          └──────────────┼──────────────┘
                         ▼
                Optimal Whole-Body Input

             [v, omega, qdot1 ... qdot6]

                         │
             ┌───────────┴───────────┐
             ▼                       ▼
            Base                    Arm
```

对于接触任务：

```
Force Sensor
     │
     ▼
Force Processor
     │
     ▼
Admittance Controller
     │
     ▼
EE Reference Correction
     │
     ▼
Whole-Body NMPC
```

其中：

> 力控负责决定“末端任务应该如何修正”。

> NMPC 负责决定“底盘和机械臂应该如何实现这个任务”。

# 3. 推荐代码目录

```
WBMM/
├── src/
│
│   ├── core/
│   │   └── wbmm_core/
│   │       ├── include/wbmm_core/
│   │       │   ├── types.hpp
│   │       │   ├── robot_model.hpp
│   │       │   ├── trajectory.hpp
│   │       │   └── validation.hpp
│   │       └── test/
│
│   ├── robotics/
│   │   ├── wbmm_pinocchio/
│   │   │   ├── pinocchio_robot_model.hpp
│   │   │   └── pinocchio_robot_model.cpp
│   │   │
│   │   └── wbmm_collision/
│   │       ├── include/wbmm_collision/
│   │       │   ├── types.hpp
│   │       │   ├── collision_geometry.hpp
│   │       │   ├── collision_model.hpp
│   │       │   └── environment_collision_checker.hpp
│   │       ├── src/
│   │       └── config/
│
│   ├── map/
│   │   ├── wbmm_environment/
│   │   │   ├── include/wbmm_environment/
│   │   │   │   ├── types.hpp
│   │   │   │   ├── distance_field.hpp
│   │   │   │   ├── esdf_grid.hpp
│   │   │   │   └── esdf_loader.hpp
│   │   │   └── src/
│   │   │
│   │   ├── my_nvblox_bringup/
│   │   └── tracer_jaka_localization/
│   │
│   ├── metrics/
│   │   └── wbmm_robot_metrics/
│   │       ├── include/wbmm_robot_metrics/
│   │       │   ├── types.hpp
│   │       │   ├── arm_metrics.hpp
│   │       │   └── joint_limit_metrics.hpp
│   │       └── src/
│   │
│   ├── planning/
│   │   ├── search/
│   │   │   ├── kino_astar
│   │   │   ├── rrt
│   │   │   └── ...
│   │   │
│   │   ├── optimization/
│   │   │
│   │   └── task_planner/
│   │
│   ├── control/
│   │   ├── whole_body_force_control/
│   │   │
│   │   ├── whole_body_allocator/
│   │   │
│   │   ├── wbmm_ocs2/
│   │   │
│   │   └── wbmm_ocs2_ros/
│   │
│   ├── drivers/
│   ├── sim/
│   └── bringup/
│
├── config/
│   ├── robot/
│   ├── environment/
│   ├── planner/
│   └── controller/
│
├── data/
│   └── maps/
│       ├── map1/
│       │   └── esdf.npz
│       └── ...
│
└── docs/
```

# 4. 核心状态与输入定义

WBMM 当前建议统一采用：

$$
x=
\begin{bmatrix}
x_b &
y_b &
\theta_b &
q_1 &
q_2 &
\cdots &
q_6
\end{bmatrix}^{T}
$$

即 9 维状态。

控制输入为：

$$
u=
\begin{bmatrix}
v &
\omega &
\dot q_1 &
\dot q_2 &
\cdots &
\dot q_6
\end{bmatrix}^{T}
$$

即 8 维输入。

其中：

- `v`：差速底盘前向速度；
- `omega`：底盘角速度；
- `qdot`：机械臂关节速度。

# 5. 差速底盘运动学

底盘不允许直接产生世界坐标系中的任意 XY 速度。

其运动学模型为：

$$
\dot x_b=v\cos\theta_b
$$

$$
\dot y_b=v\sin\theta_b
$$

$$
\dot\theta_b=\omega
$$

机械臂：

$$
\dot q=\dot q
$$

因此整个移动机械臂系统：

$$
\dot x=
\begin{bmatrix}
v\cos\theta_b \
v\sin\theta_b \
\omega \
\dot q
\end{bmatrix}
$$

该模型已经适合直接用于 OCS2/NMPC。

当：

$$
v>0
$$

表示前进。

当：

$$
v<0
$$

表示后退。

当：

$$
v=0,\quad \omega\neq0
$$

表示原地旋转。

当：

$$
v\neq0,\quad \omega\neq0
$$

表示圆弧运动。

因此不需要额外设计：

```
FORWARD
BACKWARD
SPIN
TURN
```

这些行为应自然由优化得到的 `v` 与 `omega` 产生。

# 6. Whole-Body Jacobian

末端速度与整个机器人输入之间满足：

$$
V_{ee}=J_{whole}(x)u
$$

其中：

\begin{bmatrix}
J_{base} & J_{arm}
\end{bmatrix}
$$

因此：

J_{base}
\begin{bmatrix}
v\
\omega
\end{bmatrix}
+
J_{arm}\dot q
$$

这条公式是整个“底盘—机械臂动态分配”的核心。

同一个末端速度，可以通过不同的：

$$
v,\omega,\dot q
$$

组合实现。

这意味着移动机械臂具有冗余自由度。

优化器的任务就是利用这些冗余自由度，在满足末端任务的同时优化：

- 障碍距离；
- 机械臂奇异性；
- 关节限位；
- 控制能耗；
- 平滑性；
- 接触任务。

# 7. 当前 `base_share` 方法的问题

目前 WholeBodyKinematics 的方法本质上为：

```
EE correction
↓
取 correction 在底盘 heading 上的投影
↓
乘 base_share
↓
底盘完成一部分
↓
机械臂 IK 完成剩余部分
```

数学上类似：

\alpha_{base}
\operatorname{proj}*{heading}
\left(
\Delta p*{ee}
\right)
$$

其中：

$$
\alpha_{base}=base_share
$$

例如：

```
base_share = 0.4
```

意味着人工规定底盘承担一定比例。

该方法的问题在于：

1. 不能根据环境动态调整；
2. 不能根据机械臂奇异性动态调整；
3. 不能根据关节限位动态调整；
4. 无法提前预测未来构型；
5. 对差速底盘横向任务支持较弱；
6. 底盘 yaw 没有真正参与协调。

因此最终应逐步淘汰：

```
correctedStateWorld6D(..., base_share);
```

并由 NMPC 直接决定：

$$
v,\omega,\dot q
$$

# 8. Whole-Body 动态分配问题

未来不再定义：

```
base_share = 0.4
```

而是求解：

\arg\min_u J(x,u)
$$

最基本的目标函数可以写为：

J_{task}
+
\lambda_u J_{input}
+
\lambda_{joint}J_{joint}
+
\lambda_{sing}J_{sing}
+
\lambda_{obs}J_{obs}
$$

其中不同代价分别负责不同目标。

# 9. 末端任务代价

末端位置误差：

p_{ee}(x)-p_{ref}
$$

姿态误差：

\log
\left(
R_{ref}R_{ee}^{T}
\right)
$$

因此：

e_p^{T}Q_pe_p
+
e_R^{T}Q_Re_R
$$

这一项只规定：

> 末端应该去哪里。

但不规定：

> 底盘必须怎么动，机械臂必须怎么动。

因此为动态分配保留了自由度。

# 10. 控制输入代价

为了避免底盘和机械臂产生不必要运动：

u^{T}Ru
$$

其中：

$$
R=
\operatorname{diag}
\left(
w_v,
w_{\omega},
w_{q_1},
\dots,
w_{q_6}
\right)
$$

如果：

$$
w_v,w_\omega
$$

较大，则优化器更倾向使用机械臂。

如果机械臂输入权重较大，则更倾向使用底盘。

但这些只应该作为基础偏好，而不是固定分配规则。

# 11. 输入平滑代价

可以加入：

(u-u_{prev})^{T}
R_{\Delta}
(u-u_{prev})
$$

避免出现：

```
当前周期：
v = 0.5

下一周期：
v = -0.5
```

这种不连续输入。

# 12. 机械臂奇异性评价

机械臂 Jacobian：

$$
J_a
$$

奇异值分解：

U\Sigma V^{T}
$$

其中：

$$
\Sigma=
\operatorname{diag}
(
\sigma_1,
\sigma_2,
\dots
)
$$

最小奇异值：

$$
\sigma_{min}
$$

可以用于评价机械臂最困难运动方向。

当：

$$
\sigma_{min}\rightarrow0
$$

说明机械臂接近奇异。

# 13. Manipulability

Yoshikawa 操作度：

\sqrt{
\det
\left(
JJ^{T}
\right)
}
$$

也可以利用奇异值计算：

\prod_i\sigma_i
$$

实际用于优化时，不建议直接最大化原始值，而可以构造平滑 penalty。

例如：

\frac{1}
{
(\sigma_{min}+\epsilon)^2
}
$$

或者使用阈值 penalty：

\begin{cases}
(\sigma_{safe}-\sigma_{min})^2,
&
\sigma_{min}<\sigma_{safe}
\
0,
&
\sigma_{min}\ge\sigma_{safe}
\end{cases}
$$

# 14. 关节限位评价

对于第 i 个关节：

\frac{
\min
(
q_i-q_{min,i},
q_{max,i}-q_i
)
}
{
(q_{max,i}-q_{min,i})/2
}
$$

当关节处于中间位置：

$$
m_i\approx1
$$

接近关节限位：

$$
m_i\rightarrow0
$$

可以定义：

\min_i m_i
$$

也可以直接使用 barrier cost。

例如：

\sum_i
\left[
\frac{1}
{(q_i-q_{min,i}+\epsilon)^2}
+
\frac{1}
{(q_{max,i}-q_i+\epsilon)^2}
\right]
$$

# 15. 为什么奇异性可以让底盘自动参与

假设继续使用机械臂完成末端任务，会导致：

$$
\sigma_{min}\downarrow
$$

于是：

$$
J_{sing}\uparrow
$$

NMPC 会比较：

```
方案 A
机械臂继续伸展
→ EE误差小
→ 奇异性代价很大

方案 B
底盘旋转/后退
+ 机械臂调整
→ EE误差仍然小
→ 奇异性代价下降
```

若方案 B 总代价更小，则优化器自动选择：

$$
\boxed{
\text{机械臂接近奇异}
\Rightarrow
\text{底盘承担更多运动}
}
$$

不需要：

```
if (singular)
{
    base_share = 0.8;
}
```

# 16. ESDF 环境模块

环境地图建议抽象成：

```
DistanceField
```

核心接口：

```
class DistanceField
{
public:
    virtual ~DistanceField() = default;

    virtual bool contains(
        const Eigen::Vector3d& point) const = 0;

    virtual DistanceQuery query(
        const Eigen::Vector3d& point,
        bool compute_gradient = true) const = 0;
};
```

其中：

```
struct DistanceQuery
{
    double distance;
    Eigen::Vector3d gradient;

    bool observed;
    bool inside_map;
};
```

上层规划器和控制器不应该知道：

```
NVBlox
NPZ
Octomap
Voxel Grid
```

只需要：

```
environment->query(point);
```

# 17. ESDF 数据加载

当前 NVBlox 可以导出：

```
esdf
occupancy
observed
origin
voxel_size
bounds_max
frame_id
```

推荐：

```
data/maps/map1/esdf.npz
```

然后：

```
map1/esdf.npz
    ↓
NpzEsdfLoader
    ↓
EsdfGrid
    ↓
DistanceField
```

规划器和控制器都依赖统一的 `DistanceField`。

# 18. ESDF 查询

对于任意空间点：

$$
p=
[x,y,z]^T
$$

找到周围 8 个 voxel：

$$
d_{000},
d_{001},
\dots,
d_{111}
$$

使用三线性插值得到：

\operatorname{Trilinear}
(
d_{000},
\dots,
d_{111}
)
$$

并得到梯度：

\begin{bmatrix}
\frac{\partial d}{\partial x}\
\frac{\partial d}{\partial y}\
\frac{\partial d}{\partial z}
\end{bmatrix}
$$

梯度对于后端轨迹优化和 NMPC 很重要。

# 19. 机器人碰撞模型

第一版不建议使用复杂 Mesh 精确碰撞。

推荐采用：

```
Collision Sphere
```

对底盘和机械臂进行球体近似。

例如：

```
struct CollisionSphere
{
    std::string frame_name;

    Eigen::Vector3d center_local;

    double radius;
};
```

碰撞球在 link 坐标系中的位置：

$$
{}^{L}p_s
$$

通过机器人 FK：

$$
{}^{W}T_L
$$

得到世界坐标：

{}^{W}T_L
{}^{L}p_s
$$

# 20. 环境碰撞距离

ESDF 给出：

$$
d_{env}
$$

碰撞球半径：

$$
r
$$

安全距离：

$$
d_{safe}
$$

定义 clearance：

## d_{env}

## r

d_{safe}
$$

如果：

$$
c>0
$$

表示安全。

如果：

$$
c=0
$$

表示处于安全边界。

如果：

$$
c<0
$$

表示发生碰撞或进入安全距离。

# 21. 环境碰撞代价

可定义：

\begin{cases}
(d_{activation}-c)^2,
&
c<d_{activation}
\
0,
&
c\ge d_{activation}
\end{cases}
$$

同时可以分别计算：

J_{base,obs}
+
J_{arm,obs}
$$

这样可以单独观察：

- 底盘障碍距离；
- 机械臂障碍距离；
- 全身最小安全距离。

# 22. 为什么障碍物可以改变动态分配

假设底盘向前运动会导致：

$$
d_{base}\downarrow
$$

于是：

$$
J_{base,obs}\uparrow
$$

优化器比较：

```
方案 A
底盘继续前进
机械臂少动
→ 碰撞代价大

方案 B
底盘保持
机械臂多动
→ 碰撞代价小
```

于是自动得到：

$$
\boxed{
\text{底盘附近存在障碍}
\Rightarrow
\text{机械臂承担更多任务}
}
$$

反过来：

$$
\boxed{
\text{机械臂接近奇异/限位}
\Rightarrow
\text{底盘承担更多任务}
}
$$

这就是所需的动态全身分配。

# 23. 动态分配不是 Mode Switching

需要明确区分两个概念。

## 23.1 Dynamic Allocation

发生在同一个任务阶段内部。

例如 EXECUTE 阶段：

```
t = 1s

底盘附近有障碍
→ arm 多动


t = 3s

arm 接近奇异
→ base 多动


t = 5s

两者状态均良好
→ 根据整体 cost 自由分配
```

这是连续优化问题：

$$
\boxed{
\text{Dynamic Whole-Body Allocation}
}
$$

## 23.2 Task Mode

例如：

```
NAVIGATION

APPROACH

EXECUTE

RETREAT
```

它描述的是：

> 当前机器人正在完成什么任务？

因此：

$$
\boxed{
\text{Mode 决定当前优化目标}
}
$$

而：

$$
\boxed{
\text{Cost 决定 Base 与 Arm 如何分配}
}
$$

# 24. 力控的定位

现阶段不建议直接把实际接触力作为 OCS2 的预测变量。

原因是当前系统动力学中没有：

$$
x,u
\rightarrow
F
$$

的接触动力学模型。

如果直接定义：

(F-F_d)^2
$$

OCS2 并不知道：

> 当前状态和控制输入将导致多大的未来接触力。

因此第一版推荐：

```
Force Error
↓
Admittance
↓
EE Pose Correction
↓
OCS2
```

# 25. 导纳控制

设期望力：

$$
F_d
$$

实际力：

$$
F_m
$$

力误差：

$$
F_e=F_d-F_m
$$

导纳模型：

F_e
$$

得到法向位移修正：

$$
\Delta x_F
$$

然后修改末端参考：

p_{nom}
+
n\Delta x_F
$$

其中：

$$
n
$$

为接触表面法向。

# 26. 力控与 NMPC 的职责划分

最终建议：

```
Planner
    ↓
Nominal EE Task
    ↓
Admittance
    ↓
Corrected EE Reference
    ↓
Whole-Body NMPC
```

即：

$$
\boxed{
\text{Force Control}
:
\text{任务应该如何修正}
}
$$

$$
\boxed{
\text{NMPC}
:
\text{机器人如何实现这个任务}
}
$$

# 27. 未来可扩展 Contact-Aware NMPC

未来如果建立接触模型：

K_e\delta
+
D_e\dot\delta
$$

那么 OCS2 可以预测：

$$
F_n(x,u)
$$

此时才适合真正加入：

w_F
(F_n-F_d)^2
$$

形成：

$$
\boxed{
\text{Contact-aware NMPC}
}
$$

但该方向建议作为后续扩展，而不是第一版。

# 28. 导航与操作联合规划

联合规划不应该简单理解为：

```
Base Path
+
Arm Path
```

而应该考虑完整状态：

$$
x=
[x_b,y_b,\theta,q]^T
$$

并在任务区域考虑：

- 底盘轨迹；
- 机械臂可达性；
- 操作度；
- 关节限位；
- 环境碰撞；
- 最终 EE 任务。

总体规划目标可以写为：

J_{path}
+
\lambda_cJ_{collision}
+
\lambda_tJ_{task}
+
\lambda_mJ_{manip}
+
\lambda_lJ_{joint}
+
\lambda_sJ_{smooth}
$$

# 29. 长时域规划不建议全程高维优化

如果机器人距离任务目标很远，例如 10 m：

```
Start
  │
  │
  │     Navigation Region
  │
  │
  ▼
Task-aware Region
  │
  ▼
Contact Task
```

没有必要从起点开始就一直高维优化：

$$
[x,y,\theta,q_1,\dots,q_6]
$$

推荐分层：

```
Long-Horizon Navigation
        ↓
Kino A* / Hybrid A*
        ↓
进入 Task-aware Region
        ↓
Whole-Body Planning
        ↓
Task Execution
```

即：

$$
\boxed{
\text{Global Base-Dominant Planning}
+
\text{Local Whole-Body Planning}
}
$$

# 30. Planning 与 Control 的关系

规划器负责：

> 整体应该如何完成任务？

时间尺度：

```
几秒 ～ 数十秒
```

NMPC 负责：

> 接下来一小段时间应该如何执行？

时间尺度：

```
几十毫秒控制周期
+
1～2 秒预测窗口
```

底层控制负责：

> 电机如何真正执行输入？

时间尺度：

```
1～10 ms
```

因此：

```
Long-Horizon Planner
        ↓
Whole-Body NMPC
        ↓
Low-Level Controller
```

# 31. 导航与执行不建议使用两个完全独立控制器

不推荐：

```
Navigation Planner
↓
Navigation Controller
↓
停车
↓
Manipulator Controller
↓
Force Controller
```

推荐：

```
Task Planner
     ↓
Task Phase
     ↓
Whole-Body NMPC
     ↓
[v, omega, qdot]
```

整个过程中：

$$
x=
[x,y,\theta,q]
$$

始终不变。

$$
u=
[v,\omega,\dot q]
$$

始终不变。

变化的是：

> 当前最重要的任务是什么。

# 32. Reference 不应该只设计成一种轨迹

当前如果只使用一个：

```
TargetTrajectories.stateTrajectory
```

容易产生语义冲突。

例如：

```
WholeBodyTrajectoryCost
```

希望它表示：

$$
[x,y,\theta,q_1,\dots,q_6]
$$

而 EE Tracking 又希望它表示：

$$
[p_x,p_y,p_z,q_x,q_y,q_z,q_w]
$$

因此未来建议设计自己的：

```
struct WbmmTaskReference
{
    BaseTrajectory base;

    ArmTrajectory posture;

    EndEffectorTrajectory end_effector;

    ForceTrajectory force;

    TaskMode mode;
};
```

# 33. 推荐的 Reference Manager

可以在 OCS2 ReferenceManager 外包一层：

```
class WbmmReferenceManager
{
public:

    const BaseTrajectory&
    baseReference() const;

    const ArmTrajectory&
    postureReference() const;

    const EndEffectorTrajectory&
    eeReference() const;

    const ForceTrajectory&
    forceReference() const;

    TaskMode mode(double time) const;
};
```

逻辑结构：

```
WbmmReferenceManager
│
├── ModeSchedule
├── Base Reference
├── Arm Posture Reference
├── EE Reference
└── Force Reference
```

# 34. 不建议“切换 Reference 类型”

不推荐：

```
NAVIGATION
↓
WholeBodyReference

切换

EXECUTE
↓
EEReference
```

推荐：

```
               Reference Set
        ┌──────────┼──────────┐
        ▼          ▼          ▼
    Base Ref     EE Ref    Posture Ref
        │          │          │
        └──────────┼──────────┘
                   ▼
               Task Mode
                   ▼
            Different Costs
                   ▼
                OCS2
```

即：

$$
\boxed{
\text{Reference 共存}
}
$$

$$
\boxed{
\text{Mode 决定 Reference 的优先级}
}
$$

# 35. 推荐的 Task Mode

第一版建议只使用三个：

```
enum class TaskMode
{
    Navigation = 0,
    Approach   = 1,
    Execute    = 2
};
```

以后再扩展：

```
RETREAT
RECOVERY
CONTACT_ESTABLISH
```

# 36. Navigation Mode

导航阶段主要关注底盘轨迹。

推荐：

w_bJ_{base}
+
w_pJ_{posture}
+
w_oJ_{obs}
+
w_uJ_u
$$

其中：

$$
w_b
$$

较大。

机械臂主要保持导航舒适构型：

$$
q\approx q_{nav}
$$

即：

```
Base Tracking       Strong

Arm Posture         Medium

EE Tracking         Off / Weak

Collision           On
```

# 37. Approach Mode

接近任务区域以后：

w_{ee}J_{ee}
+
w_bJ_{base}
+
w_pJ_{posture}
+
w_sJ_{sing}
+
w_oJ_{obs}
+
w_uJ_u
$$

其中：

$$
w_{ee}>w_b
$$

此时允许：

- 底盘调整；
- 原地旋转；
- 后退；
- 机械臂提前展开；
- 全身构型重新调整。

# 38. Execute Mode

接触任务阶段：

w_{ee}J_{ee}
+
w_sJ_{sing}
+
w_lJ_{joint}
+
w_oJ_{obs}
+
w_uJ_u
$$

而 EE reference 由：

p_{task}
+
\Delta p_F
$$

产生。

此时 Base Tracking 应该非常弱甚至关闭。

否则如果强制：

$$
x_b=x_b^{ref}
$$

会严重限制 Base/Arm 动态分配。

# 39. 推荐拆分 Tracking Cost

不建议长期只保留：

```
WholeBodyTrajectoryCost
```

建议拆成：

```
BaseTrackingCost

ArmPostureCost

EndEffectorTrackingCost
```

对应：

|x_b-x_b^d|_{Q_b}^2
$$

|q-q_{nom}|_{Q_q}^2
$$

|p_{ee}-p_d|*{Q_p}^2
+
|\log(R_dR*{ee}^{T})|_{Q_R}^2
$$

这样 Mode 切换时只需要调整各项权重。

# 40. 推荐 Mode 与 Cost 的关系

| Mode       | Base Tracking | Arm Posture | EE Tracking | Singularity | Collision | Force |
| ---------- | ------------- | ----------- | ----------- | ----------- | --------- | ----- |
| Navigation | 强            | 中          | 关闭        | 弱          | 开启      | 关闭  |
| Approach   | 弱            | 弱          | 强          | 开启        | 开启      | 关闭  |
| Execute    | 关闭/很弱     | 很弱        | 强          | 强          | 开启      | 开启  |
| Retreat    | 弱            | 中          | 强          | 开启        | 开启      | 关闭  |

模式决定：

> 当前什么任务最重要。

动态优化决定：

> 这个任务由底盘还是机械臂完成。

# 41. Mode 过渡可以平滑

不一定需要：

```
Base Weight = 100
↓ 瞬间
Base Weight = 0

EE Weight = 0
↓ 瞬间
EE Weight = 100
```

可以定义：

$$
\alpha(t)\in[0,1]
$$

然后：

(1-\alpha)J_{base}
+
\alpha J_{ee}
+
J_{other}
$$

导航时：

$$
\alpha=0
$$

接近阶段：

$$
\alpha:0\rightarrow1
$$

执行阶段：

$$
\alpha=1
$$

这样 NAV → EXEC 的过渡更加平滑。

# 42. OCS2 ModeSchedule 的定位

OCS2 的 ModeSchedule 可以用于描述：

```
NAVIGATION

APPROACH

EXECUTE
```

但是第一版不需要切换动力学模型。

因为动力学始终是：

$$
\dot x=
\begin{bmatrix}
v\cos\theta\
v\sin\theta\
\omega\
\dot q
\end{bmatrix}
$$

变化的是：

- Reference；
- Cost Weight；
- Constraint Activation。

因此第一版 Mode 更适合理解成：

$$
\boxed{
\text{Task Mode}
}
$$

而不是：

$$
\boxed{
\text{Dynamics Mode}
}
$$

# 43. 规划器最终建议输出 TaskPlan

未来规划器不应该只输出：

```
9D WholeBodyTrajectory
```

推荐输出：

```
struct TaskPlan
{
    ModeSchedule mode_schedule;

    BaseTrajectory base_reference;

    ArmTrajectory posture_reference;

    EndEffectorTrajectory ee_reference;

    ForceTrajectory force_reference;
};
```

逻辑：

```
TaskPlan
│
├── NAVIGATION
│   ├── Base Ref
│   └── Arm Posture Ref
│
├── APPROACH
│   ├── Base Soft Ref
│   └── EE Ref
│
└── EXECUTE
    ├── EE Task Ref
    └── Force Ref
```

# 44. 为什么 Planner 不应该锁死全部 9D 状态

如果 Planner 输出：

[x_b,y_b,\theta,q_1,\dots,q_6]
$$

然后 MPC 强跟踪：

$$
J=
|x-x_d|_Q^2
$$

那么：

```
Base 怎么动
Arm 怎么动
```

都已经被 Planner 提前决定。

此时：

```
底盘有障碍 → Arm 多动

Arm 奇异 → Base 多动
```

几乎没有自由度。

因此更合适的是：

```
任务参考：强

Whole-body nominal：弱
```

即：

$$
w_{task}\gg w_{nominal}
$$

# 45. Base / Arm Contribution 指标

为了分析动态分配，可以计算：

J_b
\begin{bmatrix}
v\
\omega
\end{bmatrix}
$$

J_a\dot q
$$

并满足：

V_{base}
+
V_{arm}
$$

定义 Base Contribution Ratio：

\frac{
|V_{base}|
}{
|V_{base}|
+
|V_{arm}|
+
\epsilon
}
$$

机械臂贡献：

1-\alpha_b
$$

这两个值建议用于实验记录，而不是作为人为设定的控制比例。

# 46. 典型动态协调行为

假设机器人擦白板：

```
            Whiteboard
──────────────────────────

EE → → → → → → →


        Arm
         |
       Robot ↑
```

底盘 heading 与 EE 运动方向垂直。

传统 `base_share`：

```
EE 横向运动

heading projection ≈ 0

↓
Base 不动

↓
Arm 一直伸
```

最终：

```
Joint Margin ↓

sigma_min ↓

Manipulability ↓
```

NMPC 则可能预测：

```
未来机械臂即将接近奇异
```

于是提前执行：

```
Base 原地旋转

+

Arm 反向补偿

↓
EE 仍保持原任务轨迹
```

之后：

```
Base 前进

+

Arm 回收到舒适构型
```

最终实现：

$$
\boxed{
\text{保持外部任务不变}
}
$$

同时：

$$
\boxed{
\text{在线改变内部 Base/Arm 分工}
}
$$

这是真正的 Whole-Body Coordination。

# 47. 与 QP Whole-Body Allocation 的关系

在正式进入 NMPC 前，可以先实现一个速度级 QP 验证原理：

$$
\min_u
|Ju-V_d|*{W_t}^2
+
u^TW_uu
+
(u-u*{prev})^TW_s(u-u_{prev})
$$

约束：

$$
-v_{max}
\le
v
\le
v_{max}
$$

$$
-\omega_{max}
\le
\omega
\le
\omega_{max}
$$

$$
-\dot q_{max}
\le
\dot q
\le
\dot q_{max}
$$

以及：

$$
q_{min}
\le
q+\dot q\Delta t
\le
q_{max}
$$

该 QP 可以作为：

```
WholeBodyAllocator
```

用于验证：

- Base/Arm 动态分配；
- 奇异性影响；
- Joint Limit 影响；
- Input Weight 影响。

# 48. QP 与 NMPC 的区别

QP 解决的是：

> 当前这一瞬间谁应该多动？

其视野主要是：

```
Now
```

NMPC 解决的是：

> 当前怎么动，未来几秒整体更好？

例如：

```
当前机械臂还没有奇异

但是预测 1.5 s 后会奇异
```

NMPC 可以提前让底盘：

- 旋转；
- 后退；
- 调整位置；

从而避免未来进入差构型。

因此：

$$
\boxed{
\text{QP：即时分配}
}
$$

$$
\boxed{
\text{NMPC：预测性动态分配}
}
$$

# 49. 推荐最终 OCS2 Objective

最终可以形成：

w_{ee}J_{ee}
+
w_bJ_{base}
+
w_pJ_{posture}
+
w_uJ_{input}
+
w_{\Delta u}J_{\Delta u}
+
w_sJ_{sing}
+
w_lJ_{joint}
+
w_oJ_{collision}
$$

在接触阶段：

p_{task}
+
\Delta p_{force}
$$

随着 Mode 改变，各项权重变化。

# 50. 规划、控制与接触的三个时间尺度

## 长时域：Planning

时间尺度：

```
数秒 ～ 数十秒
```

负责：

> 整体应该怎样完成任务？

包括：

- 导航路径；
- 任务接近方向；
- task-aware region；
- nominal whole-body trajectory；
- mode schedule。

## 中时域：NMPC

控制周期：

```
几十毫秒
```

预测：

```
约 1 ～ 2 秒
```

负责：

> 接下来这一小段怎么协调执行？

包括：

- Base/Arm 动态分配；
- EE Tracking；
- ESDF 避障；
- 奇异性规避；
- 关节限位；
- 输入平滑。

## 快时域：Low-Level Control

时间尺度：

```
1 ～ 10 ms
```

负责：

- 底盘速度控制；
- 机械臂关节 servo；
- 驱动器控制；
- 力传感器采集。

# 51. 推荐的整体执行 Pipeline

```
                    Environment
                      D435/D455
                         │
                         ▼
                       NVBlox
                         │
                    TSDF / ESDF
                         │
                         ▼
                  wbmm_environment
                         │
                  distance + gradient
                         │
                         ▼
                  wbmm_collision
                         │
                         │
                         ▼

Task Planner ──────► TaskPlan
                         │
                         ├── Mode Schedule
                         ├── Base Reference
                         ├── EE Reference
                         ├── Posture Reference
                         └── Force Reference
                                  │
                                  ▼
                           Reference Manager
                                  │
                                  ▼
Force Sensor ─► Admittance ─► EE Reference Correction
                                  │
                                  ▼
                              OCS2 NMPC
                ┌─────────────────┼─────────────────┐
                │                 │                 │
                ▼                 ▼                 ▼
             EE Task        Collision / ESDF    Metrics
                                                sigma_min
                                                joint margin
                │                 │                 │
                └─────────────────┼─────────────────┘
                                  ▼
                    [v, omega, qdot1 ... qdot6]
                                  │
                     ┌────────────┴────────────┐
                     ▼                         ▼
                    Base                      Arm
```

# 52. 第一阶段建议实现顺序

## Step 1：整理 Reference

把当前单一 reference 逐渐拆成：

```
Base Reference

Arm Posture Reference

EE Reference
```

## Step 2：拆 Tracking Cost

新增：

```
BaseTrackingCost

ArmPostureCost

EndEffectorTrackingCost
```

降低对：

```
WholeBodyTrajectoryCost
```

的依赖。

## Step 3：去除核心 `base_share`

让 EE Tracking Cost 直接交给 OCS2。

由 OCS2 决定：

$$
v,\omega,\dot q
$$

## Step 4：加入 Joint Limit

当前已有 Joint Limit Soft Constraint，可以先保留并验证：

```
Arm 接近限位
↓
Base 是否自动开始参与
```

## Step 5：加入 Singularity Cost

优先加入：

$$
\sigma_{min}
$$

相关代价。

实验：

```
不加 singularity cost

VS

加入 singularity cost
```

观察 Base Contribution。

## Step 6：接入 ESDF

将：

```
NVBlox
↓
ESDF NPZ
↓
DistanceField
↓
Collision Checker
```

接入 OCS2 Environment Collision。

验证：

```
Base 前方有障碍

↓
Base Contribution ↓

Arm Contribution ↑
```

## Step 7：加入 Mode

第一版：

```
NAVIGATION

APPROACH

EXECUTE
```

不同 mode 使用不同 cost 权重。

## Step 8：重新接入力控

改为：

```
Admittance

↓
EE Reference Correction

↓
OCS2
```

而不是：

```
Admittance

↓
base_share + IK

↓
9D reference
```

# 53. 推荐实验设计

## 实验 A：固定底盘

```
Base fixed

Arm only
```

记录：

- EE tracking error；
- sigma_min；
- manipulability；
- joint margin；
- task success。

## 实验 B：固定 base_share

例如：

```
base_share = 0.4
```

记录同样指标。

## 实验 C：动态 Whole-Body NMPC

加入：

- EE task；
- joint limit；
- singularity；
- ESDF；
- input cost。

比较：

- EE tracking error；
- Force tracking error；
- Base Contribution Ratio；
- Arm Contribution Ratio；
- minimum sigma；
- minimum joint margin；
- minimum obstacle clearance；
- control effort；
- task success rate。

# 54. 推荐论文逻辑

整个方法章节可以形成以下逻辑：

```
Nominal Navigation-Manipulation Planning
                    │
                    ▼
              Task References
                    │
                    ▼
            Contact Adaptation
                    │
                    ▼
         Whole-Body Predictive Control
                    │
        ┌───────────┼───────────┐
        ▼           ▼           ▼
    Task Error   Environment   Configuration
                  Safety        Quality
        │           │           │
        └───────────┼───────────┘
                    ▼
        Adaptive Base-Arm Allocation
```

核心思想：

$$
\boxed{
\text{Nominal Planning}
+
\text{Online Whole-Body Reallocation}
}
$$

# 55. 可能的核心方法描述

可以将整个方法概括为：

> 本系统首先通过长时域导航—操作联合规划生成包含导航、接近和任务执行阶段的名义任务参考；随后由短时域全身 NMPC 根据末端任务误差、环境距离、机械臂奇异性、关节限位以及控制代价，在线优化差速底盘与机械臂的运动分配。在接触任务阶段，导纳控制根据力反馈在线修正末端任务参考，而 NMPC 负责在满足接触任务的同时实现底盘与机械臂的自适应协调。

其数学形式为：

\arg\min_{u}
\left(
J_{task}
+
\lambda_{obs}J_{collision}
+
\lambda_{sing}J_{singularity}
+
\lambda_{joint}J_{joint-limit}
+
\lambda_uJ_{input}
\right)
$$

其中：

$$
u=
[v,\omega,\dot q]^T
$$

从而实现：

$$
\boxed{
\text{Environment-aware and Configuration-aware Dynamic Base-Arm Allocation}
}
$$

# 56. 最终设计原则

整个 WBMM 后续开发应始终遵循以下原则：

1. `RobotModel` 负责描述机器人，而不是决定控制策略。
2. `wbmm_pinocchio` 负责 FK、Jacobian 等运动学计算。
3. `wbmm_environment` 负责回答环境距离与梯度。
4. `wbmm_collision` 负责机器人与环境之间的碰撞关系。
5. `wbmm_robot_metrics` 负责计算机器人构型质量。
6. `Planner` 负责生成长时域名义任务。
7. `Admittance` 负责根据力反馈修正任务。
8. `OCS2 NMPC` 负责短时域全身最优执行。
9. 不再人为规定固定 `base_share`。
10. Base/Arm 的运动比例应成为优化结果，而不是控制参数。
11. Mode 负责改变“当前任务重点”，而不是硬编码 Base/Arm 比例。
12. Planner 不应把全部 9D 状态锁死，否则会破坏在线动态分配能力。

最终整个系统的核心关系可以浓缩为：

$$
\boxed{
\text{Planner decides WHAT to do}
}
$$

$$
\boxed{
\text{Force Control adjusts WHAT the task should be}
}
$$

$$
\boxed{
\text{NMPC decides HOW the whole body should do it}
}
$$

最终目标：
$$
\boxed{
\text{Navigation-Manipulation Joint Planning}
+
\text{Adaptive Whole-Body Execution}
}
$$