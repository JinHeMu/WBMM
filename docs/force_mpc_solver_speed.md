# Force-MPC 求解速度优化

2026-10-09。保留当前 task.info、奇异值／操作度定义、过程／终端权重和控制链路，仅优化计算。

## 定位结果

DeepSeek 指出的重复计算是主要结构问题：原近似需要当前构型加六个关节的正负扰动，共 13 次运动学和完整指标评价。公共评价还计算了不参与当前代价的逆矩阵和关节限位诊断。

此外，实际构建中 wbmm_robot_metrics 的编译选项只有 `-Wall -Wextra -Wpedantic`，没有优化级别。它包含 Eigen 分解计算，原控制包即使使用 Release，也无法优化已经编译好的这部分代码。

“201 个节点”不能作为每轮固定的计算量。当前 rollout 使用 ODE45 自适应积分，timeStep=0.01 是积分初始步长，实际轨迹节点数随积分结果变化；ddp.timeStep=0.001 用于反向积分。没有通过增大步长、降低裕度阈值或缩短 2 秒预测窗口来提速，也没有新增执行 gate。

## 实现

`wbmm_ocs2/cost/ArmManipulabilityCost.cpp` 中求解器采用一次运动学评价：

1. computeJointJacobians(q) 已更新关节放置，随后只更新目标 TCP frame；省去重复 FK 和全部 frame 更新。
2. Pinocchio WORLD kinematic Hessian 提供空间 Jacobian 的导数。将它与 TCP 点速度一起转换到 TCP 原点、世界轴向。WBMM 根关节是 [PX,PY,RZ] composite；世界平移两列是常量，其坐标导数显式为零。
3. 对选定的 arm／whole-body、translation／pose Jacobian 及其导数应用同一列映射和角速度行缩放。全身输入映射还包含底盘航向对 cos(yaw)、sin(yaw) 的导数。
4. 普通构型只做一次 SVD，使用 `d_sigma_i = u_i^T dJ v_i`。Yoshikawa 用乘积求导；条件数包含分母 floor 的分支；方向指标按固定世界方向求导。启用逆操作度时，使用 `sum 1/(sigma_i^2+regularization)`，避免构建逆矩阵。
5. 零／重合奇异值处没有唯一的 SVD 向量导数，使用 `J +/- finiteDiffStep*dJ` 的中心差分。这个分支可能多做小矩阵 SVD，但始终只有一次运动学计算。它是数值导数处理，不是运行时保护开关。

梯度仍用于原有 hinge／linear cost，二次近似仍为 PSD Gauss–Newton，保留 hessianRegularization。完整 `computeMetrics()` 诊断保持公共接口和字段。各代价克隆独立拥有工作区。

`wbmm_robot_metrics` 默认采用 Release，显式 Debug／RelWithDebInfo 仍优先；公共诊断 SVD 不再构建未使用的 U/V。没有新增包依赖、task 参数或 YAML 参数。

## 验证结果

使用完整 `force_mpc.launch.py backend:=sim fake_wrench:=true keyboard_wrench:=true`，保留 MuJoCo viewer，关闭 RViz。每组重新启动；按仿真时钟执行零力 3 秒、TCP -Z 施力 8 秒、释放 4 秒、反向施力 6 秒、释放 4 秒。before 和 after 的 task.info 字节完全一致，均开启奇异值过程和终端代价。

| 指标 | 优化前 | 优化后 |
|---|---:|---:|
| solve_ms P95，沿用原探针统计 | 31.675 ms | 3.421 ms |
| 去除重复诊断消息后的求解 P50 | 25.062 ms | 2.455 ms |
| 去除重复诊断消息后的求解 P95 | 31.748 ms | 3.409 ms |
| 实测机械臂最小奇异值 | 0.042881 | 0.043764 |
| 实测最小 Yoshikawa 操作度 | 0.0004576 | 0.0004602 |
| TCP 跟踪误差 RMS | 32.158 mm | 33.198 mm |
| 最大 TCP 跟踪误差 | 61.775 mm | 63.653 mm |
| 控制故障／意外碰撞 | 无 | 无 |

P95 耗时降低 89.2%，约 9.26 倍。裕度保持相近，跟踪误差 RMS 增加约 1.04 mm；优化提高求解更新频率，两次独立仿真的姿态轨迹并不逐点相同。本轮不以跟踪精度提升作为结论。solve_ms 覆盖同步求解段，不包括后续预测指标诊断，也不是端到端控制延迟。

单线程固定构型基准测量完整代价回调，先预热，再统计 25 个批次，每批 800 次，循环八种构型：

| 回调中位数 | 优化前 | 优化后 |
|---|---:|---:|
| 奇异值代价值 | 155.96 us | 4.70 us |
| 奇异值二次近似 | 2014.15 us | 8.04 us |
| 奇异值＋Yoshikawa 二次近似 | 2006.51 us | 8.44 us |

该基准反映代价计算和构建优化的合计收益，不等于整个 MPC 的加速倍数。

优化后同时开启奇异值与 Yoshikawa 的独立完整仿真也通过：求解 P95 为 3.628 ms，实测最小奇异值为 0.043690，未出现控制故障或意外碰撞。这组增加了代价项，属于功能验证。

7 项公共指标测试、27 项 OCS2 核心测试和 10 项 ROS 控制测试全部通过。验证包括解析梯度／PSD 二次近似与独立完整指标中心差分的对照。对照覆盖两种 scope、两种任务维度、两种缩放、TCP 偏移、固定世界方向、条件数 floor、逆操作度、零奇异值和并发克隆。仿真证据不包含实机运行。

## 构建与复现

```bash
source /opt/ros/humble/setup.bash
source /home/a/WBMM/install/setup.bash
colcon build --packages-select wbmm_robot_metrics wbmm_ocs2 wbmm_ocs2_ros \
  --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select wbmm_robot_metrics wbmm_ocs2 wbmm_ocs2_ros

build/wbmm_ocs2/benchmark_arm_manipulability \
  src/robotics/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf

/usr/bin/python3 -s src/bringup/test/run_force_mpc_arm_margin_ablation.py \
  --directory docs/diagnostics/force_mpc_solver_speed --cases after --domain 82
```

复现仿真会更新对应 after 记录。当前源码已优化，重跑 before.info 也会使用优化后的算法；原 before 耗时是修改前采集的记录。原始配置、逐帧数据、统计和图在 [诊断目录](diagnostics/force_mpc_solver_speed/README.md)。常用参数不需要调整；finiteDiffStep 现在只控制不可微谱分支，hessianRegularization 继续控制二次近似的对角正则。
