# 2D Pose Goal 后机械臂逐次下垂：排查与修复

日期：2026-09-29。入口为 `wbmm_tracking.launch.py`，地图为 `maps/map1/site_remani.npz`。测试均在独立 ROS domain 的 MuJoCo 中完成。

## 原因与证据

保持底盘不动，连续四次发送当前位置作为 2D Pose Goal。每次规划参考的首尾关节角相同，未规划低头动作；但实测关节角与 MRT 发送的位置指令存在约 0.01 rad 的静态差值。MuJoCo 使用纯 PD 位置执行器，未补偿机械臂重力载荷。

新目标会取消旧轨迹、保持当前实测状态，并在停稳后用实测关节位置生成规划种子。重力导致的静态下垂因此不断成为下一次保持目标，造成逐次累积。保留实测状态作为规划起点是合理的，修复放在仿真执行器层。

加入执行器前馈后，仍存在约 1.1 mm/次的轻微上移，与现有 MPC 全局关节限位 barrier 的居中偏置一致。按用户要求保留该行为，**未修改 MPC 代码和 `task_esdf_tracking.info` 配置**；尝试过的限位代价实验已全部撤回，相关库已重新构建。

## 最终保留的修改

- `src/sim/tracer_jaka_mujoco/tracer_jaka_mujoco/arm_servo.py`：新增直接位置执行器的重力/科氏力前馈。单位传动比时，`ctrl = q_desired + qfrc_bias / kp`。通过原有执行器施力，保留控制范围和力限幅，不写入 qpos，不补偿接触力。
- `src/sim/tracer_jaka_mujoco/tracer_jaka_mujoco/mujoco_bridge_node.py`：新增可选 `arm_bias_compensation`，默认关闭；启用时校验执行器模型。
- `src/bringup/launch/mujoco_hardware_interface.launch.py`、`wbmm.launch.py`：传递仿真补偿开关，通用入口默认关闭。
- `src/bringup/launch/wbmm_tracking.launch.py`：tracking 仿真入口默认开启 `sim_arm_bias_compensation`。MPC 参数、权重、限位和模式切换逻辑不变。
- `src/sim/tracer_jaka_mujoco/test/test_arm_servo.py`：实际 MuJoCo 动力学下的下垂消除、力/控制限幅、模型兼容性测试。
- `src/bringup/test/arm_goal_hold_check.py`：新增重复目标回归脚本，记录关节、指令、参考和相对底盘的末端高度。

## 测量结果

| 相对底盘的末端高度变化 | 修复前 | 仅执行器补偿 |
|---|---:|---:|
| 第 1 次发送目标 | -19.96 mm | +1.07 mm |
| 第 2 次发送目标 | -18.51 mm | +1.11 mm |
| 第 3 次发送目标 | -17.14 mm | +1.11 mm |
| 第 4 次发送目标 | -15.60 mm | +1.09 mm |
| 四次累计 | -71.21 mm | +4.38 mm |

每次目标均到达 SUCCEEDED。正值表示上移，负值表示下垂。这里比较的是实测 TF 高度，不是规划轨迹高度。数据：[修复前](arm_goal_hold_results/before.json)、[仅补偿](arm_goal_hold_results/compensated.json)。

25 个 Python 检查通过（4 个执行器测试 + 21 个现有 launch 检查）。撤回 MPC 实验并重新构建后，完整模式回归通过：导航位置误差 0.98 mm、原地旋转 yaw 误差 0.00254 rad、末端追踪误差 1.62 mm、返回导航位置误差 0.10 mm。详见 [模式回归](arm_goal_hold_results/mode_regression.json)。

上述结果仅证明当前模型的仿真修复，不涉及真实硬件控制器。执行器前馈依赖模型质量、惯量及直接位置执行器结构；替换模型时仍需验证。补偿只在 tracking 仿真入口默认启用，其他仿真任务需显式选择，以免改变力传感任务既有标定条件。

## 使用

重新启动原命令即可：

```bash
ros2 launch tracer_jaka_bringup wbmm_tracking.launch.py esdf_file:=maps/map1/site_remani.npz
```

重复目标回归：

```bash
ROS_DOMAIN_ID=117 ROS_LOCALHOST_ONLY=1 \
  python3 src/bringup/test/arm_goal_hold_check.py --output /tmp/wbmm_arm_goal_hold
```

复现原始执行器行为可追加 `--record-only --launch-arg sim_arm_bias_compensation:=false`。单纯手工启动时在 launch 命令后追加 `sim_arm_bias_compensation:=false`。
