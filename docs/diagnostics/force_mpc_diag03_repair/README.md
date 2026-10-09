# diag03 修复验证产物

参见 [完整报告](../../force_mpc_diag03_repair.md)。所有执行均为离线求解或模拟节点，没有连接实机。

- `summary.json`：离线、MuJoCo、驱动和联锁结果汇总。
- `base_response_comparison.png`：完整 MuJoCo 同输入对照。
- `previous/ideal/response.info`：原参考/降低参考/响应模型三个任务快照；底盘权重一致。
- `initial.csv`：diag03 初始几何状态，附原始 EE 目标（离线闭环另行计算 FK 起始目标）。
- `*_closed_loop.csv`：22 s 离线闭环结果，`response_delay150` 的底盘输入延迟增大 50%。
- `headless/`：完整同输入 MuJoCo 对照 JSON、压缩逐样本 JSONL、launch/probe 日志。
- `interlock/`：最终响应模型下的力反馈中断和显式恢复验证。
- `keyboard_attempt/`：保留首次键盘焦点输入失败记录，未用于有效对照。
- `base_aligned.csv`、`fit_first_order.py`、`effective_first_order_fit.json`：只读包导出的速度数据和有效一阶拟合。
- `base_dynamics_fit_with_delay.json`：前轮惯性＋延迟拟合，仿真底盘参数来源。
- `driver_scheduling.*`、`check_driver_scheduling.py`：无 CAN、1 Hz 状态发布 / 125 Hz 指令的独立驱动检查。
- `*.log.gz`：编译和回归日志。

从工作区根目录 source ROS 和 install 后，可重现离线比较（`simulate_base_response` 位于 build 中，无 ROS 初始化）：

```bash
build/wbmm_ocs2_ros/simulate_base_response \
  docs/diagnostics/force_mpc_diag03_repair/response.info \
  install/tracer_jaka_description/share/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf \
  docs/diagnostics/force_mpc_diag03_repair/initial.csv \
  /tmp/response_closed_loop.csv /tmp/response_test_libraries 1
```

完整仿真对照应使用新的输出目录，将 `ideal.info`、`response.info` 复制到其中，然后执行：

```bash
/usr/bin/python3 -s src/bringup/test/run_force_mpc_arm_margin_ablation.py \
  --directory /tmp/new_diag03_sim_comparison --cases ideal,response \
  --sequence 'w:5,zero:3,s:5,zero:5' --domain 82 \
  --base-response-config "$PWD/src/sim/tracer_jaka_mujoco/config/base_response_diag03.yaml" \
  --direct-force --force-n 5
```

改为 `--cases response --integration-probe` 可执行完整联锁回归。仿真脚本固定 `backend:=sim fake_wrench:=true`。不要在实机 ROS 域运行测试。
