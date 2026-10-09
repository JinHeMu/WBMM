# diag02 修复验证记录

完整解释见 [修复报告](../../force_mpc_diag02_repair.md)，汇总见 [summary.json](summary.json)。

- `diag02_input.csv`：长包 72–75.48 s 的 218 个降采样 observation 和最近 EE target。首列是录制相对时间，后续为 state[9]、target[7]。
- `original.info`：诊断任务快照；`original_wrapped.csv`、`original_continuous.csv`、`repaired_continuous.csv` 为三种离线求解输出。
- `sigma_010/020/050.info` 与对应 JSON：相同 5 N 仿真输入的权重比较。`sigma_010_force10.json` 保留 10 N 输入触发跟踪保护的失败记录。
- `tf_timing.json`：20/125/250 Hz 发布上限的合成输入时序对照。
- 三份 `*_tests.log`：回归记录。详细逐帧仿真记录和启动日志还在本次会话的 `/tmp/wbmm_force_repair`。

复现离线求解（没有 ROS 初始化或命令发布）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select wbmm_ocs2_ros --cmake-args -DBUILD_TESTING=ON
build/wbmm_ocs2_ros/replay_force_mpc \
  src/bringup/config/real/task_force_mpc.info \
  src/robotics/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf \
  docs/diagnostics/force_mpc_diag02_repair/diag02_input.csv \
  /tmp/repaired_replay.csv continuous /tmp/wbmm_replay_libraries
```

复现无界面仿真和合成输入 TF 对照（脚本固定使用独立的 ROS 域 97/98，不启动实机驱动；需要本机 ROS 通信权限，会覆盖目录中的对应结果）：

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -s docs/diagnostics/force_mpc_diag02_repair/run_sim.py
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -s docs/diagnostics/force_mpc_diag02_repair/tf_timing_probe.py
```

`virtual_probe.py` 用合成输入替代 X11 按键，沿用仓库的力控观测探针；5 N 输入每 20 ms 发布一次。测试脚本只写 virtual wrench command，运动由隔离仿真中的既有控制链路产生。
