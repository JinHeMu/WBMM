# diag04 离线审查资料

结论见 [完整报告](../../force_mpc_diag04_review.md)。本目录仅增加审查资料，没有控制代码或运行参数修改。

- `bag_metadata.yaml`：原包 `/tmp/wbmm_force_diag_04` 元数据。
- `params/`：`/tmp/wbmm_force_diag_04.params` 的原始参数快照。
- `summary.json`：话题计数、状态、日志、TF/力/策略时序与初步窗口统计。
- `details.json`：正常与故障窗口、模型轨迹回归、驱动时序、描述性震荡比较及停止过程。
- `followup.json`：最后两段施力与目标/实际速度对比、故障后保持指令恒定性。力模长三个值依次为 P50/P95/max；速度用 0.2 s 居中位置差分，排除窗口两端各 0.1 s。
- `overview.png`、`fault_detail.png`：全程和最后持续牵引阶段曲线。故障后的误差骤降来自参考切换为保持位姿。
- 三个 Python 文件：生成本次统计与图的离线脚本。解析器以 SQLite 只读模式打开包，不初始化 ROS 节点。

脚本保留本次运行所用的本机路径，原始包和较大数组没有复制进仓库。实际分析工作目录为 `/tmp/wbmm_force_diag04_analysis`，其中保留 `series.npz`、`driver_series.npz`。若临时目录被清理，在 `/home/a/WBMM` 中运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p /tmp/wbmm_force_diag04_analysis
cp docs/diagnostics/force_mpc_diag04_review/*.py /tmp/wbmm_force_diag04_analysis/
/usr/bin/python3 -s /tmp/wbmm_force_diag04_analysis/detail_diag04.py > /tmp/wbmm_force_diag04_analysis/detail.log
/usr/bin/python3 -s /tmp/wbmm_force_diag04_analysis/plot_review.py
```

`detail_diag04.py` 还读取 `/tmp/wbmm_force_diag03_analysis/series.npz` 进行历史对比；重跑需要该文件或先去掉历史比较循环。`followup.json` 是另行计算的补充统计，不由这两个命令重建。

奇异值采用当前 URDF 的六关节手臂 Jacobian，角速度部分乘 0.30 后计算最小奇异值；与本次任务的尺度一致。末端运动学由 observation 的底盘位姿和六关节位置重建。统计按录包接收时间对齐异步话题，临界瞬间以控制器自身的错误日志为准。图上故障时间取首次 FAULT 状态的录包接收时间 104.207583 s。

`details.json.stop.hold_command_max_change_rad` 描述最早保持指令至后续指令的变化，包含进入锁存目标时的限速过渡；不能解释为持续漂移。补充统计确认故障后 0.1 s 起保持指令完全不变。
