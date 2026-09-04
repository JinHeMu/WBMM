# WBMM 验证矩阵

> 基线：`test` / `0e9b7e53ea4260fe1191f8a19a42745c038da099`
>
> 规则：没有对应证据的等级一律记为“未验证”；构建或 launch 成功不等于
> MuJoCo/实机闭环完成。

| 场景 | L0 合同 | L1 单测 | L2 离线规划 | L3 MuJoCo 无力 | L4 MuJoCo 接触 | L5 实机只读 | L6 实机运动 |
|---|---|---|---|---|---|---|---|
| TA-WBMP table | 通过 | 通过 | 通过 | 未验证 | 不适用 | 不适用 | 不适用 |
| TA-WBMP blackboard | 通过 | 通过 | 通过 | 未验证 | 未验证 | 不适用 | 不适用 |
| TA-WBMP RAS 0.9 m × 0.6 m | 通过 | 通过 | 通过 | 未验证 | 未验证 | 不适用 | 不适用 |
| REMANI → Coordinator 显式交权 | 通过 | 部分：bridge 1→0→1 | 不适用 | 未验证 | 未验证 | 未验证 | 未验证 |
| localized real dry-run | 通过：默认门禁和拒绝路径 | readiness 工具自检通过 | 不适用 | 不适用 | 不适用 | 未验证 | 未验证 |

截至 2026-08-31，本次证据：四个修改包构建成功；TA-WBMP 7 个 gtest、
WipePlanner 18 个 gtest 全部通过；bridge ownership service 的 target publisher
数量实测为 `1 → 0 → 1`；未显式设置 `safety_release:=true` 时，实机 launch 在
创建硬件/控制节点前 fail-closed。共享 REMANI ESDF adapter 尚未实现，因此
Coordinator 会拒绝生产执行，L3–L6 仍全部是未验证状态。

## 本次 P0 验收命令

```bash
python3 -m py_compile \
  src/bringup/tracer_jaka_bringup/launch/*.launch.py \
  src/bringup/tracer_jaka_bringup/scripts/readiness_check.py \
  src/applications/wiping/wipe_planner/launch/wipe_real*.launch.py

colcon build --symlink-install --packages-select \
  ta_wbmp tracer_jaka_ocs2 tracer_jaka_bringup wipe_planner
colcon test --packages-select ta_wbmp tracer_jaka_bringup wipe_planner
colcon test-result --test-result-base build/ta_wbmp --all
colcon test-result --test-result-base build/wipe_planner --all
git diff --check
```

运行时交权验收还必须保存：

```bash
ros2 topic info /mobile_manipulator_mpc_target --verbose
ros2 run tracer_jaka_bringup readiness_check.py --ros-args \
  -p expected_target_publishers:=1 \
  -p command_output_enabled:=false
```

NAVIGATING 与 TASK_EXEC 两个阶段都必须各自观测到恰好一个 target publisher；
交权中间态允许短暂为零，不允许为二。实机检查只允许只读运行，除非另行完成
L6 风险评审和显式三重放行。
