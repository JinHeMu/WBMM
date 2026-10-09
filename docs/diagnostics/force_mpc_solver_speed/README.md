# 本次求解速度对比记录

- before：本次修改前，当前仓库的奇异值过程＋终端配置，并非关闭裕度代价。
- after：相同配置和相同仿真力历史，优化后的算法。
- after_yoshikawa：优化后同时启用 Yoshikawa；作为功能验证，不与 before 视为相同代价配置。
- `*.info` 保存任务配置；`*.json` 是探针结果；`*.jsonl` 是实测状态和预测诊断；`*_launch.log` / `*_probe.log` 保存运行日志。
- `benchmark_before.jsonl` / `benchmark_after.jsonl` 测量单线程完整代价回调。基准源码位于 wbmm_ocs2/test/benchmark_arm_manipulability.cpp。
- `comparison.json` 含原探针 P95、去除重复预测诊断后的 P50/P95、输入配置校验和及基准结果；`comparison.png` 展示相同力历史下的指标和耗时。
- `core_tests.log`、`metrics_tests.log`、`test_results.log` 保存最终验证记录。

详细实现、数值和复现方法见 [速度优化说明](../../force_mpc_solver_speed.md)。
