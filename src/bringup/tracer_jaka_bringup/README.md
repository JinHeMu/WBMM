# tracer_jaka_bringup

WBMM 的顶层系统组合包，只负责 launch、部署 profile、RViz 和运行检查。

当前状态：**目录/包骨架**，完整组合 launch 尚未创建。规划中的入口如下（待实现后再使用）：

```bash
ros2 launch tracer_jaka_bringup wipe_sim.launch.py
ros2 launch tracer_jaka_bringup wipe_commissioning.launch.py
ros2 launch tracer_jaka_bringup wipe_production.launch.py
```

现阶段实际可用的仿真/实机入口仍位于各功能包中，速查见 [docs/QUICKSTART.md](../../docs/QUICKSTART.md)。
