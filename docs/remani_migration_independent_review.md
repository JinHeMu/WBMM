# REMANI → WBMM 迁移独立复核与修复记录

日期：2026-09-24  
审阅对象：`docs/remani_migration_review.md` 及当前 `/home/a/WBMM` 工作区  
状态：已完成源码复核与本轮修复；只通过下述离线与 ROS 参考链验证，未作真机放行。

## 1. 结论

分包方向合理：保留 ROS-free 搜索、轨迹整形、规划编排，ROS 节点负责消息、地图/模型加载与 TF，参考桥独立于 OCS2 求解器。这一结构值得保留。

但当前实现是一个重新设计的、静态地图下的静止到静止导航规划器，不能称为 REMANI 功能等价迁移。主链没有调用 `WholeBodyOptimizer`；现有单向逐点 Halton 臂种子、固定时长 MINCO 整形，也不等价于原来的机械臂 RRT* 与联合轨迹优化。

本次优先修复会导致错误参考、碰撞漏检或接口错接的缺陷，没有将尚未验证的优化器接入运行链。

## 2. 已修复的主要问题

| 优先级 | 原实现的问题与影响 | 本轮改动 | 证据 |
| --- | --- | --- | --- |
| P1 | 搜索和臂种子检查通过后，MINCO 整形、抽稀及航向重建改变几何，却没有最终碰撞复核 | 发布前检查最终样本与桥实际使用的线性插值；检查关节位置限位、终点位置/航向；碰撞回调错误直接拒绝 | 新增障碍位于种子检查间隙的回归，原始种子能通过而最终轨迹必须拒绝 |
| P1 | 弧长弦向独立平滑 yaw，通常不满足差速关系 `xdot=v cos(yaw), ydot=v sin(yaw)`；较小 ω 不能证明参考可执行 | 移动时采用真实速度切向；静止开头使用第一个有效切向，之后保持上一有效切向；保留航向跳变与速度包络检查 | 新增弯曲 MINCO 轨迹测试，对 x/y 解析速度逐点验证，误差界 1e-8 |
| P1 | 直线末尾短段受最小时长限制时，MINCO 会过冲反向；时间缩放无法修正这种几何错误 | 删除 `[x,y,q]` 空间中的冗余共线控制点，再进行整形和最终碰撞检查 | TF 级别浮点扰动使 0.8 m 路径出现 0.05 m 尾段，修复前航向跳变 π；新增短尾段回归通过 |
| P1 | 固定 8 维 MINCO 接收非六关节请求会触发 Eigen 维度错误 | 在整形前验证六关节结构、唯一名称、有限值、位置范围及速度限制；验证时间参数和采样预算 | 非六关节、NaN 时长测试；请求速度限制参与实际输出包络 |
| P1 | planner 不检查 odom 的 frame，直接把坐标解释成 ESDF frame | 对 odom 与 goal 进行带消息时间戳的 TF 变换；缺失 frame/TF、非法四元数直接拒绝 | 独立 ROS 域中验证 odom → map 的平移与 90° 旋转 |
| P1 | bridge 自动采纳 planner frame，却无法证明与 OCS2 状态坐标系相同 | 显式配置 `bridge_world_frame`；每个参考窗口用 TF 从规划坐标系变换到控制坐标系；缺 TF 不发布 | map → odom 含平移、90° 旋转实测；缺 TF 时无参考输出 |
| P1 | 两个组合 launch 把规划地图误接到可为空的 OCS2 `esdf_file` | planner 使用 `static_esdf_file`；OCS2 独立保留 `esdf_file` | 新测试用非空规划地图和空 OCS2 地图实际解析转发参数 |
| P1 | 局部化入口声明 `start_bridge` / `start_ocs2` 却没有完整执行开关；真实入口继承模拟速度包络 | 落实启动条件；真实组合入口参考上限改为底盘 0.1 m/s、0.4 rad/s、关节 0.15 rad/s | 启动参数与 Node 条件回归；未启动硬件 |
| P1 | 过期状态可继续规划，过期 observation 可无限外推时钟；释放所有权后会继续旧轨迹 | 新增状态/观测超时；观测失效或 MPC 时间回退使旧轨迹失效；释放参考权清除旧计划；恢复后需要新计划 | ROS 测试：状态过期不规划、观测过期不发布、恢复观测/所有权不恢复旧计划 |
| P1 | bridge 只比较关节个数，错序名称会静默下发错误维度语义 | 检查完整关节顺序；拒绝重复/空名称与非零时间起点 | 消息单测与错序关节 ROS 测试 |
| P2 | 到轨迹范围外只钳制位姿，仍保留非零末端速度 | 超出轨迹区间时保持位姿并清零全部前馈速度 | 使用非零末端速度的回归测试 |
| P1 | 实验优化器碰撞扫描失败仍设 `success=true`；扫描遇到无法查询的点可能忽略 | 碰撞扫描失败不返回成功或候选轨迹；无效/越界距离查询拒绝；初始航向采用配置值 | 全占据 ESDF 回归，避免依赖某个现场点是否碰撞 |

最终轨迹检查默认平移步长 0.02 m、角度/关节步长 0.03 rad，最多 100000 次检查。它是有界离散验证，不是连续碰撞证明。检查器覆盖范围由注入的碰撞回调决定；当前 ROS 回调主要是机器人球模型与 ESDF 的环境碰撞。

## 3. 对原评审文档的纠正

1. **第 4.2 节的弧长航向不能用峰值角速度下降证明正确。** 移动底盘的 yaw 必须与位置轨迹导数满足非完整约束。仅把 yaw 平滑而不重构 x/y，会得到有侧向运动残差的参考。本轮恢复真实切向，而非把弧长航向照搬进优化器。
2. **第 5.5 节的上游时间采样疑似 bug 不成立。** 本地 vendor 的 `poly_traj_optimizer.cpp::addPVAJGradCost2CT()` 设置 `step=T/K`，循环末尾执行 `s1 += step`；因此采样范围为 `[0,T]`。`s1` 附近“归一化时间”注释有误，不能据此认定只采样 `[0,1]`。
3. **第 5.4 节“只检查收敛性”与当前源码不符。** `MatchesFiniteDifferences` 除了 `fine < coarse`，还有 `fine < 2e-2`。但 y 分量被跳过，另一个测试允许误差到 4.0，因此仍不能声称所有梯度通过正确性验收。
4. **Halton 的确定性只保证复现，不保证成功或完备性。** 逐点贪心选臂姿态没有回溯，早期选择可能把后续路径堵死。
5. **参考消息到达不等于整机闭环成功。** 原 76 点检查主要证明消息、时间与维数合法，无法证明 MPC 跟踪、控制输出限幅、模式切换、接触或真实机器人行为。
6. 原文关于“乱飞根因”和特定 CGAL 构建失败来自前一个 agent 的历史证据，本次没有重新执行该场景或全工作区构建，不将其作为本轮确认结论。

## 4. 保留的设计与尚未完成的能力

| 项目 | 当前真实状态 | 建议与验收门槛 |
| --- | --- | --- |
| 采样轨迹契约 | 合理，解耦控制器与 MINCO；时间起点、名称顺序、frame 需要严格约束 | 保留；后续补充计划替换、开始确认与取消协议 |
| 优化器接入 | 未接入；时长不在决策变量；横向梯度仍未验收 | 先在充分远离奇异点且所有相关代价激活的算例中逐分量检查；再解决静止航向与导数的一致定义；最后接到与现有整形相同的硬验收出口 |
| 优化器 `safe` | 只是离散环境碰撞诊断，不是控制包络、连续碰撞或全部自碰撞的证明 | 不把 `success` / `safe` 当成直接发布许可；仍需外部完整硬检查。本轮已更正接口说明 |
| 自碰撞与地面 | 当前主链 EnvironmentCollisionChecker 没有完整自碰撞硬检查；实验优化器存在相关软代价 | 定义允许碰撞对/相邻链排除规则，然后做独立硬检查；不能直接把所有球对相交当成碰撞 |
| 地图未知区 | 保留原来的可配置开放策略，map1 的未知区数据统计不能推广到其他地图 | 每个部署确认 unknown 策略、地图边界与碰撞模型覆盖；未擅自改变既有地图策略 |
| 模型几何 | 当前导入器主要使用 URDF sphere；不能认为 mesh/box/cylinder 自动被完整覆盖 | 更换模型时需要显式球近似或扩展几何支持及覆盖检查 |
| 全身目标 | ROS 入口仍主要是 2D goal；臂种子寻找可行姿态，没有指定终端关节姿态的完整任务契约 | 下一阶段通过现有 core request 扩展全身 goal 与终端约束，避免重新绑定 vendor 消息 |
| 前进/倒车切换及原地转向 | 混合挡位不支持；单段 x/y 平坦输出不能独立表达静止旋转 | 在轨迹契约中显式表达分段停止、转向、换挡；本轮不伪造可行输出 |
| 运行状态机 | 新节点同步处理目标，缺少 REMANI 式在线重规划、失败恢复与执行确认；本轮明确静止到静止边界 | 增加目标接受/规划/执行/取消/故障状态，并验证移动中重规划边界连续性与参考所有权 |
| TF 与采样参考 | 已补变换；动态定位跳变、TF 新鲜度、分段线性插值的动态一致性尚未完整验收 | 做非平凡 map→odom、定位修正、跟踪误差与撤销测试；不要把当前单窗口变换视为完整执行协议 |

接下来最有价值的顺序是：补齐硬碰撞与执行契约 → 做带障碍的 MuJoCo 跟踪/停止回归 → 修复优化器全梯度并比较规划质量 → 完整全身目标与分段换挡。暂时扩大优化权重或直接接入优化器不能替代这些条件。

## 5. 验证记录

- 受影响的五个包构建通过：`wbmm_traj_opt`、`wbmm_planner`、`wbmm_planner_ros`、`wbmm_reference_bridge`、`tracer_jaka_bringup`。
- 原十二组单元测试加本轮回归，共 **120 项**；具体结果以随附 `verification.txt` 为准。
- bringup 测试 **35 项通过**，包含新增的地图参数、坐标系分离和 bridge 开关检查。
- 独立 ROS 域中的 planner → bridge 回归通过；真实 ESDF/URDF、合成状态和 MPC observation，测试不启动 MPC/MRT、MuJoCo 或任何硬件驱动。
- 未执行全工作区构建、MuJoCo 闭环或真机实验；不代表完整任务流程与硬件可用性验收。

复现构建与启动测试：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select wbmm_traj_opt wbmm_planner wbmm_planner_ros wbmm_reference_bridge tracer_jaka_bringup --parallel-workers 2
python3 -m pytest src/bringup/test/ -q -p no:anyio -p no:cacheprovider
```

复现 ROS 回归（脚本会启动并回收自己的两个节点，在当前目录写测试日志）：

```bash
mkdir -p /tmp/wbmm_migration_regression
cd /tmp/wbmm_migration_regression
source /opt/ros/humble/setup.bash
source /home/a/WBMM/install/setup.bash
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/wbmm_migration_regression/roslog
python3 /home/a/WBMM/src/bringup/test/planning_migration_regression.py   /home/a/WBMM/src/robotics/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf   /home/a/WBMM/maps/map1/site_remani.npz
```

普通启动新增参数：`world_frame` 是规划 frame（可留空从 ESDF 读取），`bridge_world_frame` 必须等于 OCS2 的控制 frame（默认 `odom`），`start_bridge:=false` 禁用参考桥。不同 frame 需要 TF。原纯消息 smoke test 若统一使用 map，应显式传 `bridge_world_frame:=map`。

## 6. 变更范围

本轮在既有未提交迁移工作上追加修复，没有提交或推送，没有删除日志或还原 vendor 改动。只修改规划、参考桥、相应 launch/test 与复核文档；没有更改既有 MRT、MuJoCo 和硬件写入开关。

附带的 `remani_review_changes.patch` 相对本轮开始时保存的文件生成，只包含本轮增量，不把前一个 agent 的全部迁移算成本轮修改。
