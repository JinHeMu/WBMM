# 第三方源码与项目定制台账

本文区分可替换的上游内容、项目维护的 fork，以及只负责分发二进制 SDK 的 vendor
包。历史只能证明仓库中的导入和后续提交，无法恢复的信息明确标为“未知”，不把当前
目录误报为未修改的上游快照。

## 1. 当前状态

| 目录 / 包 | 已知来源 | 仓库基线 | 当前定性 |
|---|---|---|---|
| `src/vendor/ocs2_ros2` | `https://github.com/legubiao/ocs2_ros2` | 首次导入提交 `b8f0e4b`；`6e463a7` 移入当前目录 | 上游快照；精确 upstream commit 未记录 |
| `src/vendor/remani_planner` | REMANI-Planner；原仓库 URL 与精确 commit 未记录 | 首次导入 `11f82a8`；`6e463a7` 移入当前目录 | 项目维护 fork，不可直接覆盖更新 |
| `src/vendor/jaka_sdk_vendor` | JAKA SDK 二进制发行物；版本号与下载来源未记录 | 由两个驱动包内的相同副本归一化 | 二进制 vendor 包；不修改 SDK 内容 |

## 2. OCS2 ROS 2

- 上游 README 指向 `legubiao/ocs2_ros2`，其子模块配置指向
  `legubiao/ocs2_robotic_assets`。
- Git 历史显示该源码随 `b8f0e4b` 导入，之后由 `6e463a7` 做目录重构。
- 当前历史没有保存导入时的 upstream commit/tag，因此只能把 `b8f0e4b` 作为本仓库
  比较基线，不能声称与上游某个 SHA 完全一致。
- 此轮没有在 `src/vendor/ocs2_ros2` 内增加功能补丁；WBMM 集成代码继续放在
  `src/algorithms/control/tracer_jaka_ocs2`。

## 3. REMANI-Planner 项目 fork

REMANI 已包含面向 Tracer + JAKA 的模型、地图、重规划、OCS2 桥接和任务执行定制。
Git 中可识别的主要演进提交如下：

| 提交 | 本地改动类别 |
|---|---|
| `11f82a8` | 首次导入 REMANI |
| `2c0ba33` | 移动机械臂配置、grid map、REMANI→OCS2 bridge 与 tracking launch |
| `4260d4d` | 差异化重规划、FSM 与约束参数 |
| `cb2b975`、`ee13e9d`、`905bb79`、`44480fe` | 场景、示例、nvblox 和擦拭任务集成 |
| `6e463a7` | 迁移到 `src/vendor/remani_planner` |
| `ef0845c` 至 `fcd925a` | 实机 MPC、擦拭、碰撞/运动学和任务执行修正 |
| 当前工作树 | `mm_param.yaml` 移除开发机绝对 URDF 路径，改由系统入口注入 |
| 当前工作树 | `plan_env/grid_map.cpp` 强制读取静态 ESDF 的 `frame_id`，并与 `grid_map.frame_id` 精确匹配；缺失或不一致时 fail-closed |

因此升级 REMANI 时必须按 fork 合并，重点人工检查 `mm_config`、`plan_env`、
`path_searching`、`plan_manage`、`traj_opt` 和 `traj_utils`，禁止直接复制新上游目录覆盖。

静态 ESDF 坐标系校验属于 WBMM 的安全补丁。它防止将 `odom` 归档静默标记为
`map` 后参与碰撞检查。回归要求包括：同 frame 归档正常加载、不同 frame 和缺失
`frame_id` 的归档均在规划器初始化阶段拒绝；不得通过修改标签代替地图变换或重导出。

## 4. JAKA SDK

`jaka_sdk_vendor` 是 SDK 唯一所有者，安装：

- `include/jaka_driver/{JAKAZuRobot.h,jkerr.h,jktypes.h}`；
- `lib/libjakaAPI.so`；
- CMake 导出的 `jaka_sdk_vendor_INCLUDE_DIRS` 和 `jaka_sdk_vendor_LIBRARIES`。

`jaka_driver` 与 `jaka_hardware_interface` 通过 `find_package(jaka_sdk_vendor REQUIRED)`
链接，不再保存副本。SDK 版本和校验值尚未从原始发行说明恢复；升级前必须补录厂商
版本、glibc 要求和来源，再在仿真/只读实机/实机运动三个等级分别回归。当前归档物
已确认是 x86-64 ELF，SONAME 为 `libjakaAPI.so`，SHA256 为
`64362541f964460e41495817c679894f55b1878b978528812bc8229862151fa5`；该校验值只标识
当前文件，不能替代厂商版本号。

## 5. 更新流程

1. 在独立分支记录上游 URL、tag/SHA 和许可证，不直接在主工作树覆盖。
2. 用本仓库导入基线列出差异：
   `git log -- <vendor-path>`、`git diff <baseline> -- <vendor-path>`。
3. 将可下沉到 WBMM 集成层的修改移出 vendor；必须留在上游目录的补丁在本文增加
   “文件、原因、本地提交、上游对应 issue/PR、回归测试”。
4. 构建受影响包，运行离线测试，再分别执行 MuJoCo 和只读实机验证。
5. 更新本表的 upstream SHA；若 SHA 不明，发布门保持关闭。
