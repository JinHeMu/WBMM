# tracer_jaka_bringup 使用说明

在仓库根目录操作。每个功能入口会一起启动机器人后端、所需算法及 RViz；
`backend:=sim` 使用 MuJoCo，`backend:=real` 使用实机。每次只运行一个功能入口，
切换功能前先按 `Ctrl+C`，无需另外启动硬件接口或算法节点。

## 1. 编译与环境

已安装 ROS 2 Humble 及所需系统依赖后，编译本功能包及仓库内依赖：

```bash
cd ~/WBMM
./deploy/build.sh
source deploy/env/real.env
source /opt/ros/${ROS_DISTRO}/setup.bash
source install/setup.bash
```

只修改 bringup 的启动文件或配置后，可以增量编译：

```bash
colcon build --symlink-install --packages-select tracer_jaka_bringup
source install/setup.bash
```

## 2. 机器人上电与下电

使用实机前：打开底盘和机械臂控制柜电源，释放急停，连接 CAN、网线及传感器。
在 `deploy/env/real.env` 设置机械臂 IP、本机 IP 和 CAN 接口；本机网卡应已配置到
机械臂所在网段。IMU 串口和其他硬件设置在 `config/real/startup.yaml`。

```bash
./deploy/start.sh
```

该脚本配置 CAN，并登录、上电及使能 JAKA；随后在同一个终端选择下面的功能。
实机算法默认关闭控制输出，JAKA 默认只读；需要执行动作时，在启动命令末尾添加
`hardware_write:=true`。仿真默认启用控制输出，不需要运行上电脚本。

停止功能后下电：

```bash
# 先 Ctrl+C 停止 launch，再执行
./deploy/poweroff.sh
```

## 3. 按功能启动

下面以仿真为例；使用实机只需将 `backend:=sim` 改为 `backend:=real`。
需要地图的功能应选择与当前场景对应的文件。

### 硬件与状态

```bash
ros2 launch tracer_jaka_bringup robot_hardware.launch.py backend:=sim
```

启动后端、机器人状态及传感器；适合检查关节、里程计和传感器数据。

### Cartographer 建图

```bash
ros2 launch tracer_jaka_bringup mapping.launch.py backend:=sim save_state_file:=/tmp/site.pbstream
```

启动后端、EKF、Cartographer 和 RViz。驱动机器人扫描场景；正常退出时保存
`.pbstream`，输出目录需已存在，文件名应使用新的名称。

### 保存地图定位

```bash
ros2 launch tracer_jaka_bringup robot_localization.launch.py backend:=sim state_file:=/tmp/site.pbstream
```

启动后端、EKF、Cartographer 纯定位和 RViz。初始位置在对应的
`config/<backend>/startup.yaml` 中设置。若使用 AMCL，在同一配置中将
`localization_backend` 改为 `amcl`，填写 `map_file`，启动时无需 `.pbstream`。

### 全身导航

```bash
ros2 launch tracer_jaka_bringup navigation.launch.py backend:=sim
```

一次启动 MuJoCo、WBMM 规划器、轨迹桥、OCS2 和 RViz；在 RViz 用
**2D Goal Pose** 设置目标。默认是现有 map1 ESDF 的虚拟障碍演示。

实机入口会同时启动保存地图定位。先在 `config/real/startup.yaml` 的
`navigation` 中填写匹配的 `state_file` 和 `esdf_file`，然后：

```bash
ros2 launch tracer_jaka_bringup navigation.launch.py backend:=real hardware_write:=true
```

也可在命令中覆盖这两个文件路径。ESDF 必须声明 `frame_id=map`，并已与定位地图
对齐；同名坐标系并不代表几何上已经对齐。实机不会复用仿真的单位变换，规划器等待
定位就绪后启动。当前实机 OCS2 使用 odom 跟踪，地图碰撞检查由 map 下的规划器负责。

### MoveIt 机械臂规划

```bash
ros2 launch tracer_jaka_bringup arm_moveit.launch.py backend:=sim
```

```bash
ros2 launch tracer_jaka_bringup arm_moveit.launch.py backend:=real hardware_write:=true
```



启动后端、轨迹控制接口、MoveIt 和 RViz。选择 `arm` 规划组后操作
**Plan / Execute**；实机执行时需添加 `hardware_write:=true`。

### OCS2 末端跟踪

```bash
ros2 launch tracer_jaka_bringup end_effector_tracking.launch.py backend:=sim
```

启动后端、OCS2 和末端目标交互工具。在 RViz 调整并提交末端目标；此模式只由
OCS2 控制机械臂，底盘控制输出已隔离。实机执行同样需添加 `hardware_write:=true`。

### 力跟随与 OCS2 联合控制

```bash
ros2 launch tracer_jaka_bringup force_mpc.launch.py backend:=sim fake_wrench:=true keyboard_wrench:=true
```

实机默认只读反馈、导纳关闭：

```bash
ros2 launch tracer_jaka_bringup force_mpc.launch.py backend:=real
```

允许输出的入口为：

```bash
ros2 launch tracer_jaka_bringup force_mpc.launch.py \
  backend:=real hardware_write:=true tare:=true
ros2 service call /whole_body_force_control/enable std_srvs/srv/SetBool '{data: true}'
```

默认三轴平移导纳、K=0 持续拖动；`backend:=real` 默认只读并关闭导纳。
启用、标定和故障恢复见 [联合控制说明](../../docs/force_mpc_integration.md)。

## 4. 修改配置

| 配置 | 用途 |
| --- | --- |
| `config/real/startup.yaml`、`config/sim/startup.yaml` | 硬件、场景、地图路径、初始定位位置及算法配置路径 |
| `config/common/ekf.yaml`、`config/<backend>/ekf.yaml` | EKF 参数 |
| `../map/wbmm_localization/config/cartographer_*.lua`、`readiness.yaml` | Cartographer 与定位就绪条件 |
| `config/<backend>/planner.yaml` | WBMM 规划速度、优化与碰撞参数 |
| `config/common/ocs2.yaml`、`config/sim/ocs2.yaml`、`config/<backend>/task*.info` | OCS2 参数、代价与限位 |
| `../robotics/tracer_jaka_moveit_config/config/` | MoveIt 规划器、运动学与控制器配置 |
| `config/common/force_mpc.yaml`、`config/real/force_mpc_calibration.yaml` | 联合力控参数与实机标定 |

启动时通常只设置 `backend`、地图路径以及实机的 `hardware_write`。
无显示环境可添加 `use_rviz:=false viewer:=false`；需要另一套部署配置可使用
`config_file:=/绝对路径/startup.yaml`。数值调参直接修改上述配置；规划器配置中的
数值优先于旧算法入口的同名数值参数。底层与旧实验入口仍保留以兼容现有调用。
