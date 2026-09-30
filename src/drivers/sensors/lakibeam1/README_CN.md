[README](<https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver/blob/main/README.md>) - English Version of the readme

# 1 关于此驱动

Lakibeam ROS2 Deiver 由锐驰智光（北京）科技有限公司针对LakiBeam1S/LakiBeam1/LakiBeam1L激光雷达开发。启动后，该驱动将监听雷达发送的UDP数据，解析数据并将点云发布到 ROS2 的/scan 或/pcd 话题中。

# 2 环境和依赖关系

系统环境要求：Linux+ROS2
推荐：
Ubuntu 20.04 - with ROS2 foxy fitzroy desktop-full installed 或

Ubuntu 22.04 - with ROS2 humble hawksbill desktop-full installed。

Ubuntu 安装指南请参考 http://ros.org 。

# 3 安装 ROS2 驱动
## 3.1 创建工作空间
```
cd~
mkdir -p catkin_ws/src
```
## 3.2 编译
在 Lakibeam ROS2 Driver 的工作空间中，执行以下指令编译工程:
```
cd catkin_ws/src
git clone https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver.git
colcon build
```

# 4 配置电脑 IP 地址

当通过 RJ45 网线和直流电源连接时，LakiBeam1(L/S)的 IP 地址默认为 192.168.198.2，其目标计算机的 IP 地址为 192.168.198.1。所以我们需要将 PC 的静态 IP 设置为 "192.168.198.1"，子网掩码设置为 "255.255.255.0"，网关地址为非必填项。

当使用 USB Type-C 线缆连接时，LakiBeam1(L/S)的 IP 地址默认为 192.168.8.2，目标计算机的 IP 地址配置为 "192.168.8.1"，PC将识别到一个USB大容量存储设备以及一个RNDIS网络设备（虚拟网卡），此虚拟网卡的的静态 IP 则不需要设置。输入雷达的 IP 地址：192.168.8.2 到 web 浏览器，然后设置 web 服务器中雷达 Host IP 为 "192.168.8.1"，并设置网络模式为 DHCP 模式并保存设置。雷达将在几秒钟延迟后重置网络配置。

雷达的 web 服务器上通过 USB Type-C 线缆连接进行的 IP 配置如下图所示：

![image](https://github.com/RichbeamTechnology/Lakibeam_ROS1_Driver/assets/158011589/09c012cb-5c99-4fb3-996d-7c98fd5fa67b)

# 5 launch 文件

在从雷达接收数据之前，我们应根据需要在 launch 文件中配置参数。可配置的参数如下表所示：
| 参数名称     | 配置说明     | 
| -------- | -------- |
| inverted | 翻转雷达，“true”为被翻转 |
| hostip | 目标 IP 地址，当设置为 0.0.0.0 时，可监听到所有 IP 地址 |
| port | 通过交换机使用双雷达并将数据发送到同一台PC时，监听端口必须与雷达 WebSever 上设置的端口号相同 |
| angle_offset | 点云绕 z 轴的旋转角度，可以设置为负数 |
| scanfreq | 扫描频率，范围：10、20、25、30 |
| filter | 滤波选项，范围：0、1、2、3 |
| laser_enable | 扫描使能，范围：true、false |
| scan_range_start | 扫描起始角度，范围：45°~315° |
| scan_range_stop | 扫描结束角度，范围：45°~315°，结束角度必须大于起始角度 |


# 6 查看实时数据

1. 通过 RJ45 网口和直流电源或 USB Type-C 线将 LakiBeam1(L/S)连接到 PC 上。
2. 驱动内提供了几个标准的launch文件，例如 "lakibeam1_scan.launch.py" 和"lakibeam1_scan_view.launch.py"。要启动 LaserScan 节点，我们可以运行带有 scan 名称的 launch 文件来查看实时点云数据。打开终端：
```
cd ~/catkin_ws
source ./install/setup.bash
ros2 launch lakibeam1 lakibeam1_scan.launch.py
(run LaserScan node)
ros2 launch lakibeam1 lakibeam1_scan_view.launch.py
(run LaserScan node in RViz)
```
RViz 中 运行 LaserScan 节点时的实时点云数据如下图所示：

![image](https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver/blob/main/assets/ros2.png)


## 扫描时间与离线回归

扫描发布和 UDP 解析均使用 C++。有效 MSOP UDP payload 为 1206 字节，
12 个 100 字节块加 6 字节尾部；字节序按厂商示例低字节在前。
方位角单位 0.01°，相邻块角差 200/400 对应 0.125°/0.25°，末包
0xFFFF 标志和方位角是填充块。参见厂商手册 RBDOC-UM-1001 第 7 节：
[协议手册](https://ae-pic-a1.aliexpress-media.com/kf/S21052410e29c40d8b5f73e211f265982d.pdf)。

`scan_assembler.cpp` 跳过启动时不完整的一圈，仅将高角度到低角度的大回绕视为新圈；
小角度回退、重复和乱序包不会产生新圈。丢包位置保留 infinity，不压缩角度。
倒装时使用负 angle_increment，保留采样顺序和递增 time_increment。
header.stamp 对应当前圈第一束，scan_time 为当前圈到下一圈的起点间隔。

内部微秒时间戳通过 uint32 模运算检查重复、乱序并映射到 ROS 时间；首包接收时
减去估算采样跨度作为初始锚点。scanfreq 用作初始扫描频率提示，随后用圈边界
测量实际周期；不主动配置设备时也不要求提示值完全吻合。
该映射不是硬件时钟同步，固定传输延迟和时间戳在固件中的具体采样位置仍需实机核对。
断流超过一秒后的设备时钟重启会重新锚定；主机时钟回退时丢弃旧时间扫描，
直到时间追上已发布扫描，避免重新引入不递增时间。

```bash
colcon build --packages-select lakibeam1 --symlink-install
source install/setup.bash
colcon test --packages-select lakibeam1 --ctest-args -R 'scan_assembler_tests|scan_udp_ros' --output-on-failure
```

C++ 合成包测试覆盖末包填充、跨圈、丢包、重复、乱序、畸形包、时间戳回绕，
另一个 C++ 测试通过本机 UDP 启动实际驱动验证 LaserScan 和无数据时 SIGINT 退出。
