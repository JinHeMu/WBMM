/**
 * @file mm_config.cpp
 * @brief 移动机械臂运动学模型与碰撞检测实现 — REMANI-Planner 的"数字孪生"
 *
 * ============================================================
 * 核心职责: 封装移动机械臂的物理模型, 提供运动学计算与安全检测
 * ============================================================
 *
 * 三大功能模块:
 *   1. 运动学模型:
 *      - getAJointTran(): 单关节正运动学 (FastArmer / UR5 两种机械臂)
 *      - getJointTrans(): 全关节链式变换 (根→末端)
 *      - CarState2T():    基底平面位姿 → 齐次变换矩阵
 *      - calYaw()/calR(): 速度方向 → yaw角/旋转矩阵
 *
 *   2. 碰撞几何 (球体近似):
 *      - setLinkPoint():     为每个连杆生成采样球心点 (硬件编码的碰撞代理球)
 *      - getCarPts():        移动基底边界框 → 碰撞检测球心点集
 *      - getCarPtsGrad():    同上 + 对速度的雅可比 (用于优化梯度)
 *      - getCarPtsGradNew(): 同上 + 对yaw的梯度
 *
 *   3. 4 层碰撞检测:
 *      Layer 1: checkCarObsCollision()   — 底盘 ↔ 障碍物
 *      Layer 2: checkManiObsCollision()  — 机械臂 ↔ 障碍物
 *      Layer 3: checkCarManiCollision()  — 底盘 ↔ 机械臂 (自碰撞)
 *      Layer 4: checkManiManiCollision() — 机械臂 ↔ 机械臂 (自碰撞)
 *
 *   4. RViz 可视化:
 *      - visMM():        碰撞球 + 网格模型
 *      - visMMCheckBall(): 仅碰撞球
 *      - 支持 FastArmer / UR5 两种机械臂 mesh 渲染
 *
 * 支持的机器人配置:
 *   - 移动基底: 矩形差速轮底盘
 *   - 机械臂 FastArmer: 6-DOF 自定义臂 (useFastArmer_ = true)
 *   - 机械臂 UR5:       6-DOF Universal Robots (useFastArmer_ = false)
 *
 * 调用方:
 *   - PolyTrajOptimizer::obstacleGradCostforMM() ← 轨迹优化的碰撞代价
 *   - PlannerManager::getLocalTarget()           ← 局部目标碰撞检查
 *   - REMANIReplanFSM::checkCollisionCallback()  ← 实时碰撞监控
 *   - KinoAstar / RrtPlanning                    ← 前端路径搜索
 *
 * @see mm_config.hpp (类定义 + MMState 结构体)
 * @see poly_traj_optimizer.cpp (主要碰撞代价调用方)
 */

#include "mm_config/mm_config.hpp"

namespace remani_planner
{

/**
 * @brief 带地图引用的参数设置（重载版本）
 * @param nh  ROS 节点句柄，用于读取参数服务器配置
 * @param env GridMap 共享指针，提供 ESDF 碰撞查询能力
 *
 * 调用 setParam(nh) 前先绑定 grid_map_ 指针,
 * 使后续的碰撞检测函数可以直接查询 ESDF 距离值.
 */
void MMConfig::setParam(rclcpp::Node::SharedPtr node, const std::shared_ptr<GridMap>& env){
    grid_map_ = env; node_ = node;
    setParam(node);
}

/**
 * @brief 从 ROS 参数服务器加载全部机器人模型参数
 * @param nh ROS 节点句柄
 *
 * 加载的参数分为 6 类:
 *   1. 底盘几何:       长度/宽度/高度/碰撞检测球半径
 *   2. 底盘运动学:     轴距/轮半径/最大轮速/最大轮角加速度
 *   3. 机械臂配置:     DOF/关节厚度/关节限位/基准位姿
 *   4. 安全边界:       底盘安全边距/机械臂安全边距/自碰撞边距/地面安全距离
 *   5. 网格模型路径:   FastArmer + UR5 各连杆的 STL/DAE 文件 URI
 *   6. 可视化:         色板 / 显示索引
 *
 * 关键计算:
 *   - mobile_base_max_vel_ = max_wheel_omega * wheel_radius (近似最大线速度)
 *   - mobile_base_max_acc_ = max_wheel_alpha * wheel_radius (近似最大线加速度)
 *   - T_q_0_: 移动基底 → 机械臂基座的固定坐标变换
 *   - B_h_:   B_h_ = [0 -1; 1 0] — 旋转 90° 的反对称矩阵 (用于 yaw 梯度计算)
 */
void MMConfig::setParam(rclcpp::Node::SharedPtr node){
    node_ = node;

    // Helper macro for declaring and reading parameters
    #define DECLARE_GET_PARAM(type, key, var, default_val) \
      if (!node_->has_parameter(key)) node_->declare_parameter<type>(key, default_val); \
      var = node_->get_parameter(key).as_##type()

    DECLARE_GET_PARAM(int, "mm.mobile_base_dof", mobile_base_dof_, -1);
    DECLARE_GET_PARAM(double, "mm.mobile_base_length", mobile_base_length_, -1.0);
    DECLARE_GET_PARAM(double, "mm.mobile_base_width", mobile_base_width_, -1.0);
    DECLARE_GET_PARAM(double, "mm.mobile_base_height", mobile_base_height_, -1.0);
    DECLARE_GET_PARAM(double, "mm.mobile_base_check_radius", mobile_base_check_radius_, -1.0);

    DECLARE_GET_PARAM(double, "mm.mobile_base_wheel_base", mobile_base_wheel_base_, -1.0);
    DECLARE_GET_PARAM(double, "mm.mobile_base_wheel_radius", mobile_base_wheel_radius_, -1.0);
    DECLARE_GET_PARAM(double, "mm.mobile_base_max_wheel_omega", mobile_base_max_wheel_omega_, -1.0);
    DECLARE_GET_PARAM(double, "mm.mobile_base_max_wheel_alpha", mobile_base_max_wheel_alpha_, -1.0);
    mobile_base_max_vel_ = mobile_base_max_wheel_omega_ * mobile_base_wheel_radius_;
    mobile_base_max_acc_ = mobile_base_max_wheel_alpha_ * mobile_base_wheel_radius_;

    DECLARE_GET_PARAM(int, "mm.manipulator_dof", manipulator_dof_, -1);
    DECLARE_GET_PARAM(double, "mm.manipulator_thickness", manipulator_thickness_, -1.0);

    DECLARE_GET_PARAM(double, "grid_map.resolution", map_resolution_, 0.05);

    DECLARE_GET_PARAM(double, "optimization.safe_margin", car_safe_margin_, -1.0);
    DECLARE_GET_PARAM(double, "optimization.safe_margin_mani", mani_safe_margin_, -1.0);
    DECLARE_GET_PARAM(double, "optimization.self_safe_margin", self_safe_margin_, -1.0);
    DECLARE_GET_PARAM(double, "optimization.ground_safe_dis", ground_safe_dis_, 0.1);

    manipulator_min_pos_.resize(manipulator_dof_);
    manipulator_max_pos_.resize(manipulator_dof_);
    if (!node_->has_parameter("mm.manipulator_min_pos"))
      node_->declare_parameter<std::vector<double>>("mm.manipulator_min_pos", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::vector<double> pos_limit = node_->get_parameter("mm.manipulator_min_pos").as_double_array();
    for(int i = 0; i < manipulator_dof_; i++){
        manipulator_min_pos_(i) = pos_limit[i];
    }
    if (!node_->has_parameter("mm.manipulator_max_pos"))
      node_->declare_parameter<std::vector<double>>("mm.manipulator_max_pos", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    pos_limit = node_->get_parameter("mm.manipulator_max_pos").as_double_array();
    for(int i = 0; i < manipulator_dof_; i++){
        manipulator_max_pos_(i) = pos_limit[i];
    }

    if (!node_->has_parameter("mm.manipulator_config"))
      node_->declare_parameter<std::vector<double>>("mm.manipulator_config", std::vector<double>{});
    std::vector<double> manipulator_config = node_->get_parameter("mm.manipulator_config").as_double_array();
    manipulator_config_.resize(manipulator_config.size());
    for(int i = 0; i < manipulator_config.size(); i++){
        manipulator_config_(i) = manipulator_config[i];
    }

    if (!node_->has_parameter("mm.base_mani_fixed_joint_xyz_ypr"))
      node_->declare_parameter<std::vector<double>>("mm.base_mani_fixed_joint_xyz_ypr", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::vector<double> base_mani_fixed_joint_xyz_ypr = node_->get_parameter("mm.base_mani_fixed_joint_xyz_ypr").as_double_array();
    T_q_0_ = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond quad = Eigen::AngleAxisd(base_mani_fixed_joint_xyz_ypr[3], Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(base_mani_fixed_joint_xyz_ypr[4], Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(base_mani_fixed_joint_xyz_ypr[5], Eigen::Vector3d::UnitX());
    T_q_0_.block(0, 0, 3, 3) = quad.toRotationMatrix();
    T_q_0_(0, 3) = base_mani_fixed_joint_xyz_ypr[0];
    T_q_0_(1, 3) = base_mani_fixed_joint_xyz_ypr[1];
    T_q_0_(2, 3) = base_mani_fixed_joint_xyz_ypr[2];

    DECLARE_GET_PARAM(bool, "mm.use_fast_armer", useFastArmer_, true);

    #undef DECLARE_GET_PARAM

    std::string mesh_path = ament_index_cpp::get_package_share_directory("mm_config")  + "/meshes/";

    mesh_resource_mobile_base_ = "file://" + mesh_path + "mobile_base.STL";

    mesh_resource_fastarmer_base0_ = "file://" + mesh_path + "FastArmer/base_link.STL";
    mesh_resource_fastarmer_link1_ = "file://" + mesh_path + "FastArmer/link1.STL";
    mesh_resource_fastarmer_link2_ = "file://" + mesh_path + "FastArmer/link2.STL";
    mesh_resource_fastarmer_link3_ = "file://" + mesh_path + "FastArmer/link3.STL";
    mesh_resource_fastarmer_link4_ = "file://" + mesh_path + "FastArmer/link4.STL";
    mesh_resource_fastarmer_link5_ = "file://" + mesh_path + "FastArmer/link5.STL";
    mesh_resource_fastarmer_link6_ = "file://" + mesh_path + "FastArmer/link6.STL";
    mesh_resource_gripper_base_    = "file://" + mesh_path + "FastArmer/gripper_base.dae";
    mesh_resource_gripper_left_    = "file://" + mesh_path + "FastArmer/gripper_left.dae";
    mesh_resource_gripper_right_   = "file://" + mesh_path + "FastArmer/gripper_right.dae";

    mesh_resource_ur5_base_     = "file://" + mesh_path + "ur5/base.dae";
    mesh_resource_ur5_shoulder_ = "file://" + mesh_path + "ur5/shoulder.dae";
    mesh_resource_ur5_upperarm_ = "file://" + mesh_path + "ur5/upperarm.dae";
    mesh_resource_ur5_forearm_  = "file://" + mesh_path + "ur5/forearm.dae";
    mesh_resource_ur5_wrist1_   = "file://" + mesh_path + "ur5/wrist1.dae";
    mesh_resource_ur5_wrist2_   = "file://" + mesh_path + "ur5/wrist2.dae";
    mesh_resource_ur5_wrist3_   = "file://" + mesh_path + "ur5/wrist3.dae";

    B_h_ << 0.0, -1.0,
            1.0,  0.0;

    T_q_0_ << 1.0, 0  , 0  , 0.03,
                0  , 1.0, 0  , -0.02,
                0  , 0  , 1.0, mobile_base_height_,
                0  , 0  , 0  , 1.0;

    vis_idx_size_ = 100;

    setColorSet();
    setLinkPoint();
}

/**
 * @brief 初始化可视化色板 (8 种固定 RGB 颜色)
 *
 * 色板用于 RViz 中区分不同连杆的碰撞球:
 *   - 索引 0 → 红色    #FF1F5B (底盘碰撞球)
 *   - 索引 1 → 绿色    #00CD6C (连杆 1)
 *   - 索引 2 → 蓝色    #009ADE (连杆 2)
 *   - 索引 3 → 黄色    #FFC61E (连杆 3)
 *   - 索引 4 → 灰色    #A0B1BA (连杆 4)
 *   - 索引 5 → 橙色    #EA6016 (连杆 5)
 *   - 索引 6 → 紫色    #AF58BA (连杆 6)
 *   - 索引 7 → 棕色    #A6761D (备用)
 */
void MMConfig::setColorSet(){
    color_set_.clear();
    // red
    Eigen::Vector3d color;
    color << 255, 31, 91;
    color /= 255.0;
    color_set_.push_back(color);
    // green
    color << 0, 205, 108;
    color /= 255.0;
    color_set_.push_back(color);
    // blue
    color << 0, 154, 222;
    color /= 255.0;
    color_set_.push_back(color);
    // yellow
    color << 255, 198, 30;
    color /= 255.0;
    color_set_.push_back(color);
    // grey
    color << 160, 177, 186;
    color /= 255.0;
    color_set_.push_back(color);
    // orange
    color << 234, 96, 22;
    color /= 255.0;
    color_set_.push_back(color);
    // purple
    color << 175, 88, 186;
    color /= 255.0;
    color_set_.push_back(color);
    // brown
    color << 166, 118, 29;
    color /= 255.0;
    color_set_.push_back(color);
}

/**
 * @brief 计算单个关节的正运动学变换矩阵及其对关节角的导数
 *
 * @param joint_num 关节索引 (0 ~ manipulator_dof_-1)
 * @param theta     该关节的当前角度值 [rad]
 * @param[out] T      关节局部变换矩阵 (4×4 齐次)
 * @param[out] T_grad T 对 theta 的导数 dT/dθ (4×4, 用于优化梯度)
 *
 * 实现两种机械臂的运动学:
 *   - FastArmer (useFastArmer_ = true): 自定义 6-DOF 臂
 *   - UR5        (useFastArmer_ = false): Universal Robots 标准运动学
 *
 * 每个关节的变换矩阵 T 包含旋转部分 (R, 3×3) 和平移部分 (t, 3×1):
 *   T = [R t; 0 1]
 *
 * FastArmer 关节布局 (依次绕不同轴旋转):
 *   J0: 绕 Y 轴旋转 + Z 平移
 *   J1: 绕 Z 轴旋转 + X 平移 (linkLength*cosθ / -linkLength*sinθ)
 *   J2: 绕 Y 轴旋转 + X/Z 平移
 *   J3: 绕 Y 轴旋转 + Z 平移
 *   J4: 绕 Y 轴旋转
 *   J5: 绕 Z 轴旋转 + Z 平移
 *
 * UR5 关节布局:
 *   J0/J3:      绕 Y 轴旋转 -90° + Z 平移
 *   J1/J2:      绕 Z 轴旋转 + X 平移
 *   J4:         绕 Y 轴旋转 +90° + Z 平移
 *   J5:         绕 Z 轴旋转 + Z 平移
 *
 * @note T_grad 是解析导数，用于轨迹优化中的链式法则梯度传播
 * @note 两种机械臂的硬编码运动学参数来自各自的 URDF/厂商参数
 */
void MMConfig::getAJointTran(int joint_num, double theta, Eigen::Matrix4d &T, Eigen::Matrix4d &T_grad){
    double sinTheta = sin(theta);
    double cosTheta = cos(theta);
    double linkLength = manipulator_config_(joint_num);
    T = Eigen::Matrix4d::Identity();
    T_grad = Eigen::Matrix4d::Zero();
    if(useFastArmer_){
        switch(joint_num){
            case 0:{
                T(0, 0) = cosTheta;
                T(0, 2) = sinTheta;
                T(1, 0) = sinTheta;
                T(1, 1) = 0;
                T(1, 2) = -cosTheta;
                T(2, 1) = 1;
                T(2, 2) = 0;
                T(2, 3) = linkLength;

                T_grad(0, 0) = -sinTheta;
                T_grad(0, 2) = cosTheta;
                T_grad(1, 0) = cosTheta;
                T_grad(1, 2) = sinTheta;
                break;
            }
            case 1:{
                T(0, 0) = cosTheta;
                T(0, 1) = -sinTheta;
                T(0, 3) = -linkLength * cosTheta;
                T(1, 0) = sinTheta;
                T(1, 1) = cosTheta;
                T(1, 3) = -linkLength * sinTheta;

                T_grad(0, 0) = -sinTheta;
                T_grad(0, 1) = -cosTheta;
                T_grad(0, 3) = linkLength * sinTheta;
                T_grad(1, 0) = cosTheta;
                T_grad(1, 1) = -sinTheta;
                T_grad(1, 3) = -linkLength * cosTheta;
                break;
            }
            case 2:{
                T(0, 0) = -sinTheta;
                T(0, 2) = cosTheta;
                T(0, 3) = -linkLength * sinTheta;
                T(1, 0) = cosTheta;
                T(1, 1) = 0.0;
                T(1, 2) = sinTheta;
                T(1, 3) = linkLength * cosTheta;
                T(2, 1) = 1.0;
                T(2, 2) = 0.0;

                T_grad(0, 0) = -cosTheta;
                T_grad(0, 2) = -sinTheta;
                T_grad(0, 3) = -linkLength * cosTheta;
                T_grad(1, 0) = -sinTheta;
                T_grad(1, 2) = cosTheta;
                T_grad(1, 3) = -linkLength * sinTheta;
                break;
            }
            case 3:{
                T(0, 0) = cosTheta;
                T(0, 2) = -sinTheta;
                T(1, 0) = sinTheta;
                T(1, 1) = 0;
                T(1, 2) = cosTheta;
                T(2, 1) = -1;
                T(2, 2) = 0;
                T(2, 3) = linkLength;

                T_grad(0, 0) = -sinTheta;
                T_grad(0, 2) = -cosTheta;
                T_grad(1, 0) = cosTheta;
                T_grad(1, 2) = -sinTheta;
                break;
            }
            case 4:{
                T(0, 0) = sinTheta;
                T(0, 2) = cosTheta;
                T(1, 0) = -cosTheta;
                T(1, 1) = 0;
                T(1, 2) = sinTheta;
                T(2, 1) = -1;
                T(2, 2) = 0;

                T_grad(0, 0) = cosTheta;
                T_grad(0, 2) = -sinTheta;
                T_grad(1, 0) = sinTheta;
                T_grad(1, 2) = cosTheta;
                break;
            }
            case 5:{
                T(0, 0) = cosTheta;
                T(0, 1) = -sinTheta;
                T(1, 0) = sinTheta;
                T(1, 1) = cosTheta;
                T(2, 3) = linkLength;

                T_grad(0, 0) = -sinTheta;
                T_grad(0, 1) = -cosTheta;
                T_grad(1, 0) = cosTheta;
                T_grad(1, 1) = -sinTheta;
                break;
            }
                
            default:{
                RCLCPP_ERROR(node_->get_logger(), "err joint_num: %d", joint_num);
                break;
            }
        }
    }else{
        if(joint_num == 0 || joint_num == 3){
            T(0, 0) = cosTheta;
            T(0, 2) = -sinTheta;
            T(1, 0) = sinTheta;
            T(1, 1) = 0;
            T(1, 2) = cosTheta;
            T(2, 1) = -1;
            T(2, 2) = 0;
            T(2, 3) = linkLength;

            T_grad(0, 0) = -sinTheta;
            T_grad(0, 2) = -cosTheta;
            T_grad(1, 0) = cosTheta;
            T_grad(1, 2) = -sinTheta;
        }else if(joint_num == 4){
            T(0, 0) = cosTheta;
            T(0, 2) = sinTheta;
            T(1, 0) = sinTheta;
            T(1, 1) = 0;
            T(1, 2) = -cosTheta;
            T(2, 1) = 1;
            T(2, 2) = 0;
            T(2, 3) = linkLength;

            T_grad(0, 0) = -sinTheta;
            T_grad(0, 2) = cosTheta;
            T_grad(1, 0) = cosTheta;
            T_grad(1, 2) = sinTheta;
        }else if(joint_num == 1 || joint_num == 2){
            T(0, 0) = cosTheta;
            T(0, 1) = -sinTheta;
            T(0, 3) = linkLength * cosTheta;
            T(1, 0) = sinTheta;
            T(1, 1) = cosTheta;
            T(1, 3) = linkLength * sinTheta;

            T_grad(0, 0) = -sinTheta;
            T_grad(0, 1) = -cosTheta;
            T_grad(0, 3) = -linkLength * sinTheta;
            T_grad(1, 0) = cosTheta;
            T_grad(1, 1) = -sinTheta;
            T_grad(1, 3) = linkLength * cosTheta;
        }else if (joint_num == 5){
            T(0, 0) = cosTheta;
            T(0, 1) = -sinTheta;
            T(1, 0) = sinTheta;
            T(1, 1) = cosTheta;
            T(2, 3) = linkLength;

            T_grad(0, 0) = -sinTheta;
            T_grad(0, 1) = -cosTheta;
            T_grad(1, 0) = cosTheta;
            T_grad(1, 1) = -sinTheta;
        }
    }

    
}

/**
 * @brief 发布完整机器人可视化 (网格模型 + 碰撞球)
 * @param pub          ROS Publisher (Marker 类型)
 * @param ns           命名空间 (用于 RViz 分组)
 * @param idx          轨迹时间索引 (确定 Marker ID 的偏移)
 * @param alpha        透明度 (0=全透明, 1=不透明)
 * @param car_state    基底位姿 [x, y, yaw]
 * @param joint_state  机械臂关节角 [θ₀, θ₁, ..., θ₅]
 * @param gripper_close true=夹爪闭合, false=夹爪张开
 *
 * 如果无订阅者则提前返回 (ROS 优化).
 * 内部调用 getMMMarkerArray() 组装所有 Mesh + Sphere Marker.
 */
void MMConfig::visMM(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state, const bool &gripper_close){
    if(pub->get_subscription_count() < 1) return;
    visualization_msgs::msg::MarkerArray marker_array;
    getMMMarkerArray(marker_array, ns, idx, alpha, car_state, joint_state, gripper_close);
    pub->publish(marker_array);
}

/**
 * @brief 发布仅碰撞球的可视化 (不含网格模型, 更轻量)
 * @param pub          ROS Publisher
 * @param ns           命名空间
 * @param idx          轨迹时间索引
 * @param alpha        透明度
 * @param car_state    基底位姿 [x, y, yaw]
 * @param joint_state  机械臂关节角
 *
 * 分别调用 visCarCheckBall() + visManiCheckBall() 发布底盘和臂的碰撞球.
 */
void MMConfig::visMMCheckBall(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state){
    if(pub->get_subscription_count() < 1) return;
    visCarCheckBall(pub, ns, idx, alpha, car_state);
    visManiCheckBall(pub, ns, idx, alpha, car_state, joint_state);
}

void MMConfig::getMMMarkerArray(visualization_msgs::msg::MarkerArray &marker_array, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state, const bool &gripper_close){
    marker_array.markers.clear();
    visualization_msgs::msg::MarkerArray car_marker_array = getCarMarkerArray(ns, idx, alpha, car_state);
    marker_array.markers.insert(marker_array.markers.end(), car_marker_array.markers.begin(), car_marker_array.markers.end());;
    visualization_msgs::msg::MarkerArray mani_marker_array = getManiMarkerArray(ns, idx, alpha, car_state, joint_state, gripper_close);
    marker_array.markers.insert(marker_array.markers.end(), mani_marker_array.markers.begin(), mani_marker_array.markers.end());;
}

/**
 * @brief 从速度向量计算基底 yaw 角
 * @param vel_car  速度向量 [vx, vy]
 * @param traj_dir 运动方向 (±1: 前进/后退)
 * @return yaw 角 [rad], 范围 (-π, π]
 *
 * 前进时 (traj_dir=+1): yaw = atan2(vy, vx)
 * 后退时 (traj_dir=-1): yaw = atan2(-vy, -vx) ← 相当于前进方向反转
 */
double MMConfig::calYaw(const Eigen::Vector2d &vel_car, int traj_dir){
    return atan2(traj_dir * vel_car(1), traj_dir * vel_car(0));
}

/**
 * @brief 从速度向量计算 2D 旋转矩阵
 * @param vel_car  速度向量 [vx, vy]
 * @param traj_dir 运动方向 (±1)
 * @return 2×2 旋转矩阵 R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)]
 *
 * 前进时: yaw 指向速度方向
 * 后退时: yaw 指向速度反方向 (速度向后，但"朝向"仍为前进方向)
 */
Eigen::Matrix2d MMConfig::calR(const Eigen::Vector2d &vel_car, int traj_dir){
    double yaw = atan2(traj_dir * vel_car(1), traj_dir * vel_car(0));
    Eigen::Matrix2d R;
    R << cos(yaw), -sin(yaw),
         sin(yaw), cos(yaw);
    // R = R * traj_dir / vel_car.norm();
    return R;
}

/**
 * @brief 计算 R*δp 对速度 v 的导数 (2×2 雅可比矩阵)
 * @param vel_car 速度向量 [vx, vy]
 * @param delta_p 局部偏移量 (基底坐标系下的点)
 * @param traj_dir 运动方向
 * @return d(R·δp)/dv  (2×2 矩阵)
 *
 * 推导: R(θ(v))·δp 对 v 求导:
 *   dR/dv = [δp, B_h·δp]·[dθ/dv]  +  R·(d/dv 没有, δp 是常数)
 *   其中 dθ/dv = traj_dir·B_h·v / |v|²
 *   最终: d(Rδp)/dv = traj_dir·[δp B_hδp] / |v| - Rδp·vᵀ/|v|²
 *
 * 用于 getCarPtsGrad() 中的链式梯度传播.
 */
Eigen::Matrix2d MMConfig::caldRldv(const Eigen::Vector2d &vel_car, const Eigen::Vector2d &delta_p, int traj_dir){
    Eigen::Matrix2d dRldv;
    dRldv << delta_p, B_h_ * delta_p;
    dRldv = dRldv * traj_dir / vel_car.norm() - calR(vel_car, traj_dir) * delta_p * vel_car.transpose() / vel_car.squaredNorm();
    return dRldv;
}

/**
 * @brief 计算 yaw 角对速度向量的梯度 dyaw/dv (2×1)
 * @param vel_car 速度向量 [vx, vy]
 * @return dyaw/dv = B_h·v / |v|² = [-vy, vx]ᵀ / |v|²
 *
 * 推导: yaw = atan2(vy, vx)
 *   dyaw/dvx = -vy / (vx²+vy²)
 *   dyaw/dvy =  vx / (vx²+vy²)
 *   dyaw/dv  = B_h·v / |v|²  (使用反对称矩阵 B_h = [0 -1; 1 0])
 */
Eigen::Vector2d MMConfig::caldYawdV(const Eigen::Vector2d &vel_car){
    Eigen::Vector2d dYawdV;
    dYawdV = B_h_ * vel_car / vel_car.squaredNorm();
    return dYawdV;
}

void MMConfig::getCarPts(const Eigen::Vector3d &car_state, std::vector<Eigen::Vector3d> &car_pts){
    getCarPts(car_state, car_pts, Eigen::Vector3d(0, 0, 0));
}

/**
 * @brief 获取基底碰撞检测球心点集 (默认无膨胀)
 *
 * 采样策略 — 矩形边界框 + 球半径填充分层采样:
 *
 *   1. 计算 4 个圆角顶点位置 (p_i = 角点坐标 - check_radius 偏移)
 *      corner1 = R * [(L-r)/2,  (W-r)/2]ᵀ + car_pos
 *      corner2 = R * [(L-r)/2, -(W-r)/2]ᵀ + car_pos
 *      corner3 = R * [-(L-r)/2, -(W-r)/2]ᵀ + car_pos
 *      corner4 = R * [-(L-r)/2,  (W-r)/2]ᵀ + car_pos
 *
 *   2. 沿每条边等距 (step=r) 采样中间点
 *
 *   3. 在 Z 轴方向分层: height = r, 2r, 3r, ..., height_total
 *
 *   这保证了整个底盘被半径为 r 的球体完全覆盖 (r = mobile_base_check_radius_).
 *
 * @param car_state 基底位姿 [x, y, yaw]
 * @param[out] car_pts 碰撞检测球心点列表 (3D 世界坐标)
 * @param inflate_size 膨胀尺寸 [ΔL, ΔW, ΔH] (用于 safety margin)
 */
void MMConfig::getCarPts(const Eigen::Vector3d &car_state, std::vector<Eigen::Vector3d> &car_pts, const Eigen::Vector3d &inflate_size){
    car_pts.clear();
    Eigen::Vector3d point_3d;
    Eigen::Matrix2d R;
    R << cos(car_state(2)), -sin(car_state(2)),
         sin(car_state(2)),  cos(car_state(2));
    Eigen::Vector2d corner1 = car_state.head(2) + R * Eigen::Vector2d( (mobile_base_length_ + inflate_size(0)) / 2 - mobile_base_check_radius_,  (mobile_base_width_ + inflate_size(1)) / 2 - mobile_base_check_radius_);
    Eigen::Vector2d corner2 = car_state.head(2) + R * Eigen::Vector2d( (mobile_base_length_ + inflate_size(0)) / 2 - mobile_base_check_radius_, -(mobile_base_width_ + inflate_size(1)) / 2 + mobile_base_check_radius_);
    Eigen::Vector2d corner3 = car_state.head(2) + R * Eigen::Vector2d(-(mobile_base_length_ + inflate_size(0)) / 2 + mobile_base_check_radius_, -(mobile_base_width_ + inflate_size(1)) / 2 + mobile_base_check_radius_);
    Eigen::Vector2d corner4 = car_state.head(2) + R * Eigen::Vector2d(-(mobile_base_length_ + inflate_size(0)) / 2 + mobile_base_check_radius_,  (mobile_base_width_ + inflate_size(1)) / 2 - mobile_base_check_radius_);
    
    double norm12 = (corner2 - corner1).norm();
    double norm23 = (corner3 - corner2).norm();
    double norm34 = (corner4 - corner3).norm();
    double norm41 = (corner1 - corner4).norm();

    
    for(double height = mobile_base_check_radius_; height < mobile_base_height_ + inflate_size(2); height += mobile_base_check_radius_){
        // 四个顶点
        point_3d(2) = height;
        point_3d.head(2) = corner1;
        car_pts.push_back(point_3d);
        point_3d.head(2) = corner2;
        car_pts.push_back(point_3d);
        point_3d.head(2) = corner3;
        car_pts.push_back(point_3d);
        point_3d.head(2) = corner4;
        car_pts.push_back(point_3d);

        for(double dl = mobile_base_check_radius_; dl < norm12; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm12 * (corner2 - corner1) + corner1;
            car_pts.push_back(point_3d);
        }

        for(double dl = mobile_base_check_radius_; dl < norm23; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm23 * (corner3 - corner2) + corner2;
            car_pts.push_back(point_3d);
        }

        for(double dl = mobile_base_check_radius_; dl < norm34; dl+=mobile_base_check_radius_){
            point_3d.head(2) = dl / norm34 * (corner4 - corner3) + corner3;
            car_pts.push_back(point_3d);
        }

        for(double dl = mobile_base_check_radius_; dl < norm41; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm41 * (corner1 - corner4) + corner4;
            car_pts.push_back(point_3d);
        }
    }
}

/**
 * @brief 获取基底碰撞球心点集 + 对速度的雅可比 (用于轨迹优化梯度)
 *
 * 与 getCarPts() 相同的采样策略, 但额外输出每个球心对速度 v 的导数 dPt/dv.
 *
 * 雅可比推导 (以 corner1 为例):
 *   p₁ = pos_car + R(θ(v)) · δp₁
 *   d(p₁)/dv = d(R·δp₁)/dv  (调用 caldRldv())
 *
 * 边上中间点: p = pos_car + λ·(p₂-p₁) + p₁
 *   d(p)/dv = λ·(d(p₂)/dv - d(p₁)/dv) + d(p₁)/dv
 *
 * @param pos_car    基底位置 [x, y]
 * @param vel_car    基底速度 [vx, vy] (用于计算 yaw = atan2(vy, vx))
 * @param traj_dir   运动方向 (±1)
 * @param inflate_size 膨胀尺寸
 * @param[out] car_pts  碰撞球心点集 (3D)
 * @param[out] dPtdv    每个球心对速度 v 的雅可比 (2×2 矩阵列表)
 *
 * @note 与 getCarPtsGradNew() 的区别:
 *       本函数用速度 v 计算 yaw (适合 MINCO 优化的速度空间),
 *       getCarPtsGradNew() 直接用 yaw (适合 yaw 空间的优化)
 */
void MMConfig::getCarPtsGrad(const Eigen::Vector2d &pos_car, const Eigen::Vector2d &vel_car, const int traj_dir, const Eigen::Vector3d &inflate_size,
                    std::vector<Eigen::Vector3d> &car_pts, std::vector<Eigen::Matrix2d> &dPtdv){
    car_pts.clear();
    dPtdv.clear();
    Eigen::Vector3d point_3d;
    Eigen::Matrix2d R = calR(vel_car, traj_dir);
    double v_norm = vel_car.norm();
    double vTv = vel_car.squaredNorm();

    Eigen::Vector2d delta_p = Eigen::Vector2d( (mobile_base_length_ + inflate_size(0)) / 2 - mobile_base_check_radius_,  (mobile_base_width_ + inflate_size(1)) / 2 - mobile_base_check_radius_);
    Eigen::Vector2d corner1 = pos_car + R * delta_p;
    Eigen::Matrix2d dcorner1dv;
    dcorner1dv << delta_p, B_h_ * delta_p;
    dcorner1dv = dcorner1dv * traj_dir / v_norm - R * delta_p * vel_car.transpose() / vTv;

    delta_p = Eigen::Vector2d( (mobile_base_length_ + inflate_size(0)) / 2 - mobile_base_check_radius_, -(mobile_base_width_ + inflate_size(1)) / 2 + mobile_base_check_radius_);
    Eigen::Vector2d corner2 = pos_car + R * delta_p;
    Eigen::Matrix2d dcorner2dv;
    dcorner2dv << delta_p, B_h_ * delta_p;
    dcorner2dv = dcorner2dv * traj_dir / v_norm - R * delta_p * vel_car.transpose() / vTv;

    delta_p = Eigen::Vector2d(-(mobile_base_length_ + inflate_size(0)) / 2 + mobile_base_check_radius_, -(mobile_base_width_ + inflate_size(1)) / 2 + mobile_base_check_radius_);
    Eigen::Vector2d corner3 = pos_car + R * delta_p;
    Eigen::Matrix2d dcorner3dv;
    dcorner3dv << delta_p, B_h_ * delta_p;
    dcorner3dv = dcorner3dv * traj_dir / v_norm - R * delta_p * vel_car.transpose() / vTv;

    delta_p = Eigen::Vector2d(-(mobile_base_length_ + inflate_size(0)) / 2 + mobile_base_check_radius_,  (mobile_base_width_ + inflate_size(1)) / 2 - mobile_base_check_radius_);
    Eigen::Vector2d corner4 = pos_car + R * delta_p;
    Eigen::Matrix2d dcorner4dv;
    dcorner4dv << delta_p, B_h_ * delta_p;
    dcorner4dv = dcorner4dv * traj_dir / v_norm - R * delta_p * vel_car.transpose() / vTv;
    
    double norm12 = (corner2 - corner1).norm();
    double norm23 = (corner3 - corner2).norm();
    double norm34 = (corner4 - corner3).norm();
    double norm41 = (corner1 - corner4).norm();
    
    for(double height = mobile_base_check_radius_; height < mobile_base_height_ + inflate_size(2); height += mobile_base_check_radius_){
        // 四个顶点
        point_3d(2) = height;
        point_3d.head(2) = corner1;
        car_pts.push_back(point_3d);
        dPtdv.push_back(dcorner1dv);
        point_3d.head(2) = corner2;
        car_pts.push_back(point_3d);
        dPtdv.push_back(dcorner2dv);
        point_3d.head(2) = corner3;
        car_pts.push_back(point_3d);
        dPtdv.push_back(dcorner3dv);
        point_3d.head(2) = corner4;
        car_pts.push_back(point_3d);
        dPtdv.push_back(dcorner4dv);

        for(double dl = mobile_base_check_radius_; dl < norm12; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm12 * (corner2 - corner1) + corner1;
            car_pts.push_back(point_3d);
            dPtdv.push_back(dl / norm12 * (dcorner2dv - dcorner1dv) + dcorner1dv);
        }

        for(double dl = mobile_base_check_radius_; dl < norm23; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm23 * (corner3 - corner2) + corner2;
            car_pts.push_back(point_3d);
            dPtdv.push_back(dl / norm23 * (dcorner3dv - dcorner2dv) + dcorner2dv);
        }

        for(double dl = mobile_base_check_radius_; dl < norm34; dl+=mobile_base_check_radius_){
            point_3d.head(2) = dl / norm34 * (corner4 - corner3) + corner3;
            car_pts.push_back(point_3d);
            dPtdv.push_back(dl / norm34 * (dcorner4dv - dcorner3dv) + dcorner3dv);
        }

        for(double dl = mobile_base_check_radius_; dl < norm41; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm41 * (corner1 - corner4) + corner4;
            car_pts.push_back(point_3d);
            dPtdv.push_back(dl / norm41 * (dcorner1dv - dcorner4dv) + dcorner4dv);
        }
    }
}

/**
 * @brief 获取基底碰撞球心点集 + 对 yaw 的梯度 (新版本)
 *
 * 与 getCarPtsGrad() 相同的采样策略, 但输出的是 dPt/dYaw 而非 dPt/dv.
 *
 * 雅可比: p = pos + R(yaw)·δp → dp/dYaw = dR/dYaw·δp
 *   其中 dR/dYaw = [-sin(yaw) -cos(yaw); cos(yaw) -sin(yaw)]
 *
 * @param[out] dPtdYaw 每个球心对 yaw 的梯度 (2×1 向量列表)
 */
void MMConfig::getCarPtsGradNew(const Eigen::Vector2d &pos_car, const Eigen::Vector2d &vel_car, const int traj_dir, const Eigen::Vector3d &inflate_size,
                    std::vector<Eigen::Vector3d> &car_pts, std::vector<Eigen::Vector2d> &dPtdYaw){
    car_pts.clear();
    dPtdYaw.clear();
    double yaw = atan2(traj_dir * vel_car(1), traj_dir * vel_car(0));
    Eigen::Vector3d point_3d;
    Eigen::Matrix2d R, dRdYaw;
    R << cos(yaw), -sin(yaw),
         sin(yaw), cos(yaw);
    dRdYaw << -sin(yaw), -cos(yaw),
               cos(yaw), -sin(yaw);
    // double v_norm = vel_car.norm();

    Eigen::Vector2d delta_p = Eigen::Vector2d( (mobile_base_length_ + inflate_size(0)) / 2 - mobile_base_check_radius_,  (mobile_base_width_ + inflate_size(1)) / 2 - mobile_base_check_radius_);
    Eigen::Vector2d corner1 = pos_car + R * delta_p;
    Eigen::Vector2d dcorner1dYaw;
    dcorner1dYaw = dRdYaw * delta_p;

    delta_p = Eigen::Vector2d( (mobile_base_length_ + inflate_size(0)) / 2 - mobile_base_check_radius_, -(mobile_base_width_ + inflate_size(1)) / 2 + mobile_base_check_radius_);
    Eigen::Vector2d corner2 = pos_car + R * delta_p;
    Eigen::Vector2d dcorner2dYaw;
    dcorner2dYaw = dRdYaw * delta_p;

    delta_p = Eigen::Vector2d(-(mobile_base_length_ + inflate_size(0)) / 2 + mobile_base_check_radius_, -(mobile_base_width_ + inflate_size(1)) / 2 + mobile_base_check_radius_);
    Eigen::Vector2d corner3 = pos_car + R * delta_p;
    Eigen::Vector2d dcorner3dYaw;
    dcorner3dYaw = dRdYaw * delta_p;

    delta_p = Eigen::Vector2d(-(mobile_base_length_ + inflate_size(0)) / 2 + mobile_base_check_radius_,  (mobile_base_width_ + inflate_size(1)) / 2 - mobile_base_check_radius_);
    Eigen::Vector2d corner4 = pos_car + R * delta_p;
    Eigen::Vector2d dcorner4dYaw;
    dcorner4dYaw = dRdYaw * delta_p;
    
    double norm12 = (corner2 - corner1).norm();
    double norm23 = (corner3 - corner2).norm();
    double norm34 = (corner4 - corner3).norm();
    double norm41 = (corner1 - corner4).norm();
    
    for(double height = mobile_base_check_radius_; height < mobile_base_height_ + inflate_size(2); height += mobile_base_check_radius_){
        // 四个顶点
        point_3d(2) = height;
        point_3d.head(2) = corner1;
        car_pts.push_back(point_3d);
        dPtdYaw.push_back(dcorner1dYaw);
        point_3d.head(2) = corner2;
        car_pts.push_back(point_3d);
        dPtdYaw.push_back(dcorner2dYaw);
        point_3d.head(2) = corner3;
        car_pts.push_back(point_3d);
        dPtdYaw.push_back(dcorner3dYaw);
        point_3d.head(2) = corner4;
        car_pts.push_back(point_3d);
        dPtdYaw.push_back(dcorner4dYaw);

        for(double dl = mobile_base_check_radius_; dl < norm12; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm12 * (corner2 - corner1) + corner1;
            car_pts.push_back(point_3d);
            dPtdYaw.push_back(dl / norm12 * (dcorner2dYaw - dcorner1dYaw) + dcorner1dYaw);
        }

        for(double dl = mobile_base_check_radius_; dl < norm23; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm23 * (corner3 - corner2) + corner2;
            car_pts.push_back(point_3d);
            dPtdYaw.push_back(dl / norm23 * (dcorner3dYaw - dcorner2dYaw) + dcorner2dYaw);
        }

        for(double dl = mobile_base_check_radius_; dl < norm34; dl+=mobile_base_check_radius_){
            point_3d.head(2) = dl / norm34 * (corner4 - corner3) + corner3;
            car_pts.push_back(point_3d);
            dPtdYaw.push_back(dl / norm34 * (dcorner4dYaw - dcorner3dYaw) + dcorner3dYaw);
        }

        for(double dl = mobile_base_check_radius_; dl < norm41; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm41 * (corner1 - corner4) + corner4;
            car_pts.push_back(point_3d);
            dPtdYaw.push_back(dl / norm41 * (dcorner1dYaw - dcorner4dYaw) + dcorner4dYaw);
        }
    }
}

/**
 * @brief 基底 3D 位姿 → 4×4 齐次变换矩阵
 * @param car_state [x, y, yaw] — 2D 位置 + 朝向角
 * @param[out] T_car 从世界坐标系到基底坐标系的齐次变换
 *
 * T_car = [cos(yaw)  -sin(yaw)  0  x
 *          sin(yaw)   cos(yaw)  0  y
 *          0          0         1  0
 *          0          0         0  1]
 */
void MMConfig::CarState2T(const Eigen::Vector3d &car_state, Eigen::Matrix4d &T_car){
    T_car.setIdentity();
    double c = cos(car_state(2)), s = sin(car_state(2));
    Eigen::Matrix3d R;
    R << c, -s, 0, s, c, 0, 0, 0, 1;
    T_car.block(0, 3, 3, 1) = Eigen::Vector3d(car_state(0), car_state(1), 0.0);
    T_car.block(0, 0, 3, 3) = R;
}

void MMConfig::getJointTMat(const Eigen::VectorXd &theta, std::vector<Eigen::Matrix4d> &T_joint){
    T_joint.clear();
    Eigen::Matrix4d T_temp, T_temp_grad_nouse;
    T_temp = Eigen::Matrix4d::Identity();
    T_joint.push_back(T_temp);
    for(int i = 0; i < manipulator_dof_ - 1; ++i)
    {
        getAJointTran(i, theta(i), T_temp, T_temp_grad_nouse);
        T_joint.push_back(T_temp);
    }
}

/**
 * @brief 第 1 层碰撞检测: 底盘 ↔ 障碍物
 *
 * 方法:
 *   1. 调用 getCarPts() 生成底盘碰撞球心点集
 *   2. 对每个球心查询 ESDF 距离 (precise=true → 三线性插值, false → 最近网格中心)
 *   3. 若 distance < safe_dist → 碰撞!
 *
 * safe_dist = mobile_base_check_radius_ + car_safe_margin_ (+ map_resolution_)
 *   其中 mobile_base_check_radius_ 是球的半径 (球的覆盖半径)
 *
 * @param car_state 基底位姿 [x, y, yaw]
 * @param precise   是否使用精确距离 (三线性插值) vs 离散网格距离
 * @param safe      是否加 safety margin (car_safe_margin_)
 * @param[out] min_dist 检测到的最小距离 (碰撞时 = 最小穿透深度)
 * @return true=碰撞, false=安全
 */
bool MMConfig::checkCarObsCollision(Eigen::Vector3d car_state, bool precise, bool safe, double &min_dist){
    std::vector<Eigen::Vector3d> car_pts;
    car_pts.clear();
    getCarPts(car_state, car_pts);
    double dist;
    double safe_dist = safe ? mobile_base_check_radius_ + car_safe_margin_ : mobile_base_check_radius_;
    safe_dist += map_resolution_;
    for(unsigned int i = 0; i < car_pts.size(); ++i){
        if(precise){
            dist = grid_map_->getPreciseDistance(car_pts[i]);
        }else{
            dist = grid_map_->getDistance(car_pts[i]);
        }
        if(dist < safe_dist){
            min_dist = dist;
            return true;
        }
    }
    min_dist = safe_dist;
    return false;
}

/**
 * @brief 第 2 层碰撞检测: 机械臂 ↔ 障碍物
 *
 * 方法:
 *   1. 构建基底 → 机械臂的坐标变换链: T_now = T_car * T_q_0_
 *   2. 沿关节链正向传播: T_now = T_now * T_joint[i]  (i = 0..N-1)
 *   3. 对每个关节的每个连杆采样点: pt_world = T_now * link_pts[i].col(j)
 *   4. 查询 pt_world 的 ESDF 距离 + 检查地面碰撞 (z < ground_safe_dis_)
 *
 * safe_dist = manipulator_thickness_ + mani_safe_margin_
 *
 * @param car_state  基底位姿 [x, y, yaw]
 * @param mani_state 机械臂关节角 [θ₀, ..., θ₅]
 * @param safe       是否加 safety margin
 * @param[out] min_dist 最小距离
 * @return true=碰撞/地面穿透, false=安全
 */
bool MMConfig::checkManiObsCollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe, double &min_dist){
    Eigen::Matrix4d T_q = Eigen::Matrix4d::Identity();
    T_q(0, 0) = cos(car_state(2));
    T_q(0, 1) = -sin(car_state(2));
    T_q(0, 3) = car_state(0);
    T_q(1, 0) = sin(car_state(2));
    T_q(1, 1) = cos(car_state(2));
    T_q(1, 3) = car_state(1);
    double safe_dist = safe ? manipulator_thickness_ + mani_safe_margin_ : manipulator_thickness_;
    // safe_dist += map_resolution_ / 2.0;
    geometry_msgs::msg::Point pt;
    Eigen::Vector3d pt_on_link;
    Eigen::Matrix4d T_now = T_q * T_q_0_;
    std::vector<Eigen::Matrix4d> T_joint, T_joint_grad_nouse;
    T_joint.clear();
    getJointTrans(mani_state, T_joint, T_joint_grad_nouse);
    double dist;
    for(int i = 0; i < manipulator_dof_; ++i){
        T_now = T_now * T_joint[i];
        // get ESDF value
        int pts_size = manipulator_link_pts_[i].cols();
        for(int j = 0; j < pts_size; ++j){
            pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);
            pt.x = pt_on_link(0);
            pt.y = pt_on_link(1);
            pt.z = pt_on_link(2);
            if(pt_on_link(2) < ground_safe_dis_){
                min_dist = pt_on_link(2);
                sphere_occ_.points.push_back(pt);
                return true;
            }
            dist = grid_map_->getPreciseDistance(pt_on_link);
            // dist = grid_map_->getDistance(pt_on_link);
            if(dist < safe_dist){
                sphere_occ_.points.push_back(pt);
                min_dist = dist;
                return true;
            }
        }
    }
    min_dist = safe_dist;
    return false;
}

/**
 * @brief 第 3 层碰撞检测: 底盘 ↔ 机械臂 (自碰撞)
 *
 * 方法:
 *   1. 假设基底位于原点 (car_state = [0,0,0]ᵀ), 生成底盘碰撞球
 *   2. 沿关节链正向传播机械臂连杆点 (相对于基底坐标系)
 *   3. 对每个 机械臂点 ↔ 底盘点 对: 检查欧几里得距离
 *
 * safe_dist = mobile_base_check_radius_ + manipulator_thickness_ + self_safe_margin_
 *
 * @note 使用局部坐标系 (基座原点), 因为自碰撞距离与全局位置无关
 * @note 复杂度 O(M·N) 但 M(底盘点数) 和 N(臂点数) 都不大 (~几十)
 */
bool MMConfig::checkCarManiCollision(Eigen::VectorXd mani_state, bool safe, double &min_dist){
    double safe_dist = safe ? mobile_base_check_radius_ + manipulator_thickness_ + self_safe_margin_ : mobile_base_check_radius_ + manipulator_thickness_;
    std::vector<Eigen::Vector3d> car_pts;
    getCarPts(Eigen::Vector3d::Zero(), car_pts);
    Eigen::Vector3d pt_on_link;
    std::vector<Eigen::Vector3d> pt_to_check_list;
    std::vector<Eigen::Matrix4d> T_joint, T_joint_grad_nouse;
    T_joint.clear();
    getJointTrans(mani_state, T_joint, T_joint_grad_nouse);
    int car_pts_size = car_pts.size();
    int pts_size;
    Eigen::Matrix4d T_now = T_q_0_ * T_joint[0];
    for(int i = 1; i < manipulator_dof_; ++i){
        T_now = T_now * T_joint[i];
        pts_size = manipulator_link_pts_[i].cols();
        for(int j = 0; j < pts_size; ++j){
            pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);
            for(int k = 0; k < car_pts_size; ++k){
                if((pt_on_link - car_pts[k]).norm() < safe_dist){
                    min_dist = (pt_on_link - car_pts[k]).norm();
                    return true;
                }
            }
        }
    }
    min_dist = safe_dist;
    return false;
}

/**
 * @brief 第 4 层碰撞检测: 机械臂 ↔ 机械臂 (关节间自碰撞)
 *
 * 方法:
 *   1. 沿关节链正向传播，收集每个关节的连杆点 (相对于基座)
 *   2. 对于关节 i 的新增连杆点, 与之前关节 j ≤ i-2 的所有点比较距离
 *      (跳过相邻关节 i-1, 因为相邻关节本身不会自碰撞)
 *
 * safe_dist = 2 × manipulator_thickness_ + self_safe_margin_
 *
 * @note 这防止了非相邻关节之间的自碰撞 (如手腕碰到肩部)
 * @note checkCarManiCollision() 已处理底盘→臂碰撞, 本函数仅处理臂→臂碰撞
 */
bool MMConfig::checkManiManiCollision(Eigen::VectorXd mani_state, bool safe, double &min_dist){
    double safe_dist = safe ? 2.0 * manipulator_thickness_ + self_safe_margin_ : 2.0 * manipulator_thickness_;
    Eigen::Vector3d pt_on_link;
    std::vector<Eigen::Vector3d> pt_to_check_list;
    pt_to_check_list.reserve(20);
    std::vector<Eigen::Matrix4d> T_joint, T_joint_grad_nouse;
    T_joint.reserve(manipulator_dof_);
    T_joint_grad_nouse.reserve(manipulator_dof_);
    getJointTrans(mani_state, T_joint, T_joint_grad_nouse);
    int num_to_check = 0;
    int pts_size;
    Eigen::Matrix4d T_now = Eigen::Matrix4d::Identity();
    for(int i = 0; i < manipulator_dof_; ++i){
        T_now *= T_joint[i];
        if(i >= 2)
            num_to_check += manipulator_link_pts_[i - 2].cols();
        // if(i == manipulator_dof_ - 1)
        //     num_to_check += manipulator_link_pts_[i - 1].cols();
        pts_size = manipulator_link_pts_[i].cols();
        for(int j = 0; j < pts_size; ++j){
            pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);
            pt_to_check_list.push_back(pt_on_link);
            for(int k = 0; k < num_to_check; ++k){
                if((pt_on_link - pt_to_check_list[k]).norm() < safe_dist){
                    // printf("(%d, %d, %d)\n", i, j, k);
                    min_dist = (pt_on_link - pt_to_check_list[k]).norm();
                    return true;
                }
            }
        }
    }
    min_dist = safe_dist;
    return false;
}

/**
 * @brief 机械臂相关碰撞综合检测 (3 层: mani-obs + car-mani + mani-mani)
 *
 * 不包含 checkCarObsCollision (底盘→障碍物), 仅检查与机械臂相关的碰撞.
 * 用于机械臂规划时快速评估构型安全性.
 */
bool MMConfig::checkManicollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe){
    double min_dist;
    if(checkManiObsCollision(car_state, mani_state, safe, min_dist)){
        return true;
    }
    if(checkCarManiCollision(mani_state, safe, min_dist)){
        return true;
    }
    if(checkManiManiCollision(mani_state, safe, min_dist)){
        return true;
    }
    return false;
}

/**
 * @brief 完整 4 层碰撞检测 + 碰撞类型识别
 *
 * 按优先级依次检测 (先检测最可能的碰撞):
 *   Layer 1 (coll_type=0): checkCarObsCollision()    底盘 ↔ 障碍物
 *   Layer 2 (coll_type=1): checkManiObsCollision()   机械臂 ↔ 障碍物
 *   Layer 3 (coll_type=2): checkCarManiCollision()   底盘 ↔ 机械臂 (自碰撞)
 *   Layer 4 (coll_type=3): checkManiManiCollision()  机械臂 ↔ 机械臂 (自碰撞)
 *
 * @param[out] coll_type 碰撞类型: 0=底盘, 1=臂, 2=底盘-臂自碰, 3=臂-臂自碰, -1=无碰撞
 * @return true=有碰撞, false=完全安全
 */
bool MMConfig::checkcollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe, int &coll_type /*0: car, 1: mani, 2: car-mani, 3: mani-mani*/){
    double min_dist;
    if(checkCarObsCollision(car_state, true, safe, min_dist)){
        coll_type = 0;
        return true;
    }
    if(checkManiObsCollision(car_state, mani_state, safe, min_dist)){
        coll_type = 1;
        return true;
    }
    if(checkCarManiCollision(mani_state, safe, min_dist)){
        coll_type = 2;
        return true;
    }
    if(checkManiManiCollision(mani_state, safe, min_dist)){
        coll_type = 3;
        return true;
    }
    coll_type = -1;
    return false;
}

bool MMConfig::checkcollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe){
    double min_dist;
    if(checkCarObsCollision(car_state, true, safe, min_dist)){
        return true;
    }
    if(checkManiObsCollision(car_state, mani_state, safe, min_dist)){
        return true;
    }
    if(checkCarManiCollision(mani_state, safe, min_dist)){
        return true;
    }
    if(checkManiManiCollision(mani_state, safe, min_dist)){
        return true;
    }
    return false;
}

/**
 * @brief 初始化机械臂各连杆的碰撞代理球心点集 (硬件编码)
 *
 * 为每个关节的连杆定义一组 4D 齐次坐标点 (相对于该关节的局部坐标系).
 * 这些点在碰撞检测中作为球体的球心 (球半径 = manipulator_thickness_).
 *
 * 点集的设计原则:
 *   - 沿连杆轴向和径向分布, 保证被厚度半径的球完全覆盖
 *   - 球心间距 ≈ 连杆直径/2, 确保无死角
 *
 * FastArmer 布局:
 *   J0: 4 点  — 基座 (垂直轴) [中心, +Z, -Z, -Y]
 *   J1: 6 点  — 沿 X 轴 0→0.35m 线性分布 (大臂)
 *   J2: 1 点  — 原点 (短连杆)
 *   J3: 5 点  — 沿 Y 轴 0.07→0.35m 线性分布 (前臂)
 *   J4: 1 点  — 原点 (短连杆)
 *   J5: 7 点  — 手腕 + 夹爪区域 (3D 分布)
 *
 * UR5 布局:
 *   J0: 2 点  — 基座
 *   J1: 5 点  — 沿 -X 轴 0→-0.4m 线性分布
 *   J2: 5 点  — 沿 -X 轴 0→-0.4m 线性分布
 *   J3/J4/J5: 各 1 点 — 原点
 *
 * @note 这些硬编码值与网格模型的大小匹配, 修改网格时需同步更新
 */
void MMConfig::setLinkPoint()
{
    manipulator_link_pts_.clear();
    Eigen::Matrix4Xd link_pts;
    if(useFastArmer_){
        for(int i = 0; i < manipulator_dof_; ++i){
            switch(i){
            case 0:{
                link_pts.resize(4, 4);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                link_pts.col(1) = Eigen::Vector4d(0, 0, 0.05, 1);
                link_pts.col(2) = Eigen::Vector4d(0, 0, -0.05, 1);
                link_pts.col(3) = Eigen::Vector4d(0, -0.05, 0, 1);
                break;
            }
            case 1:{
                link_pts.resize(4, 6);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0.0, 1);
                link_pts.col(1) = Eigen::Vector4d(0.07, 0, 0.0, 1);
                link_pts.col(2) = Eigen::Vector4d(0.14, 0, 0.0, 1);
                link_pts.col(3) = Eigen::Vector4d(0.21, 0, 0.0, 1);
                link_pts.col(4) = Eigen::Vector4d(0.28, 0, 0.0, 1);
                link_pts.col(5) = Eigen::Vector4d(0.35, 0, 0.0, 1);
                break;
            }
            case 2:{
                link_pts.resize(4, 1);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                break;
            }
            case 3:{
                link_pts.resize(4, 5);
                link_pts.col(0) = Eigen::Vector4d(0.0, 0.07, 0.0, 1);
                link_pts.col(1) = Eigen::Vector4d(0.0, 0.14, 0.0, 1);
                link_pts.col(2) = Eigen::Vector4d(0.0, 0.21, 0.0, 1);
                link_pts.col(3) = Eigen::Vector4d(0.0, 0.28, 0.0, 1);
                link_pts.col(4) = Eigen::Vector4d(0.0, 0.35, 0.0, 1);
                // link_pts.col(4) = Eigen::Vector4d(0, 0.4, 0.0, 1);
                break;
            }
            case 4:{
                link_pts.resize(4, 1);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                break;
            }
            case 5:{
                link_pts.resize(4, 7);
                link_pts.col(0) = Eigen::Vector4d(0, 0, -0.10, 1);
                link_pts.col(1) = Eigen::Vector4d(0, 0.03, -0.10, 1);
                link_pts.col(2) = Eigen::Vector4d(0, -0.03, -0.10, 1);
                link_pts.col(3) = Eigen::Vector4d(0, 0.05, -0.05, 1);
                link_pts.col(4) = Eigen::Vector4d(0, -0.05, -0.05, 1);
                link_pts.col(5) = Eigen::Vector4d(0, 0.06, -0.00, 1);
                link_pts.col(6) = Eigen::Vector4d(0, -0.06, -0.00, 1);
                break;
            }
            default:
                break;
            }
            manipulator_link_pts_.push_back(link_pts);
        }
    }else{
        for(int i = 0; i < manipulator_dof_; ++i){
            switch(i){
            case 0:{
                link_pts.resize(4, 2);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                link_pts.col(1) = Eigen::Vector4d(0, 0.05, 0, 1);
                break;
            }
            case 1:{
                link_pts.resize(4, 5);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0.14, 1);
                link_pts.col(1) = Eigen::Vector4d(-0.1, 0, 0.14, 1);
                link_pts.col(2) = Eigen::Vector4d(-0.2, 0, 0.14, 1);
                link_pts.col(3) = Eigen::Vector4d(-0.3, 0, 0.14, 1);
                link_pts.col(4) = Eigen::Vector4d(-0.4, 0, 0.14, 1);
                break;
            }
            case 2:{
                link_pts.resize(4, 5);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                link_pts.col(1) = Eigen::Vector4d(-0.1, 0, 0, 1);
                link_pts.col(2) = Eigen::Vector4d(-0.2, 0, 0, 1);
                link_pts.col(3) = Eigen::Vector4d(-0.3, 0, 0, 1);
                link_pts.col(4) = Eigen::Vector4d(-0.4, 0, 0, 1);
                break;
            }
            case 3:{
                link_pts.resize(4, 1);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                break;
            }
            case 4:{
                link_pts.resize(4, 1);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                break;
            }
            case 5:{
                link_pts.resize(4, 1);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                break;
            }
            default:
                break;
            }
            manipulator_link_pts_.push_back(link_pts);
        }
    }
}

/**
 * @brief 更新夹爪手指碰撞点位置 (仅 FastArmer)
 * @param gripper_close true=夹爪闭合 (手指内收, y偏移减小到 ±0.02), false=夹爪张开 (手指外展, y偏移=±0.06)
 *
 * 修改 manipulator_link_pts_[5] 的 col(5) 和 col(6) (两个手指的碰撞球心).
 *
 * @note 仅对 FastArmer 有效 (useFastArmer_=true), UR5 不处理
 */
void MMConfig::setGripperPoint(const bool gripper_close){
    if(!useFastArmer_) return;
    if(gripper_close){
        manipulator_link_pts_[5].col(5) = Eigen::Vector4d(0, 0.02, -0.01, 1);
        manipulator_link_pts_[5].col(6) = Eigen::Vector4d(0, -0.02, -0.01, 1);
    }else{
        manipulator_link_pts_[5].col(5) = Eigen::Vector4d(0, 0.06, -0.00, 1);
        manipulator_link_pts_[5].col(6) = Eigen::Vector4d(0, -0.06, -0.00, 1);
    }
}

/**
 * @brief 发布底盘碰撞检测球 (SPHERE_LIST) 的 RViz Marker
 *
 * 球半径 = mobile_base_check_radius_ × 2 (ROS Marker 用直径)
 * 球颜色 = color_set_[0] (红色)
 *
 * @param state 基底位姿 [x, y, yaw]
 */
void MMConfig::visCarCheckBall(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &state){
    std::vector<Eigen::Vector3d> car_pts;
    getCarPts(state, car_pts, Eigen::Vector3d::Zero());

    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = node_->now();
    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.id = idx;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = 1.0;
    sphere.color.g = 0;
    sphere.color.b = 0;
    sphere.color.r = color_set_[0](0);
    sphere.color.g = color_set_[0](1);
    sphere.color.b = color_set_[0](2);
    sphere.color.a = alpha > 1e-5 ? alpha : 1.0;
    sphere.scale.x = mobile_base_check_radius_ * 2.0;
    sphere.scale.y = mobile_base_check_radius_ * 2.0;
    sphere.scale.z = mobile_base_check_radius_ * 2.0;
    geometry_msgs::msg::Point pt;
    for (unsigned int i = 0; i < car_pts.size(); i++){
      pt.x = car_pts[i](0);
      pt.y = car_pts[i](1);
      pt.z = car_pts[i](2);
      sphere.points.push_back(pt);
    }
    pub->publish(sphere);
}

/**
 * @brief 发布机械臂碰撞检测球的 RViz Marker
 *
 * 每个关节的连杆点使用不同的颜色 (color_set_[i+1]), 逐关节 publish.
 * 球半径 = manipulator_thickness_ × 2 (ROS Marker 用直径)
 * Marker ID = idx + 1000 + joint_num (避免与底盘碰撞球冲突)
 *
 * @param car_state   基底位姿 [x, y, yaw]
 * @param joint_state 机械臂关节角
 */
void MMConfig::visManiCheckBall(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state){
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = node_->now();
    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.id = idx + 1000;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = 1.0;
    sphere.color.g = 0;
    sphere.color.b = 0;
    sphere.color.a = alpha > 1e-5 ? alpha : 1.0;
    sphere.scale.x = manipulator_thickness_ * 2.0;
    sphere.scale.y = manipulator_thickness_ * 2.0;
    sphere.scale.z = manipulator_thickness_ * 2.0;
    geometry_msgs::msg::Point pt;
    
    Eigen::Matrix4d T_q;
    T_q << cos(car_state(2)), -sin(car_state(2)), 0.0, car_state(0),
            sin(car_state(2)),  cos(car_state(2)), 0.0, car_state(1),
            0.0              , 0.0               , 1.0, 0.0,
            0.0              , 0.0               , 0.0, 1.0;
    Eigen::Matrix4d T_now = T_q * T_q_0_;
    Eigen::Vector3d pt_on_link;

    for(int i = 0; i < manipulator_dof_; ++i){
        sphere.color.r = color_set_[i + 1](0);
        sphere.color.g = color_set_[i + 1](1);
        sphere.color.b = color_set_[i + 1](2);
        ++sphere.id;
        Eigen::Matrix4d temp, temp_grad;
        getAJointTran(i, joint_state(i), temp, temp_grad);
        T_now = T_now * temp;
        int pts_size = manipulator_link_pts_[i].cols();
        for(int j = 0; j < pts_size; ++j){
            pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);
            pt.x = pt_on_link(0);
            pt.y = pt_on_link(1);
            pt.z = pt_on_link(2);
            sphere.points.push_back(pt);
        }
        pub->publish(sphere);
        sphere.points.clear();
    }
    
}

/**
 * @brief 生成底盘的 MESH_RESOURCE Marker (仅车身网格)
 *
 * Marker ID = idx * vis_idx_size_ + 0 (vis_idx_size_ = 100, 为其他部分预留空间)
 */
visualization_msgs::msg::MarkerArray MMConfig::getCarMarkerArray(std::string ns, int idx, double alpha, const Eigen::Vector3d &state){
    Eigen::Matrix2d R;
    visualization_msgs::msg::MarkerArray marker_array;

    R << cos(state(2)), -sin(state(2)),
            sin(state(2)),  cos(state(2));
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    pos.head(2) = state.head(2);
    pos(2) = 0.06;
    T.block(0, 0, 2, 2) = R;
    T.block(0, 3, 3, 1) = pos;
    marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 0, ns, alpha, T, mesh_resource_mobile_base_));
    
    return marker_array;
}

/**
 * @brief 生成机械臂各连杆 + 夹爪的 MESH_RESOURCE Marker Array
 *
 * 运动学链 (FastArmer):
 *   T_car → T_q_0_ → R_z(-π/2) → base0 → link1 → link2 → link3 → link4 → link5 → link6 → gripper_base → gripper_left/right
 *
 * 运动学链 (UR5):
 *   T_car → T_q_0_ → base → shoulder → upperarm → forearm → wrist1 → wrist2 → wrist3
 *
 * 每个连杆 Marker:
 *   - ID = idx * vis_idx_size_ + 11..20 (11 起偏移, 为底盘预留)
 *   - Mesh 文件路径在 setParam() 中从参数服务器加载
 *   - gripper_close=true 时夹爪绕 X 轴旋转 ±30°
 *
 * @note FastArmer 和 UR5 的运动学链差异很大, 由两个独立分支处理
 * @note getMarker() 最终生成单个 MESH_RESOURCE Marker
 */
visualization_msgs::msg::MarkerArray MMConfig::getManiMarkerArray(std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state, const bool &gripper_close){
    Eigen::VectorXd theta = joint_state;
    visualization_msgs::msg::MarkerArray marker_array;

    Eigen::Matrix4d T_q;
    T_q << cos(car_state(2)), -sin(car_state(2)), 0.0, car_state(0),
            sin(car_state(2)),  cos(car_state(2)), 0.0, car_state(1),
            0.0              , 0.0               , 1.0, 0.0,
            0.0              , 0.0               , 0.0, 1.0;

    Eigen::Matrix4d T_now = T_q * T_q_0_;

    if(useFastArmer_){
        Eigen::Matrix4d T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, -M_PI_2);
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 11, ns, alpha, T_now, mesh_resource_fastarmer_base0_));

        
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, -M_PI_2);
        T_temp(2, 3) = manipulator_config_(0);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(0));
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 12, ns, alpha, T_now, mesh_resource_fastarmer_link1_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(-M_PI_2, 0, 0);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(1));
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 13, ns, alpha, T_now, mesh_resource_fastarmer_link2_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, -M_PI_2);
        T_temp(0, 3) = manipulator_config_(1);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(2));
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 14, ns, alpha, T_now, mesh_resource_fastarmer_link3_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(M_PI_2, 0, -M_PI);
        T_temp(0, 3) = 0.0650000000000004;
        T_temp(1, 3) = -manipulator_config_(3);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(3));
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 15, ns, alpha, T_now, mesh_resource_fastarmer_link4_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(M_PI_2, 0, M_PI_2);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(4)); // theta(4)
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 16, ns, alpha, T_now, mesh_resource_fastarmer_link5_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(-M_PI_2, 0, 0);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(5)); // theta(5)
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 17, ns, alpha, T_now, mesh_resource_fastarmer_link6_));
        
        T_temp.setIdentity();
        T_temp.block(0, 3, 3, 1) << 0.005, 0.005, 0.061;
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 18, ns, alpha, T_now, mesh_resource_gripper_base_));

        Eigen::Matrix4d T_gripper = T_now;

        T_temp.setIdentity();
        T_temp.block(0, 3, 3, 1) << -0.003, 0.035, 0.061;
        T_now = T_gripper * T_temp;

        if(gripper_close){
            T_temp.setZero();
            T_temp(3, 3) = 1.0;
            T_temp.block(0, 0, 3, 3) = euler2rotation(M_PI / 6, 0, 0); // theta(4)
            T_now = T_now * T_temp;
        }
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 19, ns, alpha, T_now, mesh_resource_gripper_left_));

        T_temp.setIdentity();
        T_temp.block(0, 3, 3, 1) << -0.003, -0.035, 0.061;
        T_now = T_gripper * T_temp;

        if(gripper_close){
            T_temp.setZero();
            T_temp(3, 3) = 1.0;
            T_temp.block(0, 0, 3, 3) = euler2rotation(-M_PI / 6, 0, 0); // theta(4)
            T_now = T_now * T_temp;
        }
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 20, ns, alpha, T_now, mesh_resource_gripper_right_));
        
    }else{
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 11, ns, alpha, T_now, mesh_resource_ur5_base_));

        Eigen::Matrix4d T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(0));
        T_temp(2, 3) = manipulator_config_(0);
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 12, ns, alpha, T_now, mesh_resource_ur5_shoulder_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, M_PI_2, 0);
        T_temp(1, 3) = 0.138;
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, theta(1), 0);
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 13, ns, alpha, T_now, mesh_resource_ur5_upperarm_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, 0);
        T_temp(1, 3) = -0.131;
        T_temp(2, 3) = manipulator_config_(1);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, theta(2), 0);
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 14, ns, alpha, T_now, mesh_resource_ur5_forearm_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, M_PI_2, 0);
        T_temp(2, 3) = manipulator_config_(2);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, theta(3), 0); // theta(3)
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 15, ns, alpha, T_now, mesh_resource_ur5_wrist1_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, 0);
        T_temp(1, 3) = manipulator_config_(3);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(4)); // theta(4)
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 16, ns, alpha, T_now, mesh_resource_ur5_wrist2_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, 0);
        T_temp(2, 3) = manipulator_config_(4);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, theta(5), 0); // theta(5)
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 17, ns, alpha, T_now, mesh_resource_ur5_wrist3_));
    }
    

    return marker_array;
}

/**
 * @brief 发布单个 MESH_RESOURCE Marker (直接发布, 不返回)
 * @param id    Marker ID
 * @param ns    命名空间
 * @param alpha 透明度 (>=0 时设置, <0 保持默认)
 * @param color_rgb 未使用 (marker 使用 mesh 自带的材质)
 * @param T      4×4 齐次变换 (位姿来源)
 * @param mesh_file 网格文件 URI (file://...)
 *
 * @note 由于 mesh_use_embedded_materials=true, 颜色由 STL/DAE 自带材质决定
 * @note 特殊处理: id=2 或 102 时 scale 设为 0.001 (可能是某些特定微调标记)
 */
void MMConfig::visMesh(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, int id, std::string ns, double alpha, Eigen::Vector3d color_rgb, const Eigen::Matrix4d &T, const std::string &mesh_file){
    Eigen::Matrix3d rotation_matrix = T.block(0, 0, 3, 3);
    Eigen::Quaterniond quad;
    quad = rotation_matrix;
    visualization_msgs::msg::Marker meshMarker;
    meshMarker.header.frame_id = "world";
    meshMarker.header.stamp = node_->now();
    meshMarker.ns = ns;
    meshMarker.id = id;
    meshMarker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    meshMarker.action = visualization_msgs::msg::Marker::ADD;
    meshMarker.mesh_use_embedded_materials = true;
    meshMarker.pose.position.x = T(0, 3);
    meshMarker.pose.position.y = T(1, 3);
    meshMarker.pose.position.z = T(2, 3);
    meshMarker.pose.orientation.w = quad.w();
    meshMarker.pose.orientation.x = quad.x();
    meshMarker.pose.orientation.y = quad.y();
    meshMarker.pose.orientation.z = quad.z();
    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;
    if(id == 2 || id == 102){
        meshMarker.scale.x = 0.001;
        meshMarker.scale.y = 0.001;
        meshMarker.scale.z = 0.001;
    }
    // meshMarker.color.r = color_rgb(0);
    // meshMarker.color.g = color_rgb(1);
    // meshMarker.color.r = color_rgb(2);
    if(alpha >= 0.0)
        meshMarker.color.a = alpha;
    meshMarker.mesh_resource = mesh_file;
    pub->publish(meshMarker);
}

/**
 * @brief 构造单个 MESH_RESOURCE Marker (返回, 用于 MarkerArray 组装)
 * @param id    Marker ID
 * @param ns    命名空间
 * @param alpha 透明度
 * @param T      4×4 齐次变换 (提取位置和旋转)
 * @param mesh_file 网格文件 URI
 * @return 配置完成的 Marker (类型 MESH_RESOURCE, 使用嵌入材质)
 *
 * 从 T 提取:
 *   - position: T(0,3), T(1,3), T(2,3)
 *   - orientation: 从旋转矩阵 (3×3) 转换为四元数
 */
visualization_msgs::msg::Marker MMConfig::getMarker(int id, std::string ns, double alpha, const Eigen::Matrix4d &T, const std::string &mesh_file){
    Eigen::Matrix3d rotation_matrix = T.block(0, 0, 3, 3);
    Eigen::Quaterniond quad;
    quad = rotation_matrix;
    visualization_msgs::msg::Marker meshMarker;
    meshMarker.header.frame_id = "world";
    meshMarker.header.stamp = node_->now();
    meshMarker.ns = ns;
    meshMarker.id = id;
    meshMarker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    meshMarker.action = visualization_msgs::msg::Marker::ADD;
    meshMarker.mesh_use_embedded_materials = true;
    meshMarker.pose.position.x = T(0, 3);
    meshMarker.pose.position.y = T(1, 3);
    meshMarker.pose.position.z = T(2, 3);
    meshMarker.pose.orientation.w = quad.w();
    meshMarker.pose.orientation.x = quad.x();
    meshMarker.pose.orientation.y = quad.y();
    meshMarker.pose.orientation.z = quad.z();
    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;
    if(alpha >= 0.0)
        meshMarker.color.a = alpha;
    meshMarker.mesh_resource = mesh_file;
    return meshMarker;
}

}
