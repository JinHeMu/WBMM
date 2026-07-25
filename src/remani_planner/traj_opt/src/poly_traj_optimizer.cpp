/**
 * @file poly_traj_optimizer.cpp
 * @brief 后端轨迹优化器实现 — 时空联合优化的核心引擎
 *
 * ============================================================
 * 核心职责: 接收前端 MINCO 初值 → L-BFGS 优化 → 输出最优轨迹
 * ============================================================
 *
 * 代价函数 J = w₁·J_obs_car(底盘避障) + w₂·J_obs_mani(机械臂避障)
 *            + w₃·J_self(自碰撞) + w₄·J_feas_car(底盘可行性)
 *            + w₅·J_feas_joint(关节可行性) + w₆·J_time(时间代价)
 *            + J_snap(平滑性)
 *
 * 优化变量: 每段 MINCO 的中间点位置 + 段时间分配 + 换向点位置/角度
 * 求解器: L-BFGS (无约束优化, 通过虚拟时间映射处理时间正约束)
 *
 * @see planner_manager.cpp (调用者)
 * @see lbfgs.hpp (L-BFGS 求解器)
 */

#include "optimizer/poly_traj_optimizer.hpp"
#include <chrono>
#include <thread>

namespace remani_planner
{
  template<typename T>
  static void getOptParam(const rclcpp::Node::SharedPtr &node, const std::string &name,
                          T &value, const T &default_value) {
    if (!node->has_parameter(name)) {
      node->declare_parameter<T>(name, default_value);
    }
    node->get_parameter(name, value);
  }
  // ============================================================================
  // 参数加载与子模块初始化
  // ============================================================================
  void PolyTrajOptimizer::setParam(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<GridMap> &map, const std::shared_ptr<MMConfig> &mm_config){
    node_ = node;
    grid_map_ = map;
    map_resolution_ = grid_map_->getResolution();

    mm_config_ = mm_config;
    max_vel_ = mm_config_->getBaseMaxVel();             // 基底最大线速度 (m/s)
    max_acc_ = mm_config_->getBaseMaxAcc();             // 基底最大加速度 (m/s²)
    manipulator_config_ = mm_config_->getManiConfig();  // 机械臂配置
    T_q_0_ = mm_config_->getTq0();                      // 基底到机械臂基座的变换矩阵
    manipulator_link_pts_ = mm_config_->getLinkPoint();  // 各连杆上的碰撞球体位置

    /* ---------- 优化参数 ---------- */
    getOptParam(node_, "optimization.constrain_points_perPiece", cps_num_prePiece_, -1);
    getOptParam(node_, "optimization.weight_obstacle", wei_obs_, -1.0);
    getOptParam(node_, "optimization.weight_base_feasibility", wei_feas_, -1.0);
    getOptParam(node_, "optimization.weight_time", wei_time_, -1.0);
    wei_mani_obs_ = wei_obs_ / 5.0;                      // 机械臂避障权重 (通常比底盘低)
    getOptParam(node_, "optimization.weight_manipulator_self", wei_mani_self_, -1.0);
    getOptParam(node_, "optimization.weight_manipulator_feasibility", wei_mani_feas_, -1.0);

    getOptParam(node_, "optimization.dense_sample_resolution", dense_sample_resolution_, -1);

    /* ---------- 安全距离参数 ---------- */
    getOptParam(node_, "optimization.safe_margin", safe_margin_, 0.1);
    getOptParam(node_, "optimization.safe_margin_mani", safe_margin_mani_, 0.1);
    getOptParam(node_, "optimization.self_safe_margin", self_safe_margin_, 0.1);
    getOptParam(node_, "optimization.ground_safe_dis", ground_safe_dis_, 0.1);
    getOptParam(node_, "optimization.ground_safe_margin", ground_safe_margin_, 0.1);
    getOptParam(node_, "optimization.mobile_base_opt_gear", opt_gear_, true);

    /* ---------- 机器人参数 ---------- */
    getOptParam(node_, "mm.mobile_base_dof", mobile_base_dof_, -1);
    getOptParam(node_, "mm.mobile_base_non_singul_vel", non_singul_v_, -1.0);
    getOptParam(node_, "mm.mobile_base_max_wheel_omega", max_wheel_omega_, -1.0);
    getOptParam(node_, "mm.mobile_base_max_wheel_alpha", max_wheel_alpha_, -1.0);
    getOptParam(node_, "mm.mobile_base_wheel_base", mobile_base_wheel_base_, -1.0);
    getOptParam(node_, "mm.mobile_base_wheel_radius", mobile_base_wheel_radius_, -1.0);
    getOptParam(node_, "mm.mobile_base_length", mobile_base_length_, -1.0);
    getOptParam(node_, "mm.mobile_base_width", mobile_base_width_, -1.0);
    getOptParam(node_, "mm.mobile_base_height", mobile_base_height_, -1.0);
    getOptParam(node_, "mm.mobile_base_check_radius", mobile_base_check_radius_, -1.0);

    getOptParam(node_, "mm.manipulator_dof", manipulator_dof_, -1);
    getOptParam(node_, "mm.manipulator_thickness", manipulator_thickness_, -1.0);

    /* ---------- 关节限位 ---------- */
    std::vector<double> joint_pos_limit;
    getOptParam(node_, "mm.manipulator_min_pos", joint_pos_limit, std::vector<double>{});
    min_joint_pos_.resize(joint_pos_limit.size());
    for(unsigned int i = 0; i < joint_pos_limit.size(); i++){
        min_joint_pos_(i) = joint_pos_limit[i];
    }
    joint_pos_limit.clear();
    getOptParam(node_, "mm.manipulator_max_pos", joint_pos_limit, std::vector<double>{});
    max_joint_pos_.resize(joint_pos_limit.size());
    for(unsigned int i = 0; i < joint_pos_limit.size(); i++){
        max_joint_pos_(i) = joint_pos_limit[i];
    }
    getOptParam(node_, "mm.manipulator_max_vel", max_joint_vel_, -1.0);
    getOptParam(node_, "mm.manipulator_max_acc", max_joint_acc_, -1.0);

    firs_plot_ = true;

    /* ---------- 辅助矩阵 ---------- */
    // B_h_ = [0, -1; 1, 0] — 用于计算角速度 ω = (aᵀ·B·v) / (vᵀ·v)
    B_h_ << 0.0, -1.0,
            1.0,  0.0;
    C_h_ = Eigen::MatrixXd::Zero(3, 4);
    C_h_.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

    /* ---------- 前端搜索器 (KinoA*) ---------- */
    kino_a_star_.reset(new KinoAstar);
    kino_a_star_->setParam(node_, map, mm_config);

    traj_dim_ = mobile_base_dof_ + manipulator_dof_;

    /* ---------- 可视化发布者 ---------- */
    traj_pt_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("traj_pt_vis", 10);
    traj_init_pt_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("traj_init_pt_vis", 10);
    front_end_mm_mesh_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("front_end_mm_mesh_vis", 50);
    back_end_mm_mesh_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("back_end_mm_mesh_vis", 50);
  }

  // ============================================================================
  // 主优化入口: 接收前端初值 → 调用 L-BFGS 优化 → 输出优化后轨迹
  //
  // 优化变量组成 (按顺序):
  //   [1] 各段 MINCO 的中间点       (traj_dim_ × (piece_num-1)) × traj_num_
  //   [2] 各段的虚拟时间分配          piece_num × traj_num_
  //   [3] 段间换向点的位置 (Gear)     traj_dim_ × (traj_num_-1)
  //   [4] 段间换向点的角度 (Angle)    1 × (traj_num_-1)
  //
  // 关键技术: 虚拟时间映射 (RealT2VirtualT/VirtualT2RealT)
  //   将严格为正的时间 T > 0 映射到无约束的虚拟时间 τ ∈ ℝ,
  //   使 L-BFGS 可以自由优化时间而不违反约束.
  //
  // @param iniState_container    各段的起始边界状态 (pos, vel, acc, jerk)
  // @param finState_container    各段的终止边界状态
  // @param initInnerPts_container 各段的中间点 (前端 A* 的输出)
  // @param initT_container        各段的初始时间分配
  // @param singul_container       各段的前进/后退标志
  // @param optCps_container       [输出] 优化后的控制点
  // @param optWps_container       [输出] 优化后的路径点
  // @param optT_container         [输出] 优化后的时间分配
  // @return true 优化成功且轨迹安全
  // ============================================================================
  bool PolyTrajOptimizer::OptimizeTrajectory_lbfgs(
      const std::vector<Eigen::MatrixXd> &iniState_container,
      const std::vector<Eigen::MatrixXd> &finState_container,
      const std::vector<Eigen::MatrixXd> &initInnerPts_container,
      const std::vector<Eigen::VectorXd> &initT_container,
      const std::vector<int> &singul_container,
      std::vector<Eigen::MatrixXd> &optCps_container,
      std::vector<Eigen::MatrixXd> &optWps_container,
      std::vector<Eigen::VectorXd> &optT_container,
      std::vector<Eigen::VectorXd> &optEECps_container){

    traj_num_ = initInnerPts_container.size();   // 轨迹段数 (每段一个前进方向)
    iniState_container_ = iniState_container;
    finState_container_ = finState_container;
    singul_container_ = singul_container;
    variable_num_ = 0;
    SnapOpt_container_.clear();
    piece_num_container_.clear();
    SnapOpt_container_.resize(traj_num_);
    piece_num_container_.resize(traj_num_);
    if (traj_num_ != (int)initT_container.size()){
      RCLCPP_ERROR(node_->get_logger(), "traj_num_ != initT_container.size()");
      return false;
    }

    // ---- Step 1: 解析段数和总变量维数 ----
    int piece_num;
    int piece_num_all = 0;
    for(int i = 0; i < traj_num_; i++){
      if(initInnerPts_container[i].cols()==0){
        RCLCPP_ERROR(node_->get_logger(), "There is only a piece?");
        return false;
      }
      piece_num = initInnerPts_container[i].cols() + 1;  // 中间点数量+1 = 多项式段数
      piece_num_container_[i] = piece_num;
      piece_num_all += piece_num;

      // 钳制起始/终止速度/加速度到限制内
      if(iniState_container_[i].col(1).head(2).norm() >= max_vel_){
        iniState_container_[i].col(1).head(2) = iniState_container_[i].col(1).head(2).normalized() * (max_vel_ - 1.0e-2);
      }
      if(finState_container_[i].col(1).head(2).norm() >= max_vel_){
        finState_container_[i].col(1).head(2) = finState_container_[i].col(1).head(2).normalized() * (max_vel_ - 1.0e-2);
      }
      if(iniState_container_[i].col(2).head(2).norm() >= max_acc_){
        iniState_container_[i].col(2).head(2) = iniState_container_[i].col(2).head(2).normalized() * (max_acc_ - 1.0e-2);
      }
      if(finState_container_[i].col(2).head(2).norm() >= max_acc_){
        finState_container_[i].col(2).head(2) = finState_container_[i].col(2).head(2).normalized() * (max_acc_ - 1.0e-2);
      }

      // 创建 MINCO 优化器
      SnapOpt_container_[i].reset(piece_num);
      variable_num_ += traj_dim_ * (piece_num - 1);  // 中间点变量
      variable_num_ += piece_num;                     // 时间分配变量
    }

    variable_num_ += traj_dim_ * (traj_num_ - 1);  // 换向点位置变量 (Gear)
    variable_num_ += 1 * (traj_num_ - 1);           // 换向点角度变量 (Angle)

    // ---- Step 2: 组装初始优化变量 x ----
    Eigen::VectorXd x;
    x.resize(variable_num_);
    int offset = 0;

    // [2.1] 中间点 (inner points)
    for(int i = 0; i < traj_num_; i++){
      memcpy(x.data() + offset, initInnerPts_container[i].data(),
             initInnerPts_container[i].size() * sizeof(x[0]));
      offset += initInnerPts_container[i].size();
    }

    // [2.2] 时间分配 → 映射到虚拟时间
    Eigen::VectorXd initT;
    int offset_temp = 0;
    initT.resize(piece_num_all);
    for(int i = 0; i < traj_num_; i++){
      memcpy(x.data() + offset, initT_container[i].data(),
             initT_container[i].size() * sizeof(x[0]));
      memcpy(initT.data() + offset_temp, initT_container[i].data(),
             initT_container[i].size() * sizeof(initT[0]));
      offset += initT_container[i].size();
      offset_temp += initT_container[i].size();
    }
    Eigen::Map<Eigen::VectorXd> Vt(x.data() + offset - piece_num_all, piece_num_all);
    RealT2VirtualT(initT, Vt);  // 真实时间 → 无约束虚拟时间

    // [2.3] 换向点位置 (终止状态的位置作为换向点初值)
    for(int i = 0; i < traj_num_ - 1; i++){
      memcpy(x.data() + offset, finState_container_[i].col(0).data(),
             traj_dim_ * sizeof(x[0]));
      offset += traj_dim_;
    }

    // [2.4] 换向点角度 (从终止速度方向计算)
    Eigen::Map<Eigen::VectorXd> angles(x.data() + offset, traj_num_ - 1);
    for(int i = 0; i < traj_num_ - 1; i++){
      Eigen::Vector2d gearv = finState_container_[i].col(1).head(2);
      angles[i] = std::atan2(gearv[1], gearv[0]);
    }

    // ---- Step 3: 配置 L-BFGS 参数 ----
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 256;         // 内存大小 (历史梯度数量)
    lbfgs_params.g_epsilon = 0.0;        // 梯度终止阈值
    lbfgs_params.delta = 1e-3;           // 优化变量变化量用于有限差分
    lbfgs_params.max_iterations = 1000;  // 最大迭代次数

    double final_cost;
    iter_num_ = 0;
    force_stop_type_ = DONT_STOP;

    // ---- Step 4: 执行 L-BFGS 优化 ----
    int result = lbfgs::lbfgs_optimize(
        x,
        final_cost,
        PolyTrajOptimizer::costFunctionCallback,  // 代价函数 + 梯度
        NULL,                                      // 无额外过程数据
        PolyTrajOptimizer::earlyExitCallback,      // 提前退出回调
        this,                                      // 实例指针 (传给 callback)
        lbfgs_params);

    // ---- Step 5: 结果有效性检查 ----
    if((result != lbfgs::LBFGS_CONVERGENCE)&&(result != lbfgs::LBFGS_STOP)&&(result != lbfgs::LBFGSERR_MAXIMUMLINESEARCH)){
      RCLCPP_ERROR(node_->get_logger(), "The optimization result is : %s", lbfgs::lbfgs_strerror(result));
    }else if(result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH){
      RCLCPP_WARN(node_->get_logger(), "The optimization result is : %s", lbfgs::lbfgs_strerror(result));
    }else{
      RCLCPP_INFO(node_->get_logger(), "The optimization result is : %s", lbfgs::lbfgs_strerror(result));
    }

    // 构建完整轨迹做安全检查
    SingulTrajData singul_traj_data;
    double traj_start_time = 0;
    singul_traj_data.clearSingulTraj();
    for(unsigned int i = 0; i < singul_container.size(); ++i){
      singul_traj_data.addSingulTraj(SnapOpt_container_[i].getTraj(singul_container[i]), traj_start_time);
      traj_start_time = singul_traj_data.singul_traj.back().end_time;
    }
    bool good_traj = true;
    good_traj = IsTrajSafe(singul_traj_data);  // 碰撞+可行性检查

    if(result == lbfgs::LBFGSERR_INVALID_FUNCVAL){
      return false;  // 函数求值无效 → 直接失败
    }

    // ---- Step 6: 输出优化结果 ----
    optCps_container.clear();
    optWps_container.clear();
    optT_container.clear();
    optEECps_container.clear();
    for(int i = 0; i < traj_num_; i++){
      optCps_container.push_back(cps_container_[i].points);                        // 控制点
      optWps_container.push_back(SnapOpt_container_[i].getInitConstrainPoints(1)); // 路径点
      optT_container.push_back(SnapOpt_container_[i].get_T1());                    // 段时间
    }

    return good_traj;
  }

  // ============================================================================
  // 可视化: 后端优化后的轨迹碰撞球体
  // ============================================================================
  void PolyTrajOptimizer::displayBackEndMesh(const SingulTrajData &traj_data, bool init, bool gripper_close){
    Eigen::Vector3d car_state;
    Eigen::VectorXd joint_state;
    joint_state.resize(manipulator_dof_);

    visualization_msgs::msg::MarkerArray marker_array, marker_array_all, marker_array_i;
    visualization_msgs::msg::Marker marker_delete_all;
    marker_delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array_all.markers.push_back(marker_delete_all);

    Eigen::VectorXd pos, vel;
    int i = 0;
    double yaw;
    double t_step = 0.5 / max_vel_;
    double duration = traj_data.duration;
    for(double t = 0; t < duration - 1e-3; t += t_step, ++i){
      pos = traj_data.getPos(t);
      vel = traj_data.getVel(t);
      yaw = traj_data.getCarAngle(t);

      car_state.head(2) = pos.head(2);
      car_state(2) = yaw;
      joint_state = pos.tail(manipulator_dof_);
      mm_config_->getMMMarkerArray(marker_array, "vis_mm_back_end", i, 0.17, car_state, joint_state, gripper_close);
      marker_array_all.markers.insert(marker_array_all.markers.end(), marker_array.markers.begin(), marker_array.markers.end());
      back_end_mm_mesh_vis_pub_->publish(marker_array_all);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  // ============================================================================
  // 平滑 L1 惩罚函数 (Smoothed L1)
  //
  // 对约束违反量 x 进行平滑近似, 使梯度可导:
  //   x ≤ 0:        f = 0, df = 0       (无违反)
  //   0 < x ≤ μ:    三次平滑过渡
  //   x > μ:        f = x - μ/2, df = 1 (线性惩罚)
  //
  // @param x  约束违反量 (penalty = constraint - limit)
  // @param mu 平滑参数 (过渡区宽度)
  // @param f  [输出] 惩罚函数值
  // @param df [输出] 导数
  // @return true 表示违反约束 (需要加入代价)
  // ============================================================================
  bool PolyTrajOptimizer::smoothedL1(const double &x, const double &mu, double &f, double &df){
    if (x < 0.0)
    {
      f=0;
      df=0;
      return false;  // 无违反
    }
    else if (x > mu)
    {
      f = x - 0.5 * mu;  // 线性区
      df = 1.0;
      return true;
    }
    else
    {
      // 三次平滑过渡区
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      f = mumxd2 * sqrxdmu * xdmu;
      df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
      return true;
    }
  }

  // ============================================================================
  // 平滑三次惩罚: f = max(x, 0)³ 的平滑近似
  // ============================================================================
  bool PolyTrajOptimizer::smoothedMax3(const double &x, double &f, double &df){
    if (x < 0.0)
    {
      f=0;
      df=0;
      return false;
    }else{
      f = x * x * x;
      df = 3.0 * x * x;
      return true;
    }
  }

  // ============================================================================
  // 平滑 Log 函数 (用于近似阶跃/指示函数)
  //
  // 从 0 → 1 的平滑过渡, 支持梯度计算.
  // 用于最小速度惩罚中的"密集采样开关" (只有在低速度区才激活密集采样).
  // ============================================================================
  bool PolyTrajOptimizer::smoothedLog(const double &x, const double &mu, double &l, double &grad){
    if (x <= -mu){
      l = 0.0;
      grad = 0.0;
      return false;
    }else if (x <= 0){
      double e1 = x + mu;
      double e1_2 = e1 * e1;
      double mu_4 = 1.0 / (mu * mu * mu * mu);
      l = 0.5 * e1_2 * e1 * (mu - x) * mu_4;
      grad = e1_2 * (mu - 2 * x) * mu_4;
      return true;
    }else if(x <= mu){
      double e2 = x - mu;
      double e2_2 = e2 * e2;
      double mu_4 = 1.0 / (mu * mu * mu * mu);
      l = 0.5 * (x + mu) * e2_2 * e2 * mu_4 + 1.0;
      grad = e2_2 * (mu + 2 * x) * mu_4;
      return true;
    }else{
      l = 1.0;
      grad = 0.0;
      return true;
    }
  }

  // ============================================================================
  // 可行性检查: 在时间 t 检查轨迹是否满足运动学约束
  //
  // 检查项:
  //   1. 底盘: 左右轮转速、左右轮角加速度 (基于差速轮模型)
  //   2. 机械臂: 关节速度、关节加速度
  //
  // @return true 表示不满足可行性
  // ============================================================================
  bool PolyTrajOptimizer::IsNotFeasibie(const SingulTrajData &traj_data, double t){
    Eigen::VectorXd vel, acc, jer;
    double pen;
    double feas_tol_percent_ = 0.05;  // 5% 容忍度

    vel = traj_data.getVel(t).head(2);
    acc = traj_data.getAcc(t).head(2);
    jer = traj_data.getJer(t).head(2);
    int singul = traj_data.getSingul(t);

    // 差速轮运动学:
    //   ω = (aᵀ·B·v) / (vᵀ·v)                   — 航向角速度
    //   ω_left  = (2·s·|v| - b·ω) / (2·r)        — 左轮转速
    //   ω_right = (2·s·|v| + b·ω) / (2·r)        — 右轮转速
    //   α = (jᵀ·B·v - 2·aᵀ·B·v·aᵀ·v / |v|²) / |v|²  — 角加速度
    double aTv = acc.transpose() * vel;
    double aTBv = acc.transpose() * B_h_ * vel;
    double jTBv = jer.transpose() * B_h_ * vel;
    double v_norm = vel.norm();
    double vTv_inv = 1.0 / vel.squaredNorm();
    double vTv_inv2 = vTv_inv * vTv_inv;

    double omega = aTBv * vTv_inv;

    // 左轮转速检查
    double wheel_omega_left = (2.0 * singul * v_norm - mobile_base_wheel_base_ * omega) / (2 * mobile_base_wheel_radius_);
    pen =  wheel_omega_left * wheel_omega_left - max_wheel_omega_ * max_wheel_omega_ * (1.0 + feas_tol_percent_) * (1.0 + feas_tol_percent_);
    if(pen > 0.0){
      RCLCPP_WARN(node_->get_logger(), "%f max left wheel omega is not feasible at relative time %f! opt failed", wheel_omega_left, t );
      return true;
    }

    // 右轮转速检查
    double wheel_omega_right = (2.0 * singul * v_norm + mobile_base_wheel_base_ * omega) / (2 * mobile_base_wheel_radius_);
    pen =  wheel_omega_right * wheel_omega_right - max_wheel_omega_ * max_wheel_omega_ * (1.0 + feas_tol_percent_) * (1.0 + feas_tol_percent_);
    if(pen > 0.0){
      RCLCPP_WARN(node_->get_logger(), "%f max right wheel omega is not feasible at relative time %f! opt failed", wheel_omega_right, t );
      return true;
    }

    double alpha = jTBv * vTv_inv - 2.0 * aTBv * aTv * vTv_inv2;

    // 左轮角加速度检查
    double wheel_alpha_left = (2.0 * singul * aTv / v_norm - mobile_base_wheel_base_ * alpha) / (2 * mobile_base_wheel_radius_);
    pen =  wheel_alpha_left * wheel_alpha_left - max_wheel_alpha_ * max_wheel_alpha_ * (1.0 + feas_tol_percent_) * (1.0 + feas_tol_percent_);
    if(pen > 0.0){
      RCLCPP_WARN(node_->get_logger(), "%f max left wheel alpha is not feasible at relative time %f! opt failed", wheel_alpha_left, t );
      return true;
    }

    // 右轮角加速度检查
    double wheel_alpha_right = (2.0 * singul * aTv / v_norm + mobile_base_wheel_base_ * alpha) / (2 * mobile_base_wheel_radius_);
    pen =  wheel_alpha_right * wheel_alpha_right - max_wheel_alpha_ * max_wheel_alpha_ * (1.0 + feas_tol_percent_) * (1.0 + feas_tol_percent_);
    if(pen > 0.0){
      RCLCPP_WARN(node_->get_logger(), "%f max right wheel alpha is not feasible at relative time %f! opt failed", wheel_alpha_right, t );
      return true;
    }

    // 机械臂关节速度/加速度检查
    vel = traj_data.getVel(t).tail(manipulator_dof_);
    acc = traj_data.getAcc(t).tail(manipulator_dof_);
    if(vel.lpNorm<Eigen::Infinity>() - max_joint_vel_ * (1.0 + feas_tol_percent_) > 0){
      RCLCPP_WARN(node_->get_logger(), "mani vel %f is not feasible at relative time %f! opt failed", vel.lpNorm<Eigen::Infinity>(), t);
      return true;
    }
    if(acc.lpNorm<Eigen::Infinity>() - max_joint_acc_ * (1.0 + feas_tol_percent_) > 0){
      RCLCPP_WARN(node_->get_logger(), "mani acc %f is not feasible at relative time %f! opt failed", acc.lpNorm<Eigen::Infinity>(), t);
      return true;
    }

    return false;
  }

  // ============================================================================
  // 整条轨迹安全性检查 (优化后调用)
  //
  // 沿轨迹密集采样, 检查每个时刻的:
  //   1. 碰撞 (底盘/机械臂/自碰撞)
  //   2. 运动学可行性
  // ============================================================================
  bool PolyTrajOptimizer::IsTrajSafe(const SingulTrajData &traj_data){
    double dt = 0.01;
    double T_all = traj_data.duration;
    int i_end = floor(T_all / dt);
    double t = 0.05;  // 跳过起点 (起点通常碰撞且尚未开始运动)
    int coll_type;
    for (int i = 5; i < i_end; i++){
      if(checkCollision(traj_data, t, coll_type)){
        if(coll_type == 0){
          RCLCPP_WARN(node_->get_logger(), "car collision at time %f!", t);
        }else if (coll_type == 1){
          RCLCPP_WARN(node_->get_logger(), "mani collision at time %f!", t);
        }else if (coll_type == 2){
          RCLCPP_WARN(node_->get_logger(), "car-mani collision at time %f!", t);
        }else if (coll_type == 3){
          RCLCPP_WARN(node_->get_logger(), "mani-mani collision at time %f!", t);
        }
        return false;
      }
      if(IsNotFeasibie(traj_data, t)){
        return false;
      }
      t += dt;
    }
    return true;
  }

  // ============================================================================
  // 单点碰撞检测 (封装 MMConfig 的 checkcollision)
  //
  // @param t  相对时间
  // @param coll_type [输出] 碰撞类型: 0=car-obs, 1=mani-obs, 2=car-mani, 3=mani-mani
  // ============================================================================
  bool PolyTrajOptimizer::checkCollision(const SingulTrajData &traj, double t, int &coll_type)
  {
    Eigen::VectorXd pos = traj.getPos(t);
    Eigen::VectorXd vel = traj.getVel(t);
    double yaw = traj.getCarAngle(t);
    return mm_config_->checkcollision(Eigen::Vector3d(pos(0), pos(1), yaw),
                                      pos.tail(manipulator_dof_), false, coll_type);
  }

  // ============================================================================
  // L-BFGS 代价函数回调 (static, 通过 instance 指针访问成员)
  //
  // 这是优化器的核心: 计算完整代价函数和解析梯度.
  //
  // 代价组成:
  //   total = J_snap (平滑性) + Σ(obs/feas 代价) + J_time (时间代价)
  //
  // 梯度计算: 通过 MINCO 的伴随方法 (chain rule via getGrad2TP)
  //   从控制点到优化变量的链式法则梯度传播
  // ============================================================================
  double PolyTrajOptimizer::costFunctionCallback(void *instance,
                                       const Eigen::VectorXd &x,
                                       Eigen::VectorXd &g)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(instance);
    double total_smoo_cost = 0, total_time_cost = 0;
    Eigen::VectorXd obs_feas_costs(6), total_obs_feas_costs(6);
    // obs_feas_costs: [0]=car-obs, [1]=mani-obs, [2]=self-coll, [3]=car-feas, [4]=joint-feas, [5]=unused
    obs_feas_costs.setZero();
    total_obs_feas_costs.setZero();
    int offset = 0;

    // ---- 解析优化变量: 将 x 分块映射到各段 ----

    // [1] 中间点 (P_container) / 对应的梯度 (gradP_container)
    std::vector<Eigen::Map<const Eigen::MatrixXd>> P_container;
    std::vector<Eigen::Map<Eigen::MatrixXd>> gradP_container;
    for(int trajid = 0; trajid < opt->traj_num_; trajid++){
      Eigen::Map<const Eigen::MatrixXd> P(x.data() + offset, opt->traj_dim_,
                                           opt->piece_num_container_[trajid] - 1);
      Eigen::Map<Eigen::MatrixXd>gradP(g.data() + offset, opt->traj_dim_,
                                        opt->piece_num_container_[trajid] - 1);
      offset += opt->traj_dim_ * (opt->piece_num_container_[trajid] - 1);
      gradP.setZero();
      P_container.push_back(P);
      gradP_container.push_back(gradP);
    }

    // [2] 虚拟时间 (t_container) → 真实时间 (T_container)
    std::vector<Eigen::VectorXd> T_container;
    std::vector<Eigen::VectorXd> gradT_container;
    std::vector<Eigen::Map<const Eigen::VectorXd>> t_container;
    std::vector<Eigen::Map<Eigen::VectorXd>> gradt_container;
    for(int trajid = 0; trajid < opt->traj_num_; trajid++){
      int piece_num = opt->piece_num_container_[trajid];
      Eigen::Map<const Eigen::VectorXd> t(x.data() + offset, piece_num);
      Eigen::Map<Eigen::VectorXd> gradt(g.data() + offset, piece_num);
      gradt.setZero();
      t_container.push_back(t);
      gradt_container.push_back(gradt);

      Eigen::VectorXd T(piece_num);
      opt->VirtualT2RealT(t, T);           // 虚拟时间 → 真实时间
      Eigen::VectorXd gradT(piece_num);
      gradT.setZero();
      T_container.push_back(T);
      gradT_container.push_back(gradT);

      offset += piece_num;
    }

    // [3] 换向点位置 (Gear)
    std::vector<Eigen::Map<const Eigen::MatrixXd>> Gear_container;
    std::vector<Eigen::Map<Eigen::MatrixXd>> gradGear_container;
    for(int trajid = 0; trajid < opt->traj_num_ - 1; trajid++){
      Eigen::Map<const Eigen::MatrixXd> Gear(x.data() + offset, opt->traj_dim_, 1);
      Eigen::Map<Eigen::MatrixXd>gradGear(g.data() + offset, opt->traj_dim_, 1);
      offset += opt->traj_dim_;
      gradGear.setZero();
      Gear_container.push_back(Gear);
      gradGear_container.push_back(gradGear);
    }

    // [4] 换向点角度 (Angles)
    Eigen::Map<const Eigen::VectorXd> Angles(x.data() + offset, opt->traj_num_ - 1);
    Eigen::Map<Eigen::VectorXd>gradAngles(g.data() + offset, opt->traj_num_ - 1);
    gradAngles.setZero();

    // ---- 对每段轨迹计算代价和梯度 ----
    for(int trajid = 0; trajid < opt->traj_num_; trajid++){
      Eigen::MatrixXd IniS, FinS;
      IniS = opt->iniState_container_[trajid];
      FinS = opt->finState_container_[trajid];

      // 如果不是第一段, 用换向点更新起始边界条件
      if(trajid > 0){
        double theta = Angles[trajid - 1];
        IniS.col(0) = Gear_container[trajid - 1];      // 位置 = 换向点
        IniS.col(1).head(2) = Eigen::Vector2d(
            -opt->non_singul_v_ * cos(theta),
            -opt->non_singul_v_ * sin(theta));          // 速度 = -v·[cosθ, sinθ] (后退)
      }
      // 如果不是最后一段, 用换向点更新终止边界条件
      if(trajid < opt->traj_num_ - 1){
        double theta = Angles[trajid];
        FinS.col(0) = Gear_container[trajid];           // 位置 = 换向点
        FinS.col(1).head(2) = Eigen::Vector2d(
            opt->non_singul_v_ * cos(theta),
            opt->non_singul_v_ * sin(theta));            // 速度 = +v·[cosθ, sinθ] (前进)
      }

      // 给定中间点和时间, 生成 MINCO 多项式
      opt->SnapOpt_container_[trajid].generate(P_container[trajid], T_container[trajid], IniS, FinS);

      // [a] 平滑性代价 (min-snap)
      double smoo_cost = 0;
      opt->initAndGetSmoothnessGradCost2PT(trajid, gradT_container[trajid], smoo_cost);

      // [b] 障碍物 + 可行性代价 (时间积分)
      obs_feas_costs.setZero();
      opt->addPVAJGradCost2CT(trajid, gradT_container[trajid], obs_feas_costs, opt->cps_num_prePiece_);

      total_smoo_cost += smoo_cost;
      total_obs_feas_costs += obs_feas_costs;
    }

    // ---- 通过链式法则传播梯度到优化变量 ----
    for(int trajid = 0; trajid < opt->traj_num_; trajid++){
      double time_cost = 0.0;
      Eigen::MatrixXd gradIni, gradFin;

      // MINCO 伴随: 从多项式系数梯度 → 中间点/时间/起止状态梯度
      opt->SnapOpt_container_[trajid].getGrad2TP(gradT_container[trajid],
                                                  gradP_container[trajid],
                                                  gradIni, gradFin);

      // 将起止状态梯度传播到换向点 (Gear/Angle) 变量
      if(opt->opt_gear_){
        if(trajid > 0){
          double theta = Angles[trajid - 1];
          gradGear_container[trajid - 1] += gradIni.col(0);
          gradAngles[trajid - 1] += gradIni.topRows(2).col(1).transpose()
                                  * Eigen::Vector2d(opt->non_singul_v_ * sin(theta),
                                                    -opt->non_singul_v_ * cos(theta));
        }
        if(trajid < opt->traj_num_ - 1){
          double theta = Angles[trajid];
          gradGear_container[trajid] += gradFin.col(0);
          gradAngles[trajid] += gradFin.topRows(2).col(1).transpose()
                              * Eigen::Vector2d(-opt->non_singul_v_ * sin(theta),
                                                 opt->non_singul_v_ * cos(theta));
        }
      }

      // [c] 时间代价 + 虚拟时间梯度映射
      opt->VirtualTGradCost(T_container[trajid], t_container[trajid],
                            gradT_container[trajid], gradt_container[trajid],
                            time_cost);
      total_time_cost += time_cost;
    }

    double costall = total_smoo_cost + total_obs_feas_costs.sum() + total_time_cost;
    opt->iter_num_ += 1;

    return costall;
  }

  // ============================================================================
  // L-BFGS 提前退出回调
  //
  // 当检测到错误/需要重新规划时, 提前终止优化
  // ============================================================================
  int PolyTrajOptimizer::earlyExitCallback(void *func_data,const Eigen::VectorXd &x,
                                    const Eigen::VectorXd &g,
                                    const double fx,
                                    const double step,
                                    const int k,
                                    const int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  // ============================================================================
  // 时间映射: 真实时间 → 无约束虚拟时间
  //
  // 公式: τ = f(T)
  //   T > 1:  τ = sqrt(2T - 1) - 1       (τ ∈ (0, ∞))
  //   T ≤ 1:  τ = 1 - sqrt(2/T - 1)      (τ ∈ (-∞, 0])
  //
  // 满足: T > 0 对任意 τ ∈ ℝ 成立
  // 使得 L-BFGS 可以自由优化 τ 而不需要约束 T > 0
  // ============================================================================
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  // ============================================================================
  // 时间映射: 无约束虚拟时间 → 真实时间 (逆映射)
  //
  // 公式: T = g(τ)
  //   τ > 0:  T = 0.5τ² + τ + 1
  //   τ ≤ 0:  T = 1 / (0.5τ² - τ + 1)
  // ============================================================================
  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  // ============================================================================
  // 虚拟时间梯度映射 + 时间代价
  //
  // 将真实时间的梯度通过链式法则映射到虚拟时间:
  //   dJ/dτ = (dJ/dT + w_time) × dT/dτ
  //
  // 时间代价: J_time = w_time × Σ(T_i)
  // ============================================================================
  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }
      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }
    costT = RT.sum() * wei_time_;
  }

  // ============================================================================
  // 平滑性代价与梯度初始化
  //
  // 调用 MINCO 的 initGradCost, 计算 min-snap 代价:
  //   J_snap = ∫ ||d⁴p/dt⁴||² dt
  //
  // 同时初始化多项式系数梯度 gdC (通过伴随方法)
  // ============================================================================
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(const int trajid, EIGENVEC &gdT, double &cost)
  {
    SnapOpt_container_[trajid].initGradCost(gdT, cost);
  }

  // ============================================================================
  // 添加 P-V-A-J 相关的代价和梯度 (时间积分)
  //
  // 在每个积分点上计算:
  //   1. 障碍物代价 (底盘 + 机械臂 + 自碰撞)
  //   2. 基底可行性代价 (差速轮运动学)
  //   3. 密集采样代价 (低速度区的额外可行性检查)
  //   4. 机械臂关节可行性代价 (位置/速度/加速度限位)
  //
  // 梯度通过链式法则: dJ/dc = Σ β_k × (∂J/∂q)ᵀ
  //   其中 β_k 是多项式基函数在采样点的值
  //   最终汇聚到多项式系数梯度 get_gdC()
  //
  // @param trajid  轨迹段ID
  // @param K       每段内的采样点数 (控制精度)
  // ============================================================================
  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAJGradCost2CT(const int trajid, EIGENVEC &gdT,
                                              Eigen::VectorXd &costs, const int &K){
    costs.setZero();
    int N = gdT.size();  // 多项式段数
    Eigen::VectorXd pos(traj_dim_), pos_next(traj_dim_),
                    vel(traj_dim_), acc(traj_dim_), jer(traj_dim_), snap(traj_dim_);
    Eigen::VectorXd dense_pos(traj_dim_), dense_vel(traj_dim_),
                    dense_acc(traj_dim_), dense_jer(traj_dim_), dense_sna(traj_dim_);
    Eigen::VectorXd gradp(traj_dim_), gradv(traj_dim_), grada(traj_dim_), gradj(traj_dim_),
                    gradp_traj_piece(traj_dim_);
    Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
    Eigen::Matrix<double, 8, 1> Densebeta0, Densebeta1, Densebeta2, Densebeta3, Densebeta4;
    double s1, s2, s3, s4, s5, s6, s7;
    double step, alpha;
    Eigen::MatrixXd gradViolaPc, gradViolaVc, gradViolaAc, gradViolaJc;
    Eigen::MatrixXd gradViolaVc_dense, gradViolaAc_dense, gradViolaJc_dense;
    gradViolaPc.resize(8, traj_dim_);
    gradViolaVc.resize(8, traj_dim_);
    gradViolaAc.resize(8, traj_dim_);
    gradViolaJc.resize(8, traj_dim_);
    gradViolaVc_dense.resize(8, traj_dim_);
    gradViolaAc_dense.resize(8, traj_dim_);
    gradViolaJc_dense.resize(8, traj_dim_);
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaJt,
           gradViolaVt_dense, gradViolaAt_dense, gradViolaJt_dense;

    double omg, dense_omg;
    int i_dp = 0;

    for (int i = 0; i < N; ++i){  // 遍历每段多项式
      // 获取多项式系数矩阵 c (8阶 × traj_dim_)
      Eigen::MatrixXd c;
      c.resize(8, traj_dim_);
      c = SnapOpt_container_[trajid].get_b().block(i * 8, 0, 8, traj_dim_);

      step = SnapOpt_container_[trajid].get_T1()(i) / K;  // 采样步长
      s1 = 0.0;  // 归一化时间 [0, 1]
      gradp_traj_piece.setZero();

      for (int j = 0; j <= K; ++j){
        // 计算基函数值 (8阶多项式: 1, s, s², ..., s⁷)
        s2 = s1 * s1; s3 = s2 * s1; s4 = s2 * s2;
        s5 = s4 * s1; s6 = s3 * s3; s7 = s6 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;                    // pos
        beta1 << 0.0, 1.0, 2*s1, 3*s2, 4*s3, 5*s4, 6*s5, 7*s6;      // vel
        beta2 << 0.0, 0.0, 2.0, 6*s1, 12*s2, 20*s3, 30*s4, 42*s5;   // acc
        beta3 << 0.0, 0.0, 0.0, 6.0, 24*s1, 60*s2, 120*s3, 210*s4;  // jer
        beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120*s1, 360*s2, 840*s3;  // snap
        alpha = 1.0 / K * j;

        // 计算该点的状态 (pos, vel, acc, jer, snap)
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        snap = c.transpose() * beta4;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;  // Simpson 积分权重
        cps_container_[trajid].points.col(i_dp) = pos;  // 保存控制点位置

        // ---------- [a] 障碍物代价 (obs) ----------
        gradp.setZero(); gradv.setZero();
        double cost_car = 0, cost_mani = 0, cost_self = 0;
        if(obstacleGradCostforMM(i_dp, pos, vel, trajid, gradp, gradv,
                                 cost_car, cost_mani, cost_self))
        {
          gradp_traj_piece += gradp;
          // 位置梯度 → 多项式系数梯度 (通过基函数传播)
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_)
              += omg * step * gradViolaPc;
          gdT(i) += omg * ((cost_car + cost_mani + cost_self) / K + step * gradViolaPt);

          // 速度梯度 → 多项式系数梯度
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_)
              += omg * step * gradViolaVc;
          gdT(i) += omg * (step * gradViolaVt);

          costs(0) += omg * step * cost_car;    // 累计底盘避障代价
          costs(1) += omg * step * cost_mani;   // 累计机械臂避障代价
          costs(2) += omg * step * cost_self;   // 累计自碰撞代价
        }

        // ---------- [b] 基底可行性代价 (feasibility) ----------
        Eigen::Vector2d gradv_2d, grada_2d, gradj_2d;
        gradv_2d.setZero(); grada_2d.setZero(); gradj_2d.setZero();
        double cost_mm_feasible = 0.0;
        if(feasibilityGradCostCar(vel.head(2), acc.head(2), jer.head(2), trajid,
                                  gradv_2d, grada_2d, gradj_2d,
                                  cost_mm_feasible, false)){
          gradv.setZero(); grada.setZero(); gradj.setZero();
          gradv.head(2) = gradv_2d; grada.head(2) = grada_2d; gradj.head(2) = gradj_2d;

          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_)
              += omg * step * gradViolaVc;
          gdT(i) += omg * (cost_mm_feasible / K + step * gradViolaVt);

          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_)
              += omg * step * gradViolaAc;
          gdT(i) += omg * (step * gradViolaAt);

          gradViolaJc = beta3 * gradj.transpose();
          gradViolaJt = alpha * gradj.transpose() * snap;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_)
              += omg * step * gradViolaJc;
          gdT(i) += omg * (step * gradViolaJt);

          costs(3) += omg * step * cost_mm_feasible;
        }

        // ---------- [c] 密集采样 (低速度区额外可行性检查) ----------
        double min_dense_vel = 0.1;
        double pen_min_vel = min_dense_vel * min_dense_vel - vel.head(2).squaredNorm();
        double pen_min_vel_log, grad_pen_min_vel_log;
        if(smoothedLog(pen_min_vel, 0.001, pen_min_vel_log, grad_pen_min_vel_log)){
          double special_step = step / dense_sample_resolution_;
          double special_s1 = s1;
          int disQuantity;
          double dense_alpha;
          Eigen::VectorXd gradv_dense(traj_dim_), grada_dense(traj_dim_), gradj_dense(traj_dim_);
          if(j == 0){
            disQuantity = dense_sample_resolution_ / 2;
            dense_alpha = 1.0 / K * j - 1.0 / K / dense_sample_resolution_;
          }else if(j == K){
            special_s1 = special_s1 - step / 2.0;
            disQuantity = dense_sample_resolution_ / 2;
            dense_alpha = 1.0 / K * j - 0.5 / K - 1.0 / K / dense_sample_resolution_;
          }else{
            special_s1 = special_s1 - step / 2.0;
            disQuantity = dense_sample_resolution_;
            dense_alpha = 1.0 / K * j - 0.5 / K - 1.0 / K / dense_sample_resolution_;
          }

          for(int l = 0; l <= disQuantity; l++){
            s2 = special_s1 * special_s1; s3 = s2 * special_s1;
            s4 = s2 * s2; s5 = s4 * special_s1; s6 = s3 * s3; s7 = s4 * s3;
            Densebeta0 << 1.0, special_s1, s2, s3, s4, s5, s6, s7;
            Densebeta1 << 0.0, 1.0, 2*special_s1, 3*s2, 4*s3, 5*s4, 6*s5, 7*s6;
            Densebeta2 << 0.0, 0.0, 2.0, 6*special_s1, 12*s2, 20*s3, 30*s4, 42*s5;
            Densebeta3 << 0.0, 0.0, 0.0, 6.0, 24*special_s1, 60*s2, 120*s3, 210*s4;
            Densebeta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120*special_s1, 360*s2, 840*s3;
            dense_alpha += 1.0 / K / dense_sample_resolution_;
            special_s1 += special_step;

            dense_pos = c.transpose() * Densebeta0;
            dense_vel = c.transpose() * Densebeta1;
            dense_acc = c.transpose() * Densebeta2;
            dense_jer = c.transpose() * Densebeta3;
            dense_sna = c.transpose() * Densebeta4;

            dense_omg = (l == 0 || l == disQuantity) ? 0.5 : 1.0;

            gradv_2d.setZero(); grada_2d.setZero(); gradj_2d.setZero();
            gradv_dense.setZero(); grada_dense.setZero(); gradj_dense.setZero(); gradv.setZero();
            double cost_mm_feasible = 0.0;
            if(feasibilityGradCostCar(dense_vel.head(2), dense_acc.head(2), dense_jer.head(2),
                                      trajid, gradv_2d, grada_2d, gradj_2d,
                                      cost_mm_feasible, true)){
              double cost_dense_log = pen_min_vel_log * cost_mm_feasible;
              gradv.head(2)       = -2 * grad_pen_min_vel_log * cost_mm_feasible * vel.head(2);
              gradv_dense.head(2) = pen_min_vel_log * gradv_2d;
              grada_dense.head(2) = pen_min_vel_log * grada_2d;
              gradj_dense.head(2) = pen_min_vel_log * gradj_2d;

              gradViolaVc = beta1 * gradv.transpose();
              gradViolaVt = alpha * gradv.transpose() * acc;
              gradViolaVc_dense = Densebeta1 * gradv_dense.transpose();
              gradViolaVt_dense = dense_alpha * gradv_dense.transpose() * dense_acc;
              gradViolaAc_dense = Densebeta2 * grada_dense.transpose();
              gradViolaAt_dense = dense_alpha * grada_dense.transpose() * dense_jer;
              gradViolaJc_dense = Densebeta3 * gradj_dense.transpose();
              gradViolaJt_dense = dense_alpha * gradj_dense.transpose() * dense_sna;

              SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_)
                  += dense_omg * special_step
                   * (gradViolaVc + gradViolaVc_dense + gradViolaAc_dense + gradViolaJc_dense);
              gdT(i) += dense_omg
                      * (cost_dense_log / K / dense_sample_resolution_
                         + special_step
                         * (gradViolaVt + gradViolaVt_dense + gradViolaAt_dense + gradViolaJt_dense));
              costs(3) += dense_omg * special_step * cost_dense_log;
            }
          }
        }

        // ---------- [d] 机械臂关节可行性代价 ----------
        gradp.setZero(); gradv.setZero(); grada.setZero();
        double cost_joint_feasible = 0.0;
        if(feasibilityGradCostJoint(pos, vel, acc, gradp, gradv, grada, cost_joint_feasible)){
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_)
              += omg * step * gradViolaPc;
          gdT(i) += omg * (cost_joint_feasible / K + step * gradViolaPt);

          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_)
              += omg * step * gradViolaVc;
          gdT(i) += omg * (step * gradViolaVt);

          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_)
              += omg * step * gradViolaAc;
          gdT(i) += omg * (step * gradViolaAt);

          costs(4) += omg * step * cost_joint_feasible;
        }

        s1 += step;
        if (j != K || (j == K && i == N - 1)){
          ++i_dp;
        }
      }
    }
  }

  // ============================================================================
  // 障碍物代价与梯度计算 (完整 4 层碰撞)
  //
  // 核心: 对所有碰撞球计算 ESDF 距离惩罚, 并通过链式法则求解析梯度.
  //
  // 碰撞类型:
  //   1. 底盘-障碍物 (car-obs):    从 getCarPtsGradNew 获取底盘碰撞球
  //   2. 机械臂-障碍物 (mani-obs):  从正运动学获取连杆碰撞球位置
  //   3. 地面碰撞 (ground):        检查机械臂碰撞球 z 坐标
  //   4. 底盘-机械臂自碰撞 (car-mani):  底盘球 vs 机械臂球
  //   5. 机械臂自碰撞 (mani-mani):     不同连杆之间的碰撞球
  //
  // 代价函数: J = wei_obs × Σ max(0, safe_margin - dist)³
  // 梯度: dJ/dp = -3·wei_obs·(safe_margin - dist)² · ∇(dist) · (球位置/优化变量)
  //
  // @param i_dp   控制点索引 (用于可视化)
  // @param pos    当前轨迹位置
  // @param vel    当前轨迹速度
  // @param trajid 轨迹段ID
  // @param gradp  [输出] 对位置 p 的梯度
  // @param gradv  [输出] 对速度 v 的梯度
  // @param costp  [输出] 底盘避障代价
  // @param costp_mani  [输出] 机械臂避障代价
  // @param costp_self  [输出] 自碰撞代价
  // @return true 表示有碰撞代价贡献
  // ============================================================================
  bool PolyTrajOptimizer::obstacleGradCostforMM(const int i_dp,
                                            const Eigen::VectorXd &pos,
                                            const Eigen::VectorXd &vel,
                                            const int trajid,
                                            Eigen::VectorXd &gradp,
                                            Eigen::VectorXd &gradv,
                                            double &costp,
                                            double &costp_mani,
                                            double &costp_self){
    bool ret = false;

    gradp.setZero();
    gradv.setZero();
    costp = 0;
    costp_mani = 0;
    costp_self = 0;

    double dist, dist_err, dist_err_2, dist_err_3;
    Eigen::Vector4d dist_grad4 = Eigen::Vector4d::Zero();
    Eigen::VectorXd curr_grad(traj_dim_);
    curr_grad.setZero();

    std::vector<Eigen::Vector3d> car_pts;
    std::vector<Eigen::Matrix2d> car_dPtsdv_list;
    std::vector<Eigen::Vector2d> car_dPtsdYaw_list;
    Eigen::Vector2d dYawdV = mm_config_->caldYawdV(vel.head(2));
    mm_config_->getCarPtsGradNew(pos.head(2), vel.head(2), singul_container_[trajid],
                                 Eigen::Vector3d(0, 0, 0), car_pts, car_dPtsdYaw_list);

    // ---------- [1] 底盘-障碍物碰撞 ----------
    for(unsigned int i = 0; i < car_pts.size(); ++i){
      Eigen::Vector3d dist_grad;
      grid_map_->evaluateEDTWithGrad(car_pts[i], dist, dist_grad);  // ESDF 查询
      dist_err = mobile_base_check_radius_ + safe_margin_ - dist;
      if (dist_err > 0){
        dist_err_2 = dist_err * dist_err;
        dist_err_3 = dist_err_2 * dist_err;
        ret = true;
        costp += wei_obs_ * dist_err_3;
        gradp.head(mobile_base_dof_) -= wei_obs_ * 3.0 * dist_err_2 * dist_grad.head(2);
        gradv.head(mobile_base_dof_) -= wei_obs_ * 3.0 * dist_err_2
                                      * dist_grad.head(2).transpose() * car_dPtsdYaw_list[i] * dYawdV;
      }
    }

    if(manipulator_dof_ < 1) return ret;

    // ---------- [2] 机械臂-障碍物 + 地面碰撞 ----------
    Eigen::Matrix4d T_w_q = Eigen::Matrix4d::Identity(), T_w_q_grad_x, T_w_q_grad_y;
    Eigen::Matrix2d R = mm_config_->calR(vel.head(2), singul_container_[trajid]);
    T_w_q.block(0, 0, 2, 2) = R;
    T_w_q.block(0, 3, 2, 1) = pos.head(2);
    T_w_q_grad_x.setZero(); T_w_q_grad_x(0, 3) = 1;
    T_w_q_grad_y.setZero(); T_w_q_grad_y(1, 3) = 1;

    // 正运动学: 获取各关节变换矩阵
    std::vector<Eigen::Matrix4d> T_joint;
    std::vector<Eigen::Matrix4d> T_joint_grad;
    mm_config_->getJointTrans(pos.tail(manipulator_dof_), T_joint, T_joint_grad);

    Eigen::Matrix4d T_now = T_w_q * T_q_0_;
    Eigen::Vector3d pt_on_link;
    std::vector<Eigen::Matrix4d> T_grad_list(2 + manipulator_dof_),
                                  T_grad_list_self(manipulator_dof_);
    Eigen::Matrix4d T_caldv = T_q_0_;

    // yaw 对速度的梯度
    double yaw = atan2(singul_container_[trajid] * vel(1), singul_container_[trajid] * vel(0));
    Eigen::Matrix2d dRdYaw;
    dRdYaw << -sin(yaw), -cos(yaw),
              cos(yaw), -sin(yaw);
    Eigen::Matrix4d dTdYaw;
    dTdYaw.setZero();
    dTdYaw.block(0, 0, 2, 2) = dRdYaw;
    dTdYaw = dTdYaw * T_q_0_;

    T_grad_list[0] = T_w_q_grad_x * T_q_0_;  // 对 x 的梯度
    T_grad_list[1] = T_w_q_grad_y * T_q_0_;  // 对 y 的梯度

    double ground_err = 0.0;
    double wei_ground_arg = 5.0;

    // 遍历各关节
    for(int i = 0; i < manipulator_dof_; ++i){
      for(int j = 0; j < i + 2; ++j){
        T_grad_list[j] = T_grad_list[j] * T_joint[i];
      }
      T_caldv = T_caldv * T_joint[i];
      T_grad_list[i + 2] = T_now * T_joint_grad[i];  // 对第 i 个关节角的梯度
      T_now = T_now * T_joint[i];
      dTdYaw = dTdYaw * T_joint[i];

      int pts_size = manipulator_link_pts_[i].cols();
      double factor = 1.0;

      for(int j = 0; j < pts_size; ++j){
        // 碰撞球在世界坐标系中的位置
        pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);

        // --- [2a] 机械臂-障碍物 ---
        Eigen::Vector3d dist_grad;
        grid_map_->evaluateEDTWithGrad(pt_on_link, dist, dist_grad);
        dist_err = manipulator_thickness_ + safe_margin_mani_ - dist;
        if(dist_err > 0){
          ret = true;
          dist_grad4.segment(0, 3) = dist_grad;
          costp_mani += wei_mani_obs_ * factor * pow(dist_err, 3);
          curr_grad.setZero();
          for(int k = 0; k < i + 3; ++k){
            curr_grad(k) -= wei_mani_obs_ * factor * 3.0 * pow(dist_err, 2)
                          * dist_grad4.transpose()
                          * (T_grad_list[k] * manipulator_link_pts_[i].col(j));
          }
          gradp += curr_grad;

          // 速度梯度 (通过 yaw)
          curr_grad.setZero();
          double dDistdYaw = dist_grad4.transpose()
                           * (dTdYaw * manipulator_link_pts_[i].col(j));
          Eigen::Vector2d gradv_temp = wei_mani_obs_ * factor * 3.0 * pow(dist_err, 2)
                                     * dDistdYaw * dYawdV;
          curr_grad.head(2) -= gradv_temp;
          gradv += curr_grad;
        }

        // --- [2b] 地面碰撞 ---
        Eigen::VectorXd dzdP = Eigen::VectorXd::Zero(4);
        dzdP(2) = 1;
        ground_err = manipulator_thickness_ + ground_safe_dis_ + ground_safe_margin_
                   + map_resolution_ - pt_on_link(2);
        if(ground_err > 0){
          ground_err = manipulator_thickness_ + ground_safe_dis_ + ground_safe_margin_
                     - pt_on_link(2);
          double f_temp = 0, df_ground = 0;
          if(smoothedL1(ground_err, 0.0005, f_temp, df_ground)){
            costp_mani += wei_mani_obs_ * wei_ground_arg * factor * f_temp;
            curr_grad.setZero();
            for(int k = 0; k < i + 3; ++k){
              if(ground_err > 0){
                curr_grad(k) -= wei_mani_obs_ * wei_ground_arg * factor * df_ground
                              * dzdP.transpose()
                              * (T_grad_list[k] * manipulator_link_pts_[i].col(j));
              }
            }
            gradp += curr_grad;
          }
        }
      }
    }

    // ---------- [3] 底盘-机械臂自碰撞 + [4] 机械臂自碰撞 ----------
    T_now = T_q_0_ * T_joint[0];
    T_grad_list_self[0] = T_q_0_ * T_joint_grad[0];
    mm_config_->getCarPts(Eigen::Vector3d::Zero(), car_pts, Eigen::Vector3d::Zero());
    double factor2 = 1.0;
    for(int i = 1; i < manipulator_dof_; ++i){
      for(int j = 0; j < i; ++j){
        T_grad_list_self[j] = T_grad_list_self[j] * T_joint[i];
      }
      T_grad_list_self[i] = T_now * T_joint_grad[i];
      T_now *= T_joint[i];

      int pts_size = manipulator_link_pts_[i].cols();
      double factor = 1.0;
      double wei_temp = wei_mani_self_ * factor * factor2;

      for(int j = 0; j < pts_size; ++j){
        pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);

        // --- [3] 底盘-机械臂 ---
        for(unsigned int m = 0; m < car_pts.size(); ++m){
          dist = (car_pts[m] - pt_on_link).norm();
          dist_err = manipulator_thickness_ + mobile_base_check_radius_ + self_safe_margin_ - dist;
          if(dist_err > 0){
            dist_err_2 = dist_err * dist_err;
            dist_err_3 = dist_err_2 * dist_err;
            dist_grad4.segment(0, 3) = (pt_on_link - car_pts[m]) / dist;
            ret = true;
            costp_self += wei_temp * dist_err_3;
            curr_grad.setZero();
            for(int k = 0; k < i + 1; ++k){
              curr_grad(mobile_base_dof_ + k) -= 3.0 * wei_temp * dist_err_2
                  * (T_grad_list_self[k] * manipulator_link_pts_[i].col(j)).transpose()
                  * dist_grad4;
            }
            gradp += curr_grad;
          }
        }

        // --- [4] 机械臂自碰撞 (连杆 i 与之前各连杆的碰撞) ---
        if(i < 2) continue;
        Eigen::Matrix4d T_now_self_mani = T_joint[i];
        std::vector<Eigen::Matrix4d> T_grad_self_mani_list;
        T_grad_self_mani_list.reserve(i);
        T_grad_self_mani_list.push_back(T_joint_grad[i]);

        for(int m = i - 2; m >= 0; --m){
          for(unsigned int n = 0; n < T_grad_self_mani_list.size(); ++n){
            T_grad_self_mani_list[n] = T_joint[m + 1] * T_grad_self_mani_list[n];
          }
          T_grad_self_mani_list.push_back(T_joint_grad[m + 1] * T_now_self_mani);
          T_now_self_mani = T_joint[m + 1] * T_now_self_mani;

          Eigen::Vector3d pt_on_link_to_check_m, pt_on_link_m;
          pt_on_link_m = (T_now_self_mani * manipulator_link_pts_[i].col(j)).head(3);
          int pts_size_temp = manipulator_link_pts_[m].cols();
          double factor = 1.0 / (double)pts_size_temp;
          for(int n = 0; n < pts_size_temp; ++n){
            pt_on_link_to_check_m = (manipulator_link_pts_[m].col(n)).head(3);
            dist = (pt_on_link_m - pt_on_link_to_check_m).norm();
            dist_err = 2 * manipulator_thickness_ + self_safe_margin_ - dist;
            if(dist_err > 0){
              dist_err_2 = dist_err * dist_err;
              dist_err_3 = dist_err_2 * dist_err;
              dist_grad4.segment(0, 3) = (pt_on_link_m - pt_on_link_to_check_m) / dist;
              ret = true;
              costp_self += wei_mani_self_ * factor * dist_err_3;
              curr_grad.setZero();
              int temp_grad_list_size = T_grad_self_mani_list.size() - 1;
              for(int k = temp_grad_list_size; k >= 0; --k){
                curr_grad(mobile_base_dof_ + m + k + 1) -= 3 * wei_mani_self_ * factor * dist_err_2
                    * (T_grad_self_mani_list[temp_grad_list_size - k]
                       * manipulator_link_pts_[i].col(j)).transpose() * dist_grad4;
              }
              gradp += curr_grad;
            }
          }
        }
      }
    }

    return ret;
  }

  // ============================================================================
  // 底盘可行性代价与梯度 (差速轮运动学模型)
  //
  // 约束项:
  //   (a) 最小速度 (仅密集采样时): |v| ≥ non_singul_v_
  //   (b) 最大速度: |v| ≤ max_vel_
  //   (c) 左右轮最大转速: |ω_left|, |ω_right| ≤ max_wheel_omega_
  //   (d) 左右轮最大角加速度: |α_left|, |α_right| ≤ max_wheel_alpha_
  //
  // 差速轮模型:
  //   ω = (aᵀ·B·v) / (vᵀ·v)     (航向角速度)
  //   ω_left  = (2·s·|v| - b·ω) / (2·r)
  //   ω_right = (2·s·|v| + b·ω) / (2·r)
  //
  // 每个约束均使用 smoothedL1 惩罚, 并计算解析梯度.
  // ============================================================================
  bool PolyTrajOptimizer::feasibilityGradCostCar(const Eigen::Vector2d &vel,
                                                  const Eigen::Vector2d &acc,
                                                  const Eigen::Vector2d &jer,
                                                  const int trajid,
                                                  Eigen::Vector2d &gradv_2d,
                                                  Eigen::Vector2d &grada_2d,
                                                  Eigen::Vector2d &gradj_2d,
                                                  double &cost_mm_feasible,
                                                  bool dense_sample){
    bool ret = false;
    double pen, f, df;
    cost_mm_feasible = 0;

    const double vTv = vel.transpose() * vel;
    const double aTv = acc.transpose() * vel;
    const double aTBv = acc.transpose() * B_h_ * vel;
    const double jTBv = jer.transpose() * B_h_ * vel;
    const double v_norm = vel.norm();
    const double vTv_inv = 1.0 / vTv;
    const double vTv_inv2 = vTv_inv * vTv_inv;

    // (a) 最小速度 (仅密集采样时激活)
    if(dense_sample){
      pen = non_singul_v_ * non_singul_v_ - vel.squaredNorm();
      f = 0; df = 0;
      if(smoothedL1(pen, 0.005, f, df)){
        cost_mm_feasible += wei_feas_ * 1e6 * f;   // 极高权重: 强制避免零速
        gradv_2d += -wei_feas_ * 1e6 * df * 2.0 * vel;
        ret = true;
      }
    }

    // (b) 最大速度
    pen = vel.squaredNorm() - max_vel_ * max_vel_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * vel;
      ret = true;
    }

    double omega = aTBv * vTv_inv;

    // ω 对 v 和 a 的解析梯度
    Eigen::Vector2d dOmegadV = B_h_.transpose() * acc / vTv
                             - (aTBv * vel + vel.transpose() * B_h_.transpose() * acc * vel) / vTv / vTv;
    Eigen::Vector2d dOmegadA = vTv_inv * (B_h_ * vel);

    // (c) 左轮最大转速
    double wheel_omega_left = (2.0 * singul_container_[trajid] * v_norm
                              - mobile_base_wheel_base_ * omega) / (2.0 * mobile_base_wheel_radius_);
    pen = wheel_omega_left * wheel_omega_left - max_wheel_omega_ * max_wheel_omega_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_omega_left
                * (2.0 * singul_container_[trajid] / v_norm * vel
                   - mobile_base_wheel_base_ * dOmegadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += -wei_feas_ * df * 2.0 * wheel_omega_left
                * mobile_base_wheel_base_ * dOmegadA / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }

    // (c) 右轮最大转速
    double wheel_omega_right = (2.0 * singul_container_[trajid] * v_norm
                               + mobile_base_wheel_base_ * omega) / (2 * mobile_base_wheel_radius_);
    pen = wheel_omega_right * wheel_omega_right - max_wheel_omega_ * max_wheel_omega_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_omega_right
                * (2.0 * singul_container_[trajid] / v_norm * vel
                   + mobile_base_wheel_base_ * dOmegadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += wei_feas_ * df * 2.0 * wheel_omega_right
                * mobile_base_wheel_base_ * dOmegadA / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }

    // 角加速度 α 的表达式
    double alpha = jTBv * vTv_inv - 2.0 * aTBv * aTv * vTv_inv2;

    // α 对 v, a, j 的解析梯度
    Eigen::Vector2d dAlphadV = (B_h_.transpose() * jer * vTv - 2.0 * jTBv * vel) * vTv_inv2
                             - 2.0 * (aTBv * acc + aTv * B_h_.transpose() * acc) * vTv_inv2
                             + 8.0 * vTv_inv2 * vTv_inv * aTBv * aTv * vel;
    Eigen::Vector2d dAlphadA = -2.0 * vTv_inv2 * (aTv * B_h_ * vel + aTBv * vel);
    Eigen::Vector2d dAlphadJ = B_h_ * vel * vTv_inv;

    // (d) 左轮最大角加速度
    double wheel_alpha_left = (2.0 * singul_container_[trajid] * aTv / v_norm
                              - mobile_base_wheel_base_ * alpha) / (2.0 * mobile_base_wheel_radius_);
    pen = wheel_alpha_left * wheel_alpha_left - max_wheel_alpha_ * max_wheel_alpha_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_alpha_left
                * (2.0 * singul_container_[trajid] * (v_norm * acc - aTv * vel / v_norm) * vTv_inv
                   - mobile_base_wheel_base_ * dAlphadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += wei_feas_ * df * 2.0 * wheel_alpha_left
                * (2.0 * singul_container_[trajid] * vel / v_norm
                   - mobile_base_wheel_base_ * dAlphadA) / (2.0 * mobile_base_wheel_radius_);
      gradj_2d += -wei_feas_ * df * 2.0 * wheel_alpha_left
                * mobile_base_wheel_base_ * dAlphadJ / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }

    // (d) 右轮最大角加速度
    double wheel_alpha_right = (2.0 * singul_container_[trajid] * aTv / v_norm
                               + mobile_base_wheel_base_ * alpha) / (2.0 * mobile_base_wheel_radius_);
    pen = wheel_alpha_right * wheel_alpha_right - max_wheel_alpha_ * max_wheel_alpha_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_alpha_right
                * (2.0 * singul_container_[trajid] * (v_norm * acc - aTv * vel / v_norm) * vTv_inv
                   + mobile_base_wheel_base_ * dAlphadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += wei_feas_ * df * 2.0 * wheel_alpha_right
                * (2.0 * singul_container_[trajid] * vel / v_norm
                   + mobile_base_wheel_base_ * dAlphadA) / (2.0 * mobile_base_wheel_radius_);
      gradj_2d += wei_feas_ * df * 2.0 * wheel_alpha_right
                * mobile_base_wheel_base_ * dAlphadJ / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }
    return ret;
  }

  // ============================================================================
  // 机械臂关节可行性代价与梯度
  //
  // 约束项:
  //   1. 关节位置限位: pos_min ≤ q ≤ pos_max
  //   2. 关节速度限位: |q̇| ≤ max_vel
  //   3. 关节加速度限位: |q̈| ≤ max_acc
  //
  // 全部使用 smoothedL1 惩罚函数
  // ============================================================================
  bool PolyTrajOptimizer::feasibilityGradCostJoint(const Eigen::VectorXd &pos,
                                                    const Eigen::VectorXd &vel,
                                                    const Eigen::VectorXd &acc,
                                                    Eigen::VectorXd &gradp,
                                                    Eigen::VectorXd &gradv,
                                                    Eigen::VectorXd &grada,
                                                    double &cost_joint_feasible){
    bool ret = false;
    double pen, f, df;

    // (1) 位置限位
    for(int i = 0; i < manipulator_dof_; ++i){
      pen = pos(mobile_base_dof_ + i) - max_joint_pos_[i];
      f = 0; df = 0;
      if(smoothedL1(pen, 0.005, f, df)){
        cost_joint_feasible += wei_mani_feas_ * f;
        gradp(mobile_base_dof_ + i) += wei_mani_feas_ * df;
        ret = true;
      }

      pen = min_joint_pos_[i] - pos(mobile_base_dof_ + i);
      f = 0; df = 0;
      if(smoothedL1(pen, 0.005, f, df)){
        cost_joint_feasible += wei_mani_feas_ * f;
        gradp(mobile_base_dof_ + i) += wei_mani_feas_ * df * (-1.0);
        ret = true;
      }
    }

    // (2) 速度限位
    for(int i = 0; i < manipulator_dof_; ++i){
      pen = vel(mobile_base_dof_ + i) * vel(mobile_base_dof_ + i) - max_joint_vel_ * max_joint_vel_;
      f = 0; df = 0;
      if(smoothedL1(pen, 0.005, f, df)){
        gradv(mobile_base_dof_ + i) += wei_mani_feas_ * df * 2.0 * vel(mobile_base_dof_ + i);
        cost_joint_feasible += wei_mani_feas_ * f;
        ret = true;
      }
    }

    // (3) 加速度限位
    for(int i = 0; i < manipulator_dof_; ++i){
      pen = acc(mobile_base_dof_ + i) * acc(mobile_base_dof_ + i) - max_joint_acc_ * max_joint_acc_;
      f = 0; df = 0;
      if(smoothedL1(pen, 0.005, f, df)){
        grada(mobile_base_dof_ + i) += wei_mani_feas_ * df * 2.0 * acc(mobile_base_dof_ + i);
        cost_joint_feasible += wei_mani_feas_ * f;
        ret = true;
      }
    }

    return ret;
  }

  // ============================================================================
  // 前端 A* 搜索 + MINCO 轨迹生成
  //
  // 步骤:
  //   1. 调用 KinoAstar 搜索基底路径 (x, y, yaw), 含前进/后退切换
  //   2. 为每段 singul 轨迹创建 MINCO (MinSnapOpt)
  //   3. 根据 A* 搜索的路径点和时间生成 MINCO 初值
  //
  // 时间分配: 考虑机械臂关节最大速度/加速度, 确保关节运动可行
  //   对每段: t = max(基底时间, 各关节的梯形速度时间)
  //
  // @param simple_path_container [输出] A* 搜索路径 (按 singul 分段)
  // @param yaw_list_container    [输出] 每段的 yaw 序列
  // @param frontendMJ_container  [输出] 每段对应的 MINCO 优化器
  // @param singul_container      [输出] 每段的前进/后退标志
  // @return KinoAstar 返回状态码
  // ============================================================================
  int PolyTrajOptimizer::astarWithMinTraj(const Eigen::MatrixXd &iniState,
                                           const Eigen::MatrixXd &finState,
                                           const double start_yaw,
                                           const int _start_singul,
                                           const bool start_gripper,
                                           const double end_yaw,
                                           const bool end_gripper,
                                           const Eigen::Vector2d init_ctrl,
                                           const int continous_failures_count,
                                           std::vector<std::vector<Eigen::VectorXd>> &simple_path_container,
                                           std::vector<std::vector<double>> &yaw_list_container,
                                           std::vector<poly_traj::MinSnapOpt<8>> &frontendMJ_container,
                                           std::vector<int> &singul_container){
    Eigen::VectorXd start_pos = iniState.col(0);
    Eigen::VectorXd end_pos = finState.col(0);
    Eigen::VectorXd start_vel = iniState.col(1);
    Eigen::VectorXd end_vel = finState.col(1);
    int start_singul = _start_singul;
    std::vector<Eigen::VectorXd> t_list_container;
    t_list_container.clear();
    vector<Eigen::VectorXd> simple_path; simple_path.clear();
    std::vector<double> yaw_list; yaw_list.clear();
    Eigen::VectorXd t_list;

    simple_path_container.clear();
    yaw_list_container.clear();
    frontendMJ_container.clear();
    singul_container.clear();

    // ---- Step 1: KinoA* 搜索 ----
    int status = kino_a_star_->KinoAstarSearchAndGetSimplePath(
        start_pos, start_vel, start_yaw, start_singul, start_gripper,
        end_pos, end_vel, end_yaw, end_gripper, init_ctrl,
        continous_failures_count,
        simple_path_container, yaw_list_container,
        singul_container, t_list_container);

    if(status == KinoAstar::NO_PATH || status == KinoAstar::START_COLLISION
       || status == KinoAstar::GOAL_COLLISION){
      return status;
    }

    // ---- Step 2: 为每段路径创建 MINCO 轨迹 ----
    Eigen::MatrixXd innerPts;
    Eigen::MatrixXd headState, tailState;
    headState.resize(traj_dim_, 4);
    tailState.resize(traj_dim_, 4);
    frontendMJ_container.resize(simple_path_container.size());

    for(unsigned int i = 0; i < simple_path_container.size(); ++i){
      int piece_num = simple_path_container[i].size() - 1;

      if (piece_num > 1){
        innerPts.resize(traj_dim_, piece_num - 1);
        for (int j = 0; j < piece_num - 1; ++j){
          innerPts.col(j) = (simple_path_container[i])[j + 1];
        }
      }else{
        // 路径太短: 在中间插一个点
        piece_num = 2;
        innerPts.resize(traj_dim_, 1);
        innerPts.col(0) = ((simple_path_container[i])[0] + (simple_path_container[i])[1]) / 2;
        simple_path_container[i].insert(simple_path_container[i].begin() + 1, innerPts.col(0));
        t_list.resize(2);
        t_list.setConstant((t_list_container[i])[0] / 2);
        t_list_container[i] = t_list;
      }

      // ---- Step 3: 时间分配 (考虑关节运动) ----
      t_list = t_list_container[i];
      double t_acc = max_joint_vel_ / max_joint_acc_;     // 加速时间
      double dist_acc = max_joint_acc_ * t_acc * t_acc;   // 加速距离

      for (int j = 0; j < piece_num; ++j){
        Eigen::VectorXd angle1 = (simple_path_container[i])[j].tail(manipulator_dof_);
        Eigen::VectorXd angle2 = (simple_path_container[i])[j + 1].tail(manipulator_dof_);
        double max_t = t_list[j];

        // 对每个关节: 计算梯形速度曲线所需时间
        for(int k = 0; k < manipulator_dof_; ++k){
          double err = fabs(angle1(k) - angle2(k));
          double t_acc_vel = 0;
          if(j == 0 || j == piece_num - 1){
            // 首尾段: 假设从零速开始/结束
            if(err <= dist_acc){
              t_acc_vel = sqrt(err / max_joint_acc_);
            }else{
              t_acc_vel = (err - dist_acc) / max_joint_vel_ + 2 * t_acc;
            }
          }else{
            t_acc_vel = err / max_joint_vel_;  // 中间段: 匀速通过
          }
          max_t = max(t_acc_vel, max_t);
        }
        t_list[j] = max_t;
      }
      t_list[0] *= 1.5;               // 首段增加安全裕度
      t_list[piece_num - 1] *= 1.5;   // 尾段增加安全裕度

      // ---- Step 4: 构造起止状态 ----
      if(i > 0){
        headState.setZero();
        headState.col(0) = (simple_path_container[i])[0];
        headState.col(1).head(2) = singul_container[i] * non_singul_v_
                                 * Eigen::Vector2d(cos(yaw_list_container[i][0]), sin(yaw_list_container[i][0]));
      }else{
        headState = iniState;
        if(_start_singul == 0){
          headState.col(1).head(2) = singul_container[i] * non_singul_v_
                                   * Eigen::Vector2d(cos(yaw_list_container[i][0]), sin(yaw_list_container[i][0]));
        }
      }

      if(i < simple_path_container.size() - 1 || status == KinoAstar::REACH_HORIZON){
        tailState.setZero();
        tailState.col(0) = (simple_path_container[i]).back();
        tailState.col(1).head(2) = singul_container[i] * non_singul_v_
                                 * Eigen::Vector2d(cos(yaw_list_container[i].back()), sin(yaw_list_container[i].back()));
      }else{
        tailState = finState;
        tailState.col(1).head(2) = singul_container[i] * tailState.col(1).head(2);
        tailState.col(2).head(2) = singul_container[i] * tailState.col(2).head(2);
        tailState.col(3).head(2) = singul_container[i] * tailState.col(3).head(2);
      }

      // ---- Step 5: 生成 MINCO 轨迹 ----
      frontendMJ_container[i].reset(headState, tailState, piece_num);
      frontendMJ_container[i].generate(innerPts, t_list_container[i]);
    }

    return status;
  }

  // ============================================================================
  // 可视化: 前端搜索的碰撞球体
  // ============================================================================
  void PolyTrajOptimizer::displayFrontEndMesh(std::vector<Eigen::VectorXd> &simple_path_full,
                                               vector<double> &yaw_list){
    Eigen::Vector3d car_state;
    Eigen::VectorXd joint_state;
    joint_state.resize(manipulator_dof_);

    visualization_msgs::msg::MarkerArray marker_array, marker_array_all;
    visualization_msgs::msg::Marker marker_delete_all;
    marker_delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array_all.markers.push_back(marker_delete_all);

    for(unsigned int i = 0; i < simple_path_full.size(); i+=1){
      car_state.head(2) = simple_path_full[i].head(2);
      car_state(2) = yaw_list[i];
      joint_state = simple_path_full[i].tail(manipulator_dof_);
      mm_config_->getMMMarkerArray(marker_array, "vis_mm_front_end", i, 0.07,
                                   car_state, joint_state, true);
      marker_array_all.markers.insert(marker_array_all.markers.end(),
                                       marker_array.markers.begin(), marker_array.markers.end());
    }
    front_end_mm_mesh_vis_pub_->publish(marker_array_all);
  }

  // ============================================================================
  // 辅助函数: 清理和设置控制点容器
  // ============================================================================
  void PolyTrajOptimizer::clear_resize_Cps_container(int container_size){
    cps_container_.clear();
    cps_container_.resize(container_size);
  }

  void PolyTrajOptimizer::setControlPoints(const int trajid, const Eigen::MatrixXd &points){
    cps_container_[trajid].resize_cp(points.cols());
    cps_container_[trajid].points = points;
  }

}
