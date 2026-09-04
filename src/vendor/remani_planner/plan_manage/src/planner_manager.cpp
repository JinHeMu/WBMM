/**
 * @file planner_manager.cpp
 * @brief 规划管理器实现 — FSM 与前后端之间的"胶水层"
 *
 * ============================================================
 * 核心职责: 串联前端搜索 + 后端优化, 管理轨迹数据容器
 * ============================================================
 *
 * 调用链:
 *   REMANIReplanFSM::callReboundReplan()
 *     └── MMPlannerManager::reboundReplan()          ← 入口
 *           ├── computeInitReferenceState()           ← Step1: 前端搜索/热启动
 *           │     ├── A* + MINCO (首次/全局重规划)
 *           │     └── 从旧轨迹采样 (局部热启动重规划)
 *           ├── 控制点初始化
 *           └── PolyTrajOptimizer::OptimizeTrajectory_lbfgs()  ← Step2: 后端优化
 *
 * 数据流:
 *   FSM 传入起止状态 → 前端生成 MINCO 初值 → 后端优化 → 写入 traj_container_
 *
 * @see REMANIReplanFSM (调用者): remani_replan_fsm.cpp
 * @see PolyTrajOptimizer (后端优化器): poly_traj_optimizer.cpp
 */

#include <plan_manage/planner_manager.h>
#include <thread>
#include <visualization_msgs/msg/marker.hpp>

namespace remani_planner
{
  template<typename T>
  static T declareAndGetManagerParam(const rclcpp::Node::SharedPtr &node,
                                     const std::string &name,
                                     const T &default_value)
  {
    if (!node->has_parameter(name)) {
      node->declare_parameter<T>(name, default_value);
    }
    return node->get_parameter(name).get_value<T>();
  }

  MMPlannerManager::MMPlannerManager() {}

  MMPlannerManager::~MMPlannerManager()
  {
    std::cout << "destory manager" << std::endl;
    std_msgs::msg::Bool destory_cmd;
    destory_cmd.data = true;
  }

  /**
   * @brief 初始化所有规划子模块
   *
   * 加载参数并创建三大核心模块:
   *   1. GridMap          — ESDF 占据地图 (碰撞检测用)
   *   2. MMConfig          — 机器人运动学/碰撞模型
   *   3. PolyTrajOptimizer — 后端轨迹优化器 (L-BFGS)
   */
  void MMPlannerManager::initPlanModules(rclcpp::Node::SharedPtr node, PlanningVisualization::Ptr vis)
  {
    node_ = node;
    visualization_ = vis;

    /* ---------- 算法参数加载 ---------- */

    pp_.max_mani_vel_ =
        declareAndGetManagerParam<double>(node_, "mm.manipulator_max_vel", -1.0);
    pp_.max_mani_acc_ =
        declareAndGetManagerParam<double>(node_, "mm.manipulator_max_acc", -1.0);
    pp_.feasibility_tolerance_ =
        declareAndGetManagerParam<double>(node_, "manager.feasibility_tolerance", 0.0);
    pp_.polyTraj_piece_length =
        declareAndGetManagerParam<double>(node_, "manager.polyTraj_piece_length", -1.0);
    pp_.polyTraj_piece_time =
        declareAndGetManagerParam<double>(node_, "search.time_resolution", -1.0);
    double dist_resolution;
    dist_resolution =
        declareAndGetManagerParam<double>(node_, "search.dist_resolution", -1.0);
    pp_.polyTraj_piece_time = dist_resolution / pp_.max_vel_;
    pp_.drone_id = declareAndGetManagerParam<int>(node_, "manager.drone_id", -1);

    pp_.planning_horizen_ =
        declareAndGetManagerParam<double>(node_, "fsm.planning_horizon", 5.0);
    const bool global_plan =
        declareAndGetManagerParam<bool>(node_, "fsm.global_plan", false);
    if(global_plan) pp_.planning_horizen_ = 1e3;

    pp_.mobile_base_dim_ =
        declareAndGetManagerParam<int>(node_, "mm.mobile_base_dof", -1);
    pp_.manipulator_dim_ =
        declareAndGetManagerParam<int>(node_, "mm.manipulator_dof", -1);
    pp_.mobile_base_non_singul_vel_ =
        declareAndGetManagerParam<double>(node_, "mm.mobile_base_non_singul_vel", -1.0);
    destory_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/mm_controller_node/destory_cmd", 10);

    pp_.traj_dim_ = pp_.mobile_base_dim_ + pp_.manipulator_dim_;
    total_time_.clear();

    /* ---------- 模块1: 栅格地图 (ESDF) ---------- */
    grid_map_.reset(new GridMap);
    grid_map_->initMap(node_);

    /* ---------- 模块2: 机器人运动学/碰撞模型 ---------- */
    mm_config_.reset(new MMConfig);
    mm_config_->setParam(node_, grid_map_);

    /* ---------- 模块3: 轨迹优化器 ---------- */
    pp_.max_vel_ = mm_config_->getBaseMaxVel();
    pp_.max_acc_ = mm_config_->getBaseMaxAcc();

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(node_, grid_map_, mm_config_);
  }

  /**
   * @brief 前端: 计算初始参考状态 (MINCO 初值)
   *
   * 两种模式:
   *   Case 1 (flag_polyInit=true 或首次): 执行 A* 搜索 → MINCO 轨迹
   *   Case 2 (flag_polyInit=false, 有旧轨迹): 从上一轮轨迹采样 → 热启动
   *
   * @param start_pt / vel / acc / jerk  起始边界条件
   * @param start_yaw / singul           起始朝向与前进/后退
   * @param local_target_*               局部目标点 (从全局轨迹采样)
   * @param initMJO_container            [输出] 每段 MINCO 优化器实例
   * @param singul_container             [输出] 每段的前进/后退标志
   * @param flag_polyInit                是否执行 A* 搜索 (true) 或热启动 (false)
   * @param continous_failures_count     连续失败次数 (影响搜索策略)
   *
   * @return false 表示前端搜索失败
   *
   * 重要: 每段 singul 轨迹对应一个连续同向运动段 (换向时另起一段)
   */
  bool MMPlannerManager::computeInitReferenceState(const Eigen::VectorXd &start_pt,
                                                    const Eigen::VectorXd &start_vel,
                                                    const Eigen::VectorXd &start_acc,
                                                    const Eigen::VectorXd &start_jerk,
                                                    const double start_yaw,
                                                    const int start_singul,
                                                    const bool start_gripper,
                                                    const Eigen::VectorXd &local_target_pt,
                                                    const Eigen::VectorXd &local_target_vel,
                                                    const Eigen::VectorXd &local_target_acc,
                                                    const double local_target_yaw,
                                                    const bool local_target_gripper,
                                                    std::vector<poly_traj::MinSnapOpt<8>> &initMJO_container,
                                                    std::vector<int> &singul_container,
                                                    const bool flag_polyInit,
                                                    const int continous_failures_count)
  {
    static bool flag_first_call = true;
    initMJO_container.clear();
    singul_container.clear();

    /**
     * @brief Case 1: 使用 A* 搜索生成初值 (首次或全局重规划)
     */
    if (flag_first_call || flag_polyInit || true){
      flag_first_call = false;

      /* ---- 构造边界状态矩阵 ---- */
      Eigen::MatrixXd headState, tailState;
      headState.resize(pp_.traj_dim_, 4);   // [pos, vel, acc, jerk] × traj_dim
      tailState.resize(pp_.traj_dim_, 4);
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;

      // 起始状态: [pos, vel, acc, jerk]
      headState.col(0) = start_pt;
      headState.col(1) = start_vel;
      headState.col(2) = start_acc;
      headState.col(3) = start_jerk;

      // 终止状态: [pos, vel, acc, jerk=0]
      tailState.col(0) = local_target_pt;
      tailState.col(1) = local_target_vel;
      tailState.col(2) = local_target_acc;
      tailState.col(3) = Eigen::VectorXd::Zero(pp_.traj_dim_);

      /* ---- Step 1: A* 搜索生成分段 MINCO 轨迹 ---- */
      vector<vector<Eigen::VectorXd>> simple_path_container;  // A* 搜索路径 (含机械臂)
      vector<Eigen::Vector2d> simple_path;
      vector<vector<double>> yaw_list_container;
      yaw_list_container.clear();
      Eigen::Vector2d init_ctrl;
      init_ctrl.setZero();

      // 核心调用: astarWithMinTraj 执行完整的:
      //   (a) Hybrid A* 搜索基底路径 (x, y, yaw)
      //   (b) 全身 RRT 生成机械臂构型
      //   (c) MINCO 轨迹生成 (每段换向 = 一个 MinSnapOpt)
      int status = ploy_traj_opt_->astarWithMinTraj(headState, tailState, start_yaw, start_singul, start_gripper,
                                                    local_target_yaw, local_target_gripper, init_ctrl,
                                                    continous_failures_count,
                                                    simple_path_container, yaw_list_container,
                                                    initMJO_container, singul_container);
      if(status == KinoAstar::NO_PATH || status == KinoAstar::START_COLLISION || status == KinoAstar::GOAL_COLLISION){
        return false;  // 搜索失败
      }

      /* ---- 可视化 A* 搜索结果 ---- */
      vector<vector<Eigen::Vector2d>> path_view;
      vector<Eigen::Vector2d> display_pts;
      std::vector<Eigen::Vector2d> display_point_set;
      std::vector<Eigen::VectorXd> display_simple_path;
      std::vector<double> display_yaw;
      for(unsigned int i = 0; i < simple_path_container.size(); ++i){
        simple_path.clear();
        for(unsigned int j = 0; j < simple_path_container[i].size(); ++j){
          simple_path.push_back((simple_path_container[i])[j].head(2));
          display_simple_path.push_back((simple_path_container[i])[j]);
          display_yaw.push_back((yaw_list_container[i])[j]);
        }
        path_view.push_back(simple_path);

        // 提取 MINCO 的中间约束点 (waypoints) 和时间分配
        Eigen::MatrixXd waypoints;
        Eigen::VectorXd time_list;
        Eigen::MatrixXd ctl_points;

        Eigen::Vector2d pts;
        waypoints = initMJO_container[i].getInitConstrainPoints(1);  // 中间路径点
        time_list = initMJO_container[i].get_T1();                    // 段时间分配
        ctl_points = initMJO_container[i].getInitConstrainPoints(8); // 控制点 (8阶多项式)
        pts = waypoints.col(0).head(2);
        display_pts.push_back(pts);
        for(unsigned int j = 1; j < waypoints.cols(); ++j)
        {
          pts = waypoints.col(j).head(2);
          display_pts.push_back(pts);
        }
        for (int j = 0; j < ctl_points.cols(); ++j)
          display_point_set.push_back(ctl_points.col(j).head(2));
      }
      visualization_->displayAStarList(path_view, 0);               // 显示A*路径
      visualization_->displayInitWaypoints(display_pts, 0.2, 0);    // 显示中间路径点
      visualization_->displayInitPathListDebug(display_point_set, 0.1, 0); // 显示控制点
      ploy_traj_opt_->displayFrontEndMesh(display_simple_path, display_yaw); // 显示前端搜索的碰撞球
    }

    /**
     * @brief Case 2: 从上一轮优化轨迹热启动 (用于连续重规划)
     */
    else{
      RCLCPP_INFO(node_->get_logger(),"get init from local traj");

      if (traj_container_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        RCLCPP_ERROR(node_->get_logger(),"You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* 时间系统说明:
       *   passed_t_on_lctraj: 当前局部轨迹已经执行的时间
       *   t_to_lc_end:        当前局部轨迹剩余的执行时间
       *   t_to_lc_tgt:        从当前时间到局部目标点总剩余时间
       *                       (= 当前段剩余 + 后续全局段)
       */
      double passed_t_on_lctraj = node_->now().seconds() - traj_container_.singul_traj_data.start_time;
      double t_to_lc_end = traj_container_.singul_traj_data.duration - passed_t_on_lctraj;
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_container_.global_traj.glb_t_of_lc_tgt - traj_container_.global_traj.last_glb_t_of_lc_tgt);

      if(t_to_lc_end <= 0){
        RCLCPP_ERROR(node_->get_logger(),"You are initialzing a trajectory from a previous optimal trajectory, but previous trajectories are out of date.");
        return false;
      }

      /* 从旧轨迹的当前时间点往后, 为每段 singul 轨迹重建 MINCO */
      int start_piece_num = traj_container_.singul_traj_data.getPieceIdx(passed_t_on_lctraj);
      int singul_traj_num = traj_container_.singul_traj_data.singul_traj.size() - start_piece_num;
      initMJO_container.resize(singul_traj_num);
      singul_container.resize(singul_traj_num);
      int piece_nums;
      for(int i = 0; i < singul_traj_num; ++i){
        double temp_passed_t, temp_glb_t_remain;
        if(i == 0){
          temp_passed_t = passed_t_on_lctraj;
          traj_container_.singul_traj_data.locatePieceIdx(temp_passed_t);
        }else{
          temp_passed_t = 0;
        }
        if(i == singul_traj_num - 1){
          temp_glb_t_remain = t_to_lc_tgt - t_to_lc_end;
        }else{
          temp_glb_t_remain = 0;
        }
        printf("%d i, duration: %lf, passed_t: %lf, glb_t_remain: %lf", i,
               traj_container_.singul_traj_data.singul_traj[start_piece_num + i].duration,
               temp_passed_t, temp_glb_t_remain);
        double duration_now = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].duration
                              - temp_passed_t + temp_glb_t_remain;
        std::cout << i << " piece time: " << pp_.polyTraj_piece_time << "\n";
        piece_nums = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getPieceNum();
        std::cout << i << " piece num: " << piece_nums << "\n";
        if(piece_nums < 2) piece_nums = 2;

        /* 从旧轨迹上采样中间点 */
        Eigen::MatrixXd innerPs(pp_.traj_dim_, piece_nums - 1);
        std::cout << "duration: " << duration_now << "\n";
        std::cout << "singul traj duration: " << traj_container_.singul_traj_data.singul_traj[start_piece_num + i].duration << "\n";
        Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, duration_now / piece_nums);
        double t = piece_dur_vec(0);
        for (int j = 0; j < piece_nums - 1; ++j){
          if (t + temp_passed_t < traj_container_.singul_traj_data.singul_traj[start_piece_num + i].duration){
            innerPs.col(j) = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getPos(t + temp_passed_t);
            std::cout << "piece: " << start_piece_num + i << " time: " << t + temp_passed_t
                      << " local: " << innerPs.col(j).head(2).transpose() << "\n";
          }
          else if (t <= duration_now){
            double glb_t = t + temp_passed_t - traj_container_.singul_traj_data.singul_traj[start_piece_num + i].duration
                           + traj_container_.global_traj.last_glb_t_of_lc_tgt - traj_container_.global_traj.global_start_time;
            innerPs.col(j) = traj_container_.global_traj.traj.getPos(glb_t);
            std::cout << "global: " << innerPs.col(j).head(2).transpose() << "\n";
          }
          else{
            RCLCPP_ERROR(node_->get_logger(),"Should not happen! x_x 0x88");
          }
          t += piece_dur_vec(j + 1);
        }

        /* 构造头尾状态 */
        Eigen::MatrixXd headState, tailState;
        headState.resize(pp_.traj_dim_, 4);
        tailState.resize(pp_.traj_dim_, 4);
        if(i == 0){
          headState.block(0, 0, pp_.traj_dim_, 4) << start_pt, start_vel, start_acc, start_jerk;
        }else{
          headState.block(0, 0, pp_.traj_dim_, 4) << traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncPos(0),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncVel(0),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncAcc(0),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncJerk(0);
        }

        if(i == singul_traj_num - 1){
          tailState.block(0, 0, pp_.traj_dim_, 4) << local_target_pt, local_target_vel, local_target_acc, Eigen::VectorXd::Zero(pp_.traj_dim_);
        }else{
          int PN = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getPieceNum();
          tailState.block(0, 0, pp_.traj_dim_, 4) << traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncPos(PN),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncVel(PN),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncAcc(PN),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncJerk(PN);
        }

        // 用旧轨迹的中间点和时间生成新的 MINCO 初值 (热启动)
        initMJO_container[i].reset(headState, tailState, piece_nums);
        initMJO_container[i].generate(innerPs, piece_dur_vec);
        singul_container[i] = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].singul;
      }
    }

    return true;
  }

  /**
   * @brief 沿全局轨迹选取局部目标点
   *
   * 从上次选取的时间(glb_t_of_lc_tgt)开始, 沿全局轨迹向前扫描,
   * 直到距离起点达到 planning_horizen 且该点无碰撞.
   *
   * @param planning_horizen   局部规划视距 (m)
   * @param start_pt / yaw     当前机器人位置
   * @param global_end_pt/yaw  全局终点
   * @param local_target_pos/vel/acc  [输出] 局部目标状态
   * @param reach_horizon      [输出] 是否到达规划视距 (false=已到全局终点)
   *
   * 两次调用间的状态持续:
   *   glb_t_of_lc_tgt 持续前进 (不会回退), 避免重复选取同一段
   */
  void MMPlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::VectorXd &start_pt, const double &start_yaw,
      const Eigen::VectorXd &global_end_pt, const double global_end_yaw,
      Eigen::VectorXd &local_target_pos, Eigen::VectorXd &local_target_vel,Eigen::VectorXd &local_target_acc, bool &reach_horizon)
  {
    reach_horizon = true;
    double t;

    // 保存上一次的 glb_t_of_lc_tgt 为 last, 用于 Case2 热启动
    traj_container_.global_traj.last_glb_t_of_lc_tgt = traj_container_.global_traj.glb_t_of_lc_tgt;

    double t_step = planning_horizen / 20 / pp_.max_vel_;  // 自适应时间步长
    for (t = traj_container_.global_traj.glb_t_of_lc_tgt;
         t < (traj_container_.global_traj.global_start_time + traj_container_.global_traj.duration);
         t += t_step){
      Eigen::VectorXd pos_t = traj_container_.global_traj.traj.getPos(t - traj_container_.global_traj.global_start_time);

      double dist = ((pos_t - start_pt).head(pp_.mobile_base_dim_)).norm();
      if (dist >= planning_horizen){
        // 检查该点是否碰撞 (碰撞则跳过, 继续向前)
        bool occ;
        double yaw;
        Eigen::VectorXd vel;
        if ((t - traj_container_.global_traj.global_start_time) >= traj_container_.global_traj.duration){
          yaw = global_end_yaw;
        }else{
          vel = traj_container_.global_traj.traj.getVel(t - traj_container_.global_traj.global_start_time);
          yaw = atan2(vel(1), vel(0));
        }
        occ = mm_config_->checkcollision(Eigen::Vector3d(pos_t(0), pos_t(1), yaw), pos_t.tail(pp_.manipulator_dim_), false);
        if(occ) continue;  // 碰撞点跳过, 继续向前找

        local_target_pos = pos_t;
        traj_container_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    // 已扫描到全局轨迹末尾 → 终点就是局部目标
    if ((t - traj_container_.global_traj.global_start_time) >= traj_container_.global_traj.duration){
      local_target_pos = global_end_pt;
      traj_container_.global_traj.glb_t_of_lc_tgt = traj_container_.global_traj.global_start_time + traj_container_.global_traj.duration;
      reach_horizon = false;  // 标志: 局部目标就是全局终点
    }

    // 判断是否进入终点减速区 (运动学停车距离: v²/(2a))
    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_)){
      local_target_vel = Eigen::VectorXd::Zero(pp_.traj_dim_);
      local_target_vel.head(2) = pp_.mobile_base_non_singul_vel_ * Eigen::Vector2d(cos(global_end_yaw), sin(global_end_yaw));
      local_target_acc = Eigen::VectorXd::Zero(pp_.traj_dim_);
    }else{
      local_target_vel = traj_container_.global_traj.traj.getVel(t - traj_container_.global_traj.global_start_time);
      local_target_acc = traj_container_.global_traj.traj.getAcc(t - traj_container_.global_traj.global_start_time);
    }
  }

  /**
   * @brief 核心规划管线 (FSM 调用的主力接口)
   *
   * 完整流程:
   *   Step 1 — 前端初始化 (computeInitReferenceState)
   *             → A* 搜索 或 旧轨迹热启动 → MINCO 分段轨迹
   *   Step 2 — 后端优化 (OptimizeTrajectory_lbfgs)
   *             → L-BFGS 优化: obstacle + feasibility + time + snap
   *   Step 3 — 结果写入 traj_container_
   *
   * @param start_* / end_* / ...  起止状态
   * @param flag_polyInit           是否使用 A* 搜索 (false=热启动)
   * @param flag_randomPolyTraj     是否随机初值 (跳出局部最优)
   * @param have_local_traj         是否已有局部轨迹 (决定时间同步方式)
   * @param init_time, opt_time     [输出] 前端/后端耗时 (ms)
   *
   * @return true 规划成功
   */
  bool MMPlannerManager::reboundReplan(
      const Eigen::VectorXd &start_pt, const Eigen::VectorXd &start_vel,
      const Eigen::VectorXd &start_acc,const Eigen::VectorXd &start_jerk,
      const double start_yaw, const int start_singul, const bool start_gripper,
      const double trajectory_start_time,
      const Eigen::VectorXd &local_target_pt, const Eigen::VectorXd &local_target_vel,
      const Eigen::VectorXd &local_target_acc, double local_target_yaw, const bool local_target_gripper,
      const bool flag_polyInit, const bool flag_randomPolyTraj,
      const bool have_local_traj, double &init_time, double &opt_time)
  {
    static int count = 0;
    printf("\033[47;30m\n[replan %d]==============================================\033[0m\n", count++);

    rclcpp::Time t_start = node_->now();
    rclcpp::Duration t_init(0, 0), t_opt(0, 0);

    /**
     * @brief STEP 1: 前端初始化 — 生成 MINCO 初值
     */
    std::vector<poly_traj::MinSnapOpt<8>> initMJO_container;  // 每段 singul 对应的 MINCO 优化器
    std::vector<int> singul_container;                         // 每段的 singul 标志
    if (!computeInitReferenceState(start_pt, start_vel, start_acc, start_jerk, start_yaw, start_singul, start_gripper,
                                   local_target_pt, local_target_vel, local_target_acc, local_target_yaw, local_target_gripper,
                                   initMJO_container, singul_container,
                                   flag_polyInit, continous_failures_count_)){
      ++continous_failures_count_;
      return false;  // 前端搜索失败
    }

    // 统计初始轨迹信息 (长度、时长) — 仅用于日志
    Eigen::VectorXd init_len(7);
    double init_dura = 0.0;
    init_len.setZero();
    for(unsigned int i = 0; i < initMJO_container.size(); ++i){
      Eigen::MatrixXd cst_pts = initMJO_container[i].getInitConstrainPoints(1);
      Eigen::VectorXd time_list = initMJO_container[i].get_T1();
      Eigen::VectorXd pos1 = cst_pts.col(0), pos2;
      init_dura += time_list.lpNorm<1>();
      for(unsigned int j = 1; j < cst_pts.cols(); ++j){
        pos2 = cst_pts.col(j);
        init_len(0) += (pos2 - pos1).head(2).norm();
        init_len.tail(6) += ((pos2 - pos1).tail(6)).cwiseAbs();
        pos1 = pos2;
      }
    }

    // 大小检查
    if(initMJO_container.size() != singul_container.size()){
      RCLCPP_ERROR(node_->get_logger(),"initMJO_container size = %d != singul_container size = %d",
                (int)initMJO_container.size(), (int)singul_container.size());
    }

    /* ---- 提取优化器的控制点、起止状态、中间点 ---- */
    ploy_traj_opt_->clear_resize_Cps_container(initMJO_container.size());
    std::vector<Eigen::Vector2d> disp_point_set;
    std::vector<Eigen::MatrixXd> iniStates_container;
    std::vector<Eigen::MatrixXd> finStates_container;
    std::vector<Eigen::MatrixXd> initInnerPts_container;
    std::vector<Eigen::VectorXd> initT_container;
    Eigen::MatrixXd cstr_pts;
    for(unsigned int i = 0; i < initMJO_container.size(); ++i){
      cstr_pts = initMJO_container[i].getInitConstrainPoints(ploy_traj_opt_->get_cps_num_prePiece_());
      ploy_traj_opt_->setControlPoints(i, cstr_pts);

      poly_traj::Trajectory<7> initTraj = initMJO_container[i].getTraj(singul_container[i]);

      initT_container.push_back(initTraj.getDurations());

      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, pp_.traj_dim_, PN - 1);
      initInnerPts_container.push_back(innerPts);

      Eigen::MatrixXd headState, tailState;
      headState.resize(pp_.traj_dim_, 4);
      tailState.resize(pp_.traj_dim_, 4);
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0), initTraj.getJuncJerk(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN), initTraj.getJuncJerk(PN);
      iniStates_container.push_back(headState);
      finStates_container.push_back(tailState);
      for (int j = 0; j < cstr_pts.cols(); ++j)
        disp_point_set.push_back(cstr_pts.col(j).head(2));
    }
    visualization_->displayInitCtrlPts(disp_point_set, 0.06, 0);  // 显示前端控制点

    t_init = node_->now() - t_start;  // 前端耗时
    t_start = node_->now();

    /**
     * @brief STEP 2: 后端优化 — L-BFGS 轨迹优化
     */
    bool flag_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    Eigen::MatrixXd opt_waypoints;
    Eigen::VectorXd time_list;
    vector<vector<Eigen::Vector2d>> his_trajs;

    std::vector<Eigen::VectorXd> optT_container, optEECps_container;
    std::vector<Eigen::MatrixXd> optWps_container;
    std::vector<Eigen::MatrixXd> optCps_container;

    // 核心优化调用:
    //   输入: iniStates (起止状态), initInnerPts (中间点), initT (段时间), singul (换向标志)
    //   输出: optCps (控制点), optWps (优化后路径点), optT (优化后时间), optEECps
    flag_success = ploy_traj_opt_->OptimizeTrajectory_lbfgs(iniStates_container, finStates_container,
                                                            initInnerPts_container, initT_container, singul_container,
                                                            optCps_container, optWps_container, optT_container, optEECps_container);
    t_opt = node_->now() - t_start;  // 后端耗时

    // 统计优化后轨迹 (snap 代价 + 长度 + 时长)
    double snap_cost = 0.0, traj_dura = 0.0;
    Eigen::VectorXd traj_len(7);
    traj_len.setZero();
    double traj_time;
    Eigen::VectorXd pos1, pos2;
    for(unsigned int i = 0; i < singul_container.size(); ++i){
      snap_cost += (*ploy_traj_opt_->getMinSnapOptContainerPtr())[i].getTrajJerkCost();
      traj_time = (*ploy_traj_opt_->getMinSnapOptContainerPtr())[i].getTraj(singul_container[i]).getTotalDuration();
      traj_dura += traj_time;

      pos1 = (*ploy_traj_opt_->getMinSnapOptContainerPtr())[i].getTraj(singul_container[i]).getPos(0.0);
      for(double j = 0.0; j < traj_time + 1.0e-3; j += 0.01){
        pos2 = (*ploy_traj_opt_->getMinSnapOptContainerPtr())[i].getTraj(singul_container[i]).getPos(j);
        traj_len(0) += (pos2 - pos1).head(2).norm();
        traj_len.tail(6) += ((pos2 - pos1).tail(6)).cwiseAbs();
        pos1 = pos2;
      }
    }

    // 优化失败处理
    if(!flag_success){
      for(unsigned int i = 0; i < optCps_container.size(); ++i){
        visualization_->displayFailedList(optCps_container[i], i);  // 显示失败路径
      }
      continous_failures_count_++;  // 累计失败次数 (影响下次前端策略)
      return false;
    }

    // 统计成功规划的平均耗时
    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt).seconds();
    count_success++;
    init_time = t_init.seconds() * 1000;   // 转为 ms
    opt_time = t_opt.seconds() * 1000;     // 转为 ms
    cout << "\033[34mtotal time: " << (t_init + t_opt).seconds() * 1000
         << ", init: " << t_init.seconds() * 1000
         << ", optimize: " << t_opt.seconds() * 1000
         << ", avg_time: " << sum_time / count_success * 1000.0
         << ", count_success: " << count_success << "\033[0m"<< endl;
    average_plan_time_ = sum_time / count_success;

    /**
     * @brief STEP 3: 写入轨迹容器
     */

    // 时间同步: 如果是从旧轨迹热启动, 需要 sleep 等待到 trajectory_start_time
    double traj_start_time;
    if(have_local_traj){
      double delta_replan_time = trajectory_start_time - node_->now().seconds();
      if (delta_replan_time > 0) {
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(delta_replan_time)));
      }
      traj_start_time = trajectory_start_time;
    }
    else{
      traj_start_time = node_->now().seconds();
    }

    // 将优化后的各段轨迹写入容器 (按 singul 分段的顺序)
    traj_container_.singul_traj_data.clearSingulTraj();
    for(unsigned int i = 0; i < singul_container.size(); ++i){
      traj_container_.singul_traj_data.addSingulTraj(
          (*ploy_traj_opt_->getMinSnapOptContainerPtr())[i].getTraj(singul_container[i]), traj_start_time);
      traj_start_time = traj_container_.singul_traj_data.singul_traj.back().end_time;
    }

    // 可视化: 优化后的控制点与路径点
    visualization_->displayOptimalCtrlPts(optCps_container, 0);

    vector<Eigen::Vector2d> display_pts;
    Eigen::Vector2d pts;
    for(unsigned int i = 0; i < optWps_container.size(); ++i){
      display_pts.clear();
      opt_waypoints = optWps_container[i];
      time_list = optT_container[i];
      pts = opt_waypoints.col(0).head(2);
      display_pts.push_back(pts);
      for(unsigned int j = 1; j < opt_waypoints.cols(); ++j)
      {
        pts = opt_waypoints.col(j).head(2);
        display_pts.push_back(pts);
      }
      visualization_->displayOptWaypoints(display_pts, 0.2, i);
    }

    // 成功: 重置失败计数
    continous_failures_count_ = 0;
    return true;
  }

  /**
   * @brief 紧急停止轨迹生成
   *
   * 生成一条零速停止轨迹 (位置不变, vel=acc=jerk=0)
   * 轨迹为 2 段 MINCO 多项式, 时长 1.0s
   *
   * @param stop_pos  停止位置
   * @param stop_yaw  停止朝向
   * @param singul    停止时的前进/后退标志
   */
  bool MMPlannerManager::EmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw,  const int singul){
    auto ZERO = Eigen::VectorXd::Zero(pp_.traj_dim_);
    Eigen::MatrixXd headState, tailState;
    headState.resize(pp_.traj_dim_, 4);
    tailState.resize(pp_.traj_dim_, 4);
    headState.block(0, 0, pp_.traj_dim_, 4) << stop_pos, ZERO, ZERO, ZERO;
    tailState = headState;                                   // 起点=终点, vel=acc=jerk=0
    poly_traj::MinSnapOpt<8> stopMJO;
    stopMJO.reset(headState, tailState, 2);                  // 2段轨迹
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));  // 每段 1.0s

    traj_container_.singul_traj_data.clearSingulTraj();      // 清除旧轨迹
    traj_container_.singul_traj_data.addSingulTraj(stopMJO.getTraj(singul), node_->now().seconds());

    return true;
  }

  /**
   * @brief 生成全局参考轨迹 (通过航点)
   *
   * 使用 MINCO (MinSnapOpt) 连接一系列航点, 生成平滑的全局参考轨迹.
   * 时间分配基于基底速度 max_vel/1.5 和机械臂速度 max_mani_vel 的较大值.
   * 如果生成轨迹的最大速度超过 max_vel, 会降低期望速度重试 (最多5次).
   *
   * @param start_pos / yaw / vel / acc  起始状态
   * @param waypoints                     航点序列 [x, y, q1, ..., qN]
   * @param end_yaw / vel / acc           终点的约束
   *
   * 结果写入 traj_container_.global_traj
   *
   * FIXME: singul 处理 — 当前仅支持前进(singul=1)
   */
  bool MMPlannerManager::planGlobalTrajWaypoints(
      const Eigen::VectorXd &start_pos, const double start_yaw,
      const Eigen::VectorXd &start_vel, const Eigen::VectorXd &start_acc,
      const std::vector<Eigen::VectorXd> &waypoints,
      const double end_yaw, const Eigen::VectorXd &end_vel, const Eigen::VectorXd &end_acc)
  {
    int start_singul = 1;  // FIXME: 目前只支持前进
    poly_traj::MinSnapOpt<8> globalMJO;
    Eigen::MatrixXd headState, tailState;
    headState.resize(pp_.traj_dim_, 4);
    tailState.resize(pp_.traj_dim_, 4);
    headState << start_pos, start_vel, start_acc, Eigen::VectorXd::Zero(pp_.traj_dim_);
    tailState << waypoints.back(), end_vel, end_acc, Eigen::VectorXd::Zero(pp_.traj_dim_);
    Eigen::MatrixXd innerPts;

    if (waypoints.size() > 1)
    {
      innerPts.resize(pp_.traj_dim_, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; i++)
        innerPts.col(i) = waypoints[i];
    }

    globalMJO.reset(headState, tailState, waypoints.size());

    // 时间分配先按位移估计，再对生成的多项式进行全维度密集采样。
    // 旧实现只降低底盘 des_vel；当关节峰值超限时，机械臂段时长完全不变，
    // 因而会重复生成同一条不可行轨迹。
    const double des_vel = pp_.max_vel_ / 1.5;
    constexpr double kLimitReserve = 0.90;
    Eigen::VectorXd time_vec(waypoints.size());
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
      const Eigen::VectorXd delta = i == 0 ?
        waypoints[0] - start_pos : waypoints[i] - waypoints[i - 1];
      time_vec(i) = delta.head(pp_.mobile_base_dim_).norm() / des_vel;
      for(int j = 0; j < pp_.manipulator_dim_; ++j){
        // Cubic/quintic trajectories peak above their average velocity. Start
        // with a 1.5 factor and let dense sampling below add any extra scaling.
        const double t_temp = 1.5 * std::abs(
          delta[pp_.mobile_base_dim_ + j]) /
          std::max(1.0e-3, kLimitReserve * pp_.max_mani_vel_);
        time_vec(i) = std::max(time_vec(i), t_temp);
      }
      time_vec(i) = std::max(time_vec(i), 0.20);
    }

    double sampled_base_vel = 0.0;
    double sampled_joint_vel = 0.0;
    double sampled_joint_acc = 0.0;
    for (int attempt = 0; attempt < 8; ++attempt)
    {
      globalMJO.generate(innerPts, time_vec);
      const auto candidate = globalMJO.getTraj(start_singul);
      const double duration = candidate.getTotalDuration();
      sampled_base_vel = sampled_joint_vel = sampled_joint_acc = 0.0;
      const int samples = std::max(100, static_cast<int>(std::ceil(duration / 0.01)));
      for (int sample = 0; sample <= samples; ++sample)
      {
        const double t = duration * static_cast<double>(sample) /
          static_cast<double>(samples);
        const Eigen::VectorXd vel = candidate.getVel(t);
        const Eigen::VectorXd acc = candidate.getAcc(t);
        sampled_base_vel = std::max(sampled_base_vel,
          vel.head(pp_.mobile_base_dim_).norm());
        sampled_joint_vel = std::max(sampled_joint_vel,
          vel.tail(pp_.manipulator_dim_).cwiseAbs().maxCoeff());
        sampled_joint_acc = std::max(sampled_joint_acc,
          acc.tail(pp_.manipulator_dim_).cwiseAbs().maxCoeff());
      }
      const double scale = std::max({
        1.0,
        sampled_base_vel / std::max(1.0e-3, kLimitReserve * pp_.max_vel_),
        sampled_joint_vel /
          std::max(1.0e-3, kLimitReserve * pp_.max_mani_vel_),
        std::sqrt(sampled_joint_acc /
          std::max(1.0e-3, kLimitReserve * pp_.max_mani_acc_))});
      if (scale <= 1.001)
        break;
      time_vec *= 1.05 * scale;
    }

    RCLCPP_INFO(node_->get_logger(),
      "Global reference time-scaled: duration=%.2f s, sampled base=%.3f m/s, "
      "joint=%.3f rad/s, joint_acc=%.3f rad/s^2",
      time_vec.sum(), sampled_base_vel, sampled_joint_vel, sampled_joint_acc);

    auto time_now = node_->now();
    traj_container_.setGlobalTraj(globalMJO.getTraj(start_singul), time_now.seconds());

    return true;
  }

} // namespace remani_planner
