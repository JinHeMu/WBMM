/**
 * @file rrt.cpp
 * @brief 全身 RRT 规划器实现 — 移动机械臂的 Bidirectional RRT* 搜索
 *
 * ============================================================
 * 核心职责: 在全身状态空间 (x, y, yaw, q1...qN) 中搜索无碰撞路径
 * ============================================================
 *
 * 算法: Bidirectional RRT* (双向 + 渐进最优)
 *   - 正向树 (IN_TREE):     从起点出发
 *   - 反向树 (IN_ANTI_TREE): 从终点出发
 *   - 当两棵树相遇时成功
 *
 * 基底路径: Dubins 曲线 (OMPL) 用于连接基底状态
 * 机械臂:   在 Dubins 曲线采样点上线性插值
 * 采样:     基于 Informed RRT* 的椭球采样 (有路径后缩小采样空间)
 *
 * @see sample_mani_RRT.cpp (调用者, 提供起止点列表)
 */

#include "path_searching/rrt.h"

namespace remani_planner{
  template<typename T>
  static void getRrtParam(const rclcpp::Node::SharedPtr &node, const std::string &name,
                          T &value, const T &default_value) {
    if (!node->has_parameter(name)) {
      node->declare_parameter<T>(name, default_value);
    }
    node->get_parameter(name, value);
  }

  // ============================================================================
  // 主入口: RRT 搜索 → 路径提取 (供 SampleMani 调用)
  // ============================================================================
  int RrtPlanning::RRTSearchAndGetSimplePath(
      const std::vector<Eigen::VectorXd>& start_pt_list, const std::vector<double>& start_yaw_list,
      const std::vector<Eigen::VectorXd>& end_pt_list, const std::vector<double>& end_yaw_list,
      const std::vector<double>& start_g_score_list, const std::vector<int>& start_layer_list,
      const std::vector<double>& end_g_score_list, const std::vector<int>& end_layer_list,
      const std::vector<int>& start_singul_list, const std::vector<int>& end_singul_list,
      std::vector<Eigen::VectorXd>& path, std::vector<double>& yaw_list, std::vector<double>& t_list){
    std::vector<Eigen::VectorXd> path_full;
    std::vector<double> t_list_full, yaw_list_full;
    if(start_pt_list.empty()){
      RCLCPP_ERROR(node_->get_logger(), "[RRT plan]: start pt < 1");
      return false;
    }
    if(end_pt_list.empty()){
      RCLCPP_ERROR(node_->get_logger(), "[RRT plan]: end pt < 1");
      return false;
    }
    if(start_yaw_list.size() != start_pt_list.size()
       || start_g_score_list.size() != start_pt_list.size()
       || start_layer_list.size() != start_pt_list.size()
       || start_singul_list.size() != start_pt_list.size()
       || end_yaw_list.size() != end_pt_list.size()
       || end_g_score_list.size() != end_pt_list.size()
       || end_layer_list.size() != end_pt_list.size()
       || end_singul_list.size() != end_pt_list.size()){
      RCLCPP_ERROR(node_->get_logger(), "[RRT plan]: inconsistent seed-list sizes");
      return false;
    }

    init(start_pt_list, end_pt_list);
    bool status = search(start_pt_list, start_yaw_list, end_pt_list, end_yaw_list,
                         start_g_score_list, start_layer_list, end_g_score_list, end_layer_list,
                         start_singul_list, end_singul_list);

    if (status == false){
      return status;
    }
    if(!getTraj(path_full, yaw_list_full, t_list_full)
       || path_full.empty()
       || yaw_list_full.size() != path_full.size()
       || t_list_full.size() + 1 != path_full.size()){
      RCLCPP_ERROR(node_->get_logger(), "[RRT plan]: invalid or empty result");
      return false;
    }

    // 输出格式转换: t_list 存放每个节点间的累积时间
    path.clear();
    t_list.clear();
    yaw_list.clear();
    path.push_back(path_full[0]);
    yaw_list.push_back(yaw_list_full[0]);
    double t_total = 0.0;
    for(size_t i = 0; i < t_list_full.size(); i++){
      t_total += t_list_full[i];
      path.push_back(path_full[i + 1]);
      yaw_list.push_back(yaw_list_full[i + 1]);
      t_list.push_back(t_total);
      t_total = 0.0;
    }

    return status;
  }

  // ============================================================================
  // Bidirectional RRT* 搜索主体
  //
  // 输入: 多个起点/终点 (由 SampleMani 传入)
  //   每棵树可以有多个根节点 (降低失败率)
  //
  // 算法流程:
  //   1. 将所有起点插入 IN_TREE, 所有终点插入 IN_ANTI_TREE
  //   2. 交替扩展两棵树 (谁节点少谁扩展)
  //   3. sample() → near() → steer() → 碰撞检查
  //   4. 使用 RRT* 的 rewire 优化路径
  //   5. 两棵树相遇时记录最优路径 (最小化 g1 + g2 + heuristic)
  //   6. 合并两棵树 → 输出完整路径
  // ============================================================================
  bool RrtPlanning::search(const std::vector<Eigen::VectorXd>& start_pt_list, const std::vector<double>& start_yaw_list,
                          const std::vector<Eigen::VectorXd>& end_pt_list, const std::vector<double>& end_yaw_list,
                          const std::vector<double>& start_g_score_list, const std::vector<int>& start_layer_list,
                          const std::vector<double>& end_g_score_list, const std::vector<int>& end_layer_list,
                          const std::vector<int>& start_singul_list, const std::vector<int>& end_singul_list){
    const rclcpp::Time time_1 = node_->now();
    PathNodeRRTPtr start_node, end_node;
    PathNodeRRTPtr path_node_1 = nullptr, path_node_2 = nullptr;

    // ---- 初始化起点树 (IN_TREE) ----
    int start_num = start_pt_list.size();
    if(start_num < 1){
      RCLCPP_ERROR(node_->get_logger(), "[RRT]: Fail! Start list size zero!");
      return false;
    }
    for(int i = 0; i < start_num; ++i){
      start_node = new PathNodeRRT(traj_dim_);
      start_node->state = start_pt_list[i];
      start_node->yaw = start_yaw_list[i];
      start_node->node_state = PathNodeRRT::NODE_STATE::IN_TREE;
      start_node->g_score = start_g_score_list[i];
      start_node->index = calculateValue(start_node);
      start_node->layer = start_layer_list[i];
      start_node->singul = start_singul_list[i];
      node_pool_.insert(std::make_pair(start_node->index, start_node));
      ++tree_count_;
    }

    // ---- 初始化终点树 (IN_ANTI_TREE) ----
    int end_num = end_pt_list.size();
    if(end_num < 1){
      RCLCPP_ERROR(node_->get_logger(), "[RRT]: Fail! End list size zero!");
      return false;
    }
    for(int i = 0; i < end_num; ++i){
      end_node = new PathNodeRRT(traj_dim_);
      end_node->state = end_pt_list[i];
      end_node->yaw = end_yaw_list[i];
      end_node->node_state = PathNodeRRT::NODE_STATE::IN_ANTI_TREE;
      end_node->g_score = end_g_score_list[i];
      end_node->index = calculateValue(end_node);
      end_node->layer = end_layer_list[i];
      end_node->singul = end_singul_list[i];
      node_pool_.insert(std::make_pair(end_node->index, end_node));
      ++anti_tree_count_;
    }

    // ---- 主循环 ----
    int loop = 0;
    PathNodeRRTPtr q_new, q_near;
    PathNodeRRTPtr q_new_1, q_near_1;
    PathNodeRRTPtr q_new_2;
    Eigen::VectorXd rand_s;
    double rand_yaw;
    bool dir = false;

    while(true){
      // 超时检查
      if( (have_path_ && (node_->now() - time_1).seconds() > 0.01)
          || (node_->now() - time_1).seconds() > max_sample_time_){
        break;
      }

      // 交替扩展: 节点少的树优先扩展 (平衡生长)
      if(tree_count_ > anti_tree_count_){
        dir = false;
      }else{
        dir = true;
      }
      ++loop;

      // 随机采样状态
      sample(rand_s, rand_yaw);
      q_near = near(rand_s, rand_yaw, dir);       // 找最近邻
      if(q_near == nullptr) continue;

      // 从近邻向采样点扩展 (Dubins曲线 + 机械臂线性插值)
      q_new = steer(q_near, rand_s, rand_yaw, time_resolution_);
      if(q_new == nullptr) continue;

      // ---- 两棵树相遇检测 ----
      if((dir && q_new->node_state == PathNodeRRT::IN_ANTI_TREE)
         || (!dir && q_new->node_state == PathNodeRRT::IN_TREE)){
        // RRT*: 记录代价更小的路径
        if(q_near->g_score + q_new->g_score + estimateHeuristic(q_near, q_new) < c_max_){
          c_max_ = q_near->g_score + q_new->g_score + estimateHeuristic(q_near, q_new);
          path_node_1 = q_near;
          path_node_2 = q_new;
          have_path_ = true;
        }
        continue;
      }

      // ---- 扩展节点到树中 ----
      if(q_new->node_state == PathNodeRRT::EXPAND
         || (q_new->node_state == q_near->node_state
             && q_new->g_score > q_near->g_score + estimateHeuristic(q_near, q_new))){
        linkNode(q_near, q_new);
        q_new->node_state = q_near->node_state;
        if(q_new->node_state == PathNodeRRT::IN_TREE)
          ++tree_count_;
        else
          ++anti_tree_count_;
        rewire(q_new, 0.45);  // RRT* 重新布线

        // ---- 从另一棵树也扩展, 尝试连接 ----
        q_near_1 = near(q_new->state, q_new->yaw, !dir);
        if(q_near_1 == nullptr) continue;

        q_new_1 = steer(q_near_1, q_new->state, q_new->yaw, time_resolution_);
        if(q_new_1 == nullptr) continue;

        // 相遇检测
        if((!dir && q_new_1->node_state == PathNodeRRT::IN_ANTI_TREE)
           || (dir && q_new_1->node_state == PathNodeRRT::IN_TREE)){
          if(q_new_1->g_score + q_near_1->g_score + estimateHeuristic(q_new_1, q_near_1) < c_max_){
            c_max_ = q_new_1->g_score + q_near_1->g_score + estimateHeuristic(q_new_1, q_near_1);
            path_node_1 = q_new_1;
            path_node_2 = q_near_1;
            have_path_ = true;
          }
          continue;
        }

        // ---- 连接两棵树之间的剩余间隙 ----
        if(q_new_1->node_state == PathNodeRRT::EXPAND
           || (q_new_1->node_state == q_near_1->node_state
               && q_new_1->g_score > q_near_1->g_score + estimateHeuristic(q_near_1, q_new_1))){
          linkNode(q_near_1, q_new_1);
          q_new_1->node_state = q_near_1->node_state;
          rewire(q_new_1, 0.45);
          if(q_new_1->node_state == PathNodeRRT::IN_TREE)
            ++tree_count_;
          else
            ++anti_tree_count_;

          // 用桥接节点填补间隙: 从 q_new_1 向 q_new 逐步扩展
          while( !((q_new->state - q_new_1->state).norm() < 1.0e-2
                   && fabs(q_new->yaw - q_new_1->yaw) < 1.0e-2) ){
            q_new_2 = steer(q_new_1, q_new->state, q_new->yaw, time_resolution_);
            if(q_new_2 != nullptr
               && (q_new_2->node_state == PathNodeRRT::EXPAND
                   || (q_new_2->node_state == q_new_1->node_state
                       && q_new_2->g_score > q_new_1->g_score + estimateHeuristic(q_new_1, q_new_2)))){
              linkNode(q_new_1, q_new_2);
              q_new_2->node_state = q_new_1->node_state;
              rewire(q_new_2, 0.45);
              if(q_new_2->node_state == PathNodeRRT::IN_TREE)
                ++tree_count_;
              else
                ++anti_tree_count_;
              q_new_1 = q_new_2;
            }
            else if(q_new_2 != nullptr
                    && ((dir && q_new_2->node_state == PathNodeRRT::IN_TREE)
                        || (!dir && q_new_2->node_state == PathNodeRRT::IN_ANTI_TREE))){
              if(q_new_2->g_score + q_new_1->g_score + estimateHeuristic(q_new_2, q_new_1) < c_max_){
                c_max_ = q_new_2->g_score + q_new_1->g_score + estimateHeuristic(q_new_2, q_new_1);
                path_node_1 = q_new_2;
                path_node_2 = q_new_1;
                have_path_ = true;
              }
              break;
            }
            else if(q_new_2 != nullptr
                    && q_new_2->node_state == q_new_1->node_state
                    && q_new_2->g_score < q_new_1->g_score + estimateHeuristic(q_new_1, q_new_2)){
              q_new_1 = q_new_2;
            }
            else{
              break;
            }
          }
        }
      }
    }

    if(!have_path_ || path_node_1 == nullptr || path_node_2 == nullptr){
      return false;
    }else{
      // 合并两棵树 → 完整路径
      mergeTree(path_node_1, path_node_2);
      return true;
    }
  }

  // ============================================================================
  // 初始化/获取节点 (如果已存在则返回, 否则新建)
  // ============================================================================
  PathNodeRRTPtr RrtPlanning::initNode(const Eigen::VectorXd &s, const double yaw){
    auto it = node_pool_.find(calculateValue(s, yaw));
    if(it != node_pool_.end()){
      return it->second;
    }
    PathNodeRRTPtr node = new PathNodeRRT;
    node->node_state = PathNodeRRT::NODE_STATE::EXPAND;
    node->state = s;
    node->yaw = yaw;
    node->index = calculateValue(node);
    node_pool_.insert(std::make_pair(node->index, node));
    return node;
  }

  // ============================================================================
  // 采样: Informed RRT* 椭球采样 (有路径后) / 均匀采样 (初始)
  //
  // 椭球采样: 以 start-end 为焦点的超椭球体
  //   长轴: c_max_  (当前最优路径长度)
  //   短轴: √(c_max² - c_min²)
  //   通过 SVD 分解旋转到正确朝向
  // ============================================================================
  void RrtPlanning::sample(Eigen::VectorXd &s_state, double &s_yaw){
    Eigen::VectorXd sample_state(traj_dim_);
    if(c_max_ < 1.0e5){
      // --- Informed RRT* 椭球采样 ---
      Eigen::VectorXd s_center = (sample_start_ + sample_end_) / 2.0;
      Eigen::VectorXd a1 = (sample_end_ - sample_start_).normalized();
      Eigen::MatrixXd M = a1 * Eigen::VectorXd::Ones(traj_dim_).transpose();
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
      Eigen::MatrixXd V = svd.matrixV();
      Eigen::MatrixXd U = svd.matrixU();
      Eigen::MatrixXd C_middle(traj_dim_, traj_dim_);
      C_middle.setZero();
      for(int i = 0; i < traj_dim_ - 1; ++i)
        C_middle(i, i) = 1.0;
      C_middle(traj_dim_ - 1, traj_dim_ - 1) = U.determinant() * V.determinant();
      Eigen::MatrixXd C = U * C_middle;

      Eigen::MatrixXd L(traj_dim_, traj_dim_);
      L.setZero();
      double c_max = c_max_ + (sample_start_radius_ + sample_end_radius_) / 2.0;
      for(int i = 1; i < traj_dim_; ++i)
        L(i, i) = sqrt(c_max * c_max - c_min_ * c_min_) / 2;
      L(0, 0) = c_max / 2.0;

      // 单位球内均匀采样
      for(int i = 0; i < traj_dim_; ++i){
        sample_state(i) = norm_dis_(random_gen_);
      }
      sample_state.normalize();
      double r = pow(random_dis_(random_gen_), 1.0/traj_dim_);
      sample_state *= r;

      // 变换到椭球
      s_state = C * L * sample_state + s_center;
      s_yaw = min_size_[traj_dim_] + (max_size_[traj_dim_] - min_size_[traj_dim_]) * random_dis_(random_gen_);

      // 钳位到边界
      for(int i = 0; i < traj_dim_; ++i){
        s_state[i] = min(max(min_size_[i], s_state[i]), max_size_[i]);
      }
    }
    else{
      // --- 均匀采样 ---
      for(int i = 0; i < traj_dim_; ++i){
        sample_state(i) = min_size_[i] + (max_size_[i] - min_size_[i]) * random_dis_(random_gen_);
      }
      s_state = sample_state;
      s_yaw = min_size_[traj_dim_] + (max_size_[traj_dim_] - min_size_[traj_dim_]) * random_dis_(random_gen_);
    }
  }

  // ============================================================================
  // 最近邻搜索 (暴力遍历)
  //
  // @param s    随机采样状态
  // @param yaw  随机采样朝向
  // @param flag true=在 IN_TREE 中找, false=在 IN_ANTI_TREE 中找
  // ============================================================================
  PathNodeRRTPtr RrtPlanning::near(const Eigen::VectorXd &s, const double yaw, bool flag){
    PathNodeRRTPtr q_near = nullptr;
    PathNodeRRTPtr node;
    double min_dis = 1.0e6;
    for(auto it = node_pool_.begin(); it != node_pool_.end(); ++it){
      node = it->second;
      if((flag && node->node_state == PathNodeRRT::NODE_STATE::IN_TREE)
         || (!flag && node->node_state == PathNodeRRT::NODE_STATE::IN_ANTI_TREE)){
        if(((fabs(yaw - node->yaw) > 1.0e-2) || ((s - node->state).norm() > 1.0e-2))
           && estimateHeuristic(s, yaw, node->state, node->yaw) < min_dis){
          min_dis = estimateHeuristic(s, yaw, node->state, node->yaw);
          q_near = node;
        }
      }
    }
    return q_near;
  }

  // ============================================================================
  // 控制扩展: 从 q_near 向目标 (s_rand, s_yaw) 扩展一步
  //
  // 使用 Dubins 曲线连接基底状态, 机械臂关节线性插值.
  // 扩展距离受 max_vel × step_time 限制.
  //
  // @param  q_near     当前节点
  // @param  s_rand     目标位置 (含机械臂)
  // @param  s_yaw      目标朝向
  // @param  step_time  扩展时间步长
  // @return q_new      新节点 (nullptr 表示扩展失败)
  // ============================================================================
  PathNodeRRTPtr RrtPlanning::steer(PathNodeRRTPtr &q_near, const Eigen::VectorXd &s_rand,
                                     double s_yaw, double step_time){
    Eigen::VectorXd dir = s_rand - q_near->state;
    Eigen::VectorXd s_new(traj_dim_);
    int singul;

    // 判断前进/后退方向
    if(q_near->parent == nullptr){
      singul = ((s_rand - q_near->state).head(2)).dot(
                 Eigen::Vector2d(cos(q_near->yaw), sin(q_near->yaw))) >= 0 ? 1 : -1;
    }else{
      singul = ((s_rand - q_near->state).head(2)).dot(
                 (q_near->state - q_near->parent->state).head(2)) >= 0 ? q_near->singul : -q_near->singul;
    }
    if(q_near->singul != 0 && singul != q_near->singul)
      singul = q_near->singul;

    // Dubins 曲线: 如果后退则 yaw + π
    ompl::base::ScopedState<> from(dubins_curve_), to(dubins_curve_), s(dubins_curve_);
    from[0] = q_near->state[0]; from[1] = q_near->state[1];
    if(singul == 1)
      from[2] = q_near->yaw;
    else{
      from[2] = q_near->yaw + M_PI;
      if(from[2] > M_PI) from[2] -= 2 * M_PI;
    }
    to[0] = s_rand[0]; to[1] = s_rand[1];
    if(singul == 1)
      to[2] = s_yaw;
    else{
      to[2] = s_yaw + M_PI;
      if(to[2] > M_PI) to[2] -= 2 * M_PI;
    }

    double len = dubins_curve_->distance(from(), to());
    double yaw;
    if(len > max_vel_ * step_time){
      // 限制步长: 沿 Dubins 曲线走一步
      dubins_curve_->interpolate(from(), to(), max_vel_ * step_time / len, s());
      auto reals = s.reals();
      s_new(0) = reals[0];
      s_new(1) = reals[1];
      yaw = reals[2];
    }else{
      // 直接到达目标
      s_new.head(mobile_base_dof_) = s_rand.head(mobile_base_dof_);
      yaw = to[2];
    }

    if(singul == -1){
      yaw += M_PI;
      if(yaw > M_PI) yaw -= 2 * M_PI;
    }

    // 机械臂关节线性插值 (限制速度)
    if(dir.tail(manipulator_dof_).norm() > 1.0e-2){
      s_new.tail(manipulator_dof_) = q_near->state.tail(manipulator_dof_)
                                   + dir.tail(manipulator_dof_).normalized() * max_joint_vel_ * step_time;
    }else{
      s_new.tail(manipulator_dof_) = q_near->state.tail(manipulator_dof_);
    }

    // 钳位: 确保机械臂不超出插值范围
    for(int i = mobile_base_dof_; i < traj_dim_; ++i){
      if(s_new(i) < std::min(s_rand(i), q_near->state(i))
         || s_new(i) > std::max(s_rand(i), q_near->state(i))){
        s_new(i) = s_rand(i);
      }
    }

    // 碰撞检查
    if(checkcollision(q_near, s_new, yaw)){
      return nullptr;
    }

    PathNodeRRTPtr q_new = initNode(s_new, yaw);
    return q_new;
  }

  // ============================================================================
  // RRT* 重新布线: 在临近区域内优化父节点选择
  //
  // 对 q_new 附近的节点:
  //   1. 如果邻节点 + heuristic 更小 → 选邻节点为父节点
  //   2. 如果 q_new + heuristic 更小 → 重设邻节点的父节点为 q_new
  // ============================================================================
  void RrtPlanning::rewire(PathNodeRRTPtr q_new, double near_time){
    std::vector<PathNodeRRTPtr> neighbour;
    PathNodeRRTPtr temp;
    for(auto it = node_pool_.begin(); it != node_pool_.end(); ++it){
      bool flag = false;
      temp = it->second;
      if(temp->node_state != q_new->node_state || temp == q_new) continue;
      // 距离阈值: 基底距离
      if((temp->state.head(mobile_base_dof_) - q_new->state.head(mobile_base_dof_)).norm()
         > max_vel_ * near_time) continue;
      // 距离阈值: 关节距离
      for(int j = 0; j < manipulator_dof_; ++j){
        if(calAngleErr(temp->state(mobile_base_dof_ + j), q_new->state(mobile_base_dof_ + j))
           > max_joint_vel_ * near_time){
          flag = true;
          break;
        }
      }
      if(flag) continue;
      neighbour.push_back(temp);
    }

    if(neighbour.size() < 1) return;

    // 尝试为 q_new 找更好的父节点
    temp = nullptr;
    double min = q_new->g_score;
    for(int i = 0; i < (int)neighbour.size(); ++i){
      if(neighbour[i]->g_score + estimateHeuristic(neighbour[i], q_new) < min
         && !checkcollision(neighbour[i], q_new)){
        min = neighbour[i]->g_score + estimateHeuristic(neighbour[i], q_new);
        temp = neighbour[i];
      }
    }
    if(temp != nullptr) linkNode(temp, q_new);

    // 尝试将邻节点的父节点设为 q_new
    for(int i = 0; i < (int)neighbour.size(); ++i){
      if(q_new->g_score + estimateHeuristic(q_new, neighbour[i]) < neighbour[i]->g_score
         && !checkcollision(neighbour[i], q_new)){
        linkNode(q_new, neighbour[i]);
      }
    }
  }

  // ============================================================================
  // 节点连接: 设置 parent-child 关系, 并更新 singul 和 g_score
  // ============================================================================
  void RrtPlanning::linkNode(PathNodeRRTPtr &parent, PathNodeRRTPtr &child){
    if(parent == nullptr || child == nullptr || parent == child) return;
    for(PathNodeRRTPtr ancestor = parent; ancestor != nullptr; ancestor = ancestor->parent){
      if(ancestor == child){
        RCLCPP_WARN(node_->get_logger(), "Rejected a cyclic RRT-tree link");
        return;
      }
    }
    PathNodeRRTPtr pre_parent = child->parent;
    if(pre_parent == parent){
      return;
    }else if(pre_parent != nullptr){
      pre_parent->children.erase(child->index);
    }
    child->parent = parent;
    parent->children.insert(std::make_pair(child->index, child));

    // 更新 singul (前进/后退方向)
    int singul;
    if(parent->parent != nullptr){
      singul = ((child->state - parent->state).head(2)).dot(
                 (parent->state - parent->parent->state).head(2)) >= 0 ? parent->singul : -parent->singul;
    }else{
      singul = ((child->state - parent->state).head(2)).dot(
                 Eigen::Vector2d(cos(parent->yaw), sin(parent->yaw))) >= 0 ? 1 : -1;
    }
    child->singul = singul;
    if(parent->singul != 0 && singul != parent->singul){
      child->singul = parent->singul;
    }
    expandGscore(child);
  }

  // ============================================================================
  // 递归更新子树 g_score
  // ============================================================================
  void RrtPlanning::expandGscore(PathNodeRRTPtr q){
    if(q == nullptr || q->parent == nullptr) return;
    if(q->g_score == q->parent->g_score + estimateHeuristic(q->parent, q)){
      return;
    }
    q->g_score = q->parent->g_score + estimateHeuristic(q->parent, q);
    for(auto it = q->children.begin(); it != q->children.end(); ++it){
      if(it->second != q && it->second->parent == q)
        expandGscore(it->second);
    }
  }

  // ============================================================================
  // 启发式函数 (两点之间距离)
  // ============================================================================
  double RrtPlanning::estimateHeuristic(PathNodeRRTPtr &x1, PathNodeRRTPtr &x2){
    return estimateHeuristic(x1->state, x1->yaw, x2->state, x2->yaw);
  }

  // ============================================================================
  // 合并两棵树: 将 ANTI_TREE 的节点并入 TREE
  // ============================================================================
  void RrtPlanning::mergeTree(PathNodeRRTPtr &s1, PathNodeRRTPtr &s2){
    PathNodeRRTPtr q1, q2, q_temp;
    if(s1->node_state == PathNodeRRT::IN_TREE){
      q1 = s1;
      q2 = s2;
    }else{
      q1 = s2;
      q2 = s1;
    }

    std::vector<PathNodeRRTPtr> s_list;
    q_temp = q2;
    while(q_temp != nullptr){
      s_list.push_back(q_temp);
      q_temp->node_state = PathNodeRRT::IN_TREE;  // 全部转为 TREE
      q_temp->children.clear();
      q_temp = q_temp->parent;
    }
    linkNode(q1, s_list[0]);
    for(unsigned int i = 1; i < s_list.size(); ++i){
      linkNode(s_list[i-1], s_list[i]);
    }
    this->end_node_ = s_list.back();
  }

  // ============================================================================
  // 启发式函数: 欧几里得距离 + yaw 差
  // ============================================================================
  double RrtPlanning::estimateHeuristic(const Eigen::VectorXd& s1, const double& yaw1,
                                         const Eigen::VectorXd& s2, const double& yaw2){
    double ret = 0.0;
    ret += sqrt((s1 - s2).squaredNorm() + (yaw1 - yaw2) * (yaw1 - yaw2));
    return ret;
  }

  // ============================================================================
  // 节点索引 (字符串哈希): 状态量 × 100 取整 → 拼接字符串
  // ============================================================================
  string RrtPlanning::calculateValue(PathNodeRRTPtr &q){
    return calculateValue(q->state, q->yaw);
  }

  string RrtPlanning::calculateValue(const Eigen::VectorXd &s, const double yaw){
    int k;
    string ret;
    for(int i = 0; i < traj_dim_; ++i){
      k = round(s(i) * 100.0);
      string s(std::to_string(k));
      ret += s;
    }
    ret += std::to_string(round(yaw * 100.0));
    return ret;
  }

  // ============================================================================
  // 初始化: 计算采样空间边界和椭球参数
  // ============================================================================
  void RrtPlanning::init(const std::vector<Eigen::VectorXd>& start_pt_list,
                          const std::vector<Eigen::VectorXd>& end_pt_list){
    this->reset();
    for(int i = 0; i < mobile_base_dof_; ++i){
      max_size_[i] = -1.0e6;
      min_size_[i] = 1.0e6;
    }
    c_max_ = 1.0e6;

    // 计算起点集群中心和半径
    sample_start_radius_ = 0.0;
    if(start_pt_list.size() == 1){
      sample_start_ = start_pt_list[0];
      sample_start_radius_ = 0;
    }else{
      for(unsigned int i = 0; i < start_pt_list.size(); ++i){
        for(unsigned int j = i + 1; j < start_pt_list.size(); ++j){
          if((start_pt_list[i] - start_pt_list[j]).norm() > sample_start_radius_){
            sample_start_ = (start_pt_list[i] + start_pt_list[j]) / 2.0;
            sample_start_radius_ = (start_pt_list[i] - start_pt_list[j]).norm();
          }
        }
      }
    }

    // 计算终点集群中心和半径
    sample_end_radius_ = 0.0;
    if(end_pt_list.size() == 1){
      sample_end_ = end_pt_list[0];
      sample_end_radius_ = 0;
    }else{
      for(unsigned int i = 0; i < end_pt_list.size(); ++i){
        for(unsigned int j = i + 1; j < end_pt_list.size(); ++j){
          if((end_pt_list[i] - end_pt_list[j]).norm() > sample_end_radius_){
            sample_end_ = (end_pt_list[i] + end_pt_list[j]) / 2.0;
            sample_end_radius_ = (end_pt_list[i] - end_pt_list[j]).norm();
          }
        }
      }
    }

    c_min_ = (sample_start_ - sample_end_).norm();

    // 设置采样空间边界 (扩展20m裕度)
    for(unsigned int i = 0; i < start_pt_list.size(); ++i)
      for(unsigned int j = 0; j < end_pt_list.size(); ++j)
        for(int k = 0; k < mobile_base_dof_; ++k){
          max_size_[k] = max(start_pt_list[i][k] + fabs(end_pt_list[j][k] - start_pt_list[i][k]) + 20.0, max_size_[k]);
          min_size_[k] = min(start_pt_list[i][k] - fabs(end_pt_list[j][k] - start_pt_list[i][k]) - 20.0, min_size_[k]);
        }
  }

  // ============================================================================
  // 角度差 (简化)
  // ============================================================================
  double RrtPlanning::calAngleErr(double angle1, double angle2){
    return fabs(angle1 - angle2);
  }

  // ============================================================================
  // 碰撞检查 (节点自身)
  // ============================================================================
  bool RrtPlanning::checkcollision(PathNodeRRTPtr& cur_state){
    Eigen::Vector3d xt;
    xt[0] = cur_state->state[0];
    xt[1] = cur_state->state[1];
    xt[2] = cur_state->yaw;
    return mm_config_->checkcollision(xt, cur_state->state.tail(manipulator_dof_), false);
  }

  // ============================================================================
  // 碰撞检查 (两个节点之间的路径)
  //
  // 沿 Dubins 曲线和机械臂线性插值密集采样, 逐点碰撞检查.
  // ============================================================================
  bool RrtPlanning::checkcollision(PathNodeRRTPtr& cur_state, const Eigen::VectorXd& next_state,
                                    const double next_yaw){
    if(cur_state->node_state == PathNodeRRT::NODE_STATE::COLLISION){
      return true;
    }

    int singul;
    if(cur_state->parent == nullptr){
      singul = ((next_state - cur_state->state).head(2)).dot(
                 Eigen::Vector2d(cos(cur_state->yaw), sin(cur_state->yaw))) >= 0 ? 1 : -1;
    }else{
      singul = ((next_state - cur_state->state).head(2)).dot(
                 (cur_state->state - cur_state->parent->state).head(2)) >= 0 ? cur_state->singul : -cur_state->singul;
    }
    if(cur_state->singul != 0 && singul != cur_state->singul){
      singul = cur_state->singul;
    }

    // Dubins 曲线采样
    ompl::base::ScopedState<> from(dubins_curve_), to(dubins_curve_), s(dubins_curve_);
    from[0] = cur_state->state[0]; from[1] = cur_state->state[1]; from[2] = cur_state->yaw;
    to[0]   = next_state[0]; to[1]   = next_state[1]; to[2]   = next_yaw;
    if(singul == -1){
      from[2] = cur_state->yaw + M_PI;
      if(from[2] > M_PI) from[2] -= 2 * M_PI;
      to[2] = next_yaw + M_PI;
      if(to[2] > M_PI) to[2] -= 2 * M_PI;
    }

    Eigen::Vector3d xt;
    std::vector<double> reals;
    double dis = dubins_curve_->distance(from(), to());

    // 自适应采样数 (基于基底和机械臂的最大变化量)
    int check_num_car = ceil(dis / 0.01);
    Eigen::VectorXd delta_theta = next_state.tail(manipulator_dof_) - cur_state->state.tail(manipulator_dof_);
    double max_delta_theta = delta_theta.lpNorm<Eigen::Infinity>();
    int check_num_theta = ceil(max_delta_theta / 0.01);
    int piece_num_temp = std::max(check_num_car, check_num_theta);
    piece_num_temp = std::max(piece_num_temp, 10);

    for(int i = 0; i < piece_num_temp; ++i){
      double temp_i = (double)i / (double)piece_num_temp;
      dubins_curve_->interpolate(from(), to(), temp_i, s());
      reals = s.reals();
      if(singul == -1){
        reals[2] = reals[2] + M_PI;
        if(reals[2] > M_PI) reals[2] -= 2 * M_PI;
      }
      xt = Eigen::Vector3d(reals[0], reals[1], reals[2]);
      if(mm_config_->checkcollision(xt, cur_state->state.tail(manipulator_dof_) + delta_theta * temp_i, false)){
        return true;
      }
    }
    return false;
  }

  bool RrtPlanning::checkcollision(PathNodeRRTPtr& cur_state, PathNodeRRTPtr& next_state){
    if(cur_state->node_state == PathNodeRRT::NODE_STATE::COLLISION
       || next_state->node_state == PathNodeRRT::NODE_STATE::COLLISION){
      return true;
    }
    return checkcollision(cur_state, next_state->state, next_state->yaw);
  }

  // ============================================================================
  // 轨迹平滑 (后端处理)
  // ============================================================================
  void RrtPlanning::smoothTraj(std::vector<Eigen::VectorXd> &traj, std::vector<double> &yaw_list,
                                std::vector<double> &t_list){
    PathNodeRRTPtr temp_end = end_node_;
    PathNodeRRTPtr last = nullptr, temp;
    for(unsigned int i = 0; i < traj.size(); ++i){
      temp = initNode(traj[i], yaw_list[i]);
      temp->parent = last;
      last = temp;
    }
    end_node_ = last;
    have_path_ = true;
    getTraj(traj, yaw_list, t_list);
    end_node_ = temp_end;
    return;
  }

  // ============================================================================
  // 路径平滑: 尝试跳过中间节点, 用 Dubins 曲线直接连接
  // ============================================================================
  bool RrtPlanning::smooth(PathNodeRRTPtr smooth_node){
    if(!have_path_) return false;

    PathNodeRRTPtr node = smooth_node;
    PathNodeRRTPtr temp, new_node;
    // 跳过中间节点 (无碰撞检查)
    while(node != nullptr){
      temp = node->parent;
      while(temp != nullptr){
        if(!checkcollision(temp, node)){
          linkNode(temp, node);
        }
        temp = temp->parent;
      }
      node = node->parent;
    }

    // 在现有节点间插值, 使轨迹更稠密
    ompl::base::ScopedState<> from(dubins_curve_), to(dubins_curve_), s(dubins_curve_);
    double t;
    int rel;
    Eigen::VectorXd temp_state(traj_dim_);
    double temp_yaw;
    node = smooth_node;
    while(node != nullptr){
      temp = node->parent;
      if(temp != nullptr){
        t = getTime(temp, node);
        rel = floor(t / time_resolution_);
        from[0] = node->parent->state[0]; from[1] = node->parent->state[1]; from[2] = node->parent->yaw;
        to[0] = node->state[0]; to[1] = node->state[1]; to[2] = node->yaw;

        for(int i = 1; i < rel; ++i){
          dubins_curve_->interpolate(from(), to(), (double)i / rel, s());
          temp_state.tail(manipulator_dof_) = temp->state.tail(manipulator_dof_)
                                            + (node->state.tail(manipulator_dof_)
                                               - temp->state.tail(manipulator_dof_)) * (double)i / (double)rel;
          temp_state[0] = s[0]; temp_state[1] = s[1];
          if(node->singul == 1)
            temp_yaw = s[2];
          else{
            temp_yaw = s[2] + M_PI;
            if(temp_yaw > M_PI) temp_yaw -= 2 * M_PI;
          }
          new_node = initNode(temp_state, temp_yaw);
          linkNode(node->parent, new_node);
          linkNode(new_node, node);
        }
      }
      node = temp;
    }
    return true;
  }

  // ============================================================================
  // 获取完整路径 (从 end_node_ 回溯到根)
  // ============================================================================
  bool RrtPlanning::getTraj(std::vector<Eigen::VectorXd> &traj, std::vector<double> &yaw_list,
                             std::vector<double> &t_list){
    traj.clear();
    yaw_list.clear();
    t_list.clear();
    if(!have_path_) return false;

    // 先做平滑
    PathNodeRRTPtr node = end_node_;
    while(node != nullptr){
      smooth(node);
      node = node->parent;
    }

    // 回溯
    node = end_node_;
    Eigen::VectorXd temp_state(traj_dim_);
    while(node != nullptr){
      traj.push_back(node->state);
      yaw_list.push_back(node->yaw);
      if(node->parent != nullptr)
        t_list.push_back(getTime(node->parent, node));
      node = node->parent;
    }
    reverse(traj.begin(), traj.end());
    reverse(yaw_list.begin(), yaw_list.end());
    reverse(t_list.begin(), t_list.end());
    return true;
  }

  // ============================================================================
  // 计算两个节点之间的运动时间
  //
  // 时间 = max(Dubins弧长/最大速度, 各关节角度差/最大关节速度)
  // ============================================================================
  double RrtPlanning::getTime(PathNodeRRTPtr &pre_node, PathNodeRRTPtr &cur_node){
    double t = -1.0;
    Eigen::VectorXd pre_state = pre_node->state;
    Eigen::VectorXd cur_state = cur_node->state;

    ompl::base::ScopedState<> from(dubins_curve_), to(dubins_curve_);
    from[0] = pre_node->state[0]; from[1] = pre_node->state[1]; from[2] = pre_node->yaw;
    to[0]   = cur_node->state[0]; to[1]   = cur_node->state[1]; to[2]   = cur_node->yaw;
    if(cur_node->singul == -1){
      from[2] = pre_node->yaw + M_PI;
      if(from[2] > M_PI) from[2] -= 2 * M_PI;
      to[2] = cur_node->yaw + M_PI;
      if(to[2] > M_PI) to[2] -= 2 * M_PI;
    }

    double len = dubins_curve_->distance(from(), to());
    t = max(len / max_vel_, t);

    for(int i = 0; i < manipulator_dof_; ++i){
      t = max(calAngleErr(pre_state(mobile_base_dof_ + i), cur_state(mobile_base_dof_ + i)) / max_joint_vel_, t);
    }
    return t;
  }

  double RrtPlanning::getCost(){
    if(!have_path_) return -1.0;
    return end_node_->g_score;
  }

  bool RrtPlanning::getLayer(int& start_layer, int& end_layer){
    if(!have_path_) return false;
    end_layer = end_node_->layer;
    PathNodeRRTPtr node = end_node_;
    while(node != nullptr){
      if(node->parent == nullptr){
        start_layer = node->layer;
        break;
      }
      node = node->parent;
    }
    return true;
  }

  int RrtPlanning::getPathLen(){
    if(!have_path_) return -1;
    PathNodeRRTPtr temp = end_node_;
    int len = 0;
    while(temp != nullptr){
      ++len;
      temp = temp->parent;
    }
    return len;
  }

  void RrtPlanning::reset(){
    this->max_index_ = 0;
    this->have_path_ = false;
    this->tree_count_ = 0;
    this->anti_tree_count_ = 0;
    this->end_node_ = nullptr;

    for(auto it = node_pool_.begin(); it != node_pool_.end(); ++it){
      delete it->second;
    }
    node_pool_.clear();
  }

  void RrtPlanning::setParam(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<MMConfig> &mm_config){
    node_ = node;
    mm_config_ = mm_config;
    manipulator_link_pts_ = mm_config_->getLinkPoint();
    max_vel_ = mm_config_->getBaseMaxVel();
    T_q_0_ = mm_config_->getTq0();

    getRrtParam(node_, "search.check_num", check_num_, -1);
    getRrtParam(node_, "search.goal_rate", goal_rate_, 0.0);
    getRrtParam(node_, "search.max_sample_time", max_sample_time_, 0.0);
    getRrtParam(node_, "search.max_loop_num", max_loop_num_, 100);
    getRrtParam(node_, "search.time_resolution", time_resolution_, 1.0);
    getRrtParam(node_, "optimization.self_safe_margin", self_safe_margin_, 0.1);
    getRrtParam(node_, "optimization.safe_margin_mani", safe_margin_mani_, 0.1);

    getRrtParam(node_, "mm.mobile_base_dof", mobile_base_dof_, -1);

    double dist_resolution;
    getRrtParam(node_, "search.dist_resolution", dist_resolution, 1.0);
    time_resolution_ = dist_resolution / max_vel_;

    std::vector<double> joint_pos_limit;
    getRrtParam(node_, "mm.manipulator_min_pos", joint_pos_limit, std::vector<double>{});
    min_joint_pos_.resize(joint_pos_limit.size());
    for(unsigned int i = 0; i < joint_pos_limit.size(); i++){
        min_joint_pos_(i) = joint_pos_limit[i];
    }
    joint_pos_limit.clear();
    getRrtParam(node_, "mm.manipulator_max_pos", joint_pos_limit, std::vector<double>{});
    max_joint_pos_.resize(joint_pos_limit.size());
    for(unsigned int i = 0; i < joint_pos_limit.size(); i++){
        max_joint_pos_(i) = joint_pos_limit[i];
    }
    getRrtParam(node_, "mm.manipulator_max_vel", max_joint_vel_, -1.0);
    getRrtParam(node_, "mm.manipulator_max_acc", max_joint_acc_, -1.0);
    getRrtParam(node_, "mm.manipulator_dof", manipulator_dof_, -1);
    getRrtParam(node_, "mm.manipulator_thickness", mani_thickness_, 0.1);

    dubins_curve_ = std::make_shared<ompl::base::DubinsStateSpace>(0.1);
    traj_dim_ = mobile_base_dof_ + manipulator_dof_;

    for(int i = 0; i < mobile_base_dof_; ++i){
      this->max_size_.push_back(0.0);
      this->min_size_.push_back(0.0);
    }
    for(int i = 0 ; i < manipulator_dof_; ++i){
      this->max_size_.push_back(max_joint_pos_[i]);
      this->min_size_.push_back(min_joint_pos_[i]);
    }
    this->max_size_.push_back(M_PI);   // yaw max
    this->min_size_.push_back(-M_PI);  // yaw min
  }
}  //namespace remani_planner
