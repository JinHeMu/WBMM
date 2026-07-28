/**
 * @file sample_mani_RRT.cpp
 * @brief 机械臂构型采样器 (SampleMani) — 沿基底路径规划机械臂构型
 *
 * ============================================================
 * 核心职责: 给定基底轨迹 (x, y, yaw 序列), 为每个时间点分配
 *           无碰撞的机械臂构型 (q1...qN)
 * ============================================================
 *
 * 分层规划架构:
 *   Level 1 (KinoAstar): 基底路径搜索 (x, y, yaw, singul)
 *   Level 2 (SampleMani): 在基底路径的每个时间步上采样机械臂构型
 *   Level 2.5 (RrtPlanning): 填充机械臂 RRT 无法直接连接的间隙
 *
 * 工作流:
 *   KinoAstar 输出基底路径 → SampleMani 在基底路径点上
 *   → 对机械臂空间做 bidirectional RRT* 搜索
 *   → 输出完整状态路径 [x, y, q1...qN]
 *
 * @see kino_astar.cpp (调用方)
 * @see rrt.cpp (底层 RRT 规划器)
 */

#include "path_searching/sample_mani_RRT.h"

namespace mani_sample{
  template<typename T>
  static void getManiParam(const rclcpp::Node::SharedPtr &node, const std::string &name,
                           T &value, const T &default_value) {
    if (!node->has_parameter(name)) {
      node->declare_parameter<T>(name, default_value);
    }
    node->get_parameter(name, value);
  }

  /**
   * @brief 主入口: 在基底路径上采样机械臂构型
   *
   * @param astar_succ           KinoA* 是否搜到路径
   * @param start_state          起始机械臂构型
   * @param end_state            终止机械臂构型
   * @param car_state_list       基底路径序列 (x, y, yaw)
   * @param car_state_list_check 基底路径稠密采样 (用于碰撞检查)
   * @param t_list               基底路径时间分配
   * @param singul_container     基底路径各段方向
   * @param simple_path_container [输出] 完整路径 (按 singul 分段)
   * @param singul_container_new  [输出] 每段方向
   * @param yaw_list_container    [输出] 每段 yaw 序列
   * @param t_list_container      [输出] 每段时间
   */
  bool SampleMani::sampleManiSearch(const bool astar_succ,
                      const Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state,
                      const std::vector<Eigen::Vector3d> &car_state_list,
                      const std::vector<Eigen::Vector3d> &car_state_list_check,
                      const std::vector<double> &t_list,
                      const std::vector<int> &singul_container, const int start_singul,
                      std::vector<std::vector<Eigen::VectorXd>> &simple_path_container,
                      std::vector<int> &singul_container_new,
                      std::vector<std::vector<double>> &yaw_list_container,
                      std::vector<Eigen::VectorXd> &t_list_container){
    simple_path_container.clear();
    singul_container_new.clear();
    yaw_list_container.clear();
    t_list_container.clear();

    std::vector<Eigen::VectorXd> simple_path;
    std::vector<double> yaw_list, t_list_temp;
    Eigen::VectorXd t_vector;
    std::vector<Eigen::VectorXd> mani_path;
    Eigen::VectorXd state_full(traj_dim_);

    // ---- 初始化 + 双向 RRT* 搜索机械臂构型 ----
    init(car_state_list, car_state_list_check, t_list);
    bool mani_status = search(start_state, end_state);

    if(mani_status && astar_succ){
      // ---- 成功: 提取路径并按 singul 分段输出 ----
      getTraj(mani_path);

      int singul_now = singul_container[0];
      state_full.head(mobile_base_dof_) = car_state_list[0].head(mobile_base_dof_);
      state_full.tail(manipulator_dof_) = mani_path[0];
      simple_path.push_back(state_full);
      yaw_list.push_back(car_state_list[0](2));

      for(int i = 1; i < max_index_; ++i){
        // singul 切换 → 新的一段
        if(singul_now != singul_container[i - 1]){
          singul_container_new.push_back(singul_now);
          singul_now = singul_container[i - 1];

          simple_path_container.push_back(simple_path);
          yaw_list_container.push_back(yaw_list);
          t_vector.resize(t_list_temp.size());
          for(unsigned int j = 0; j < t_list_temp.size(); ++j){
            t_vector[j] = t_list_temp[j];
          }
          t_list_container.push_back(t_vector);
          simple_path.clear();
          yaw_list.clear();
          t_list_temp.clear();

          simple_path.push_back(state_full);
          yaw_list.push_back(car_state_list[i - 1](2));
        }

        // 组合基底 + 机械臂 → 完整状态
        state_full.head(mobile_base_dof_) = car_state_list[i].head(mobile_base_dof_);
        state_full.tail(manipulator_dof_) = mani_path[i];
        simple_path.push_back(state_full);
        yaw_list.push_back(car_state_list[i](2));
        t_list_temp.push_back(t_list[i - 1]);
      }

      singul_container_new.push_back(singul_now);
      simple_path_container.push_back(simple_path);
      yaw_list_container.push_back(yaw_list);
      t_vector.resize(t_list_temp.size());
      for(unsigned int j = 0; j < t_list_temp.size(); ++j){
        t_vector[j] = t_list_temp[j];
      }
      t_list_container.push_back(t_vector);

      return true;
    }

    // ---- A* 失败: 尝试 RRT 填充间隙 ----
    if(!astar_succ){
      if(end_node_ != nullptr)
        end_node_->parent = nullptr;
    }

    // 构建 RRT 起点/终点列表
    std::vector<Eigen::VectorXd> start_list, end_list;
    std::vector<double> start_yaw_list, end_yaw_list, start_g_score_list, end_g_score_list;
    std::vector<int> start_layer_list, end_layer_list, start_singul_list, end_singul_list;

    // 从 TREE 中找起点 (tree_min_index_ 附近的节点)
    int index_front, index_last;
    if(tree_max_index_ <= tree_min_index_){
      index_front = max(tree_max_index_ - 1, 0);
      index_last = tree_max_index_;
    }else{
      index_front = max(tree_min_index_ - 1, 0);
      index_last = tree_max_index_;
    }

    auto it = node_pool_.lower_bound(string(1, index_front));
    for(; it != node_pool_.end() && it->second->index <= index_last; ++it){
      if(it->second->node_state == ManiPathNode::NODE_STATE::IN_TREE){
        state_full.head(mobile_base_dof_) = car_state_list[it->second->index].head(mobile_base_dof_);
        state_full.tail(manipulator_dof_) = it->second->state;
        start_list.push_back(state_full);
        start_yaw_list.push_back(car_state_list[it->second->index](2));
        start_g_score_list.push_back(it->second->g_score);
        start_layer_list.push_back(it->second->index);
        if(it->second->index == 0)
          start_singul_list.push_back(start_singul);
        else
          start_singul_list.push_back(singul_container[it->second->index - 1]);
      }
    }

    // 从 ANTI_TREE 中找终点
    if(tree_max_index_ <= tree_min_index_){
      index_front = tree_min_index_;
      index_last = min(tree_min_index_ + 1, max_index_ - 1);
    }else{
      index_front = tree_min_index_;
      index_last = min(tree_max_index_ + 1, max_index_ - 1);
    }
    it = node_pool_.lower_bound(string(1, index_front));
    for(; it != node_pool_.end() && it->second->index <= index_last; ++it){
      if(it->second->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE){
        state_full.head(mobile_base_dof_) = car_state_list[it->second->index].head(mobile_base_dof_);
        state_full.tail(manipulator_dof_) = it->second->state;
        end_list.push_back(state_full);
        end_yaw_list.push_back(car_state_list[it->second->index](2));
        end_g_score_list.push_back(it->second->g_score);
        end_layer_list.push_back(it->second->index);
        if(it->second->index == max_index_ - 1)
          end_singul_list.push_back(0);
        else
          end_singul_list.push_back(-singul_container[it->second->index]);
      }
    }

    // 调用底层 RRT 填充间隙
    std::vector<Eigen::VectorXd> path_fill;
    std::vector<double> yaw_list_fill, t_list_fill;
    bool status = rrt_plan_->RRTSearchAndGetSimplePath(
        start_list, start_yaw_list, end_list, end_yaw_list,
        start_g_score_list, start_layer_list, end_g_score_list, end_layer_list,
        start_singul_list, end_singul_list,
        path_fill, yaw_list_fill, t_list_fill);

    if(!status){
      // 完全失败 → 重置随机数种子并返回
      goal_gen_.seed(std::random_device{}());
      state_gen_.seed(std::random_device{}());
      node_gen_.seed(std::random_device{}());
      return false;
    }
    if(path_fill.empty() || yaw_list_fill.size() != path_fill.size()
       || t_list_fill.size() + 1 != path_fill.size()){
      RCLCPP_ERROR(node_->get_logger(),
                   "RRT returned inconsistent data: path=%zu, yaw=%zu, durations=%zu",
                   path_fill.size(), yaw_list_fill.size(), t_list_fill.size());
      return false;
    }

    // ---- 合并: 原 TREE 路径 + RRT 间隙填充 + 原 ANTI_TREE 路径 ----
    Eigen::Vector3d short_path_start, short_path_end;
    Eigen::VectorXd short_mani_start(manipulator_dof_), short_mani_end(manipulator_dof_);
    int path_fill_num = path_fill.size();

    short_path_start.head(2) = path_fill[0].head(2);
    short_path_start(2) = yaw_list_fill[0];
    short_mani_start = path_fill[0].tail(manipulator_dof_);

    short_path_end.head(2) = path_fill[path_fill_num-1].head(2);
    short_path_end(2) = yaw_list_fill[path_fill_num-1];
    short_mani_end = path_fill[path_fill_num-1].tail(manipulator_dof_);

    int part_idx_begin = -1, part_idx_end = -1;
    rrt_plan_->getLayer(part_idx_begin, part_idx_end);

    // 在 TREE 中找到 RRT 起点的对应节点
    ManiPathNodePtr q_t = nullptr, q_an_t = nullptr, q_temp;
    it = node_pool_.find(calculateValue(part_idx_begin, short_mani_start));
    if(it != node_pool_.end()){
      q_t = it->second;
    }else{
      RCLCPP_ERROR(node_->get_logger(), "find part_idx_begin, short_mani_start fail!");
      goal_gen_.seed(std::random_device{}());
      state_gen_.seed(std::random_device{}());
      node_gen_.seed(std::random_device{}());
      return false;
    }

    // 在 ANTI_TREE 中找到 RRT 终点的对应节点
    it = node_pool_.find(calculateValue(part_idx_end, short_mani_end));
    if(it != node_pool_.end()){
      q_an_t = it->second;
    }else{
      RCLCPP_ERROR(node_->get_logger(), "find part_idx_end, short_mani_end fail!");
      goal_gen_.seed(std::random_device{}());
      state_gen_.seed(std::random_device{}());
      node_gen_.seed(std::random_device{}());
      return false;
    }

    // 组装完整路径: TREE段 → RRT填充段 → ANTI_TREE段
    std::vector<Eigen::VectorXd> state_vector_temp;
    std::vector<double> t_vector_temp, yaw_vector_temp;
    std::vector<int> singul_vector_temp;

    // [1] TREE 段 (从 q_t 到根)
    q_temp = q_t;
    while(q_temp != nullptr){
      state_full.head(mobile_base_dof_) = car_state_list[q_temp->index].head(mobile_base_dof_);
      state_full.tail(manipulator_dof_) = q_temp->state;
      state_vector_temp.push_back(state_full);
      yaw_vector_temp.push_back(car_state_list[q_temp->index](2));
      if(q_temp->index > 0){
        t_vector_temp.push_back(t_list[q_temp->index - 1]);
        singul_vector_temp.push_back(1);
      }
      q_temp = q_temp->parent;
    }
    reverse(state_vector_temp.begin(), state_vector_temp.end());
    reverse(t_vector_temp.begin(), t_vector_temp.end());
    reverse(singul_vector_temp.begin(), singul_vector_temp.end());
    reverse(yaw_vector_temp.begin(), yaw_vector_temp.end());

    // [2] RRT 填充段
    t_vector_temp.push_back(t_list_fill[0]);
    int singul;
    if(singul_vector_temp.size() == 0)
      singul = (path_fill[1] - path_fill[0]).head(2).dot(
                 Eigen::Vector2d(cos(yaw_list_fill[0]), sin(yaw_list_fill[0]))) >= 0 ? 1 : -1;
    else
      singul = singul_vector_temp.back();
    singul_vector_temp.push_back(1);

    for(int i = 1; i < path_fill_num - 1; ++i){
      singul = (path_fill[i + 1] - path_fill[i]).head(2).dot(
                 (path_fill[i] - path_fill[i - 1]).head(2)) >= 0 ? singul : -singul;
      singul_vector_temp.push_back(1);
      state_vector_temp.push_back(path_fill[i]);
      yaw_vector_temp.push_back(yaw_list_fill[i]);
      t_vector_temp.push_back(t_list_fill[i]);
    }

    // [3] ANTI_TREE 段
    q_temp = q_an_t;
    while(q_temp != nullptr){
      state_full.head(mobile_base_dof_) = car_state_list[q_temp->index].head(mobile_base_dof_);
      state_full.tail(manipulator_dof_) = q_temp->state;
      state_vector_temp.push_back(state_full);
      yaw_vector_temp.push_back(car_state_list[q_temp->index](2));
      if(q_temp->index < max_index_ - 1){
        t_vector_temp.push_back(t_list[q_temp->index]);
        singul_vector_temp.push_back(1);
      }
      q_temp = q_temp->parent;
    }

    // ---- 按 singul 分段输出 ----
    simple_path.clear();
    yaw_list.clear();
    t_list_temp.clear();
    int singul_now = singul_vector_temp[0];
    simple_path.push_back(state_vector_temp[0]);
    yaw_list.push_back(yaw_vector_temp[0]);

    for(unsigned int i = 1; i < state_vector_temp.size(); ++i){
      if(singul_now != singul_vector_temp[i - 1]){
        singul_container_new.push_back(singul_now);
        singul_now = singul_vector_temp[i - 1];

        simple_path_container.push_back(simple_path);
        yaw_list_container.push_back(yaw_list);
        t_vector.resize(t_list_temp.size());
        for(unsigned int j = 0; j < t_list_temp.size(); ++j){
          t_vector[j] = t_list_temp[j];
        }
        t_list_container.push_back(t_vector);
        simple_path.clear();
        yaw_list.clear();
        t_list_temp.clear();

        simple_path.push_back(state_vector_temp[i - 1]);
        yaw_list.push_back(yaw_vector_temp[i - 1]);
      }

      simple_path.push_back(state_vector_temp[i]);
      yaw_list.push_back(yaw_vector_temp[i]);
      t_list_temp.push_back(t_vector_temp[i - 1]);
    }

    singul_container_new.push_back(singul_now);
    simple_path_container.push_back(simple_path);
    yaw_list_container.push_back(yaw_list);
    t_vector.resize(t_list_temp.size());
    for(unsigned int j = 0; j < t_list_temp.size(); ++j){
      t_vector[j] = t_list_temp[j];
    }
    t_list_container.push_back(t_vector);

    return true;
  }

  /**
   * @brief 机械臂构型 Bidirectional RRT* 搜索
   *
   * 状态空间: 机械臂关节角 (q1, ..., qN)
   * 索引:     基底路径的时间步索引 (0 ~ max_index_-1)
   *
   * 特殊: RRT 不是在连续空间生长, 而是在离散的"层"上生长.
   *   每层对应基底路径的一个时间步, 扩展只能在相邻层之间进行.
   *   这是因为机械臂的运动必须与基底的运动同步.
   */
  bool SampleMani::search(const Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state){
    std::uniform_real_distribution<double> goal_dis(0.0, 1.0);
    ManiPathNodePtr start_node, end_node;

    // 清理节点池
    if(!node_pool_.empty()){
      delete node_pool_.begin()->second;
      node_pool_.erase(node_pool_.begin());
    }

    // 起点节点 (index=0)
    start_node = new ManiPathNode(manipulator_dof_);
    start_node->index = 0;
    start_node->state = start_state;
    start_node->node_state = ManiPathNode::NODE_STATE::IN_TREE;
    start_node->g_score = 0.0;

    // 将已有 index=1 的节点连接到起点
    for(auto it = node_pool_.begin(); it != node_pool_.end() && it->second->index == 1; ++it){
      it->second->parent = start_node;
      start_node->children.insert(std::make_pair(calculateValue(it->second), it->second));
    }
    node_pool_.insert(std::make_pair(calculateValue(start_node), start_node));

    // 整理树结构 (连接已存在的节点)
    organizeTree(start_node);

    tree_min_index_ = max_index_ - 1;

    // 终点节点
    end_node = initNode(max_index_ - 1, end_state);
    if(end_node->node_state == ManiPathNode::NODE_STATE::IN_TREE){
      // 终点已在树中 → 直接成功
      this->end_node_ = end_node;
      have_path_ = true;
      return true;
    }
    end_node->node_state = ManiPathNode::NODE_STATE::IN_ANTI_TREE;
    end_node->g_score = 0.0;
    ++anti_tree_count_;

    // 起点索引=0, 终点索引=max_index_-1, 中间只有2层
    if(max_index_ == 2){
      if(!checkcollision(start_node, end_node)){
        have_path_ = true;
        linkNode(start_node, end_node);
        this->end_node_ = end_node;
        return true;
      }
      return false;
    }

    // ---- 双向 RRT* 主循环 ----
    int loop = 0;
    ManiPathNodePtr q_rand, q_new, q_near;
    ManiPathNodePtr q_new_1, q_near_1;
    ManiPathNodePtr q_new_2;
    ManiPathNodePtr path_node_1 = nullptr, path_node_2 = nullptr;
    double c_min = 1.0e6;
    bool dir = false;
    const rclcpp::Time t_start = node_->now();

    while(true){
      if((node_->now() - t_start).seconds() > max_mani_search_time_){
        break;  // 超时
      }

      // 交替扩展 (节点少的树优先)
      if(tree_count_ > anti_tree_count_)
        dir = false;
      else
        dir = true;
      ++loop;

      // 随机采样
      if(goal_dis(goal_gen_) < goal_rate_){
        q_rand = dir ? end_node : start_node;  // 直接采目标点 (概率 goal_rate_)
      }else{
        q_rand = getSampleNode();  // 随机采
      }
      if(q_rand == nullptr) continue;

      // 找最近邻 (只能在相邻层找)
      q_near = getNearestNode(q_rand, dir);
      if(q_near == nullptr) continue;

      // 向采样点扩展
      q_new = extendNode(q_near, q_rand, dir);
      if(q_new == nullptr) continue;

      // 相遇检测
      if((dir && q_new->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE)
         || (!dir && q_new->node_state == ManiPathNode::NODE_STATE::IN_TREE)){
        if(q_new->g_score + q_near->g_score + estimateHeuristic(q_new, q_near) < c_min){
          have_path_ = true;
          c_min = q_new->g_score + q_near->g_score + estimateHeuristic(q_new, q_near);
          path_node_1 = q_new;
          path_node_2 = q_near;
        }
        continue;
      }

      // 连接到树
      linkNode(q_near, q_new);
      q_new->node_state = q_near->node_state;
      adjustTree(q_new, dir);
      if(q_new->node_state == ManiPathNode::NODE_STATE::IN_TREE)
        ++tree_count_;
      else
        ++anti_tree_count_;

      // 更新树索引范围
      if(q_new->node_state == ManiPathNode::NODE_STATE::IN_TREE
         && q_new->index > tree_max_index_)
        tree_max_index_ = q_new->index;
      else if(q_new->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE
              && q_new->index < tree_min_index_)
        tree_min_index_ = q_new->index;

      // 从另一棵树扩展, 尝试连接
      q_near_1 = getNearestNode(q_new, !dir);
      if(q_near_1 == nullptr) continue;
      q_new_1 = extendNode(q_near_1, q_new, !dir);
      if(q_new_1 == nullptr) continue;

      if((!dir && q_new_1->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE)
         || (dir && q_new_1->node_state == ManiPathNode::NODE_STATE::IN_TREE)){
        if(q_new_1->g_score + q_near_1->g_score + estimateHeuristic(q_new_1, q_near_1) < c_min){
          have_path_ = true;
          c_min = q_new_1->g_score + q_near_1->g_score + estimateHeuristic(q_new_1, q_near_1);
          path_node_1 = q_new_1;
          path_node_2 = q_near_1;
        }
        continue;
      }

      linkNode(q_near_1, q_new_1);
      q_new_1->node_state = q_near_1->node_state;

      if(q_new_1->node_state == ManiPathNode::NODE_STATE::IN_TREE
         && q_new_1->index > tree_max_index_)
        tree_max_index_ = q_new_1->index;
      else if(q_new_1->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE
              && q_new_1->index < tree_min_index_)
        tree_min_index_ = q_new_1->index;

      if(q_new_1->node_state == ManiPathNode::NODE_STATE::IN_TREE)
        ++tree_count_;
      else
        ++anti_tree_count_;
      adjustTree(q_new_1, !dir);

      // 桥接间隙: 逐步连接两棵树
      while(q_new_1->index != q_new->index){
        q_new_2 = extendNode(q_new_1, q_new, !dir);
        if(q_new_2 != nullptr){
          if((!dir && q_new_2->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE)
             || (dir && q_new_2->node_state == ManiPathNode::NODE_STATE::IN_TREE)){
            if(q_new_2->g_score + q_new_1->g_score + estimateHeuristic(q_new_2, q_new_1) < c_min){
              have_path_ = true;
              c_min = q_new_2->g_score + q_new_1->g_score + estimateHeuristic(q_new_2, q_new_1);
              path_node_1 = q_new_2;
              path_node_2 = q_new_1;
            }
            break;
          }
          linkNode(q_new_1, q_new_2);
          q_new_2->node_state = q_new_1->node_state;
          if(q_new_2->node_state == ManiPathNode::NODE_STATE::IN_TREE
             && q_new_2->index > tree_max_index_)
            tree_max_index_ = q_new_2->index;
          else if(q_new_2->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE
                  && q_new_2->index < tree_min_index_)
            tree_min_index_ = q_new_2->index;

          if(q_new_1->node_state == ManiPathNode::NODE_STATE::IN_TREE)
            ++tree_count_;
          else
            ++anti_tree_count_;

          q_new_2->node_state = q_new_1->node_state;
          q_new_1 = q_new_2;
        }else{
          break;
        }
      }
    }

    if(have_path_){
      mergeTrees(path_node_1, path_node_2);
    }
    return have_path_;
  }

  /**
   * @brief 初始化/获取节点 (检查碰撞)
   */
  ManiPathNodePtr SampleMani::initNode(int idx, const Eigen::VectorXd &s){
    string key = calculateValue(idx, s);
    auto it = node_pool_.find(key);
    if(it != node_pool_.end()){
      return it->second;
    }
    ManiPathNodePtr node = new ManiPathNode;
    node->index = idx;
    node->node_state = ManiPathNode::NODE_STATE::EXPAND;
    node->state = s;
    if(mm_config_->checkManicollision(car_state_list_[node->index], node->state, false)){
      node->node_state = ManiPathNode::NODE_STATE::COLLISION;
    }
    node_pool_.insert(std::make_pair(key, node));
    return node;
  }

  /**
   * @brief 随机采样机械臂节点
   *
   * 随机选择基底路径索引, 随机生成机械臂关节角.
   * 如果碰撞则重新采样, 最多尝试20次.
   */
  ManiPathNodePtr SampleMani::getSampleNode(){
    std::uniform_int_distribution<int> node_dis(1, max_index_ - 2);
    std::vector<std::uniform_real_distribution<double>> state_dis_vec;
    state_dis_vec.reserve(manipulator_dof_);
    for(int i = 0; i < manipulator_dof_; ++i){
      std::uniform_real_distribution<double> state_dis(min_joint_pos_[i], max_joint_pos_[i]);
      state_dis_vec.push_back(state_dis);
    }

    Eigen::VectorXd sample_state(manipulator_dof_);
    ManiPathNodePtr sample_node = nullptr;
    int sample_idx;
    int count = 0;
    while(true){
      if(count > 20) return nullptr;
      ++count;

      sample_idx = node_dis(node_gen_);
      for(int i = 0; i < manipulator_dof_; ++i){
        sample_state(i) = state_dis_vec[i](state_gen_);
      }
      sample_node = initNode(sample_idx, sample_state);
      if(sample_node->node_state == ManiPathNode::NODE_STATE::COLLISION)
        continue;
      break;
    }
    return sample_node;
  }

  /**
   * @brief 最近邻搜索 (只在指定方向的相邻层中搜索)
   *
   * @param x   目标节点
   * @param dir true=向前扩展 (index+1), false=向后扩展 (index-1)
   */
  ManiPathNodePtr SampleMani::getNearestNode(ManiPathNodePtr &x, bool dir){
    int pre_idx;
    if(dir){
      pre_idx = x->index - 1;
      if(pre_idx > tree_max_index_) pre_idx = tree_max_index_;
    }else{
      pre_idx = x->index + 1;
      if(pre_idx < tree_min_index_) pre_idx = tree_min_index_;
    }

    Eigen::VectorXd cur_state = x->state;
    auto it = node_pool_.lower_bound(string(1, pre_idx));
    double min_dis = 1.0e6;
    ManiPathNodePtr temp, q_near = nullptr;
    for(; it != node_pool_.end() && it->second->index == pre_idx; ++it){
      if((dir && it->second->node_state != ManiPathNode::NODE_STATE::IN_TREE)
         || (!dir && it->second->node_state != ManiPathNode::NODE_STATE::IN_ANTI_TREE))
        continue;
      temp = it->second;
      if(estimateHeuristic(x, temp) < min_dis){
        min_dis = estimateHeuristic(x, temp);
        q_near = temp;
      }
    }
    return q_near;
  }

  /**
   * @brief 节点扩展: 从 q_near 向 q_rand 方向扩展到下一层
   *
   * 机械臂关节线性插值, 限制速度 ≤ max_joint_vel × 0.3 (安全裕度)
   */
  ManiPathNodePtr SampleMani::extendNode(ManiPathNodePtr &q_near, ManiPathNodePtr &q_rand, bool flag){
    ManiPathNodePtr q_new = nullptr;
    Eigen::VectorXd dir = q_rand->state - q_near->state;
    double t_total = 0;

    for(int i = min<int>(q_near->index, q_rand->index);
        i < max<int>(q_near->index, q_rand->index); ++i){
      t_total += t_list_[i];
    }

    // 限速
    Eigen::VectorXd vel_dir = dir / (double)t_total;
    for(int i = 0; i < manipulator_dof_; ++i){
      if(vel_dir(i) > max_joint_vel_ * 0.3)
        vel_dir(i) = max_joint_vel_ * 0.3;
      else if(vel_dir(i) < -max_joint_vel_ * 0.3)
        vel_dir(i) = -max_joint_vel_ * 0.3;
    }

    // 只扩展一步 (到下一层)
    Eigen::VectorXd new_state;
    if(flag)
      new_state = q_near->state + vel_dir * (double)t_list_[q_near->index];
    else
      new_state = q_near->state + vel_dir * (double)t_list_[q_near->index - 1];

    int new_idx;
    if(flag)
      new_idx = q_near->index + 1;
    else
      new_idx = q_near->index - 1;

    q_new = initNode(new_idx, new_state);
    if(q_new->node_state == ManiPathNode::NODE_STATE::COLLISION
       || checkcollision(q_near, q_new)){
      return nullptr;
    }
    return q_new;
  }

  /**
   * @brief RRT* 调整: 对新节点 q_new 在下一层中寻找更好的连接
   */
  void SampleMani::adjustTree(ManiPathNodePtr &q_new, bool dir){
    if(q_new == nullptr || q_new->index < 1) return;

    int next_idx;
    if(dir){
      next_idx = q_new->index + 1;
      if(next_idx > tree_max_index_) return;
    }else{
      next_idx = q_new->index - 1;
      if(next_idx < tree_min_index_) return;
    }

    auto it = node_pool_.lower_bound(string(1, next_idx));
    ManiPathNodePtr q_temp;
    for(; it != node_pool_.end() && it->second->index == next_idx; ++it){
      q_temp = it->second;
      if((dir && q_temp->node_state != ManiPathNode::NODE_STATE::IN_TREE)
         || (!dir && q_temp->node_state != ManiPathNode::NODE_STATE::IN_ANTI_TREE))
        continue;

      if(q_temp->g_score > q_new->g_score + estimateHeuristic(q_new, q_temp)
         && !feasibleCheck(q_new, q_temp)){
        if(checkcollision(q_new, q_temp)) continue;
        linkNode(q_new, q_temp);
      }
    }
  }

  /**
   * @brief 合并两棵树
   */
  void SampleMani::mergeTrees(const ManiPathNodePtr &q1, const ManiPathNodePtr &q2){
    ManiPathNodePtr q_start, q_end;
    if(q1->node_state == ManiPathNode::NODE_STATE::IN_TREE){
      q_start = q1;
      q_end = q2;
    }else if(q1->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE){
      q_start = q2;
      q_end = q1;
    }else{
      return;
    }

    std::vector<ManiPathNodePtr> q_list;
    ManiPathNodePtr q_temp = q_end;
    while(q_temp != nullptr){
      q_list.push_back(q_temp);
      q_temp->children.clear();
      q_temp = q_temp->parent;
    }

    for(int i = q_list.size() - 1; i > 0; --i){
      q_list[i]->parent = q_list[i-1];
      q_list[i]->node_state = ManiPathNode::NODE_STATE::IN_TREE;
      q_list[i-1]->children.insert(std::make_pair(calculateValue(q_list[i]), q_list[i]));
    }
    linkNode(q_start, q_list[0]);
    q_list[0]->node_state = ManiPathNode::NODE_STATE::IN_TREE;
    this->end_node_ = q_list.back();
  }

  /**
   * @brief 整理树: 对已有节点重建父子关系 (用于初始化)
   */
  void SampleMani::organizeTree(){
    if(node_pool_.size() < 2) return;

    std::map<string, ManiPathNodePtr>::iterator it = node_pool_.begin(), temp_it, pre_it;
    ++it;
    ManiPathNodePtr curr_node, pre_node;
    while(it != node_pool_.end()){
      if(it->second->index > max_index_ - 1) break;
      temp_it = it;
      ++it;

      curr_node = temp_it->second;
      curr_node->parent = nullptr;
      curr_node->children.clear();
      curr_node->g_score = 1.0e6;

      if(mm_config_->checkManicollision(car_state_list_[curr_node->index], curr_node->state, false)){
        curr_node->node_state = ManiPathNode::NODE_STATE::COLLISION;
        continue;
      }

      // 找上一层的节点作为父节点
      int pre_idx = curr_node->index - 1;
      pre_it = node_pool_.lower_bound(string(1, pre_idx));

      for(; pre_it != node_pool_.end() && pre_it->second->index == pre_idx; ++pre_it){
        pre_node = pre_it->second;
        if(pre_node->g_score + estimateHeuristic(pre_node, curr_node) < curr_node->g_score){
          if(feasibleCheck(pre_node, curr_node) || checkcollision(pre_node, curr_node))
            continue;
          curr_node->parent = pre_node;
          curr_node->g_score = pre_node->g_score + estimateHeuristic(pre_node, curr_node);
        }
      }

      if(curr_node->parent == nullptr){
        delete curr_node;
        node_pool_.erase(temp_it);
      }else{
        curr_node->parent->children.insert(std::make_pair(calculateValue(curr_node), curr_node));
        ++tree_count_;
        if(curr_node->index > tree_max_index_)
          tree_max_index_ = curr_node->index;
      }
    }
  }

  /**
   * @brief 整理子树 (从 q 开始递归)
   */
  void SampleMani::organizeTree(ManiPathNodePtr &q){
    if(q->index > max_index_ - 1) return;
    if(q->index > tree_max_index_) tree_max_index_ = q->index;
    ++tree_count_;
    if(q->index > max_index_ - 2) return;

    std::map<string, ManiPathNodePtr>::iterator it = q->children.begin(), temp_it;
    while(it != q->children.end()){
      temp_it = it;
      ++it;
      if(feasibleCheck(q, temp_it->second)
         || mm_config_->checkManicollision(car_state_list_[temp_it->second->index],
                                            temp_it->second->state, false)
         || checkcollision(q, temp_it->second)){
        clearSubTree(temp_it->second);
        q->children.erase(temp_it);
        continue;
      }
      temp_it->second->g_score = q->g_score + estimateHeuristic(q, temp_it->second);
      organizeTree(temp_it->second);
    }
  }

  /**
   * @brief 递归清理子树 (内存释放)
   */
  void SampleMani::clearSubTree(ManiPathNodePtr &q){
    std::map<string, ManiPathNodePtr>::iterator it;
    for(it = q->children.begin(); it != q->children.end(); ++it){
      clearSubTree(it->second);
    }
    node_pool_.erase(calculateValue(q));
    delete q;
  }

  /**
   * @brief "一次射击"优化: 尝试跨越多层直接连接
   *
   * 如果从祖先节点直接连接到当前节点无碰撞, 则跳过中间节点.
   * 类似于 Reeds-Shepp shot, 但用于机械臂关节空间.
   */
  void SampleMani::oneShot(ManiPathNodePtr &q){
    if(q->index <= 1 || q->index > tree_max_index_) return;

    ManiPathNodePtr temp = q->parent->parent;
    ManiPathNodePtr q_check1, q_check2;
    double t_total = t_list_[q->parent->index];
    std::vector<double> t_total_list;
    t_total_list.push_back(0.0);
    t_total_list.push_back(t_total);
    Eigen::VectorXd vel(manipulator_dof_);
    bool is_occ = false;

    for(; temp != nullptr; temp = temp->parent){
      is_occ = false;
      t_total += t_list_[temp->index];
      t_total_list.push_back(t_total);

      if(feasibleCheck(temp, q)) continue;

      vel = (temp->state - q->state) / t_total;
      for(int i = 0; i < q->index - temp->index; ++i){
        q_check1 = initNode(q->index - i, q->state + t_total_list.at(i) * vel);
        q_check2 = initNode(q->index - i - 1, q->state + t_total_list.at(i + 1) * vel);
        if(checkcollision(q_check2, q_check1)){
          is_occ = true;
          break;
        }
      }
      if(is_occ) continue;

      // 无碰撞 → 直接连接
      std::vector<ManiPathNodePtr> node_list;
      node_list.push_back(temp);
      for(int i = q->index - temp->index - 1; i >= 0; --i){
        node_list.push_back(initNode(q->index - i - 1, q->state + t_total_list.at(i) * vel));
      }

      for(size_t i = 0; i + 1 < node_list.size(); ++i){
        q_check1 = node_list[i];
        q_check2 = node_list[i+1];
        if(q_check2->node_state == ManiPathNode::NODE_STATE::IN_TREE){
          if(q_check2->g_score < q_check1->g_score + estimateHeuristic(q_check1, q_check2))
            continue;
        }
        linkNode(q_check1, q_check2);
        if(q_check2->node_state != ManiPathNode::NODE_STATE::IN_TREE)
          ++tree_count_;
        q_check2->node_state = ManiPathNode::NODE_STATE::IN_TREE;
      }
    }
  }

  void SampleMani::trajShot(ManiPathNodePtr &q){
    if(q->parent) trajShot(q->parent);
    oneShot(q);
  }

  /**
   * @brief 节点连接 (设置 parent-child 关系, 更新 g_score)
   */
  void SampleMani::linkNode(ManiPathNodePtr &parent, ManiPathNodePtr &child){
    if(parent == nullptr || child == nullptr || parent == child) return;
    for(ManiPathNodePtr ancestor = parent; ancestor != nullptr; ancestor = ancestor->parent){
      if(ancestor == child){
        RCLCPP_WARN(node_->get_logger(), "Rejected a cyclic manipulator-tree link");
        return;
      }
    }
    ManiPathNodePtr pre_parent = child->parent;
    if(pre_parent == parent) return;
    else if(pre_parent != nullptr)
      pre_parent->children.erase(calculateValue(child));

    child->parent = parent;
    parent->children.insert(std::make_pair(calculateValue(child), child));
    expandGscore(child);
  }

  void SampleMani::expandGscore(ManiPathNodePtr &p){
    if(p == nullptr || p->parent == nullptr || p->index > max_index_ - 1) return;
    if(p->g_score == p->parent->g_score + estimateHeuristic(p->parent, p)) return;
    p->g_score = p->parent->g_score + estimateHeuristic(p->parent, p);
    for(auto it = p->children.begin(); it != p->children.end(); ++it){
      if(it->second != p && it->second->parent == p)
        expandGscore(it->second);
    }
  }

  /**
   * @brief 可行性检查: 速度是否在关节限速内
   */
  bool SampleMani::feasibleCheck(ManiPathNodePtr &x1, ManiPathNodePtr &x2){
    double t_total = 0.0;
    for(int i = min<int>(x1->index, x2->index); i < max<int>(x1->index, x2->index); ++i){
      t_total += t_list_[i];
    }
    Eigen::VectorXd dif = x2->state - x1->state;
    if(fabs(dif(0)) > M_PI) dif(0) = 2.0 * M_PI - fabs(dif(0));
    Eigen::VectorXd vel = dif / t_total;
    if(vel.lpNorm<Eigen::Infinity>() > max_joint_vel_) return true;
    return false;
  }

  /**
   * @brief 节点索引字符串 (哈希键)
   */
  string SampleMani::calculateValue(int &idx, const Eigen::VectorXd &state){
    string ret(1, idx);
    int k;
    for(int i = 0; i < manipulator_dof_; ++i){
      k = round(state(i) * 100.0);
      string s(std::to_string(k));
      ret += s;
    }
    return ret;
  }

  string SampleMani::calculateValue(ManiPathNodePtr &x){
    return calculateValue(x->index, x->state);
  }

  /**
   * @brief 启发式: L1 误差 / 时间
   */
  double SampleMani::estimateHeuristic(ManiPathNodePtr &x1, ManiPathNodePtr &x2){
    double t_total = 0.0;
    for(int i = min<int>(x1->index, x2->index); i < max<int>(x1->index, x2->index); ++i){
      t_total += t_list_[i];
    }
    Eigen::VectorXd err = x1->state - x2->state;
    return t_total > 1.0e-9 ? err.lpNorm<1>() / t_total : err.lpNorm<1>();
  }

  /**
   * @brief 初始化: 保存基底路径数据
   */
  void SampleMani::init(const std::vector<Eigen::Vector3d> &car_state_list,
                        const std::vector<Eigen::Vector3d> &car_state_list_check,
                        const std::vector<double> &t_list){
    this->reset();
    this->car_state_list_ = car_state_list;
    this->car_state_list_check_ = car_state_list_check;
    this->t_list_.clear();
    for(unsigned int i = 0; i < t_list.size(); ++i){
      this->t_list_.push_back(t_list[i] * 3.0);  // 时间放大3倍 (安全裕度)
    }
    this->max_index_ = car_state_list.size();
  }

  double SampleMani::calAngleErr(double angle1, double angle2){
    return fabs(angle1 - angle2);
  }

  /**
   * @brief 碰撞检查 (沿基底路径密集采样)
   */
  bool SampleMani::checkcollision(const ManiPathNodePtr& state1, const ManiPathNodePtr& state2){
    ManiPathNodePtr cur_state = state1->index < state2->index ? state1 : state2;
    ManiPathNodePtr next_state = state1->index < state2->index ? state2 : state1;

    if(cur_state->node_state == ManiPathNode::NODE_STATE::COLLISION
       || next_state->node_state == ManiPathNode::NODE_STATE::COLLISION)
      return true;

    // 检查速度可行性
    double tau = t_list_[cur_state->index];
    double dif = (cur_state->state - next_state->state).lpNorm<Eigen::Infinity>();
    if(dif > max_joint_vel_ * tau) return true;

    // 沿基底路径采样碰撞检查
    Eigen::Vector3d xt;
    for(int i = 1; i < check_num_; ++i){
      xt = car_state_list_check_[cur_state->index * check_num_ + i];
      if(mm_config_->checkManicollision(xt,
          (cur_state->state + (next_state->state - cur_state->state) * (double)i / (double)check_num_),
          false)){
        return true;
      }
    }
    return false;
  }

  /**
   * @brief 获取完整路径 (回溯 + oneShot 优化)
   */
  bool SampleMani::getTraj(std::vector<Eigen::VectorXd> &traj){
    traj.clear();
    if(!have_path_) return false;

    ManiPathNodePtr node = end_node_;
    trajShot(node);  // 对全路径做 oneShot 优化

    while(node != nullptr){
      traj.push_back(node->state);
      node = node->parent;
    }
    reverse(traj.begin(), traj.end());
    return true;
  }

  double SampleMani::getCost(){
    if(!have_path_) return -1.0;
    ManiPathNodePtr node = end_node_;
    double cost = 0.0;
    while(node != nullptr && node->parent != nullptr){
      cost += estimateHeuristic(node->parent, node);
      node = node->parent;
    }
    return cost;
  }

  void SampleMani::reset(){
    this->max_index_ = 0;
    this->car_state_list_.clear();
    this->car_state_list_check_.clear();
    this->t_list_.clear();
    this->have_path_ = false;
    this->tree_max_index_ = 0;
    this->tree_min_index_ = 1 << 20;
    this->tree_count_ = 0;
    this->anti_tree_count_ = 0;
    this->end_node_ = nullptr;

    for(auto it = node_pool_.begin(); it != node_pool_.end(); ++it){
      delete it->second;
    }
    node_pool_.clear();
  }

  void SampleMani::setParam(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<remani_planner::MMConfig> &mm_config){
    node_ = node;
    mm_config_ = mm_config;
    manipulator_link_pts_ = mm_config_->getLinkPoint();
    rrt_plan_.reset(new remani_planner::RrtPlanning);
    rrt_plan_->setParam(node_, mm_config);

    getManiParam(node_, "mm.mobile_base_dof", mobile_base_dof_, -1);
    getManiParam(node_, "mm.manipulator_dof", manipulator_dof_, -1);
    getManiParam(node_, "mm.manipulator_thickness", mani_thickness_, -1.0);

    std::vector<double> joint_pos_limit;
    getManiParam(node_, "mm.manipulator_min_pos", joint_pos_limit, std::vector<double>{});
    min_joint_pos_.resize(joint_pos_limit.size());
    for(unsigned int i = 0; i < joint_pos_limit.size(); i++)
        min_joint_pos_(i) = joint_pos_limit[i];

    joint_pos_limit.clear();
    getManiParam(node_, "mm.manipulator_max_pos", joint_pos_limit, std::vector<double>{});
    max_joint_pos_.resize(joint_pos_limit.size());
    for(unsigned int i = 0; i < joint_pos_limit.size(); i++)
        max_joint_pos_(i) = joint_pos_limit[i];

    getManiParam(node_, "mm.manipulator_max_vel", max_joint_vel_, -1.0);
    getManiParam(node_, "mm.manipulator_max_acc", max_joint_acc_, -1.0);
    getManiParam(node_, "search.check_num", check_num_, -1);
    getManiParam(node_, "search.goal_rate", goal_rate_, 0.4);
    getManiParam(node_, "search.max_loop_num", max_loop_num_, 500);
    getManiParam(node_, "search.max_mani_search_time", max_mani_search_time_, 0.1);
    getManiParam(node_, "optimization.self_safe_margin", self_safe_margin_, 0.1);
    getManiParam(node_, "optimization.safe_margin_mani", safe_margin_mani_, 0.1);
    getManiParam(node_, "mm.mobile_base_check_radius", mobile_base_check_radius_, 0.1);

    traj_dim_ = mobile_base_dof_ + manipulator_dof_;
    phi_.setIdentity(3, 3);
    T_q_0_ = mm_config_->getTq0();
  }
}  //namespace mani_sample
