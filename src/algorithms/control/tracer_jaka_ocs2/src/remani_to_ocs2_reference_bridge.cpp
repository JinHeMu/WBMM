// =============================================================================
// REMANI PolynomialTraj → OCS2 全身轨迹参考 桥接节点
// =============================================================================
//
// 【背景】
// REMANI 规划器在平坦输出空间中进行规划：
//   p = [x, y, q1, q2, q3, q4, q5, q6]                        (8 维)
//
// 它针对每个恒定齿轮方向（singul = +1 前进 / -1 后退）的分段发布一条
// PolynomialTraj 消息。OCS2 MPC 则在全状态空间中工作：
//   x = [x, y, yaw, q1, q2, q3, q4, q5, q6]                   (9 维)
//   u = [v, omega, qdot1, qdot2, qdot3, qdot4, qdot5, qdot6]   (8 维)
//
// 【职责】
// 1. 将 REMANI 分段按 trajectory_id 排序并组装成完整轨迹。
// 2. 在每个采样点解析多项式系数，得到 (位置, 速度, 加速度)。
// 3. 由 (vx, vy, singul) 重建 yaw, v, omega。
// 4. 按照 OCS2 observation 时间线发布滚动的局部参考窗口。
//
// 【设计要点】
// - 通过 debounce timer 检测分段结束时点，避免消息流中途发布不完整轨迹。
// - 显式处理零速度退化情况（起/终点、换向点）。
// - yaw 逐点解包，确保参考方向连续。
// - 在参考轨迹起始位置锚定当前观测状态，消除跳变。
// - 轨迹结束后继续保持终端位姿，直到新的轨迹或 abort 指令到达。
//
// 【对应文档】
// 参见 docs/REMANI_OCS2_INTEGRATION.md 与 docs/总体 Pipeline.md。
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

#include <quadrotor_msgs/msg/polynomial_matrix.hpp>
#include <quadrotor_msgs/msg/polynomial_traj.hpp>

// =============================================================================
// 匿名命名空间 —— 内部工具函数与数据结构
// =============================================================================
namespace
{

// ---------------------------------------------------------------------------
// wrapToPi
//     将角度规整到 [-π, π] 区间。
//     使用 atan2(sin,cos) 实现，较 fmod 方案更稳定。
// ---------------------------------------------------------------------------
double wrapToPi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// ---------------------------------------------------------------------------
// unwrapNear
//     以 reference 为基准对 angle 进行连续解包。
//     保证多次采样中 yaw 不会因为跨越 ±π 而出现整圈跳变。
//
//     例：上一帧 yaw=3.10 rad，当前帧 yaw=-3.12 rad，
//         则解包后返回约 3.16 rad。
// ---------------------------------------------------------------------------
double unwrapNear(double angle, double reference)
{
  return reference + wrapToPi(angle - reference);
}

// ---------------------------------------------------------------------------
// PolynomialSample
//     单次多项式采样的"快照"结果。
//     · position[dim]      — 平坦输出位置 (x, y, q1..q6)
//     · velocity[dim]      — 平坦输出速度 (vx, vy, qdot1..qdot6)
//     · acceleration[dim]  — 平坦输出加速度 (ax, ay, ...)
//     · singul             — 当前段齿轮方向 (REMANI 定义 +1/-1)
// ---------------------------------------------------------------------------
struct PolynomialSample
{
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
  int singul = 1;
};

// ---------------------------------------------------------------------------
// TrajectorySection
//     对应一条 REMANI PolynomialTraj 消息（即轨迹的一个恒速齿轮分段）。
//     一条分段包含若干多项式碎片（pieces），每个 piece 拥有独立的 duration
//     和系数矩阵。duration() 返回该分段的总时长。
// ---------------------------------------------------------------------------
struct TrajectorySection
{
  uint32_t id = 0;                                       ///< REMANI trajectory_id
  int singul = 1;                                        ///< 齿轮方向: +1 前进, -1 倒退
  std::vector<quadrotor_msgs::msg::PolynomialMatrix> pieces; ///< 多项式碎片列表

  /// 返回本分段内所有碎片时长之和
  double duration() const
  {
    double total = 0.0;
    for (const auto &piece : pieces)
    {
      total += std::max(0.0, piece.duration);
    }
    return total;
  }
};

// ---------------------------------------------------------------------------
// AssembledTrajectory
//     由多个 TrajectorySection 按 id 排序拼接而成的完整轨迹。
//
//     · startStamp  — 基于 ROS 时钟的轨迹起始时刻（源自第一条消息的
//                      header.stamp）。
//     · sections    — 按 id 升序排列的分段列表。
//     · totalDuration — 所有分段时长之和。
//     · generation  — 单调递增的代次计数器，每次收到新 id=1 时增加。
//
//     该结构体是可拷贝的值类型，允许在锁外安全地访问完整轨迹数据：
//     publishReference() 持有锁时会将其拷贝到 local 变量。
// ---------------------------------------------------------------------------
struct AssembledTrajectory
{
  rclcpp::Time startStamp{0, 0, RCL_ROS_TIME};
  std::vector<TrajectorySection> sections;
  double totalDuration = 0.0;
  uint64_t generation = 0;

  bool empty() const
  {
    return sections.empty() || totalDuration <= 0.0;
  }
};

// ---------------------------------------------------------------------------
// evaluatePiece
//     在时刻 t 处对一段多项式碎片进行采样。
//
//     REMANI 的多项式系数按如下方式存储：
//       · num_dim  — 输出维度数（= 8）
//       · num_order — 多项式次数
//       · data     — 列主序系数矩阵: data[col * dim + d]，其中
//                     col=0 → 最高次项, col=degree → 常量项
//
//     对于每个维度 d，采样时计算：
//       position[d] = Σ_{col=0}^{degree} c_{d,col} · t^{degree-col}
//       velocity[d] = Σ_{col=0}^{degree-1}
//                       (degree-col)·c_{d,col} · t^{degree-col-1}
//       accel[d]    = Σ_{col=0}^{degree-2}
//                       (degree-col)(degree-col-1)·c_{d,col} · t^{degree-col-2}
//
//     返回 false 表示系数数据无效（维度或长度不匹配）。
// ---------------------------------------------------------------------------
bool evaluatePiece(
    const quadrotor_msgs::msg::PolynomialMatrix &piece,
    double time,
    PolynomialSample &sample)
{
  const size_t dim = static_cast<size_t>(piece.num_dim);
  const size_t degree = static_cast<size_t>(piece.num_order);
  const size_t expected = dim * (degree + 1U);
  if (dim == 0U || piece.data.size() != expected)
  {
    return false;
  }

  // 将时间钳位到碎片的有效时间段 [0, duration]
  const double t = std::clamp(time, 0.0, std::max(0.0, piece.duration));
  sample.position.assign(dim, 0.0);
  sample.velocity.assign(dim, 0.0);
  sample.acceleration.assign(dim, 0.0);

  // REMANI 将 Eigen 列主序系数矩阵直接拷贝至 data。
  // 第 0 列为最高次幂，第 degree 列为常数项。
  for (size_t column = 0; column <= degree; ++column)
  {
    const size_t power = degree - column;            // 当前列的幂次
    const double tPower = std::pow(t, static_cast<int>(power));
    for (size_t d = 0; d < dim; ++d)
    {
      const double c = piece.data[column * dim + d]; // 第 d 维、第 column 列的系数

      // 位置：c · t^power
      sample.position[d] += c * tPower;

      // 速度：一阶导数，power·c · t^{power-1}
      if (power >= 1U)
      {
        sample.velocity[d] +=
            static_cast<double>(power) * c *
            std::pow(t, static_cast<int>(power - 1U));
      }

      // 加速度：二阶导数，power·(power-1)·c · t^{power-2}
      if (power >= 2U)
      {
        sample.acceleration[d] +=
            static_cast<double>(power * (power - 1U)) * c *
            std::pow(t, static_cast<int>(power - 2U));
      }
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// sampleTrajectory
//     在已拼接完成的轨迹上按 relativeTime 采样。
//
//     搜索策略：
//       1. 遍历 sections，累加 section.duration()。
//       2. 当 relativeTime 落入某个 section 的区间（或已到达最后一个 section），
//          进入其 piece 列表。
//       3. 如步骤 1 和 2，遍历 pieces 直至 relativeTime 落入对应碎片的
//          [0, duration] 区间。
//     若通过上述流程找到目标 piece，则调用 evaluatePiece() 并设置 sample.singul。
//     若未找到（理论上不会发生），返回 false。
// ---------------------------------------------------------------------------
bool sampleTrajectory(
    const AssembledTrajectory &trajectory,
    double relativeTime,
    PolynomialSample &sample)
{
  if (trajectory.empty())
  {
    return false;
  }

  // 将相对时间钳位到轨迹有效区间 [0, totalDuration]
  double t = std::clamp(relativeTime, 0.0, trajectory.totalDuration);
  for (size_t sectionIndex = 0;
       sectionIndex < trajectory.sections.size();
       ++sectionIndex)
  {
    const auto &section = trajectory.sections[sectionIndex];
    const double sectionDuration = section.duration();
    const bool lastSection = sectionIndex + 1U == trajectory.sections.size();

    // 如果 t 落在当前 section 内，或者当前已是最后一个 section（安全钳位）
    if (t <= sectionDuration || lastSection)
    {
      double pieceTime = std::clamp(t, 0.0, sectionDuration);
      for (size_t pieceIndex = 0; pieceIndex < section.pieces.size(); ++pieceIndex)
      {
        const auto &piece = section.pieces[pieceIndex];
        const bool lastPiece = pieceIndex + 1U == section.pieces.size();
        if (pieceTime <= piece.duration || lastPiece)
        {
          if (!evaluatePiece(piece, pieceTime, sample))
          {
            return false;
          }
          // singul 的符号控制 yaw 重建时的方向
          sample.singul = section.singul >= 0 ? 1 : -1;
          return true;
        }
        pieceTime -= piece.duration;
      }
      return false;  // 理论上不会执行到这里
    }
    // 尚未进入目标 section，跳过当前 section 的时长
    t -= sectionDuration;
  }
  return false;  // 理论上不会执行到这里
}

}  // namespace

// =============================================================================
// RemaniToOcs2ReferenceBridge 类
//     主节点类，负责接收 REMANI 多项式轨迹并发布 OCS2 TargetTrajectories。
//
// 【ROS 接口】
//   Subscribers:
//     · /planning/trajectory          — quadrotor_msgs/msg/PolynomialTraj
//     · {robot_name}_mpc_observation  — ocs2_msgs/msg/MpcObservation
//   Publishers:
//     · {robot_name}_mpc_target       — ocs2_msgs/msg/MpcTargetTrajectories
//                                      （通过 TargetTrajectoriesRosPublisher）
//
// 【定时器】
//   · assembly_timer_  — 分段收集 debounce timer（单次触发，时长 = assembly_timeout_）
//   · publish_timer_   — 周期参考发布 timer（频率 = publish_rate_, 默认 20 Hz）
//
// 【线程模型】
//   两个 subscriber 回调与两个 timer 回调共享数据，所有共享字段均受 mutex_ 保护。
//   发布路径（publishReference → sampleAt）在持有锁期间将 active_/pending_ 拷贝
//   到栈上，从而避免在锁内调用执行时间不定的采样逻辑。
// =============================================================================
class RemaniToOcs2ReferenceBridge final : public rclcpp::Node
{
public:
  // ---------------------------------------------------------------------------
  // 构造函数
  //     从参数服务器读入所有可配置参数并设置 subscriber 与 timer。
  //     参数校验失败（如 state_dim != arm_dim+3）会抛出异常。
  //     对象构造完成后必须调用 init() 才能激活 TargetTrajectoriesRosPublisher。
  // ---------------------------------------------------------------------------
  RemaniToOcs2ReferenceBridge()
      : Node("remani_to_ocs2_reference_bridge")
  {
    // ---- 参数声明 ----------------------------------------------------------
    robotName_ = declare_parameter<std::string>(
        "robot_name", "mobile_manipulator");
    trajectoryTopic_ = declare_parameter<std::string>(
        "trajectory_topic", "/planning/trajectory");
    stateDim_ = declare_parameter<int>("state_dim", 9);
    inputDim_ = declare_parameter<int>("input_dim", 8);
    armDim_ = declare_parameter<int>("arm_dim", 6);
    sampleDt_ = declare_parameter<double>("sample_dt", 0.04);
    referenceHorizon_ = declare_parameter<double>("reference_horizon", 3.0);
    startLead_ = declare_parameter<double>("start_lead", 0.05);
    publishRate_ = declare_parameter<double>("publish_rate", 20.0);
    assemblyTimeout_ = declare_parameter<double>(
        "assembly_timeout", 0.04);
    zeroVelocityThreshold_ = declare_parameter<double>(
        "zero_velocity_threshold", 1.0e-4);
    holdAtEnd_ = declare_parameter<double>("hold_at_end", 2.0);
    transformX_ = declare_parameter<double>("planner_to_ocs2_x", 0.0);
    transformY_ = declare_parameter<double>("planner_to_ocs2_y", 0.0);
    transformYaw_ = declare_parameter<double>("planner_to_ocs2_yaw", 0.0);
    plannerFrame_ = declare_parameter<std::string>("planner_frame", "map");
    targetFrame_ = declare_parameter<std::string>("target_frame", "odom");
    useTfTransform_ = declare_parameter<bool>("use_tf_transform", true);
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    // ---- 参数校验 ----------------------------------------------------------
    if (stateDim_ != armDim_ + 3 || inputDim_ != armDim_ + 2)
    {
      throw std::runtime_error(
          "Expected state_dim=arm_dim+3 and input_dim=arm_dim+2.");
    }
    // 对非正或过小参数进行下界约束
    sampleDt_ = std::max(sampleDt_, 0.005);
    referenceHorizon_ = std::max(referenceHorizon_, sampleDt_);
    publishRate_ = std::max(publishRate_, 1.0);
    assemblyTimeout_ = std::max(assemblyTimeout_, 0.005);

    // ---- Subscriber: REMANI 轨迹 -------------------------------------------
    const auto trajectoryQos =
        rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    trajectorySub_ =
        create_subscription<quadrotor_msgs::msg::PolynomialTraj>(
            trajectoryTopic_, trajectoryQos,
            std::bind(
                &RemaniToOcs2ReferenceBridge::trajectoryCallback,
                this, std::placeholders::_1));

    // ---- Subscriber: MPC 观测 (best-effort, 仅保留最新一条) ---------------
    const std::string observationTopic =
        robotName_ + "_mpc_observation";
    observationSub_ =
        create_subscription<ocs2_msgs::msg::MpcObservation>(
            observationTopic, rclcpp::QoS(1).best_effort(),
            std::bind(
                &RemaniToOcs2ReferenceBridge::observationCallback,
                this, std::placeholders::_1));

    // ---- Timer: 分段拼接 debounce timer（创建后立即取消，按需激活） --------
    assemblyTimer_ = create_wall_timer(
        std::chrono::duration<double>(assemblyTimeout_),
        std::bind(
            &RemaniToOcs2ReferenceBridge::finishAssembly, this));
    assemblyTimer_->cancel();  // 初始不启用，收到第一条分段后 activate

    // ---- Timer: 周期参考发布 ------------------------------------------------
    publishTimer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publishRate_),
        std::bind(
            &RemaniToOcs2ReferenceBridge::publishReference, this));

    RCLCPP_INFO(
        get_logger(),
        "Bridge ready: %s -> %s_mpc_target, sample_dt=%.3f s, "
        "horizon=%.2f s.",
        trajectoryTopic_.c_str(), robotName_.c_str(), sampleDt_,
        referenceHorizon_);
  }

  // ---------------------------------------------------------------------------
  // init
  //     初始化 OCS2 TargetTrajectories 发布器。
  //     必须作为构造后调用的第一步（约等于二阶段构造），因为在构造函数中
  //     shared_from_this() 尚不可用。
  // ---------------------------------------------------------------------------
  void init()
  {
    targetPublisher_ =
        std::make_unique<ocs2::TargetTrajectoriesRosPublisher>(
            shared_from_this(), robotName_);
  }

private:
  // TF / frame configuration for dynamic map->odom reference transformation.
  std::string plannerFrame_;
  std::string targetFrame_;
  bool useTfTransform_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  // ===========================================================================
  // observationCallback
  //     接收 MPC 观测消息，缓存最新的观测状态、时间和 ROS 时间戳。
  //
  //     注意：观测时间采用"MPC 内部时钟"——节点启动时从 0 开始。
  //     因此需要记录对应的 ROS wall time，以便 publishReference() 中
  //     计算 clock skew：
  //         obsTimeNow = observationTime_ + (now() - observationRosStamp_).seconds()
  // ===========================================================================
  void observationCallback(
      const ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg)
  {
    // 维度检查，拒绝与预期不匹配的观测
    if (static_cast<int>(msg->state.value.size()) != stateDim_)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Ignoring MPC observation with state dimension %zu (expected %d).",
          msg->state.value.size(), stateDim_);
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    observationTime_ = msg->time;
    observationState_.resize(stateDim_);
    for (int i = 0; i < stateDim_; ++i)
    {
      observationState_(i) =
          static_cast<double>(msg->state.value[static_cast<size_t>(i)]);
    }
    observationRosStamp_ = now();  // 记录消息到达时的 ROS wall time
    haveObservation_ = true;
  }

  // ===========================================================================
  // trajectoryCallback
  //     REMANI 轨迹消息的主入口，实现了分段收集状态机。
  //
  //     【action 分发】
  //       · ACTION_ADD            — 正常追加分段
  //       · ACTION_ABORT          — 清空所有轨迹并发布 hold 目标
  //       · ACTION_WARN_IMPOSSIBLE — 同上
  //
  //     【分段拼接协议】
  //       REMANI 通过 trajectory_id 实现无 count 字段的多消息批量传输：
  //         · id == 1   → 批开始，清空 assembling_。
  //         · id > 1    → 追加到已缓存的 assembling_ map 中。
  //       每次收到消息后重置 debounce timer；timer 到期后视为批结束，
  //       由 finishAssembly() 完成拼接。
  //
  //     【数据校验】
  //       每条分段在入队前对维度、系数数量和 duration 进行检查。
  // ===========================================================================
  void trajectoryCallback(
      const quadrotor_msgs::msg::PolynomialTraj::ConstSharedPtr msg)
  {
    using Message = quadrotor_msgs::msg::PolynomialTraj;

    // ---- 异常/终止指令 → 立即清除所有轨迹并发布 hold 参考 ---------------
    if (msg->action == Message::ACTION_ABORT ||
        msg->action == Message::ACTION_WARN_IMPOSSIBLE)
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        assembling_.clear();
        active_ = AssembledTrajectory{};
        pending_ = AssembledTrajectory{};
        havePending_ = false;
      }
      assemblyTimer_->cancel();
      RCLCPP_WARN(
          get_logger(), "REMANI aborted the trajectory; publishing hold target.");
      publishReference();  // 立即发布当前位姿保持目标
      return;
    }

    // ---- 仅处理 ACTION_ADD，且轨迹必须非空 --------------------------------
    if (msg->action != Message::ACTION_ADD || msg->trajectory.empty())
    {
      return;
    }

    // ---- 分段校验：维度、系数长度、duration --------------------------------
    TrajectorySection section;
    section.id = msg->trajectory_id;
    section.singul = msg->singul >= 0 ? 1 : -1;
    section.pieces = msg->trajectory;

    for (const auto &piece : section.pieces)
    {
      if (static_cast<int>(piece.num_dim) != armDim_ + 2 ||
          piece.data.size() !=
              static_cast<size_t>(piece.num_dim) *
                  (static_cast<size_t>(piece.num_order) + 1U) ||
          piece.duration <= 0.0)
      {
        RCLCPP_ERROR(
            get_logger(),
            "Rejected REMANI section %u: invalid dimension, coefficient "
            "count or duration.",
            msg->trajectory_id);
        return;
      }
    }

    // ---- 分段收集（持有锁） ------------------------------------------------
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (msg->trajectory_id == 1U)
      {
        // 新轨迹批次开始：清空上一批次的分段缓存
        assembling_.clear();
        assemblyStartStamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
        // 若 header.stamp 为零（时钟未设置），回退为当前墙钟时间
        if (assemblyStartStamp_.nanoseconds() == 0)
        {
          assemblyStartStamp_ = now();
        }
        ++assemblyGeneration_;  // 递增代次计数器，用于日志追踪
      }
      else if (assembling_.empty())
      {
        // id != 1 但 assembling_ 为空：表示我们错过了第 1 条消息，
        // 无法正确构建连续的完整轨迹
        RCLCPP_WARN(
            get_logger(),
            "Ignoring REMANI section %u because section 1 has not arrived.",
            msg->trajectory_id);
        return;
      }
      assembling_[section.id] = std::move(section);
    }

    // ---- 重置 debounce timer，延迟拼接 ------------------------------------
    // 由于 timer 已创建但初始为 cancel 状态，reset() 会重新激活。
    assemblyTimer_->reset();
  }

  // ===========================================================================
  // finishAssembly
  //     分段拼接 debounce timer 到期时被调用。
  //
  //     1. 将 assembling_ map（按 id 排序）逐个拷贝至 AssembledTrajectory。
  //     2. 检查 id 连续性：期望 id=1,2,3,... 不允许跳号。
  //     3. 累加总持续时间。
  //     4. 将组装结果存入 pending_ 并设置 havePending_。
  //     5. 最后调用 publishReference() 立即推送组装完成的新轨迹。
  //
  //     注意：新轨迹不会立即激活。只有当 ROS wall time 到达其 startStamp 时，
  //     sampleAt() 才会将 pending_ 移动至 active_。这保证了在轨迹的 REMANI 起始
  //     时刻到来之前，系统继续使用旧轨迹参考。
  // ===========================================================================
  void finishAssembly()
  {
    assemblyTimer_->cancel();  // 停用 timer，等待下次收到分段时重新激活

    AssembledTrajectory assembled;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (assembling_.empty())
      {
        return;  // 在 timer 触发与回调执行之间 assembling_ 可能已被清空
      }

      // 按 id 顺序遍历 std::map（已排序），检查连续性
      uint32_t expectedId = 1U;
      assembled.startStamp = assemblyStartStamp_;
      assembled.generation = assemblyGeneration_;
      for (const auto &[id, section] : assembling_)
      {
        if (id != expectedId)
        {
          // 存在跳号 → 分段序列不完整，丢弃该批次
          RCLCPP_ERROR(
              get_logger(),
              "REMANI trajectory has a segment gap: expected %u, received %u.",
              expectedId, id);
          assembling_.clear();
          return;
        }
        assembled.totalDuration += section.duration();
        assembled.sections.push_back(section);
        ++expectedId;
      }
      assembling_.clear();  // 消费完毕

      pending_ = assembled;
      havePending_ = true;
    }

    RCLCPP_INFO(
        get_logger(),
        "Assembled REMANI generation %lu: %zu sections, %.3f s.",
        static_cast<unsigned long>(assembled.generation),
        assembled.sections.size(), assembled.totalDuration);
    // 立即推送组装完成的新轨迹参考
    publishReference();
  }

  // ===========================================================================
  // getPlannerToTargetTransform
  //     Look up the current planner_frame -> target_frame transform for the
  //     given ROS timestamp. Returns false if TF is unavailable (caller then
  //     falls back to the legacy fixed planner_to_ocs2_* parameters).
  // ===========================================================================
  bool getPlannerToTargetTransform(
      const rclcpp::Time & stamp,
      double & tx,
      double & ty,
      double & yaw)
  {
    if (!useTfTransform_ || plannerFrame_ == targetFrame_)
    {
      return false;
    }
    try
    {
      const auto transform = tfBuffer_->lookupTransform(
          targetFrame_, plannerFrame_, stamp,
          tf2::durationFromSec(0.1));
      const auto & q = transform.transform.rotation;
      tx = transform.transform.translation.x;
      ty = transform.transform.translation.y;
      yaw = std::atan2(
          2.0 * (q.w * q.z + q.x * q.y),
          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return true;
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Cannot transform %s -> %s: %s; using fixed transform.",
          plannerFrame_.c_str(), targetFrame_.c_str(), ex.what());
      return false;
    }
  }

  // ===========================================================================
  // sampleAt
  //     在指定 ROS 时间戳采样参考轨迹，输出单个 (state, input) 对。
  //
  //     【调用约定】
  //     调用方（publishReference）在调用前必须已持有 mutex_。
  //
  //     【参数】
  //       · rosStamp   — 目标采样 ROS 时间（墙钟）。
  //       · holdState  — 当无可用轨迹或采样失败时的回退状态。
  //       · yawReference — [in/out] 上一次采样的 yaw 值，用于解包；
  //                         输入时用于 unwrap，输出时写入新 yaw。
  //       · state      — [out] OCS2 状态向量，维度 = stateDim_。
  //       · input      — [out] OCS2 输入向量，维度 = inputDim_。
  //
  //     【返回值】
  //       成功返回 true，采样失败返回 false。
  //
  //     【轨迹激活时机】
  //       当 havePending_ 为 true 且 rosStamp ≥ pending_.startStamp 时，
  //       将 pending_ 移动到 active_。在此之前系统继续使用旧 active_。
  //
  //     【数据转换】
  //       REMANI 输出 → OCS2 状态/控制的关键步骤：
  //       1. 在 planner 与 odom 坐标系之间应用可配置的 2D 刚体变换。
  //       2. 由 vx, vy, singul 重建 yaw、前向/后退速度和偏航角速度。
  //          - yaw = unwrapNear(atan2(singul·vy, singul·vx), yawReference)
  //          - v   = singul · √(vx²+vy²)
  //          - ω   = (vx·ay − vy·ax) / (vx²+vy²)
  //       3. 速度低于阈值时跳过 yaw/ω 计算，避免数值退化。
  //       4. 轨迹结束后将 input 强制置零（保持位姿），但不影响 state。
  // ===========================================================================
  bool sampleAt(
      const rclcpp::Time &rosStamp,
      const ocs2::vector_t &holdState,
      double &yawReference,
      ocs2::vector_t &state,
      ocs2::vector_t &input)
  {
    // ---- 选择轨迹：如果有 pending 且到时了，则激活 ---------------------------
    AssembledTrajectory selected;
    bool haveSelected = false;
    {
      // 注：调用方 publishReference() 已持有 mutex_
      if (havePending_ && rosStamp >= pending_.startStamp)
      {
        // 新轨迹的起始时刻到了 → 切换到 pending 轨迹
        active_ = pending_;
        pending_ = AssembledTrajectory{};  // 清空 pending
        havePending_ = false;
      }

      if (!active_.empty() && rosStamp >= active_.startStamp)
      {
        selected = active_;           // 拷贝到栈上，后续采样无需锁
        haveSelected = true;
      }
    }

    // ---- 无可用的 active 轨迹 → 回退到保持状态 ------------------------------
    if (!haveSelected)
    {
      state = holdState;                          // 保持当前观测位姿
      input = ocs2::vector_t::Zero(inputDim_);    // 零输入
      yawReference = holdState(2);                // 使用当前测量 yaw
      return true;
    }

    // ---- 计算轨迹内的相对时间 -----------------------------------------------
    const double relativeTime =
        (rosStamp - selected.startStamp).seconds();

    // ---- 多项式采样 ---------------------------------------------------------
    PolynomialSample sample;
    if (!sampleTrajectory(selected, relativeTime, sample) ||
        static_cast<int>(sample.position.size()) != armDim_ + 2)
    {
      return false;
    }

    // ---- 提取并变换平面的位置、速度和加速度 ---------------------------------
    // 优先使用动态 TF（planner_frame -> target_frame），否则退回到可配置的
    // 2D 刚体变换: (transformX_, transformY_, transformYaw_)
    double tfX = transformX_;
    double tfY = transformY_;
    double tfYaw = transformYaw_;
    if (getPlannerToTargetTransform(rosStamp, tfX, tfY, tfYaw))
    {
      RCLCPP_DEBUG(
          get_logger(), "Using dynamic TF %s -> %s.",
          plannerFrame_.c_str(), targetFrame_.c_str());
    }
    const double c = std::cos(tfYaw);
    const double s = std::sin(tfYaw);
    const double px = sample.position[0];
    const double py = sample.position[1];
    const double vx = sample.velocity[0];
    const double vy = sample.velocity[1];
    const double ax = sample.acceleration[0];
    const double ay = sample.acceleration[1];

    // 位置：p_ocs2 = t + R · p_planner
    const double x = tfX + c * px - s * py;
    const double y = tfY + s * px + c * py;
    // 速度：v_ocs2 = R · v_planner
    const double vxWorld = c * vx - s * vy;
    const double vyWorld = s * vx + c * vy;
    // 加速度：a_ocs2 = R · a_planner
    const double axWorld = c * ax - s * ay;
    const double ayWorld = s * ax + c * ay;
    const double speed2 = vxWorld * vxWorld + vyWorld * vyWorld;

    // ---- 由速度向量重建 yaw, forward velocity, omega ------------------------
    double yaw = yawReference;        // 默认保持上一帧 yaw
    double forwardVelocity = 0.0;
    double omega = 0.0;

    if (speed2 >
        zeroVelocityThreshold_ * zeroVelocityThreshold_)
    {
      // 速度足够大 → 航向角定义良好
      //
      // yaw = atan2(singul·vy, singul·vx)
      // 其中 singul 为 +1（前进）或 -1（倒车），使得 yaw 始终指向车头方向，
      // 而非运动方向。
      const double rawYaw = std::atan2(
          static_cast<double>(sample.singul) * vyWorld,
          static_cast<double>(sample.singul) * vxWorld);
      yaw = unwrapNear(rawYaw, yawReference);  // 确保与上一帧 yaw 连续

      // 带符号的线速度：singul=1 时 v>0（前进），singul=-1 时 v<0（倒车）
      forwardVelocity =
          static_cast<double>(sample.singul) * std::sqrt(speed2);

      // 偏航角速度，由平面运动学关系推导：
      //   ω = (vx·ay − vy·ax) / (vx²+vy²)
      omega =
          (vxWorld * ayWorld - vyWorld * axWorld) / speed2;
    }
    // 若速度低于阈值：yaw 保持上一帧值；v=0；omega=0
    // 这可以避免在起/终点或换向点处因 atan2 的数值不稳定导致 yaw 来回震荡。

    // ---- 组装 OCS2 状态和控制向量 ------------------------------------------
    state = ocs2::vector_t::Zero(stateDim_);
    input = ocs2::vector_t::Zero(inputDim_);

    // 状态: [x, y, yaw, q1, q2, q3, q4, q5, q6]
    state(0) = x;
    state(1) = y;
    state(2) = yaw;

    // 输入: [v, omega, qdot1, ..., qdot6]
    input(0) = forwardVelocity;
    input(1) = omega;

    for (int joint = 0; joint < armDim_; ++joint)
    {
      state(3 + joint) =
          sample.position[static_cast<size_t>(2 + joint)];    // REMANI dims 2..7 → OCS2 3..8
      input(2 + joint) =
          sample.velocity[static_cast<size_t>(2 + joint)];    // 前馈关节速度
    }

    // ---- 终点后处理：停止所有控制，保持终端姿态 ------------------------------
    if (relativeTime >= selected.totalDuration)
    {
      // 轨迹已到终点：将 input 设为零（速度归零，保持位姿）
      input.setZero();
      // holdAtEnd_ 结束后仍继续发布终端状态，active_ 轨迹一直保留至
      // REMANI 提供替代轨迹或发送 abort 指令。
      if (relativeTime > selected.totalDuration + holdAtEnd_)
      {
        // 超过保持期：继续沿用同样的 state（终端姿态），
        // 外部逻辑无需额外动作。
      }
    }

    yawReference = yaw;  // 将本帧 yaw 传出，作为下一帧解包的基准
    return true;
  }

  // ===========================================================================
  // publishReference
  //     通过 publishTimer_ 以 publishRate_ Hz 频率调用的主发布循环。
  //
  //     每个周期内：
  //       1. 检查 targetPublisher_ 和 haveObservation_ 状态。
  //       2. 在锁内根据当前 OCS2 observation 时钟生成滚动参考窗口：
  //          · 第 0 帧 = 最新观测状态（锚定帧，消除参考跳变）。
  //          · 第 1..N 帧: 逐 sampleDt_ 调用 sampleAt() 采样。
  //       3. 构建 ocs2::TargetTrajectories 并发布。
  //
  //     采样窗口参数：
  //       · sampleCount ≈ referenceHorizon_ / sampleDt_
  //       · 默认约 76 帧 × dt=0.04 s ≈ 3 s 前瞻窗口
  //       · 第 1 帧偏移 = startLead_（默认 0.05 s），避免直接采样当前时刻
  //         因数字微分引入噪声。
  // ===========================================================================
  void publishReference()
  {
    if (!targetPublisher_)
    {
      return;  // init() 尚未调用
    }

    ocs2::scalar_array_t timeTrajectory;
    ocs2::vector_array_t stateTrajectory;
    ocs2::vector_array_t inputTrajectory;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!haveObservation_)
      {
        return;  // 在收到首次 MPC 观测之前无法发布参考
      }

      // ---- 锚定帧：以当前观测状态作为第一条参考点，避免参考跳变 ------------
      const ocs2::vector_t holdState = observationState_;

      // 估算当前 OCS2 时钟：
      //   observationTime_ 是最近一次测得的时间，observationRosStamp_ 是其对应的
      //   ROS wall time；(now() - observationRosStamp_) 为从观测到此刻所经过的时间。
      const double obsTimeNow =
          observationTime_ + (now() - observationRosStamp_).seconds();
      const rclcpp::Time rosNow = now();
      double yawReference = holdState(2);  // 初始 yaw = 当前测量 yaw

      // 锚定帧
      timeTrajectory.push_back(obsTimeNow);
      stateTrajectory.push_back(holdState);
      inputTrajectory.push_back(ocs2::vector_t::Zero(inputDim_));

      // ---- 滚动采样 ---------------------------------------------------------
      const int sampleCount =
          static_cast<int>(std::ceil(referenceHorizon_ / sampleDt_)) + 1;
      timeTrajectory.reserve(static_cast<size_t>(sampleCount + 1));
      stateTrajectory.reserve(static_cast<size_t>(sampleCount + 1));
      inputTrajectory.reserve(static_cast<size_t>(sampleCount + 1));

      for (int i = 0; i < sampleCount; ++i)
      {
        const double offset = startLead_ + static_cast<double>(i) * sampleDt_;
        ocs2::vector_t state;
        ocs2::vector_t input;
        if (!sampleAt(
                rosNow + rclcpp::Duration::from_seconds(offset),
                holdState, yawReference, state, input))
        {
          // 多项式采样失败 → 跳过一个发布周期
          RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Failed to evaluate REMANI polynomial reference.");
          return;
        }
        timeTrajectory.push_back(obsTimeNow + offset);
        stateTrajectory.push_back(std::move(state));
        inputTrajectory.push_back(std::move(input));
      }
    }  // mutex_ 解锁

    // ---- 发布 TargetTrajectories -------------------------------------------
    ocs2::TargetTrajectories target(
        std::move(timeTrajectory),
        std::move(stateTrajectory),
        std::move(inputTrajectory));
    targetPublisher_->publishTargetTrajectories(target);
  }

  // ===========================================================================
  // 参数（由 launch 文件/参数服务器配置）
  // ===========================================================================
  std::string robotName_;               ///< 机器人名称，用于构造 topic 前缀
  std::string trajectoryTopic_;         ///< REMANI 轨迹话题名称
  int stateDim_ = 9;                    ///< OCS2 状态维度（默认 9）
  int inputDim_ = 8;                    ///< OCS2 输入维度（默认 8）
  int armDim_ = 6;                      ///< 机器人手臂关节数（默认 6）
  double sampleDt_ = 0.04;              ///< 参考采样步长 [s]
  double referenceHorizon_ = 3.0;       ///< 参考窗口前瞻时间 [s]
  double startLead_ = 0.05;             ///< 首帧采样点相对当前时刻的偏移 [s]
  double publishRate_ = 20.0;           ///< 参考发布频率 [Hz]
  double assemblyTimeout_ = 0.04;       ///< 分段拼接 debounce 超时 [s]
  double zeroVelocityThreshold_ = 1.0e-4; ///< 零速度判定阈值（平方值用于比较）
  double holdAtEnd_ = 2.0;              ///< 轨迹终点保持时间 [s]
  double transformX_ = 0.0;             ///< planner→ocs2 坐标系 x 平移
  double transformY_ = 0.0;             ///< planner→ocs2 坐标系 y 平移
  double transformYaw_ = 0.0;           ///< planner→ocs2 坐标系偏航角旋转 [rad]

  // ===========================================================================
  // 共享状态（受 mutex_ 保护）
  // ===========================================================================
  std::mutex mutex_;

  // -- MPC 观测缓存 --
  bool haveObservation_ = false;        ///< 是否已收到首次观测
  double observationTime_ = 0.0;        ///< 最近一次观测的 OCS2 时间（启动后从 0 开始）
  rclcpp::Time observationRosStamp_{0, 0, RCL_ROS_TIME}; ///< 观测消息到达时的 ROS wall time
  ocs2::vector_t observationState_;     ///< 最近一次观测的状态向量

  // -- 分段拼接状态 --
  std::map<uint32_t, TrajectorySection> assembling_; ///< 批次进行中的分段集合（按 id 排序）
  rclcpp::Time assemblyStartStamp_{0, 0, RCL_ROS_TIME}; ///< 当前批次的起始 ROS 时间戳
  uint64_t assemblyGeneration_ = 0;     ///< 当前批次的代次编号

  // -- 轨迹缓存（双缓冲：active_ + pending_） --
  AssembledTrajectory active_;           ///< 当前正在执行的轨迹
  AssembledTrajectory pending_;          ///< 队列中等待激活的新轨迹
  bool havePending_ = false;             ///< 是否有排队中的轨迹

  // ===========================================================================
  // ROS 通信与定时器
  // ===========================================================================
  rclcpp::Subscription<quadrotor_msgs::msg::PolynomialTraj>::SharedPtr
      trajectorySub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr
      observationSub_;
  rclcpp::TimerBase::SharedPtr assemblyTimer_;   ///< 分段拼接 debounce timer
  rclcpp::TimerBase::SharedPtr publishTimer_;    ///< 周期参考发布 timer
  std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher> targetPublisher_;
};

// =============================================================================
// main
//     节点入口点，构造 bridge 实例后进入 ROS spin 循环。
// =============================================================================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RemaniToOcs2ReferenceBridge>();
  node->init();          // 必须紧接构造后调用，以初始化 TargetTrajectoriesRosPublisher
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
