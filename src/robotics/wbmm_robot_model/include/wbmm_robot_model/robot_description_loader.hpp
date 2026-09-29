#pragma once

#include "wbmm_robot_model/robot_description.hpp"

#include <string>

namespace wbmm::robot_model
{

struct RobotDescriptionLoadResult
{
  bool success{false};
  std::string message;
  RobotDescription description;
};

// URDF 统一读取入口。底层用 urdfdom（ROS-free），不再维护第二套 XML 语义
// 解析器。两个入口解析出的 RobotDescription 完全等价。
class RobotDescriptionLoader
{
public:
  [[nodiscard]] static RobotDescriptionLoadResult fromFile(
    const std::string & urdf_path);

  [[nodiscard]] static RobotDescriptionLoadResult fromXml(
    const std::string & xml, const std::string & source = "<xml>");
};

// 失败即抛异常的便捷包装，供已经持有有效 URDF 的节点使用。
[[nodiscard]] RobotDescription loadRobotDescription(const std::string & urdf_path);
[[nodiscard]] RobotDescription parseRobotDescription(
  const std::string & xml, const std::string & source = "<xml>");

}  // namespace wbmm::robot_model
