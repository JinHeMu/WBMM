/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "wbmm_ocs2/FactoryFunctions.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include <urdf_parser/urdf_parser.h>

#include <ocs2_pinocchio_interface/urdf.h>

namespace wbmm_ocs2
{

// 迁移期：原 OCS2 mobile_manipulator 代码位于 ocs2 命名空间内，
// 这里用文件级 using-directive 保持上游类型的可见性，
// 不污染被 include 的头文件。
using namespace ocs2;


PinocchioInterface createWbmmPinocchioInterface(
  const std::string & robotUrdfPath,
  const std::vector<std::string> & jointNames)
{
  using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;

  const auto urdfTree = ::urdf::parseURDFFile(robotUrdfPath);
  if (!urdfTree) {
    throw std::runtime_error(
            "[createWbmmPinocchioInterface] Failed to parse URDF: " + robotUrdfPath);
  }
  auto newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);

  // 第一遍：显式指定为 fixed 的关节。
  for (joint_pair_t & jointPair : newModel->joints_) {
    if (std::find(jointNames.begin(), jointNames.end(), jointPair.first) !=
      jointNames.end())
    {
      jointPair.second->type = ::urdf::Joint::FIXED;
    }
  }

  // 第二遍：自动移除 mimic 关节，避免把被动关节当成驱动 DoF。
  std::vector<std::string> mimicJointNames;
  for (const joint_pair_t & jointPair : newModel->joints_) {
    if (jointPair.second->mimic) {
      mimicJointNames.push_back(jointPair.first);
      std::cerr << " #### Auto-detected mimic joint: \"" << jointPair.first
                << "\" -> \"" << jointPair.second->mimic->joint_name
                << "\"" << std::endl;
    }
  }
  for (joint_pair_t & jointPair : newModel->joints_) {
    if (std::find(
        mimicJointNames.begin(), mimicJointNames.end(),
        jointPair.first) != mimicJointNames.end())
    {
      jointPair.second->type = ::urdf::Joint::FIXED;
      std::cerr << " #### Auto-removed mimic joint: \"" << jointPair.first
                << "\"" << std::endl;
    }
  }

  // WBMM 固定模型：在 URDF 根链路前加 planar base [PX, PY, RZ]。
  pinocchio::JointModelComposite jointComposite(3);
  jointComposite.addJoint(pinocchio::JointModelPX());
  jointComposite.addJoint(pinocchio::JointModelPY());
  jointComposite.addJoint(pinocchio::JointModelRZ());
  return getPinocchioInterfaceFromUrdfModel(newModel, jointComposite);
}

PinocchioInterface createWbmmPinocchioInterface(const std::string & robotUrdfPath)
{
  return createWbmmPinocchioInterface(robotUrdfPath, {});
}

WbmmModelInfo createWbmmModelInfo(
  const PinocchioInterface & interface,
  const std::string & baseFrame,
  const std::string & eeFrame,
  const std::string & eeFrame1)
{
  const auto & model = interface.getModel();

  constexpr std::size_t kStateDim = 9;
  constexpr std::size_t kInputDim = 8;
  constexpr std::size_t kArmDim = 6;
  if (static_cast<std::size_t>(model.nq) != kStateDim ||
    static_cast<std::size_t>(model.nv) != kStateDim)
  {
    throw std::invalid_argument(
            "[createWbmmModelInfo] expected a 9D planar-base plus six-joint "
            "model (nq=nv=9), got nq=" + std::to_string(model.nq) +
            ", nv=" + std::to_string(model.nv));
  }

  WbmmModelInfo info;
  info.stateDim = kStateDim;
  info.inputDim = kInputDim;
  info.armDim = kArmDim;
  info.baseFrame = baseFrame;
  info.eeFrame = eeFrame;
  info.eeFrame1 = eeFrame1;

  const auto & jointNames = model.names;
  info.dofNames = std::vector<std::string>(
    jointNames.end() - static_cast<std::ptrdiff_t>(info.armDim), jointNames.end());
  return info;
}

}  // namespace wbmm_ocs2
