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

#pragma once

#include <string>
#include <vector>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "wbmm_ocs2/WbmmModelInfo.h"

namespace wbmm_ocs2
{

// 从 URDF 创建 WBMM 轮式移动机械臂的 ocs2::PinocchioInterface：
//   1. 把 jointNames 中的关节改为 fixed（例如 URDF 中真实存在的轮子关节）；
//   2. 自动把 mimic 关节改为 fixed；
//   3. 在机械臂基座前插入 [PX, PY, RZ] 三个 planar 自由度，
//      得到 Pinocchio 配置 q = [x, y, yaw, q_arm]。
ocs2::PinocchioInterface createWbmmPinocchioInterface(
  const std::string & robotUrdfPath,
  const std::vector<std::string> & jointNames);

// 不做 removeJoints 的便捷重载。
ocs2::PinocchioInterface createWbmmPinocchioInterface(const std::string & robotUrdfPath);

// 从 ocs2::PinocchioInterface 解析 WBMM 模型维度与 frame 名：
//   stateDim = nq, inputDim = nq - 1, armDim = inputDim - 2。
WbmmModelInfo createWbmmModelInfo(
  const ocs2::PinocchioInterface & interface,
  const std::string & baseFrame,
  const std::string & eeFrame,
  const std::string & eeFrame1 = "");

}  // namespace wbmm_ocs2
