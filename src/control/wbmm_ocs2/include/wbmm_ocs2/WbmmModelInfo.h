/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <cstddef>
#include <string>
#include <vector>

namespace wbmm_ocs2
{

// ============================================================================
// WbmmModelInfo —— WBMM 第一版移动机械臂 MPC 模型合同。
//
//   x = [base_x, base_y, base_yaw, q1..qN]   stateDim
//   u = [v, omega, qdot1..qdotN]             inputDim
//
// 当前只实现 WBMM 差速底盘 + 六轴臂：
//   stateDim = 9, inputDim = 8, armDim = 6。
//
// 不再保留 OCS2 mobile_manipulator 的 Default / FloatingArm /
// FullyActuatedFloatingArm 分支；出现第二种真实底盘需求时再单独引入，
// 禁止提前为假想模型抽象。
// ============================================================================
struct WbmmModelInfo
{
  std::size_t stateDim{0};
  std::size_t inputDim{0};
  std::size_t armDim{0};

  std::string baseFrame;   // URDF 根链路名（机械臂基座）
  std::string eeFrame;     // 末端 frame，单臂时即主末端
  std::string eeFrame1;    // 预留双末端兼容，单臂为空
  std::vector<std::string> dofNames;  // 驱动关节名，Pinocchio 顺序
};

// 旧 task.info 中 wheelBasedMobileManipulator 子树的键名。
// 迁移期保留该字符串以兼容现有配置；语义固定为差速底盘 + 机械臂。
inline constexpr const char * kWheelBasedModelKey = "wheelBasedMobileManipulator";

}  // namespace wbmm_ocs2
