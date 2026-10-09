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
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace wbmm_ocs2
{

// ============================================================================
// WbmmModelInfo —— WBMM 第一版移动机械臂 MPC 模型合同。
//
//   x = [base_x, base_y, base_yaw, q1..qN, (v_actual, omega_actual)]
//   u = [v, omega, qdot1..qdotN]             inputDim
//
// 当前只实现 WBMM 差速底盘 + 六轴臂：
//   stateDim = 9 (ideal base) or 11 (base response), inputDim = 8, armDim = 6。
// Pinocchio q and whole-body references always contain the first 9 coordinates.
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

  struct BaseResponse {
    bool enabled{false};
    double linearTimeConstant{0.35};
    double angularTimeConstant{0.60};
    double linearGain{0.94};
    double angularGain{1.31};
  } baseResponse;

  std::size_t configurationDim() const noexcept { return 3 + armDim; }
  std::size_t baseVelocityIndex() const noexcept { return configurationDim(); }

  void configureBaseResponse(BaseResponse settings) {
    for (const double value : {settings.linearTimeConstant, settings.angularTimeConstant,
                               settings.linearGain, settings.angularGain}) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("baseResponse gains/time constants must be finite and positive");
      }
    }
    baseResponse = settings;
    stateDim = configurationDim() + (settings.enabled ? 2 : 0);
  }

  std::string baseFrame;   // URDF 根链路名（机械臂基座）
  std::string eeFrame;     // 唯一末端 frame
  std::vector<std::string> dofNames;  // 驱动关节名，Pinocchio 顺序
};

// 旧 task.info 中 wheelBasedMobileManipulator 子树的键名。
// 迁移期保留该字符串以兼容现有配置；语义固定为差速底盘 + 机械臂。
inline constexpr const char * kWheelBasedModelKey = "wheelBasedMobileManipulator";

}  // namespace wbmm_ocs2
