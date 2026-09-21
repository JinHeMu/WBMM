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

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>

#include "wbmm_environment/esdf_grid.hpp"

namespace wbmm_ocs2
{

/**
 * 将 Pinocchio 碰撞球近似与 ESDF 地图连接起来。
 *
 * 对每个碰撞球中心 p_i：
 *   - 查询 ESDF，得到 d_i = ESDF(p_i)
 *   - 查询 ESDF 梯度 g_i = d(d_i)/d(p_i)
 *
 * 净间隙由调用方计算：
 *   h_i = d_i - r_i - margin_i
 *
 * 梯度链式法则：
 *   dh_i/dx = J_{p_i}(x)^T g_i
 */
class EsdfEnvironmentInterface
{
public:
    struct DistanceResult
    {
        double distance{0.0};
        double radius{0.0};
        double minimumDistance{0.0};
        Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
        bool gradientValid{false};
        std::string linkName;
    };

    EsdfEnvironmentInterface(
        const ocs2::PinocchioInterface& pinocchioInterface,
        std::shared_ptr<const wbmm::environment::EsdfGrid> grid,
        const std::vector<std::string>& collisionLinks,
        const std::vector<ocs2::scalar_t>& maxExcesses,
        ocs2::scalar_t shrinkRatio,
        ocs2::scalar_t defaultMinimumDistance);

    ~EsdfEnvironmentInterface() = default;

    /**
     * 计算所有碰撞球中心处的 ESDF 距离与梯度。
     * @note 调用前 PinocchioInterface 必须已经执行 forwardKinematics 和
     *       updateFramePlacements。
     */
    std::vector<DistanceResult> computeDistances(
        const ocs2::PinocchioInterface& pinocchioInterface) const;

    std::size_t getNumSpheres() const;

    const ocs2::PinocchioSphereInterface& getSphereInterface() const;

    std::string getFrameId() const;

    ocs2::scalar_t getDefaultMinimumDistance() const;

private:
    std::shared_ptr<const wbmm::environment::EsdfGrid> grid_;
    ocs2::PinocchioSphereInterface sphereInterface_;
    std::vector<double> sphereRadii_;
    std::vector<std::string> sphereLinks_;
    double defaultMinimumDistance_{0.0};
    std::string frameId_;
};

}  // namespace wbmm_ocs2
