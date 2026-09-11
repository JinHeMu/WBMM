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

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_sqp/SqpSettings.h>

#include <wbmm_ocs2/FactoryFunctions.h>
#include <wbmm_ocs2/collision/EnvironmentGeometryInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

namespace wbmm_ocs2
{
    /**
    * WBMM 轮式移动机械臂的 OCS2 问题定义入口。
    *
    * 负责组装：
    *   - 9D/8D 模型信息与 Pinocchio 映射；
    *   - 差速底盘 + 关节速度动力学；
    *   - 输入代价、全身轨迹跟踪代价；
    *   - 关节限位、末端、body-relative、自碰撞、环境碰撞约束；
    *   - rollout、initializer、ReferenceManager。
    *
    * 具体模型固定为 Tracer 差速底盘 + 六轴机械臂，见 WbmmModelInfo.h。
    */
    class WbmmInterface final : public ocs2::RobotInterface
    {
    public:
        /**
         * Constructor
         *
         * @note Creates directory for generated library into if it does not exist.
         * @throw Invalid argument error if input task file or urdf file does not exist.
         *
         * @param [in] taskFile: The absolute path to the configuration file for the MPC.
         * @param [in] libraryFolder: The absolute path to the directory to generate CppAD library into.
         * @param [in] urdfFile: The absolute path to the URDF file for the robot.
         */
        WbmmInterface(const std::string& taskFile, const std::string& libraryFolder,
                                   const std::string& urdfFile);

        const ocs2::vector_t& getInitialState() { return initialState_; }

        ocs2::ddp::Settings& ddpSettings() { return ddpSettings_; }

        ocs2::mpc::Settings& mpcSettings() { return mpcSettings_; }

        ocs2::sqp::Settings& sqpSettings() { return sqpSettings_; }

        const ocs2::OptimalControlProblem& getOptimalControlProblem() const override { return problem_; }

        std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const override
        {
            return referenceManagerPtr_;
        }

        const ocs2::Initializer& getInitializer() const override { return *initializerPtr_; }

        const ocs2::RolloutBase& getRollout() const { return *rolloutPtr_; }

        const ocs2::PinocchioInterface& getPinocchioInterface() const { return *pinocchioInterfacePtr_; }

        const WbmmModelInfo& getWbmmModelInfo() const { return modelInfo_; }

        // 获取自碰撞几何接口
        std::unique_ptr<ocs2::PinocchioGeometryInterface> getPinocchioGeometryInterface() const;

        // 获取自碰撞激活距离
        ocs2::scalar_t getSelfCollisionActivationDistance() const { return selfCollisionActivationDistance_; }

        // 获取自碰撞最小安全距离
        ocs2::scalar_t getSelfCollisionMinimumDistance() const { return selfCollisionMinimumDistance_; }

        // 获取自碰撞约束是否启用
        bool isSelfCollisionEnabled() const { return selfCollisionEnabled_; }

        // ========== 环境碰撞接口 ==========

        /**
         * @brief 获取环境几何接口（用于动态添加/移除障碍物）
         * @return 环境几何接口的共享指针，如果未启用则返回nullptr
         */
        std::shared_ptr<EnvironmentGeometryInterface> getEnvironmentGeometryInterface() const {
            return envGeomInterfacePtr_;
        }

        /**
         * @brief 获取环境碰撞最小安全距离
         */
        ocs2::scalar_t getEnvironmentCollisionMinimumDistance() const {
            return envCollisionMinimumDistance_;
        }

        /**
         * @brief 获取环境碰撞激活距离
         */
        ocs2::scalar_t getEnvironmentCollisionActivationDistance() const {
            return envCollisionActivationDistance_;
        }

        /**
         * @brief 检查环境碰撞约束是否启用
         */
        bool isEnvironmentCollisionEnabled() const {
            return envCollisionEnabled_;
        }

        bool dual_arm_ = false;

    private:
        std::unique_ptr<ocs2::StateInputCost> getQuadraticInputCost(const std::string& taskFile);
        std::unique_ptr<ocs2::StateCost> getEndEffectorConstraint(const ocs2::PinocchioInterface& pinocchioInterface,
                                                            const std::string& taskFile,
                                                            const std::string& prefix, bool useCaching,
                                                            const std::string& libraryFolder,
                                                            bool recompileLibraries);
        std::unique_ptr<ocs2::StateCost> getSelfCollisionConstraint(const ocs2::PinocchioInterface& pinocchioInterface,
                                                              const std::string& taskFile,
                                                              const std::string& urdfFile,
                                                              const std::string& prefix,
                                                              bool useCaching,
                                                              const std::string& libraryFolder,
                                                              bool recompileLibraries);
        std::unique_ptr<ocs2::StateCost> getBodyRelativeConstraint(const ocs2::PinocchioInterface& pinocchioInterface,
                                                             const std::string& taskFile,
                                                             const std::string& prefix,
                                                             bool usePreComputation,
                                                             const std::string& libraryFolder,
                                                             bool recompileLibraries);
        std::unique_ptr<ocs2::StateInputCost> getJointLimitSoftConstraint(const ocs2::PinocchioInterface& pinocchioInterface,
                                                                    const std::string& taskFile);
        std::unique_ptr<ocs2::StateCost> getEnvironmentCollisionConstraint(const ocs2::PinocchioInterface& pinocchioInterface,
                                                                     const std::string& taskFile,
                                                                     const std::string& prefix);


                /** 全身轨迹跟踪 cost, 参考来自 ROS 发布的 ocs2::TargetTrajectories */
        std::unique_ptr<ocs2::StateCost> getWholeBodyTrajectoryCost(const std::string& taskFile,
                                                            const std::string& prefix,
                                                            bool isFinal);

        void loadInitialObstacles(const std::string& taskFile, const std::string& prefix);

        ocs2::ddp::Settings ddpSettings_;
        ocs2::mpc::Settings mpcSettings_;
        ocs2::sqp::Settings sqpSettings_;

        ocs2::OptimalControlProblem problem_;
        std::shared_ptr<ocs2::ReferenceManager> referenceManagerPtr_;

        std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;
        std::unique_ptr<ocs2::Initializer> initializerPtr_;

        std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr_;
        WbmmModelInfo modelInfo_;

        // 自碰撞几何接口
        std::unique_ptr<ocs2::PinocchioGeometryInterface> pinocchioGeometryInterfacePtr_;

        // 自碰撞激活距离
        ocs2::scalar_t selfCollisionActivationDistance_ = 0.0;

        // 自碰撞最小安全距离
        ocs2::scalar_t selfCollisionMinimumDistance_ = 0.0;

        // 自碰撞约束是否启用
        bool selfCollisionEnabled_ = false;

        // 环境碰撞几何接口
        std::shared_ptr<EnvironmentGeometryInterface> envGeomInterfacePtr_;

        // 环境碰撞激活距离
        ocs2::scalar_t envCollisionActivationDistance_ = 0.0;

        // 环境碰撞最小安全距离
        ocs2::scalar_t envCollisionMinimumDistance_ = 0.0;

        // 环境碰撞约束是否启用
        bool envCollisionEnabled_ = false;

        ocs2::vector_t initialState_;

        bool endEffectorEnabled_{true};
        bool wholeBodyTrackingEnabled_{false};
    };
}
