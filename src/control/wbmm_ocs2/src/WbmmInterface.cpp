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

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include "wbmm_ocs2/WbmmInterface.h"

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>

#include "wbmm_ocs2/WbmmModelInfo.h"
#include "wbmm_ocs2/PreComputation.h"
#include "wbmm_ocs2/constraint/SelfCollisionConstraint.h"
#include "wbmm_ocs2/constraint/EnvironmentCollisionConstraint.h"
#include "wbmm_ocs2/constraint/EsdfEnvironmentCollisionConstraint.h"
#include "wbmm_ocs2/collision/EsdfEnvironmentInterface.h"
#include <wbmm_environment/esdf_loader.hpp>
#include "wbmm_ocs2/cost/QuadraticInputCost.h"
#include "wbmm_ocs2/cost/WholeBodyTrajectoryCost.h"
#include "wbmm_ocs2/cost/EndEffectorTrackingCost.h"
#include "wbmm_ocs2/cost/ArmManipulabilityCost.h"
#include "wbmm_ocs2/cost/PhaseWeightedStateCost.h"
#include "wbmm_ocs2/Dynamics.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace wbmm_ocs2
{

    // 迁移期：原 OCS2 mobile_manipulator 代码位于 ocs2 命名空间内，
    // 这里用文件级 using-directive 保持上游类型的可见性，
    // 不污染被 include 的头文件。
    using namespace ocs2;

    namespace
    {

        std::array<scalar_t, 4> loadPhaseWeights(
            const std::string &taskFile,
            const boost::property_tree::ptree &pt,
            const std::string &prefix,
            const std::array<scalar_t, 4> &defaults)
        {
            if (!pt.get_child_optional(prefix + ".phaseWeights"))
            {
                return defaults;
            }

            std::array<scalar_t, 4> result = defaults;

            // Support both OCS2 matrix style:
            //   phaseWeights { (0,0) 1.0 (1,0) 0.5 ... }
            // and std-vector style:
            //   phaseWeights { [0] 1.0 [1] 0.5 ... }
            if (pt.get_child_optional(prefix + ".phaseWeights.(0,0)"))
            {
                vector_t weights = vector_t::Zero(4);
                weights << defaults[0], defaults[1], defaults[2], defaults[3];
                loadData::loadEigenMatrix(taskFile, prefix + ".phaseWeights", weights);
                for (std::size_t i = 0; i < result.size(); ++i)
                {
                    result[i] = weights(static_cast<Eigen::Index>(i));
                }
            }
            else
            {
                std::vector<scalar_t> weights;
                loadData::loadStdVector<scalar_t>(
                    taskFile, prefix + ".phaseWeights", weights, false);
                if (weights.size() != result.size())
                {
                    throw std::runtime_error(
                        "[WbmmInterface] " + prefix +
                        ".phaseWeights must contain exactly 4 values.");
                }
                for (std::size_t i = 0; i < result.size(); ++i)
                {
                    result[i] = weights[i];
                }
            }

            for (const scalar_t weight : result)
            {
                if (!std::isfinite(weight) || weight < 0.0)
                {
                    throw std::runtime_error(
                        "[WbmmInterface] " + prefix +
                        ".phaseWeights must be finite and non-negative.");
                }
            }
            return result;
        }

    } // namespace

    WbmmInterface::WbmmInterface(const std::string &taskFile,
                                 const std::string &libraryFolder,
                                 const std::string &urdfFile,
                                 const std::string &esdfFileOverride,
                                 const std::string &worldFrame)
    {
        esdfFileOverride_ = esdfFileOverride;
        worldFrame_ = worldFrame;

        // check that task file exists
        boost::filesystem::path taskFilePath(taskFile);
        if (boost::filesystem::exists(taskFilePath))
        {
            std::cerr << "[WbmmInterface] Loading task file: " << taskFilePath << std::endl;
        }
        else
        {
            throw std::invalid_argument(
                "[WbmmInterface] Task file not found: " + taskFilePath.string());
        }
        // check that urdf file exists
        boost::filesystem::path urdfFilePath(urdfFile);
        if (boost::filesystem::exists(urdfFilePath))
        {
            std::cerr << "[WbmmInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
        }
        else
        {
            throw std::invalid_argument(
                "[WbmmInterface] URDF file not found: " + urdfFilePath.string());
        }
        // create library folder if it does not exist
        boost::filesystem::path libraryFolderPath(libraryFolder);
        boost::filesystem::create_directories(libraryFolderPath);
        std::cerr << "[WbmmInterface] Generated library path: " << libraryFolderPath << std::endl;

        // read the task file
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        // WBMM 固定模型：差速底盘 + 机械臂。不再读取四模型分支配置。
        // 旧 task.info 中的 model_information.manipulatorModelType 字段被忽略。
        // read the joints to make fixed
        std::vector<std::string> removeJointNames;
        loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, false);
        // read the frame names
        std::string baseFrame, eeFrame;
        loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);
        loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", false);

        std::cerr << "\n #### Model Information:";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr
            << "\n #### model: wheelBasedMobileManipulator (fixed by wbmm_ocs2)";
        std::cerr << "\n #### model_information.removeJoints: ";
        for (const auto &name : removeJointNames)
        {
            std::cerr << "\"" << name << "\" ";
        }
        std::cerr << "\n #### Note: All mimic joints will be automatically detected and removed";
        std::cerr << "\n #### model_information.baseFrame: \"" << baseFrame << "\"";
        std::cerr << "\n #### model_information.eeFrame: \"" << eeFrame << "\"" << std::endl;
        std::cerr << " #### =============================================================================" << std::endl;

        // create pinocchio interface
        pinocchioInterfacePtr_ = std::make_unique<PinocchioInterface>(
            createWbmmPinocchioInterface(urdfFile, removeJointNames));
        std::cerr << *pinocchioInterfacePtr_;

        // WbmmModelInfo
        modelInfo_ = createWbmmModelInfo(
            *pinocchioInterfacePtr_, baseFrame, eeFrame);

        bool usePreComputation = true;
        bool recompileLibraries = true;
        std::cerr << "\n #### Model Settings:";
        std::cerr << "\n #### =============================================================================\n";
        loadData::loadPtreeValue(pt, usePreComputation, "model_settings.usePreComputation", true);
        loadData::loadPtreeValue(pt, recompileLibraries, "model_settings.recompileLibraries", true);
        std::cerr << " #### =============================================================================\n";

        // Default initial state
        initialState_.setZero(modelInfo_.stateDim);
        const int baseStateDim = modelInfo_.stateDim - modelInfo_.armDim;
        const int armStateDim = modelInfo_.armDim;

        // arm base DOFs initial state
        if (baseStateDim > 0)
        {
            vector_t initialBaseState = vector_t::Zero(baseStateDim);
            loadData::loadEigenMatrix(
                taskFile, std::string("initialState.base.") + kWheelBasedModelKey,
                initialBaseState);
            initialState_.head(baseStateDim) = initialBaseState;
        }

        // arm joints DOFs velocity limits
        vector_t initialArmState = vector_t::Zero(armStateDim);
        loadData::loadEigenMatrix(taskFile, "initialState.arm", initialArmState);
        initialState_.tail(armStateDim) = initialArmState;

        std::cerr << "Initial State:   " << initialState_.transpose() << std::endl;

        // DDP-MPC settings
        ddpSettings_ = ddp::loadSettings(taskFile, "ddp");
        mpcSettings_ = mpc::loadSettings(taskFile, "mpc");

        // SQP settings (optional, will use defaults if not present)
        try
        {
            sqpSettings_ = sqp::loadSettings(taskFile, "sqp");
        }
        catch (const std::exception &e)
        {
            std::cerr << " #### SQP settings not found in task file, using defaults.\n";
        }

        // Reference Manager
        modeSwitchEnabled_ = false;
        loadData::loadPtreeValue(
            pt, modeSwitchEnabled_, "modeSwitch.activate", false);

        int initialPhaseValue = 0;
        loadData::loadPtreeValue(
            pt, initialPhaseValue, "modeSwitch.initialPhase", false);
        if (initialPhaseValue < 0 || initialPhaseValue > 3)
        {
            throw std::runtime_error(
                "[WbmmInterface] modeSwitch.initialPhase must be in [0, 3].");
        }

        // 双参考管理器需要在 generic setTargetTrajectories() 中按维度路由：
        //   whole-body target = modelInfo_.stateDim (9D)
        //   end-effector target = 7D [x,y,z,qx,qy,qz,qw]
        wbmmRefManagerPtr_ = std::make_shared<WbmmReferenceManager>(
            static_cast<TaskPhase>(initialPhaseValue),
            modelInfo_.stateDim, static_cast<std::size_t>(7));
        referenceManagerPtr_ = wbmmRefManagerPtr_;

        /*
         * Optimal control problem
         */
        // Cost
        problem_.costPtr->add("inputCost", getQuadraticInputCost(taskFile));

        // Constraints
        // joint limits constraint
        problem_.softConstraintPtr->add(
            "jointLimits",
            getJointLimitSoftConstraint(*pinocchioInterfacePtr_, taskFile));

        if (modeSwitchEnabled_)
        {
            std::cerr << "\n #### Dual-reference phase switching is ENABLED.\n";
            std::cerr << " #### wholeBodyTracking and endEffectorTracking are both "
                         "registered and weighted by task phase.\n";

            const auto wholeBodyWeights = loadPhaseWeights(
                taskFile, pt, "wholeBodyTracking",
                std::array<scalar_t, 4>{1.0, 0.5, 0.0, 0.5});
            const auto endEffectorWeights = loadPhaseWeights(
                taskFile, pt, "endEffectorTracking",
                std::array<scalar_t, 4>{0.0, 0.5, 1.0, 0.0});

            problem_.stateCostPtr->add(
                "wholeBodyTracking",
                std::make_unique<PhaseWeightedStateCost>(
                    getWholeBodyTrajectoryCost(
                        taskFile, "wholeBodyTracking", false),
                    wbmmRefManagerPtr_, TargetKind::kWholeBody,
                    wholeBodyWeights));

            problem_.stateCostPtr->add(
                "endEffectorTracking",
                std::make_unique<PhaseWeightedStateCost>(
                    getEndEffectorTrackingCost(
                        *pinocchioInterfacePtr_, taskFile,
                        "endEffectorTracking", usePreComputation,
                        libraryFolder, recompileLibraries),
                    wbmmRefManagerPtr_, TargetKind::kEndEffector,
                    endEffectorWeights));

            problem_.finalCostPtr->add(
                "finalWholeBodyTracking",
                std::make_unique<PhaseWeightedStateCost>(
                    getWholeBodyTrajectoryCost(
                        taskFile, "wholeBodyTracking", true),
                    wbmmRefManagerPtr_, TargetKind::kWholeBody,
                    wholeBodyWeights));

            problem_.finalCostPtr->add(
                "finalEndEffectorTracking",
                std::make_unique<PhaseWeightedStateCost>(
                    getEndEffectorTrackingCost(
                        *pinocchioInterfacePtr_, taskFile,
                        "endEffectorTracking", usePreComputation,
                        libraryFolder, recompileLibraries),
                    wbmmRefManagerPtr_, TargetKind::kEndEffector,
                    endEffectorWeights));

            wholeBodyTrackingEnabled_ = true;
        }
        else
        {
            // ------------------------------------------------------------------
            // whole-body trajectory tracking cost
            //   L(x,t) = 0.5 * (x - x_d(t))' Q (x - x_d(t))
            //   x_d(t) 由 ROS 发布的 TargetTrajectories 提供 (stateDim 维)
            // ------------------------------------------------------------------
            wholeBodyTrackingEnabled_ = false;
            loadData::loadPtreeValue(
                pt, wholeBodyTrackingEnabled_, "wholeBodyTracking.activate",
                true);
            if (wholeBodyTrackingEnabled_)
            {
                problem_.stateCostPtr->add(
                    "wholeBodyTracking",
                    getWholeBodyTrajectoryCost(
                        taskFile, "wholeBodyTracking", false));

                problem_.finalCostPtr->add(
                    "finalWholeBodyTracking",
                    getWholeBodyTrajectoryCost(
                        taskFile, "wholeBodyTracking", true));
            }
        }

        // Reusable kinematic metrics are computed by wbmm_robot_metrics;
        // this term only maps them into an OCS2 state cost. Keep disabled by
        // default so existing deployments do not change before weight tuning.
        loadData::loadPtreeValue(
            pt, armManipulabilityEnabled_,
            "armManipulability.activate", false);
        if (armManipulabilityEnabled_)
        {
            problem_.stateCostPtr->add(
                "armManipulability",
                getArmManipulabilityCost(
                    *pinocchioInterfacePtr_, taskFile,
                    "armManipulability"));
        }

        // self-collision avoidance constraint
        selfCollisionEnabled_ = true;
        loadData::loadPtreeValue(pt, selfCollisionEnabled_, "selfCollision.activate", true);
        if (selfCollisionEnabled_)
        {
            problem_.stateSoftConstraintPtr->add(
                "selfCollision", getSelfCollisionConstraint(*pinocchioInterfacePtr_, taskFile, urdfFile,
                                                            "selfCollision", usePreComputation,
                                                            libraryFolder, recompileLibraries));
        }

        // environment collision avoidance constraint
        envCollisionEnabled_ = false;
        loadData::loadPtreeValue(pt, envCollisionEnabled_, "environmentCollision.activate", false);
        if (envCollisionEnabled_)
        {
            problem_.stateSoftConstraintPtr->add(
                "environmentCollision", getEnvironmentCollisionConstraint(*pinocchioInterfacePtr_, taskFile,
                                                                          "environmentCollision"));
        }

        // Dynamics：WBMM 固定为轮式移动机械臂。
        problem_.dynamicsPtr = std::make_unique<WbmmDynamics>(
            modelInfo_, "dynamics", libraryFolder,
            recompileLibraries, true);

        /*
         * Pre-computation
         */
        if (usePreComputation)
        {
            problem_.preComputationPtr = std::make_unique<WbmmPreComputation>(
                *pinocchioInterfacePtr_, modelInfo_);
        }

        // Rollout
        const auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
        rolloutPtr_ = std::make_unique<TimeTriggeredRollout>(*problem_.dynamicsPtr, rolloutSettings);

        // Initialization
        initializerPtr_ = std::make_unique<DefaultInitializer>(modelInfo_.inputDim);
    }

    std::unique_ptr<StateInputCost> WbmmInterface::getQuadraticInputCost(const std::string &taskFile)
    {
        matrix_t R = matrix_t::Zero(modelInfo_.inputDim, modelInfo_.inputDim);
        const int baseInputDim = modelInfo_.inputDim - modelInfo_.armDim;
        const int armStateDim = modelInfo_.armDim;

        // arm base DOFs input costs
        if (baseInputDim > 0)
        {
            matrix_t R_base = matrix_t::Zero(baseInputDim, baseInputDim);
            loadData::loadEigenMatrix(
                taskFile, std::string("inputCost.R.base.") + kWheelBasedModelKey,
                R_base);
            R.topLeftCorner(baseInputDim, baseInputDim) = R_base;
        }

        // arm joints DOFs input costs
        matrix_t R_arm = matrix_t::Zero(armStateDim, armStateDim);
        loadData::loadEigenMatrix(taskFile, "inputCost.R.arm", R_arm);
        R.bottomRightCorner(armStateDim, armStateDim) = R_arm;

        std::cerr << "\n #### Input Cost Settings: ";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << "inputCost.R:  \n"
                  << R << '\n';
        std::cerr << " #### =============================================================================\n";

        return std::make_unique<QuadraticInputCost>(std::move(R), modelInfo_.stateDim);
    }

    std::unique_ptr<StateCost> WbmmInterface::getWholeBodyTrajectoryCost(
        const std::string &taskFile, const std::string &prefix, bool isFinal)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        const int stateDim = modelInfo_.stateDim;
        const int armStateDim = modelInfo_.armDim;
        const int baseStateDim = stateDim - armStateDim;

        matrix_t Q = matrix_t::Zero(stateDim, stateDim);

        // base 部分 (wheelBased: [x, y, yaw])
        if (baseStateDim > 0)
        {
            matrix_t Q_base = matrix_t::Zero(baseStateDim, baseStateDim);
            loadData::loadEigenMatrix(taskFile, prefix + ".Q.base", Q_base);
            Q.topLeftCorner(baseStateDim, baseStateDim) = Q_base;
        }

        // arm 部分 (q1..qN)
        matrix_t Q_arm = matrix_t::Zero(armStateDim, armStateDim);
        loadData::loadEigenMatrix(taskFile, prefix + ".Q.arm", Q_arm);
        Q.bottomRightCorner(armStateDim, armStateDim) = Q_arm;

        // 终端权重缩放 (默认与中间时刻相同)
        scalar_t finalWeightScale = 1.0;
        loadData::loadPtreeValue(pt, finalWeightScale, prefix + ".finalWeightScale", false);
        if (isFinal)
        {
            Q *= finalWeightScale;
        }

        // 轮式模型 state = [x, y, yaw, q...]，yaw 固定在索引 2。
        const int yawIndex = 2;

        std::cerr << "\n #### " << prefix << (isFinal ? " (final)" : " (intermediate)")
                  << " Settings: ";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << " #### stateDim : " << stateDim << "  (base " << baseStateDim
                  << " + arm " << armStateDim << ")\n";
        std::cerr << " #### yawIndex : " << yawIndex << '\n';
        std::cerr << " #### Q:\n"
                  << Q << '\n';
        std::cerr << " #### 参考轨迹来源: ROS topic <robot>_mpc_target (TargetTrajectories, "
                  << stateDim << " 维 state)\n";
        std::cerr << " #### =============================================================================\n";

        // TargetTrajectories 为空时的兜底参考 = initialState (保持不动)
        return std::make_unique<WholeBodyTrajectoryCost>(std::move(Q), yawIndex, initialState_);
    }

    std::unique_ptr<StateCost> WbmmInterface::getEndEffectorTrackingCost(
        const PinocchioInterface &pinocchioInterface,
        const std::string &taskFile,
        const std::string &prefix,
        bool usePreComputation,
        const std::string &libraryFolder,
        bool recompileLibraries)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        matrix_t positionWeight = matrix_t::Identity(3, 3);
        matrix_t orientationWeight = matrix_t::Identity(3, 3);
        if (pt.get_child_optional(prefix + ".Q.position"))
        {
            loadData::loadEigenMatrix(
                taskFile, prefix + ".Q.position", positionWeight);
        }
        if (pt.get_child_optional(prefix + ".Q.orientation"))
        {
            loadData::loadEigenMatrix(
                taskFile, prefix + ".Q.orientation", orientationWeight);
        }

        std::cerr << "\n #### " << prefix << " Settings: ";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << " #### Q.position:\n"
                  << positionWeight << '\n';
        std::cerr << " #### Q.orientation:\n"
                  << orientationWeight << '\n';
        std::cerr << " #### =============================================================================\n";

        if (usePreComputation)
        {
            WbmmPinocchioMapping pinocchioMapping(modelInfo_);
            PinocchioEndEffectorKinematics eeKinematics(
                pinocchioInterface, pinocchioMapping, {modelInfo_.eeFrame});
            return std::make_unique<EndEffectorTrackingCost>(
                eeKinematics, std::move(positionWeight),
                std::move(orientationWeight));
        }

        WbmmPinocchioMappingCppAd pinocchioMappingCppAd(modelInfo_);
        PinocchioEndEffectorKinematicsCppAd eeKinematics(
            pinocchioInterface, pinocchioMappingCppAd, {modelInfo_.eeFrame},
            modelInfo_.stateDim, modelInfo_.inputDim,
            "end_effector_tracking_kinematics", libraryFolder,
            recompileLibraries, false);
        return std::make_unique<EndEffectorTrackingCost>(
            eeKinematics, std::move(positionWeight),
            std::move(orientationWeight));
    }

    std::unique_ptr<StateCost> WbmmInterface::getArmManipulabilityCost(
        const PinocchioInterface &pinocchioInterface,
        const std::string &taskFile,
        const std::string &prefix)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        ArmManipulabilitySettings settings;
        loadData::loadPtreeValue(
            pt, settings.frameName, prefix + ".frameName", false);
        loadData::loadPtreeValue(
            pt, settings.stateFrame, prefix + ".stateFrame", false);

        std::string scope = "arm";
        std::string task = "pose";
        std::string scaling = "raw";
        loadData::loadPtreeValue(
            pt, scope, prefix + ".metrics.scope", false);
        loadData::loadPtreeValue(
            pt, task, prefix + ".metrics.task", false);
        loadData::loadPtreeValue(
            pt, scaling, prefix + ".metrics.scaling", false);

        if (scope == "arm")
        {
            settings.metricsOptions.scope =
                wbmm::metrics::JacobianScope::kArmColumns;
        }
        else if (scope == "whole_body")
        {
            settings.metricsOptions.scope =
                wbmm::metrics::JacobianScope::kWholeBodyInput;
        }
        else
        {
            throw std::runtime_error(
                "[WbmmInterface] " + prefix +
                ".metrics.scope must be 'arm' or 'whole_body'.");
        }

        if (task == "translation")
        {
            settings.metricsOptions.task =
                wbmm::metrics::JacobianTask::kTranslation;
        }
        else if (task == "pose")
        {
            settings.metricsOptions.task = wbmm::metrics::JacobianTask::kPose;
        }
        else
        {
            throw std::runtime_error(
                "[WbmmInterface] " + prefix +
                ".metrics.task must be 'translation' or 'pose'.");
        }

        if (scaling == "raw")
        {
            settings.metricsOptions.scaling =
                wbmm::metrics::JacobianScaling::kRaw;
        }
        else if (scaling == "characteristic_length")
        {
            settings.metricsOptions.scaling =
                wbmm::metrics::JacobianScaling::kCharacteristicLength;
        }
        else
        {
            throw std::runtime_error(
                "[WbmmInterface] " + prefix +
                ".metrics.scaling must be 'raw' or 'characteristic_length'.");
        }

        loadData::loadPtreeValue(
            pt, settings.metricsOptions.characteristic_length,
            prefix + ".metrics.characteristicLength", false);
        loadData::loadPtreeValue(
            pt, settings.metricsOptions.regularization,
            prefix + ".metrics.regularization", false);
        loadData::loadPtreeValue(
            pt, settings.metricsOptions.singular_value_floor,
            prefix + ".metrics.singularValueFloor", false);
        loadData::loadPtreeValue(
            pt, settings.metricsOptions.use_task_direction,
            prefix + ".metrics.useTaskDirection", false);

        if (settings.metricsOptions.use_task_direction)
        {
            const Eigen::Index taskRows =
                settings.metricsOptions.task ==
                        wbmm::metrics::JacobianTask::kTranslation
                    ? 3
                    : 6;
            settings.metricsOptions.task_direction = vector_t::Zero(taskRows);
            if (!pt.get_child_optional(prefix + ".metrics.taskDirection"))
            {
                throw std::runtime_error(
                    "[WbmmInterface] " + prefix +
                    ".metrics.taskDirection is required when enabled.");
            }
            loadData::loadEigenMatrix(
                taskFile, prefix + ".metrics.taskDirection",
                settings.metricsOptions.task_direction);
        }

        loadData::loadPtreeValue(
            pt, settings.useMinSingularValue,
            prefix + ".useMinSingularValue", false);
        loadData::loadPtreeValue(
            pt, settings.minSingularWeight,
            prefix + ".minSingularWeight", false);
        loadData::loadPtreeValue(
            pt, settings.minSingularRef,
            prefix + ".minSingularRef", false);
        loadData::loadPtreeValue(
            pt, settings.useYoshikawa,
            prefix + ".useYoshikawa", false);
        loadData::loadPtreeValue(
            pt, settings.yoshikawaWeight,
            prefix + ".yoshikawaWeight", false);
        loadData::loadPtreeValue(
            pt, settings.yoshikawaRef,
            prefix + ".yoshikawaRef", false);
        loadData::loadPtreeValue(
            pt, settings.taskDirectionWeight,
            prefix + ".taskDirectionWeight", false);
        loadData::loadPtreeValue(
            pt, settings.taskDirectionRef,
            prefix + ".taskDirectionRef", false);
        loadData::loadPtreeValue(
            pt, settings.useInverseManipulability,
            prefix + ".useInverseManipulability", false);
        loadData::loadPtreeValue(
            pt, settings.inverseManipulabilityWeight,
            prefix + ".inverseManipulabilityWeight", false);
        loadData::loadPtreeValue(
            pt, settings.useConditionNumber,
            prefix + ".useConditionNumber", false);
        loadData::loadPtreeValue(
            pt, settings.conditionWeight,
            prefix + ".conditionWeight", false);
        loadData::loadPtreeValue(
            pt, settings.conditionMax,
            prefix + ".conditionMax", false);
        loadData::loadPtreeValue(
            pt, settings.finiteDiffStep,
            prefix + ".finiteDiffStep", false);
        loadData::loadPtreeValue(
            pt, settings.hessianRegularization,
            prefix + ".hessianRegularization", false);
        loadData::loadPtreeValue(
            pt, settings.invalidMetricsPenalty,
            prefix + ".invalidMetricsPenalty", false);

        std::cerr << "\n #### Arm Manipulability Settings:";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << " #### frameName: "
                  << (settings.frameName.empty() ? modelInfo_.eeFrame
                                                 : settings.frameName)
                  << '\n';
        std::cerr << " #### stateFrame: " << settings.stateFrame << '\n';
        std::cerr << " #### metrics.scope/task/scaling: "
                  << scope << " / " << task << " / " << scaling << '\n';
        std::cerr << " #### =============================================================================\n";

        return std::make_unique<ArmManipulabilityCost>(
            pinocchioInterface, modelInfo_, std::move(settings));
    }

    std::unique_ptr<StateCost> WbmmInterface::getSelfCollisionConstraint(
        const PinocchioInterface &pinocchioInterface,
        const std::string &taskFile, const std::string &urdfFile,
        const std::string &prefix, bool usePreComputation,
        const std::string &libraryFolder,
        bool recompileLibraries)
    {
        std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
        std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
        scalar_t mu = 1e-2;
        scalar_t delta = 1e-3;
        scalar_t minimumDistance = 0.0;
        scalar_t activationDistance = -1.0; // -1 means use default (5 * minimumDistance)

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        std::cerr << "\n #### SelfCollision Settings: ";
        std::cerr << "\n #### =============================================================================\n";
        loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
        loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
        loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", true);
        loadData::loadPtreeValue(pt, activationDistance, prefix + ".activationDistance", false);
        loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, true);
        loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, true);

        // If activationDistance not specified, default to 5 * minimumDistance
        if (activationDistance < 0.0)
        {
            activationDistance = 5.0 * minimumDistance;
        }
        // Store distances for visualization and collision detection
        selfCollisionMinimumDistance_ = minimumDistance;
        selfCollisionActivationDistance_ = activationDistance;
        std::cerr << " #### minimumDistance: " << minimumDistance << " (minimum allowed distance)\n";
        std::cerr << " #### activationDistance: " << activationDistance << " (penalty only active when distance < this value)\n";
        std::cerr << " #### =============================================================================\n";

        // Create geometry interface (also stored for environment collision to reuse)
        pinocchioGeometryInterfacePtr_ = std::make_unique<PinocchioGeometryInterface>(
            pinocchioInterface, urdfFile, collisionLinkPairs, collisionObjectPairs);

        const size_t numCollisionPairs = pinocchioGeometryInterfacePtr_->getNumCollisionPairs();
        std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";

        std::unique_ptr<StateConstraint> constraint;
        if (usePreComputation)
        {
            PinocchioGeometryInterface geometryInterfaceCopy(*pinocchioGeometryInterfacePtr_);
            constraint = std::make_unique<WbmmSelfCollisionConstraint>(
                WbmmPinocchioMapping(modelInfo_),
                std::move(geometryInterfaceCopy), minimumDistance);
        }
        else
        {
            PinocchioGeometryInterface geometryInterfaceCopy(*pinocchioGeometryInterfacePtr_);
            constraint = std::make_unique<SelfCollisionConstraintCppAd>(
                pinocchioInterface, WbmmPinocchioMapping(modelInfo_),
                std::move(geometryInterfaceCopy), minimumDistance,
                "self_collision", libraryFolder, recompileLibraries, false);
        }

        // Use ThresholdRelaxedBarrierPenalty with activation distance
        // The activationThreshold in penalty space is (activationDistance - minimumDistance)
        // because constraint value h = actual_distance - minimumDistance
        const scalar_t activationThreshold = activationDistance - minimumDistance;
        auto penalty = std::make_unique<ThresholdRelaxedBarrierPenalty>(
            ThresholdRelaxedBarrierPenalty::Config{mu, delta, activationThreshold});

        return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
    }

    std::unique_ptr<StateInputCost> WbmmInterface::getJointLimitSoftConstraint(
        const PinocchioInterface &pinocchioInterface,
        const std::string &taskFile)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        bool activateJointPositionLimit = true;
        loadData::loadPtreeValue(pt, activateJointPositionLimit, "jointPositionLimits.activate", true);

        const int baseStateDim = modelInfo_.stateDim - modelInfo_.armDim;
        const int armStateDim = modelInfo_.armDim;
        const int baseInputDim = modelInfo_.inputDim - modelInfo_.armDim;
        const int armInputDim = modelInfo_.armDim;
        const auto &model = pinocchioInterface.getModel();

        // Load position limits
        std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
        if (activateJointPositionLimit)
        {
            scalar_t muPositionLimits = 1e-2;
            scalar_t deltaPositionLimits = 1e-3;

            // arm joint DOF limits from the parsed URDF
            const vector_t lowerBound = model.lowerPositionLimit.tail(armStateDim);
            const vector_t upperBound = model.upperPositionLimit.tail(armStateDim);

            std::cerr << "\n #### JointPositionLimits Settings: ";
            std::cerr << "\n #### =============================================================================\n";
            std::cerr << " #### lowerBound: " << lowerBound.transpose() << '\n';
            std::cerr << " #### upperBound: " << upperBound.transpose() << '\n';
            loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", true);
            loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", true);
            std::cerr << " #### =============================================================================\n";

            stateLimits.reserve(armStateDim);
            for (int i = 0; i < armStateDim; ++i)
            {
                StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
                boxConstraint.index = baseStateDim + i;
                boxConstraint.lowerBound = lowerBound(i);
                boxConstraint.upperBound = upperBound(i);
                boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
                stateLimits.push_back(std::move(boxConstraint));
            }
        }

        // load velocity limits
        std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
        {
            vector_t lowerBound = vector_t::Zero(modelInfo_.inputDim);
            vector_t upperBound = vector_t::Zero(modelInfo_.inputDim);
            scalar_t muVelocityLimits = 1e-2;
            scalar_t deltaVelocityLimits = 1e-3;

            // Base DOFs velocity limits
            if (baseInputDim > 0)
            {
                vector_t lowerBoundBase = vector_t::Zero(baseInputDim);
                vector_t upperBoundBase = vector_t::Zero(baseInputDim);
                loadData::loadEigenMatrix(
                    taskFile,
                    std::string("jointVelocityLimits.lowerBound.base.") + kWheelBasedModelKey,
                    lowerBoundBase);
                loadData::loadEigenMatrix(
                    taskFile,
                    std::string("jointVelocityLimits.upperBound.base.") + kWheelBasedModelKey,
                    upperBoundBase);
                lowerBound.head(baseInputDim) = lowerBoundBase;
                upperBound.head(baseInputDim) = upperBoundBase;
            }

            // arm joint DOFs velocity limits
            vector_t lowerBoundArm = vector_t::Zero(armInputDim);
            vector_t upperBoundArm = vector_t::Zero(armInputDim);
            loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound.arm", lowerBoundArm);
            loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound.arm", upperBoundArm);
            lowerBound.tail(armInputDim) = lowerBoundArm;
            upperBound.tail(armInputDim) = upperBoundArm;

            std::cerr << "\n #### JointVelocityLimits Settings: ";
            std::cerr << "\n #### =============================================================================\n";
            std::cerr << " #### 'lowerBound':  " << lowerBound.transpose() << std::endl;
            std::cerr << " #### 'upperBound':  " << upperBound.transpose() << std::endl;
            loadData::loadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", true);
            loadData::loadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", true);
            std::cerr << " #### =============================================================================\n";

            inputLimits.reserve(modelInfo_.inputDim);
            for (std::size_t i = 0; i < modelInfo_.inputDim; ++i)
            {
                StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
                boxConstraint.index = i;
                boxConstraint.lowerBound = lowerBound(i);
                boxConstraint.upperBound = upperBound(i);
                boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muVelocityLimits, deltaVelocityLimits}));
                inputLimits.push_back(std::move(boxConstraint));
            }
        }

        auto boxConstraints = std::make_unique<StateInputSoftBoxConstraint>(stateLimits, inputLimits);
        boxConstraints->initializeOffset(0.0, vector_t::Zero(modelInfo_.stateDim),
                                         vector_t::Zero(modelInfo_.stateDim));
        return boxConstraints;
    }

    std::unique_ptr<PinocchioGeometryInterface> WbmmInterface::getPinocchioGeometryInterface() const
    {
        if (pinocchioGeometryInterfacePtr_)
        {
            // 返回一个副本，因为原始指针是私有的
            return std::make_unique<PinocchioGeometryInterface>(*pinocchioGeometryInterfacePtr_);
        }
        return nullptr;
    }

    std::unique_ptr<StateCost> WbmmInterface::getEnvironmentCollisionConstraint(
        const PinocchioInterface &pinocchioInterface,
        const std::string &taskFile,
        const std::string &prefix)
    {
        std::vector<std::string> collisionLinks;
        scalar_t mu = 1e-2;
        scalar_t delta = 1e-3;
        scalar_t minimumDistance = 0.0;
        scalar_t activationDistance = -1.0;
        std::string backend = "geometry";

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        std::cerr << "\n #### EnvironmentCollision Settings: ";
        std::cerr << "\n #### =============================================================================\n";
        loadData::loadPtreeValue(pt, backend, prefix + ".backend", false);
        loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
        loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
        loadData::loadPtreeValue(
            pt, minimumDistance, prefix + ".minimumDistance", true);
        loadData::loadPtreeValue(
            pt, activationDistance, prefix + ".activationDistance", false);
        loadData::loadStdVector<std::string>(
            taskFile, prefix + ".collisionLinks", collisionLinks, true);

        if (activationDistance < 0.0)
        {
            activationDistance = 5.0 * minimumDistance;
        }

        envCollisionMinimumDistance_ = minimumDistance;
        envCollisionActivationDistance_ = activationDistance;

        std::cerr << " #### backend: " << backend << '\n';
        std::cerr << " #### minimumDistance: " << minimumDistance
                  << " (minimum allowed distance)\n";
        std::cerr << " #### activationDistance: " << activationDistance
                  << " (penalty only active when distance < this value)\n";
        std::cerr << " #### collisionLinks: [";
        for (std::size_t i = 0; i < collisionLinks.size(); ++i)
        {
            std::cerr << collisionLinks[i];
            if (i + 1U < collisionLinks.size())
                std::cerr << ", ";
        }
        std::cerr << "]\n";
        std::cerr << " #### =============================================================================\n";

        const scalar_t activationThreshold =
            activationDistance - minimumDistance;

        if (backend == "esdf")
        {
            if (collisionLinks.empty())
            {
                throw std::runtime_error(
                    "[EnvironmentCollision] esdf backend requires at least "
                    "one collisionLink.");
            }

            std::string esdfFile;
            std::string esdfFrame;
            loadData::loadPtreeValue(
                pt, esdfFile, prefix + ".esdf.file", false);
            loadData::loadPtreeValue(
                pt, esdfFrame, prefix + ".esdf.frame", false);
            if (!esdfFileOverride_.empty()) {
                esdfFile = esdfFileOverride_;
            }
            if (esdfFile.empty())
            {
                throw std::runtime_error(
                    "[EnvironmentCollision] esdf backend requires "
                    "environmentCollision.esdf.file or an esdf_file override.");
            }

            const auto loadResult =
                wbmm::environment::NpzEsdfLoader::load(esdfFile);
            if (loadResult.status !=
                    wbmm::environment::LoadStatus::kSuccess ||
                loadResult.grid == nullptr)
            {
                throw std::runtime_error(
                    "[EnvironmentCollision] Failed to load ESDF: " +
                    loadResult.message);
            }

            const std::string gridFrame = loadResult.grid->info().frame_id;
            if (!esdfFrame.empty() && esdfFrame != gridFrame)
            {
                throw std::runtime_error(
                    "[EnvironmentCollision] Configured ESDF frame '" +
                    esdfFrame + "' does not match NPZ frame '" +
                    gridFrame + "'.");
            }
            if (!worldFrame_.empty() && gridFrame != worldFrame_)
            {
                throw std::runtime_error(
                    "[EnvironmentCollision] ESDF frame '" + gridFrame +
                    "' does not match OCS2 world_frame '" + worldFrame_ +
                    "'; refusing to query an ESDF in a different frame. "
                    "No implicit TF conversion is performed.");
            }

            scalar_t maxExcess = 0.01;
            scalar_t shrinkRatio = 0.8;
            loadData::loadPtreeValue(
                pt, maxExcess, prefix + ".maxExcess", false);
            loadData::loadPtreeValue(
                pt, shrinkRatio, prefix + ".shrinkRatio", false);
            if (!(maxExcess > 0.0) || !std::isfinite(maxExcess))
            {
                throw std::runtime_error(
                    "[EnvironmentCollision] maxExcess must be positive and "
                    "finite.");
            }

            std::vector<scalar_t> maxExcesses(
                collisionLinks.size(), maxExcess);
            esdfEnvInterfacePtr_ = std::make_shared<EsdfEnvironmentInterface>(
                pinocchioInterface, loadResult.grid, collisionLinks,
                maxExcesses, shrinkRatio, minimumDistance);

            auto constraint =
                std::make_unique<EsdfEnvironmentCollisionConstraint>(
                    WbmmPinocchioMapping(modelInfo_), esdfEnvInterfacePtr_,
                    minimumDistance);

            auto penalty = std::make_unique<ThresholdRelaxedBarrierPenalty>(
                ThresholdRelaxedBarrierPenalty::Config{
                    mu, delta, activationThreshold});

            return std::make_unique<StateSoftConstraint>(
                std::move(constraint), std::move(penalty));
        }

        // Legacy coal/FCL backend.
        if (!pinocchioGeometryInterfacePtr_)
        {
            throw std::runtime_error(
                "[EnvironmentCollision] geometry backend requires "
                "selfCollision to be enabled first!");
        }

        envGeomInterfacePtr_ = std::make_shared<EnvironmentGeometryInterface>(
            *pinocchioGeometryInterfacePtr_, pinocchioInterface, collisionLinks);

        loadInitialObstacles(taskFile, prefix);

        auto constraint = std::make_unique<WbmmEnvironmentCollisionConstraint>(
            WbmmPinocchioMapping(modelInfo_), envGeomInterfacePtr_,
            minimumDistance);

        auto penalty = std::make_unique<ThresholdRelaxedBarrierPenalty>(
            ThresholdRelaxedBarrierPenalty::Config{
                mu, delta, activationThreshold});

        return std::make_unique<StateSoftConstraint>(
            std::move(constraint), std::move(penalty));
    }

    void WbmmInterface::loadInitialObstacles(const std::string &taskFile, const std::string &prefix)
    {
        if (!envGeomInterfacePtr_)
        {
            return;
        }

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        // Try to get the obstacles subtree
        const std::string obstaclesKey = prefix + ".obstacles";
        auto obstaclesOpt = pt.get_child_optional(obstaclesKey);
        if (!obstaclesOpt)
        {
            std::cerr << " #### No initial obstacles configured.\n";
            return;
        }

        std::cerr << " #### Loading initial obstacles:\n";
        int obstacleCount = 0;

        for (const auto &obstaclePair : obstaclesOpt.get())
        {
            const std::string &obstacleName = obstaclePair.first;
            const auto &obstacleNode = obstaclePair.second;

            // Get obstacle type
            std::string type = obstacleNode.get<std::string>("type", "");
            if (type.empty())
            {
                std::cerr << " ####   Warning: obstacle '" << obstacleName << "' has no type, skipping.\n";
                continue;
            }

            // Get position (required)
            vector_t position = vector_t::Zero(3);
            auto posOpt = obstacleNode.get_child_optional("position");
            if (posOpt)
            {
                int idx = 0;
                for (const auto &val : posOpt.get())
                {
                    if (idx < 3)
                        position(idx++) = std::stod(val.second.data());
                }
            }

            // Get orientation (optional, default identity)
            Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
            auto orientOpt = obstacleNode.get_child_optional("orientation");
            if (orientOpt)
            {
                std::vector<double> quat;
                for (const auto &val : orientOpt.get())
                {
                    quat.push_back(std::stod(val.second.data()));
                }
                if (quat.size() == 4)
                {
                    orientation = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]); // w, x, y, z
                }
            }

            // Get per-obstacle minimumDistance (optional, 0 uses default)
            double obsMinDist = obstacleNode.get<double>("minimumDistance", 0.0);

            if (type == "box")
            {
                vector_t halfExtents = vector_t::Zero(3);
                auto sizeOpt = obstacleNode.get_child_optional("halfExtents");
                if (sizeOpt)
                {
                    int idx = 0;
                    for (const auto &val : sizeOpt.get())
                    {
                        if (idx < 3)
                            halfExtents(idx++) = std::stod(val.second.data());
                    }
                }
                envGeomInterfacePtr_->addBox(obstacleName, halfExtents, position, orientation, obsMinDist);
                std::cerr << " ####   - Box '" << obstacleName << "': halfExtents=(" << halfExtents.transpose()
                          << "), pos=(" << position.transpose() << "), minDist=" << obsMinDist << "\n";
            }
            else if (type == "sphere")
            {
                double radius = obstacleNode.get<double>("radius", 0.1);
                envGeomInterfacePtr_->addSphere(obstacleName, radius, position, obsMinDist);
                std::cerr << " ####   - Sphere '" << obstacleName << "': radius=" << radius
                          << ", pos=(" << position.transpose() << "), minDist=" << obsMinDist << "\n";
            }
            else if (type == "cylinder")
            {
                double radius = obstacleNode.get<double>("radius", 0.1);
                double height = obstacleNode.get<double>("height", 0.2);
                envGeomInterfacePtr_->addCylinder(obstacleName, radius, height, position, orientation, obsMinDist);
                std::cerr << " ####   - Cylinder '" << obstacleName << "': radius=" << radius
                          << ", height=" << height << ", pos=(" << position.transpose() << "), minDist=" << obsMinDist << "\n";
            }
            else
            {
                std::cerr << " ####   Warning: unknown obstacle type '" << type << "' for '" << obstacleName << "'\n";
                continue;
            }
            obstacleCount++;
        }
        std::cerr << " #### Loaded " << obstacleCount << " initial obstacles.\n";
    }
} // namespace wbmm_ocs2
