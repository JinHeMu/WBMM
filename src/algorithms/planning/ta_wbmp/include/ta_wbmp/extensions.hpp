#pragma once

#include <Eigen/Core>

#include <memory>
#include <string>

namespace ta_wbmp
{

struct StateValidityResult
{
  bool valid{true};
  double clearance{0.0};
  std::string reason;
};

// Extension point for a shared REMANI/ESDF collision model. The default
// implementation preserves the current kinematics-only demo; production code
// should inject an adapter backed by the same robot model and ESDF as REMANI.
class WholeBodyStateValidityChecker
{
public:
  virtual ~WholeBodyStateValidityChecker() = default;
  virtual StateValidityResult check(const Eigen::VectorXd & state) const = 0;
  virtual bool checksEnvironment() const {return false;}
};

class AcceptAllStateValidityChecker final : public WholeBodyStateValidityChecker
{
public:
  StateValidityResult check(const Eigen::VectorXd &) const override
  {
    return {};
  }
};

// Safe default for the standalone planner: uses the same URDF collision
// geometry and non-adjacent-link pairing rule as the REMANI/WipePlanner stack.
// Environment/ESDF checks can be composed by injecting another checker; they
// must never be replaced by AcceptAll in a production execution path.
class UrdfSelfCollisionStateValidityChecker final :
  public WholeBodyStateValidityChecker
{
public:
  explicit UrdfSelfCollisionStateValidityChecker(
    const std::string & urdf_file, double minimum_clearance = 0.0);
  ~UrdfSelfCollisionStateValidityChecker() override;

  StateValidityResult check(const Eigen::VectorXd & state) const override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

StateValidityResult checkInterpolatedMotion(
  const WholeBodyStateValidityChecker & checker,
  const Eigen::VectorXd & first, const Eigen::VectorXd & second,
  double base_step = 0.02, double yaw_step = 0.05,
  double joint_step = 0.05);

// Extension point for navigation-aware entry ranking. The default estimator
// is deterministic and map-free; a REMANI/2D-map estimator can be injected
// without changing task generation or task-constrained whole-body planning.
class NavigationCostEstimator
{
public:
  virtual ~NavigationCostEstimator() = default;
  virtual double estimate(const Eigen::VectorXd & start,
                          const Eigen::VectorXd & goal) const = 0;
};

class Se2NavigationCostEstimator final : public NavigationCostEstimator
{
public:
  explicit Se2NavigationCostEstimator(double yaw_weight = 0.2)
  : yaw_weight_(yaw_weight) {}

  double estimate(const Eigen::VectorXd & start,
                  const Eigen::VectorXd & goal) const override;

private:
  double yaw_weight_{0.2};
};

}  // namespace ta_wbmp
