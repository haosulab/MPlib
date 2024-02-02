#pragma once

#include <functional>

#include <ompl/base/Constraint.h>

#include "mplib/planning/ompl/types.h"
#include "mplib/types.h"

namespace mplib::planning::ompl {

class GeneralConstraint : public ob::Constraint {
 public:
  GeneralConstraint(
      size_t dim, const std::function<void(const VectorXd &, Eigen::Ref<VectorXd>)> &f,
      const std::function<void(const VectorXd &, Eigen::Ref<VectorXd>)> &j)
      : ob::Constraint(dim, 1), f_(f), j_(j) {}

  void function(const Eigen::Ref<const VectorXd> &x,
                Eigen::Ref<VectorXd> out) const override {
    f_(x, out);
  }

  void jacobian(const Eigen::Ref<const VectorXd> &x,
                Eigen::Ref<MatrixXd> out) const override {
    j_(x, out);
  }

 private:
  std::function<void(const VectorXd &, Eigen::Ref<VectorXd>)> f_, j_;
};

}  // namespace mplib::planning::ompl
