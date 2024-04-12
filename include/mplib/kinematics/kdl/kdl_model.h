#pragma once

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include <kdl/tree.hpp>

#include "mplib/macros/class_forward.h"
#include "mplib/types.h"
#include "mplib/utils/pose.h"

namespace mplib::kinematics::kdl {

// KDLModelTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(KDLModelTpl);

/**
 * KDL model of an articulation
 *
 * See https://github.com/orocos/orocos_kinematics_dynamics
 */
template <typename S>
class KDLModelTpl {
 public:
  KDLModelTpl(const std::string &urdf_filename,
              const std::vector<std::string> &link_names,
              const std::vector<std::string> &joint_names, bool verbose = false);

  const std::string &getTreeRootName() const { return tree_root_name_; }

  std::tuple<VectorX<S>, int> chainIKLMA(size_t index, const VectorX<S> &q0,
                                         const Pose<S> &pose) const;

  std::tuple<VectorX<S>, int> chainIKNR(size_t index, const VectorX<S> &q0,
                                        const Pose<S> &pose) const;

  std::tuple<VectorX<S>, int> chainIKNRJL(size_t index, const VectorX<S> &q0,
                                          const Pose<S> &pose, const VectorX<S> &q_min,
                                          const VectorX<S> &q_max) const;

  std::tuple<VectorX<S>, int> TreeIKNRJL(const std::vector<std::string> endpoints,
                                         const VectorX<S> &q0,
                                         const std::vector<Pose<S>> &poses,
                                         const VectorX<S> &q_min,
                                         const VectorX<S> &q_max) const;

 private:
  KDL::Tree tree_;
  std::string tree_root_name_;

  std::vector<std::string> user_link_names_;
  std::vector<std::string> user_joint_names_;

  std::map<std::string, int> user_joint_idx_mapping_;
  std::vector<int> joint_mapping_kdl_2_user_;

  bool verbose_ {};
};

// Common Type Alias ===================================================================
using KDLModelf = KDLModelTpl<float>;
using KDLModeld = KDLModelTpl<double>;
using KDLModelfPtr = KDLModelTplPtr<float>;
using KDLModeldPtr = KDLModelTplPtr<double>;

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_KDL_MODEL(S) extern template class KDLModelTpl<S>

DECLARE_TEMPLATE_KDL_MODEL(float);
DECLARE_TEMPLATE_KDL_MODEL(double);

}  // namespace mplib::kinematics::kdl
