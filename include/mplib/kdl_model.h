#pragma once
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/joint.hpp>
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolverpos_nr_jl.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <urdf_parser/urdf_parser.h>

#include "macros_utils.h"
#include "urdf_utils.h"

/**
 * KDL model of an articulation
 *
 * See https://github.com/orocos/orocos_kinematics_dynamics
 */
template <typename S>
class KDLModelTpl {
 private:
  DEFINE_TEMPLATE_EIGEN(S)
  KDL::Tree tree_;
  std::string tree_root_name_;
  std::vector<std::string> user_link_names_;
  std::vector<std::string> user_joint_names_;
  std::vector<int> joint_mapping_kdl_2_user_;
  std::map<std::string, int> user_joint_idx_mapping_;
  bool verbose_;

 public:
  KDLModelTpl(const std::string &urdf_filename,
              const std::vector<std::string> &joint_names,
              const std::vector<std::string> &link_names, const bool &verbose);

  std::string getTreeRootName() { return tree_root_name_; }

  std::tuple<VectorX, int> chainIKLMA(const size_t &index, const VectorX &q0,
                                      const Vector7 &pose);
  std::tuple<VectorX, int> chainIKNR(const size_t &index, const VectorX &q0,
                                     const Vector7 &pose);
  std::tuple<VectorX, int> chainIKNRJL(const size_t &index, const VectorX &q0,
                                       const Vector7 &pose, const VectorX &q_min,
                                       const VectorX &q_max);
  std::tuple<VectorX, int> TreeIKNRJL(const std::vector<std::string> endpoints,
                                      const VectorX &q0,
                                      const std::vector<Vector7> &poses,
                                      const VectorX &q_min, const VectorX &q_max);
};

template <typename T>
using KDLModelTplPtr = std::shared_ptr<KDLModelTpl<T>>;

using KDLModeld = KDLModelTpl<double>;
using KDLModelf = KDLModelTpl<float>;
using KDLModeldPtr = KDLModelTplPtr<double>;
using KDLModelfPtr = KDLModelTplPtr<float>;
