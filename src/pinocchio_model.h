#pragma once
#include <vector>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/joint/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <urdf_model/types.h>
#include <urdf_world/types.h>

#include "color_printing.h"
#include "macros_utils.hpp"

template <typename DATATYPE>
class PinocchioModelTpl {
 private:
  DEFINE_TEMPLATE_EIGEN(DATATYPE)
  DEFINE_TEMPLATE_PINOCCHIO(DATATYPE)

  // pinocchio::ModelTpl<float>;

  urdf::ModelInterfaceSharedPtr urdf_model;
  Model model;
  Data data;

  VectorXI joint_index_user2pinocchio, joint_index_pinocchio2user;
  VectorXI v_index_user2pinocchio;  // the joint index in model
  // map between user and pinocchio
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> v_map_user2pinocchio;
  VectorXI link_index_user2pinocchio;

  std::vector<std::string> user_link_names;
  std::vector<std::string> user_joint_names;
  std::vector<std::string> leaf_links;
  VectorXI qidx, vidx, nqs, nvs, parents;

  const std::string joint_prefix = "JointModel";
  bool verbose;

  VectorX qposUser2Pinocchio(const VectorX &qpos);

  VectorX qposPinocchio2User(const VectorX &qpos);

  void init(const urdf::ModelInterfaceSharedPtr &urdfTree, const Vector3 &gravity);

  void dfs_parse_tree(urdf::LinkConstSharedPtr link, UrdfVisitorBase &visitor);

 public:
  PinocchioModelTpl(const urdf::ModelInterfaceSharedPtr &urdfTree,
                    const Vector3 &gravity, const bool &verbose = true);

  PinocchioModelTpl(const std::string &urdf_filename, const Vector3 &gravity,
                    const bool &verbose = true);

  inline Model &getModel(void) { return model; }

  inline Data &getData(void) { return data; }

  std::vector<std::string> getLeafLinks() { return leaf_links; }

  inline std::string getJointType(const size_t &index, const bool &user = true) {
    if (user)
      return model.joints[joint_index_user2pinocchio[index]].shortname();
    else
      return model.joints[index].shortname();
  }

  inline std::vector<std::string> getJointTypes(const bool &user = true) {
    std::vector<std::string> ret;
    auto njoints = user ? user_joint_names.size() : model.joints.size();
    for (size_t i = 0; i < njoints; i++) ret.push_back(getJointType(i, user));
    return ret;
  }

  inline std::vector<Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>>
  getJointLimits(const bool &user = true) {
    std::vector<Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>> ret;
    auto njoints = user ? user_joint_names.size() : model.joints.size();
    for (size_t i = 0; i < njoints; i++) ret.push_back(getJointLimit(i, user));
    return ret;
  }

  inline Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic> getJointLimit(
      const size_t &index, const bool &user = true) {
    auto joint_type = getJointType(index, user);
    size_t pinocchio_idx = user ? joint_index_user2pinocchio[index] : index;
    size_t start_idx = model.idx_qs[pinocchio_idx], nq = model.nqs[pinocchio_idx],
           dim_joint = getJointDim(index, user);
    Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic> ret;
    ASSERT(dim_joint == 1, "Only support joint with dim 1 but joint" +
                               getJointNames(user)[index] + " has dim " +
                               std::to_string(dim_joint));
    // std::cout << joint_type << " " << joint_type[joint_prefix.size()] << " " <<
    // joint_type[joint_prefix.size() + 1] << " " <<  nq << " " << dim_joint << " " <<
    // std::endl;
    if (joint_type[joint_prefix.size()] == 'P' ||
        (joint_type[joint_prefix.size()] == 'R' &&
         joint_type[joint_prefix.size() + 1] != 'U')) {
      ret = Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>(nq, 2);
      for (size_t j = 0; j < nq; j++) {
        ret(j, 0) = model.lowerPositionLimit[start_idx + j];
        ret(j, 1) = model.upperPositionLimit[start_idx + j];
      }
    } else if (joint_type[joint_prefix.size()] == 'R' &&
               joint_type[joint_prefix.size() + 1] == 'U') {
      ret = Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>(1, 2);
      ret(0, 0) = -3.14159265359, ret(0, 1) = 3.14159265359;
    }
    return ret;
  }

  inline size_t getJointId(const size_t &index, const bool &user = true) {
    return user ? vidx[index] : model.idx_vs[index];
  }

  inline VectorXI getJointIds(const bool &user = true) {
    if (user)
      return vidx;
    else {
      auto ret = VectorXI(model.idx_vs.size());
      for (size_t i = 0; i < model.idx_vs.size(); i++) ret[i] = model.idx_vs[i];
      return ret;
    }
  }

  inline size_t getJointDim(const size_t &index, const bool &user = true) {
    return user ? nvs[index] : model.nvs[index];
  }

  inline VectorXI getJointDims(const bool &user = true) {
    if (user)
      return nvs;
    else {
      auto ret = VectorXI(model.nvs.size());
      for (size_t i = 0; i < model.nvs.size(); i++) ret[i] = model.nvs[i];
      return ret;
    }
  }

  inline size_t getParent(const size_t &index, const bool &user = true) {
    return user ? parents[index] : model.parents[index];
  }

  inline VectorXI getParents(const bool &user = true) {
    if (user)
      return parents;
    else {
      auto ret = VectorXI(model.parents.size());
      for (size_t i = 0; i < model.parents.size(); i++) ret[i] = model.parents[i];
      return ret;
    }
  }

  inline std::vector<std::string> getLinkNames(const bool &user = true) {
    if (user)
      return user_link_names;
    else {
      std::vector<std::string> link_names;
      for (size_t i = 0; i < model.frames.size(); i++)
        if (model.frames[i].type == pinocchio::BODY)
          link_names.push_back(model.frames[i].name);
      return link_names;
    }
  }

  inline std::vector<std::string> getJointNames(const bool &user = true) {
    if (user) return user_joint_names;
    // we need to ignore the "universe" joint
    return std::vector<std::string>(model.names.begin() + 1, model.names.end());
  }

  std::vector<std::vector<size_t>> getSupports(const bool &user = true) {
    if (user) {
      std::vector<std::vector<size_t>> ret;
      return ret;
    } else
      return model.supports;
  }

  std::vector<std::vector<size_t>> getSubtrees(const bool &user = true) {
    if (user) {
      std::vector<std::vector<size_t>> ret;
      return ret;
    } else
      return model.subtrees;
  }

  // Frame is a Pinocchio internal data type which is not supported outside this class.
  void printFrames(void);

  int getNFrames(void) { return model.nframes; }

  /*
      // The original model for debug only
      inline size_t getDimQpos(void) { return model.nv; }

      int getNQ(void) { return model.nq; }

      int getNV(void) { return model.nv; }

      int getNJoints(void) { return model.njoints; }

      int getNBodies(void) { return model.nbodies; }

      std::vector<std::vector<size_t>> getSupports(void) { return model.supports; };

      std::vector<std::vector<size_t>> getSubtrees(void) { return model.subtrees; };

      int getNLinks(void);

      int getNFrames(void) { return model.nframes; }

      std::vector<int> getNQs(void) { return model.nqs; }

      std::vector<int> getNVs(void) { return model.nvs; }

      std::vector<int> getIDXVs(void) { return model.idx_vs; }

      std::vector<int> getIDXQs(void) { return model.idx_qs; }

      std::vector<std::string> getJointNames(void) { return model.names; }

      std::vector<std::string> getFrameNames(void);

      std::vector<std::string> getLinkNames(void);

      std::vector<size_t> getParents(void) { return model.parents; }

      void printFrames(void);*/
  //

  void setJointOrder(const std::vector<std::string> &names);

  void setLinkOrder(const std::vector<std::string> &names);

  std::vector<std::size_t> getChainJointIndex(const std::string &end_effector);

  std::vector<std::string> getChainJointName(const std::string &end_effector);

  VectorX getRandomConfiguration();

  void computeForwardKinematics(const VectorX &qpos);

  Vector7 getLinkPose(const size_t &index);

  Vector7 getJointPose(const size_t &index);  // TODO not same as sapien

  void computeFullJacobian(const VectorX &qpos);

  Matrix6x getLinkJacobian(const size_t &index, const bool &local = false);

  Matrix6x computeSingleLinkJacobian(const VectorX &qpos, const size_t &index,
                                     bool local = false);

  std::tuple<VectorX, bool, Vector6> computeIKCLIK(
      const size_t &index, const Vector7 &pose, const VectorX &q_init,
      const std::vector<bool> &mask, const double &eps = 1e-5,
      const int &maxIter = 1000, const double &dt = 1e-1, const double &damp = 1e-12);

  std::tuple<VectorX, bool, Vector6> computeIKCLIKJL(
      const size_t &index, const Vector7 &pose, const VectorX &q_init,
      const VectorX &qmin, const VectorX &qmax, const double &eps = 1e-5,
      const int &maxIter = 1000, const double &dt = 1e-1, const double &damp = 1e-12);
};

template <typename T>
using PinocchioModelTpl_ptr = std::shared_ptr<PinocchioModelTpl<T>>;

using PinocchioModeld = PinocchioModelTpl<double>;
using PinocchioModelf = PinocchioModelTpl<float>;
using PinocchioModeld_ptr = PinocchioModelTpl_ptr<double>;
using PinocchioModelf_ptr = PinocchioModelTpl_ptr<float>;
