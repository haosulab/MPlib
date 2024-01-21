#include "kdl_model.h"

#define DEFINE_TEMPLATE_FM(DATATYPE) template class KDLModelTpl<DATATYPE>;

DEFINE_TEMPLATE_FM(double)

DEFINE_TEMPLATE_FM(float)

template <typename DATATYPE>
KDLModelTpl<DATATYPE>::KDLModelTpl(const std::string &urdf_filename,
                                   const std::vector<std::string> &joint_names,
                                   const std::vector<std::string> &link_names,
                                   const bool &verbose)
    : user_link_names(link_names), user_joint_names(joint_names), verbose(verbose) {
  // std::cout << "Verbose" << verbose << std::endl;
  for (size_t i = 0; i < joint_names.size(); i++)
    user_joint_idx_mapping[joint_names[i]] = i;
  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDFFile(urdf_filename);
  treeFromUrdfModel(urdf, tree, tree_root_name, verbose);

  KDL::SegmentMap segments = tree.getSegments();
  joint_mapping_kdl_2_user.resize(tree.getNrOfJoints());

  for (KDL::SegmentMap::const_iterator it = segments.begin(); it != segments.end();
       ++it) {
    std::string joint_name = it->second.segment.getJoint().getName();
    std::map<std::string, int>::iterator it1 = user_joint_idx_mapping.find(joint_name);
    if (it1 != user_joint_idx_mapping.end())
      joint_mapping_kdl_2_user[it->second.q_nr] = user_joint_idx_mapping[joint_name];
  }
}

template <typename DATATYPE>
std::tuple<Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1>, int>
KDLModelTpl<DATATYPE>::chainIKLMA(const size_t &index, const VectorX &q0,
                                  const Vector7 &pose) {
  KDL::Chain chain;
  Eigen::Matrix<double, 6, 1> L;
  L(0) = 1;
  L(1) = 1;
  L(2) = 1;
  L(3) = 0.01;
  L(4) = 0.01;
  L(5) = 0.01;
  ASSERT(index < user_link_names.size(), "link index out of bound");
  tree.getChain(tree_root_name, user_link_names[index], chain);

  KDL::Frame frame_goal =
      KDL::Frame(KDL::Rotation::Quaternion(pose[4], pose[5], pose[6], pose[3]),
                 KDL::Vector(pose[0], pose[1], pose[2]));
  KDL::ChainIkSolverPos_LMA solver(chain, L);
  int n = chain.getNrOfJoints();
  KDL::JntArray q_init(n);
  KDL::JntArray q_sol(n);
  std::vector<int> idx;
  for (auto seg : chain.segments) {
    auto joint = seg.getJoint();
    if (joint.getType() != KDL::Joint::Fixed)
      idx.push_back(user_joint_idx_mapping[joint.getName()]);
  }
  for (int i = 0; i < n; i++) q_init(i) = q0[idx[i]];
  auto retval = solver.CartToJnt(q_init, frame_goal, q_sol);
  VectorX q1 = q0;
  for (int i = 0; i < n; i++) q1[idx[i]] = q_sol(i);
  return {q1, retval};
}

template <typename DATATYPE>
std::tuple<Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1>, int>
KDLModelTpl<DATATYPE>::chainIKNR(const size_t &index, const VectorX &q0,
                                 const Vector7 &pose) {
  KDL::Chain chain;
  ASSERT(index < user_link_names.size(), "link index out of bound");
  tree.getChain(tree_root_name, user_link_names[index], chain);

  KDL::Frame frame_goal =
      KDL::Frame(KDL::Rotation::Quaternion(pose[4], pose[5], pose[6], pose[3]),
                 KDL::Vector(pose[0], pose[1], pose[2]));

  KDL::ChainFkSolverPos_recursive fkpossolver(chain);
  KDL::ChainIkSolverVel_pinv ikvelsolver(chain);

  KDL::ChainIkSolverPos_NR solver(chain, fkpossolver, ikvelsolver);

  int n = chain.getNrOfJoints();
  KDL::JntArray q_init(n);
  KDL::JntArray q_sol(n);
  std::vector<int> idx;
  for (auto seg : chain.segments) {
    auto joint = seg.getJoint();
    if (joint.getType() != KDL::Joint::Fixed)
      idx.push_back(user_joint_idx_mapping[joint.getName()]);
  }
  for (int i = 0; i < n; i++) q_init(i) = q0[idx[i]];
  auto retval = solver.CartToJnt(q_init, frame_goal, q_sol);
  VectorX q1 = q0;
  for (int i = 0; i < n; i++) q1[idx[i]] = q_sol(i);
  return {q1, retval};
}

template <typename DATATYPE>
std::tuple<Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1>, int>
KDLModelTpl<DATATYPE>::chainIKNRJL(const size_t &index, const VectorX &q0,
                                   const Vector7 &pose, const VectorX &qmin,
                                   const VectorX &qmax) {
  KDL::Chain chain;
  ASSERT(index < user_link_names.size(), "link index out of bound");
  tree.getChain(tree_root_name, user_link_names[index], chain);

  KDL::Frame frame_goal =
      KDL::Frame(KDL::Rotation::Quaternion(pose[4], pose[5], pose[6], pose[3]),
                 KDL::Vector(pose[0], pose[1], pose[2]));

  KDL::ChainFkSolverPos_recursive fkpossolver(chain);
  KDL::ChainIkSolverVel_pinv ikvelsolver(chain);
  KDL::JntArray q_min(chain.getNrOfJoints()), q_max(chain.getNrOfJoints());
  int n = chain.getNrOfJoints();
  KDL::JntArray q_init(n);
  KDL::JntArray q_sol(n);
  std::vector<int> idx;
  for (auto seg : chain.segments) {
    auto joint = seg.getJoint();
    if (joint.getType() != KDL::Joint::Fixed)
      idx.push_back(user_joint_idx_mapping[joint.getName()]);
  }

  for (int i = 0; i < n; i++) {
    q_min(i) = qmin[idx[i]];
    q_max(i) = qmax[idx[i]];
    // printf("%lf %lf\n", qmin[idx[i]], qmax[idx[i]]);
  }

  KDL::ChainIkSolverPos_NR_JL solver(chain, q_min, q_max, fkpossolver, ikvelsolver);

  for (int i = 0; i < n; i++) q_init(i) = q0[idx[i]];
  auto retval = solver.CartToJnt(q_init, frame_goal, q_sol);
  VectorX q1 = q0;
  for (int i = 0; i < n; i++) q1[idx[i]] = q_sol(i);
  return {q1, retval};
}

template <typename DATATYPE>
std::tuple<Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1>, int>
KDLModelTpl<DATATYPE>::TreeIKNRJL(const std::vector<std::string> endpoints,
                                  const VectorX &q0, const std::vector<Vector7> &poses,
                                  const VectorX &qmin, const VectorX &qmax) {
  KDL::TreeFkSolverPos_recursive fkpossolver(tree);
  KDL::TreeIkSolverVel_wdls ikvelsolver(tree, endpoints);
  ikvelsolver.setLambda(1e-6);

  int n = tree.getNrOfJoints();
  KDL::JntArray q_min(n), q_max(n), q_init(n), q_sol(n);

  for (int i = 0; i < n; i++) {
    q_min(i) = qmin[joint_mapping_kdl_2_user[i]];
    q_max(i) = qmax[joint_mapping_kdl_2_user[i]];
  }

  KDL::Frames frames;
  for (size_t i = 0; i < endpoints.size(); i++) {
    frames[endpoints[i]] = KDL::Frame(
        KDL::Rotation::Quaternion(poses[i][4], poses[i][5], poses[i][6], poses[i][3]),
        KDL::Vector(poses[i][0], poses[i][1], poses[i][2]));
  }

  for (int i = 0; i < n; i++) q_init(i) = q0[joint_mapping_kdl_2_user[i]];

  KDL::TreeIkSolverPos_NR_JL solver(tree, endpoints, q_min, q_max, fkpossolver,
                                    ikvelsolver, 1000, 1e-6);

  auto retval = solver.CartToJnt(q_init, frames, q_sol);

  VectorX q1 = q0;
  for (int i = 0; i < n; i++) q1[joint_mapping_kdl_2_user[i]] = q_sol(i);

  return {q1, retval};
}
