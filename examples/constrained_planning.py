#!/usr/bin/env python3

import mplib
import numpy as np
import transforms3d

from demo_setup import DemoSetup

"""
class LevelConstraint : public ob::Constraint {
    ArticulatedModeld_ptr model;
    size_t link_idx;
    Eigen::Vector3d v;  // the unit vector we want the constraint to align to
    double k;  // if k = 1, then we want the z axis of the end effector to be exactly v
public:
    LevelConstraint(ArticulatedModeld_ptr model, size_t link_idx, Eigen::Vector3d v, double k=1)
    : ob::Constraint(model->getQposDim(), 1),
      model(model),
      link_idx(link_idx),
      v(v),
      k(k) {
        ASSERT(0.99 < v.norm() && v.norm() < 1.01, "The align axis must be a unit vector.");
    }

    Eigen::Vector3d getEndEffectorZ() const {
        auto pinocchio_model = model->getPinocchioModel();
        auto dim = model->getQposDim();
        auto ee_pose = model->getPinocchioModel().getLinkPose(link_idx);
        auto ee_quat = ee_pose.tail(4);
        auto ee_rot = Eigen::Quaternion(ee_quat[0], ee_quat[1], ee_quat[2], ee_quat[3]).matrix();
        auto ee_z = ee_rot.col(2);
        return ee_z;
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override {
        model->setQpos(x);  // not full cuz we don't care about non-movegroup joints
        auto ee_z = getEndEffectorZ();
        out[0] = ee_z.dot(v) - k;
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override {
        model->setQpos(x);
        auto ee_z = getEndEffectorZ();
        auto pinocchio_model = model->getPinocchioModel();
        auto dim = model->getQposDim();
        pinocchio_model.computeFullJacobian(model->getQpos());
        auto link_jacobian = pinocchio_model.computeSingleLinkJacobian(model->getQpos(), dim-1);
        auto rot_jacobian = link_jacobian.bottomRows<3>();
        
        // need to select only the move group joints using model->getMoveGroupJointIndices()
        auto move_group_joint_indices = model->getMoveGroupJointIndices();
        auto rot_jacobian_move_group = rot_jacobian(Eigen::all, move_group_joint_indices);

        for (size_t i = 0; i < dim; i++) {
            out(0, i) = rot_jacobian_move_group.col(i).cross(ee_z).dot(v);
        }
    }
};
"""

class ConstrainedPlanningDemo(DemoSetup):
  def __init__(self):
    super().__init__()
    self.setup_scene()
    self.load_robot()
    self.setup_planner()

  def add_point_cloud(self):
    import trimesh
    box = trimesh.creation.box([0.1, 0.4, 0.2])
    points, _ = trimesh.sample.sample_surface(box, 1000)
    all_pts = np.concatenate([points+[-0.65, -0.1, 0.4], points+[0.55, 0, 0.1]], axis=0)
    self.planner.update_point_cloud(all_pts, radius=0.02)
    return

  def get_eef_z(self):
    ee_idx = self.planner.link_name_2_idx[self.planner.move_group]
    ee_pose = self.planner.robot.get_pinocchio_model().get_link_pose(ee_idx)
    mat = transforms3d.quaternions.quat2mat(ee_pose[3:])
    return mat[:,2]

  def make_f(self):
    def f(x, out):
      self.planner.robot.set_qpos(x)
      out[0] = self.get_eef_z().dot(np.array([0,0,-1]))-1
    return f

  def make_j(self):
    def j(x, out):
      full_qpos = self.planner.pad_qpos(x)
      jac = self.planner.robot.get_pinocchio_model().compute_single_link_jacobian(full_qpos, len(self.planner.move_group_joint_indices)-1)
      rot_jac = jac[3:, self.planner.move_group_joint_indices]
      for i in range(len(self.planner.move_group_joint_indices)):
        out[i] = np.cross(rot_jac[:,i], self.get_eef_z()).dot(np.array([0,0,-1]))
    return j

  def demo(self):
    starting_qpos = [0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78, 0, 0]
    self.robot.set_qpos(starting_qpos)
    self.planner.robot.set_qpos(starting_qpos[:7])
    poses = [[-0.4, -0.3, 0.28, 0.0000563, -0.0707372, -0.9974947, -0.0007943],
             [0.6, 0.1, 0.44, 0.0006988, -0.8775823, -0.4794254, -0.0003818],
             [0, -0.3, 0.5, 0, 1, 0, 0]]
    
    self.add_point_cloud()

    # with constraint
    print("with constraint")
    for pose in poses:
      result = self.planner.plan(
        pose,
        self.robot.get_qpos(),
        time_step=1/250,
        use_point_cloud=True,
        use_attach=False,
        planner_name="RRTConnect",
        constraint_function=self.make_f(),
        constraint_jacobian=self.make_j()
      )
      if result['status'] != "Success":
        print(result['status'])
        return -1
      self.follow_path(result)

    # without constraint
    print("without constraint")
    for pose in poses:
      result = self.planner.plan(
        pose,
        self.robot.get_qpos(),
        time_step=1/250,
        use_point_cloud=True,
        use_attach=False,
        planner_name="RRTConnect",
        no_simplification=True
      )
      if result['status'] != "Success":
        print(result['status'])
        return -1
      self.follow_path(result)


if __name__ == '__main__':
  demo = ConstrainedPlanningDemo()
  demo.demo()
