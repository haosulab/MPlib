#include "mplib/core/attached_body.h"

namespace mplib {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_ATTACHED_BODY(S) template class AttachedBodyTpl<S>

DEFINE_TEMPLATE_ATTACHED_BODY(float);
DEFINE_TEMPLATE_ATTACHED_BODY(double);

template <typename S>
AttachedBodyTpl<S>::AttachedBodyTpl(const std::string &name, const FCLObjectPtr &object,
                                    const ArticulatedModelPtr &attached_articulation,
                                    int attached_link_id, const Pose<S> &pose,
                                    const std::vector<std::string> &touch_links)
    : name_(name),
      object_(object),
      attached_articulation_(attached_articulation),
      pinocchio_model_(attached_articulation->getPinocchioModel()),
      attached_link_id_(attached_link_id),
      pose_(pose.toIsometry()),
      touch_links_(touch_links) {
  updatePose();  // updates global pose using link_pose and attached_pose
}

template <typename S>
void AttachedBodyTpl<S>::updatePose() const {
  auto object_pose = getGlobalPose();
  object_->pose = object_pose;
  for (size_t i = 0; i < object_->shapes.size(); i++)
    object_->shapes[i]->setTransform(object_pose * object_->shape_poses[i]);
}

}  // namespace mplib
