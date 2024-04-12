#pragma once

#include <string>
#include <vector>

#include "mplib/collision_detection/types.h"
#include "mplib/core/articulated_model.h"
#include "mplib/kinematics/types.h"
#include "mplib/macros/class_forward.h"
#include "mplib/types.h"
#include "mplib/utils/pose.h"

namespace mplib {

// AttachedBodyTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(AttachedBodyTpl);

/**
 * Object defining bodies that can be attached to robot links.
 * This is useful when handling objects picked up by the robot.
 *
 * Mimicking MoveIt2's ``moveit::core::AttachedBody``
 *
 * https://moveit.picknik.ai/main/api/html/classmoveit_1_1core_1_1AttachedBody.html
 */
template <typename S>
class AttachedBodyTpl {
 public:
  // Common type alias
  using FCLObjectPtr = collision_detection::FCLObjectPtr<S>;
  using ArticulatedModelPtr = ArticulatedModelTplPtr<S>;

  /**
   * Construct an attached body for a specified link.
   *
   * @param name: name of the attached body
   * @param object: collision object of the attached body
   * @param attached_articulation: robot articulated model to attach to
   * @param attached_link_id: id of the articulation link to attach to
   * @param pose: attached pose (relative pose from attached link to object)
   * @param touch_links: the link names that the attached body touches
   */
  AttachedBodyTpl(const std::string &name, const FCLObjectPtr &object,
                  const ArticulatedModelPtr &attached_articulation,
                  int attached_link_id, const Pose<S> &pose,
                  const std::vector<std::string> &touch_links = {});

  /// @brief Gets the attached object name
  const std::string &getName() const { return name_; }

  /// @brief Gets the attached object (``FCLObjectPtr``)
  FCLObjectPtr getObject() const { return object_; }

  /// @brief Gets the articulation that this body is attached to
  ArticulatedModelPtr getAttachedArticulation() const { return attached_articulation_; }

  /// @brief Gets the articulation link id that this body is attached to
  int getAttachedLinkId() const { return attached_link_id_; }

  /// @brief Gets the attached pose (relative pose from attached link to object)
  const Isometry3<S> &getPose() const { return pose_; }

  /// @brief Sets the attached pose (relative pose from attached link to object)
  void setPose(const Isometry3<S> &pose) { pose_ = pose; }

  /// @brief Gets the global pose of the articulation link that this body is attached to
  Isometry3<S> getAttachedLinkGlobalPose() const {
    return pinocchio_model_->getLinkPose(attached_link_id_);
  }

  /// @brief Gets the global pose of the attached object
  Isometry3<S> getGlobalPose() const { return getAttachedLinkGlobalPose() * pose_; }

  /// @brief Updates the global pose of the attached object using current state
  void updatePose() const;

  /// @brief Gets the link names that the attached body touches
  const std::vector<std::string> &getTouchLinks() const { return touch_links_; }

  /// @brief Sets the link names that the attached body touches
  void setTouchLinks(const std::vector<std::string> &touch_links) {
    touch_links_ = touch_links;
  }

 private:
  std::string name_;
  FCLObjectPtr object_;
  ArticulatedModelPtr attached_articulation_;
  kinematics::PinocchioModelTplPtr<S> pinocchio_model_;
  int attached_link_id_ {};
  Isometry3<S> pose_;
  std::vector<std::string> touch_links_;
};

// Common Type Alias ===================================================================
using AttachedBodyf = AttachedBodyTpl<float>;
using AttachedBodyd = AttachedBodyTpl<double>;
using AttachedBodyfPtr = AttachedBodyTplPtr<float>;
using AttachedBodydPtr = AttachedBodyTplPtr<double>;

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_ATTACHED_BODY(S) extern template class AttachedBodyTpl<S>

DECLARE_TEMPLATE_ATTACHED_BODY(float);
DECLARE_TEMPLATE_ATTACHED_BODY(double);

}  // namespace mplib
