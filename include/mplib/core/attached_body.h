#pragma once

#include <string>
#include <vector>

#include "mplib/core/articulated_model.h"
#include "mplib/macros/class_forward.h"
#include "mplib/utils/conversion.h"
#include "mplib/kinematics/types.h"
#include "mplib/types.h"

namespace mplib {

// AttachedBodyTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(AttachedBodyTpl);

template <typename S>
class AttachedBodyTpl {
 public:
  // Common type alias
  using CollisionObjectPtr = fcl::CollisionObjectPtr<S>;
  using ArticulatedModelPtr = ArticulatedModelTplPtr<S>;

  AttachedBodyTpl(const std::string &name, const CollisionObjectPtr &object,
                  const ArticulatedModelPtr &attached_articulation,
                  int attached_link_id, const Isometry3<S> &pose,
                  const std::vector<std::string> &touch_links = {});

  /// @brief Gets the attached object name
  const std::string &getName() const { return name_; }

  /// @brief Gets the attached object (CollisionObjectPtr)
  CollisionObjectPtr getObject() const { return object_; }

  /// @brief Gets the articulation this body attached to
  ArticulatedModelPtr getAttachedArticulation() const { return attached_articulation_; }

  /// @brief Gets the articulation this body attached to
  int getAttachedLinkId() const { return attached_link_id_; }

  /// @brief Gets the attached pose (relative pose from attached link to object)
  const Isometry3<S> &getPose() const { return pose_; }

  /// @brief Sets the attached pose (relative pose from attached link to object)
  void setPose(const Isometry3<S> &pose) { pose_ = pose; }

  /// @brief Gets the global pose of the attached object
  Isometry3<S> getGlobalPose() const {
    return toIsometry(pinocchio_model_->getLinkPose(attached_link_id_)) *
           pose_;
  }

  /// @brief Updates the global pose of the attached object using current state
  void updatePose() const { object_->setTransform(getGlobalPose()); }

  /// @brief Gets the link names that the attached body touches
  const std::vector<std::string> &getTouchLinks() const { return touch_links_; }

  /// @brief Sets the link names that the attached body touches
  void setTouchLinks(const std::vector<std::string> &touch_links) {
    touch_links_ = touch_links;
  }

 private:
  std::string name_;
  CollisionObjectPtr object_;
  ArticulatedModelPtr attached_articulation_;
  kinematics::PinocchioModelTplPtr<S> pinocchio_model_;
  int attached_link_id_;
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
