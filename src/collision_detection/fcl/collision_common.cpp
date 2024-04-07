#include "mplib/collision_detection/fcl/collision_common.h"

namespace mplib::collision_detection::fcl {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_FCL_COMMON(S)                                                  \
  template size_t collide<S>(const FCLObjectPtr<S> &obj1, const FCLObjectPtr<S> &obj2, \
                             const fcl::CollisionRequest<S> &request,                  \
                             fcl::CollisionResult<S> &result);                         \
  template S distance<S>(const FCLObjectPtr<S> &obj1, const FCLObjectPtr<S> &obj2,     \
                         const fcl::DistanceRequest<S> &request,                       \
                         fcl::DistanceResult<S> &result)

DEFINE_TEMPLATE_FCL_COMMON(float);
DEFINE_TEMPLATE_FCL_COMMON(double);

template <typename S>
size_t collide(const FCLObjectPtr<S> &obj1, const FCLObjectPtr<S> &obj2,
               const fcl::CollisionRequest<S> &request,
               fcl::CollisionResult<S> &result) {
  // TODO: can request.enable_contact and request.enable_cost be used to short-circuit?
  for (const auto &shape1 : obj1->shapes)
    for (const auto &shape2 : obj2->shapes) {
      if (request.isSatisfied(result)) return result.numContacts();

      fcl::CollisionResult<S> tmp_result;
      fcl::collide(shape1.get(), shape2.get(), request, tmp_result);

      for (size_t i = 0; i < tmp_result.numContacts(); i++)
        result.addContact(tmp_result.getContact(i));

      auto cost_sources = std::vector<fcl::CostSource<S>>();
      tmp_result.getCostSources(cost_sources);
      for (const auto &cost_source : cost_sources)
        result.addCostSource(cost_source, request.num_max_cost_sources);
    }

  return result.numContacts();
}

template <typename S>
S distance(const FCLObjectPtr<S> &obj1, const FCLObjectPtr<S> &obj2,
           const fcl::DistanceRequest<S> &request, fcl::DistanceResult<S> &result) {
  // TODO: can request.enable_nearest_points be used to short-circuit?
  for (const auto &shape1 : obj1->shapes)
    for (const auto &shape2 : obj2->shapes) {
      if (request.isSatisfied(result)) return result.min_distance;

      fcl::DistanceResult<S> tmp_result;
      fcl::distance(shape1.get(), shape2.get(), request, tmp_result);

      result.update(tmp_result);
    }

  return result.min_distance;
}

}  // namespace mplib::collision_detection::fcl
