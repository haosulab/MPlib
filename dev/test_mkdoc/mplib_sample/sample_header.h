#include <string>
#include <vector>

namespace mplib {

/**
 * PlanningWorld
 *
 * - Begin first unordered list element. Volutpat blandit aliquam etiam erat
 *   velit scelerisque. End first unordered list element.
 * - Begin second unordered list element. Eget est lorem ipsum dolor sit amet.
 *   Ipsum dolor sit amet consectetur adipiscing. End second unordered list
 *   element.
 * - Begin third unordered list element. Hac habitasse platea dictumst quisque
 *   sagittis purus sit. End third unordered list element.
 */
template <typename S>
class PlanningWorldTpl {
 public:
  /**
   * Attaches existing normal object to specified link of articulation.
   *
   * If the object is currently attached, disallow collision between the object
   * and previous touch_links.
   * Updates acm_ to allow collisions between attached object and touch_links.
   *
   * @param urdf_filename: path to URDF file, can be relative to the current working
   *  directory
   * @param[in] name: normal object name to attach
   * @param[in] art_name: name of the planned articulation to attach to
   * @param[in] link_id: index of the link of the planned articulation to attach to.
   *    Begin precondition. Cras fermentum odio eu feugiat pretium nibh.
   * @param[out] pose: attached pose (relative pose from attached link to object)
   * @param[out] touch_links: link names that the attached object touches
   * @return: the attached object
   * @throws std::out_of_range if normal object with given name does not exist
   *    or if planned articulation with given name does not exist
   */
  void attachObject(const std::string &name, const std::string &art_name, int link_id,
                    const std::vector<int> &pose,
                    const std::vector<std::string> &touch_links);
};

template <typename S>
std::vector<S> compoundstate2vector(const std::vector<int> &state_raw);

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_PLANNING_WORLD(S)                \
  extern template std::vector<S> compoundstate2vector<S>( \
      const std::vector<int> &state_raw);                 \
  extern template class PlanningWorldTpl<S>

DECLARE_TEMPLATE_PLANNING_WORLD(float);
DECLARE_TEMPLATE_PLANNING_WORLD(double);

}  // namespace mplib
