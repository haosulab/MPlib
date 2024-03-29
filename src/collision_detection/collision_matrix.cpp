#include "mplib/collision_detection/collision_matrix.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

namespace mplib::collision_detection {

AllowedCollisionMatrix::AllowedCollisionMatrix() {}

std::optional<AllowedCollision> AllowedCollisionMatrix::getEntry(
    const std::string &name1, const std::string &name2) const {
  const auto it1 = entries_.find(name1);
  if (it1 == entries_.end()) return std::nullopt;
  const auto it2 = it1->second.find(name2);
  if (it2 == it1->second.end()) return std::nullopt;
  return it2->second;
}

bool AllowedCollisionMatrix::hasEntry(const std::string &name) const {
  return entries_.find(name) != entries_.end();
}

bool AllowedCollisionMatrix::hasEntry(const std::string &name1,
                                      const std::string &name2) const {
  const auto it1 = entries_.find(name1);
  if (it1 == entries_.end()) return false;
  return it1->second.find(name2) != it1->second.end();
}

void AllowedCollisionMatrix::setEntry(const std::string &name1,
                                      const std::string &name2, bool allowed) {
  const auto v = allowed ? AllowedCollision::ALWAYS : AllowedCollision::NEVER;
  entries_[name1][name2] = entries_[name2][name1] = v;

  /* unused for now
  // remove function pointers, if any
  auto it = allowed_contacts_.find(name1);
  if (it != allowed_contacts_.end()) {
    auto jt = it->second.find(name2);
    if (jt != it->second.end()) it->second.erase(jt);
  }
  it = allowed_contacts_.find(name2);
  if (it != allowed_contacts_.end()) {
    auto jt = it->second.find(name1);
    if (jt != it->second.end()) it->second.erase(jt);
  }
  */
}

void AllowedCollisionMatrix::setEntry(const std::string &name,
                                      const std::vector<std::string> &other_names,
                                      bool allowed) {
  for (const auto &other_name : other_names)
    if (other_name != name) setEntry(other_name, name, allowed);
}

void AllowedCollisionMatrix::setEntry(const std::vector<std::string> &names1,
                                      const std::vector<std::string> &names2,
                                      bool allowed) {
  for (const auto &name1 : names1) setEntry(name1, names2, allowed);
}

void AllowedCollisionMatrix::setEntry(const std::string &name, bool allowed) {
  // NOTE: If name is a new element, need to reserve spot so iterators remain
  // valid in the range-based for loop over entries_
  // See: https://en.cppreference.com/w/cpp/container/unordered_map/operator_at
  entries_.reserve(entries_.size() + 1);

  for (const auto &entry : entries_)
    if (name != entry.first) setEntry(name, entry.first, allowed);
}

void AllowedCollisionMatrix::setEntry(const std::vector<std::string> &names,
                                      bool allowed) {
  for (const auto &name : names) setEntry(name, allowed);
}

void AllowedCollisionMatrix::setEntry(bool allowed) {
  const auto v = allowed ? AllowedCollision::ALWAYS : AllowedCollision::NEVER;
  for (auto &entry : entries_)
    for (auto &it2 : entry.second) it2.second = v;
  /* unused for now
  allowed_contacts_.clear();
  */
}

void AllowedCollisionMatrix::removeEntry(const std::string &name1,
                                         const std::string &name2) {
  if (auto it = entries_.find(name1); it != entries_.end())
    if (it->second.erase(name2) == 1 && it->second.empty()) entries_.erase(it);
  if (auto it = entries_.find(name2); it != entries_.end())
    if (it->second.erase(name1) == 1 && it->second.empty()) entries_.erase(it);

  /* unused for now
  if (auto it = allowed_contacts_.find(name1); it != allowed_contacts_.end())
    if (it->second.erase(name2) == 1 && it->second.empty())
      allowed_contacts_.erase(it);
  if (auto it = allowed_contacts_.find(name2); it != allowed_contacts_.end())
    if (it->second.erase(name1) == 1 && it->second.empty())
      allowed_contacts_.erase(it);
  */
}

void AllowedCollisionMatrix::removeEntry(const std::string &name,
                                         const std::vector<std::string> &other_names) {
  for (const auto &other_name : other_names)
    if (other_name != name) removeEntry(other_name, name);
}

void AllowedCollisionMatrix::removeEntry(const std::vector<std::string> &names1,
                                         const std::vector<std::string> &names2) {
  for (const auto &name1 : names1) removeEntry(name1, names2);
}

void AllowedCollisionMatrix::removeEntry(const std::string &name) {
  entries_.erase(name);
  for (auto it = entries_.begin(); it != entries_.end();)
    if (it->second.erase(name) == 1 && it->second.empty())
      it = entries_.erase(it);
    else
      ++it;
  /* unused for now
  allowed_contacts_.erase(name);
  for (auto it = allowed_contacts_.begin(); it != allowed_contacts_.end();)
    if (it->second.erase(name) == 1 && it->second.empty())
      it = allowed_contacts_.erase(it);
    else
      ++it;
  */
}

void AllowedCollisionMatrix::removeEntry(const std::vector<std::string> &names) {
  for (const auto &name : names) removeEntry(name);
}

std::optional<AllowedCollision> AllowedCollisionMatrix::getDefaultEntry(
    const std::string &name) const {
  auto it = default_entries_.find(name);
  if (it == default_entries_.end()) return std::nullopt;
  return it->second;
}

std::optional<AllowedCollision> AllowedCollisionMatrix::getDefaultEntry(
    const std::string &name1, const std::string &name2) const {
  auto t1 = getDefaultEntry(name1), t2 = getDefaultEntry(name2);
  if (!t1 && !t2)
    return std::nullopt;
  else if (t1 && !t2)
    return t1;
  else if (!t1 && t2)
    return t2;
  else {  // t1 && t2
    if (t1 == AllowedCollision::NEVER || t2 == AllowedCollision::NEVER)
      return AllowedCollision::NEVER;
    else if (t1 == AllowedCollision::CONDITIONAL || t2 == AllowedCollision::CONDITIONAL)
      return AllowedCollision::CONDITIONAL;
    else  // ALWAYS is the only remaining case
      return AllowedCollision::ALWAYS;
  }
}

bool AllowedCollisionMatrix::hasDefaultEntry(const std::string &name) const {
  return default_entries_.find(name) != default_entries_.end();
}

void AllowedCollisionMatrix::setDefaultEntry(const std::string &name, bool allowed) {
  const auto v = allowed ? AllowedCollision::ALWAYS : AllowedCollision::NEVER;
  default_entries_[name] = v;
  /* unused for now
  default_allowed_contacts_.erase(name);
  */
}

void AllowedCollisionMatrix::setDefaultEntry(const std::vector<std::string> &names,
                                             bool allowed) {
  for (const auto &name : names) setDefaultEntry(name, allowed);
}

void AllowedCollisionMatrix::removeDefaultEntry(const std::string &name) {
  default_entries_.erase(name);
  /* unused for now
  default_allowed_contacts_.erase(name);
  */
}

void AllowedCollisionMatrix::removeDefaultEntry(const std::vector<std::string> &names) {
  for (const auto &name : names) removeDefaultEntry(name);
}

std::optional<AllowedCollision> AllowedCollisionMatrix::getAllowedCollision(
    const std::string &name1, const std::string &name2) const {
  auto t1 = getEntry(name1, name2);
  return t1 ? t1 : getDefaultEntry(name1, name2);
}

void AllowedCollisionMatrix::clear() {
  entries_.clear();
  default_entries_.clear();
  /* unused for now
  allowed_contacts_.clear();
  default_allowed_contacts_.clear();
  */
}

std::vector<std::string> AllowedCollisionMatrix::getAllEntryNames() const {
  std::vector<std::string> names;
  for (const auto &entry : entries_) names.push_back(entry.first);
  for (const auto &entry : default_entries_) names.push_back(entry.first);

  std::sort(names.begin(), names.end());
  names.erase(std::unique(names.begin(), names.end()), names.end());

  return names;
}

void AllowedCollisionMatrix::print(std::ostream &out) const {
  std::vector<std::string> names = getAllEntryNames();

  size_t spacing = 4;
  for (const auto &name : names)
    if (const size_t length = name.length(); length > spacing) spacing = length;
  ++spacing;

  size_t number_digits = 2;
  while (names.size() > std::pow(10, number_digits) - 1) number_digits++;

  // print indices along the top of the matrix
  for (size_t j = 0; j < number_digits; ++j) {
    out << std::setw(spacing + number_digits + 8) << "";
    for (size_t i = 0; i < names.size(); ++i) {
      std::stringstream ss;
      ss << std::setw(number_digits) << i;
      out << std::setw(3) << ss.str()[j];
    }
    out << '\n';
  }

  const std::string indicator = "01?";  // NEVER / ALWAYS / CONDITIONAL
  for (size_t i = 0; i < names.size(); ++i) {
    out << std::setw(spacing) << names[i];
    out << std::setw(number_digits + 1) << i;
    out << " | ";
    // print default value
    if (auto type = getDefaultEntry(names[i]))
      out << indicator[static_cast<int>(*type)];
    else
      out << '-';
    out << " | ";
    // print pairs
    for (size_t j = 0; j < names.size(); ++j)
      if (auto type = getAllowedCollision(names[i], names[j]))
        out << std::setw(3) << indicator[static_cast<int>(*type)];
      else
        out << std::setw(3) << '-';
    out << '\n';
  }
}

}  // namespace mplib::collision_detection
