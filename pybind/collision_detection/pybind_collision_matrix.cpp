#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "docstring/collision_detection/collision_matrix.h"
#include "mplib/collision_detection/collision_matrix.h"

namespace py = pybind11;

namespace mplib::collision_detection {

void build_pycollision_matrix(py::module &m) {
  auto PyAllowedCollision = py::enum_<AllowedCollision>(
      m, "AllowedCollision", DOC(mplib, collision_detection, AllowedCollision));
  PyAllowedCollision
      .value("NEVER", AllowedCollision::NEVER,
             DOC(mplib, collision_detection, AllowedCollision, NEVER))
      .value("ALWAYS", AllowedCollision::ALWAYS,
             DOC(mplib, collision_detection, AllowedCollision, ALWAYS))
      .value("CONDITIONAL", AllowedCollision::CONDITIONAL,
             DOC(mplib, collision_detection, AllowedCollision, CONDITIONAL));

  auto PyAllowedCollisionMatrix =
      py::class_<AllowedCollisionMatrix, std::shared_ptr<AllowedCollisionMatrix>>(
          m, "AllowedCollisionMatrix",
          DOC(mplib, collision_detection, AllowedCollisionMatrix));
  PyAllowedCollisionMatrix
      .def(py::init<>(), DOC(mplib, collision_detection, AllowedCollisionMatrix,
                             AllowedCollisionMatrix))
      .def("get_entry", &AllowedCollisionMatrix::getEntry, py::arg("name1"),
           py::arg("name2"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, getEntry))
      .def("has_entry",
           py::overload_cast<const std::string &>(&AllowedCollisionMatrix::hasEntry,
                                                  py::const_),
           py::arg("name"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, hasEntry))
      .def("has_entry",
           py::overload_cast<const std::string &, const std::string &>(
               &AllowedCollisionMatrix::hasEntry, py::const_),
           py::arg("name1"), py::arg("name2"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, hasEntry, 2))
      .def("set_entry",
           py::overload_cast<const std::string &, const std::string &, bool>(
               &AllowedCollisionMatrix::setEntry),
           py::arg("name1"), py::arg("name2"), py::arg("allowed"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, setEntry))
      .def("set_entry",
           py::overload_cast<const std::string &, const std::vector<std::string> &,
                             bool>(&AllowedCollisionMatrix::setEntry),
           py::arg("name"), py::arg("other_names"), py::arg("allowed"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, setEntry, 2))
      .def("set_entry",
           py::overload_cast<const std::vector<std::string> &,
                             const std::vector<std::string> &, bool>(
               &AllowedCollisionMatrix::setEntry),
           py::arg("names1"), py::arg("names2"), py::arg("allowed"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, setEntry, 3))
      .def("set_entry",
           py::overload_cast<const std::string &, bool>(
               &AllowedCollisionMatrix::setEntry),
           py::arg("name"), py::arg("allowed"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, setEntry, 4))
      .def("set_entry",
           py::overload_cast<const std::vector<std::string> &, bool>(
               &AllowedCollisionMatrix::setEntry),
           py::arg("names"), py::arg("allowed"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, setEntry, 5))
      .def("set_entry", py::overload_cast<bool>(&AllowedCollisionMatrix::setEntry),
           py::arg("allowed"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, setEntry, 6))
      .def("remove_entry",
           py::overload_cast<const std::string &, const std::string &>(
               &AllowedCollisionMatrix::removeEntry),
           py::arg("name1"), py::arg("name2"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, removeEntry))
      .def("remove_entry",
           py::overload_cast<const std::string &, const std::vector<std::string> &>(
               &AllowedCollisionMatrix::removeEntry),
           py::arg("name"), py::arg("other_names"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, removeEntry, 2))
      .def("remove_entry",
           py::overload_cast<const std::vector<std::string> &,
                             const std::vector<std::string> &>(
               &AllowedCollisionMatrix::removeEntry),
           py::arg("names1"), py::arg("names2"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, removeEntry, 3))
      .def("remove_entry",
           py::overload_cast<const std::string &>(&AllowedCollisionMatrix::removeEntry),
           py::arg("name"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, removeEntry, 4))
      .def("remove_entry",
           py::overload_cast<const std::vector<std::string> &>(
               &AllowedCollisionMatrix::removeEntry),
           py::arg("names"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, removeEntry, 5))
      .def(
          "__len__", [](const AllowedCollisionMatrix &acm) { return acm.getSize(); },
          DOC(mplib, collision_detection, AllowedCollisionMatrix, getSize))

      .def("get_default_entry",
           py::overload_cast<const std::string &>(
               &AllowedCollisionMatrix::getDefaultEntry, py::const_),
           py::arg("name"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, getDefaultEntry))
      .def("has_default_entry", &AllowedCollisionMatrix::hasDefaultEntry,
           py::arg("name"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, hasDefaultEntry))
      .def("set_default_entry",
           py::overload_cast<const std::string &, bool>(
               &AllowedCollisionMatrix::setDefaultEntry),
           py::arg("name"), py::arg("allowed"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, setDefaultEntry))
      .def("set_default_entry",
           py::overload_cast<const std::vector<std::string> &, bool>(
               &AllowedCollisionMatrix::setDefaultEntry),
           py::arg("names"), py::arg("allowed"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, setDefaultEntry, 2))
      .def("remove_default_entry",
           py::overload_cast<const std::string &>(
               &AllowedCollisionMatrix::removeDefaultEntry),
           py::arg("name"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, removeDefaultEntry))
      .def("remove_default_entry",
           py::overload_cast<const std::vector<std::string> &>(
               &AllowedCollisionMatrix::removeDefaultEntry),
           py::arg("names"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, removeDefaultEntry,
               2))

      .def("get_allowed_collision", &AllowedCollisionMatrix::getAllowedCollision,
           py::arg("name1"), py::arg("name2"),
           DOC(mplib, collision_detection, AllowedCollisionMatrix, getAllowedCollision))

      .def("clear", &AllowedCollisionMatrix::clear,
           DOC(mplib, collision_detection, AllowedCollisionMatrix, clear))

      .def("get_all_entry_names", &AllowedCollisionMatrix::getAllEntryNames,
           DOC(mplib, collision_detection, AllowedCollisionMatrix, getAllEntryNames))
      .def(
          "__str__",
          [](const AllowedCollisionMatrix &acm) {
            std::stringstream ss;
            acm.print(ss);
            return ss.str();
          },
          DOC(mplib, collision_detection, AllowedCollisionMatrix, print));
}

}  // namespace mplib::collision_detection
