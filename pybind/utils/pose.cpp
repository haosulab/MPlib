#include "mplib/utils/pose.h"

#include <memory>
#include <stdexcept>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include "docstring/utils/pose.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib {

using Matrix4 = Eigen::Matrix<S, 4, 4, Eigen::RowMajor>;
using pyarray_t = py::array_t<S, py::array::c_style | py::array::forcecast>;

void build_utils_pose(py::module &pymp) {
  auto PyPose =
      py::class_<Pose<S>, std::shared_ptr<Pose<S>>>(pymp, "Pose", DOC(mplib, Pose));

  // Enable implicit conversions on the Python side from any py::object to Pose<S>
  py::implicitly_convertible<py::object, Pose<S>>();

  PyPose.def(py::init<>(), DOC(mplib, Pose, Pose))
      .def(py::init<const Vector3<S> &, const Vector4<S> &>(),
           py::arg("p") = Vector3<S> {0.0, 0.0, 0.0},
           py::arg("q") = Vector4<S> {1.0, 0.0, 0.0, 0.0}, DOC(mplib, Pose, Pose, 2))
      .def(py::init([](const Matrix4 &mat) {
             return Pose<S> {mat.block<3, 1>(0, 3),
                             Quaternion<S>(mat.block<3, 3>(0, 0))};
           }),
           py::arg("matrix"), DOC(mplib, Pose, Pose, 5))
      .def(py::init([](const py::object &obj) {
             if (py::hasattr(obj, "p") && py::hasattr(obj, "q")) {
               auto p = pyarray_t::ensure(obj.attr("p"));
               auto q = pyarray_t::ensure(obj.attr("q"));

               if (!p || !q)
                 throw std::runtime_error(
                     "Failed to convert 'p' and 'q' to numpy arrays");

               if (p.ndim() != 1 || p.size() != 3)
                 throw std::range_error("'p' must be 1D array of size 3");
               if (q.ndim() != 1 || q.size() != 4)
                 throw std::range_error("'q' must be 1D array of size 4");

               return Pose<S>(Vector3<S>(static_cast<S *>(p.request().ptr)),
                              Vector4<S>(static_cast<S *>(q.request().ptr)));
             }

             auto mat = pyarray_t::ensure(obj);
             if (!mat)
               throw std::runtime_error(
                   "Unknown object type, cannot contruct a Pose instance from it!");

             if (mat.ndim() != 2 || mat.shape(0) != 4 || mat.shape(1) != 4)
               throw std::range_error("Input must be 2D array of shape (4, 4)");

             auto eigen_mat = Matrix4(static_cast<S *>(mat.request().ptr));
             return Pose<S> {eigen_mat.block<3, 1>(0, 3),
                             Quaternion<S>(eigen_mat.block<3, 3>(0, 0))};
           }),
           py::arg("obj"), DOC(mplib, Pose, Pose, 6))
      .def(
          "to_transformation_matrix",
          [](const Pose<S> &pose) {
            Matrix4 mat = Matrix4::Identity();
            mat.block<3, 3>(0, 0) = pose.q.toRotationMatrix();
            mat.block<3, 1>(0, 3) = pose.p;
            return mat;
          },
          DOC(mplib, Pose, to_transformation_matrix))

      .def_readwrite("p", &Pose<S>::p, DOC(mplib, Pose, p))
      .def(
          "set_p", [](Pose<S> &pose, const Vector3<S> &p) { pose.p = p; }, py::arg("p"),
          DOC(mplib, Pose, set_p))
      .def(
          "get_p", [](const Pose<S> &pose) { return pose.p; }, DOC(mplib, Pose, get_p))

      .def_property(
          "q",
          [](const Pose<S> &pose) {
            return Vector4<S> {pose.q.w(), pose.q.x(), pose.q.y(), pose.q.z()};
          },
          [](Pose<S> &pose, const Vector4<S> &q) {
            pose.q = Quaternion<S> {q(0), q(1), q(2), q(3)}.normalized();
          },
          DOC(mplib, Pose, q))
      .def(
          "set_q",
          [](Pose<S> &pose, const Vector4<S> &q) {
            pose.q = Quaternion<S> {q(0), q(1), q(2), q(3)}.normalized();
          },
          py::arg("q"), DOC(mplib, Pose, set_q))
      .def(
          "get_q",
          [](const Pose<S> &pose) {
            return Vector4<S> {pose.q.w(), pose.q.x(), pose.q.y(), pose.q.z()};
          },
          DOC(mplib, Pose, get_q))

      .def("inv", &Pose<S>::inverse, DOC(mplib, Pose, inverse))
      .def("distance", &Pose<S>::distance, py::arg("other"), DOC(mplib, Pose, distance))
      .def(py::self * Vector3<S>(), py::arg("v"), DOC(mplib, Pose, operator_mul))
      .def(py::self * py::self, py::arg("other"), DOC(mplib, Pose, operator_mul, 2))
      .def(py::self *= py::self, py::arg("other"), DOC(mplib, Pose, operator_imul))

      .def("__repr__",
           [](const Pose<S> &pose) {
             std::ostringstream oss;
             oss << pose;
             return oss.str();
           })
      .def(py::pickle(
          [](const Pose<S> &p) {  // __getstate__
            return py::make_tuple(p.p(0), p.p(1), p.p(2), p.q.w(), p.q.x(), p.q.y(),
                                  p.q.z());
          },
          [](py::tuple t) {  // __setstate__
            if (t.size() != 7) {
              throw std::runtime_error("Invalid state!");
            }
            return Pose<S> {{t[0].cast<S>(), t[1].cast<S>(), t[2].cast<S>()},
                            Quaternion<S> {t[3].cast<S>(), t[4].cast<S>(),
                                           t[5].cast<S>(), t[6].cast<S>()}};
          }));
}

}  // namespace mplib
