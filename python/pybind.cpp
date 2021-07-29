#include <pybind11/pybind11.h>
#include "pybind_fcl.hpp"
#include "pybind_pinocchio.hpp"
#include "pybind_articulation.hpp"
#include "pybind_planning_world.hpp"
#include "pybind_ompl.hpp"
#include "pybind_kdl.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_mplib, m)
{
    m.doc() = "Motion planning python binding";
    build_pyfcl(m);
    build_pypinocchio(m);
    build_pykdl(m);
    build_pyarticulation(m);
    build_planning_world(m);
    build_pyompl(m);
    //build_pytopp(m);
}


