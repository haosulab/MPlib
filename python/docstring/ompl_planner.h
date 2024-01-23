#pragma once

/*
  This file contains docstrings for use in the Python bindings.
  Do not edit! They were automatically extracted by mkdoc.py.
 */

#define __EXPAND(x)                                      x
#define __COUNT(_1, _2, _3, _4, _5, _6, _7, COUNT, ...)  COUNT
#define __VA_SIZE(...)                                   __EXPAND(__COUNT(__VA_ARGS__, 7, 6, 5, 4, 3, 2, 1, 0))
#define __CAT1(a, b)                                     a ## b
#define __CAT2(a, b)                                     __CAT1(a, b)
#define __DOC1(n1)                                       __doc_##n1
#define __DOC2(n1, n2)                                   __doc_##n1##_##n2
#define __DOC3(n1, n2, n3)                               __doc_##n1##_##n2##_##n3
#define __DOC4(n1, n2, n3, n4)                           __doc_##n1##_##n2##_##n3##_##n4
#define __DOC5(n1, n2, n3, n4, n5)                       __doc_##n1##_##n2##_##n3##_##n4##_##n5
#define __DOC6(n1, n2, n3, n4, n5, n6)                   __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6
#define __DOC7(n1, n2, n3, n4, n5, n6, n7)               __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6##_##n7
#define DOC(...)                                         __EXPAND(__EXPAND(__CAT2(__DOC, __VA_SIZE(__VA_ARGS__)))(__VA_ARGS__))

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

static const char *__doc_mplib_ompl_FixedJointTpl = R"doc()doc";

static const char *__doc_mplib_ompl_FixedJointTpl_FixedJointTpl =
R"doc(
)doc";

static const char *__doc_mplib_ompl_FixedJointTpl_articulation_idx = R"doc()doc";

static const char *__doc_mplib_ompl_FixedJointTpl_joint_idx = R"doc()doc";

static const char *__doc_mplib_ompl_FixedJointTpl_operator_lt =
R"doc(
)doc";

static const char *__doc_mplib_ompl_FixedJointTpl_value = R"doc()doc";

static const char *__doc_mplib_ompl_GeneralConstraint = R"doc()doc";

static const char *__doc_mplib_ompl_GeneralConstraint_GeneralConstraint =
R"doc(
)doc";

static const char *__doc_mplib_ompl_GeneralConstraint_function =
R"doc(
)doc";

static const char *__doc_mplib_ompl_GeneralConstraint_jacobian =
R"doc(
)doc";

static const char *__doc_mplib_ompl_OMPLPlannerTpl = R"doc(OMPL Planner)doc";

static const char *__doc_mplib_ompl_OMPLPlannerTpl_OMPLPlannerTpl =
R"doc(
Construct an OMPLPlanner from a PlanningWorld

:param world: planning world)doc";

static const char *__doc_mplib_ompl_OMPLPlannerTpl_build_compound_state_space =
R"doc(
Build a new state space given the current planning world and a set of fixed
joints

:param fixed_joints: a vector of FixedJoint)doc";

static const char *__doc_mplib_ompl_OMPLPlannerTpl_get_dim =
R"doc(
)doc";

static const char *__doc_mplib_ompl_OMPLPlannerTpl_get_world =
R"doc(
)doc";

static const char *__doc_mplib_ompl_OMPLPlannerTpl_plan =
R"doc(
Plan a path from start state to goal states.

:param start_state: start state of the movegroup joints
:param goal_states: list of goal states. Planner will stop when one of them is
    reached
:param planner_name: name of the planner pick between {RRTConnect, RRT*}
:param time: planning time limit
:param range: planning range (for RRT family of planners and represents the
    maximum step size)
:param verbose: print debug information
:param fixed_joints: list of fixed joints not considered in planning for this
    particular call
:param no_simplification: if ``True``, the path will not be simplified
    (constained planning does not support simplification)
:param constraint_function: a R^d to R^1 function that evals to 0 when
    constraint is satisfied. Constraint ignored if fixed joints not empty
:param constraint_jacobian: the jacobian of the constraint w.r.t. the joint
    angles
:param constraint_tolerance: tolerance of what level of deviation from 0 is
    acceptable
:return: pair of planner status and path. If planner succeeds, status is "Exact
    solution.")doc";

static const char *__doc_mplib_ompl_OMPLPlannerTpl_random_sample_nearby =
R"doc(
)doc";

static const char *__doc_mplib_ompl_OMPLPlannerTpl_simplify_path =
R"doc(
Simplify the provided path.

:param path: path to be simplified (numpy array of shape (n, dim))
:return: simplified path)doc";

static const char *__doc_mplib_ompl_ValidityCheckerTpl = R"doc()doc";

static const char *__doc_mplib_ompl_ValidityCheckerTpl_ValidityCheckerTpl =
R"doc(
)doc";

static const char *__doc_mplib_ompl_ValidityCheckerTpl_isValid =
R"doc(
)doc";

static const char *__doc_mplib_ompl_ValidityCheckerTpl_isValid_2 =
R"doc(
)doc";

static const char *__doc_mplib_ompl_ValidityCheckerTpl_update_fixed_joints =
R"doc(
)doc";

static const char *__doc_mplib_ompl_add_fixed_joints =
R"doc(
)doc";

static const char *__doc_mplib_ompl_compoundstate2vector =
R"doc(
)doc";

static const char *__doc_mplib_ompl_eigen2vector =
R"doc(
)doc";

static const char *__doc_mplib_ompl_is_fixed_joint =
R"doc(
)doc";

static const char *__doc_mplib_ompl_remove_fixed_joints =
R"doc(
)doc";

static const char *__doc_mplib_ompl_rvssstate2vector =
R"doc(
)doc";

static const char *__doc_mplib_ompl_state2eigen =
R"doc(
)doc";

static const char *__doc_mplib_ompl_vector2eigen =
R"doc(
)doc";

/* ----- Begin of custom docstring section ----- */

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
