"""
OMPL submodule
"""

import typing

import numpy

__all__ = ["FixedJoint", "OMPLPlanner"]
M = typing.TypeVar("M", bound=int)
N = typing.TypeVar("N", bound=int)

class FixedJoint:
    """ """
    def __eq__(self, arg0: FixedJoint) -> bool: ...
    def __hash__(self) -> int: ...
    def __init__(self, articulation_idx: int, joint_idx: int, value: float) -> None: ...
    def __lt__(self, arg0: FixedJoint) -> bool: ...
    @property
    def articulation_idx(self) -> int: ...
    @property
    def joint_idx(self) -> int: ...
    @property
    def value(self) -> float: ...

class OMPLPlanner:
    """
    OMPL Planner
    """
    def __init__(self, world: ...) -> None:
        """
        Construct an OMPLPlanner from a PlanningWorld

        :param world: planning world
        """
    def plan(
        self,
        start_state: numpy.ndarray[
            tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        goal_states: list[
            numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
        ],
        *,
        time: float = 1.0,
        range: float = 0.0,
        fixed_joints: set[...] = set(),
        simplify: bool = True,
        constraint_function: typing.Callable[
            [
                numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
                numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
            ],
            None,
        ] = None,
        constraint_jacobian: typing.Callable[
            [
                numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
                numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
            ],
            None,
        ] = None,
        constraint_tolerance: float = 0.001,
        verbose: bool = False,
    ) -> tuple[str, numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]]:
        """
        Plan a path from start state to goal states.

        :param start_state: start state of the movegroup joints
        :param goal_states: list of goal states. Planner will stop when one of them is
            reached
        :param time: planning time limit
        :param range: planning range (for RRT family of planners and represents the
            maximum step size)
        :param fixed_joints: list of fixed joints not considered in planning for this
            particular call
        :param simplify: whether the path will be simplified by calling
            ``_simplifyPath()`` (constained planning does not support simplification)
        :param constraint_function: a R^d to R^1 function that evals to 0 when
            constraint is satisfied. Constraint ignored if fixed joints not empty
        :param constraint_jacobian: the jacobian of the constraint w.r.t. the joint
            angles
        :param constraint_tolerance: tolerance of what level of deviation from 0 is
            acceptable
        :param verbose: print debug information. Default: ``False``.
        :return: pair of planner status and path. If planner succeeds, status is "Exact
            solution."
        """
    def simplify_path(
        self, path: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]
    ) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Simplify the provided path.

        :param path: path to be simplified (numpy array of shape (n, dim))
        :return: simplified path
        """
