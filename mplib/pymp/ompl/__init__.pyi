import mplib.pymp.ompl
import typing
import mplib.pymp.planning_world
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "FixedJoint",
    "OMPLPlanner"
]


class FixedJoint():
    def __init__(self, articulation_idx: int, joint_idx: int, value: float) -> None: ...
    @property
    def articulation_idx(self) -> int:
        """
        :type: int
        """
    @articulation_idx.setter
    def articulation_idx(self, arg0: int) -> None:
        pass
    @property
    def joint_idx(self) -> int:
        """
        :type: int
        """
    @joint_idx.setter
    def joint_idx(self, arg0: int) -> None:
        pass
    @property
    def value(self) -> float:
        """
        :type: float
        """
    @value.setter
    def value(self, arg0: float) -> None:
        pass
    pass
class OMPLPlanner():
    def __init__(self, world: mplib.pymp.planning_world.PlanningWorld, robot_idx: int = 0) -> None: 
        """
            Args:
                world: planning world
            Returns:
                OMPLPlanner object
        """
    def plan(self, start_state: numpy.ndarray[numpy.float64, _Shape[m, 1]], goal_states: list[numpy.ndarray[numpy.float64, _Shape[m, 1]]], planner_name: str = 'RRTConnect', time: float = 1.0, range: float = 0.0, verbose: bool = False, fixed_joints: set[FixedJoint] = set(), no_simplification: bool = False, constraint_function: typing.Callable[[numpy.ndarray[numpy.float64, _Shape[m, 1]], numpy.ndarray[numpy.float64, _Shape[m, 1]]], None] = None, constraint_jacobian: typing.Callable[[numpy.ndarray[numpy.float64, _Shape[m, 1]], numpy.ndarray[numpy.float64, _Shape[m, 1]]], None] = None, constraint_tolerance: float = 0.001) -> tuple[str, numpy.ndarray[numpy.float64, _Shape[m, n]]]: 
        """
            Plan a path from start state to goal states.
            Args:
                start_state: start state of the movegroup joints
                goal_states: list of goal states. planner will stop when one of them is reached
                planner_name: name of the planner pick between {RRTConnect, RRT*}
                time: planning time limit
                range: planning range (for RRT family of planners and represents the maximum step size)
                verbose: print debug information
                fixed_joints: list of fixed joints not considered in planning for this particular call
                no_simplification: if true, the path will not be simplified (constained planning does not support simplification)
                constraint_function: a R^d to R^1 function that evals to 0 when constraint is satisfied. constraint ignored if fixed joints not empty
                constraint_jacobian: the jacobian of the constraint w.r.t. the joint angles
                constraint_tolerance: tolerance of what level of deviation from 0 is acceptable
            Returns:
                pair of planner status and path. If planner succeeds, status is "Exact solution."
        """
    def simplify_path(self, path: numpy.ndarray[numpy.float64, _Shape[m, n]]) -> numpy.ndarray[numpy.float64, _Shape[m, n]]: 
        """
            Args:
                path: path to be simplified (numpy array of shape (n, dim))
            Returns:
                simplified path
        """
    pass
