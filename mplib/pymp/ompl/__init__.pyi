import mplib.pymp.ompl
import typing
import mplib.pymp.planning_world
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "OMPLPlanner"
]


class OMPLPlanner():
    def __init__(self, world: mplib.pymp.planning_world.PlanningWorld, robot_idx: int = 0) -> None: 
        """
            Args:
                world: planning world
            Returns:
                OMPLPlanner object
        """
    def plan(self, start_state: numpy.ndarray[numpy.float64, _Shape[m, 1]], goal_states: list[numpy.ndarray[numpy.float64, _Shape[m, 1]]], planner_name: str = 'RRTConnect', time: float = 1.0, range: float = 0.0, verbose: bool = False, align_axis: numpy.ndarray[numpy.float64, _Shape[3, 1]] = array([0., 0., 0.]), align_angle: float = 0.0, no_simplification: bool = False) -> tuple[str, numpy.ndarray[numpy.float64, _Shape[m, n]]]: 
        """
            Plan a path from start state to goal states.
            Args:
                start_state: start state of the movegroup joints
                goal_states: list of goal states. planner will stop when one of them is reached
                planner_name: name of the planner pick between {RRTConnect, RRT*}
                time: planning time limit
                range: planning range (for RRT family of planners and represents the maximum step size)
                verbose: print debug information
                align_axis: axis to align the end effector z-axis to
                align_angle: angle between the end effector z-axis and the align_axis
                no_simplification: if true, the path will not be simplified
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
