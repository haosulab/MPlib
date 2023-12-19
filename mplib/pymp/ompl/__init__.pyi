import mplib.pymp.ompl
import typing
import mplib.pymp.planning_world
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "OMPLPlanner"
]


class OMPLPlanner():
    def __init__(self, world: mplib.pymp.planning_world.PlanningWorld) -> None: 
        """
            Args:
                world: planning world
            Returns:
                OMPLPlanner object
        """
    def plan(self, start_state: numpy.ndarray[numpy.float64, _Shape[m, 1]], goal_states: list[numpy.ndarray[numpy.float64, _Shape[m, 1]]], planner_name: str = 'RRTConnect', time: float = 1.0, range: float = 0.0, verbose: bool = False) -> tuple[str, numpy.ndarray[numpy.float64, _Shape[m, n]]]: 
        """
            Plan a path from start state to goal states.
            Args:
                start_state: start state of the movegroup joints
                goal_states: list of goal states. planner will stop when one of them is reached
                planner_name: name of the planner pick between {RRTConnect, RRT*}
                time: planning time limit
                range: planning range (for RRT family of planners and represents the maximum step size)
                verbose: print debug information
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
