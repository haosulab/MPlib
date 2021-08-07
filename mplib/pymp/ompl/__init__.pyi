import mplib.pymp.ompl
import typing
import mplib.pymp.planning_world
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "OMPLPlanner"
]


class OMPLPlanner():
    def __init__(self, world: mplib.pymp.planning_world.PlanningWorld) -> None: ...
    def plan(self, start_state: numpy.ndarray[numpy.float64, _Shape[m, 1]], goal_state: numpy.ndarray[numpy.float64, _Shape[m, 1]], planner_name: str = 'RRTConnect', time: float = 1.0, range: float = 0.0, verbose: bool = False) -> typing.Tuple[str, numpy.ndarray[numpy.float64, _Shape[m, n]]]: ...
    pass
