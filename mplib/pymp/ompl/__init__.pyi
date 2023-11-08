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
    def __init__(self, world: mplib.pymp.planning_world.PlanningWorld) -> None: ...
    def plan(self, start_state: numpy.ndarray[numpy.float64, _Shape[m, 1]], goal_states: list[numpy.ndarray[numpy.float64, _Shape[m, 1]]], planner_name: str = 'RRTConnect', time: float = 1.0, range: float = 0.0, verbose: bool = False, fixed_joints: set[FixedJoint] = set()) -> tuple[str, numpy.ndarray[numpy.float64, _Shape[m, n]]]: ...
    pass
