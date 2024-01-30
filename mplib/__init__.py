"""
.. include:: ./README.md
"""

from importlib.metadata import version

from mplib.planner import Planner
from mplib.pymp import (
    ArticulatedModel,
    PlanningWorld,
    collision_detection,
    kinematics,
    planning,
)

# from .pymp import set_global_seed

__version__ = version("mplib")
