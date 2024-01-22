"""
.. include:: ./README.md
"""

from importlib.metadata import version

from mplib.planner import Planner
from mplib.pymp import articulation, fcl, kdl, ompl, pinocchio, planning_world

# from .pymp import set_global_seed

__version__ = version("mplib")
