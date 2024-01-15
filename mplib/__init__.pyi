"""

.. include:: ./README.md
"""
from importlib.metadata import version
from mplib.planner import Planner
from . import planner
from . import pymp
__all__ = ['Planner', 'planner', 'pymp', 'version']
__version__: str = '0.0.9.dev20240115+git.3449b367.dirty'
