"""
Collision detection submodule
"""

from . import fcl

__all__ = ["WorldCollisionResult", "fcl"]

class WorldCollisionResult:
    """
    Result of the collision checking.
    """
    @property
    def collision_type(self) -> str:
        """
        type of the collision
        """
    @property
    def link_name1(self) -> str:
        """
        link name of the first object in collision
        """
    @property
    def link_name2(self) -> str:
        """
        link name of the second object in collision
        """
    @property
    def object_name1(self) -> str:
        """
        name of the first object
        """
    @property
    def object_name2(self) -> str:
        """
        name of the second object
        """
    @property
    def res(self) -> ...:
        """
        the fcl CollisionResult
        """
