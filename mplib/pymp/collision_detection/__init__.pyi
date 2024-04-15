"""
Collision detection submodule
"""

import typing

from . import fcl

__all__ = [
    "AllowedCollision",
    "AllowedCollisionMatrix",
    "WorldCollisionResult",
    "WorldDistanceResult",
    "fcl",
]

class AllowedCollision:
    """
    AllowedCollision Enum class

    Members:

      NEVER : Collision is never allowed

      ALWAYS : Collision is always allowed

      CONDITIONAL : Collision contact is allowed depending on a predicate
    """

    ALWAYS: typing.ClassVar[AllowedCollision]  # value = <AllowedCollision.ALWAYS: 1>
    CONDITIONAL: typing.ClassVar[
        AllowedCollision
    ]  # value = <AllowedCollision.CONDITIONAL: 2>
    NEVER: typing.ClassVar[AllowedCollision]  # value = <AllowedCollision.NEVER: 0>
    __members__: typing.ClassVar[
        dict[str, AllowedCollision]
    ]  # value = {'NEVER': <AllowedCollision.NEVER: 0>, 'ALWAYS': <AllowedCollision.ALWAYS: 1>, 'CONDITIONAL': <AllowedCollision.CONDITIONAL: 2>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class AllowedCollisionMatrix:
    """
    AllowedCollisionMatrix for collision checking

    All elements in the collision world are referred to by their names. This class
    represents which collisions are allowed to happen and which are not.

    Mimicking MoveIt2's ``collision_detection::AllowedCollisionMatrix``

    https://moveit.picknik.ai/main/api/html/classcollision__detection_1_1AllowedCollisionMatrix.html
    """
    def __init__(self) -> None: ...
    def __len__(self) -> int:
        """
        Get the size of the allowed collision matrix (number of entries)
        """
    def __str__(self) -> str:
        """
        Print the allowed collision matrix. "01?-" corresponds to NEVER / ALWAYS /
        CONDITIONAL / Entry not found
        """
    def clear(self) -> None:
        """
        Clear all data in the allowed collision matrix
        """
    def get_all_entry_names(self) -> list[str]:
        """
        Get sorted names of all existing elements (including default_entries_)
        """
    def get_allowed_collision(self, name1: str, name2: str) -> AllowedCollision | None:
        """
        Get the type of the allowed collision between two elements

        :return: AllowedCollision. This is * ``None`` if the entry does not exist
            (collision is not allowed) * the entry if an entry or a default entry
            exists.
        """
    def get_default_entry(self, name: str) -> AllowedCollision | None:
        """
        Get the default type of the allowed collision for an element

        :param name: name of the element
        :return: an AllowedCollision Enum or ``None`` if the default entry does not
            exist
        """
    def get_entry(self, name1: str, name2: str) -> AllowedCollision | None:
        """
        Get the type of the allowed collision between two elements

        :param name1: name of the first element
        :param name2: name of the second element
        :return: an AllowedCollision Enum or ``None`` if the entry does not exist.
        """
    def has_default_entry(self, name: str) -> bool:
        """
        Check if a default entry exists for an element
        """
    @typing.overload
    def has_entry(self, name: str) -> bool:
        """
        Check if an entry exists for an element
        """
    @typing.overload
    def has_entry(self, name1: str, name2: str) -> bool:
        """
        Check if an entry exists for a pair of elements
        """
    @typing.overload
    def remove_default_entry(self, name: str) -> None:
        """
        Remove the default entry for the element if exists
        """
    @typing.overload
    def remove_default_entry(self, names: list[str]) -> None:
        """
        Remove the existing default entries for the elements
        """
    @typing.overload
    def remove_entry(self, name1: str, name2: str) -> None:
        """
        Remove the entry for a pair of elements if exists
        """
    @typing.overload
    def remove_entry(self, name: str, other_names: list[str]) -> None:
        """
        Remove existing entries between the element and each element in other_names
        """
    @typing.overload
    def remove_entry(self, names1: list[str], names2: list[str]) -> None:
        """
        Remove any existing entries for all possible pairs among two sets of elements
        """
    @typing.overload
    def remove_entry(self, name: str) -> None:
        """
        Remove all entries for all possible pairs between the element and existing
        elements
        """
    @typing.overload
    def remove_entry(self, names: list[str]) -> None:
        """
        Remove all entries for all possible pairs between each of the elements and
        existing elements
        """
    @typing.overload
    def set_default_entry(self, name: str, allowed: bool) -> None:
        """
        Set the default value for entries that include name but are not set explicitly
        with setEntry(). Apply to future changes of the element set.
        """
    @typing.overload
    def set_default_entry(self, names: list[str], allowed: bool) -> None:
        """
        Set the default entries for the elements. Apply to future changes of the element
        set.
        """
    @typing.overload
    def set_entry(self, name1: str, name2: str, allowed: bool) -> None:
        """
        Set an entry for a pair of elements
        """
    @typing.overload
    def set_entry(self, name: str, other_names: list[str], allowed: bool) -> None:
        """
        Set the entries between the element and each element in other_names
        """
    @typing.overload
    def set_entry(self, names1: list[str], names2: list[str], allowed: bool) -> None:
        """
        Set the entries for all possible pairs among two sets of elements
        """
    @typing.overload
    def set_entry(self, name: str, allowed: bool) -> None:
        """
        Set the entries for all possible pairs between the element and existing
        elements. As the set of elements might change in the future, consider using
        setDefaultEntry() instead.
        """
    @typing.overload
    def set_entry(self, names: list[str], allowed: bool) -> None:
        """
        Set the entries for all possible pairs between each of the elements and existing
        elements. As the set of elements might change in the future, consider using
        setDefaultEntry() instead.
        """
    @typing.overload
    def set_entry(self, allowed: bool) -> None:
        """
        Set the entries for all possible pairs among all existing elements
        """

class WorldCollisionResult:
    """
    Result of the collision checking.
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor
        """
    @typing.overload
    def __init__(
        self,
        res: ...,
        collision_type: str,
        object_name1: str,
        object_name2: str,
        link_name1: str,
        link_name2: str,
    ) -> None:
        """
        Constructor with all members
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

class WorldDistanceResult:
    """
    Result of minimum distance-to-collision query.
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor
        """
    @typing.overload
    def __init__(
        self,
        res: ...,
        min_distance: float,
        distance_type: str,
        object_name1: str,
        object_name2: str,
        link_name1: str,
        link_name2: str,
    ) -> None:
        """
        Constructor with all members
        """
    @property
    def distance_type(self) -> str:
        """
        type of the distance result
        """
    @property
    def link_name1(self) -> str:
        """
        link name of the first object
        """
    @property
    def link_name2(self) -> str:
        """
        link name of the second object
        """
    @property
    def min_distance(self) -> float:
        """
        minimum distance between the two objects
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
        the fcl DistanceResult
        """
