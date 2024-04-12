"""
KDL submodule
"""

import typing

import numpy

import mplib.pymp

__all__ = ["KDLModel"]
M = typing.TypeVar("M", bound=int)

class KDLModel:
    """
    KDL model of an articulation

    See https://github.com/orocos/orocos_kinematics_dynamics
    """
    def __init__(
        self,
        urdf_filename: str,
        link_names: list[str],
        joint_names: list[str],
        *,
        verbose: bool = False,
    ) -> None: ...
    def chain_IK_LMA(
        self,
        index: int,
        q_init: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        goal_pose: mplib.pymp.Pose,
    ) -> tuple[
        numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], int
    ]: ...
    def chain_IK_NR(
        self,
        index: int,
        q_init: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        goal_pose: mplib.pymp.Pose,
    ) -> tuple[
        numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], int
    ]: ...
    def chain_IK_NR_JL(
        self,
        index: int,
        q_init: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        goal_pose: mplib.pymp.Pose,
        q_min: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        q_max: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
    ) -> tuple[
        numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], int
    ]: ...
    def get_tree_root_name(self) -> str: ...
    def tree_IK_NR_JL(
        self,
        endpoints: list[str],
        q_init: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        goal_poses: list[mplib.pymp.Pose],
        q_min: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        q_max: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
    ) -> tuple[
        numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], int
    ]: ...
