"""
articulated model submodule, i.e. models with moving parts
"""

import typing

import numpy

import mplib.pymp.fcl
import mplib.pymp.pinocchio

__all__ = ["ArticulatedModel"]
M = typing.TypeVar("M", bound=int)

class ArticulatedModel:
    """
    Supports initialization from URDF and SRDF files, and provides access to
    underlying Pinocchio and FCL models.
    """
    def __init__(
        self,
        urdf_filename: str,
        srdf_filename: str,
        gravity: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        link_names: list[str],
        joint_names: list[str],
        convex: bool = False,
        verbose: bool = False,
    ) -> None:
        """
        Construct an articulated model from URDF and SRDF files.

        :param urdf_filename: path to URDF file, can be relative to the current working
            directory
        :param srdf_filename: path to SRDF file, we use it to disable self-collisions
        :param gravity: gravity vector
        :param link_names: list of links that are considered for planning
        :param joint_names: list of joints that are considered for planning
        :param convex: use convex decomposition for collision objects. Default:
            ``False``.
        :param verbose: print debug information. Default: ``False``.
        """
    def get_base_pose(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]:
        """
        Get the base pose of the robot.

        :return: base pose of the robot in [x, y, z, qw, qx, qy, qz] format
        """
    def get_fcl_model(self) -> mplib.pymp.fcl.FCLModel:
        """
        Get the underlying FCL model.

        :return: FCL model used for collision checking
        """
    def get_move_group_end_effectors(self) -> list[str]:
        """
        Get the end effectors of the move group.

        :return: list of end effectors of the move group
        """
    def get_move_group_joint_indices(self) -> list[int]:
        """
        Get the joint indices of the move group.

        :return: list of user joint indices of the move group
        """
    def get_move_group_joint_names(self) -> list[str]:
        """
        Get the joint names of the move group.

        :return: list of joint names of the move group
        """
    def get_move_group_qpos_dim(self) -> int:
        """
        Get the dimension of the move group qpos.

        :return: dimension of the move group qpos
        """
    def get_pinocchio_model(self) -> mplib.pymp.pinocchio.PinocchioModel:
        """
        Get the underlying Pinocchio model.

        :return: Pinocchio model used for kinematics and dynamics computations
        """
    def get_qpos(
        self,
    ) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Get the current joint position of all active joints inside the URDF.

        :return: current qpos of all active joints
        """
    def get_user_joint_names(self) -> list[str]:
        """
        Get the joint names that the user has provided for planning.

        :return: list of joint names of the user
        """
    def get_user_link_names(self) -> list[str]:
        """
        Get the link names that the user has provided for planning.

        :return: list of link names of the user
        """
    def set_base_pose(
        self,
        pose: numpy.ndarray[
            tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Set the base pose of the robot.

        :param pose: base pose of the robot in [x, y, z, qw, qx, qy, qz] format
        """
    @typing.overload
    def set_move_group(self, end_effector: str) -> None:
        """
        Set the move group, i.e. the chain ending in end effector for which to compute
        the forward kinematics for all subsequent queries.

        :param chain: list of links extending to the end effector
        """
    @typing.overload
    def set_move_group(self, end_effectors: list[str]) -> None:
        """
        Set the move group but we have multiple end effectors in a chain. I.e., Base -->
        EE1 --> EE2 --> ... --> EEn

        :param end_effectors: names of the end effector link
        """
    def set_qpos(
        self,
        qpos: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        full: bool = False,
    ) -> None:
        """
        Let the planner know the current joint positions.

        :param qpos: current qpos of all active joints or just the move group joints
        :param full: whether to set the full qpos or just the move group qpos. If full
            is ``False``, we will pad the missing joints with current known qpos. The
            default is ``False``
        """
    def update_SRDF(self, SRDF: str) -> None:
        """
        Update the SRDF file to disable self-collisions.

        :param srdf: path to SRDF file, can be relative to the current working directory
        """
