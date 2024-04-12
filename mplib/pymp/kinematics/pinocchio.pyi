"""
Pinocchio submodule
"""

import typing

import numpy

import mplib.pymp

__all__ = ["PinocchioModel"]
M = typing.TypeVar("M", bound=int)
N = typing.TypeVar("N", bound=int)

class PinocchioModel:
    """
    Pinocchio model of an articulation

    See https://github.com/stack-of-tasks/pinocchio
    """
    @staticmethod
    def create_from_urdf_string(
        urdf_string: str,
        *,
        gravity: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ] = ...,
        verbose: bool = False,
    ) -> PinocchioModel:
        """
        Constructs a PinocchioModel from URDF string

        :param urdf_string: URDF string (without visual/collision elements for links)
        :param gravity: gravity vector, by default is ``[0, 0, -9.81]`` in -z axis
        :param verbose: print debug information. Default: ``False``.
        :return: a unique_ptr to PinocchioModel
        """
    def __init__(
        self,
        urdf_filename: str,
        *,
        gravity: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ] = ...,
        verbose: bool = False,
    ) -> None:
        """
        Construct a Pinocchio model from the given URDF file.

        :param urdf_filename: path to the URDF file
        :param gravity: gravity vector, by default is ``[0, 0, -9.81]`` in -z axis
        :param verbose: print debug information. Default: ``False``.
        """
    def compute_IK_CLIK(
        self,
        index: int,
        pose: mplib.pymp.Pose,
        q_init: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        mask: list[bool] = [],
        eps: float = 1e-05,
        max_iter: int = 1000,
        dt: float = 0.1,
        damp: float = 1e-12,
    ) -> tuple[
        numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        bool,
        numpy.ndarray[
            tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ]:
        """
        Compute the inverse kinematics using close loop inverse kinematics.

        :param index: index of the link (in the order you passed to the constructor or
            the default order)
        :param pose: desired pose of the link
        :param q_init: initial joint configuration
        :param mask: if the value at a given index is ``True``, the joint is *not* used
            in the IK
        :param eps: tolerance for the IK
        :param max_iter: maximum number of iterations
        :param dt: time step for the CLIK
        :param damp: damping for the CLIK
        :return: joint configuration
        """
    def compute_IK_CLIK_JL(
        self,
        index: int,
        pose: mplib.pymp.Pose,
        q_init: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        q_min: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        q_max: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        eps: float = 1e-05,
        max_iter: int = 1000,
        dt: float = 0.1,
        damp: float = 1e-12,
    ) -> tuple[
        numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        bool,
        numpy.ndarray[
            tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ]:
        """
        The same as ``compute_IK_CLIK()`` but with it clamps the joint configuration to
        the given limits.

        :param index: index of the link (in the order you passed to the constructor or
            the default order)
        :param pose: desired pose of the link
        :param q_init: initial joint configuration
        :param q_min: minimum joint configuration
        :param q_max: maximum joint configuration
        :param eps: tolerance for the IK
        :param max_iter: maximum number of iterations
        :param dt: time step for the CLIK
        :param damp: damping for the CLIK
        :return: joint configuration
        """
    def compute_forward_kinematics(
        self,
        qpos: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
    ) -> None:
        """
        Compute forward kinematics for the given joint configuration.

        If you want the result you need to call ``get_link_pose()``

        :param qpos: joint configuration. Needs to be full configuration, not just the
            movegroup joints.
        """
    def compute_full_jacobian(
        self,
        qpos: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
    ) -> None:
        """
        Compute the full jacobian for the given joint configuration. Note you need to
        call computeForwardKinematics() first. If you want the result you need to call
        ``get_link_jacobian()``

        :param qpos: joint configuration. Needs to be full configuration, not just the
            movegroup joints.
        """
    def compute_single_link_jacobian(
        self,
        qpos: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        index: int,
        local: bool = False,
    ) -> numpy.ndarray[tuple[typing.Literal[6], N], numpy.dtype[numpy.float64]]:
        """
        Compute the jacobian of the given link. Note you need to call
        computeForwardKinematics() first.

        :param qpos: joint configuration. Needs to be full configuration, not just the
            movegroup joints.
        :param index: index of the link (in the order you passed to the constructor or
            the default order)
        :param local: if ``True`` return the jacobian w.r.t. the instantaneous local
            frame of the link
        :return: 6 x n jacobian of the link
        """
    def get_adjacent_links(self) -> set[tuple[str, str]]:
        """
        Get the all adjacent link names.

        :return: adjacent link names as a set of pairs of strings
        """
    def get_chain_joint_index(self, end_effector: str) -> list[int]:
        """
        Get the joint indices of the joints in the chain from the root to the given
        link.

        :param index: index of the link (in the order you passed to the constructor or
            the default order)
        :return: joint indices of the joints in the chain
        """
    def get_chain_joint_name(self, end_effector: str) -> list[str]:
        """
        Get the joint names of the joints in the chain from the root to the given link.

        :param index: index of the link (in the order you passed to the constructor or
            the default order)
        :return: joint names of the joints in the chain
        """
    def get_joint_dim(self, index: int, user: bool = True) -> int:
        """
        Get the dimension of the joint with the given index.

        :param index: joint index to query
        :param user: if ``True``, the joint index follows the order you passed to the
            constructor or the default order
        :return: dimension of the joint with the given index
        """
    def get_joint_dims(
        self, user: bool = True
    ) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.int32]]:
        """
        Get the dimension of all the joints. Again, Pinocchio might split a joint into
        multiple joints.

        :param user: if ``True``, we get the dimension of the joints in the order you
            passed to the constructor or the default order
        :return: dimention of the joints
        """
    def get_joint_id(self, index: int, user: bool = True) -> int:
        """
        Get the id of the joint with the given index.

        :param index: joint index to query
        :param user: if ``True``, the joint index follows the order you passed to the
            constructor or the default order
        :return: id of the joint with the given index
        """
    def get_joint_ids(
        self, user: bool = True
    ) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.int32]]:
        """
        Get the id of all the joints. Again, Pinocchio might split a joint into multiple
        joints.

        :param user: if ``True``, we get the id of the joints in the order you passed to
            the constructor or the default order
        :return: id of the joints
        """
    def get_joint_limit(
        self, index: int, user: bool = True
    ) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Get the limit of the joint with the given index.

        :param index: joint index to query
        :param user: if ``True``, the joint index follows the order you passed to the
            constructor or the default order
        :return: limit of the joint with the given index
        """
    def get_joint_limits(
        self, user: bool = True
    ) -> list[numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]]:
        """
        Get the limit of all the joints. Again, Pinocchio might split a joint into
        multiple joints.

        :param user: if ``True``, we get the limit of the joints in the order you passed
            to the constructor or the default order
        :return: limit of the joints
        """
    def get_joint_names(self, user: bool = True) -> list[str]:
        """
        Get the name of all the joints. Again, Pinocchio might split a joint into
        multiple joints.

        :param user: if ``True``, we get the name of the joints in the order you passed
            to the constructor or the default order
        :return: name of the joints
        """
    def get_joint_parent(self, index: int, user: bool = True) -> int:
        """
        Get the parent of the joint with the given index.

        :param index: joint index to query
        :param user: if ``True``, the joint index follows the order you passed to the
            constructor or the default order
        :return: parent of the joint with the given index
        """
    def get_joint_parents(
        self, user: bool = True
    ) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.int32]]:
        """
        Get the parent of all the joints. Again, Pinocchio might split a joint into
        multiple joints.

        :param user: if ``True``, we get the parent of the joints in the order you
            passed to the constructor or the default order
        :return: parent of the joints
        """
    def get_joint_type(self, index: int, user: bool = True) -> str:
        """
        Get the type of the joint with the given index.

        :param index: joint index to query
        :param user: if ``True``, the joint index follows the order you passed to the
            constructor or the default order
        :return: type of the joint with the given index
        """
    def get_joint_types(self, user: bool = True) -> list[str]:
        """
        Get the type of all the joints. Again, Pinocchio might split a joint into
        multiple joints.

        :param user: if ``True``, we get the type of the joints in the order you passed
            to the constructor or the default order
        :return: type of the joints
        """
    def get_leaf_links(self) -> list[str]:
        """
        Get the leaf links (links without child) of the kinematic tree.

        :return: list of leaf links
        """
    def get_link_jacobian(
        self, index: int, local: bool = False
    ) -> numpy.ndarray[tuple[typing.Literal[6], N], numpy.dtype[numpy.float64]]:
        """
        Get the jacobian of the given link. You must call ``compute_full_jacobian()``
        first.

        :param index: index of the link (in the order you passed to the constructor or
            the default order)
        :param local: if ``True``, the jacobian is expressed in the local frame of the
            link, otherwise it is expressed in the world frame
        :return: 6 x n jacobian of the link
        """
    def get_link_names(self, user: bool = True) -> list[str]:
        """
        Get the name of all the links.

        :param user: if ``True``, we get the name of the links in the order you passed
            to the constructor or the default order
        :return: name of the links
        """
    def get_link_pose(self, index: int) -> mplib.pymp.Pose:
        """
        Get the pose of the given link in robot's base (root) frame.

        :param index: index of the link (in the order you passed to the constructor or
            the default order)
        :return: pose of the link in robot's base (root) frame.
        """
    def get_random_configuration(
        self,
    ) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Get a random configuration.

        :return: random joint configuration
        """
    def print_frames(self) -> None:
        """
        Frame is a Pinocchio internal data type which is not supported outside this
        class.
        """
    def set_joint_order(self, names: list[str]) -> None:
        """
        Pinocchio might have a different joint order or it might add additional joints.

        If you do not pass the the list of joint names, the default order might not be
        the one you want.

        :param names: list of joint names in the order you want
        """
    def set_link_order(self, names: list[str]) -> None:
        """
        Pinocchio might have a different link order or it might add additional links.

        If you do not pass the the list of link names, the default order might not be
        the one you want.

        :param names: list of link names in the order you want
        """
