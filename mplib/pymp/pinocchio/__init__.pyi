import mplib.pymp.pinocchio
import typing
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "PinocchioModel"
]


class PinocchioModel():
    def __init__(self, urdf_filename: str, gravity: numpy.ndarray[numpy.float64, _Shape[3, 1]] = array([ 0. , 0. , -9.81]), verbose: bool = True) -> None: 
        """
             Args:
                  urdf_filename: path to the urdf file
                  gravity: gravity vector
                  verbose: print debug information
             Returns:
                  PinocchioModel object
        """
    def compute_IK_CLIK(self, index: int, pose: numpy.ndarray[numpy.float64, _Shape[7, 1]], q_init: numpy.ndarray[numpy.float64, _Shape[m, 1]], mask: list[bool] = [], eps: float = 1e-05, maxIter: int = 1000, dt: float = 0.1, damp: float = 1e-12) -> tuple[numpy.ndarray[numpy.float64, _Shape[m, 1]], bool, numpy.ndarray[numpy.float64, _Shape[6, 1]]]: 
        """
             Compute the inverse kinematics using close loop inverse kinematics.
             Args:
                  index: index of the link (in the order you passed to the constructor or the default order)
                  pose: desired pose of the link [x,y,z,qw,qx,qy,qz]
                  q_init: initial joint configuration
                  mask: if the value at a given index is True, the joint is *not* used in the IK
                  eps: tolerance for the IK
                  maxIter: maximum number of iterations
                  dt: time step for the CLIK
                  damp: damping for the CLIK
             Returns:
                  joint configuration
        """
    def compute_IK_CLIK_JL(self, index: int, pose: numpy.ndarray[numpy.float64, _Shape[7, 1]], q_init: numpy.ndarray[numpy.float64, _Shape[m, 1]], q_min: numpy.ndarray[numpy.float64, _Shape[m, 1]], q_max: numpy.ndarray[numpy.float64, _Shape[m, 1]], eps: float = 1e-05, maxIter: int = 1000, dt: float = 0.1, damp: float = 1e-12) -> tuple[numpy.ndarray[numpy.float64, _Shape[m, 1]], bool, numpy.ndarray[numpy.float64, _Shape[6, 1]]]: 
        """
             The same as compute_IK_CLIK but with it clamps the joint configuration to the given limits.
             Args:
                  index: index of the link (in the order you passed to the constructor or the default order)
                  pose: desired pose of the link [x,y,z,qw,qx,qy,qz]
                  q_init: initial joint configuration
                  q_min: minimum joint configuration
                  q_max: maximum joint configuration
                  eps: tolerance for the IK
                  maxIter: maximum number of iterations
                  dt: time step for the CLIK
                  damp: damping for the CLIK
             Returns:
                  joint configuration
        """
    def compute_forward_kinematics(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None: 
        """
             Compute forward kinematics for the given joint configuration.
             Args:
                  qpos: joint configuration. Needs to be full configuration, not just the movegroup joints.
             Returns:
                  None. If you want the result you need to call get_link_pose
        """
    def compute_full_jacobian(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None: 
        """
             Compute the full jacobian for the given joint configuration.
             Args:
                  qpos: joint configuration. Needs to be full configuration, not just the movegroup joints.
             Returns:
                  None. If you want the result you need to call get_link_jacobian
        """
    def compute_single_link_local_jacobian(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]], index: int) -> numpy.ndarray[numpy.float64, _Shape[6, n]]: 
        """
             Compute the jacobian of the given link.
             Args:
                  qpos: joint configuration. Needs to be full configuration, not just the movegroup joints.
                  index: index of the link (in the order you passed to the constructor or the default order)
             Returns:
                  6 x n jacobian of the link
        """
    def get_chain_joint_index(self, end_effector: str) -> list[int]: 
        """
             Get the joint indices of the joints in the chain from the root to the given link.
             Args:
                  index: index of the link (in the order you passed to the constructor or the default order)
             Returns:
                  joint indices of the joints in the chain
        """
    def get_chain_joint_name(self, end_effector: str) -> list[str]: 
        """
             Get the joint names of the joints in the chain from the root to the given link.
             Args:
                  index: index of the link (in the order you passed to the constructor or the default order)
             Returns:
                  joint names of the joints in the chain
        """
    def get_joint_dim(self, index: int, user: bool = True) -> int: ...
    def get_joint_dims(self, user: bool = True) -> numpy.ndarray[numpy.int32, _Shape[m, 1]]: ...
    def get_joint_ids(self, user: bool = True) -> numpy.ndarray[numpy.int32, _Shape[m, 1]]: 
        """
             Get the id of the all the joints. Again, Pinocchio might split a joint into multiple joints.
             Args:
                  user: if True, we get the id of the joints in the order you passed to the constructor or the default order
             Returns:
                  ids of the joint
        """
    def get_joint_limits(self, user: bool = True) -> list[numpy.ndarray[numpy.float64, _Shape[m, n]]]: ...
    def get_joint_names(self, user: bool = True) -> list[str]: ...
    def get_joint_types(self, user: bool = True) -> list[str]: ...
    def get_leaf_links(self) -> list[str]: 
        """
             Get the leaf links (links without child) of the kinematic tree.
             Returns:
                  list of leaf links
        """
    def get_link_jacobian(self, index: int, local: bool = False) -> numpy.ndarray[numpy.float64, _Shape[6, n]]: 
        """
             Get the jacobian of the given link.
             Args:
                  index: index of the link (in the order you passed to the constructor or the default order)
                  local: if True, the jacobian is expressed in the local frame of the link, otherwise it is expressed in the world frame
             Returns:
                  6 x n jacobian of the link
        """
    def get_link_names(self, user: bool = True) -> list[str]: ...
    def get_link_pose(self, index: int) -> numpy.ndarray[numpy.float64, _Shape[7, 1]]: 
        """
             Get the pose of the given link.
             Args:
                  index: index of the link (in the order you passed to the constructor or the default order)
             Returns:
                  pose of the link [x,y,z,qw,qx,qy,qz]
        """
    def get_parents(self, user: bool = True) -> numpy.ndarray[numpy.int32, _Shape[m, 1]]: 
        """
             Get the parent of the all the joints. Again, Pinocchio might split a joint into multiple joints.
             Args:
                  user: if True, we get the parent of the joints in the order you passed to the constructor or the default order
             Returns:
                  parents of the joints
        """
    def get_random_configuration(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]: 
        """
             Get a random configuration.
             Returns:
                  random joint configuration
        """
    def print_frames(self) -> None: ...
    def set_joint_order(self, names: list[str]) -> None: 
        """
             Pinocchio might have a different joint order or it might add additional joints.
             If you do not pass the the list of joint names, the default order might not be the one you want.
             Args:
                  names: list of joint names in the order you want
        """
    def set_link_order(self, names: list[str]) -> None: 
        """
             Pinocchio might have a different link order or it might add additional links.
             If you do not pass the the list of link names, the default order might not be the one you want.
             Args:
                  names: list of link names in the order you want
        """
    pass
