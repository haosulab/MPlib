"""articulated model submodule, i.e. models with moving parts"""
import mplib.pymp.articulation
import typing
import mplib.pymp.fcl
import mplib.pymp.pinocchio
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "ArticulatedModel"
]


class ArticulatedModel():
    """
    Supports initialization from URDF and SRDF files, and provides access to underlying Pinocchio and FCL models.
    """
    def __init__(self, urdf_filename: str, srdf_filename: str, gravity: numpy.ndarray[numpy.float64, _Shape[3, 1]], joint_names: list[str], link_names: list[str], verbose: bool = True, convex: bool = False) -> None: 
        """
            Construct an articulated model from URDF and SRDF files.
            Args:
                urdf_filename: path to URDF file, can be relative to the current working directory
                srdf_filename: path to SRDF file, we use it to disable self-collisions
                gravity: gravity vector
                joint_names: list of joints that are considered for planning
                link_names: list of links that are considered for planning
                verbose: print debug information
                convex: use convex decomposition for collision objects
        """
    def get_base_pose(self) -> numpy.ndarray[numpy.float64, _Shape[7, 1]]: 
        """
            Get the base pose of the robot.
            Returns:
                base pose of the robot in [x, y, z, qw, qx, qy, qz] format
        """
    def get_fcl_model(self) -> mplib.pymp.fcl.FCLModel: 
        """
            Get the underlying FCL model.
            Returns:
                FCL model used for collision checking
        """
    def get_move_group_end_effectors(self) -> list[str]: 
        """
            Get the end effectors of the move group.
            Returns:
                list of end effectors of the move group
        """
    def get_move_group_joint_indices(self) -> list[int]: 
        """
            Get the joint indices of the move group.
            Returns:
                list of user joint indices of the move group
        """
    def get_move_group_joint_names(self) -> list[str]: 
        """
            Get the joint names of the move group.
            Returns:
                list of joint names of the move group
        """
    def get_move_group_qpos_dim(self) -> int: 
        """
            Get the dimension of the move group qpos.
            Returns:
                dimension of the move group qpos
        """
    def get_pinocchio_model(self) -> mplib.pymp.pinocchio.PinocchioModel: 
        """
            Get the underlying Pinocchio model.
            Returns:
                Pinocchio model used for kinematics and dynamics computations
        """
    def get_qpos(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]: 
        """
            Get the current joint position of all active joints inside the URDF.
            Returns:
                current qpos of all active joints
        """
    def get_user_joint_names(self) -> list[str]: 
        """
            Get the joint names that the user has provided for planning.
            Returns:
                list of joint names of the user
        """
    def get_user_link_names(self) -> list[str]: 
        """
            Get the link names that the user has provided for planning.
            Returns:
                list of link names of the user
        """
    def set_base_pose(self, pose: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> None: 
        """
            Set the base pose of the robot.
            Args:
                pose: base pose of the robot in [x, y, z, qw, qx, qy, qz] format
        """
    @typing.overload
    def set_move_group(self, end_effector: str) -> None: 
        """
            Set the move group, i.e. the chain ending in end effector for which to compute the forward kinematics for all subsequent queries.
            Args:
                chain: list of links extending to the end effector


            Set the move group but we have multiple end effectors in a chain. i.e. Base --> EE1 --> EE2 --> ... --> EEn
            Args:
                end_effectors: names of the end effector link
        """
    @typing.overload
    def set_move_group(self, end_effectors: list[str]) -> None: ...
    def set_qpos(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]], full: bool = False) -> None: 
        """
            Let the planner know the current joint positions.
            Args:
                qpos: current qpos of all active joints or just the move group joints
                full: whether to set the full qpos or just the move group qpos. if full false, we will pad the missing joints with current known qpos. the default is false
        """
    def update_SRDF(self, SRDF: str) -> None: 
        """
            Update the SRDF file to disable self-collisions.
            Args:
                srdf: path to SRDF file, can be relative to the current working directory
        """
    pass
