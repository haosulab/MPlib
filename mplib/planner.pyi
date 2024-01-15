from __future__ import annotations
from mplib.pymp import articulation
import mplib.pymp.articulation
from mplib.pymp import fcl
from mplib.pymp import kdl
from mplib.pymp import ompl
from mplib.pymp import pinocchio
from mplib.pymp import planning_world
import numpy
import numpy as np
import os as os
import toppra as ta
from toppra import algorithm as algo
from toppra import constraint
from transforms3d.quaternions import mat2quat
from transforms3d.quaternions import quat2mat
__all__ = ['Planner', 'algo', 'articulation', 'constraint', 'fcl', 'kdl', 'mat2quat', 'np', 'ompl', 'os', 'pinocchio', 'planning_world', 'quat2mat', 'ta']
class Planner:
    """
    Motion planner.
    """
    def IK(self, goal_pose, start_qpos, mask = list(), n_init_qpos = 20, threshold = 0.001):
        """
        
                Inverse kinematics
        
                Args:
                    goal_pose: [x,y,z,qw,qx,qy,qz] pose of the goal
                    start_qpos: initial configuration
                    mask: if the value at a given index is True, the joint is *not* used in the IK
                    n_init_qpos: number of random initial configurations
                    threshold: threshold for the distance between the goal pose and the result pose
                
        """
    def TOPP(self, path, step = 0.1, verbose = False):
        """
        
                Time-Optimal Path Parameterization
        
                Args:
                    path: numpy array of shape (n, dof)
                    step: step size for the discretization
                    verbose: if True, will print the log of TOPPRA
                
        """
    def __init__(self, urdf: str, move_group: str, srdf: str = '', package_keyword_replacement: str = '', user_link_names: typing.Sequence[str] = list(), user_joint_names: typing.Sequence[str] = list(), joint_vel_limits: typing.Union[typing.Sequence[float], numpy.ndarray] = list(), joint_acc_limits: typing.Union[typing.Sequence[float], numpy.ndarray] = list(), **kwargs):
        """
        Motion planner for robots.
        
                Args:
                    urdf: Unified Robot Description Format file.
                    user_link_names: names of links, the order. if empty, all links will be used.
                    user_joint_names: names of the joints to plan.  if empty, all active joints will be used.
                    move_group: target link to move, usually the end-effector.
                    joint_vel_limits: maximum joint velocities for time parameterization,
                        which should have the same length as
                    joint_acc_limits: maximum joint accelerations for time parameterization,
                        which should have the same length as
                    srdf: Semantic Robot Description Format file.
                References:
                    http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/urdf_srdf/urdf_srdf_tutorial.html
        
                
        """
    def check_for_collision(self, collision_function, articulation: mplib.pymp.articulation.ArticulatedModel = None, qpos: numpy.ndarray = None) -> list:
        """
        helper function to check for collision
        """
    def check_for_env_collision(self, articulation: mplib.pymp.articulation.ArticulatedModel = None, qpos: numpy.ndarray = None, with_point_cloud = False, use_attach = False) -> list:
        """
        Check if the robot is in collision with the environment
        
                Args:
                    articulation: robot model. if none will be self.robot
                    qpos: robot configuration. if none will be the current pose
                    with_point_cloud: whether to check collision against point cloud
                    use_attach: whether to include the object attached to the end effector in collision checking
                Returns:
                    A list of collisions.
                
        """
    def check_for_self_collision(self, articulation: mplib.pymp.articulation.ArticulatedModel = None, qpos: numpy.ndarray = None) -> list:
        """
        Check if the robot is in self-collision.
        
                Args:
                    articulation: robot model. if none will be self.robot
                    qpos: robot configuration. if none will be the current pose
        
                Returns:
                    A list of collisions.
                
        """
    def check_joint_limit(self, q):
        """
        
                check if the joint configuration is within the joint limits
        
                Args:
                    q: joint configuration
        
                Returns:
                    True if the joint configuration is within the joint limits
                
        """
    def distance_6D(self, p1, q1, p2, q2):
        """
        
                compute the distance between two poses
        
                Args:
                    p1: position of pose 1
                    q1: quaternion of pose 1
                    p2: position of pose 2
                    q2: quaternion of pose 2
                
        """
    def generate_collision_pair(self, sample_time = 1000000, echo_freq = 100000):
        """
        
                we read the srdf file to get the link pairs that should not collide.
                if not provided, we need to randomly sample configurations to find the link pairs that will always collide.
                
        """
    def pad_qpos(self, qpos, articulation = None):
        """
        
                if the user does not provide the full qpos but only the move_group joints,
                pad the qpos with the rest of the joints
                
        """
    def plan_qpos_to_pose(self, goal_pose, current_qpos, mask = list(), time_step = 0.1, rrt_range = 0.1, planning_time = 1, fix_joint_limits = True, use_point_cloud = False, use_attach = False, verbose = False, wrt_world = False, planner_name = 'RRTConnect', no_simplification = False, constraint_function = None, constraint_jacobian = None, constraint_tolerance = 0.001):
        """
        
                plan from a start configuration to a goal pose of the end-effector
        
                Args:
                    goal_pose: [x,y,z,qw,qx,qy,qz] pose of the goal
                    current_qpos: current joint configuration (either full or move_group joints)
                    mask: if the value at a given index is True, the joint is *not* used in the IK
                    time_step: time step for TOPPRA (time parameterization of path)
                    rrt_range: step size for RRT
                    planning_time: time limit for RRT
                    fix_joint_limits: if True, will clip the joint configuration to be within the joint limits
                    use_point_cloud: if True, will use the point cloud to avoid collision
                    use_attach: if True, will consider the attached tool collision when planning
                    verbose: if True, will print the log of OMPL and TOPPRA
                    wrt_world: if true, interpret the target pose with respect to the world frame instead of the base frame
                
        """
    def plan_qpos_to_qpos(self, goal_qposes: list, current_qpos, time_step = 0.1, rrt_range = 0.1, planning_time = 1, fix_joint_limits = True, use_point_cloud = False, use_attach = False, verbose = False, planner_name = 'RRTConnect', no_simplification = False, constraint_function = None, constraint_jacobian = None, constraint_tolerance = 0.001, fixed_joint_indices = list()):
        """
        
                plan a path from a specified joint position to a goal pose
        
                Args:
                    goal_pose: 7D pose of the end-effector [x,y,z,qw,qx,qy,qz]
                    current_qpos: current joint configuration (either full or move_group joints)
                    mask: mask for IK. When set, the IK will leave certain joints out of planning
                    time_step: time step for TOPP
                    rrt_range: step size for RRT
                    planning_time: time limit for RRT
                    fix_joint_limits: if True, will clip the joint configuration to be within the joint limits
                    use_point_cloud: if True, will use the point cloud to avoid collision
                    use_attach: if True, will consider the attached tool collision when planning
                    verbose: if True, will print the log of OMPL and TOPPRA
                    planner_name: planner name pick from {"RRTConnect", "RRT*"}
                    fixed_joint_indices: list of indices of joints that are fixed during planning
                    constraint_function: evals to 0 when constraint is satisfied
                    constraint_jacobian: jacobian of constraint_function
                    constraint_tolerance: tolerance for constraint_function
                    no_simplification: if true, will not simplify the path. constraint planning does not support simplification
                
        """
    def plan_screw(self, target_pose, qpos, qpos_step = 0.1, time_step = 0.1, use_point_cloud = False, use_attach = False, verbose = False):
        """
        
                plan from a start configuration to a goal pose of the end-effector using screw motion
        
                Args:
                    target_pose: [x,y,z,qw,qx,qy,qz] pose of the goal
                    qpos: current joint configuration (either full or move_group joints)
                    qpos_step: size of the random step for RRT
                    time_step: time step for the discretization
                    use_point_cloud: if True, will use the point cloud to avoid collision
                    use_attach: if True, will use the attached tool to avoid collision
                    verbose: if True, will print the log of TOPPRA
                
        """
    def remove_normal_object(self, name):
        """
        returns true if the object was removed, false if it was not found
        """
    def replace_package_keyword(self, package_keyword_replacement):
        """
        
                some ROS URDF files use package:// keyword to refer the package dir
                replace it with the given string (default is empty)
        
                Args:
                    package_keyword_replacement: the string to replace 'package://' keyword
                
        """
    def set_base_pose(self, pose):
        """
        
                tell the planner where the base of the robot is w.r.t the world frame
        
                Args:
                    pose: [x,y,z,qw,qx,qy,qz] pose of the base
                
        """
    def set_normal_object(self, name, collision_object):
        """
        adds or updates a non-articulated collision object in the scene
        """
    def update_attached_box(self, size, pose, link_id = -1):
        """
        
                attach a box to some link
        
                Args:
                    size: [x,y,z] size of the box
                    pose: [x,y,z,qw,qx,qy,qz] pose of the box
                    link_id: if not provided, the end effector will be the target.
                
        """
    def update_attached_mesh(self, mesh_path, pose, link_id = -1):
        """
        
                attach a mesh to some link
        
                Args:
                    mesh_path: path to the mesh
                    pose: [x,y,z,qw,qx,qy,qz] pose of the mesh
                    link_id: if not provided, the end effector will be the target.
                
        """
    def update_attached_sphere(self, radius, pose, link_id = -1):
        """
        
                attach a sphere to some link
        
                Args:
                    radius: radius of the sphere
                    pose: [x,y,z,qw,qx,qy,qz] pose of the sphere
                    link_id: if not provided, the end effector will be the target.
                
        """
    def update_attached_tool(self, fcl_collision_geometry, pose, link_id = -1):
        """
        helper function to update the attached tool
        """
    def update_point_cloud(self, pc, radius = 0.001):
        """
        
                Args:
                    pc: numpy array of shape (n, 3)
                    radius: radius of each point. This gives a buffer around each point that planner will avoid
                
        """
