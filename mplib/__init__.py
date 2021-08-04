import toppra
import numpy as np
from _mplib import *
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
from transforms3d.quaternions import quat2mat
from typing import Tuple


class Planner(object):
	def __init__(
		self,
		urdf="./panda/panda.urdf",
		srdf="./panda/panda.srdf",
		# align the order of user links
		user_link_names=['panda_link0', 'panda_link1', 'panda_link2', 'panda_link3',
		                'panda_link4', 'panda_link5', 'panda_link6', 'panda_link7',
						'panda_link8', 'panda_hand', 'panda_leftfinger', 'panda_rightfinger'],
		# align the order of user joints
		user_joint_names=['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
		                'panda_joint5', 'panda_joint6', 'panda_joint7',
						'panda_finger_joint1', 'panda_finger_joint2'],
		move_group="panda_hand",
		joint_vel_limits=np.ones(7),
		joint_acc_limits=np.ones(7)
	):
		self.urdf = urdf
		self.srdf = srdf
		self.user_link_names = user_link_names
		self.user_joint_names = user_joint_names
		self.joint_name_2_idx = {}
		for i, joint in enumerate(self.user_joint_names):
			self.joint_name_2_idx[joint] = i
		self.link_name_2_idx = {}
		for i, link in enumerate(self.user_link_names):
			self.link_name_2_idx[link] = i
		self.robot = articulation.ArticulatedModel(urdf, srdf, [0, 0, -9.81], self.user_joint_names, 
													self.user_link_names, verbose=False, convex=True)
		self.planning_world = planning_world.PlanningWorld([self.robot], ["robot"], [], [])
		self.move_group = move_group
		self.robot.set_move_group(self.move_group)
		self.move_group_joint_indices = self.robot.get_move_group_joint_indices()
		self.pinocchio_model = self.robot.get_pinocchio_model()
		self.joint_types = self.pinocchio_model.get_joint_types()
		self.joint_limits = np.concatenate(self.pinocchio_model.get_joint_limits())
		self.planner = ompl.OMPLPlanner(world=self.planning_world)
		self.joint_vel_limits = joint_vel_limits
		self.joint_acc_limits = joint_acc_limits
		self.move_group_link_id = self.link_name_2_idx[self.move_group]
		assert(len(self.joint_vel_limits) == len(self.move_group_joint_indices))
		assert(len(self.joint_acc_limits) == len(self.move_group_joint_indices))
		
	def distance_6D(self, p1, q1, p2, q2):
		return np.linalg.norm(p1 - p2) + min(np.linalg.norm(q1 - q2), np.linalg.norm(q1 + q2))

	def check_joint_limit(self, q):
		n = len(q)
		flag = True
		for i in range(n):
			if self.joint_types[i].startswith("JointModelR"):
				if (np.abs(q[i] - self.joint_limits[i][0]) < 1e-3):
					continue
				q[i] -= 2 * np.pi * np.floor((q[i] - self.joint_limits[i][0]) / (2 * np.pi))
				if q[i] > self.joint_limits[i][1] + 1e-3:
					flag = False
			else:
				if q[i] < self.joint_limits[i][0] - 1e-3 or q[i] > self.joint_limits[i][1] + 1e-3:
					flag = False
		return flag

	def IK(self, goal_pose, start_qpos, n_init_qpos = 20, threshold = 1e-3):
		index = self.link_name_2_idx[self.move_group]
		min_dis = 1e9
		result = np.zeros(len(self.user_joint_names))
		for i in range(n_init_qpos):
			ik_results = self.pinocchio_model.compute_IK_CLIK(index, goal_pose, start_qpos) 
			flag = self.check_joint_limit(np.copy(ik_results[0]))
			if flag:
				self.pinocchio_model.compute_forward_kinematics(ik_results[0])
				new_pose = self.pinocchio_model.get_link_pose(index)
				tmp_dis = self.distance_6D(goal_pose[:3], goal_pose[3:], new_pose[:3], new_pose[3:])
				if tmp_dis < min_dis:
					min_dis = tmp_dis
					result = ik_results[0] 
				if min_dis < threshold:
					return "Success", result
			start_qpos = self.pinocchio_model.get_random_configuration()
		if min_dis != 1e9:
			status = "IK Failed! Distance %lf is greater than threshold %lf." % (min_dis, threshold)
		else:
			status = "IK Failed! Cannot find valid solution."
		return status, result

	def TOPP(self, path, step = 0.1, verbose = False):
		N_samples = path.shape[0]
		dof = path.shape[1]
		assert(dof == len(self.joint_vel_limits))
		assert(dof == len(self.joint_acc_limits))
		ss = np.linspace(0, 1, N_samples)
		path = ta.SplineInterpolator(ss, path)
		pc_vel = constraint.JointVelocityConstraint(self.joint_vel_limits)
		pc_acc = constraint.JointAccelerationConstraint(self.joint_acc_limits)
		instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
		jnt_traj = instance.compute_trajectory()
		ts_sample = np.linspace(0, jnt_traj.duration, int(jnt_traj.duration / step))
		qs_sample = jnt_traj(ts_sample)
		qds_sample = jnt_traj(ts_sample, 1)
		qdds_sample = jnt_traj(ts_sample, 2)
		return ts_sample, qs_sample, qds_sample, qdds_sample, jnt_traj.duration

	def update_point_cloud(self, pc, resolution = 1e-3):
		self.planning_world.update_point_cloud(pc, resolution)

	def update_attached_box(self, size, pose, link_id = -1):
		if link_id == -1:
			link_id = self.move_group_link_id
		self.planning_world.update_attached_box(size, link_id, pose)

	def plan(self, goal_pose, current_qpos, time_step = 0.1, rrt_range = 0.1, planning_time = 1, fix_joint_limits = True, use_point_cloud = False, use_attach = False, verbose = False):
		self.planning_world.set_use_point_cloud(use_point_cloud)
		self.planning_world.set_use_attach(use_attach)
		n = current_qpos.shape[0]
		if fix_joint_limits:
			for i in range(n):
				if current_qpos[i] < self.joint_limits[i][0]:
					current_qpos[i] = self.joint_limits[i][0] + 1e-3
				if current_qpos[i] > self.joint_limits[i][1]:
					current_qpos[i] = self.joint_limits[i][1] - 1e-3
		
		idx = self.move_group_joint_indices  
		ik_status, goal_qpos = self.IK(goal_pose, current_qpos)

		if ik_status != "Success":
			return {"status": ik_status}

		self.robot.set_qpos(current_qpos, True)
		status, path = self.planner.plan(current_qpos[idx], goal_qpos[idx], range = rrt_range, verbose = verbose, time = planning_time)
		if status == "Exact solution":
			if verbose:
				ta.setup_logging("INFO")
			else:
				ta.setup_logging("WARNING")
			times, pos, vel, acc, duration = self.TOPP(path, time_step)
			return {"status": "Success",
					"time": times,
					"position": pos,
					"velocity": vel,
					"acceleration": acc,
					"duration": duration}
		else:
			return {"status": "RRT Failed. %s" % status}

	def plan_screw(self, target_pose, qpos, qpos_step = 0.1, time_step = 0.1, use_point_cloud = False, use_attach = False, verbose = False):
		self.planning_world.set_use_point_cloud(use_point_cloud)
		self.planning_world.set_use_attach(use_attach)
		qpos = np.copy(qpos)
		self.robot.set_qpos(qpos, True)
		def pose7D2mat(pose):
			mat = np.eye(4)
			mat[0:3, 3] = pose[:3]
			mat[0:3, 0:3] = quat2mat(pose[3:])
			return mat

		def skew(vec):
			return np.array([[0, -vec[2], vec[1]],
							[vec[2], 0, -vec[0]],
							[-vec[1], vec[0], 0]])

		def pose2exp_coordinate(pose: np.ndarray) -> Tuple[np.ndarray, float]:
			def rot2so3(rotation: np.ndarray):
				assert rotation.shape == (3, 3)
				if np.isclose(rotation.trace(), 3):
					return np.zeros(3), 1
				if np.isclose(rotation.trace(), -1):
					return np.zeros(3), -1e6
				theta = np.arccos((rotation.trace() - 1) / 2)
				omega = 1 / 2 / np.sin(theta) * np.array(
					[rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]]).T
				return omega, theta

			omega, theta = rot2so3(pose[:3, :3])
			if theta < -1e5:
				return omega, theta
			ss = skew(omega)
			inv_left_jacobian = np.eye(3) / theta - 0.5 * ss + (
					1.0 / theta - 0.5 / np.tan(theta / 2)) * ss @ ss
			v = inv_left_jacobian @ pose[:3, 3]
			return np.concatenate([v, omega]), theta

		self.pinocchio_model.compute_forward_kinematics(qpos)
		ee_index = self.link_name_2_idx[self.move_group]
		current_p = pose7D2mat(self.pinocchio_model.get_link_pose(ee_index))
		target_p = pose7D2mat(target_pose) 
		relative_transform = target_p @ np.linalg.inv(current_p)

		omega, theta = pose2exp_coordinate(relative_transform)

		if theta < -1e4:
			return {"status": "screw plan failed."}
		omega = omega.reshape((-1, 1)) * theta

		index = self.move_group_joint_indices
		path = [np.copy(qpos[index])]

		while True:
			self.pinocchio_model.compute_full_jacobian(qpos)
			J = self.pinocchio_model.get_link_jacobian(ee_index, local = False)
			delta_q = np.linalg.pinv(J) @ omega
			delta_q *= qpos_step / (np.linalg.norm(delta_q))
			delta_twist = J @ delta_q

			flag = False
			if np.linalg.norm(delta_twist) > np.linalg.norm(omega):
				ratio = np.linalg.norm(omega) / np.linalg.norm(delta_twist)
				delta_q = delta_q * ratio
				delta_twist = delta_twist * ratio
				flag = True

			qpos += delta_q.reshape(-1)
			omega -= delta_twist

			def check_joint_limit(q):
				n = len(q)
				for i in range(n):
					if q[i] < self.joint_limits[i][0] - 1e-3 or q[i] > self.joint_limits[i][1] + 1e-3:
						return False
				return True

			within_joint_limit = check_joint_limit(qpos)
			self.planning_world.set_qpos_all(qpos[index])
			collide = self.planning_world.collide()

			if np.linalg.norm(delta_twist) < 1e-4 or collide or within_joint_limit == False:
				return {"status": "screw plan failed"}
			
			path.append(np.copy(qpos[index]))

			if flag:
				if verbose:
					ta.setup_logging("INFO")
				else:
					ta.setup_logging("WARNING")
				times, pos, vel, acc, duration = self.TOPP(np.vstack(path), time_step)
				return {"status": "Success",
						"time": times,
						"position": pos,
						"velocity": vel,
						"acceleration": acc,
						"duration": duration}
