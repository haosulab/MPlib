import os
import unittest

import numpy as np
from transforms3d.quaternions import mat2quat, quat2mat

from mplib import Planner, Pose
from mplib.collision_detection import fcl
from mplib.kinematics.pinocchio import PinocchioModel

FILE_ABS_DIR = os.path.dirname(os.path.abspath(__file__))

PANDA_SPEC = {
    "urdf": f"{FILE_ABS_DIR}/../data/panda/panda.urdf",
    "srdf": f"{FILE_ABS_DIR}/../data/panda/panda.srdf",
    "move_group": "panda_hand",
}


class TestFunctionalValidation(unittest.TestCase):
    """
    Comprehensive functional verification suite under modern dependencies
    (NumPy >= 2.x, Pinocchio >= 3.x, modern Boost, urdfdom).
    """

    def setUp(self):
        self.planner = Planner(**PANDA_SPEC)
        self.model = self.planner.pinocchio_model
        self.ee_idx = self.planner.link_name_2_idx[self.planner.move_group]
        self.init_qpos = np.array([0, 0.2, 0, -2.6, 0, 3.0, 0.8, 0, 0], dtype=np.float64)
        self.joint_limits = np.concatenate(self.model.get_joint_limits())
        self.target_pose = Pose([0.4, 0.3, 0.12], [0, 1, 0, 0])

    def sample_valid_qpos(self):
        qpos = []
        for qmin, qmax in self.joint_limits:
            qpos.append(np.random.uniform(qmin, qmax))
        return np.array(qpos, dtype=np.float64)

    # =========================================================================
    # 1. Kinematics Validation
    # =========================================================================

    def test_forward_kinematics_poses_and_quaternions(self):
        """Verify FK link poses, root placement, base transforms, and unit quaternions."""
        self.model.compute_forward_kinematics(self.init_qpos)

        # Root link (panda_link0) should be at identity pose under default base pose
        root_idx = self.model.get_link_names().index("panda_link0")
        root_pose = self.model.get_link_pose(root_idx)
        np.testing.assert_allclose(root_pose.get_p(), [0, 0, 0], atol=1e-6)
        np.testing.assert_allclose(root_pose.get_q(), [1, 0, 0, 0], atol=1e-6)

        # All links must produce valid positions and unit quaternions
        for idx in range(len(self.model.get_link_names())):
            pose = self.model.get_link_pose(idx)
            p = pose.get_p()
            q = pose.get_q()
            self.assertTrue(np.all(np.isfinite(p)), f"Non-finite position in link {idx}")
            self.assertTrue(np.all(np.isfinite(q)), f"Non-finite quaternion in link {idx}")
            quat_norm = np.linalg.norm(q)
            self.assertAlmostEqual(
                quat_norm, 1.0, places=5,
                msg=f"Quaternion in link {idx} is not unit norm: {quat_norm}"
            )

        # Verify base pose setting propagates to robot base_pose and FCL collision objects
        base_shift = np.array([1.5, -2.0, 0.8], dtype=np.float64)
        base_pose = Pose(p=base_shift, q=[1, 0, 0, 0])
        self.planner.set_base_pose(base_pose)
        np.testing.assert_allclose(self.planner.robot.get_base_pose().get_p(), base_shift, atol=1e-5)

        self.planner.robot.set_qpos(self.init_qpos, full=True)
        fcl_objs = self.planner.robot.get_fcl_model().get_collision_objects()
        shifted_root_fcl = fcl_objs[0].shapes[0].get_pose()
        np.testing.assert_allclose(shifted_root_fcl.get_p(), base_shift, atol=1e-5)

        # Reset base pose
        self.planner.set_base_pose(Pose([0, 0, 0], [1, 0, 0, 0]))
        self.planner.robot.set_qpos(self.init_qpos, full=True)

    def test_inverse_kinematics_convergence_and_limits(self):
        """Verify IK converges within joint limits and matches target poses accurately."""
        # Generate reachable target poses from known valid joint configurations
        num_trials = 15
        success_count = 0
        for _ in range(num_trials):
            q_sample = self.sample_valid_qpos()
            self.model.compute_forward_kinematics(q_sample)
            target = self.model.get_link_pose(self.ee_idx)

            status, ik_results = self.planner.IK(
                target, self.init_qpos, n_init_qpos=20, threshold=1e-3
            )
            if status == "Success":
                success_count += 1
                for ik_q in ik_results:
                    # Verify each IK solution satisfies joint limits
                    for j, (val, (qmin, qmax)) in enumerate(zip(ik_q, self.joint_limits)):
                        self.assertTrue(
                            qmin - 1e-3 <= val <= qmax + 1e-3,
                            f"Joint {j} value {val} exceeds limits [{qmin}, {qmax}]"
                        )
                    # Verify FK of IK solution reproduces target pose
                    self.model.compute_forward_kinematics(ik_q)
                    solved_pose = self.model.get_link_pose(self.ee_idx)
                    dist = solved_pose.distance(target)
                    self.assertLess(
                        dist, 1e-3,
                        f"IK pose distance {dist} exceeds tolerance 1e-3"
                    )

        # Ensure high convergence rate on reachable poses
        self.assertGreaterEqual(
            success_count, 10,
            f"IK success count {success_count}/{num_trials} below expected minimum"
        )

    def test_inverse_kinematics_with_joint_mask(self):
        """Verify masked joints remain locked at their initial values during IK."""
        self.model.compute_forward_kinematics(self.init_qpos)
        target = self.model.get_link_pose(self.ee_idx)

        # Mask out joint 0 and joint 1
        mask = [True, True, False, False, False, False, False, False, False]
        status, ik_results = self.planner.IK(
            target, self.init_qpos, mask=mask, n_init_qpos=10
        )
        if status == "Success":
            for ik_q in ik_results:
                self.assertAlmostEqual(ik_q[0], self.init_qpos[0], places=4)
                self.assertAlmostEqual(ik_q[1], self.init_qpos[1], places=4)

    # =========================================================================
    # 2. Collision Validation (Self-Collision & Added Primitives)
    # =========================================================================

    def test_self_collision_detection(self):
        """Verify FCL detects self-collision accurately and respects SRDF allowed pairs."""
        # Nominal configuration must be collision-free
        nominal_q = np.array([0, 0.2, 0, -2.6, 0, 3.0, 0.8, 0, 0], dtype=np.float64)
        self.assertFalse(self.planner.check_for_self_collision(nominal_q))

        # Known folding self-collision configuration
        folded_q = np.array([0, 1.36, 0, -3.0, -3.0, 3.0, -1.0, 0, 0], dtype=np.float64)
        self_colls = self.planner.check_for_self_collision(folded_q)
        self.assertGreater(
            len(self_colls), 0,
            "Failed to detect expected self-collision in folded configuration"
        )

    def test_collision_with_added_box_primitive(self):
        """Verify collision detection against added FCL Box primitives."""
        self.planner.robot.set_qpos(self.init_qpos, full=True)
        ee_pose = self.model.get_link_pose(self.ee_idx)

        # 1. Place a box right at the end effector
        box = fcl.Box([0.2, 0.2, 0.2])
        box_obj = fcl.CollisionObject(box, Pose(p=ee_pose.get_p()))
        self.planner.planning_world.add_object("test_box_obstacle", box_obj)

        colls = self.planner.planning_world.check_robot_collision()
        self.assertGreater(
            len(colls), 0,
            "Expected collision with box placed at end effector, but none detected"
        )

        # 2. Move box far away: no collision
        self.planner.planning_world.remove_object("test_box_obstacle")
        box_obj_far = fcl.CollisionObject(box, Pose(p=[10.0, 10.0, 10.0]))
        self.planner.planning_world.add_object("test_box_obstacle", box_obj_far)
        colls_far = self.planner.planning_world.check_robot_collision()
        self.assertEqual(
            len(colls_far), 0,
            f"Unexpected collision with box placed far away: {colls_far}"
        )

        # 3. Clean up
        self.planner.planning_world.remove_object("test_box_obstacle")
        self.assertEqual(len(self.planner.planning_world.check_robot_collision()), 0)

    def test_collision_with_added_capsule_primitive(self):
        """Verify collision detection against added FCL Capsule primitives."""
        self.planner.robot.set_qpos(self.init_qpos, full=True)
        ee_pose = self.model.get_link_pose(self.ee_idx)

        # 1. Place a capsule right at the end effector
        capsule = fcl.Capsule(radius=0.1, lz=0.3)
        capsule_obj = fcl.CollisionObject(capsule, Pose(p=ee_pose.get_p()))
        self.planner.planning_world.add_object("test_capsule_obstacle", capsule_obj)

        colls = self.planner.planning_world.check_robot_collision()
        self.assertGreater(
            len(colls), 0,
            "Expected collision with capsule placed at end effector, but none detected"
        )

        # 2. Move capsule far away: no collision
        self.planner.planning_world.remove_object("test_capsule_obstacle")
        capsule_obj_far = fcl.CollisionObject(capsule, Pose(p=[10.0, 10.0, 10.0]))
        self.planner.planning_world.add_object("test_capsule_obstacle", capsule_obj_far)
        colls_far = self.planner.planning_world.check_robot_collision()
        self.assertEqual(
            len(colls_far), 0,
            f"Unexpected collision with capsule placed far away: {colls_far}"
        )

        # 3. Clean up
        self.planner.planning_world.remove_object("test_capsule_obstacle")
        self.assertEqual(len(self.planner.planning_world.check_robot_collision()), 0)

    # =========================================================================
    # 3. Motion Planning Trajectory Validation
    # =========================================================================

    def test_plan_qpos_trajectory_quality(self):
        """Verify plan_qpos() produces valid, continuous, and collision-free trajectories."""
        target_qpos = np.array([0.3, 0.1, -0.2, -2.0, 0.1, 2.5, 0.6, 0, 0], dtype=np.float64)

        result = self.planner.plan_qpos([target_qpos[:7]], self.init_qpos)
        self.assertEqual(result["status"], "Success")

        pos = result["position"]
        vel = result["velocity"]
        acc = result["acceleration"]
        times = result["time"]

        # No NaN or Inf
        self.assertFalse(np.isnan(pos).any(), "NaN found in planned positions")
        self.assertFalse(np.isinf(pos).any(), "Inf found in planned positions")
        self.assertFalse(np.isnan(vel).any(), "NaN found in planned velocities")
        self.assertFalse(np.isinf(vel).any(), "Inf found in planned velocities")
        self.assertFalse(np.isnan(acc).any(), "NaN found in planned accelerations")
        self.assertFalse(np.isinf(acc).any(), "Inf found in planned accelerations")

        # Start and Goal matching
        np.testing.assert_allclose(
            pos[0], self.init_qpos[:7], atol=1e-3,
            err_msg="Trajectory does not start at initial configuration"
        )
        np.testing.assert_allclose(
            pos[-1], target_qpos[:7], atol=1e-3,
            err_msg="Trajectory does not reach target configuration"
        )

        # Monotonic time
        self.assertTrue(np.all(np.diff(times) > 0), "Time steps are not strictly increasing")

        # Velocity limits
        vel_limits = self.planner.joint_vel_limits
        self.assertTrue(
            np.all(np.abs(vel) <= vel_limits + 1e-2),
            f"Planned velocities exceed limits: max={np.max(np.abs(vel), axis=0)}, limit={vel_limits}"
        )

        # Waypoint continuity (smoothness: no sudden teleports)
        step_diffs = np.abs(np.diff(pos, axis=0))
        max_step = np.max(step_diffs)
        self.assertLess(max_step, 0.5, f"Discontinuous jump detected in waypoints: max_step={max_step}")

        # Collision-free along entire trajectory
        for i, waypoint in enumerate(pos):
            self_colls = self.planner.check_for_self_collision(waypoint)
            self.assertEqual(
                len(self_colls), 0,
                f"Waypoint {i} in self-collision: {self_colls}"
            )
            env_colls = self.planner.check_for_env_collision(waypoint)
            self.assertEqual(
                len(env_colls), 0,
                f"Waypoint {i} in environment collision: {env_colls}"
            )

    def test_plan_pose_trajectory_quality(self):
        """Verify plan_pose() reaches goal pose smoothly and collision-free."""
        result = self.planner.plan_pose(self.target_pose, self.init_qpos)
        self.assertEqual(result["status"], "Success")

        pos = result["position"]
        times = result["time"]

        # No NaN/Inf and monotonic time
        self.assertFalse(np.isnan(pos).any(), "NaN found in position trajectory")
        self.assertFalse(np.isinf(pos).any(), "Inf found in position trajectory")
        self.assertTrue(np.all(np.diff(times) > 0), "Time is not strictly increasing")

        # Start match
        np.testing.assert_allclose(
            pos[0], self.init_qpos[:7], atol=1e-3,
            err_msg="Trajectory start does not match initial configuration"
        )

        # Goal pose reached
        self.planner.robot.set_qpos(pos[-1])
        actual_ee_pose = self.model.get_link_pose(self.ee_idx)
        pose_dist = actual_ee_pose.distance(self.target_pose)
        self.assertLess(
            pose_dist, 1e-3,
            f"End-effector final pose distance {pose_dist} exceeds tolerance 1e-3"
        )

        # Collision-free along waypoints
        for i, waypoint in enumerate(pos):
            self.assertEqual(
                len(self.planner.check_for_self_collision(waypoint)), 0,
                f"Pose trajectory waypoint {i} in self-collision"
            )

    # =========================================================================
    # 4. NumPy 2 ABI & C++/Python Boundary Validation
    # =========================================================================

    def test_numpy2_array_boundary_and_types(self):
        """Verify arrays crossing pybind11 boundary are contiguous float64 and support NumPy 2 ufuncs."""
        self.model.compute_forward_kinematics(self.init_qpos)

        # 1. Jacobian matrix
        jac = self.model.get_link_jacobian(self.ee_idx)
        self.assertIsInstance(jac, np.ndarray)
        self.assertEqual(jac.dtype, np.float64)
        self.assertEqual(jac.shape, (6, 9))

        # Advanced NumPy 2 operations on returned arrays
        gram_matrix = jac @ jac.T
        self.assertEqual(gram_matrix.shape, (6, 6))
        u, s, vh = np.linalg.svd(jac)
        self.assertTrue(np.all(np.isfinite(s)))

        # 2. Joint limits
        limits = self.model.get_joint_limits()
        self.assertIsInstance(limits, list)
        for lim in limits:
            self.assertIsInstance(lim, np.ndarray)
            self.assertEqual(lim.dtype, np.float64)
            self.assertEqual(lim.shape, (1, 2))

        # 3. Random configuration
        rand_q = self.model.get_random_configuration()
        self.assertIsInstance(rand_q, np.ndarray)
        self.assertEqual(rand_q.dtype, np.float64)
        self.assertEqual(rand_q.shape, (9,))

        # 4. Robot qpos
        qpos = self.planner.robot.get_qpos()
        self.assertIsInstance(qpos, np.ndarray)
        self.assertEqual(qpos.dtype, np.float64)
        self.assertTrue(qpos.flags.c_contiguous)

        # 5. Pose arrays
        pose = self.model.get_link_pose(self.ee_idx)
        p = pose.get_p()
        q = pose.get_q()
        mat4 = pose.to_transformation_matrix()
        self.assertEqual(p.dtype, np.float64)
        self.assertEqual(p.shape, (3,))
        self.assertTrue(p.flags.c_contiguous)
        self.assertEqual(q.dtype, np.float64)
        self.assertEqual(q.shape, (4,))
        self.assertTrue(q.flags.c_contiguous)
        self.assertEqual(mat4.dtype, np.float64)
        self.assertEqual(mat4.shape, (4, 4))

    def test_numpy2_non_contiguous_and_views_input(self):
        """Verify passing non-contiguous views and Fortran arrays from NumPy 2 into C++ functions."""
        # Create non-contiguous slice
        wide_array = np.zeros(18, dtype=np.float64)
        wide_array[::2] = self.init_qpos
        sliced_q = wide_array[::2]
        self.assertFalse(sliced_q.flags.c_contiguous)

        # Pass non-contiguous array into FK and Jacobian computation
        self.model.compute_forward_kinematics(sliced_q)
        self.model.compute_full_jacobian(sliced_q)
        jac1 = self.model.get_link_jacobian(self.ee_idx)
        jac2 = self.model.compute_single_link_jacobian(sliced_q, self.ee_idx)
        np.testing.assert_allclose(jac1, jac2, atol=1e-8)

        # Pass Fortran-contiguous array
        f_q = np.asfortranarray(self.init_qpos.copy())
        self.assertTrue(f_q.flags.f_contiguous)
        self.model.compute_forward_kinematics(f_q)
        pose_f = self.model.get_link_pose(self.ee_idx)
        self.model.compute_forward_kinematics(self.init_qpos)
        pose_c = self.model.get_link_pose(self.ee_idx)
        self.assertAlmostEqual(pose_f.distance(pose_c), 0, places=6)


if __name__ == "__main__":
    unittest.main()
