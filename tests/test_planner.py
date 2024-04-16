import os
import unittest
from copy import deepcopy

import numpy as np
import trimesh
from transforms3d.quaternions import mat2quat, quat2mat

from mplib import Planner, Pose
from mplib.collision_detection import fcl

FILE_ABS_DIR = os.path.dirname(os.path.abspath(__file__))

PANDA_SPEC = {
    "urdf": f"{FILE_ABS_DIR}/../data/panda/panda.urdf",
    "srdf": f"{FILE_ABS_DIR}/../data/panda/panda.srdf",
    "move_group": "panda_hand",
}


class TestPlanner(unittest.TestCase):
    def setUp(self):
        self.planner = Planner(**PANDA_SPEC)
        self.target_pose = Pose([0.4, 0.3, 0.12], [0, 1, 0, 0])
        self.init_qpos = np.array([0, 0.2, 0, -2.6, 0, 3.0, 0.8, 0, 0])
        self.joint_limits = [
            [-2.8973, 2.8973],
            [-1.7628, 1.7628],
            [-2.8973, 2.8973],
            [-3.0718, -0.0698],
            [-2.8973, 2.8973],
            [-0.0175, 3.7525],
            [-2.8973, 2.8973],
            [0, 0.04],
            [0, 0.04],
        ]
        self.joint_types = self.planner.joint_types

    def get_end_effector_pose(self):
        ee_idx = self.planner.link_name_2_idx[self.planner.move_group]
        return self.planner.robot.get_pinocchio_model().get_link_pose(ee_idx)

    def sample_qpos(self):
        qpos = []
        for limit in self.joint_limits:
            qpos.append(np.random.uniform(limit[0], limit[1]))
        return np.array(qpos)

    def sample_pose(self):
        pos = np.random.uniform(-1, 1, size=3)
        quat = np.random.uniform(0, 1, size=4)
        quat /= np.linalg.norm(quat)
        pose = Pose(pos, quat)
        return pose

    def test_planning_to_pose(self):
        result_sampling = self.planner.plan_pose(self.target_pose, self.init_qpos)
        self.assertEqual(result_sampling["status"], "Success")
        last_qpos = result_sampling["position"][-1]
        self.planner.robot.set_qpos(last_qpos)
        self.assertAlmostEqual(
            self.get_end_effector_pose().distance(self.target_pose), 0, places=3
        )

    def test_planning_to_qpos(self):
        num_success = 0
        expected_least_num_success = 5
        for _ in range(10):
            target_qpos = self.sample_qpos()[:7]
            result_sampling = self.planner.plan_qpos([target_qpos], self.init_qpos)
            # might not always be a valid pose, so check if the planner succeeded
            if result_sampling["status"] == "Success":
                num_success += 1
                last_qpos = result_sampling["position"][-1]
                self.planner.robot.set_qpos(last_qpos)
                self.assertTrue(np.allclose(last_qpos, target_qpos, atol=1e-3))
        self.assertGreaterEqual(num_success, expected_least_num_success)

    def test_planning_screw(self):
        result_sampling = self.planner.plan_screw(self.target_pose, self.init_qpos)
        self.assertEqual(result_sampling["status"], "Success")
        last_qpos = result_sampling["position"][-1]
        self.planner.robot.set_qpos(last_qpos)
        self.assertAlmostEqual(
            self.get_end_effector_pose().distance(self.target_pose), 0, places=1
        )

    def test_wrap_joint_limit(self, tolerance=1e-3):
        # Test using np.ceil
        for _ in range(100):
            qpos = np.random.uniform(-100, 100, size=7)

            in_limit = True
            for i, (q, (q_min, q_max)) in enumerate(zip(qpos, self.joint_limits)):
                if self.joint_types[i].startswith("JointModelR"):
                    if -1e-3 <= q - q_min < 0:
                        continue
                    q += 2 * np.pi * np.ceil((q_min - q) / (2 * np.pi))
                if not (q_min - tolerance <= q <= q_max + tolerance):
                    in_limit = False
                    break

            self.assertEqual(
                self.planner.wrap_joint_limit(qpos.copy()),
                in_limit,
                f"Joint limit check failed for qpos: {qpos} which should be "
                f"{'in' if in_limit else 'out of'} limit",
            )

    def test_pad_move_group_qpos(self):
        for _ in range(100):
            full_qpos = self.sample_qpos()
            non_full_qpos = self.sample_qpos()[:7]
            self.planner.robot.set_qpos(full_qpos, full=True)
            padded_qpos = self.planner.pad_move_group_qpos(non_full_qpos)
            self.planner.robot.set_qpos(non_full_qpos, full=False)
            self.assertTrue(
                np.allclose(padded_qpos, self.planner.robot.get_qpos(), atol=1e-3)
            )

    def test_self_collision(self):
        self_collision_qpos = [0, 1.36, 0, -3, -3, 3, -1]
        self.assertTrue(self.planner.check_for_self_collision(self_collision_qpos))
        self_collision_qpos[0] += (
            1  # rotating the robot around the base should not cause self-collision to disappear
        )
        self.assertTrue(self.planner.check_for_self_collision(self_collision_qpos))
        collision_free_qpos = [0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78]
        self.assertFalse(self.planner.check_for_self_collision(collision_free_qpos))

    def test_env_collision(self):
        floor = fcl.Box([2, 2, 0.1])  # create a 2 x 2 x 0.1m box
        # create a collision object for the floor, with a 10cm offset in the z direction
        floor_fcl_collision_object = fcl.CollisionObject(floor, Pose(p=[0, 0, -0.1]))
        # update the planning world with the floor collision object
        self.planner.planning_world.add_object("floor", floor_fcl_collision_object)

        env_collision_qpos = [
            0,
            1.5,
            0,
            -1.5,
            0,
            0,
            0,
        ]  # this qpos causes several joints to dip below the floor
        self.assertTrue(self.planner.check_for_env_collision(env_collision_qpos))

        # remove the floor and check for env-collision returns no collision
        self.planner.remove_object("floor")
        self.assertFalse(self.planner.check_for_env_collision(env_collision_qpos))

    def test_IK(self):
        num_success = 0
        expected_least_num_success = 10
        for _ in range(100):
            # generate some random poses that should be within the robot's workspace
            pose = self.sample_pose()
            status, results = self.planner.IK(pose, self.init_qpos)
            num_success += status == "Success"
            if status == "Success":
                for result_qpos in results:
                    self.planner.robot.set_qpos(result_qpos, full=True)
                    self.assertAlmostEqual(
                        self.get_end_effector_pose().distance(pose), 0, places=3
                    )
        self.assertGreaterEqual(num_success, expected_least_num_success)

        # now put down a floor and check that the robot can't reach the pose
        floor = fcl.Box([2, 2, 0.1])  # create a 2 x 2 x 0.1m box
        # create a collision object for the floor, with a 10cm offset in the z direction
        floor_fcl_collision_object = fcl.CollisionObject(floor, Pose(p=[0, 0, -0.1]))

        under_floor_target_pose = deepcopy(self.target_pose)
        under_floor_target_pose.set_p([0.4, 0.3, -0.1])
        status, _ = self.planner.IK(under_floor_target_pose, self.init_qpos)
        self.assertEqual(status, "Success")
        self.planner.planning_world.add_object("floor", floor_fcl_collision_object)
        status, _ = self.planner.IK(under_floor_target_pose, self.init_qpos)
        self.assertNotEqual(status, "Success")

    def test_set_base_pose(self):
        for _ in range(10):
            base_pose = self.sample_pose()
            self.planner.set_base_pose(base_pose)

            result_sampling = self.planner.plan_pose(
                base_pose * self.target_pose, self.init_qpos
            )
            self.assertEqual(result_sampling["status"], "Success")
            last_qpos_sampling = result_sampling["position"][-1]
            self.planner.robot.set_qpos(last_qpos_sampling)
            self.assertAlmostEqual(
                self.get_end_effector_pose().distance(self.target_pose), 0, places=2
            )

            result_screw = self.planner.plan_screw(
                base_pose * self.target_pose, self.init_qpos
            )
            self.assertEqual(result_screw["status"], "Success")
            last_qpos_screw = result_screw["position"][-1]
            self.planner.robot.set_qpos(last_qpos_screw)
            self.assertAlmostEqual(
                self.get_end_effector_pose().distance(self.target_pose), 0, places=1
            )

            result_sampling = self.planner.plan_pose(
                self.target_pose, self.init_qpos, wrt_world=False
            )
            self.assertEqual(result_sampling["status"], "Success")
            last_qpos_sampling = result_sampling["position"][-1]
            self.planner.robot.set_qpos(last_qpos_sampling)

    def add_point_cloud(self):
        box = trimesh.creation.box([0.1, 0.4, 0.2])
        points, _ = trimesh.sample.sample_surface(box, 1000)
        points += [0.55, 0, 0.1]
        self.planner.update_point_cloud(points, resolution=0.02)

    def test_update_point_cloud(self):
        # use screw based planning. first succeeds but after point cloud obstacle fails
        pose = [0.7, 0, 0.12, 0, 1, 0, 0]
        pose = Pose([0.7, 0, 0.12], [0, 1, 0, 0])
        result_screw = self.planner.plan_screw(pose, self.init_qpos)
        self.assertEqual(result_screw["status"], "Success")

        # now add a point cloud and we should fail
        self.add_point_cloud()

        result_screw = self.planner.plan_screw(pose, self.init_qpos)
        self.assertNotEqual(result_screw["status"], "Success")

    def test_update_attach(self):
        starting_qpos = [0, 0.48, 0, -1.48, 0, 1.96, 0.78]
        target_pose = Pose([0.4, 0.3, 0.33], [0, 1, 0, 0])
        self.add_point_cloud()

        result_screw = self.planner.plan_screw(target_pose, starting_qpos)
        self.assertEqual(result_screw["status"], "Success")

        # now attach a box to the end effector and we should fail
        self.planner.update_attached_box(
            [0.04, 0.04, 0.12], Pose([0, 0, 0.14], [1, 0, 0, 0])
        )

        result_screw = self.planner.plan_screw(target_pose, starting_qpos)
        self.assertNotEqual(result_screw["status"], "Success")

    def test_fixed_joint(self):
        # randomly sample qpos and fix 2 random joints
        for _ in range(10):
            qpos = self.sample_qpos()
            fixed_joints = np.random.choice(range(7), 2, replace=False)
            result = self.planner.plan_qpos(
                [qpos], self.init_qpos, fixed_joint_indices=fixed_joints
            )
            if result["status"] == "Success":
                for joint_idx in range(7):
                    if joint_idx in fixed_joints:
                        self.assertEqual(
                            result["position"][-1][joint_idx], self.init_qpos[joint_idx]
                        )
                    else:
                        self.assertAlmostEqual(
                            result["position"][-1][joint_idx], qpos[joint_idx], places=3
                        )

    def make_f(self):
        def f(x, out):
            self.planner.robot.set_qpos(x)
            eef_pose = self.get_end_effector_pose()
            eef_z_axis = quat2mat(eef_pose.get_q())[:, 2]
            out[0] = -eef_z_axis[2] - 0.883  # maintain 28 degrees w.r.t. -z axis

        return f

    def make_j(self):
        def j(x, out):
            full_qpos = self.planner.pad_move_group_qpos(x)
            jac = self.planner.robot.get_pinocchio_model().compute_single_link_jacobian(
                full_qpos, len(self.planner.move_group_joint_indices) - 1
            )
            rot_jac = jac[3:, self.planner.move_group_joint_indices]
            eef_pose = self.get_end_effector_pose()
            eef_z_axis = quat2mat(eef_pose.get_q())[:, 2]
            for i in range(len(self.planner.move_group_joint_indices)):
                out[i] = np.cross(rot_jac[:, i], eef_z_axis).dot(np.array([0, 0, -1]))

        return j

    def test_constrained_planning(self, success_percentage=0.3):
        constrained_init_pose = Pose(
            [0.4, 0.3, 0.12], [0.1710101, -0.9698463, 0.0301537, -0.1710101]
        )
        constrained_target_pose = Pose(
            [0.6, 0.1, 0.44], [0.1710101, -0.9698463, 0.0301537, -0.1710101]
        )
        # first do an ik to find the init_qpos
        status, results = self.planner.IK(constrained_init_pose, self.init_qpos)
        self.assertEqual(status, "Success")
        init_qpos = results[0]
        f = self.make_f()
        out = np.zeros(1)
        f(init_qpos[:7], out)
        self.assertAlmostEqual(
            out[0], 0, places=3
        )  # check if the initial qpos satisfies the constraint

        valid_count = 0
        total_count = 0
        for _ in range(20):
            self.planner.robot.set_qpos(init_qpos, full=True)
            result = self.planner.plan_pose(
                constrained_target_pose,
                init_qpos,
                constraint_function=self.make_f(),
                constraint_jacobian=self.make_j(),
                constraint_tolerance=0.001,
                rrt_range=0.01,
                time_step=1 / 250,
                simplify=False,
            )
            if result["status"] != "Success":
                continue
            for qpos in result["position"]:
                self.planner.robot.set_qpos(qpos)
                f(qpos[:7], out)
                valid_count += abs(out[0]) < 0.01
            total_count += len(result["position"])
        self.assertGreater(valid_count / total_count, success_percentage)


if __name__ == "__main__":
    unittest.main()
