import mplib
import numpy as np


def test():
    planner = mplib.Planner(
        urdf="data/panda/panda.urdf",
        srdf="data/panda/panda.srdf",
        move_group="panda_hand",
    )
    pose = [0.4, 0.3, 0.12, 0, 1, 0, 0]
    qpos = np.array([0, 0.2, 0, -2.6, 0, 3.0, 0.8, 0, 0])
    result = planner.plan(pose, qpos, time_step=1 / 250)


if __name__ == "__main__":
    test()
