import xml.etree.ElementTree as ET
from pathlib import Path
from xml.dom import minidom

import numpy as np

from .collision_detection import AllowedCollisionMatrix
from .pymp import ArticulatedModel


def compute_default_collisions(
    robot: ArticulatedModel, *, num_samples=100000, verbose=False
) -> str:
    """
    Compute default collision pairs for generating SRDF.

    This function mimics MoveIt2's
    ``moveit_setup::srdf_setup::computeDefaultCollisions()``

    Reference:
    https://moveit.picknik.ai/main/api/html/namespacemoveit__setup_1_1srdf__setup.html#a2812f73b447d838cd7dba1b0ee1a0c95

    :param robot: an ``ArticulatedModel``
    :param num_samples: number of samples to find the link that will always collide
    :param verbose: print debug info
    :return: SRDF content as an XML string
    """
    if verbose:
        print(
            "Generating SRDF with default collision pairs. "
            "This may take several minutes."
        )

    root = ET.Element("robot", {"name": robot.name})

    pinocchio_model = robot.get_pinocchio_model()
    user_link_names = pinocchio_model.get_link_names()
    user_joint_names = pinocchio_model.get_joint_names()
    link_name_2_idx = {link: i for i, link in enumerate(user_link_names)}
    fcl_model = robot.get_fcl_model()
    acm = AllowedCollisionMatrix()

    # 1. disable adjacent link pairs
    for link1, link2 in pinocchio_model.get_adjacent_links():
        if verbose:
            print(f"Ignore collision pair: ({link1}, {link2}), reason: Adjacent")
        acm.set_entry(link1, link2, True)
        _ = ET.SubElement(
            root,
            "disable_collisions",
            attrib={"link1": link1, "link2": link2, "reason": "Adjacent"},
        )

    # 2. disable all-zeros qpos (default) collision
    robot.set_qpos(np.zeros(len(user_joint_names)), True)
    for collision in fcl_model.check_self_collision():
        link1, link2 = collision.link_name1, collision.link_name2
        if acm.get_entry(link1, link2) is not None:  # already ignored
            continue
        if verbose:
            print(
                f"Ignore collision pair: ({link1}, {link2}), "
                "reason: Default (collides at all-zeros qpos)"
            )
        acm.set_entry(link1, link2, True)
        _ = ET.SubElement(
            root,
            "disable_collisions",
            attrib={"link1": link1, "link2": link2, "reason": "Default"},
        )

    # 3. disable collision pairs that always collide and never collide via sampling
    n_links = len(user_link_names)
    collision_cnt = np.zeros((n_links, n_links), dtype=int)
    for _ in range(num_samples):
        robot.set_qpos(pinocchio_model.get_random_configuration(), True)
        for collision in fcl_model.check_self_collision():
            u = link_name_2_idx[collision.link_name1]
            v = link_name_2_idx[collision.link_name2]
            collision_cnt[u][v] += 1

    for i, link1 in enumerate(user_link_names):
        for j in range(i + 1, n_links):
            link2 = user_link_names[j]
            if acm.get_entry(link1, link2) is not None:  # already ignored
                continue
            if (cnt := (collision_cnt[i][j] + collision_cnt[j][i])) == num_samples:
                if verbose:
                    print(
                        f"Ignore collision pair: ({link1}, {link2}), "
                        "reason: Always collide"
                    )
                _ = ET.SubElement(
                    root,
                    "disable_collisions",
                    attrib={"link1": link1, "link2": link2, "reason": "Always"},
                )
            elif cnt == 0:
                if verbose:
                    print(
                        f"Ignore collision pair: ({link1}, {link2}), "
                        "reason: Never collide"
                    )
                _ = ET.SubElement(
                    root,
                    "disable_collisions",
                    attrib={"link1": link1, "link2": link2, "reason": "Never"},
                )

    return minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")


def replace_urdf_package_keyword(
    urdf_path: str | Path,
    new_package_keyword: str = "",
) -> Path:
    """
    Some ROS URDF files use package:// keyword to refer the package dir.
    Replace it with the given string (default is empty)

    :param urdf_path: Path to a Unified Robot Description Format file.
    :param new_package_keyword: the string to replace ``package://`` keyword
    :return: Path to the modified URDF file
    """
    urdf_path = Path(urdf_path)
    with urdf_path.open("r") as in_f:
        if "package://" in (content := in_f.read()):
            # Create a new URDF file
            urdf_path = urdf_path.with_name(urdf_path.stem + "_mplib.urdf")
            with urdf_path.open("w") as out_f:
                out_f.write(content.replace("package://", new_package_keyword))
    return urdf_path


def generate_srdf(
    urdf_path: str | Path,
    new_package_keyword: str = "",
    *,
    num_samples=100000,
    verbose=False,
) -> Path:
    """
    Generate SRDF from URDF similar to MoveIt2's setup assistant.

    :param urdf_path: Path to a Unified Robot Description Format file.
    :param new_package_keyword: the string to replace ``package://`` keyword
    :param num_samples: number of samples to find the link that will always collide
    :param verbose: print debug info
    :return: Path to the generated SRDF file
    """
    assert Path(urdf_path).is_file(), f"URDF file {urdf_path} does not exist"

    # Replace 'package://' keyword
    urdf_path = replace_urdf_package_keyword(urdf_path, new_package_keyword)

    robot = ArticulatedModel(str(urdf_path), "")
    srdf_str = compute_default_collisions(
        robot, num_samples=num_samples, verbose=verbose
    )

    # Create a new SRDF file and save
    save_path = urdf_path.with_name(urdf_path.stem + "_mplib.srdf")
    with save_path.open("w") as f:
        f.write(srdf_str)

    if verbose:
        print(f"Saved the SRDF file to {save_path}")

    return save_path
