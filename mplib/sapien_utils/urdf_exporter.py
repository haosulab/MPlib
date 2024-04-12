from __future__ import annotations

from xml.etree import ElementTree as ET

from sapien.physx import (
    PhysxArticulation,
    PhysxArticulationJoint,
    PhysxArticulationLinkComponent,
)


def export_link(link: PhysxArticulationLinkComponent) -> ET.Element:
    """
    Export an articulation link. Generates
    <link name="world">
        <inertial>
        <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.0"></origin>
        <mass value="9.999999974752427e-07"></mass>
        <inertia ixx="1e-06" ixy="0" ixz="0" iyy="1e-06" iyz="0" izz="1e-06"></inertia>
        </inertial>
    </link>
    """
    cmass_pose = link.cmass_local_pose
    angles = cmass_pose.rpy

    elem_link = ET.Element("link", {"name": link.name})
    elem_inertial = ET.SubElement(elem_link, "inertial")
    ET.SubElement(
        elem_inertial,
        "origin",
        {
            "xyz": f"{cmass_pose.p[0]} {cmass_pose.p[1]} {cmass_pose.p[2]}",
            "rpy": f"{angles[0]} {angles[1]} {angles[2]}",
        },
    )
    ET.SubElement(elem_inertial, "mass", {"value": str(link.mass)})
    ET.SubElement(
        elem_inertial,
        "inertia",
        {
            "ixx": str(link.inertia[0]),
            "iyy": str(link.inertia[1]),
            "izz": str(link.inertia[2]),
            "ixy": "0",
            "ixz": "0",
            "iyz": "0",
        },
    )

    return elem_link


def export_joint(joint: PhysxArticulationJoint) -> list[ET.Element]:
    """
    Export an articulation joint. No joint mimic support yet.
    If the joint has no parent_link (joint from root link to 1st robot link), generates
    <joint name="__root_joint__" type="fixed">
        <parent link="__world__"></parent>
        <child link="link_base"></child>
    </joint>

    Otherwise, generates an extra dummy link and a dummy joint because SAPIEN allows
    joint and child link to have different frames. Sample output:
    <joint name="joint1" type="revolute">
        <origin rpy="3.1415927410125732 -0.0 0.0" xyz="0.0 0.0 0.0"></origin>
        <axis xyz="1 0 0"></axis>
        <parent link="link_base"></parent>
        <child link="link_dummy_1"></child>
        <limit effort="100.0" lower="-6.2831855" upper="6.2831855" velocity="0"></limit>
        <dynamics damping="50000.0" friction="0.05000000074505806"></dynamics>
    </joint>
    <link name="link_dummy_1"></link>
    <joint name="joint_dummy_1" type="fixed">
        <origin rpy="3.1415927410125732 -0.0 0.0" xyz="-1.0 1.0 2.0"></origin>
        <parent link="link_dummy_1"></parent>
        <child link="link_1"></child>
    </joint>
    """
    if joint.parent_link is None:  # joint from root link to first robot link
        type = "fixed" if joint.type == "fixed" else "floating"
        elem_joint = ET.Element("joint", {"name": "__root_joint__", "type": type})
        ET.SubElement(elem_joint, "parent", {"link": "__world__"})
        ET.SubElement(elem_joint, "child", {"link": joint.child_link.name})
        return [elem_joint]

    if joint.type == "fixed":
        type = "fixed"
    elif joint.type == "prismatic":
        type = "prismatic"
    elif joint.type.startswith("revolute"):
        if joint.limit[0][0] < -1e5 and joint.limit[0][1] > 1e5:
            type = "continuous"
        else:
            type = "revolute"
    else:
        raise Exception(f"invalid joint type {joint.type}")

    j2p = joint.pose_in_parent
    c2j = joint.pose_in_child.inv()

    # dummy link at joint frame
    elem_dummy_link = ET.Element(
        "link", {"name": f"link_dummy_{joint.child_link.index}"}
    )

    # joint connecting parent and dummy
    elem_joint = ET.Element("joint", {"name": joint.name, "type": type})
    angles = j2p.rpy
    ET.SubElement(
        elem_joint,
        "origin",
        {
            "xyz": f"{j2p.p[0]} {j2p.p[1]} {j2p.p[2]}",
            "rpy": f"{angles[0]} {angles[1]} {angles[2]}",
        },
    )
    ET.SubElement(elem_joint, "axis", {"xyz": "1 0 0"})
    ET.SubElement(elem_joint, "parent", {"link": joint.parent_link.name})
    ET.SubElement(elem_joint, "child", {"link": f"link_dummy_{joint.child_link.index}"})
    if type == "prismatic" or type == "revolute" or type == "continuous":
        limit_attrib = {
            "effort": str(min(joint.force_limit, 1e5)),
            "velocity": "0",
            "lower": str(max(joint.limit[0][0], -1e5)),
            "upper": str(min(joint.limit[0][1], 1e5)),
        }
        if type == "continuous":
            limit_attrib.pop("lower")
            limit_attrib.pop("upper")

        ET.SubElement(elem_joint, "limit", limit_attrib)
        ET.SubElement(
            elem_joint,
            "dynamics",
            {"damping": str(joint.damping), "friction": str(joint.friction)},
        )
        # TODO: joint mimic

    # fixed joint connecting dummy and child
    elem_dummy_joint = ET.Element(
        "joint", {"name": f"joint_dummy_{joint.child_link.index}", "type": "fixed"}
    )
    angles = c2j.rpy
    ET.SubElement(
        elem_dummy_joint,
        "origin",
        {
            "xyz": f"{c2j.p[0]} {c2j.p[1]} {c2j.p[2]}",
            "rpy": f"{angles[0]} {angles[1]} {angles[2]}",
        },
    )
    # NOTE: fixed joint does not need axis
    # ET.SubElement(elem_dummy_joint, "axis", {"xyz": "0 0 0"})
    ET.SubElement(
        elem_dummy_joint, "parent", {"link": f"link_dummy_{joint.child_link.index}"}
    )
    ET.SubElement(elem_dummy_joint, "child", {"link": joint.child_link.name})

    return [elem_joint, elem_dummy_link, elem_dummy_joint]


def export_kinematic_chain_xml(articulation: PhysxArticulation) -> ET.Element:
    from .conversion import convert_object_name

    elem_robot = ET.Element("robot", {"name": convert_object_name(articulation)})
    ET.SubElement(elem_robot, "link", {"name": "__world__"})  # root link

    for l in articulation.links:
        elem_robot.append(export_link(l))

    for j in articulation.joints:
        elem_robot.extend(export_joint(j))

    return elem_robot


def export_kinematic_chain_urdf(
    articulation: PhysxArticulation, force_fix_root: bool = False
) -> str:
    """
    Exports the articulation's kinematic chain URDF as a string
    (without visual/collision elements for links)

    :param force_fix_root: Change all floating joints to fixed joints.
    """
    xml = export_kinematic_chain_xml(articulation)
    if force_fix_root:
        for j in xml.findall("joint"):
            if j.attrib["type"] == "floating":
                j.attrib["type"] = "fixed"

    # Indent the XML for better readability (available only in python >= 3.9)
    # ET.indent(xml, '  ')
    return ET.tostring(xml, encoding="utf8").decode()
