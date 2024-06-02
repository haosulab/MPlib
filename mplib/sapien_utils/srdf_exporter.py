from __future__ import annotations

from xml.etree import ElementTree as ET

from sapien.physx import PhysxArticulation


def check_collision_group(groupA: list[int], groupB: list[int]) -> bool:
    """
    groupA and groupB are the collision groups of 2 collision shapes A and B.
    Return True if A and B will collide and False is the collision is disabled.

    Collision groups determine the collision behavior of objects.
    Let A.gx denote the collision group x of collision shape A.
    Collision shape A and B will collide iff the following condition holds:

    ((A.g0 & B.g1) or (A.g1 & B.g0)) and (
        not ((A.g2 & B.g2) and ((A.g3 & 0xffff) == (B.g3 & 0xffff)))
    )

    Here is some explanation:
    g2 is the "ignore group" and g3 is the "id group".
    The only the lower 16 bits of the id group is used
    since the upper 16 bits are reserved for other purposes in the future.
    When 2 collision shapes have the same ID (g3), then if any of their g2 bits match,
    their collisions are definitely ignored.

    If after testing g2 and g3, the objects may collide, g0 and g1 come into play.
    g0 is the "contact type group" and g1 is the "contact affinity group".
    Collision shapes collide only when a bit in the contact type of the first shape
    matches a bit in the contact affinity of the second shape.
    """
    A_g0, A_g1, A_g2, A_g3 = groupA
    B_g0, B_g1, B_g2, B_g3 = groupB
    return ((A_g0 & B_g1) or (A_g1 & B_g0)) and (
        not ((A_g2 & B_g2) and ((A_g3 & 0xFFFF) == (B_g3 & 0xFFFF)))
    )


def export_srdf_xml(articulation: PhysxArticulation) -> ET.Element:
    """
    Export an articulation link disable_collisions. Generates
    <disable_collisions link1="panda_link5" link2="panda_link6" reason="Adjacent"/>
    <disable_collisions link1="panda_link5" link2="panda_link7" reason="Default"/>
    """
    from .conversion import convert_object_name

    elem_robot = ET.Element("robot", {"name": convert_object_name(articulation)})

    # Get all links with collision shapes
    colliding_links = [l for l in articulation.links if len(l.collision_shapes) > 0]

    for i, link1 in enumerate(colliding_links):
        # parent-child collision
        parent_link = link1.parent
        if parent_link is not None and len(parent_link.collision_shapes) > 0:
            ET.SubElement(
                elem_robot,
                "disable_collisions",
                {"link1": link1.name, "link2": parent_link.name, "reason": "Adjacent"},
            )

        for j in range(i):
            link2 = colliding_links[j]
            if link2 is parent_link:  # already checked for adjacent links
                continue

            shapes1 = link1.collision_shapes
            shapes2 = link2.collision_shapes

            enabled_collisions = {
                check_collision_group(shape1.collision_groups, shape2.collision_groups)
                for shape1 in shapes1
                for shape2 in shapes2
            }
            if len(enabled_collisions) > 1:
                raise RuntimeError(
                    "collision_groups of all collision_shapes must be the same for any"
                    " link, found violating links: "
                    f"{link1.name}(id={link1.entity.per_scene_id}) "
                    f"{link2.name}(id={link2.entity.per_scene_id})"
                )

            if not enabled_collisions.pop():
                ET.SubElement(
                    elem_robot,
                    "disable_collisions",
                    {"link1": link1.name, "link2": link2.name, "reason": "Default"},
                )

    return elem_robot


def export_srdf(articulation: PhysxArticulation) -> str:
    """Exports the articulation's SRDF as a string (only disable_collisions element)"""
    xml = export_srdf_xml(articulation)

    # Indent the XML for better readability (available only in python >= 3.9)
    # ET.indent(xml, '  ')
    return ET.tostring(xml, encoding="utf8").decode()
