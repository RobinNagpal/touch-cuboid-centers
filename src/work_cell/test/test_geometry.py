import math

import numpy as np
import pytest
from work_cell.cuboids.geometry import Cuboid, largest_touchable_face

TABLE_Z = 0.75


def box(length, width, height, yaw=0.0, x=0.4, y=0.0):
    return Cuboid(
        centre=np.array([x, y, TABLE_Z + height / 2]),
        yaw=yaw,
        size=np.array([length, width, height]),
    )


def test_a_cuboid_has_six_faces_in_three_matching_pairs():
    faces = box(0.08, 0.05, 0.03).faces()
    assert len(faces) == 6
    areas = sorted(round(face.area, 6) for face in faces)
    assert areas == pytest.approx([0.0015, 0.0015, 0.0024, 0.0024, 0.004, 0.004])


def test_the_three_distinct_areas_are_the_products_of_the_side_pairs():
    areas = box(0.08, 0.05, 0.03).distinct_face_areas()
    assert areas["length x width"] == pytest.approx(0.08 * 0.05)
    assert areas["length x height"] == pytest.approx(0.08 * 0.03)
    assert areas["width x height"] == pytest.approx(0.05 * 0.03)


def test_face_centres_sit_half_a_side_out_along_the_face_normal():
    cuboid = box(0.08, 0.05, 0.03, yaw=0.3)
    halves = {"length": 0.04, "width": 0.025, "height": 0.015}
    for face in cuboid.faces():
        offset = face.centre - cuboid.centre
        half = halves[face.name.lstrip("+-")]
        assert offset == pytest.approx(face.normal * half)


def test_a_flat_box_is_touched_on_top():
    # 8 x 5 on top beats 8 x 3 and 5 x 3 on the sides.
    face = largest_touchable_face(box(0.08, 0.05, 0.03), np.zeros(3))
    assert face.name == "+height"
    assert face.normal == pytest.approx([0, 0, 1])
    assert face.centre[2] == pytest.approx(TABLE_Z + 0.03)


def test_a_tall_box_is_touched_on_its_side():
    # Standing on its 5 x 3 end, the 8 x 5 face is now vertical.
    face = largest_touchable_face(box(0.05, 0.03, 0.08), np.zeros(3))
    assert face.name in ("+width", "-width")
    assert face.normal[2] == pytest.approx(0, abs=1e-9)
    assert face.area == pytest.approx(0.05 * 0.08)


def test_the_face_underneath_is_never_chosen():
    # A cube ties on all six faces; the one against the table must lose.
    for yaw in (0.0, 0.4, -1.1):
        face = largest_touchable_face(box(0.05, 0.05, 0.05, yaw=yaw), np.zeros(3))
        assert face.normal[2] > -0.5


def test_of_two_equal_faces_the_one_nearer_the_arm_wins():
    # The two 5 x 8 faces look identical apart from which side of the box they
    # are on. Sitting the box off to one side makes one of them the near one.
    cuboid = box(0.05, 0.03, 0.08, x=0.4, y=0.2)
    face = largest_touchable_face(cuboid, np.array([0.0, 0.0, TABLE_Z]))
    assert face.name == "-width"
    assert face.normal == pytest.approx([0, -1, 0])


def test_yaw_turns_the_side_normals_with_the_box():
    face = largest_touchable_face(box(0.05, 0.03, 0.08, yaw=math.pi / 2), np.zeros(3))
    assert abs(face.normal[0]) == pytest.approx(1.0, abs=1e-9)
