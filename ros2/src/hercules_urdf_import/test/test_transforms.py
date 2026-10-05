import math
import random

from hercules_urdf_import.transforms import (
    flu_to_frd, flu_to_frd_point, flu_to_frd_quat, matrix_to_quat, OPTICAL_FROM_BODY,
    quat_from_rpy, quat_rotate, quat_to_matrix, quat_to_rpy, settings_pose, Transform)
import pytest


def close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def same_rotation(q1, q2, tol=1e-9):
    return abs(abs(sum(a * b for a, b in zip(q1, q2))) - 1.0) <= tol


def rot_x(a):
    return ((1, 0, 0), (0, math.cos(a), -math.sin(a)), (0, math.sin(a), math.cos(a)))


def rot_y(a):
    return ((math.cos(a), 0, math.sin(a)), (0, 1, 0), (-math.sin(a), 0, math.cos(a)))


def rot_z(a):
    return ((math.cos(a), -math.sin(a), 0), (math.sin(a), math.cos(a), 0), (0, 0, 1))


def matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def test_rpy_is_fixed_axis_xyz():
    roll, pitch, yaw = 0.3, -0.7, 2.1
    expected = matmul(rot_z(yaw), matmul(rot_y(pitch), rot_x(roll)))
    got = quat_to_matrix(quat_from_rpy(roll, pitch, yaw))
    for row_e, row_g in zip(expected, got):
        assert close(row_e, row_g)


def test_rpy_round_trip_random():
    rng = random.Random(7)
    for _ in range(500):
        rpy = (rng.uniform(-math.pi, math.pi), rng.uniform(-1.5, 1.5),
               rng.uniform(-math.pi, math.pi))
        assert close(quat_to_rpy(quat_from_rpy(*rpy)), rpy, 1e-9)


def test_rpy_gimbal_lock_keeps_the_rotation():
    q = quat_from_rpy(0.4, math.pi / 2, 1.0)
    assert same_rotation(quat_from_rpy(*quat_to_rpy(q)), q, 1e-9)


def test_matrix_quat_round_trip_all_branches():
    for rpy in ((0, 0, 0), (math.pi, 0, 0), (0, math.pi, 0), (0, 0, math.pi),
                (2.0, 0.1, -0.3), (0.1, 3.0, 0.2)):
        q = quat_from_rpy(*rpy)
        assert same_rotation(matrix_to_quat(quat_to_matrix(q)), q)


def test_transform_compose_and_inverse():
    a = Transform.from_xyz_rpy((1, 2, 3), (0.1, 0.2, 0.3))
    b = Transform.from_xyz_rpy((-0.5, 0.4, 2), (-0.3, 0.9, 1.4))
    p = (0.3, -0.2, 0.9)
    assert close((a * b).apply(p), a.apply(b.apply(p)))
    assert (a * a.inverse()).is_close(Transform(), 1e-12)


def test_flu_to_frd_point_and_frame_meaning():
    # 1 m left and 2 m up in ROS is Y = -1, Z = -2 in settings.
    assert flu_to_frd_point((0.5, 1.0, 2.0)) == (0.5, -1.0, -2.0)


@pytest.mark.parametrize('flu_rpy, frd_rpy', [
    # roll right-side-down is the same rotation in both frames
    ((0.2, 0, 0), (0.2, 0, 0)),
    # ROS pitch +0.2 tips the nose down; nose down is negative FRD pitch
    ((0, 0.2, 0), (0, -0.2, 0)),
    # ROS yaw +0.3 turns left; FRD yaw is clockwise seen from above
    ((0, 0, 0.3), (0, 0, -0.3)),
])
def test_flu_to_frd_rotation_signs(flu_rpy, frd_rpy):
    q = flu_to_frd_quat(quat_from_rpy(*flu_rpy))
    assert close(quat_to_rpy(q), frd_rpy)


def test_flu_to_frd_preserves_geometry():
    t = Transform.from_xyz_rpy((0.3, -0.2, 1.1), (0.4, -0.3, 2.2))
    p = (0.7, 0.1, -0.4)
    frd = flu_to_frd(t)
    assert close(frd.apply(flu_to_frd_point(p)), flu_to_frd_point(t.apply(p)))


def test_settings_pose_matches_ros_wrapper_conversion():
    # The ROS wrapper turns a settings pose back into ROS with tf2 setRPY and
    # convert_tf_msg_to_ros (negate y and z of translation and quaternion).
    t = Transform.from_xyz_rpy((0.45, -0.1, 0.2), (0.05, 0.2, -0.1))
    s = settings_pose(t)
    w, x, y, z = quat_from_rpy(*(math.radians(s[k]) for k in ('Roll', 'Pitch', 'Yaw')))
    back = Transform((w, x, -y, -z), (s['X'], -s['Y'], -s['Z']))
    assert back.is_close(t, 1e-6)


def test_optical_frame_matches_ros_wrapper_quaternion():
    # hercules_ros_wrapper: tf2::Quaternion(0.5, -0.5, 0.5, -0.5) is (x, y, z, w)
    assert same_rotation(OPTICAL_FROM_BODY.rotation, (-0.5, 0.5, -0.5, 0.5))
    # optical z looks along body x, optical x points body right (-y)
    assert close(quat_rotate(OPTICAL_FROM_BODY.rotation, (0, 0, 1)), (1, 0, 0))
    assert close(quat_rotate(OPTICAL_FROM_BODY.rotation, (1, 0, 0)), (0, -1, 0))


def test_settings_pose_has_no_negative_zero():
    s = settings_pose(Transform())
    assert all(math.copysign(1.0, v) > 0 for v in s.values())
