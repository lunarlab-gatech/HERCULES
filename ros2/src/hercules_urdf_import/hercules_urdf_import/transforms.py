"""
Rigid transforms and the frame conventions HERCULES uses.

Everything here is plain Python so the importer runs without ROS or NumPy.

Frame conventions
-----------------
* URDF and ROS (REP-103): FLU body axes (x forward, y left, z up), meters,
  ``rpy`` = fixed-axis roll/pitch/yaw, i.e. R = Rz(yaw) * Ry(pitch) * Rx(roll).
* HERCULES/AirSim settings: FRD body axes (x forward, y right, z down),
  meters for ``X/Y/Z`` and degrees for ``Roll/Pitch/Yaw`` composed the same
  way, R = Rz(yaw) * Ry(pitch) * Rx(roll) (``VectorMath::toQuaternion``).

FLU and FRD differ by a 180 degree rotation about x, ``C = diag(1, -1, -1)``,
so a point maps as ``(x, y, z) -> (x, -y, -z)`` and a rotation as
``R_frd = C * R_flu * C``.

Quaternions are ``(w, x, y, z)`` tuples throughout.
"""

import math

IDENTITY_QUAT = (1.0, 0.0, 0.0, 0.0)
ZERO_VEC = (0.0, 0.0, 0.0)


def quat_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    if n == 0.0:
        raise ValueError('zero-length quaternion')
    w, x, y, z = (c / n for c in q)
    # Canonical sign keeps outputs comparable between runs.
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    return (w, x, y, z)


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_rotate(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    # v + 2 * r x (r x v + w v), with r the vector part.
    cx = y * vz - z * vy + w * vx
    cy = z * vx - x * vz + w * vy
    cz = x * vy - y * vx + w * vz
    return (
        vx + 2.0 * (y * cz - z * cy),
        vy + 2.0 * (z * cx - x * cz),
        vz + 2.0 * (x * cy - y * cx),
    )


def quat_from_axis_angle(axis, angle):
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n == 0.0:
        raise ValueError('zero-length rotation axis')
    s = math.sin(angle / 2.0) / n
    return (math.cos(angle / 2.0), ax * s, ay * s, az * s)


def quat_from_rpy(roll, pitch, yaw):
    """Quaternion for R = Rz(yaw) * Ry(pitch) * Rx(roll), angles in radians."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def quat_to_matrix(q):
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def matrix_to_quat(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
             (m[1][0] - m[0][1]) / s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        q = ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s,
             (m[0][2] + m[2][0]) / s)
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        q = ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s,
             (m[1][2] + m[2][1]) / s)
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        q = ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s,
             (m[1][2] + m[2][1]) / s, 0.25 * s)
    return quat_normalize(q)


def quat_to_rpy(q):
    """
    Inverse of quat_from_rpy: (roll, pitch, yaw) in radians.

    At pitch = +/-90 degrees roll and yaw are not unique; roll is then set to
    zero and yaw absorbs the remaining rotation.
    """
    m = quat_to_matrix(quat_normalize(q))
    sp = max(-1.0, min(1.0, -m[2][0]))
    pitch = math.asin(sp)
    if abs(sp) > 1.0 - 1e-9:
        roll = 0.0
        yaw = math.atan2(-m[0][1], m[1][1])
    else:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    return (roll, pitch, yaw)


class Transform:
    """A rigid transform ``p_parent = R * p_child + t``."""

    __slots__ = ('rotation', 'translation')

    def __init__(self, rotation=IDENTITY_QUAT, translation=ZERO_VEC):
        self.rotation = quat_normalize(rotation)
        self.translation = tuple(float(c) for c in translation)

    @classmethod
    def from_xyz_rpy(cls, xyz=ZERO_VEC, rpy=ZERO_VEC):
        return cls(quat_from_rpy(*rpy), xyz)

    def __mul__(self, other):
        rotated = quat_rotate(self.rotation, other.translation)
        return Transform(
            quat_multiply(self.rotation, other.rotation),
            tuple(a + b for a, b in zip(self.translation, rotated)))

    def inverse(self):
        inv = quat_conjugate(self.rotation)
        t = quat_rotate(inv, self.translation)
        return Transform(inv, (-t[0], -t[1], -t[2]))

    def apply(self, point):
        r = quat_rotate(self.rotation, point)
        return (r[0] + self.translation[0], r[1] + self.translation[1],
                r[2] + self.translation[2])

    def rpy(self):
        return quat_to_rpy(self.rotation)

    def is_close(self, other, tol=1e-9):
        dq = quat_multiply(quat_conjugate(self.rotation), other.rotation)
        angle = 2.0 * math.acos(min(1.0, abs(dq[0])))
        dt = math.dist(self.translation, other.translation)
        return angle <= tol and dt <= tol

    def __repr__(self):
        return 'Transform(rotation=%r, translation=%r)' % (
            self.rotation, self.translation)


# 180 degrees about x: the FLU <-> FRD change of basis.
_FLIP_X = (0.0, 1.0, 0.0, 0.0)

# Body (x forward) -> optical (z forward, x right, y down) camera frame,
# rpy = (-pi/2, 0, -pi/2), matching REP-103 *_optical_frame links and the
# HERCULES ROS wrapper's <camera>_optical frame.
OPTICAL_FROM_BODY = Transform(quat_from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0))


def flu_to_frd_point(p):
    return (p[0], -p[1], -p[2])


def flu_to_frd_quat(q):
    """Rotation R_flu expressed in FRD axes: C * R * C."""
    return quat_normalize(quat_multiply(quat_multiply(_FLIP_X, q), _FLIP_X))


def flu_to_frd(transform):
    return Transform(flu_to_frd_quat(transform.rotation),
                     flu_to_frd_point(transform.translation))


def settings_pose(transform_flu):
    """Convert a body-relative FLU transform to settings ``X/Y/Z/Roll/Pitch/Yaw``."""
    frd = flu_to_frd(transform_flu)
    roll, pitch, yaw = quat_to_rpy(frd.rotation)
    x, y, z = frd.translation
    return {
        'X': _clean(x), 'Y': _clean(y), 'Z': _clean(z),
        'Roll': _clean(math.degrees(roll)),
        'Pitch': _clean(math.degrees(pitch)),
        'Yaw': _clean(math.degrees(yaw)),
    }


def _clean(value, digits=6):
    """Round for stable JSON and drop negative zero."""
    value = round(float(value), digits)
    return 0.0 if value == 0.0 else value
