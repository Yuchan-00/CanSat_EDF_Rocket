"""Quaternion utilities shared by the controller and the 6-DOF simulation.

Convention (docs/SPEC.md §8.1): ``q = (w, x, y, z)``, unit norm, Hamilton
product.  ``q`` is the attitude of the body frame *b* with respect to the world
frame *w*: ``R(q)`` maps body-frame vectors into the world frame,
``v_w = R(q) v_b``; the inverse rotation ``R(q)^T = R(q*)`` maps world vectors
into the body frame.  This is the spec's "w -> b" attitude quaternion (the
rotation that carries the world axes onto the body axes).  Body rates
``omega_b`` integrate as ``dq/dt = 1/2 q (x) (0, omega_b)``.

Invariant: every function is pure, numpy-only and works on plain ``(4,)`` /
``(3,)`` float arrays so the formulas transcribe 1:1 into the firmware
(``firmware/teensy_tvc/src/core/quat.h``).
"""
from __future__ import annotations

import math

import numpy as np

IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])


def normalize(q: np.ndarray) -> np.ndarray:
    """Return ``q / |q|``; a (near-)zero quaternion becomes the identity."""
    n = math.sqrt(float(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]))
    if n < 1e-12:
        return IDENTITY.copy()
    return np.asarray(q, dtype=float) / n


def multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product ``a (x) b`` (apply ``b`` first, then ``a`` in the
    body->world convention: ``R(a (x) b) = R(a) R(b)``)."""
    aw, ax, ay, az = float(a[0]), float(a[1]), float(a[2]), float(a[3])
    bw, bx, by, bz = float(b[0]), float(b[1]), float(b[2]), float(b[3])
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def conjugate(q: np.ndarray) -> np.ndarray:
    """``q* = (w, -x, -y, -z)``; equals the inverse for a unit quaternion."""
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def from_axis_angle(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    """Unit quaternion for a rotation of ``angle_rad`` about ``axis``."""
    a = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        return IDENTITY.copy()
    a = a / n
    s = math.sin(0.5 * angle_rad)
    return np.array([math.cos(0.5 * angle_rad), s * a[0], s * a[1], s * a[2]])


def from_rotvec(rv: np.ndarray) -> np.ndarray:
    """Exponential map: rotation vector (axis * angle, rad) -> quaternion."""
    rv = np.asarray(rv, dtype=float)
    angle = float(np.linalg.norm(rv))
    if angle < 1e-9:
        # first-order expansion: q ~ (1, rv/2), renormalised
        return normalize(np.array([1.0, 0.5 * rv[0], 0.5 * rv[1], 0.5 * rv[2]]))
    return from_axis_angle(rv, angle)


def to_rotvec(q: np.ndarray) -> np.ndarray:
    """Logarithmic map: quaternion -> rotation vector (rad), shortest path.

    Exact form of the spec's attitude error ``e = 2 atan2(|v|, w) v / |v|``;
    for small angles it equals ``2 v``.
    """
    w, v = float(q[0]), np.asarray(q[1:4], dtype=float)
    if w < 0.0:                      # shortest path
        w, v = -w, -v
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return 2.0 * v
    angle = 2.0 * math.atan2(n, w)
    return angle * v / n


def to_matrix(q: np.ndarray) -> np.ndarray:
    """Rotation matrix ``R(q)`` (body -> world), columns = body axes in world."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    ww, xx, yy, zz = w * w, x * x, y * y, z * z
    return np.array([
        [ww + xx - yy - zz, 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), ww - xx + yy - zz, 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), ww - xx - yy + zz],
    ])


def rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """``R(q) v`` - body vector expressed in the world frame."""
    return to_matrix(q) @ np.asarray(v, dtype=float)


def rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """``R(q)^T v`` - world vector expressed in the body frame."""
    return to_matrix(q).T @ np.asarray(v, dtype=float)


def derivative(q: np.ndarray, omega_b: np.ndarray) -> np.ndarray:
    """``dq/dt = 1/2 q (x) (0, omega_b)`` for body-frame angular rate."""
    return 0.5 * multiply(q, np.array([0.0, omega_b[0], omega_b[1], omega_b[2]]))


def integrate(q: np.ndarray, omega_b: np.ndarray, dt: float) -> np.ndarray:
    """Exact integration over ``dt`` for constant body rate: ``q (x) exp(omega dt)``."""
    return normalize(multiply(q, from_rotvec(np.asarray(omega_b, dtype=float) * dt)))


def error(q: np.ndarray, q_sp: np.ndarray) -> np.ndarray:
    """Attitude error quaternion ``q_e = q^-1 (x) q_sp`` (body frame), ``w >= 0``.

    ``R(q_e) = R(q)^T R(q_sp)``: the rotation, expressed in the current body
    frame, that carries the body onto the setpoint (SPEC §8.6).
    """
    qe = multiply(conjugate(q), q_sp)
    if qe[0] < 0.0:
        qe = -qe
    return qe


def reduced_error(qe: np.ndarray) -> np.ndarray:
    """v0.9 (the v0.8.2 study's candidate D, ported): the attitude error of ``q_e = error(q, q_sp)`` (w >= 0) split into a
    tilt part that a heading error cannot turn and a yaw part of its own - no trig, square root or division (4 + 1
    products, 2 sums, 3 doublings).

    Tilt: the set point's z axis in the current body frame is ``R(q_e) e_z = (2(xz + wy), 2(yz - wx), w^2 - x^2 - y^2
    + z^2)``; the reduced-attitude error is ``e_z x z_sp`` = (-(2(yz - wx)), 2(xz + wy)):

        e_x = 2 (w x - y z),   e_y = 2 (w y + x z)      = sin(sigma) n

    (sigma = angle between the body z axis and the set point's z axis, n = the body-frame unit axis that turns the one
    onto the other).  ``R(q_e) e_z`` does not change when q_e is multiplied on the right by a rotation about z, so a
    heading error never rotates this vector (the rotation vector's xy part turns by phi / 2 for a heading error phi and
    is scaled by (phi / 2) / sin(phi / 2)).  Quadratic in q_e: the sign of q_e does not matter.

    Yaw: with q_e = swing (x) twist (twist = (c_t, 0, 0, s_t) about body z, swing about a body-xy axis, c_s =
    cos(sigma / 2) > 0) the z component is z = c_s s_t, so

        e_z = 2 z = 2 cos(sigma / 2) sin(phi / 2)

    (phi = the twist angle in [-pi, pi] because w = c_s c_t >= 0).  Small angles: e = the rotation vector to second
    order (e_xy = sigma n - sigma^3 n / 6, e_z = phi - phi^3 / 24 at sigma = 0), so the linearised loop - gains and
    margins - is the same as with ``to_rotvec``.  Large angles: |e_xy| = sin(sigma) (the xy rate set point saturates
    later than with the rotation vector), e_z saturates at phi = 180 deg with 2.
    """
    w, x, y, z = float(qe[0]), float(qe[1]), float(qe[2]), float(qe[3])
    return np.array([2.0 * (w * x - y * z), 2.0 * (w * y + x * z), 2.0 * z])


def yaw(q: np.ndarray) -> float:
    """Heading of the body x axis projected on the world xy plane (rad, ZYX yaw)."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def tilt(q: np.ndarray) -> float:
    """Angle (rad) between the body z axis and world up: ``acos(R[2,2])``."""
    x, y = float(q[1]), float(q[2])
    c = 1.0 - 2.0 * (x * x + y * y)
    return math.acos(max(-1.0, min(1.0, c)))


def from_yaw(psi_rad: float) -> np.ndarray:
    """Upright attitude (body z = world z) with heading ``psi_rad``."""
    return np.array([math.cos(0.5 * psi_rad), 0.0, 0.0, math.sin(0.5 * psi_rad)])


def from_euler(roll: float, pitch: float, yaw_: float) -> np.ndarray:
    """ZYX (yaw-pitch-roll) Euler angles (rad) -> quaternion, body->world."""
    qz = from_axis_angle(np.array([0.0, 0.0, 1.0]), yaw_)
    qy = from_axis_angle(np.array([0.0, 1.0, 0.0]), pitch)
    qx = from_axis_angle(np.array([1.0, 0.0, 0.0]), roll)
    return normalize(multiply(multiply(qz, qy), qx))


def to_euler(q: np.ndarray) -> tuple[float, float, float]:
    """Quaternion -> ZYX Euler ``(roll, pitch, yaw)`` in rad."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, s)))
    yaw_ = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw_


def tilt_vector(q: np.ndarray) -> np.ndarray:
    """Body-frame rotation vector (rad) from the current attitude to upright
    *at the current yaw*; its xy components are the signed tilt about the
    body x and y axes (z component is ~0 by construction)."""
    return to_rotvec(error(q, from_yaw(yaw(q))))


def random_unit(rng: np.random.Generator) -> np.ndarray:
    """Uniformly distributed unit quaternion (for tests)."""
    v = rng.normal(size=4)
    return normalize(v)
