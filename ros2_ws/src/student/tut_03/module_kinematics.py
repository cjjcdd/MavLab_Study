"""Kinematics utilities for marine craft simulation.

This module implements fundamental operations used in rigid body
kinematics, including the skew‑symmetric matrix representation of the
cross product (also known as the S‑matrix), conversion of Euler
angles to rotation matrices and the mapping between body angular
velocity and Euler angle rates.  The implementations follow the
notation of Fossen (``Handbook of Marine Craft Hydrodynamics and
Motion Control``) and are widely used in marine vehicle modelling.

Functions
---------
Smat(vec):
    Return the skew‑symmetric matrix associated with ``vec``.
eul_to_rotm(eul):
    Convert roll/pitch/yaw angles to a body‑to‑inertial rotation matrix.
eul_rate_matrix(eul):
    Compute the transformation from body angular velocity to Euler
    angle rates.
"""

from __future__ import annotations

import numpy as np
from typing import Iterable


def Smat(vec: Iterable[float]) -> np.ndarray:
    """Construct a skew‑symmetric matrix from a 3‑element vector.

    Given a 3×1 vector ``v = [v₁, v₂, v₃]^T``, this function returns
    the 3×3 matrix ``S(v)`` such that ``S(v) w = v × w`` for any
    vector ``w``.  The skew‑symmetric matrix is defined as::

        S(v) = [[ 0, -v₃,  v₂],
                [ v₃,  0, -v₁],
                [-v₂, v₁,  0 ]].

    Parameters
    ----------
    vec: Iterable[float]
        The input vector.  It must have exactly three elements.

    Returns
    -------
    np.ndarray
        A 3×3 skew‑symmetric matrix.
    """
    v = np.asarray(vec, dtype=float).flatten()
    if v.size != 3:
        raise ValueError("Smat() expects a 3‑element vector")
    v1, v2, v3 = v
    return np.array([[0.0, -v3,  v2],
                     [v3,  0.0, -v1],
                     [-v2, v1,  0.0]])


def eul_to_rotm(eul: Iterable[float]) -> np.ndarray:
    """Return the rotation matrix from body to inertial coordinates.

    The Euler angles are given in the order roll ``φ`` (rotation about
    the x‑axis), pitch ``θ`` (rotation about the y‑axis) and yaw ``ψ``
    (rotation about the z‑axis).  The resulting rotation matrix
    transforms a vector expressed in the body‑fixed frame to the
    inertial frame according to the 3–2–1 (roll–pitch–yaw) sequence::

        R(φ, θ, ψ) = R_z(ψ) R_y(θ) R_x(φ)

    where ``R_z``, ``R_y`` and ``R_x`` are the standard right‑handed
    rotation matrices about the respective axes.  The explicit
    expression is taken from Fossen (2011, eq. 2.27).

    Parameters
    ----------
    eul: Iterable[float]
        The Euler angles ``[φ, θ, ψ]`` in radians.

    Returns
    -------
    np.ndarray
        A 3×3 rotation matrix ``R`` such that ``v_inertial = R v_body``.
    """
    eul = np.asarray(eul, dtype=float).flatten()
    if eul.size != 3:
        raise ValueError("eul_to_rotm() expects a 3‑element vector")
    phi, theta, psi = eul

    cphi = np.cos(phi)
    sphi = np.sin(phi)
    cth = np.cos(theta)
    sth = np.sin(theta)
    cpsi = np.cos(psi)
    spsi = np.sin(psi)

    # Rotation matrix composed of yaw, pitch and roll rotations
    R = np.array([
        [cpsi*cth, -spsi*cphi + cpsi*sth*sphi,  spsi*sphi + cpsi*sth*cphi],
        [spsi*cth,  cpsi*cphi + spsi*sth*sphi, -cpsi*sphi + spsi*sth*cphi],
        [  -sth,                   cth*sphi,                    cth*cphi]
    ])
    return R

def eul_to_quat(eul: Iterable[float], order: str = 'ZYX', deg: bool = False) -> np.ndarray:
    """
    Convert Euler angles to quaternion.

    Parameters
    ----------
    eul: Iterable[float]
        The Euler angles [roll, pitch, yaw] in radians (or degrees if deg=True).
    order: str
        The order of rotation. Default is 'ZYX'.
    deg: bool
        If True, input Euler angles are in degrees. If False, they are in radians.

    Returns
    -------
    np.ndarray
        A quaternion [w, x, y, z].
    """
    if deg:
        eul = np.radians(eul)  # Convert degrees to radians if needed

    # Unpack Euler angles
    phi, theta, psi = eul

    # Calculate half-angles
    cy = np.cos(psi * 0.5)
    sy = np.sin(psi * 0.5)
    cp = np.cos(theta * 0.5)
    sp = np.sin(theta * 0.5)
    cr = np.cos(phi * 0.5)
    sr = np.sin(phi * 0.5)

    # Calculate quaternion components (ZYX convention)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z])


def eul_rate_matrix(eul: Iterable[float]) -> np.ndarray:
    """Map body angular velocity to Euler angle rates.

    This function returns the 3×3 transformation matrix ``T(φ, θ)``
    relating the body angular velocity vector ``ω = [p, q, r]^T`` to
    the time derivatives of the Euler angles ``[φ̇, θ̇, ψ̇]^T``::

        [φ̇]   [1     sinφ tanθ     cosφ tanθ] [p]
        [θ̇] = [0      cosφ        -sinφ   ] [q]
        [ψ̇]   [0  sinφ/cosθ  cosφ/cosθ] [r]

    It is assumed that ``cosθ`` is non‑zero (i.e., the pitch angle is
    not ±90°).  The implementation guards against division by zero by
    adding a small epsilon to the denominator; however, large pitch
    angles will still result in numerically ill‑conditioned matrices.

    Parameters
    ----------
    eul: Iterable[float]
        The Euler angles ``[φ, θ, ψ]`` in radians.

    Returns
    -------
    np.ndarray
        A 3×3 matrix ``T`` such that ``[φ̇, θ̇, ψ̇]^T = T ω``.
    """
    eul = np.asarray(eul, dtype=float).flatten()
    if eul.size != 3:
        raise ValueError("eul_rate_matrix() expects a 3‑element vector")
    phi, theta, _ = eul
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    cth = np.cos(theta)
    sth = np.sin(theta)

    # Avoid division by zero at cosθ = 0
    eps = 1e-8
    cth_safe = cth if abs(cth) > eps else np.sign(cth) * eps
    tan_th = sth / cth_safe

    T = np.array([
        [1.0, sphi * tan_th,  cphi * tan_th],
        [0.0, cphi,          -sphi         ],
        [0.0, sphi / cth_safe, cphi / cth_safe]
    ])
    return T

def quat_to_eul(quat: Iterable[float], order: str = 'ZYX', deg: bool = False) -> np.ndarray:
    """
    Convert quaternion to Euler angles.

    Parameters
    ----------
    quat : Iterable[float]
        Quaternion [w, x, y, z].
    order : str
        Rotation order (default 'ZYX' → yaw-pitch-roll).
    deg : bool
        If True, return angles in degrees. Default is radians.

    Returns
    -------
    np.ndarray
        Euler angles [roll, pitch, yaw] in radians or degrees.
    """
    w, x, y, z = quat

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2  # Gimbal lock
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    eul = np.array([roll, pitch, yaw])  # [φ, θ, ψ]

    if deg:
        return np.degrees(eul)
    return eul
