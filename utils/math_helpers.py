"""Shared math and rotation utilities that grow across modules.

Module 1: 2D rotation matrices, 2D homogeneous transforms
Module 2+: DH transforms, 3D rotations, skew-symmetric, etc.

Students are expected to build many of these from scratch in TODO blocks
before importing the canonical versions from here in later modules.
"""

import numpy as np


# ---------------------------------------------------------------------------
# Module 1: 2D helpers
# ---------------------------------------------------------------------------

def rot2d(theta: float) -> np.ndarray:
    """2x2 rotation matrix for angle theta (radians).

    Args:
        theta: Rotation angle in radians.

    Returns:
        (2, 2) rotation matrix.
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s],
                     [s,  c]])


def homogeneous_2d(theta: float, tx: float, ty: float) -> np.ndarray:
    """3x3 homogeneous transformation matrix for 2D (rotation + translation).

    Args:
        theta: Rotation angle in radians.
        tx: Translation along x.
        ty: Translation along y.

    Returns:
        (3, 3) homogeneous transformation matrix.
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, tx],
                     [s,  c, ty],
                     [0,  0,  1]])


def wrap_angle(theta: float) -> float:
    """Wrap angle to [-pi, pi].

    Args:
        theta: Angle in radians.

    Returns:
        Wrapped angle in [-pi, pi].
    """
    return (theta + np.pi) % (2 * np.pi) - np.pi


# ---------------------------------------------------------------------------
# Module 2+: 3D helpers (stubs - will be expanded)
# ---------------------------------------------------------------------------

def rot_x(theta: float) -> np.ndarray:
    """3x3 rotation matrix about the x-axis."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1, 0,  0],
                     [0, c, -s],
                     [0, s,  c]])


def rot_y(theta: float) -> np.ndarray:
    """3x3 rotation matrix about the y-axis."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]])


def rot_z(theta: float) -> np.ndarray:
    """3x3 rotation matrix about the z-axis."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])


def homogeneous_3d(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """4x4 homogeneous transformation from a 3x3 rotation and 3x1 translation.

    Args:
        rotation: (3, 3) rotation matrix.
        translation: (3,) translation vector.

    Returns:
        (4, 4) homogeneous transformation matrix.
    """
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    return T


def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """4x4 homogeneous transform from Denavit-Hartenberg parameters.

    Convention: Modified DH (Craig) -
        T = Rot_z(theta) @ Trans_z(d) @ Trans_x(a) @ Rot_x(alpha)

    Args:
        theta: Joint angle (radians).
        d: Link offset along z.
        a: Link length along x.
        alpha: Link twist (radians).

    Returns:
        (4, 4) transformation matrix.
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [ 0,       sa,       ca,      d],
        [ 0,        0,        0,      1],
    ])


def skew(v: np.ndarray) -> np.ndarray:
    """3x3 skew-symmetric matrix from a 3-vector.

    Useful for computing cross products as matrix multiplications:
        skew(a) @ b == np.cross(a, b)

    Args:
        v: (3,) vector.

    Returns:
        (3, 3) skew-symmetric matrix.
    """
    return np.array([[ 0,   -v[2],  v[1]],
                     [ v[2],  0,   -v[0]],
                     [-v[1],  v[0],  0  ]])
