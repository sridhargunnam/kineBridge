"""Module 2, Step 2: Forward Kinematics via Denavit-Hartenberg Parameters.

Reference: theory.md — Sections 2.3, 2.7, 2.8

Goal:
    Implement the DH transformation matrix and use it to compute FK for
    a multi-link arm by chaining DH transforms from a parameter table.

What you will learn:
    - How the 4 DH parameters (θ, d, a, α) define a transform
    - How to chain transforms to get the full FK
    - Verifying DH-based FK against the trig-based FK from Module 1
    - Reading position AND orientation from the final transform
"""

import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from modules.module_02_3d_dh.step_01_transforms import (
    make_transform, extract_position, extract_rotation
)


# ---------------------------------------------------------------------------
# YOUR TASK 1: Implement the DH transformation matrix
# ---------------------------------------------------------------------------

def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """Compute the 4x4 homogeneous transform from DH parameters.

    Standard DH convention:
        T = Rot_z(θ) · Trans_z(d) · Trans_x(a) · Rot_x(α)

    When multiplied out:
        T = | cos(θ)  -sin(θ)·cos(α)   sin(θ)·sin(α)   a·cos(θ) |
            | sin(θ)   cos(θ)·cos(α)  -cos(θ)·sin(α)   a·sin(θ) |
            |   0        sin(α)           cos(α)            d     |
            |   0          0                0               1     |

    Args:
        theta: Joint angle in radians (variable for revolute joints).
        d: Link offset along z-axis (variable for prismatic joints).
        a: Link length along x-axis.
        alpha: Link twist about x-axis in radians.

    Returns:
        (4, 4) homogeneous transformation matrix.

    Hint:
        You can either:
        (a) Directly fill in the 4x4 matrix using the formula above, OR
        (b) Build four separate transform matrices and multiply them:
            Rz(θ) @ Tz(d) @ Tx(a) @ Rx(α)
        Option (a) is more efficient; option (b) helps build intuition.
    """
    # === TODO START ===
    raise NotImplementedError("Implement dh_transform")
    # === TODO END ===


# ---------------------------------------------------------------------------
# YOUR TASK 2: Chain DH transforms to compute FK
# ---------------------------------------------------------------------------

def fk_from_dh_table(
    dh_table: list[dict],
    joint_angles: np.ndarray,
) -> np.ndarray:
    """Compute the full FK transform by chaining DH transforms.

    Args:
        dh_table: List of dicts, one per link, each containing:
            {
                'theta_offset': float,  # constant offset added to joint angle
                'd': float,
                'a': float,
                'alpha': float,
                'joint_type': str,      # 'revolute' or 'prismatic'
            }
        joint_angles: (n,) array of joint variable values.
            For revolute joints: θ_i = theta_offset_i + joint_angles[i]
            For prismatic joints: d_i = d_i + joint_angles[i]

    Returns:
        (4, 4) homogeneous transform T_0^n (base frame to end-effector).

    Hint:
        Initialize T = np.eye(4) (identity = base frame).
        Loop over each link i:
            - Determine the actual θ and d values based on joint type
            - Compute T_i = dh_transform(θ_i, d_i, a_i, α_i)
            - Update: T = T @ T_i
        The final T is the FK result.
    """
    # === TODO START ===
    raise NotImplementedError("Implement fk_from_dh_table")
    # === TODO END ===


# ---------------------------------------------------------------------------
# VERIFICATION (do not modify)
# ---------------------------------------------------------------------------

def verify_dh_transform():
    """Test the DH transform with known cases."""
    print("=" * 60)
    print("Verifying DH Transform")
    print("=" * 60)

    all_passed = True

    # Identity case: all zeros → identity matrix
    T = dh_transform(0, 0, 0, 0)
    ok = np.allclose(T, np.eye(4))
    status = "PASS" if ok else "FAIL"
    if not ok:
        all_passed = False
    print(f"  [{status}] DH(0,0,0,0) == Identity")

    # Pure translation along x: a=1, rest zero
    T = dh_transform(0, 0, 1.0, 0)
    expected_pos = np.array([1.0, 0.0, 0.0])
    ok = np.allclose(extract_position(T), expected_pos)
    status = "PASS" if ok else "FAIL"
    if not ok:
        all_passed = False
    print(f"  [{status}] DH(0,0,1,0) → position [1,0,0]")

    # Pure translation along z: d=0.5
    T = dh_transform(0, 0.5, 0, 0)
    expected_pos = np.array([0.0, 0.0, 0.5])
    ok = np.allclose(extract_position(T), expected_pos)
    status = "PASS" if ok else "FAIL"
    if not ok:
        all_passed = False
    print(f"  [{status}] DH(0,0.5,0,0) → position [0,0,0.5]")

    # Rotation about z by π/2, with a=1
    T = dh_transform(np.pi / 2, 0, 1.0, 0)
    expected_pos = np.array([0.0, 1.0, 0.0])
    ok = np.allclose(extract_position(T), expected_pos, atol=1e-10)
    status = "PASS" if ok else "FAIL"
    if not ok:
        all_passed = False
    print(f"  [{status}] DH(π/2,0,1,0) → position [0,1,0]")

    # Link twist: α = π/2 rotates the z-axis to point along y
    T = dh_transform(0, 0, 0, np.pi / 2)
    R = extract_rotation(T)
    new_z = R[:, 2]  # where the z-axis of the new frame points
    ok = np.allclose(new_z, [0, 1, 0], atol=1e-10)
    status = "PASS" if ok else "FAIL"
    if not ok:
        all_passed = False
        print(f"         Got z-axis: {new_z}")
    print(f"  [{status}] DH(0,0,0,π/2) → new z-axis is [0,1,0]")

    print("=" * 60)
    if all_passed:
        print("All DH transform tests passed!")
    else:
        print("Some tests FAILED.")
    print()
    return all_passed


def verify_fk_chain():
    """Verify FK chain against Module 1's trig-based FK for a 2-link planar arm."""
    print("=" * 60)
    print("Verifying FK Chain (2-link planar as DH)")
    print("=" * 60)

    L1, L2 = 1.0, 0.8

    # DH table for 2-link planar arm
    dh_table = [
        {"theta_offset": 0.0, "d": 0.0, "a": L1, "alpha": 0.0, "joint_type": "revolute"},
        {"theta_offset": 0.0, "d": 0.0, "a": L2, "alpha": 0.0, "joint_type": "revolute"},
    ]

    test_cases = [
        (np.array([0.0, 0.0]), np.array([L1 + L2, 0.0, 0.0]), "Both at 0"),
        (np.array([np.pi / 2, 0.0]), np.array([0.0, L1 + L2, 0.0]), "θ1=π/2"),
        (np.array([0.0, np.pi / 2]), np.array([L1, L2, 0.0]), "θ2=π/2"),
        (np.array([np.pi / 4, np.pi / 4]),
         np.array([L1 * np.cos(np.pi / 4) + L2 * np.cos(np.pi / 2),
                    L1 * np.sin(np.pi / 4) + L2 * np.sin(np.pi / 2), 0.0]),
         "Both π/4"),
    ]

    all_passed = True
    for angles, expected_pos, desc in test_cases:
        T = fk_from_dh_table(dh_table, angles)
        pos = extract_position(T)
        ok = np.allclose(pos, expected_pos, atol=1e-10)
        status = "PASS" if ok else "FAIL"
        if not ok:
            all_passed = False
        print(f"  [{status}] {desc}: pos={pos[:2]} (expected {expected_pos[:2]})")
        if not ok:
            print(f"         Full pos: {pos}, expected: {expected_pos}")

    print("=" * 60)
    if all_passed:
        print("FK chain matches trig-based Module 1 FK!")
    else:
        print("Some tests FAILED.")
    print()
    return all_passed


def verify_3link_3d():
    """Test a 3-link arm that moves in 3D (non-planar)."""
    print("=" * 60)
    print("Verifying 3-link 3D arm FK")
    print("=" * 60)

    # 3-link arm: link1 along z, link2 rotated 90° twist, link3 back
    dh_table = [
        {"theta_offset": 0.0, "d": 0.0, "a": 1.0, "alpha": np.pi / 2, "joint_type": "revolute"},
        {"theta_offset": 0.0, "d": 0.0, "a": 1.0, "alpha": 0.0, "joint_type": "revolute"},
        {"theta_offset": 0.0, "d": 0.0, "a": 0.5, "alpha": 0.0, "joint_type": "revolute"},
    ]

    # All joints at zero
    angles = np.array([0.0, 0.0, 0.0])
    T = fk_from_dh_table(dh_table, angles)
    pos = extract_position(T)

    # With α1 = π/2, the second link's z-axis points along y.
    # At all zero angles: link1 extends to (1,0,0), link2 extends to (2,0,0),
    # link3 extends to (2.5,0,0).
    expected = np.array([2.5, 0.0, 0.0])
    ok = np.allclose(pos, expected, atol=1e-10)
    status = "PASS" if ok else "FAIL"
    print(f"  [{status}] All zeros: pos={pos} (expected {expected})")

    # Joint 1 = π/2: link1 goes to (0,1,0), rest follows
    angles2 = np.array([np.pi / 2, 0.0, 0.0])
    T2 = fk_from_dh_table(dh_table, angles2)
    pos2 = extract_position(T2)
    expected2 = np.array([0.0, 1.0, -1.5])
    ok2 = np.allclose(pos2, expected2, atol=1e-10)
    status2 = "PASS" if ok2 else "FAIL"
    print(f"  [{status2}] θ1=π/2: pos={pos2} (expected {expected2})")

    all_passed = ok and ok2

    print("=" * 60)
    if all_passed:
        print("3D FK tests passed!")
    else:
        print("Some tests FAILED. This is expected — 3D is tricky!")
        print("Tip: Work through the DH chain on paper for the θ1=π/2 case.")
    print()
    return all_passed


if __name__ == "__main__":
    dh_ok = verify_dh_transform()
    chain_ok = verify_fk_chain()
    if dh_ok and chain_ok:
        verify_3link_3d()
