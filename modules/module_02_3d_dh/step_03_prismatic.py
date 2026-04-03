"""Module 2, Step 3: Forward Kinematics with Prismatic (Slide) Joints.

Reference: theory.md — Sections 2.3, 2.4

Goal:
    Extend your DH-based FK to handle prismatic joints, where the joint
    variable controls translation (d) instead of rotation (θ).

What you will learn:
    - How prismatic joints differ from revolute in the DH framework
    - Building a SCARA-like arm (Revolute-Revolute-Prismatic: RRP)
    - Verifying that your fk_from_dh_table handles both joint types
    - The workspace shape of arms with prismatic joints (cylinder vs sphere)
"""

import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from modules.module_02_3d_dh.step_01_transforms import extract_position, extract_rotation
from modules.module_02_3d_dh.step_02_dh_params import dh_transform, fk_from_dh_table


# ---------------------------------------------------------------------------
# YOUR TASK: Define DH tables for arms with prismatic joints
# ---------------------------------------------------------------------------

def get_rrp_scara_dh_table() -> list[dict]:
    """Return the DH table for an RRP (SCARA-like) arm.

    This arm has:
        Joint 1: Revolute about z, link length a1 = 1.0
        Joint 2: Revolute about z, link length a2 = 0.8
        Joint 3: Prismatic along z (vertical slide), no link length

    The first two joints create a planar arm in the xy-plane.
    The third joint is a vertical slider that moves the end-effector up/down.

    Returns:
        List of DH parameter dicts.

    Hint:
        For a prismatic joint:
        - 'joint_type' is 'prismatic'
        - 'd' is the base offset (joint_angles[i] gets ADDED to d)
        - θ stays at its offset value (usually 0)
        - α determines axis orientation for the next frame

        For this SCARA:
        Link 1: θ=θ1, d=0,  a=1.0, α=0     (revolute)
        Link 2: θ=θ2, d=0,  a=0.8, α=0     (revolute)
        Link 3: θ=0,  d=d3, a=0,   α=0     (prismatic)
    """
    # === TODO START ===
    # Return a list of 3 dicts, one per joint.
    # Each dict has keys: 'theta_offset', 'd', 'a', 'alpha', 'joint_type'
    raise NotImplementedError("Implement get_rrp_scara_dh_table")
    # === TODO END ===


def get_rpr_arm_dh_table() -> list[dict]:
    """Return the DH table for an RPR arm.

    This arm has:
        Joint 1: Revolute about z, no link length
        Joint 2: Prismatic along z, with a 90° twist
        Joint 3: Revolute about z, link length a3 = 0.5

    The prismatic joint in the middle creates a telescoping arm segment.

    Returns:
        List of DH parameter dicts.

    Hint:
        Link 1: θ=θ1, d=0,  a=0,   α=π/2   (revolute, twists next frame)
        Link 2: θ=0,  d=d2, a=0,   α=-π/2  (prismatic, twists back)
        Link 3: θ=θ3, d=0,  a=0.5, α=0     (revolute)
    """
    # === TODO START ===
    raise NotImplementedError("Implement get_rpr_arm_dh_table")
    # === TODO END ===


# ---------------------------------------------------------------------------
# VERIFICATION (do not modify)
# ---------------------------------------------------------------------------

def verify_rrp():
    """Test the RRP SCARA arm."""
    print("=" * 60)
    print("Verifying RRP (SCARA-like) Arm")
    print("=" * 60)

    dh_table = get_rrp_scara_dh_table()
    all_passed = True

    test_cases = [
        # (joint_values, expected_position, description)
        (np.array([0.0, 0.0, 0.0]),
         np.array([1.8, 0.0, 0.0]),
         "All zeros: extended along +x"),

        (np.array([0.0, 0.0, 0.5]),
         np.array([1.8, 0.0, 0.5]),
         "Slide up 0.5: same xy, z=0.5"),

        (np.array([np.pi / 2, 0.0, 0.0]),
         np.array([0.0, 1.8, 0.0]),
         "θ1=π/2: extended along +y"),

        (np.array([0.0, np.pi / 2, 0.0]),
         np.array([1.0, 0.8, 0.0]),
         "θ2=π/2: second link points +y"),

        (np.array([np.pi / 2, -np.pi / 2, 1.0]),
         np.array([0.8, 1.0, 1.0]),
         "θ1=π/2, θ2=-π/2, d3=1.0"),
    ]

    for angles, expected, desc in test_cases:
        T = fk_from_dh_table(dh_table, angles)
        pos = extract_position(T)
        ok = np.allclose(pos, expected, atol=1e-6)
        status = "PASS" if ok else "FAIL"
        if not ok:
            all_passed = False
        print(f"  [{status}] {desc}")
        if not ok:
            print(f"         Got: {pos}, expected: {expected}")

    print("=" * 60)
    if all_passed:
        print("RRP tests passed!")
    else:
        print("Some tests FAILED.")
    print()
    return all_passed


def verify_rpr():
    """Test the RPR arm."""
    print("=" * 60)
    print("Verifying RPR Arm")
    print("=" * 60)

    dh_table = get_rpr_arm_dh_table()
    all_passed = True

    # All zeros: link3 extends along x = [0.5, 0, 0]
    angles = np.array([0.0, 0.0, 0.0])
    T = fk_from_dh_table(dh_table, angles)
    pos = extract_position(T)
    expected = np.array([0.5, 0.0, 0.0])
    ok = np.allclose(pos, expected, atol=1e-6)
    status = "PASS" if ok else "FAIL"
    if not ok:
        all_passed = False
    print(f"  [{status}] All zeros: pos={pos}")

    # Extend prismatic joint by 1.0
    angles2 = np.array([0.0, 1.0, 0.0])
    T2 = fk_from_dh_table(dh_table, angles2)
    pos2 = extract_position(T2)
    # d2=1.0 translates along z of link1 frame (which after α=π/2 twist
    # means it moves along the original y-axis... but with the twist back,
    # link3's x-axis points along the original x-axis again)
    print(f"  [INFO] d2=1.0: pos={pos2}")

    print("=" * 60)
    if all_passed:
        print("RPR basic test passed!")
    else:
        print("Some tests FAILED.")
    print()
    return all_passed


def visualize_rrp_workspace():
    """Visualize the 3D workspace of the RRP arm."""
    import matplotlib.pyplot as plt

    dh_table = get_rrp_scara_dh_table()

    n_samples = 3000
    theta1 = np.random.uniform(-np.pi, np.pi, n_samples)
    theta2 = np.random.uniform(-np.pi, np.pi, n_samples)
    d3 = np.random.uniform(0, 1.0, n_samples)

    positions = []
    for i in range(n_samples):
        angles = np.array([theta1[i], theta2[i], d3[i]])
        T = fk_from_dh_table(dh_table, angles)
        positions.append(extract_position(T))

    positions = np.array(positions)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
               s=1, alpha=0.3, c=positions[:, 2], cmap="viridis")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("RRP (SCARA) Workspace — cylindrical shape")

    save_path = Path(__file__).parent / "step_03_output.png"
    plt.tight_layout()
    plt.savefig(str(save_path), dpi=100)
    print(f"Saved workspace visualization to {save_path.name}")
    plt.show()


if __name__ == "__main__":
    rrp_ok = verify_rrp()
    rpr_ok = verify_rpr()
    if rrp_ok:
        visualize_rrp_workspace()
