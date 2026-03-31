"""Module 1, Step 3: Forward Kinematics of a 2-Link Planar Arm.

Reference: theory.md — Sections 1.3, 1.8

Goal:
    Implement FK for a 2-link arm. Given joint angles (θ1, θ2) and link
    lengths (L1, L2), compute the elbow position and end-effector position.

What you will learn:
    - Cumulative angles in a kinematic chain
    - How each joint rotates everything downstream
    - The annular workspace of a 2-link arm
"""

import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from utils.viz import plot_arm_2d, plot_workspace_2d


# ---------------------------------------------------------------------------
# YOUR TASK: Implement the 2-link FK function
# ---------------------------------------------------------------------------

def forward_kinematics_2link(
    theta1: float,
    theta2: float,
    L1: float,
    L2: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute the joint and end-effector positions of a 2-link planar arm.

    The base is fixed at the origin (0, 0).
    theta1 is measured from the +x axis (absolute).
    theta2 is measured relative to link 1.

    Args:
        theta1: First joint angle in radians.
        theta2: Second joint angle in radians (relative to link 1).
        L1: Length of link 1.
        L2: Length of link 2.

    Returns:
        Tuple of:
            elbow_pos: (2,) ndarray — position of joint 1 (elbow).
            ee_pos:    (2,) ndarray — position of the end-effector.

    Hint:
        See theory.md Section 1.3:
        - Elbow:  (L1·cos(θ1), L1·sin(θ1))
        - EE:     elbow + (L2·cos(θ1+θ2), L2·sin(θ1+θ2))
        Remember: θ2 is RELATIVE, so the absolute angle of link 2 is θ1+θ2.
    """
    # === TODO START ===
    # Step 1: Compute elbow position (x1, y1)
    #   x1 = L1 * cos(theta1)
    #   y1 = L1 * sin(theta1)
    #
    # Step 2: Compute end-effector position (x2, y2)
    #   The absolute angle of link 2 is theta1 + theta2
    #   x2 = x1 + L2 * cos(theta1 + theta2)
    #   y2 = y1 + L2 * sin(theta1 + theta2)
    #
    # Step 3: Return as numpy arrays
    #   elbow_pos = np.array([x1, y1])
    #   ee_pos = np.array([x2, y2])
    raise NotImplementedError("Implement forward_kinematics_2link")
    # === TODO END ===


# ---------------------------------------------------------------------------
# VERIFICATION (do not modify)
# ---------------------------------------------------------------------------

def verify_fk():
    """Run test cases for the 2-link FK."""
    L1, L2 = 1.0, 1.0

    test_cases = [
        # (theta1, theta2, expected_elbow, expected_ee, description)
        (0.0, 0.0,
         np.array([1.0, 0.0]), np.array([2.0, 0.0]),
         "Both joints at 0: fully extended along +x"),

        (np.pi / 2, 0.0,
         np.array([0.0, 1.0]), np.array([0.0, 2.0]),
         "θ1=π/2, θ2=0: extended along +y"),

        (0.0, np.pi / 2,
         np.array([1.0, 0.0]), np.array([1.0, 1.0]),
         "θ1=0, θ2=π/2: link2 bends 90° up"),

        (0.0, np.pi,
         np.array([1.0, 0.0]), np.array([0.0, 0.0]),
         "θ1=0, θ2=π: fully folded back to origin"),

        (np.pi / 4, np.pi / 4,
         np.array([np.sqrt(2) / 2, np.sqrt(2) / 2]),
         np.array([np.sqrt(2) / 2 + np.cos(np.pi / 2),
                    np.sqrt(2) / 2 + np.sin(np.pi / 2)]),
         "θ1=π/4, θ2=π/4: both 45° relative"),
    ]

    print("=" * 60)
    print("Verifying 2-link FK")
    print("=" * 60)

    all_passed = True
    for theta1, theta2, exp_elbow, exp_ee, desc in test_cases:
        elbow, ee = forward_kinematics_2link(theta1, theta2, L1, L2)
        ok_elbow = np.allclose(elbow, exp_elbow, atol=1e-10)
        ok_ee = np.allclose(ee, exp_ee, atol=1e-10)
        status = "PASS" if (ok_elbow and ok_ee) else "FAIL"
        if status == "FAIL":
            all_passed = False
        print(f"  [{status}] {desc}")
        if status == "FAIL":
            print(f"         Elbow:  got {elbow}, expected {exp_elbow}")
            print(f"         EE:     got {ee}, expected {exp_ee}")

    print("=" * 60)
    if all_passed:
        print("All 2-link FK tests passed!")
    else:
        print("Some tests FAILED.")
    print()
    return all_passed


def visualize_2link():
    """Visualize the 2-link arm at several configurations."""
    L1, L2 = 1.0, 0.8
    configs = [
        (0.0, 0.0, "Extended"),
        (np.pi / 4, 0.0, "θ1=45°"),
        (np.pi / 4, np.pi / 4, "Both 45°"),
        (np.pi / 3, -np.pi / 3, "Elbow down"),
        (np.pi / 2, np.pi / 2, "L-shape"),
    ]

    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    ax = axes[0]
    for theta1, theta2, label in configs:
        elbow, ee = forward_kinematics_2link(theta1, theta2, L1, L2)
        base = np.array([0.0, 0.0])
        positions = np.stack([base, elbow, ee])
        plot_arm_2d(positions, ax=ax, show=False, title="2-Link Arm Configurations")

    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-2.5, 2.5)

    plot_workspace_2d([L1, L2], ax=axes[1], show=False)

    plt.tight_layout()
    plt.savefig(str(PROJECT_ROOT / "modules" / "module_01_2d_basics" / "step_03_output.png"), dpi=100)
    print("Saved visualization to step_03_output.png")
    plt.show()


if __name__ == "__main__":
    passed = verify_fk()
    if passed:
        visualize_2link()
