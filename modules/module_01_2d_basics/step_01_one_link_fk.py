"""Module 1, Step 1: Forward Kinematics of a 1-Link Planar Arm.

Reference: theory.md — Sections 1.1, 1.2, 1.8

Goal:
    Implement the FK function for a single-link arm using basic trigonometry.
    Given a joint angle θ and link length L, compute the end-effector (x, y).

What you will learn:
    - The relationship between joint angle and Cartesian position
    - The workspace of a 1-link arm (a circle)
    - How to verify your math with numpy
"""

import sys
from pathlib import Path

import numpy as np

# Add project root to path so we can import utils
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from utils.viz import plot_arm_2d, plot_workspace_2d


# ---------------------------------------------------------------------------
# YOUR TASK: Implement the FK function
# ---------------------------------------------------------------------------

def forward_kinematics_1link(theta: float, L: float) -> tuple[float, float]:
    """Compute the end-effector position of a 1-link planar arm.

    The arm is attached at the origin (0, 0) with a revolute joint.
    The joint angle theta is measured counter-clockwise from the +x axis.

    Args:
        theta: Joint angle in radians.
        L: Link length (positive scalar).

    Returns:
        Tuple (x, y) — the end-effector position in the world frame.

    Hint:
        See theory.md Section 1.2. This is a direct application of the
        unit circle: a point on a circle of radius L at angle theta.
    """
    # === TODO START ===
    x = L * np.cos(theta)
    y = L * np.sin(theta)
    return x, y
    # === TODO END ===


# ---------------------------------------------------------------------------
# VERIFICATION (do not modify)
# ---------------------------------------------------------------------------

def verify_fk():
    """Run test cases to check your FK implementation."""
    L = 1.0

    test_cases = [
        # (theta,    expected_x,    expected_y,    description)
        (0.0,        1.0,           0.0,           "θ=0: pointing along +x"),
        (np.pi / 2,  0.0,           1.0,           "θ=π/2: pointing along +y"),
        (np.pi,      -1.0,          0.0,           "θ=π: pointing along -x"),
        (-np.pi / 2, 0.0,          -1.0,           "θ=-π/2: pointing along -y"),
        (np.pi / 4,  np.sqrt(2)/2,  np.sqrt(2)/2,  "θ=π/4: 45 degrees"),
    ]

    print("=" * 60)
    print("Verifying 1-link FK")
    print("=" * 60)

    all_passed = True
    for theta, exp_x, exp_y, desc in test_cases:
        x, y = forward_kinematics_1link(theta, L)
        ok_x = np.isclose(x, exp_x, atol=1e-10)
        ok_y = np.isclose(y, exp_y, atol=1e-10)
        status = "PASS" if (ok_x and ok_y) else "FAIL"
        if status == "FAIL":
            all_passed = False
        print(f"  [{status}] {desc}")
        if status == "FAIL":
            print(f"         Got:      ({x:.6f}, {y:.6f})")
            print(f"         Expected: ({exp_x:.6f}, {exp_y:.6f})")

    print("=" * 60)
    if all_passed:
        print("All tests passed!")
    else:
        print("Some tests FAILED. Review your implementation.")
    print()
    return all_passed


def visualize_fk():
    """Visualize the arm at several angles and show the workspace."""
    L = 1.5
    angles = [0, np.pi / 6, np.pi / 3, np.pi / 2, 2 * np.pi / 3, np.pi]

    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    ax = axes[0]
    for theta in angles:
        x, y = forward_kinematics_1link(theta, L)
        base = np.array([0.0, 0.0])
        ee = np.array([x, y])
        positions = np.stack([base, ee])
        plot_arm_2d(positions, ax=ax, show=False, title=f"1-Link Arm (L={L})")

    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)

    plot_workspace_2d([L], ax=axes[1], show=False)

    plt.tight_layout()
    plt.savefig(str(PROJECT_ROOT / "modules" / "module_01_2d_basics" / "step_01_output.png"), dpi=100)
    print("Saved visualization to step_01_output.png")
    plt.show()


if __name__ == "__main__":
    passed = verify_fk()
    if passed:
        visualize_fk()
