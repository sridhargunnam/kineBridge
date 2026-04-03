"""Module 1, Step 4: Analytic Inverse Kinematics for a 2-Link Planar Arm.

Reference: theory.md — Sections 1.4, 1.8

Goal:
    Implement the closed-form IK solution for a 2-link arm using the
    law of cosines and atan2. Handle both elbow-up and elbow-down solutions.

What you will learn:
    - IK as the inverse of FK: task space → joint space
    - The law of cosines applied to robot kinematics
    - Why IK has multiple solutions (elbow-up vs elbow-down)
    - Reachability checking (workspace boundaries)
"""

import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from modules.module_01_2d_basics.step_03_two_link_fk import forward_kinematics_2link
from utils.viz import plot_arm_2d


# ---------------------------------------------------------------------------
# YOUR TASK: Implement the analytic IK function
# ---------------------------------------------------------------------------

def inverse_kinematics_2link(
    x: float,
    y: float,
    L1: float,
    L2: float,
) -> list[tuple[float, float]]:
    """Compute the joint angles for a 2-link arm to reach target (x, y).

    Uses the law of cosines for θ2 and atan2 for θ1.
    Returns up to two solutions (elbow-up and elbow-down).

    Args:
        x: Target x-coordinate.
        y: Target y-coordinate.
        L1: Length of link 1.
        L2: Length of link 2.

    Returns:
        List of (θ1, θ2) tuples. May contain:
            - 2 solutions (elbow-up and elbow-down) for most reachable targets
            - 1 solution when at the workspace boundary
            - 0 solutions (empty list) when target is unreachable

    Hint:
        See theory.md Section 1.4:

        Step 1 — Reachability:
            Check |L1 - L2| ≤ sqrt(x² + y²) ≤ L1 + L2

        Step 2 — Solve for θ2:
            cos_θ2 = (x² + y² - L1² - L2²) / (2·L1·L2)
            Clamp cos_θ2 to [-1, 1] to handle floating-point edge cases.
            θ2 = ±arccos(cos_θ2)

        Step 3 — Solve for θ1 (for each θ2):
            k1 = L1 + L2·cos(θ2)
            k2 = L2·sin(θ2)
            θ1 = atan2(y, x) - atan2(k2, k1)
    """
    # === TODO START ===
    # Step 1: Compute distance to target and check reachability
    #   d_sq = x**2 + y**2
    #   d = sqrt(d_sq)
    #   if d > L1 + L2 or d < abs(L1 - L2): return []
    #
    # Step 2: Solve for θ2 using law of cosines
    #   cos_theta2 = (d_sq - L1**2 - L2**2) / (2 * L1 * L2)
    #   Clamp to [-1, 1]
    #   theta2_options = [+arccos(cos_theta2), -arccos(cos_theta2)]
    #
    # Step 3: For each θ2, solve for θ1
    #   solutions = []
    #   for theta2 in theta2_options:
    #       k1 = L1 + L2 * cos(theta2)
    #       k2 = L2 * sin(theta2)
    #       theta1 = atan2(y, x) - atan2(k2, k1)
    #       solutions.append((theta1, theta2))
    #
    # Step 4: Remove duplicates (when at workspace boundary)
    #   Return unique solutions.
    # raise NotImplementedError("Implement inverse_kinematics_2link")
    d_sq = x**2 + y**2
    d = np.sqrt(d_sq)
    if d > L1 + L2 or d < abs(L1 - L2): return []
    cos_theta2 = (d_sq - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1, 1)
    theta2_options = [np.arccos(cos_theta2), -np.arccos(cos_theta2)]
    solutions = []
    for theta2 in theta2_options:
        k1 = L1 + L2 * np.cos(theta2)
        k2 = L2 * np.sin(theta2)
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        solutions.append((theta1, theta2))
    return solutions
    # === TODO END ===


# ---------------------------------------------------------------------------
# VERIFICATION (do not modify)
# ---------------------------------------------------------------------------

def verify_ik():
    """Test IK by computing angles and checking FK roundtrip."""
    L1, L2 = 1.0, 1.0

    test_targets = [
        (1.5, 0.5, "General point"),
        (1.0, 1.0, "Diagonal"),
        (2.0, 0.0, "Fully extended along +x"),
        (0.0, 2.0, "Fully extended along +y"),
        (0.5, 0.0, "Close to base"),
        (0.0, 0.0, "At origin (folded)"),
    ]

    unreachable_targets = [
        (3.0, 0.0, "Beyond outer radius"),
        (0.0, 3.0, "Way too far"),
    ]

    print("=" * 60)
    print("Verifying 2-link Analytic IK")
    print("=" * 60)

    all_passed = True
    for x, y, desc in test_targets:
        solutions = inverse_kinematics_2link(x, y, L1, L2)
        if not solutions:
            print(f"  [FAIL] {desc}: No solution found for ({x}, {y})")
            all_passed = False
            continue

        for i, (theta1, theta2) in enumerate(solutions):
            _, ee = forward_kinematics_2link(theta1, theta2, L1, L2)
            error = np.sqrt((ee[0] - x) ** 2 + (ee[1] - y) ** 2)
            ok = error < 1e-6
            status = "PASS" if ok else "FAIL"
            if not ok:
                all_passed = False
            config = "elbow-up" if i == 0 else "elbow-down"
            print(f"  [{status}] {desc} ({config}): "
                  f"θ1={np.degrees(theta1):+7.2f}°, θ2={np.degrees(theta2):+7.2f}° "
                  f"| FK error={error:.2e}")

    print("\n  Unreachable targets:")
    for x, y, desc in unreachable_targets:
        solutions = inverse_kinematics_2link(x, y, L1, L2)
        status = "PASS" if len(solutions) == 0 else "FAIL"
        if status == "FAIL":
            all_passed = False
        print(f"  [{status}] {desc}: "
              f"{'correctly rejected' if len(solutions) == 0 else 'ERROR: should be unreachable'}")

    print("=" * 60)
    if all_passed:
        print("All IK tests passed!")
    else:
        print("Some tests FAILED.")
    print()
    return all_passed


def visualize_ik():
    """Show both IK solutions for a target point."""
    L1, L2 = 1.0, 0.8
    target = np.array([1.2, 0.6])

    solutions = inverse_kinematics_2link(target[0], target[1], L1, L2)

    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(solutions), figsize=(6 * len(solutions), 6))
    if len(solutions) == 1:
        axes = [axes]

    labels = ["Elbow-up", "Elbow-down"]
    for i, (theta1, theta2) in enumerate(solutions):
        elbow, ee = forward_kinematics_2link(theta1, theta2, L1, L2)
        base = np.array([0.0, 0.0])
        positions = np.stack([base, elbow, ee])
        plot_arm_2d(
            positions, ax=axes[i], target=target, show=False,
            title=f"{labels[i]}: θ1={np.degrees(theta1):.1f}°, θ2={np.degrees(theta2):.1f}°"
        )
        axes[i].set_xlim(-2, 2)
        axes[i].set_ylim(-2, 2)

    plt.tight_layout()
    plt.savefig(str(PROJECT_ROOT / "modules" / "module_01_2d_basics" / "step_04_output.png"), dpi=100)
    print("Saved visualization to step_04_output.png")
    plt.show()


if __name__ == "__main__":
    passed = verify_ik()
    if passed:
        visualize_ik()
