"""Module 1, Step 5: Full 2-Link Integration — IK + MuJoCo Verification.

Reference: theory.md — Sections 1.4, 1.6, 1.7

Goal:
    Bring everything together:
    1. Pick a target point in 2D
    2. Run your IK solver to get joint angles
    3. Set those angles in MuJoCo
    4. Verify the end-effector lands on the target
    5. Render the result

What you will learn:
    - The full IK → simulation → verification pipeline
    - Visual confirmation that your math, XML, and code are all consistent
    - Animating the arm moving to a target
"""

import sys
from pathlib import Path

import numpy as np
import mujoco

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from modules.module_01_2d_basics.step_03_two_link_fk import forward_kinematics_2link
from modules.module_01_2d_basics.step_04_two_link_ik import inverse_kinematics_2link
from utils.mujoco_renderer import MuJoCoRenderer, load_model_and_data
from utils.viz import plot_arm_2d


MODEL_XML = Path(__file__).parent / "models" / "two_link_arm.xml"

L1 = 1.0
L2 = 0.8


# ---------------------------------------------------------------------------
# YOUR TASK: Implement the full IK → MuJoCo verification pipeline
# ---------------------------------------------------------------------------

def ik_to_mujoco(target_x: float, target_y: float) -> dict:
    """Solve IK for a target, apply to MuJoCo, and verify.

    Steps (implement the TODO sections):
        1. Call your IK solver to get joint angles for the target
        2. Load the MuJoCo model
        3. For each IK solution:
           a. Set data.qpos to [theta1, theta2]
           b. Call mj_forward
           c. Read the end-effector site position
           d. Compare against the target
        4. Return the results

    Args:
        target_x: Target x-coordinate.
        target_y: Target y-coordinate.

    Returns:
        Dict with keys:
            'target': (x, y) tuple
            'solutions': list of dicts, each containing:
                'angles_deg': (θ1°, θ2°)
                'mujoco_ee': (x, y, z) from MuJoCo
                'numpy_ee': (x, y) from FK
                'error': distance between target and mujoco EE
    """
    # === TODO START ===
    # 1. Call your IK solver
    #    solutions = inverse_kinematics_2link(target_x, target_y, L1, L2)
    #    Handle the case where solutions is empty (unreachable target).
    #
    # 2. Load MuJoCo model
    #    model, data = load_model_and_data(str(MODEL_XML))
    #
    # 3. For each (theta1, theta2) solution:
    #    a. data.qpos[0] = theta1
    #       data.qpos[1] = theta2
    #    b. mujoco.mj_forward(model, data)
    #    c. mujoco_ee = data.site_xpos[model.site('end_effector').id].copy()
    #    d. _, numpy_ee = forward_kinematics_2link(theta1, theta2, L1, L2)
    #    e. error = np.linalg.norm(mujoco_ee[:2] - [target_x, target_y])
    #    f. Collect results
    #
    # 4. Return results dict
    raise NotImplementedError("Implement ik_to_mujoco")
    # === TODO END ===


# ---------------------------------------------------------------------------
# VERIFICATION AND VISUALIZATION (do not modify)
# ---------------------------------------------------------------------------

def run_full_verification():
    """Test the complete IK → MuJoCo pipeline at multiple targets."""
    targets = [
        (1.2, 0.6),
        (0.5, 1.0),
        (-0.8, 0.5),
        (1.8, 0.0),
        (0.3, 0.3),
    ]

    print("=" * 70)
    print("Full IK → MuJoCo Verification Pipeline")
    print("=" * 70)

    all_passed = True
    for tx, ty in targets:
        result = ik_to_mujoco(tx, ty)
        print(f"\n  Target: ({tx}, {ty})")
        if not result["solutions"]:
            print("    No solutions found (unreachable)")
            continue

        for i, sol in enumerate(result["solutions"]):
            config = "elbow-up" if i == 0 else "elbow-down"
            ok = sol["error"] < 1e-4
            if not ok:
                all_passed = False
            status = "PASS" if ok else "FAIL"
            print(f"    [{status}] {config}: "
                  f"angles=({sol['angles_deg'][0]:+7.2f}°, {sol['angles_deg'][1]:+7.2f}°) "
                  f"| MuJoCo EE=({sol['mujoco_ee'][0]:+.4f}, {sol['mujoco_ee'][1]:+.4f}) "
                  f"| err={sol['error']:.2e}")

    print("\n" + "=" * 70)
    if all_passed:
        print("Full pipeline verified! Math, XML, and IK are all consistent.")
    else:
        print("Some checks FAILED.")
    return all_passed


def visualize_ik_solutions():
    """Render both IK solutions in MuJoCo for a target."""
    target = np.array([1.0, 0.8])
    result = ik_to_mujoco(target[0], target[1])

    if not result["solutions"]:
        print("Target unreachable.")
        return

    model, data = load_model_and_data(str(MODEL_XML))
    renderer = MuJoCoRenderer(model, data, width=640, height=480)

    import matplotlib.pyplot as plt
    n_sols = len(result["solutions"])
    fig, axes = plt.subplots(1, n_sols, figsize=(6 * n_sols, 5))
    if n_sols == 1:
        axes = [axes]

    labels = ["Elbow-up", "Elbow-down"]
    for i, sol in enumerate(result["solutions"]):
        theta1 = np.radians(sol["angles_deg"][0])
        theta2 = np.radians(sol["angles_deg"][1])
        data.qpos[0] = theta1
        data.qpos[1] = theta2
        mujoco.mj_forward(model, data)

        frame = renderer.render_frame()
        axes[i].imshow(frame)
        axes[i].set_title(f"{labels[i]}: θ1={sol['angles_deg'][0]:.1f}°, θ2={sol['angles_deg'][1]:.1f}°")
        axes[i].axis("off")

    plt.suptitle(f"Target: ({target[0]}, {target[1]})", fontsize=14)
    plt.tight_layout()
    plt.savefig(str(PROJECT_ROOT / "modules" / "module_01_2d_basics" / "step_05_output.png"), dpi=100)
    print("Saved visualization to step_05_output.png")
    plt.show()

    renderer.close()


def animate_to_target():
    """Animate the arm sweeping from rest to a target configuration."""
    target = np.array([0.8, 1.0])
    solutions = inverse_kinematics_2link(target[0], target[1], L1, L2)
    if not solutions:
        print("Target unreachable.")
        return

    theta1_goal, theta2_goal = solutions[0]

    n_frames = 60
    trajectory = []
    for t in np.linspace(0, 1, n_frames):
        theta1 = t * theta1_goal
        theta2 = t * theta2_goal
        elbow, ee = forward_kinematics_2link(theta1, theta2, L1, L2)
        base = np.array([0.0, 0.0])
        trajectory.append(np.stack([base, elbow, ee]))

    from utils.viz import animate_arm_2d
    save_path = str(PROJECT_ROOT / "modules" / "module_01_2d_basics" / "step_05_animation.gif")
    animate_arm_2d(trajectory, target=target, title="IK → Arm Animation", save_path=save_path)
    print(f"Saved animation to {save_path}")


if __name__ == "__main__":
    passed = run_full_verification()
    if passed:
        print("\nRendering IK solutions...")
        visualize_ik_solutions()
        print("\nAnimating arm to target...")
        animate_to_target()
