"""Module 2, Step 5: Spherical Wrist — Full 6-DOF Arm.

Reference: theory.md — Sections 2.4, 2.7

Goal:
    Model a 6-DOF arm (3-link arm + 3-joint spherical wrist) in both
    DH and MJCF, then verify that DH FK matches MuJoCo.

What you will learn:
    - How a spherical wrist gives 3 DOF for orientation
    - Kinematic decoupling: position (joints 1-3) vs orientation (joints 4-6)
    - DH parameters for zero-length links (all axes at one point)
    - Extracting BOTH position and orientation from FK
    - Verifying full 6-DOF FK against MuJoCo

Prerequisites:
    - Complete spherical_wrist.xml (fill in the TODO sections)
    - Working dh_transform and fk_from_dh_table from step_02
"""

import sys
from pathlib import Path

import numpy as np
import mujoco

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from modules.module_02_3d_dh.step_01_transforms import (
    extract_position, extract_rotation, make_transform
)
from modules.module_02_3d_dh.step_02_dh_params import dh_transform, fk_from_dh_table
from utils.mujoco_renderer import MuJoCoRenderer, load_model_and_data

MODEL_XML = Path(__file__).parent / "models" / "spherical_wrist.xml"


# ---------------------------------------------------------------------------
# YOUR TASK 1: Define the DH table for the full 6-DOF arm
# ---------------------------------------------------------------------------

def get_6dof_dh_table() -> tuple[list[dict], np.ndarray]:
    """Return the DH table for the 3-link arm + spherical wrist.

    The full arm has 6 joints:
        Joints 1-3: Arm (positioning) — same as step_04
        Joints 4-6: Spherical wrist (orientation)

    Wrist DH parameters (zero link lengths, alternating twists):
        Joint 4: θ=θ4, d=L3=0.8, a=0, α=-π/2  (wrist roll)
        Joint 5: θ=θ5, d=0,      a=0, α=π/2   (wrist pitch)
        Joint 6: θ=θ6, d=0.1,    a=0, α=0     (tool roll, d=tool offset)

    Note: The exact DH depends on frame assignment. You may need to
    adjust offsets to match the MJCF model. The critical test is whether
    DH FK matches MuJoCo's site_xpos.

    Returns:
        Tuple of:
            dh_table: List of 6 DH parameter dicts
            base_offset: (3,) array — world offset to DH frame 0
    """
    # === TODO START ===
    # Define the DH table for all 6 joints.
    # This is the hardest part of the module — getting the DH parameters
    # to match the MJCF model requires careful frame assignment.
    #
    # Start with the arm parameters from step_04, then add the wrist.
    # Remember:
    #   - α (twist) rotates the z-axis for the next frame
    #   - d offsets along z
    #   - a offsets along x
    #   - For the wrist: a=0 for all three joints (axes intersect)
    raise NotImplementedError("Implement get_6dof_dh_table")
    # === TODO END ===


# ---------------------------------------------------------------------------
# YOUR TASK 2: Verify position AND orientation against MuJoCo
# ---------------------------------------------------------------------------

def verify_6dof_fk(joint_angles_deg: np.ndarray) -> dict:
    """Compare DH FK (position + orientation) with MuJoCo.

    Args:
        joint_angles_deg: (6,) array of joint angles in degrees.

    Returns:
        Dict with:
            'dh_pos': (3,) DH end-effector position
            'mujoco_pos': (3,) MuJoCo site position
            'pos_error': position error (norm)
            'dh_rot': (3,3) DH rotation matrix
            'mujoco_rot': (3,3) MuJoCo site rotation
            'rot_error': rotation error (Frobenius norm of difference)
            'match': True if both errors are small

    Steps:
        1. Load model, set qpos, run mj_forward
        2. Read site_xpos AND site_xmat (orientation as 3x3 matrix)
        3. Compute DH FK, extract position + rotation
        4. Compare both
    """
    # === TODO START ===
    # 1. model, data = load_model_and_data(str(MODEL_XML))
    #    angles_rad = np.radians(joint_angles_deg)
    #    data.qpos[:6] = angles_rad
    #    mujoco.mj_forward(model, data)
    #
    # 2. site_id = model.site('end_effector').id
    #    mujoco_pos = data.site_xpos[site_id].copy()
    #    mujoco_rot = data.site_xmat[site_id].reshape(3, 3).copy()
    #
    # 3. dh_table, base_offset = get_6dof_dh_table()
    #    T = fk_from_dh_table(dh_table, angles_rad)
    #    dh_pos = extract_position(T) + base_offset
    #    dh_rot = extract_rotation(T)
    #
    # 4. pos_error = np.linalg.norm(dh_pos - mujoco_pos)
    #    rot_error = np.linalg.norm(dh_rot - mujoco_rot)
    raise NotImplementedError("Implement verify_6dof_fk")
    # === TODO END ===

    return {
        "dh_pos": dh_pos,
        "mujoco_pos": mujoco_pos,
        "pos_error": pos_error,
        "dh_rot": dh_rot,
        "mujoco_rot": mujoco_rot,
        "rot_error": rot_error,
        "match": pos_error < 1e-3 and rot_error < 1e-2,
    }


# ---------------------------------------------------------------------------
# VERIFICATION AND VISUALIZATION (do not modify)
# ---------------------------------------------------------------------------

def run_verification():
    """Test 6-DOF FK vs MuJoCo at multiple configurations."""
    test_configs = [
        np.array([0, 0, 0, 0, 0, 0]),
        np.array([45, 0, 0, 0, 0, 0]),
        np.array([0, 30, -45, 0, 0, 0]),
        np.array([0, 0, 0, 45, 0, 0]),
        np.array([0, 0, 0, 0, 45, 0]),
        np.array([0, 0, 0, 0, 0, 90]),
        np.array([30, 45, -30, 20, -40, 60]),
        np.array([-60, 20, 50, -90, 30, -45]),
    ]

    print("=" * 80)
    print("Verifying 6-DOF Arm (3-link + Spherical Wrist): DH FK vs MuJoCo")
    print("=" * 80)

    all_passed = True
    for angles in test_configs:
        result = verify_6dof_fk(angles)
        status = "PASS" if result["match"] else "FAIL"
        if not result["match"]:
            all_passed = False
        angle_str = ", ".join(f"{a:+6.1f}°" for a in angles)
        print(f"  [{status}] ({angle_str}) "
              f"| pos_err={result['pos_error']:.2e} "
              f"| rot_err={result['rot_error']:.2e}")

    print("=" * 80)
    if all_passed:
        print("Full 6-DOF FK matches MuJoCo!")
        print("You now have a complete arm with position AND orientation control.")
    else:
        print("Some mismatches. Debug your DH table frame assignments.")
    return all_passed


def render_wrist_demo():
    """Demonstrate the wrist's orientation capability."""
    import matplotlib.pyplot as plt

    model, data = load_model_and_data(str(MODEL_XML))
    renderer = MuJoCoRenderer(model, data, width=640, height=480)

    # Same arm position, different wrist orientations
    arm_angles = [0, 45, -30]  # degrees
    wrist_configs = [
        ([0, 0, 0], "Neutral wrist"),
        ([90, 0, 0], "Wrist roll 90°"),
        ([0, 45, 0], "Wrist pitch 45°"),
        ([0, 0, 90], "Tool roll 90°"),
        ([45, 30, -45], "Combined wrist"),
    ]

    fig, axes = plt.subplots(1, len(wrist_configs), figsize=(4 * len(wrist_configs), 4))

    for i, (wrist_angles, title) in enumerate(wrist_configs):
        all_angles = np.radians(arm_angles + wrist_angles)
        data.qpos[:6] = all_angles
        mujoco.mj_forward(model, data)

        frame = renderer.render_frame()
        axes[i].imshow(frame)
        axes[i].set_title(title, fontsize=9)
        axes[i].axis("off")

    plt.suptitle("Spherical Wrist: Same Arm Position, Different Orientations", fontsize=12)
    plt.tight_layout()

    save_path = Path(__file__).parent / "step_05_output.png"
    plt.savefig(str(save_path), dpi=100)
    print(f"Saved wrist demo to {save_path.name}")
    plt.show()

    renderer.close()


def demonstrate_decoupling():
    """Show kinematic decoupling: arm controls position, wrist controls orientation."""
    print("\n" + "=" * 60)
    print("Kinematic Decoupling Demonstration")
    print("=" * 60)

    model, data = load_model_and_data(str(MODEL_XML))

    # Two configurations with SAME arm angles but DIFFERENT wrist angles
    arm = np.radians([30, 45, -20])

    configs = [
        np.concatenate([arm, np.radians([0, 0, 0])]),
        np.concatenate([arm, np.radians([90, 45, -60])]),
    ]

    print("\n  Same arm position, different orientations:")
    for i, qpos in enumerate(configs):
        data.qpos[:6] = qpos
        mujoco.mj_forward(model, data)
        site_id = model.site("end_effector").id
        pos = data.site_xpos[site_id]
        rot = data.site_xmat[site_id].reshape(3, 3)
        print(f"    Config {i + 1}: pos=[{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
        print(f"              rot z-axis=[{rot[0, 2]:.4f}, {rot[1, 2]:.4f}, {rot[2, 2]:.4f}]")

    pos_diff = np.linalg.norm(
        data.site_xpos[model.site("end_effector").id] -
        (lambda: (data.qpos.__setitem__(slice(6), configs[0]),
                  mujoco.mj_forward(model, data),
                  data.site_xpos[model.site("end_effector").id].copy())[-1])()
    )
    print(f"\n  Position difference: {pos_diff:.6f}")
    print("  (Should be near-zero: wrist doesn't change position much)")
    print("=" * 60)


if __name__ == "__main__":
    passed = run_verification()
    if passed:
        render_wrist_demo()
