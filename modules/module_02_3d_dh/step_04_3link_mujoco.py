"""Module 2, Step 4: 3-Link 3D Arm in MuJoCo — Verify DH FK.

Reference: theory.md — Sections 2.6, 2.7

Goal:
    Load your three_link_3d_arm.xml, set joint angles, and verify that
    MuJoCo's end-effector position matches your DH-based FK.

What you will learn:
    - Building and loading a 3D MJCF model
    - Mapping DH parameters to MJCF body/joint structure
    - Verifying 3D FK against MuJoCo simulation
    - Rendering a 3D arm with the dual-mode renderer

Prerequisites:
    - Complete three_link_3d_arm.xml (fill in the TODO sections)
    - Working dh_transform and fk_from_dh_table from step_02
"""

import sys
from pathlib import Path

import numpy as np
import mujoco

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from modules.module_02_3d_dh.step_01_transforms import extract_position
from modules.module_02_3d_dh.step_02_dh_params import dh_transform, fk_from_dh_table
from utils.mujoco_renderer import MuJoCoRenderer, load_model_and_data

MODEL_XML = Path(__file__).parent / "models" / "three_link_3d_arm.xml"


# ---------------------------------------------------------------------------
# YOUR TASK: Define the DH table that matches your XML arm
# ---------------------------------------------------------------------------

def get_3link_3d_dh_table() -> tuple[list[dict], np.ndarray]:
    """Return the DH table and base offset for the 3-link 3D arm.

    The XML arm has:
        - Base at height 0.5 (pedestal)
        - Joint 1: revolute about z (base rotation)
        - Joint 2: revolute about y (shoulder pitch)  
        - Joint 3: revolute about y (elbow pitch)
        - Links extend along local z: L2=1.0, L3=0.8

    You need to figure out the correct DH parameters that produce the
    same kinematic behavior as the MJCF model.

    Returns:
        Tuple of:
            dh_table: List of DH parameter dicts
            base_offset: (3,) array — offset from world origin to DH frame 0

    Hint:
        The XML arm's links go along z, but standard DH uses x for link length.
        You'll need α (twist) parameters to align the frames.

        A common approach for this arm configuration:
        Link 1: θ=θ1, d=0,   a=0,   α=-π/2  (rotate about z, twist to align)
        Link 2: θ=θ2, d=0,   a=1.0, α=0     (shoulder pitch, arm extends)
        Link 3: θ=θ3, d=0,   a=0.8, α=0     (elbow pitch, forearm extends)

        But the exact DH depends on how you assign frames. The key test:
        does your DH FK match MuJoCo's site_xpos?

        base_offset accounts for the pedestal height: [0, 0, 0.5]
    """
    # === TODO START ===
    # Define the DH table (list of dicts) and base_offset
    raise NotImplementedError("Implement get_3link_3d_dh_table")
    # === TODO END ===


# ---------------------------------------------------------------------------
# YOUR TASK: Compare DH FK against MuJoCo
# ---------------------------------------------------------------------------

def verify_fk_against_mujoco(joint_angles_deg: np.ndarray) -> dict:
    """Compare DH-based FK with MuJoCo's computed end-effector position.

    Args:
        joint_angles_deg: (3,) array of joint angles in degrees.

    Returns:
        Dict with 'dh_pos', 'mujoco_pos', 'error', 'match' keys.

    Steps (implement the TODO sections):
        1. Load the MuJoCo model
        2. Set joint angles (convert degrees to radians for qpos)
        3. Run mj_forward
        4. Read end-effector site position
        5. Compute DH FK with the same angles
        6. Compare
    """
    # === TODO START ===
    # 1. model, data = load_model_and_data(str(MODEL_XML))
    #
    # 2. Convert degrees to radians:
    #    angles_rad = np.radians(joint_angles_deg)
    #    data.qpos[:3] = angles_rad
    #
    # 3. mujoco.mj_forward(model, data)
    #
    # 4. mujoco_pos = data.site_xpos[model.site('end_effector').id].copy()
    #
    # 5. Compute DH FK:
    #    dh_table, base_offset = get_3link_3d_dh_table()
    #    T = fk_from_dh_table(dh_table, angles_rad)
    #    dh_pos = extract_position(T) + base_offset
    #
    # 6. error = np.linalg.norm(dh_pos - mujoco_pos)
    raise NotImplementedError("Implement verify_fk_against_mujoco")
    # === TODO END ===

    return {
        "dh_pos": dh_pos,
        "mujoco_pos": mujoco_pos,
        "error": error,
        "match": error < 1e-3,
    }


# ---------------------------------------------------------------------------
# VERIFICATION AND VISUALIZATION (do not modify)
# ---------------------------------------------------------------------------

def run_verification():
    """Test DH FK vs MuJoCo at multiple configurations."""
    test_configs = [
        np.array([0.0, 0.0, 0.0]),
        np.array([45.0, 0.0, 0.0]),
        np.array([0.0, 45.0, 0.0]),
        np.array([0.0, 0.0, 45.0]),
        np.array([30.0, 45.0, -30.0]),
        np.array([90.0, 30.0, 60.0]),
        np.array([-45.0, 60.0, -90.0]),
    ]

    print("=" * 70)
    print("Verifying 3-Link 3D Arm: DH FK vs MuJoCo")
    print("=" * 70)

    all_passed = True
    for angles in test_configs:
        result = verify_fk_against_mujoco(angles)
        status = "PASS" if result["match"] else "FAIL"
        if not result["match"]:
            all_passed = False
        print(f"  [{status}] angles=({angles[0]:+6.1f}°, {angles[1]:+6.1f}°, {angles[2]:+6.1f}°) "
              f"| DH={result['dh_pos']} "
              f"| MuJoCo={result['mujoco_pos']} "
              f"| err={result['error']:.2e}")

    print("=" * 70)
    if all_passed:
        print("DH FK matches MuJoCo for all test configurations!")
    else:
        print("Some mismatches detected. Debug your DH table or XML.")
    return all_passed


def render_configurations():
    """Render the arm at several configurations."""
    import matplotlib.pyplot as plt

    configs = [
        (np.array([0, 0, 0]), "Home position"),
        (np.array([0, 45, -45]), "Reaching forward"),
        (np.array([90, 30, 60]), "Rotated + bent"),
    ]

    model, data = load_model_and_data(str(MODEL_XML))
    renderer = MuJoCoRenderer(model, data, width=640, height=480)

    fig, axes = plt.subplots(1, len(configs), figsize=(6 * len(configs), 5))
    if len(configs) == 1:
        axes = [axes]

    for i, (angles_deg, title) in enumerate(configs):
        data.qpos[:3] = np.radians(angles_deg)
        mujoco.mj_forward(model, data)
        frame = renderer.render_frame()
        axes[i].imshow(frame)
        axes[i].set_title(title)
        axes[i].axis("off")

    plt.suptitle("3-Link 3D Arm Configurations", fontsize=14)
    plt.tight_layout()

    save_path = Path(__file__).parent / "step_04_output.png"
    plt.savefig(str(save_path), dpi=100)
    print(f"Saved renders to {save_path.name}")
    plt.show()

    renderer.close()


if __name__ == "__main__":
    passed = run_verification()
    if passed:
        render_configurations()
