"""Module 1, Step 2: Verify 1-Link FK Against MuJoCo Simulation.

Reference: theory.md — Sections 1.6, 1.7

Goal:
    Load your one_link_arm.xml model in MuJoCo, set joint angles, and verify
    that MuJoCo's computed end-effector position matches your numpy FK.

What you will learn:
    - Loading MJCF models with mujoco.MjModel.from_xml_path()
    - Setting joint positions via data.qpos
    - Calling mujoco.mj_forward() to compute kinematics
    - Reading site positions via data.site_xpos
    - The correspondence between your math and the physics engine

Prerequisites:
    - Complete your one_link_arm.xml (fill in the TODO sections)
    - Working forward_kinematics_1link() from step_01
"""

import sys
from pathlib import Path

import numpy as np
import mujoco

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from modules.module_01_2d_basics.step_01_one_link_fk import forward_kinematics_1link
from utils.mujoco_renderer import MuJoCoRenderer, load_model_and_data


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

MODEL_XML = Path(__file__).parent / "models" / "one_link_arm.xml"


# ---------------------------------------------------------------------------
# YOUR TASK: Load the model, set angles, and verify FK
# ---------------------------------------------------------------------------

def verify_fk_against_mujoco(theta: float, L: float = 1.0) -> dict:
    """Compare numpy FK with MuJoCo's computed position.

    Steps (implement the TODO sections):
        1. Load the MuJoCo model from one_link_arm.xml
        2. Set data.qpos[0] = theta
        3. Call mujoco.mj_forward(model, data)
        4. Read the end-effector site position from data.site_xpos
        5. Compute FK using your numpy function
        6. Compare the two

    Args:
        theta: Joint angle in radians.
        L: Link length (must match the XML model).

    Returns:
        Dict with 'numpy_pos', 'mujoco_pos', 'error', 'match' keys.
    """
    # === TODO START ===
    # 1. Load the model (hint: use load_model_and_data from utils)
    #    model, data = ???
    model, data = load_model_and_data(str(MODEL_XML))
    # 2. Set the joint angle
    #    data.qpos[0] = ???
    data.qpos[0] = theta
    # 3. Run MuJoCo's forward kinematics
    #    mujoco.mj_forward(???, ???)
    mujoco.mj_forward(model, data)
    # 4. Read the end-effector position from MuJoCo
    #    The site named "end_effector" gives a 3D position [x, y, z].
    #    For our 2D arm, z should be ~0.
    #    mujoco_pos = data.site_xpos[model.site('end_effector').id]
    mujoco_pos = data.site_xpos[model.site('end_effector').id]
    # 5. Compute your numpy FK
    #    numpy_x, numpy_y = forward_kinematics_1link(???, ???)
    numpy_x, numpy_y = forward_kinematics_1link(theta, L)
    # 6. Compare (provided below — just fill in the variables above)
    numpy_pos = np.array([numpy_x, numpy_y])
    mujoco_xy = mujoco_pos[:2]
    error = np.linalg.norm(numpy_pos - mujoco_xy)

    return {
        "numpy_pos": numpy_pos,
        "mujoco_pos": mujoco_pos,
        "error": error,
        "match": error < 1e-6,
    }
    # === TODO END ===

    numpy_pos = np.array([numpy_x, numpy_y])
    mujoco_xy = mujoco_pos[:2]
    error = np.linalg.norm(numpy_pos - mujoco_xy)

    return {
        "numpy_pos": numpy_pos,
        "mujoco_pos": mujoco_pos,
        "error": error,
        "match": error < 1e-6,
    }


# ---------------------------------------------------------------------------
# VERIFICATION AND VISUALIZATION (do not modify)
# ---------------------------------------------------------------------------

def run_verification():
    """Test FK at multiple angles."""
    test_angles = [0.0, np.pi / 6, np.pi / 4, np.pi / 3, np.pi / 2,
                   np.pi, -np.pi / 4, 3 * np.pi / 4]

    print("=" * 70)
    print("Verifying numpy FK vs MuJoCo")
    print("=" * 70)

    all_passed = True
    for theta in test_angles:
        result = verify_fk_against_mujoco(theta)
        status = "PASS" if result["match"] else "FAIL"
        if not result["match"]:
            all_passed = False
        print(f"  [{status}] θ={theta:+.4f}rad "
              f"| numpy=({result['numpy_pos'][0]:+.6f}, {result['numpy_pos'][1]:+.6f}) "
              f"| mujoco=({result['mujoco_pos'][0]:+.6f}, {result['mujoco_pos'][1]:+.6f}) "
              f"| err={result['error']:.2e}")

    print("=" * 70)
    if all_passed:
        print("All FK values match MuJoCo! Your XML and math are consistent.")
    else:
        print("MISMATCH detected. Check your XML model or FK function.")
    return all_passed


def render_arm_at_angle(theta: float = np.pi / 4):
    """Render the arm at a given angle using the MuJoCo renderer."""
    model, data = load_model_and_data(str(MODEL_XML))
    data.qpos[0] = theta
    mujoco.mj_forward(model, data)

    renderer = MuJoCoRenderer(model, data, width=640, height=480)
    frame = renderer.render_frame()
    renderer.show_frame(frame)
    renderer.close()


if __name__ == "__main__":
    passed = run_verification()
    if passed:
        print("\nRendering the arm at θ=π/4...")
        render_arm_at_angle(np.pi / 4)
