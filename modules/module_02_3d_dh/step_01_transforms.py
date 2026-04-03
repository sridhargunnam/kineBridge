"""Module 2, Step 1: 4x4 Homogeneous Transformation Matrices.

Reference: theory.md — Sections 2.2, 2.8

Goal:
    Build the fundamental building blocks of 3D kinematics from scratch:
    - 3x3 rotation matrices about x, y, z axes
    - 4x4 homogeneous transformation matrix from rotation + translation
    - Compose (chain) transforms via matrix multiplication
    - Extract position and orientation from a composed transform

What you will learn:
    - How rotation matrices encode orientation
    - How homogeneous coordinates unify rotation and translation
    - Why matrix multiplication order matters (T_A @ T_B != T_B @ T_A)
    - Verifying orthogonality: R^T @ R == I, det(R) == +1
"""

import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


# ---------------------------------------------------------------------------
# YOUR TASK 1: Implement the three elementary rotation matrices
# ---------------------------------------------------------------------------

def rotation_x(theta: float) -> np.ndarray:
    """3x3 rotation matrix for rotation about the x-axis.

    Args:
        theta: Angle in radians.

    Returns:
        (3, 3) rotation matrix.

    Hint:
        See theory.md Section 2.2, "Elementary Rotation Matrices":
        Rx(θ) = | 1    0       0    |
                | 0   cos(θ) -sin(θ) |
                | 0   sin(θ)  cos(θ) |

        The x-row and x-column are [1, 0, 0] because rotation about x
        leaves the x-component unchanged.
    """
    # === TODO START ===
    raise NotImplementedError("Implement rotation_x")
    # === TODO END ===


def rotation_y(theta: float) -> np.ndarray:
    """3x3 rotation matrix for rotation about the y-axis.

    Args:
        theta: Angle in radians.

    Returns:
        (3, 3) rotation matrix.

    Hint:
        Ry(θ) = |  cos(θ)  0  sin(θ) |
                |    0     1    0    |
                | -sin(θ)  0  cos(θ) |

        NOTE the sign pattern: the -sin is in the BOTTOM-LEFT, not
        top-right like Rz. This ensures det(R) = +1 (right-handed).
    """
    # === TODO START ===
    raise NotImplementedError("Implement rotation_y")
    # === TODO END ===


def rotation_z(theta: float) -> np.ndarray:
    """3x3 rotation matrix for rotation about the z-axis.

    Args:
        theta: Angle in radians.

    Returns:
        (3, 3) rotation matrix.

    Hint:
        Rz(θ) = | cos(θ) -sin(θ)  0 |
                | sin(θ)  cos(θ)  0 |
                |   0       0     1 |

        This is the 3D version of our Module 1 rot2d, with a 1 on the z-diagonal.
    """
    # === TODO START ===
    raise NotImplementedError("Implement rotation_z")
    # === TODO END ===


# ---------------------------------------------------------------------------
# YOUR TASK 2: Build a 4x4 homogeneous transformation matrix
# ---------------------------------------------------------------------------

def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """Create a 4x4 homogeneous transformation matrix.

    Args:
        rotation: (3, 3) rotation matrix.
        translation: (3,) translation vector.

    Returns:
        (4, 4) homogeneous transformation matrix:
            | R  p |
            | 0  1 |

    Hint:
        Start with a 4x4 identity matrix. Place R in the top-left 3x3 block
        and p in the top-right 3x1 column. The bottom row stays [0, 0, 0, 1].
    """
    # === TODO START ===
    raise NotImplementedError("Implement make_transform")
    # === TODO END ===


# ---------------------------------------------------------------------------
# YOUR TASK 3: Extract position and orientation from a transform
# ---------------------------------------------------------------------------

def extract_position(T: np.ndarray) -> np.ndarray:
    """Extract the 3D position (translation) from a 4x4 transform.

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        (3,) position vector.

    Hint:
        The position is the rightmost column of the top 3 rows.
    """
    # === TODO START ===
    raise NotImplementedError("Implement extract_position")
    # === TODO END ===


def extract_rotation(T: np.ndarray) -> np.ndarray:
    """Extract the 3x3 rotation matrix from a 4x4 transform.

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        (3, 3) rotation matrix.

    Hint:
        The rotation is the top-left 3x3 block.
    """
    # === TODO START ===
    raise NotImplementedError("Implement extract_rotation")
    # === TODO END ===


# ---------------------------------------------------------------------------
# VERIFICATION (do not modify)
# ---------------------------------------------------------------------------

def verify_rotations():
    """Verify rotation matrix properties."""
    print("=" * 60)
    print("Verifying Rotation Matrices")
    print("=" * 60)

    all_passed = True
    for name, func in [("Rx", rotation_x), ("Ry", rotation_y), ("Rz", rotation_z)]:
        for angle, desc in [(0, "0"), (np.pi / 2, "π/2"), (np.pi, "π"), (-np.pi / 4, "-π/4")]:
            R = func(angle)

            is_orthogonal = np.allclose(R.T @ R, np.eye(3), atol=1e-10)
            det_is_one = np.isclose(np.linalg.det(R), 1.0, atol=1e-10)
            shape_ok = R.shape == (3, 3)

            ok = is_orthogonal and det_is_one and shape_ok
            status = "PASS" if ok else "FAIL"
            if not ok:
                all_passed = False
            print(f"  [{status}] {name}({desc}): "
                  f"shape={R.shape}, det={np.linalg.det(R):.6f}, "
                  f"orthogonal={is_orthogonal}")

    # Specific value checks
    checks = [
        ("Rx(0) == I", np.allclose(rotation_x(0), np.eye(3))),
        ("Ry(0) == I", np.allclose(rotation_y(0), np.eye(3))),
        ("Rz(0) == I", np.allclose(rotation_z(0), np.eye(3))),
        ("Rz(π/2) @ [1,0,0] == [0,1,0]",
         np.allclose(rotation_z(np.pi / 2) @ [1, 0, 0], [0, 1, 0], atol=1e-10)),
        ("Rx(π/2) @ [0,1,0] == [0,0,1]",
         np.allclose(rotation_x(np.pi / 2) @ [0, 1, 0], [0, 0, 1], atol=1e-10)),
        ("Ry(π/2) @ [0,0,1] == [1,0,0]",
         np.allclose(rotation_y(np.pi / 2) @ [0, 0, 1], [1, 0, 0], atol=1e-10)),
    ]
    print()
    for desc, passed in checks:
        status = "PASS" if passed else "FAIL"
        if not passed:
            all_passed = False
        print(f"  [{status}] {desc}")

    print("=" * 60)
    if all_passed:
        print("All rotation tests passed!")
    else:
        print("Some tests FAILED.")
    print()
    return all_passed


def verify_transforms():
    """Verify homogeneous transform construction and extraction."""
    print("=" * 60)
    print("Verifying Homogeneous Transforms")
    print("=" * 60)

    all_passed = True

    R = rotation_z(np.pi / 4)
    p = np.array([1.0, 2.0, 3.0])
    T = make_transform(R, p)

    checks = [
        ("Shape is (4,4)", T.shape == (4, 4)),
        ("Bottom row is [0,0,0,1]", np.allclose(T[3, :], [0, 0, 0, 1])),
        ("Top-left 3x3 == R", np.allclose(T[:3, :3], R)),
        ("Top-right column == p", np.allclose(T[:3, 3], p)),
        ("extract_position(T) == p", np.allclose(extract_position(T), p)),
        ("extract_rotation(T) == R", np.allclose(extract_rotation(T), R)),
    ]
    for desc, passed in checks:
        status = "PASS" if passed else "FAIL"
        if not passed:
            all_passed = False
        print(f"  [{status}] {desc}")

    # Chaining test: translate then rotate vs rotate then translate
    T1 = make_transform(np.eye(3), np.array([1, 0, 0]))  # pure translation
    T2 = make_transform(rotation_z(np.pi / 2), np.array([0, 0, 0]))  # pure rotation

    T_chain1 = T2 @ T1  # rotate AFTER translate
    T_chain2 = T1 @ T2  # translate AFTER rotate

    p1 = extract_position(T_chain1)
    p2 = extract_position(T_chain2)
    chains_differ = not np.allclose(p1, p2)
    status = "PASS" if chains_differ else "FAIL"
    if not chains_differ:
        all_passed = False
    print(f"\n  [{status}] T2@T1 != T1@T2 (order matters): "
          f"p1={p1}, p2={p2}")

    # T2@T1: first translate [1,0,0], then rotate 90° about z → result at [0,1,0]
    status = "PASS" if np.allclose(p1, [0, 1, 0], atol=1e-10) else "FAIL"
    if not np.allclose(p1, [0, 1, 0], atol=1e-10):
        all_passed = False
    print(f"  [{status}] Rotate(90°z) after Trans([1,0,0]) → position at [0,1,0]")

    print("=" * 60)
    if all_passed:
        print("All transform tests passed!")
    else:
        print("Some tests FAILED.")
    print()
    return all_passed


def visualize_transforms():
    """Visualize coordinate frames after applying transforms."""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    def draw_frame(ax, T, label, length=0.3):
        origin = extract_position(T)
        R = extract_rotation(T)
        colors = ["r", "g", "b"]
        axis_labels = ["x", "y", "z"]
        for i in range(3):
            direction = R[:, i] * length
            ax.quiver(*origin, *direction, color=colors[i], arrow_length_ratio=0.15)

        ax.text(*(origin + 0.05), label, fontsize=10, fontweight="bold")

    # World frame
    draw_frame(ax, np.eye(4), "World", length=0.5)

    # Frame after rotating 45° about z and translating [1, 0, 0]
    T1 = make_transform(rotation_z(np.pi / 4), np.array([1, 0, 0]))
    draw_frame(ax, T1, "T1")

    # Frame after rotating 90° about x and translating [0, 1, 0.5]
    T2 = make_transform(rotation_x(np.pi / 2), np.array([0, 1, 0.5]))
    draw_frame(ax, T2, "T2")

    # Chained: T1 then T2
    T_chain = T1 @ T2
    draw_frame(ax, T_chain, "T1·T2")

    ax.set_xlim(-0.5, 2)
    ax.set_ylim(-0.5, 2)
    ax.set_zlim(-0.5, 2)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Coordinate Frames After Transforms")

    save_path = Path(__file__).parent / "step_01_output.png"
    plt.tight_layout()
    plt.savefig(str(save_path), dpi=100)
    print(f"Saved visualization to {save_path.name}")
    plt.show()


if __name__ == "__main__":
    r_ok = verify_rotations()
    t_ok = verify_transforms()
    if r_ok and t_ok:
        visualize_transforms()
