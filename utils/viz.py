"""2D arm visualization helpers using matplotlib.

Used primarily in Modules 1-2 for plotting planar robot arms,
joint positions, workspace boundaries, and target markers.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from typing import Optional


def plot_arm_2d(
    joint_positions: np.ndarray,
    ax: Optional[plt.Axes] = None,
    target: Optional[np.ndarray] = None,
    title: str = "",
    link_color: str = "royalblue",
    joint_color: str = "black",
    target_color: str = "red",
    link_width: float = 3.0,
    joint_size: float = 8.0,
    show: bool = True,
) -> plt.Axes:
    """Plot a 2D planar arm given an array of joint positions.

    Args:
        joint_positions: (N, 2) array where row 0 is the base (origin),
                         and each subsequent row is a joint or end-effector.
        ax: Existing matplotlib Axes to draw on. Created if None.
        target: Optional (2,) array for the target position marker.
        title: Plot title string.
        link_color: Color for the arm links.
        joint_color: Color for the joint markers.
        target_color: Color for the target marker.
        link_width: Line width for links.
        joint_size: Marker size for joints.
        show: If True, calls plt.show() at the end.

    Returns:
        The matplotlib Axes used for plotting.
    """
    if ax is None:
        fig, ax = plt.subplots(1, 1, figsize=(6, 6))

    joint_positions = np.asarray(joint_positions)

    ax.plot(
        joint_positions[:, 0],
        joint_positions[:, 1],
        "-o",
        color=link_color,
        linewidth=link_width,
        markersize=joint_size,
        markerfacecolor=joint_color,
        markeredgecolor=joint_color,
        zorder=3,
    )

    ax.plot(
        joint_positions[0, 0],
        joint_positions[0, 1],
        "s",
        color="green",
        markersize=joint_size + 2,
        zorder=4,
        label="Base",
    )

    ax.plot(
        joint_positions[-1, 0],
        joint_positions[-1, 1],
        "D",
        color="orange",
        markersize=joint_size,
        zorder=4,
        label="End-effector",
    )

    if target is not None:
        target = np.asarray(target)
        ax.plot(
            target[0],
            target[1],
            "x",
            color=target_color,
            markersize=12,
            markeredgewidth=3,
            zorder=5,
            label="Target",
        )

    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)
    if title:
        ax.set_title(title)
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    if show:
        plt.tight_layout()
        plt.show()

    return ax


def plot_workspace_2d(
    link_lengths: list[float],
    n_samples: int = 5000,
    ax: Optional[plt.Axes] = None,
    show: bool = True,
) -> plt.Axes:
    """Scatter-plot the reachable workspace of a planar arm by random sampling.

    Args:
        link_lengths: List of link lengths [L1, L2, ...].
        n_samples: Number of random joint angle samples.
        ax: Existing matplotlib Axes. Created if None.
        show: If True, calls plt.show().

    Returns:
        The matplotlib Axes.
    """
    if ax is None:
        fig, ax = plt.subplots(1, 1, figsize=(6, 6))

    n_links = len(link_lengths)
    angles = np.random.uniform(-np.pi, np.pi, size=(n_samples, n_links))

    xs = np.zeros(n_samples)
    ys = np.zeros(n_samples)
    cumulative_angle = np.zeros(n_samples)

    for i, length in enumerate(link_lengths):
        cumulative_angle += angles[:, i]
        xs += length * np.cos(cumulative_angle)
        ys += length * np.sin(cumulative_angle)

    ax.scatter(xs, ys, s=1, alpha=0.3, color="steelblue")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_title(f"Workspace ({n_links}-link arm)")
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    if show:
        plt.tight_layout()
        plt.show()

    return ax


def animate_arm_2d(
    trajectory: list[np.ndarray],
    interval_ms: int = 50,
    target: Optional[np.ndarray] = None,
    title: str = "Arm Animation",
    save_path: Optional[str] = None,
) -> None:
    """Animate a sequence of arm configurations.

    Args:
        trajectory: List of (N, 2) joint position arrays (one per frame).
        interval_ms: Milliseconds between frames.
        target: Optional (2,) target position to show throughout.
        title: Animation title.
        save_path: If provided, save animation to this file path.
    """
    from matplotlib.animation import FuncAnimation

    fig, ax = plt.subplots(1, 1, figsize=(6, 6))

    all_pts = np.vstack(trajectory)
    margin = 0.5
    xlim = (all_pts[:, 0].min() - margin, all_pts[:, 0].max() + margin)
    ylim = (all_pts[:, 1].min() - margin, all_pts[:, 1].max() + margin)

    (line,) = ax.plot([], [], "-o", color="royalblue", linewidth=3, markersize=8)

    if target is not None:
        ax.plot(target[0], target[1], "rx", markersize=12, markeredgewidth=3)

    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_title(title)

    def update(frame_idx):
        pts = trajectory[frame_idx]
        line.set_data(pts[:, 0], pts[:, 1])
        return (line,)

    anim = FuncAnimation(
        fig, update, frames=len(trajectory), interval=interval_ms, blit=True
    )

    if save_path:
        anim.save(save_path, writer="pillow")
    else:
        plt.show()
