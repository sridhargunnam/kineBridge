"""Dual-mode MuJoCo renderer: offscreen (default) or interactive viewer.

Offscreen mode works headless (WSL2-compatible) by rendering to numpy arrays
via mujoco.Renderer and displaying them with matplotlib or saving as video.

Interactive mode uses mujoco.viewer.launch_passive() and requires a display.
Pass --interactive on the command line or set interactive=True to enable.
"""

import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np
import mujoco


class MuJoCoRenderer:
    """Unified renderer wrapping both offscreen and interactive MuJoCo viewers."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        width: int = 640,
        height: int = 480,
        interactive: Optional[bool] = None,
    ):
        self.model = model
        self.data = data
        self.width = width
        self.height = height

        if interactive is None:
            interactive = "--interactive" in sys.argv

        self.interactive = interactive
        self._viewer = None
        self._renderer = None
        self._frames: list[np.ndarray] = []

        if self.interactive:
            self._init_interactive()
        else:
            self._init_offscreen()

    def _init_offscreen(self):
        self._renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)

    def _init_interactive(self):
        import mujoco.viewer
        self._viewer = mujoco.viewer.launch_passive(self.model, self.data)

    def render_frame(self) -> np.ndarray:
        """Render the current state to a numpy RGB array (offscreen only)."""
        if self._renderer is None:
            self._init_offscreen()
        self._renderer.update_scene(self.data)
        return self._renderer.render().copy()

    def step_and_render(self, n_steps: int = 1, record: bool = False) -> Optional[np.ndarray]:
        """Step the simulation and render.

        Args:
            n_steps: Number of mujoco.mj_step() calls.
            record: If True (offscreen mode), store the frame for video export.

        Returns:
            RGB numpy array in offscreen mode, None in interactive mode.
        """
        for _ in range(n_steps):
            mujoco.mj_step(self.model, self.data)

        if self.interactive:
            if self._viewer is not None:
                self._viewer.sync()
            return None
        else:
            frame = self.render_frame()
            if record:
                self._frames.append(frame)
            return frame

    def show_frame(self, frame: Optional[np.ndarray] = None) -> None:
        """Display a single frame using matplotlib (offscreen mode)."""
        import matplotlib.pyplot as plt

        if frame is None:
            frame = self.render_frame()
        plt.figure(figsize=(self.width / 100, self.height / 100))
        plt.imshow(frame)
        plt.axis("off")
        plt.tight_layout()
        plt.show()

    def save_video(self, path: str, fps: int = 30) -> None:
        """Save recorded frames as an MP4 video.

        Args:
            path: Output file path (e.g. "output.mp4").
            fps: Frames per second for the video.
        """
        import imageio

        if not self._frames:
            print("No frames recorded. Call step_and_render(record=True) first.")
            return
        imageio.mimwrite(path, self._frames, fps=fps)
        print(f"Saved {len(self._frames)} frames to {path}")

    def clear_frames(self) -> None:
        """Clear the recorded frame buffer."""
        self._frames.clear()

    def is_running(self) -> bool:
        """Check if the interactive viewer window is still open."""
        if self.interactive and self._viewer is not None:
            return self._viewer.is_running()
        return True

    def close(self) -> None:
        """Clean up renderer resources."""
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None


def load_model_and_data(xml_path: str) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Load a MuJoCo model and create data from an MJCF XML file.

    Args:
        xml_path: Path to the .xml MJCF file.

    Returns:
        Tuple of (MjModel, MjData).
    """
    model = mujoco.MjModel.from_xml_path(str(Path(xml_path).resolve()))
    data = mujoco.MjData(model)
    return model, data


def simple_sim_loop(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    duration: float = 5.0,
    render_every: int = 5,
    interactive: Optional[bool] = None,
    record_video: Optional[str] = None,
) -> None:
    """Run a simple simulation loop with rendering.

    Args:
        model: MuJoCo model.
        data: MuJoCo data.
        duration: Total simulation time in seconds.
        render_every: Render every N simulation steps.
        interactive: Force interactive mode (None = check CLI flag).
        record_video: If provided, save video to this path (offscreen only).
    """
    renderer = MuJoCoRenderer(model, data, interactive=interactive)
    n_steps = int(duration / model.opt.timestep)
    record = record_video is not None

    try:
        for i in range(n_steps):
            mujoco.mj_step(model, data)
            if i % render_every == 0:
                if renderer.interactive:
                    renderer._viewer.sync()
                    if not renderer.is_running():
                        break
                    time.sleep(model.opt.timestep * render_every)
                else:
                    frame = renderer.render_frame()
                    if record:
                        renderer._frames.append(frame)

        if record and record_video:
            renderer.save_video(record_video)

    finally:
        renderer.close()
