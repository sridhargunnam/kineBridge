# Module 2: Expanding to 3D & Joint Types

*Detailed content will be written when this module is started.*

## Topics Covered

- 3-link arms in 3D space
- Prismatic (sliding) joints vs revolute (hinge) joints
- Spherical (ball) wrists
- Denavit-Hartenberg (DH) parameters and convention
- 4x4 homogeneous transformation matrices
- Chaining transforms: T_0^n = T_0^1 * T_1^2 * ... * T_{n-1}^n
- 3D nested bodies and joint limits in MJCF XML
- Joint types in MJCF: hinge, slide, ball

## Step Files (to be created)

- `step_01_transforms.py` — Build 4x4 homogeneous transform matrices from scratch
- `step_02_dh_params.py` — DH parameter table to chain of transforms
- `step_03_prismatic.py` — FK with prismatic joints
- `step_04_3link_mujoco.py` — 3D MuJoCo arm simulation
- `step_05_spherical_wrist.py` — Spherical wrist modeling

## Prerequisites

- Completed Module 1 (2D FK/IK, basic MJCF)
