# Module 3: Numerical IK — The Workhorse

*Detailed content will be written when this module is started.*

## Topics Covered

- Why analytic IK breaks down beyond simple arms
- The geometric Jacobian: relating joint velocities to end-effector velocities
- Jacobian pseudo-inverse IK (J-pinv)
- Damped Least Squares (DLS / Levenberg-Marquardt)
- scipy.optimize for IK as a nonlinear optimization problem
- Redundancy in 7-DOF arms: null-space exploitation
- Joint limits and singularity avoidance

## Step Files (to be created)

- `step_01_jacobian.py` — Compute the geometric Jacobian
- `step_02_pseudoinverse.py` — J-pinv iterative IK solver
- `step_03_dls.py` — Damped least squares IK
- `step_04_scipy_ik.py` — scipy.optimize-based IK
- `step_05_7dof_redundancy.py` — Null-space exploitation for 7-DOF

## Prerequisites

- Completed Module 2 (DH transforms, 3D modeling)
