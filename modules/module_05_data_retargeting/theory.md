# Module 5: Real-World Data & Retargeting

*Detailed content will be written when this module is started.*

## Topics Covered

- Egocentric robot datasets: structure, formats (HDF5, etc.)
- Extracting 6D Cartesian waypoints (position + orientation) from demonstrations
- Coordinate frame alignment between dataset and simulated robot
- Retargeting: mapping human/teleop demonstrations to a different robot morphology
- Using classical IK solvers to create ground-truth joint trajectories
- Trajectory smoothing and temporal consistency
- Validation: replaying retargeted trajectories in MuJoCo

## Step Files (to be created)

- `step_01_data_ingest.py` — Parse egocentric/teleop dataset formats
- `step_02_waypoint_extract.py` — Extract 6D Cartesian waypoints
- `step_03_retarget.py` — Classical IK retargeting pipeline
- `step_04_validate.py` — Replay and visual verification in MuJoCo

## Prerequisites

- Completed Module 3 (numerical IK for the retargeting solver)
- A 7-DOF arm XML model from Module 3
