# Module 4: Traditional ML for Kinematics

*Detailed content will be written when this module is started.*

## Topics Covered

- Framing IK as a supervised learning problem
- Generating training data from FK (joint angles → end-effector positions)
- MLP architecture for function approximation
- The "multiple solutions" problem: why MSE loss is problematic for IK
- Mixture Density Networks (MDN) as a potential solution
- Training, validation, and evaluation pipeline in PyTorch
- Comparing ML-based IK vs classical solvers (speed, accuracy, generalization)

## Step Files (to be created)

- `step_01_dataset_gen.py` — Generate massive FK pair dataset
- `step_02_mlp_model.py` — PyTorch MLP architecture
- `step_03_train.py` — Training loop with loss curves
- `step_04_evaluate.py` — Evaluation and comparison vs classical IK

## Prerequisites

- Completed Module 3 (numerical IK solvers for ground-truth comparison)
