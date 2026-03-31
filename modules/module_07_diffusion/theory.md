# Module 7: Diffusion Policies

*Detailed content will be written when this module is started.*

## Topics Covered

- Denoising Diffusion Probabilistic Models (DDPM) fundamentals
- Forward diffusion process: adding noise over T timesteps
- Noise schedules: linear, cosine, learned
- Reverse process: learning to denoise
- Formulating trajectory generation as a diffusion process
- 1D temporal U-Net architecture for trajectory denoising
- Conditioning on observations/goals (classifier-free guidance)
- Training the diffusion policy on demonstration data
- Inference: iterative denoising to generate trajectories
- Executing generated trajectories in MuJoCo

## Step Files (to be created)

- `step_01_noise_schedule.py` — Implement forward diffusion process
- `step_02_unet.py` — 1D temporal U-Net in PyTorch
- `step_03_train.py` — Diffusion policy training loop
- `step_04_inference.py` — Denoising loop + MuJoCo execution

## Prerequisites

- Completed Module 6 (deep learning for trajectories)
- Completed Module 5 (demonstration dataset)
