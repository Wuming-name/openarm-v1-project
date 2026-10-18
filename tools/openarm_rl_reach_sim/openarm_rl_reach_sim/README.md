# OpenArm V1 simulation-only reaching

Start `[0,0,0,0.4,0,0,0]`; fixed end-effector goal FK(`[-0.3,0.3,0,0.6,0,0,0]`).
See **训练目标与操作说明.txt** for the full Chinese task definition and Ubuntu commands.

Place this directory at `tools/openarm_rl_reach_sim` next to the existing package.
The model and meshes must already exist at `../../models/openarm_v1_right/`.
Reuse the previous package's Python virtual environment, or install `requirements.txt` in a new one.

Run `prepare.py`, then `preflight.py`; proceed to `train.py` only after preflight passes.
Training and evaluation use CPU. `view_policy.py` needs a graphical desktop.

This package has no CAN interface or hardware execution command. Its model-assisted
reset represents an already-held virtual arm and is not a physical enable sequence.
PPO produces seven normalized reference velocities, not feedforward torques.
The controller integrates these into MIT-style PD reference angles with zero desired
velocity and zero feedforward fields. Model mismatch and motor torque scaling remain
unverified for physical deployment.

The deterministic preflight baseline is not a trained policy. A short training smoke
test confirms software operation, not learning convergence. See `VERIFICATION.txt`.
