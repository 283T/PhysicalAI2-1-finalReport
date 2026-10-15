# Empty-gripper wrist response, 2026-10-16

These are development comparisons, **not assembly successes**. The harness and cameras were removed; the arms, fixtures, carrier, contact monitoring and bounded force-PID controllers were retained. Native robot joint feedback measures tracking. No object pose is used for control. All failed runs are retained.

Both sixth joints receive +0.04, −0.04 and 0 rad. Each movement must pass the existing convergence checks, followed by a five-second dwell. The last second must remain below 0.01 rad absolute joint error. The figure includes transient errors; passing does not mean every transient was below the gate. An aborted sequence is a failed comparison, with unexecuted steps absent.

| Archive | Left P / I | Right P / I | Step (ms) | Sequence | Largest last-second error L / R (rad) |
|---|---:|---:|---:|---|---|
| i40 | 640 / 40 | 320 / 40 | 1 | Passed | 0.006252 / 0.006000 |
| i0 | 640 / 0 | 320 / 40 | 1 | Stopped on first movement | No completed dwell; final movement error L=0.014549 |
| dt05 | 640 / 40 | 320 / 40 | 0.5 | Passed | 0.004676 / 0.005999 |
| i0_dt05 | 640 / 0 | 320 / 40 | 0.5 | Passed | 0.008315 / 0.005999 |
| i0_dt025 | 640 / 0 | 320 / 40 | 0.25 | Passed | 0.005075 / 0.006008 |
| both_i0_dt01 | 640 / 0 | 640 / 0 | 0.1 | Passed | 0.003026 / 0.003768 |

Left P640 uses D=0.282842712474619; the added right P640 uses D=0.282842712; P320 uses D=0.2. Other gains, joint damping/friction and force limits are preserved. The last row changes both the right controller and timestep; it does not isolate their effects. Smaller timesteps change the measured response, but these measurements do not identify internal torque, PID integral state or a unique cause. JointState effort=0 is not torque evidence. No result is a loaded transport or insertion validation.

Each archive contains the exact executed world, native trajectory, final validation, server and bridge logs, and SHA-256 manifest. `summary.json` indexes these records. The first two validations predate the timestep metadata fields; their executed worlds explicitly use 0.001 s. Robot URDF hashes are recorded in each validation; use the preserved `models` under `../dual_pickup_20261016/support140_settle30_dual_hold`.

Reproduction from the repository root, using new output directories:

```sh
python3 scripts/retune_joint_controller.py \
  --case docs/evaluation/dual_pickup_20261016/support140_settle30_dual_hold/case \
  --output outputs/reproduce_wrist_left640 --arm left_arm --joint-index 6 \
  --p-gain 640 --d-gain 0.282842712474619
python3 scripts/retune_joint_controller.py \
  --case outputs/reproduce_wrist_left640 --output outputs/reproduce_wrist_left_i0 \
  --arm left_arm --joint-index 6 --p-gain 640 --d-gain 0.282842712474619 --i-gain 0
python3 scripts/retune_joint_controller.py \
  --case outputs/reproduce_wrist_left_i0 --output outputs/reproduce_wrist_both_i0 \
  --arm right_arm --joint-index 6 --p-gain 640 --d-gain 0.282842712 --i-gain 0
docker compose exec -T gui bash -lc 'source /opt/ros/jazzy/setup.bash && python3 scripts/joint_response_benchmark.py --case outputs/reproduce_wrist_both_i0 --models docs/evaluation/dual_pickup_20261016/support140_settle30_dual_hold/models --output outputs/reproduce_both_i0_dt01 --max-step-size 0.0001'
docker compose run --rm dev python3 docs/evaluation/wrist_response_20261016/plot.py
```

Run comparisons sequentially: Gazebo partitions are unique, but the benchmark uses ROS domain 84. Substitute the corresponding case and omit `--max-step-size` for the original step. To refine the complete assembly scene, use `scripts/refine_physics_timestep.py`; it preserves the harness and cameras and records provenance. Force caps and acceptance criteria must not be changed to make these comparisons pass.
