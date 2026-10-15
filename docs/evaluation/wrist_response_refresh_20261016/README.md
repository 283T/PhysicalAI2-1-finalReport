# Persistent-command refresh validation, 2026-10-16

**Empty-gripper response only. No grasp, loaded transfer or assembly success.** The benchmark's executed `world.sdf` is byte-identical to `../wrist_response_20261016/both_i0_dt01/world.sdf` (SHA-256 `f2721ea2e09c623aa865df8f24ba1199a5295eb79044e25cd921a98b9872c1ce`). Force-PID gains, limits, robot geometry and 0.1 ms physics step are unchanged.

Both wrists completed +0.04, −0.04 and 0 rad. Last-second maximum joint errors were below the unchanged 0.01 rad gate: left at most 0.003026 rad, right at most 0.003768 rad. Full trajectories, convergence checks, contacts and logs are preserved. The new execution budget changes bounded wall-clock waiting, not physical durations or convergence thresholds; 30 seconds without simulation-clock progress still stops execution.

`CommandRefresh` publishes every changed value immediately, without an epsilon. An unchanged channel is resent every 0.1 wall seconds. A passive native Gazebo subscriber measured 29 messages in 3 seconds on one unchanged channel during this benchmark (`command_probe.json`). `full_scene_command_probe.json` additionally records 30 messages in 3 seconds, all value 0, on the parked left arm during the subsequent complete high-resolution scene. Neither probe publishes commands. The source SHA of the generic capture script is recorded in the latter result; the first probe used the equivalent initial fixed-path capture.

Do not infer a simulator speedup: stored history spans about 166.67 wall seconds for 26.03 simulation seconds here; the earlier empty-gripper comparison spanned about 82.87 wall seconds for 26.02 simulation seconds. These are single runs under differing concurrent workload. Reduced repeated publication is verified; an overall performance improvement is not established.

From repository root, reproduce the benchmark using a new output directory:

```sh
docker compose exec -T -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/configs/fastdds_rgbd_hd64.xml gui bash -lc 'source /opt/ros/jazzy/setup.bash && python3 scripts/joint_response_benchmark.py --case docs/evaluation/cooperative_nest_20261016/hd_hover_wall_limit/case --models docs/evaluation/dual_pickup_20261016/support140_settle30_dual_hold/models --output outputs/reproduce_refresh_wrist'
```

The runner removes cameras and harness for this diagnostic, owns its own Gazebo server and uses ROS domain 84; run it sequentially with other benchmarks using that domain. For a passive receipt probe, pass that benchmark process's partition `wrist-response-<PID>` to the generic `command_probe.py --partition ... --output ...`. Against the default partition of a running complete scene, omit `--partition`. Probe only the intended unchanged channel and interpret counts as transport evidence, not physical force or task completion.
