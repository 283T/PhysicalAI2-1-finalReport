# Permanent stepped finger candidate, 2026-10-16

**Offline geometry only. No physical regrasp, wire support or insertion success.** This candidate is not installed in the currently running timestep comparison.

The original right C-shaped cage overlaps the nest during closing, even when its open approach is clear. This candidate uses the left arm for the later top pickup. Its fingers have broad split wire pads from local Z=0 to 67 mm (overall Y span 50 mm), followed by narrow split connector pads from Z=67 to 80 mm (overall Y span 20 mm). The jaw closing faces and mechanical travel are preserved. The complete profile is permanent; it must be present from the beginning of any physical trial.

`stepped_wire_profile.py` creates a copy of a known bare wire-finger URDF. It preserves the source mass per collision volume, then recomputes mass, center of mass and the full inertia tensor from the four solid boxes. Source input is not modified. The visual boxes match the collision boxes. Tests independently verify tip/shoulder extents, continuity, volume, center of mass and inertia.

`validation.json` passes left hover, approach, full mechanical closing stroke and post-close lift checks. It assumes both arms start at zero, the right arm remains parked, both fingers are open for approach, and the connector is at the **nominal** nest seat from an earlier offline prediction. It is not a measurement of an actual seated connector. Arm-box margin remains 5 mm; fixture margin remains 1 mm. Closing stroke includes [-0.5,20.5] mm for each jaw.

The existing wire grasp is only 6 mm deep and would use the narrow tip. To use the broad wire-support region, a deeper grasp (approximately 19 mm) must first pass fresh RGB-D, continuous IK, collision, closure and physical holding checks; the existing controller permits at most 12 mm. No depth limit or success criterion has been changed in this experiment. Actual right release/retreat and left parking are also still unverified. The current live experiment retains the original fingers.

Reproduce from the repository root using a new output directory:

```sh
docker compose run --rm dev python3 docs/evaluation/stepped_regrasp_20261016/reproduce.py --output outputs/reproduce_stepped_regrasp
```

The generated URDFs, validation and SHA manifest are preserved here. Before physical use, generate a new complete world from the same URDFs, validate its mass/collision consistency, and begin a new trial. Do not replace a gripper on a running scene.


`prepare_stepped_case.py` also builds a **separate** complete case and models directory via Gazebo URDF-to-SDF conversion, replacing only the two left finger links and their contact sensors. It checks that every other world element is identical and that all sensor references resolve (30 original contact sensors become 34). The new candidate at `outputs/stepped_bothwrist640_i0_dt01_checked_20261016` has not been started. Generated-scene validation and provenance are preserved in this directory. Example using a preserved baseline:

```sh
docker compose exec -T gui bash -lc 'source /opt/ros/jazzy/setup.bash && python3 scripts/prepare_stepped_case.py --case docs/evaluation/cooperative_nest_20261016/left_wrist640_pacing_stalled/case --models docs/evaluation/dual_pickup_20261016/support140_settle30_dual_hold/models --output outputs/reproduce_stepped_scene'
```
