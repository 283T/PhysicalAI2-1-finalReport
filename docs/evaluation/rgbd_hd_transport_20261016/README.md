# RGB-D transport comparison, 2026-10-16

These records validate image transport only. They are not grasp, loaded motion or insertion successes.

The first complete high-resolution task attempt stopped at its initial **640×480 overhead observation**. With the 128 MiB-per-participant profile, it received 20 RGB messages but only 4 depth messages and 2 exact timestamp pairs before the 90-second limit. The image bridge reported `Failed to create segment` and could not register SHM. Full task failure: `../cooperative_nest_20261016/hd_initial_transport_rejected`.

After that controller finished, image-only probes ran sequentially in the same scene:

| Probe | Camera / resolution | Participant segment | Result |
|---|---|---:|---|
| pair128 | Overhead, 640×480 | 128 MiB | 3 RGB/depth pairs, stable detection, 7.45 s |
| pair64 | Inspection, 1920×1440 | 64 MiB | 4 RGB/depth pairs, stable detection, 29.97 s |

Camera, image size and simulator state differ; these rows do not establish a speedup or an isolated effect of segment size. Their native Gazebo message counters and ROS counters both confirm paired arrivals.

During the two-participant 128 MiB probe, `df -h /dev/shm` reported 269 MiB used in the 512 MiB container; two segment files were about 135 MiB each, including transport overhead. The complete task uses four participants: the owning arm node, arm bridge, image observer and image bridge. Four allocations of this size exceed that container limit. This capacity explanation is consistent with the bridge's SHM creation failure. Fast DDS documents that each participant creates its own segment; see [Shared Memory Transport](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/transport/shared_memory/shared_memory.html). The post-exit empty shared-memory filesystem is not a measurement of peak usage.

The new `configs/fastdds_rgbd_hd64.xml` keeps a 16 MiB maximum message, enough for the 1920×1440 float depth image, but uses a 64 MiB segment. Four segments plus bookkeeping fit the container. Regression checks cover both payload capacity and four-participant capacity. The original 32 MiB profile and diagnostic 128 MiB profile remain preserved. The new full task with 64 MiB (`outputs/dual_nest_hd_shm64_20261016/pickup`) passed its initial observation: 10 RGB/depth pairs, no pending skew, stable detection in 33.12 s. That full task is still running; it is not counted as a successful assembly.

Each probe archive includes image pixels, compressed depth, calibration, CameraInfo intrinsics, transport/native counts and logs, with a SHA manifest. Both XML profiles used are copied here. Reproduce read-only probes against the running high-resolution case from repository root:

```sh
docker compose exec -T -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/configs/fastdds_rgbd_large.xml gui bash -lc 'source /opt/ros/jazzy/setup.bash && python3 scripts/observe_connectors.py --bridge --plugs P3 --require-6d --probe-native --output outputs/reproduce_overhead128'
docker compose exec -T -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/configs/fastdds_rgbd_hd64.xml gui bash -lc 'source /opt/ros/jazzy/setup.bash && python3 scripts/observe_connectors.py --bridge --plugs P3 --require-6d --end-markers --calibration outputs/dual_nest_flat_tags_hd_dt01_20261016/case/camera_calibration.json --probe-native --output outputs/reproduce_inspection64'
```

Run these probes only after the owning controller has finished. Do not reset a scene merely because an observation is pending. The first probe uses the overhead view; only the second tests high-resolution inspection payloads.
