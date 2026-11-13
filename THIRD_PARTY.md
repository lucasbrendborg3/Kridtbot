# Third-party packages

These packages in `src/` are copies of other projects (vendored as plain folders, not submodules).
Each one was added in the commit "Add third-party packages at upstream versions" exactly as it was at
the upstream commit below. Kridtbot's own changes to them are in the following commit, "Kridtbot
changes to third-party drivers", so `git show` on that commit lists everything we changed.

To update one from upstream: download the new upstream version, compare it with the upstream commit
below, and re-apply our changes from that commit. Then update the commit hash here.

| Package | Upstream | Branch | Commit (date) | Licence | Our changes |
| --- | --- | --- | --- | --- | --- |
| ros_odrive | https://github.com/odriverobotics/ros_odrive | main | `cbb492c` (2025-02-04) | MIT | Heavy: `odrive_ros2_control` hardware interface. Per-leg limits from the calibrate_suspension file (refuses to start if a leg is not homed), trapezoidal-trajectory position mode, hold legs when no controller is active, drive legs to the lower limit on shutdown. |
| myactuator_rmd | https://github.com/2b-t/myactuator_rmd | main | `1af8600` (2025-02-08) | MIT | Driver additions (protocol requests/responses, error codes, CAN node). |
| myactuator_rmd_ros | https://github.com/2b-t/myactuator_rmd_ros | main | `413ead9` (2025-01-10) | MIT | `myactuator_rmd_hardware` ros2_control interface (direction, filtering, wheel use). |
| suspension_controller | https://github.com/nich1157/suspension_controller | main | `5ea4d31` (2025-04-30) | Apache 2.0 (package.xml) | Controller changes (modes, reference topic, tuning). Upstream is a teammate's repo. |
| livox_ros_driver2 | https://github.com/tu-darmstadt-ros-pkg/livox_ros_driver2 | jazzy | `9c16332` (2025-01-20) | MIT | MID-360 config and launch for the rover. |
| Livox-SDK2 | https://github.com/tu-darmstadt-ros-pkg/Livox-SDK2 | master | `6a94015` (2024-02-18) | MIT | Build fixes, added `package.xml` so colcon builds it. |
| ros-imu-bno055 | https://github.com/dheera/ros-imu-bno055 | master | `aeb635a` (2024-07-24) | BSD | ROS 2 only (ROS 1 files removed), node fixes. |
| pointcloud_to_laserscan | https://github.com/ros-perception/pointcloud_to_laserscan | rolling | `442b332` (2024-09-11) | BSD | Sample launch file topics. |
| lidarslam_ros2 | https://github.com/rsasaki0109/lidarslam_ros2 | develop | `6f19df4` (2025-03-19) | BSD-2 | None. Includes its submodule `Thirdparty/ndt_omp_ros2` (https://github.com/rsasaki0109/ndt_omp_ros2, commit `41bdfba`) as plain files. |
| topic_tools | https://github.com/ros-tooling/topic_tools | rolling | `d03c255` (2025-04-27) | Apache 2.0 | None. |

`bno055_hardware_interface` (https://github.com/Crelle490/bno055_hardware_interface, a teammate's repo)
never had a commit upstream. It is added as our own code in the driver-changes commit.

Full commit hashes are in the commit message of "Add third-party packages at upstream versions".
