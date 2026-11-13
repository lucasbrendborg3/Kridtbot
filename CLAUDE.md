# kridtbot ROS 2 workspace

Software for **kridtbot**, a four-wheeled weeding rover with an active (height-adjustable) suspension.
ROS 2 **Jazzy**, built with colcon. This machine (`gunnar-MP200`) is the rover's on-board computer: the
CAN interfaces are real hardware, so launching things moves motors.

## Build and run

```bash
source /opt/ros/jazzy/setup.bash
cd ~/kridtbot_ws && colcon build --symlink-install --packages-select <pkg>   # build only what you changed
source install/setup.bash
ros2 launch kridtbot launch_kridtbot.launch.py use_odrive:=true          # rover with suspension
ros2 launch kridtbot main.launch.py                                      # rover + arm + weeding mission
```

`main.launch.py` also needs the arm workspace sourced first (`~/robot_arm_ws`, packages
`robot_arm_description`, `robot_arm_interfaces`). It launches the rover with `use_odrive:=false`.

## Packages in `src/`

Own code (edit freely):
- **kridtbot**: robot description, launch files, controller config, helper nodes and scripts. The main package.
- **main_weeding**: Python `mission` node. Stop-and-go: drive `step_distance` on `/diff_cont/odom`, stop, call the arm's `/arm/remove_weed` action, repeat. Publishes on `/cmd_vel`.
- **suspension_controller**: ros2_control controller plugin that levels the chassis from the IMU (PID/LQR/IK modes, `control_mode` in `controller_manager.yaml`). Upstream: nich1157/suspension_controller (teammate).
- **bno055_hardware_interface**: ros2_control hardware plugin for the BNO055 IMU.
- **linear_actuator_calibration**: older Python calibration sequence; superseded by `kridtbot/scripts/calibrate_suspension.py`.

Third-party, but locally modified (the commit "Kridtbot changes to third-party drivers" shows the diff against upstream):
- **ros_odrive**: ODrive CAN driver. `odrive_ros2_control/src/odrive_hardware_interface.cpp` is heavily modified (see Suspension below).
- **myactuator_rmd**, **myactuator_rmd_ros**: drivers and ros2_control hardware for the MyActuator RMD wheel motors.

Third-party, unmodified sensor / SLAM stack: `livox_ros_driver2` + `Livox-SDK2` (Livox MID-360 lidar), `lidarslam_ros2`, `pointcloud_to_laserscan`, `ros-imu-bno055`, `topic_tools`.

## kridtbot package layout

- `description/robot.urdf.xacro`: entry point. Includes `robot_core.xacro` (links/joints), sensors, and the ros2_control xacros below. Xacro args: `use_sim_system`, `active_suspension`, `use_odrive`, `mock_left_wheels`, `mock_right_wheels`.
- `description/ros2_control_myactuator.xacro`: the four wheels. `mock:=true` swaps a side for `mock_components/GenericSystem`.
- `description/ros2_control_odrive.xacro`: the four suspension legs (only when `use_odrive:=true`).
- `description/ros2_control_gz.xacro`: Gazebo version.
- `config/controller_manager.yaml`: all controllers (80 Hz).
- `launch/launch_kridtbot.launch.py`: main rover bring-up (robot_state_publisher via `rsp.launch.py`, ros2_control_node, spawners, joystick, twist_mux, twist_stamper). `use_mec_or_diff` is hard-coded to `'diff'` at the top.
- `nodes/`: C++ helper nodes. `linear_position_pub` (joystick height/tilt), `SwitchController` (L1 toggles joystick height control vs. suspension_controller).
- `scripts/calibrate_suspension.py`: installed as `ros2 run kridtbot calibrate_suspension`.
- `launch discarded/`: old launch files, not installed or used.

## Hardware

| Part | Bus | Details |
| --- | --- | --- |
| Wheels | `can_l`, `can_r` | MyActuator RMD. Right front ID 5, right rear 7 on `can_r`. Left front 4, left rear 6 on `can_l` (direction -1). |
| Suspension legs | `can_odrive` | ODrive, CANSimple. Node 0 left_front, 1 left_rear, 2 right_rear, 3 right_front `_linear`. Ball screw 5 mm per motor turn. |
| IMU | | BNO055, published on `/imu_sensor_broadcaster/imu`. |
| Lidar | Ethernet | Livox MID-360. |

## Driving

- `diff_cont` (DiffDriveController) drives the wheels. `mecanum_drive_controller` is configured but unused.
- Velocity goes through twist_mux: `/cmd_vel_joy` (joystick, priority 100) beats `/cmd_vel` (navigation and mission, priority 10). Then twist_stamper feeds `/diff_cont/cmd_vel`.
- Joystick (`config/joy_params.yaml`): enable button 6, turbo 7. Driving requires holding enable.

## Suspension

- Joint positions are metres above the leg's **bottom end stop** (0 = fully down). The ODrive counts from wherever it powered up, so **run `ros2 run kridtbot calibrate_suspension` after every ODrive power-up**, with the launch stopped (it refuses to run while `ros2_control_node` is up).
- **Only the bottom is ever homed.** Searching for the top end stop could unscrew the ball screw, so the script has no top search. Do not add one.
- Results go to `~/.ros/kridtbot/suspension_limits.yaml`. The ODrive hardware plugin reads it (`limits_file` param) and clamps every position command:
  - Lower limit = `hard_min_m` + `margin_mm` (default 3 mm). There is no fallback.
  - With `use_odrive:=true`, `launch_kridtbot.launch.py` checks the file before starting anything. If any leg lacks `hard_min_m`, the launch aborts with an error telling you to run calibrate_suspension. As a second guard, the ODrive plugin's `on_init` also fails for an unhomed leg. With `use_odrive:=false` there is no check.
  - The check cannot tell whether the homing is from before the last ODrive power-up. Re-homing after power-up is still up to you.
  - Upper limit = the position `max` in `ros2_control_odrive.xacro` (110 mm). It is the only top bound.
- Controllers on the legs: `linear_position_control_joy` (default, driven by `linear_position_pub`), `suspension_controller` (only spawned with `use_odrive:=true active_suspension:=true`), plus `linear_position_control` / `linear_velocity_controller` loaded inactive.
- ODrive plugin parameters (in `ros2_control_odrive.xacro`):
  - `position_input_mode` trap_traj, with `traj_vel_limit` and `traj_accel_limit` in m/s and m/s².
  - `idle_when_unused` false: a leg with no controller holds its position instead of dropping.
  - `park_on_shutdown` true: on Ctrl+C every leg with a homed lower limit is driven down to it at `park_speed`, then held. Legs without one are not moved. `park_timeout` and `park_max_torque` bound the move. The launch gives `ros2_control_node` `sigterm_timeout=15` for this.

## Conventions and gotchas

- `robot_description` is passed as a plain string (`ParameterValue(..., value_type=str)`), because a `: ` in the URDF breaks YAML parsing.
- `suspension_controller` crashes the controller manager if its joints are missing, so it is only spawned when the ODrives are present.
- Comments in this workspace explain *why* (measured torques, observed failures). Keep that style when editing.
- The whole workspace is one git repo (`~/kridtbot_ws`, GitHub lucasbrendborg3/Kridtbot, public). Third-party packages are vendored as plain folders. Their upstream commits are in `THIRD_PARTY.md`. Keep that table up to date when a driver is updated from upstream.
- Old SLAM maps are only in git history, under the tag `maps-archive`.
