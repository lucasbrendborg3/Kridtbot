# Kridtbot

ROS 2 Jazzy workspace for **Kridtbot**, a four-wheeled weeding rover with a height-adjustable
suspension and a robot arm. The repository is the whole colcon workspace: clone it as `~/ros_ws`.

## Setup

```bash
git clone https://github.com/lucasbrendborg3/Kridtbot.git ~/ros_ws
cd ~/ros_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

The arm software lives in a separate workspace (`~/robot_arm_ws`, packages `robot_arm_description` and
`robot_arm_interfaces`). It is only needed for `main.launch.py` and `main_weeding`.

The rover expects the CAN interfaces `can_l` and `can_r` (wheels) and `can_odrive` (suspension) to be up.

## Running

```bash
# Rover with suspension. The legs must be homed first (see below).
ros2 launch kridtbot launch_kridtbot.launch.py use_odrive:=true

# Rover + arm + stop-and-go weeding mission (suspension not used)
source ~/robot_arm_ws/install/setup.bash && source ~/ros_ws/install/setup.bash
ros2 launch kridtbot main.launch.py
```

Driving with the joystick needs the enable button held.

## Suspension homing: after every ODrive power-up

The suspension legs only know where they are after homing to their bottom end stop:

```bash
ros2 run kridtbot calibrate_suspension     # with the launch stopped
```

With `use_odrive:=true` the launch refuses to start if a leg has not been homed. On Ctrl+C the legs
are driven down to their lower limit before the rover shuts down. The top end stop is never searched
for, because driving into it can unscrew the ball screw.

## Repository layout

- `src/kridtbot`: robot description, launch files, controller config, helper nodes, scripts.
- `src/main_weeding`: weeding mission node.
- `src/suspension_controller`, `src/bno055_hardware_interface`: suspension levelling controller and IMU hardware.
- Drivers and other third-party packages, several with our own changes: see [THIRD_PARTY.md](THIRD_PARTY.md).
- [CLAUDE.md](CLAUDE.md): detailed notes on how the workspace fits together.

Old SLAM maps and pose graphs are kept in git history only. Restore them with:

```bash
git checkout maps-archive -- maps/
```
