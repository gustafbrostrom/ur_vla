# UR10 Simulation (Stage 1): ROS 2 Jazzy + Gazebo Harmonic

Dockerized UR10 simulation using ROS 2 Jazzy and Gazebo Harmonic with **gz_ros2_control**. One command starts Gazebo, spawns a UR10 in a simple world (ground + light), and runs `joint_state_broadcaster`, `joint_trajectory_controller`, and `gripper_action_controller`. A wrist camera and a **Robotiq 2F-85** gripper (PickNik model) on the tool flange are included. No ROS or Gazebo on the host required; GUI over X11.

## Requirements

- Docker and Docker Compose
- X11 (for Gazebo GUI)
- Host: Debian or any Linux with X11 (ROS/Gazebo not required on host)

## Quick start

### Build

The image fetches the vendored **ros2_robotiq_gripper** repo (vcstool) and builds it with the workspace. First build may take longer while cloning and building `robotiq_description`.

```bash
docker compose build
```

Or with plain Docker:

```bash
docker build -t ur10_gz:latest -f docker/Dockerfile .
```

### Run simulation

**With GUI (X11):**

1. Allow the Docker daemon to connect to your X server (run once per session):

   ```bash
   xhost +local:docker
   ```

   Or, less restrictive: `xhost +local:`

2. Start the simulation:

   ```bash
   docker compose up
   ```

   Gazebo Harmonic should open with the UR10 in a simple world. After the robot is spawned, `joint_state_broadcaster`, `joint_trajectory_controller`, and `gripper_action_controller` are loaded automatically.

   If you see **`qt.qpa.xcb: could not connect to display :0`** or **Could not load the Qt platform plugin "xcb"**, the container cannot reach your X server. Either run `xhost +local:docker` (and ensure `DISPLAY` is set), or run headless (below).

**Without GUI (headless):**

If you don’t have a display or X11 isn’t working, run the simulation without the Gazebo window (server only). The sim and ROS 2 control (e.g. `joint_trajectory_controller`) still work:

```bash
GZ_GUI=false docker compose up
```

Or set `GZ_GUI=false` in a `.env` file next to `compose.yaml`, then run `docker compose up`.

### Test JointTrajectory (move the robot)

In another terminal, run a one-off container with the same image and send a trajectory goal:

```bash
docker compose run --rm ur10_sim bash -c "
  . /opt/ros/jazzy/setup.bash && . /home/ros/ws/install/setup.bash &&
  ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory \
    \"{
      trajectory: {
        joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
        points: [
          { positions: [0.0, -0.5, 0.5, 0.0, 0.5, 0.0], time_from_start: { sec: 2, nanosec: 0 } }
        ]
      }
    }\"
"
```

Or exec into the running container and run the same `ros2 action send_goal` after sourcing the workspace.

### Robotiq 2F-85 gripper (open/close)

The sim includes a **Robotiq 2F-85** gripper on `tool0` (PickNik model). The driven joint is `robotiq_85_left_knuckle_joint` (revolute, open ≈ 0 rad, closed ≈ 0.7929 rad). After the sim is up and `gripper_action_controller` is loaded, open and close via the `GripperCommand` action:

**Open (position 0.0 rad):**
```bash
ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 0.0}}"
```

**Close (position 0.7929 rad):**
```bash
ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.7929, max_effort: 0.0}}"
```

**Demo script (inside container):**
```bash
./scripts/demo_gripper.sh open    # or close, or both
```

From the host (with sim running):
```bash
docker compose exec ur10_sim bash -c ". /opt/ros/jazzy/setup.bash && . /home/ros/ws/install/setup.bash && ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand \"{command: {position: 0.0, max_effort: 0.0}}\""
```

**Verify gripper joint state:** `ros2 topic echo /joint_states --once` — confirm `robotiq_85_left_knuckle_joint` is present; its `position` should change after open/close (≈ 0 open, ≈ 0.7929 closed).

**TF (gripper):** `tool0` → `ur_to_robotiq_link` → `robotiq_85_adapter_mount` → `robotiq_85_base_link`.

## Testing

Integration tests (launch_testing) verify that the robot spawns, the wrist camera publishes images, and the robot moves on a trajectory command. Tests use the environment’s `GZ_GUI` (default true), so with a display and `xhost +local:docker` the Gazebo window may open; they take about 1–2 minutes.

From the repo root, build the image, then run tests inside the container:

```bash
docker compose build
docker compose run --rm ur10_sim bash -c "
  . /opt/ros/jazzy/setup.bash && . /home/ros/ws/install/setup.bash &&
  colcon test --packages-select ur10_gz_bringup &&
  colcon test-result --all
"
```

To see the test log in the terminal (debug prints, robot start/end positions, image size, etc.), add `--event-handlers console_direct+` so output is streamed instead of captured:

```bash
docker compose run --rm ur10_sim bash -c "
  . /opt/ros/jazzy/setup.bash && . /home/ros/ws/install/setup.bash &&
  colcon test --packages-select ur10_gz_bringup --event-handlers console_direct+
"
```

**Test logs to file (read outside the container)**  
The compose file mounts `./test_logs` into the container at `/home/ros/ws/test_logs`. Easiest: run the script so the directory exists, is writable by the container user, and logs go to the right place:

```bash
./scripts/run_tests_with_logs.sh
```

Then open `test_logs/ur10_gz_bringup_test.log` (and `test_logs/Testing/Temporary/LastTest.log`, `test_logs/test_results/` if copied) on your machine. The script creates `test_logs`, sets it world-writable so the container user can write, and uses absolute paths inside the container. The directory `test_logs/` is in `.gitignore`.

Manual run (from repo root; ensure `test_logs` exists and is writable by the container user, e.g. `chmod 777 test_logs`):

```bash
mkdir -p test_logs && chmod 777 test_logs
docker compose run --rm ur10_sim bash -c "
  . /opt/ros/jazzy/setup.bash && . /home/ros/ws/install/setup.bash &&
  mkdir -p /home/ros/ws/test_logs &&
  colcon test --packages-select ur10_gz_bringup --event-handlers console_direct+ 2>&1 | tee /home/ros/ws/test_logs/ur10_gz_bringup_test.log &&
  colcon test-result --all &&
  cp -r build/ur10_gz_bringup/Testing build/ur10_gz_bringup/test_results /home/ros/ws/test_logs/ 2>/dev/null || true
"
```

For headless/CI (no display), set `GZ_GUI=false` before the test, e.g. `GZ_GUI=false docker compose run --rm ur10_sim bash -c "..."`.

To run tests without rebuilding (e.g. after changing only test code), mount the workspace and run the same `colcon test` and `colcon test-result` commands after building the workspace once.

## Repository structure

```
.
├── docker/
│   ├── Dockerfile
│   └── entrypoint.sh
├── compose.yaml
├── ur_vla.repos              # vcstool: fetches ros2_robotiq_gripper into src/
├── src/
│   ├── ur10_description/     # UR10 URDF/xacro + gz_ros2_control + Robotiq 2F-85
│   └── ur10_gz_bringup/     # Launch, controllers.yaml, world
├── scripts/
│   ├── demo_gripper.sh      # Open/close gripper demo
│   └── run_tests_with_logs.sh
├── README.md
```

## Troubleshooting

### X11 / GUI not showing

- **Cause:** The container cannot connect to your X server.
- **Symptoms:** `qt.qpa.xcb: could not connect to display :0`, `Could not load the Qt platform plugin "xcb"`, or “cannot open display”.
- **Fix:**
  1. On the host, run: `xhost +local:docker` (or `xhost +local:`). This allows the Docker client to use your display. Do this **before** `docker compose up`.
  2. Ensure `DISPLAY` is set in the container. The compose file sets `DISPLAY: ${DISPLAY:-:0}`; if your display is different (e.g. `:1`), set `DISPLAY` in the environment before `docker compose up`.
  3. Check that `/tmp/.X11-unix` is mounted: `docker compose run --rm ur10_sim env | grep DISPLAY`.
  4. **Alternative:** Run headless with `GZ_GUI=false docker compose up` (no Gazebo window; sim and ROS 2 control still work).

### DISPLAY issues

- **Cause:** Wrong or unset `DISPLAY` inside the container.
- **Fix:**
  - The compose file uses `DISPLAY: ${DISPLAY:-:0}`. If your host uses another display (e.g. `:1`), run: `export DISPLAY=:1` before `docker compose up`, or set it in a `.env` or in the compose `environment` section.
  - Ensure the X11 socket is mounted: volume `/tmp/.X11-unix:/tmp/.X11-unix` in `compose.yaml`.

### Gazebo plugin load failures

If the gz_ros2_control plugin fails to load or the robot does not move:

1. **Plugin name and library:** Confirm the URDF uses:
   - `filename="libgz_ros2_control-system.so"`
   - `name="gz_ros2_control::GazeboSimROS2ControlPlugin"`
2. **Controller YAML path:** The plugin’s `<parameters>` must point to the installed `controllers.yaml` (e.g. `$(find ur10_gz_bringup)/config/controllers.yaml`). After `colcon build`, this path is valid inside the container.
3. **Joint names:** The `<ros2_control>` joint names must match `controllers.yaml` and the URDF: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`, `robotiq_85_left_knuckle_joint`.
4. **Spawn order:** Controllers are spawned in order: `joint_state_broadcaster`, then `joint_trajectory_controller`, then `gripper_action_controller`. If a controller is not active, check that the spawners ran (see launch logs).

### Gazebo not starting / nothing spawned (MESA, DRM, iris driver)

- **Cause:** In Docker (or without GPU passthrough), the Gazebo server may fail to use the GPU and never advertise `/world/default/create`, so the robot never spawns. You may see: `MESA: error: Failed to query drm device`, `failed to load driver: iris`, and `Waiting for service [/world/default/create]`.
- **Fix:** The compose file sets `LIBGL_ALWAYS_SOFTWARE=1` so Gazebo uses software OpenGL and can start without a real GPU. If you run the launch outside Docker and hit similar errors, set `export LIBGL_ALWAYS_SOFTWARE=1` before launching.

### Gripper / mimic joints

- **Gripper does not move or shows errors:** Ensure `robotiq_85_left_knuckle_joint` is the only gripper joint with a command interface in `<ros2_control>`. Joint names in URDF and `controllers.yaml` must match.
- **Mimic joints:** The Robotiq 2F-85 has one driven joint (`robotiq_85_left_knuckle_joint`); the right knuckle and others are mimic. In gz_ros2_control, mimic joints must **not** have command interfaces; only state interfaces are allowed. Do not add `<command_interface>` for mimic joints.
- **“Physics engine does not support mimic constraints”:** If Gazebo logs this ([gz_ros2_control issue #340](https://github.com/ros-controls/gz_ros2_control/issues/340)), the physics engine in use does not enforce mimic constraints, so only the left knuckle is driven and the right finger may not follow. **Options:** (1) Switch the world to a physics engine that supports mimic constraints, if available for Gazebo Harmonic. (2) **Fallback:** Add `robotiq_85_right_knuckle_joint` to `<ros2_control>` with a position command interface and drive both joints in software (e.g. a small node that publishes `-left_position` to the right joint, or use `joint_trajectory_controller` with both joints and mirrored setpoints).
- **Action never succeeds:** Try `allow_stalling: true` in `gripper_action_controller` (in `controllers.yaml`) and/or increase `goal_tolerance`. Confirm joint range (0–0.7929 rad) matches the commanded position.

## Wrist Camera

A wrist-mounted RGB camera is attached to the UR10 tool flange (`tool0`) and publishes images to ROS 2.

### Topic names

- **Image:** `/wrist_camera/image_raw` (`sensor_msgs/msg/Image`)
- **CameraInfo:** `/wrist_camera/camera_info` (`sensor_msgs/msg/CameraInfo`)

### Frame names

- **TF:** `tool0` → `wrist_camera_link` (fixed joint, 2 cm along tool0 Z+ forward).
- **Image/CameraInfo header:** `frame_id` is `wrist_camera_link`.

### How to visualize the image

- **rqt_image_view:** Run `rqt_image_view` and select `/wrist_camera/image_raw`.
- **RViz:** Add an “Image” display and set the topic to `/wrist_camera/image_raw`.

### Common troubleshooting (wrist camera)

- **No image / topic not listed:** The world uses the **ogre** render engine (not ogre2) for the Sensors plugin to improve compatibility with software rendering. Two ROS topics are bridged: `/wrist_camera/image_raw` (camera on `wrist_camera_link`) and `/wrist_camera/image_raw_tool0` (camera on `tool0` if the link was lumped). **Check both:** `ros2 topic echo /wrist_camera/image_raw --once` and `ros2 topic echo /wrist_camera/image_raw_tool0 --once`; use whichever has data (e.g. in rqt_image_view). If neither has data, ensure the simulation is playing (Gazebo play button) and list Gazebo topics: `gz topic -l` and look for `.../sensor/wrist_camera/image`. With software rendering, if neither topic ever gets frames, try running without `LIBGL_ALWAYS_SOFTWARE=1` (e.g. GPU passthrough) if you need the camera.
- **Wrong frame in header:** If you use a `tf_prefix`, the camera link is `${tf_prefix}wrist_camera_link`; ensure your tools use the same prefix. Check `ros2 topic echo /wrist_camera/image_raw --field header`.
- **Black image:** Check world lighting (directional light in the world). Ensure the simulation is running (play button in Gazebo). The camera uses the `ogre2` render engine; if running headless, the Sensors system may still produce images depending on your setup.

## References

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
- [Gazebo Harmonic ROS 2 integration](https://gazebosim.org/docs/harmonic/ros2_integration)
- [ros2_control](https://control.ros.org/master/index.html)
- [gz_ros2_control](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
- [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
