# UR10 Simulation (Stage 1): ROS 2 Jazzy + Gazebo Harmonic

Dockerized UR10 simulation using ROS 2 Jazzy and Gazebo Harmonic with **gz_ros2_control**. One command starts Gazebo, spawns a UR10 in a simple world (ground + light), and runs `joint_state_broadcaster` and `joint_trajectory_controller`. No ROS or Gazebo on the host required; GUI over X11.

## Requirements

- Docker and Docker Compose
- X11 (for Gazebo GUI)
- Host: Debian or any Linux with X11 (ROS/Gazebo not required on host)

## Quick start

### Build

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

   Gazebo Harmonic should open with the UR10 in a simple world. After the robot is spawned, `joint_state_broadcaster` and `joint_trajectory_controller` are loaded automatically.

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

## Repository structure

```
.
├── docker/
│   ├── Dockerfile
│   └── entrypoint.sh
├── compose.yaml
├── src/
│   ├── ur10_description/     # UR10 URDF/xacro + gz_ros2_control
│   └── ur10_gz_bringup/      # Launch, controllers.yaml, world
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
3. **Joint names:** The `<ros2_control>` joint names must match `controllers.yaml` and the URDF: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`.
4. **Spawn order:** Controllers are spawned in order: first `joint_state_broadcaster`, then `joint_trajectory_controller`. If the trajectory controller is not active, check that both spawners ran (see launch logs).

### Gazebo not starting / nothing spawned (MESA, DRM, iris driver)

- **Cause:** In Docker (or without GPU passthrough), the Gazebo server may fail to use the GPU and never advertise `/world/default/create`, so the robot never spawns. You may see: `MESA: error: Failed to query drm device`, `failed to load driver: iris`, and `Waiting for service [/world/default/create]`.
- **Fix:** The compose file sets `LIBGL_ALWAYS_SOFTWARE=1` so Gazebo uses software OpenGL and can start without a real GPU. If you run the launch outside Docker and hit similar errors, set `export LIBGL_ALWAYS_SOFTWARE=1` before launching.

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
