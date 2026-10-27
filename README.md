# ROS2 bridge for CARLA simulator v0.9.15

This ROS package is a modified fork of the [carla-simulator/ros-bridge](https://github.com/carla-simulator/ros-bridge) package that is adopted to work with ROS2 Humble running on Ubuntu 22.04 LTS with Scenario Runner v0.9.15. The ROS bridge enables two-way communication between ROS and CARLA. The information from the CARLA server is translated to ROS topics. In the same way, the messages sent between nodes in ROS get translated to commands to be applied in CARLA.

## eagletrt additions

On top of upstream, this fork adds what the perception stack needs for evaluation
(see the [perception workspace README](https://github.com/eagletrt/perception-ws-sw)).

**World frame.** The CARLA world TF frame is the `world_frame` parameter, default
`carla_world` (upstream hardcodes `map`). `map` is reserved for the perception
stack's own world, so the two never clash.

**Ground truth pseudo-sensors**, spawned with `ros2 run carla_ros_bridge spawn_ground_truth`
once the bridge and the ego vehicle are up:

| Blueprint | Publishes |
|---|---|
| `sensor.pseudo.ground_truth` (attached to the ego vehicle) | `/ground_truth/odometry` (`nav_msgs/Odometry`, `carla_world` → `gt/base_link`), TF `carla_world → gt/base_link`, static TF `gt/base_link → <role name>` |
| `sensor.pseudo.cone_ground_truth` | `/ground_truth/cone_map` (`perception_interfaces/ConeArray` in `carla_world`; red cones = `BIG_ORANGE`) |

`gt/base_link` is the CARLA vehicle origin.

**Anchoring the perception map.** The ground truth sensor listens to
`/perception/status`. When a perception stack reports that its map started at
time t (`map_origin_stamp`), the sensor looks up the vehicle's ground truth pose at
t (from a 120 s history, interpolated) and publishes it as static TF
`carla_world → map`. A new `map_id` re-anchors. The perception stack itself never
sees ground truth.

**RGB LiDAR pseudo-sensor** `sensor.pseudo.rgb_lidar`, spawned with
`ros2 run carla_ros_bridge spawn_rgb_lidar` once the bridge and the ego vehicle are up
(`--ros-args -p host:=<carla host> -p port:=<port> -p timeout:=<s>`, default `localhost:2000`, 5 s).
It reads a rig mounted on the vehicle (carla-sim-sw setup `microlino/rgb_lidar`): a
`sensor.lidar.ray_cast` with role name `rgb_lidar` and `sensor.camera.rgb` cameras with role
names `rgb_lidar_cam_*`, all with the same `sensor_tick`, in one of two modes:

- `sensor_tick: 0`: the rig captures at every tick and a turn takes several ticks (slices);
  `1 / (rotation_frequency * fixed_delta_seconds)` must be an integer.
- `sensor_tick > 0`: one whole turn per capture, with `rotation_frequency = 1 / fixed_delta_seconds`.
  `microlino/rgb_lidar` uses this: `sensor_tick: 0.1`, `rotation_frequency: 100` with
  `fixed_delta_seconds: 0.01`, so one turn every 0.1 s.

The bridge does not register the rig sensors itself (no image topics). Every LiDAR
slice is colored with the camera images of the same frame (pinhole projection, the camera
whose optical axis is closest), and a full turn is published as one message:

| Topic | Content |
|---|---|
| `/carla/hero/rgb_lidar` | `sensor_msgs/PointCloud2`, frame `hero/rgb_lidar`, stamped at the start of the turn, unorganized (`height` 1); fields and byte layout of the Ouster driver's `color_point` (below) |
| TF `carla_world → hero/rgb_lidar` | LiDAR pose at every slice (every capture), stamped at the slice |

The cloud matches `ouster-ros` (>= 0.15.2) with `point_type: color_point` and `organized: false`
(`ouster_ros::ColorPoint` through `pcl::toROSMsg`, `point_step` 48), so the same code reads the
sim and a Rev8 RGB sensor. Configure the real driver with `organized: false`: its default (`true`)
publishes a `beams × columns` grid with NaN for the missing returns (here 128 × 1024, ~35k NaN per
turn), while this cloud, as Ouster's unorganized one, lists only the valid points (`is_dense` true).

| Field | Offset | Type | Content |
|---|---|---|---|
| `x`, `y`, `z` | 0, 4, 8 | float32 | m, right-handed LiDAR frame of the point's slice |
| `intensity` | 16 | float32 | CARLA's 0–1 attenuation, not Ouster's photon count |
| `t` | 20 | uint32 | ns since the stamp, see below |
| `reflectivity` | 24 | uint8 | 0, not modeled |
| `ring` | 26 | uint16 | LiDAR channel, 0 = top beam (`upper_fov`) as Ouster's row 0 |
| `ambient` | 28 | uint16 | 0, not modeled |
| `range` | 32 | uint32 | mm, `sqrt(x² + y² + z²)` |
| `rgb` | 36 | uint32 | `0x00RRGGBB`; 0 for points no camera sees, as Ouster |

**Timing and deskewing.** As with Ouster, the stamp is the nominal start of the turn and `t` the
time since it: `t = (slice index + azimuth fraction of the slice sweep) · fixed_delta_seconds`.
Each slice stays in the LiDAR frame of its own tick (no motion compensation). CARLA casts all
the rays of a tick from a single sensor pose (`RayCastSemanticLidar::SimulateLidar`), so inside a
slice `t` is synthetic: it follows the sweep like a real sensor, but the points were all taken at
the start of the slice. Deskewing with `t` against the per-slice TF (or odometry) is therefore
exact between slices and off by up to one tick inside a slice: up to `v · dt` in translation and
`ω · dt · range` from yaw (15 cm at 15 m/s with `dt` = 0.01 s, 10 cm at 20 m with 0.5 rad/s).
For an exact deskew in the sim, use the slice time `floor(t / dt) · dt`.
With `sensor_tick > 0` (one whole turn per tick) the turn is a single slice: `t` spans one
`fixed_delta_seconds` (0–10 ms at 0.01 s) and not the publish period, and the cloud needs no deskewing.

- A turn missing slices (frames dropped before the cameras completed them) is published anyway,
  as Ouster publishes incomplete scans, with a warning; its stamp and `t` keep the turn timing.
- About 4.5 MB per turn (~96k points).
- The Fast DDS profile `carla_ros_bridge/config/fastdds_large_data.xml` is loaded by default in the
  bridge (`carla_ros_bridge.launch.py`, argument `fastdds_profile`; empty = Fast DDS defaults) and in
  every process of `pointcloud_monitor.launch.py`. Subscribers do not need it: the writer decides
  how a message travels, and a subscriber with the default Fast DDS transports (which include
  shared memory) gets whole turns from the bridge (measured, reliable and best effort). To load it
  anyway: `export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix carla_ros_bridge)/share/carla_ros_bridge/config/fastdds_large_data.xml`.
  When the bridge runs without it, with the default transports each turn travels as ~64 KB
  fragments: under load a reader misses some, and reliable in-order delivery then holds the following
  turns until the repair, so they arrive in bursts up to ~1 s late, while a best effort reader
  (`SensorDataQoS`) drops every turn missing a fragment (18% on an idle machine, about half
  under load). The profile uses shared memory sized for whole turns (same host only): reliable and
  best effort readers then get every turn.
- Real-time factor: with `sensor_tick: 0` the cameras render at every tick and the simulation runs
  at about 0.25x real time; `microlino/rgb_lidar` (`sensor_tick: 0.1`) runs at about 0.9x.
  Consumers must use `use_sim_time:=true`.
- `ros2 launch pointcloud_monitor pointcloud_monitor.launch.py` starts the bridge in passive
  mode, spawns the pseudo-sensor and shows the flow (rate, latency, gaps), the message structure
  and the cloud in rviz. Unit tests (no CARLA server needed), with the workspace sourced:
  `python3 -m pytest carla_ros_bridge/test/test_rgb_lidar.py`.

**Notes**
- Do not spawn `sensor.pseudo.tf` next to the ground truth sensor: both would give
  the vehicle frame a parent.
- In passive mode the bridge publishes sensor TF directly from `carla_world`
  instead of from the vehicle frame; check that `<role name> → <role name>/<camera>`
  resolves for the perception stack's camera mount lookup.
- `lap_monitor`, `record_evaluation` and `evaluate_run` still use the previous
  topics (`/carla/autopilot/...`, `/slam/*`) and are due to be rewritten.

## Main Requirements

- OS: Ubuntu 22.04 LTS
- CARLA Version: 0.9.15
- Scenario Runner Version: 0.9.15
- ROS Version: Humble
- **NOTE**: All testing were peformed using Python 3.10. The default CARLA PythonAPI only supports Python 2.7 and 3.7 (and 3.8 by extension). Updated `.whl` and `.egg` files for Python 3.10 can be found at [https://github.com/gezp/carla_ros/releases/](https://github.com/gezp/carla_ros/releases/).
- > **⚠️ UPDATE (07/26)**: Starting with v0.9.16, CARLA now supports a natively integrated ros bridge. I've moved on to the native ros bridge, but if your workflow still involves a 3rd party ros bridge, try using newer `.whl` files for Python 3.10, 3.12, 3.14 found at [https://github.com/ttgamage/carla-whl-builder/releases](https://github.com/ttgamage/carla-whl-builder/releases). There are a few minor differences in more recent ROS2 versions but it's possible to run carla ros bridge on newer OS releases/python versions with a bit of an effort. `podman/docker` is your friend. :)
### Screenshot of Carla AD Demo in Action 

![rviz setup](./docs/images/ad_demo.png "AD Demo")

## Instructions (adapted from [ROS Bridge Documentation](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/))
1. Set up a project directory and clone the ROS bridge repository and submodules:
```
mkdir -p ~/Workspace/ros-bridge && cd ~/Workspace/ros-bridge
git clone --recurse-submodules https://github.com/ttgamage/carla-ros-bridge.git
mv carla-ros-bridge src
```
2. Set up ROS environment and install dependencies:
```
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r
```
3. Build the ROS bridge workspace using colcon:
```
colcon build --symlink-install
```


