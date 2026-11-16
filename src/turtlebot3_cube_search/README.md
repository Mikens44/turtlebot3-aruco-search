# TurtleBot3 Cube Search — Package Reference

This ROS 2 package provides ArUco marker detection, cube localization, and autonomous search using Nav2.

For installation, Docker setup, and step-by-step execution instructions, see the [main README](../../README.md).

## Nodes

The package contains three ROS 2 nodes:

- **`aruco_detector`** — detects ArUco markers in RGB camera images using OpenCV and estimates their poses relative to the camera.
- **`cube_localizer`** — transforms detections into the `map` frame using TF2, filters observations, and maintains estimated cube positions.
- **`cube_search_server`** — generates observation points from the occupancy map, navigates using Nav2, and manages the `/find_cubes` action.

## ROS 2 Interfaces

### Topics

| Topic | Direction | Node | Description |
|---|---|---|---|
| `/camera/image_raw` | Input | `aruco_detector` | RGB camera images |
| `/camera/camera_info` | Input | `aruco_detector` | Camera calibration |
| `/aruco/detections` | Output / Input | Detector / Localizer | Detected marker IDs and poses |
| `/cubes` | Output / Input | Localizer / Search server | Confirmed cube positions in `map` |
| `/map` | Input | `cube_search_server` | Occupancy grid used for observation point generation |
| `/tf`, `/tf_static` | Input | Localizer / Search server | Coordinate transformations |

The detector publishes marker poses in `camera_rgb_optical_frame`. The localizer converts them to the global `map` frame.

### Actions

**`/find_cubes`** — `cube_search_interfaces/action/FindCubes`

Starts an autonomous search for five unique ArUco marker IDs.

The goal accepts `timeout_sec`, which specifies the wall-clock time limit in seconds. A value of `0.0` uses the configured default.

Feedback includes the number of confirmed markers, attempted observation points, total observation points, and the current search phase.

The result contains the detected cubes, a status code, and a message. Successful completion requires five unique marker IDs.

The search server uses the following Nav2 actions:

- `/compute_path_to_pose` — checks whether a route can be planned.
- `/navigate_to_pose` — navigates to an observation point.
- `/spin` — rotates the robot to scan its surroundings.

### Services

**`/cube_localizer/reset`** — `std_srvs/srv/Trigger`

Clears stored cube observations. Use it between missions when the environment or marker positions have changed.

```bash
ros2 service call /cube_localizer/reset std_srvs/srv/Trigger '{}'
```

Do not reset the localizer while a search mission is running.

## Configuration

Default parameters are defined in [`config/search.yaml`](config/search.yaml).

### Marker Detection

| Parameter | Default | Description |
|---|---|---|
| `marker_size` | `0.07` m | Physical side length of the ArUco pattern |
| `optical_frame` | `camera_rgb_optical_frame` | Camera optical coordinate frame |
| `max_reprojection_error` | `2.5` | Maximum accepted reprojection error |
| `min_range` | `0.1` m | Minimum marker detection distance |
| `max_range` | `5.0` m | Maximum marker detection distance |
| `allowed_ids` | `[-1]` | Accepted marker IDs; `[-1]` allows all IDs |

**Important:** In the `arm05_sim` models, the cube side is 9 cm, but the ArUco pattern is approximately 7 cm. Use `marker_size:=0.07` for this environment.

### Cube Localization

| Parameter | Default | Description |
|---|---|---|
| `min_observations` | `5` | Minimum number of observations required to confirm a marker |
| `window_size` | `30` | Maximum number of stored observations per marker |
| `inlier_radius` | `0.15` m | Radius used to identify consistent observations |
| `max_spread` | `0.08` m | Maximum allowed spread of accepted observations |
| `marker_to_cube_offset` | `[0.0, 0.0, 0.0]` | Offset from marker center to reported cube position |

By default, the reported position corresponds to the **marker center**, not the geometric center of the cube.

### Search and Navigation

| Parameter | Default | Description |
|---|---|---|
| `grid_spacing` | `1.0` m | Distance between observation points |
| `robot_radius` | `0.30` m | Robot radius used for obstacle clearance |
| `safety_margin` | `0.10` m | Additional clearance from obstacles |
| `default_timeout` | `600.0` s | Default search time limit |
| `navigation_timeout` | `90.0` s | Time limit for reaching an observation point |
| `spin_timeout` | `45.0` s | Time limit for scanning rotation |

Observation points are generated in reachable free space. The robot visits them, checks routes through Nav2, and performs scanning rotations until five markers are confirmed or the search terminates.

## Launch Arguments

The launch file starts all three nodes:

```bash
ros2 launch turtlebot3_cube_search cube_search.launch.py \
  marker_size:=0.07 \
  optical_frame:=camera_rgb_optical_frame \
  use_sim_time:=true
```

The `marker_size` launch argument is required.

Other launch arguments allow customization of camera topics, map topic, coordinate frames, Nav2 action names, and the parameter file.

The launch file assumes that the robot, camera, map, localization, and Nav2 are already running.

## Troubleshooting

**No markers detected**

Check the camera topics, camera calibration, marker visibility, and `marker_size` parameter.

**Markers detected but not localized**

Verify that TF2 provides a valid transformation from `camera_rgb_optical_frame` to `map`.

```bash
ros2 run tf2_ros tf2_echo map camera_rgb_optical_frame
```

**Robot does not navigate**

Check that the map is available and that Nav2 exposes `/compute_path_to_pose`, `/navigate_to_pose`, and `/spin`.

```bash
ros2 action list -t
```

**Old cube positions remain after restarting a mission**

Stop the active mission and reset the localizer:

```bash
ros2 service call /cube_localizer/reset std_srvs/srv/Trigger '{}'
```

**Localization error is unexpectedly large**

Verify the physical marker size, camera calibration, optical frame, and TF2 transformations.

## Testing

Run the automated tests from the ROS 2 workspace:

```bash
colcon test --packages-select cube_search_interfaces turtlebot3_cube_search
colcon test-result --verbose
```

The last validated test run completed with **48 passing tests**.