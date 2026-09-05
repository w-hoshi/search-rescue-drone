# Search and Rescue Drone: LiDAR SLAM and Person Detection Payload

A low-cost drone payload for mapping in GPS-denied environments. Runs 2D LiDAR SLAM and YOLOv8 person detection onboard a Raspberry Pi 5. Senior design project, ME 461, Boston University.

**[Demo video: mapping during flight](https://youtu.be/b8vK8jm33Qs)** — 2D occupancy grid building in real time (bottom right) as the drone flies an indoor corridor.

Commercial SLAM-capable rescue drones start around $15k. The goal was to see how much of the core capability, indoor mapping without GPS plus onboard person detection, could be built from off-the-shelf parts and open-source ROS 2 packages under a $400 budget. The payload bolts onto an existing airframe and does not fly the drone.

| | |
|---|---|
| Compute | Raspberry Pi 5 |
| Mapping | `slam_toolbox`, 2D occupancy grid |
| Odometry | `rf2o_laser_odometry` (laser scan matching) |
| Detection | YOLOv8 via `yolo_ros` |
| Payload weight | 0.76 kg |
| Runtime | 31 min |
| Payload cost | ~$346 |
| Mapping error | 22.4 cm average wall-length error vs tape measure |

## Architecture

```
DFR0315 2D LiDAR ──> laser_filters ──> rf2o_laser_odometry ──> /odom
                            │                                    │
                            └────────────> slam_toolbox <─────────┘
                                                │
                                                ├──> /map (occupancy grid)
                                                └──> tf: map -> odom -> base_link

ELP USB camera ──> yolo_ros (YOLOv8) ──> /yolo/detections ──> [person_mapper]
```

Two things the diagram makes easy to misread:

**Localization is LiDAR-only.** Odometry comes from scan matching in `rf2o_laser_odometry`. The MPU-6050 IMU is wired and readable, and I tried several approaches to using it for drift correction, but none beat plain scan matching in our test environment. The final flight configuration does not fuse IMU data. The camera is monocular, so there is no visual SLAM and no depth.

**Detection and mapping ran as separate subsystems.** Bounding boxes were produced in real time during flight but never placed onto the occupancy grid. The node that would do that, `person_mapper`, is in this repo and was never run.

## Repository layout

```
src/
├── rcdrone_slam/          SLAM launch + configs (slam_toolbox, rf2o, laser filter)
├── offboard_control/      Basic PX4 offboard flight tests
└── person_mapper/         Detection-to-map projection (written, never validated)
docs/
├── frames_*.pdf           tf tree dumps from view_frames at three test stages
└── cad/                   Payload housing (PETG, 3D printed)
```

## Build and run

The repo root is a colcon workspace. Four third-party packages are not vendored and need cloning into `src/` first: `laser_filters`, `rf2o_laser_odometry`, `ros2_mpu6050_driver`, `yolo_ros`.

```bash
git clone https://github.com/w-hoshi/search-rescue-drone.git
cd search-rescue-drone/src
# clone the four dependencies here
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch rcdrone_slam mapping.launch.py
```

YOLO weights are not in the repo. We ran `yolov8n`; `yolov8m` was too slow on the Pi 5 to be useful at frame rate. Visualize on a ground station rather than on the Pi, since RViz competes with YOLO for CPU.

## Results

**Mapping worked.** Corridor geometry came out recognizable: straight wall runs, corner junctions, correct hallway topology. Wall segment lengths in the generated map differed from tape measurements of the actual corridor by 22.4 cm on average. Distortion concentrated at corners and at the far end of long runs, which is what accumulated scan-matching drift looks like.

The map also contains phantom wall segments where the corridor has windows. Glass either passes the beam or reflects it off-axis, so range readings there are inconsistent and `slam_toolbox` writes structure that isn't real. Visible in the demo video.

**The drone was overweight.** Battery, payload, and electronics put the F450 near its practical limit, and handling suffered. This shortened useful test runs and limited how the system could be flown.

**Autonomy was dropped.** `offboard_control` contains working PX4 offboard primitives (`takeoff_to_1m`, `forward_for_2s`) and nothing beyond them. Localization was not reliable enough to trust a closed-loop indoor mission, and the test environment's lighting made it worse. All demo flights were manually piloted.

## person_mapper: implemented, never validated

This is the piece that would have made the system do what it claims: put detected people onto the map. It is fully written and was never run against live SLAM and detection. Not a stub, just never integrated into a launch file and never tested.

The approach, given a monocular camera and no depth sensor:

1. Take the bounding box center pixel from a detection.
2. Back-project it to a ray in the camera frame using the intrinsics.
3. Rotate the ray into the map frame, composing the fixed 25 degree camera tilt with the drone's yaw from the `map -> base_link` transform.
4. Intersect that ray with the ground plane (z = 0) for an (x, y) map coordinate. This substitutes for depth: assume the person is on the floor and altitude is known.
5. Associate against previously seen people so one person doesn't become a trail of markers.
6. Publish a cylinder marker per person for RViz.

Issues I've since identified and have not fixed, since there are no recorded rosbags to test against:

- **Marker lifetime is 3 seconds and only current-frame detections are published.** A person would vanish from the map once the drone flew past, which defeats the purpose. Needs infinite lifetime and republishing of accumulated positions.
- **The track ID check is wrong.** `Detection.id` is a string and is empty rather than absent when tracking is off, so the `is not None` test always passes and every detection collapses onto one key. The proximity fallback never runs.
- **Projecting the bbox center biases range long.** Ground-plane intersection assumes the projected pixel is where the person contacts the floor. The torso center sits above that, so estimated positions land farther out than the truth. Systematic, not noise. Should project the bottom edge of the box.
- **Roll and pitch are discarded.** The full quaternion is read but only yaw is used. A few degrees of pitch in flight moves the ground intersection substantially.
- **Altitude is a fixed parameter.** 2D LiDAR SLAM gives no useful z estimate, so altitude is hardcoded rather than measured.

Multi-view triangulation would be the better approach than a ground-plane assumption: accumulate detections of the same person from several known drone poses and triangulate. That removes the flat-floor and fixed-altitude assumptions, both of which are bad in the environments this was meant for.


## Hardware

| Component | Part | Notes |
|---|---|---|
| Compute | Raspberry Pi 5 | USB 3.0 for LiDAR and camera, I2C for IMU |
| LiDAR | DFR0315 | 360°, ~8000 samples/sec, 5.5–10 Hz |
| Camera | ELP 48MP USB | Mounted at 25° below horizontal |
| IMU | MPU-6050 | I2C, near payload center of mass |
| Power | 3S LiPo into U6223 buck converter | Regulated to 5V |
| Housing | PETG, 3D printed | CAD in `docs/cad/` |
| Airframe | HAWK'S WORK F450 | Test platform, not part of the payload |

Cylindrical PETG body with a detachable angled front face carrying the camera, a bottom cutout giving the LiDAR unobstructed 360° coverage, and a top plate bolted to the F450 arms. The 25 degree camera tilt is fixed in the print, which is what makes the camera-to-body rotation in `person_mapper` a constant.

`docs/frames_*.pdf` are `view_frames` dumps from three points in testing, showing the tf tree that was actually running.

## Team and attribution

ME 461 senior design, Boston University. Team 248: Wataru Hoshi, Justen Li, Jonathan Thea, Vincent Lin. Advised by Prof. Geiger, with Prof. Tron as technical advisor.

My work: sensor selection (LiDAR and IMU), payload housing design and 3D printing, mechanical integration and mounting, electrical assembly with Jonathan Thea, running and tuning the SLAM pipeline, IMU drift investigation, and the `person_mapper` node. Justen Li set up the detection node.

The project concluded with the course and the hardware was returned to the school, so this repo is a record of the work rather than an active project.

## License

MIT
