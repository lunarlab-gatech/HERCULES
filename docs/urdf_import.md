# Importing a URDF Robot

`hercules_urdf_import` turns a robot description (URDF or xacro) into a HERCULES vehicle:

* the robot **looks like its URDF**: its visual meshes are attached to the vehicle's pawn;
* its **sensors sit where the URDF mounts them**: cameras, lidars, a distance sensor, an IMU,
  GPS, barometer and magnetometer, taken from the URDF's `<gazebo>` sensor blocks and/or a
  sensor file;
* its **ROS 2 TF tree matches the URDF**: `robot_state_publisher` publishes the robot's links,
  and the sensor frames the HERCULES ROS 2 wrapper stamps its data with hang off the right
  links;
* it **drives with HERCULES' own vehicle dynamics**: skid-steer and differential UGVs,
  Ackermann cars and multirotors move like the stock HERCULES vehicles of that kind.

The URDF's collision and inertial elements are not used: the stock pawn keeps its physics. Joints
are frozen at one pose (see [Joints](#joints)).

## Quick start

The importer is a ROS 2 package in `ros2/src/hercules_urdf_import`, but the converter itself is
plain Python 3 with no dependencies, so it also runs on the machine with the Unreal editor:

```bash
# with the ROS 2 workspace built and sourced
ros2 run hercules_urdf_import hercules_urdf_import my_robot.urdf.xacro \
    --name Rover1 --drive skid -o ~/hercules_robots/rover1 \
    --merge-into ros2/settings/hero_blocks_team.json

# or, without ROS (run from ros2/src/hercules_urdf_import; .xacro needs the xacro module)
python3 -m hercules_urdf_import my_robot.urdf --name Rover1 --drive skid -o out/
```

This writes, in the output directory:

| File | What it is for |
|---|---|
| `Rover1.settings.json` | A settings file with the new vehicle (merged into `--merge-into` when given; that input file is not modified). Copy it to `~/Documents/AirSim/settings.json`. |
| `Rover1.visuals.json`, `meshes/*.stl` | The visual manifest and meshes the Unreal plugin builds the robot's look from. |
| `Rover1.urdf` | The robot description for `robot_state_publisher`: the input URDF plus the HERCULES sensor frames, rooted at the vehicle body frame. |

Then start the simulator and, on the ROS 2 side, the bridges plus the robot description:

```bash
ros2 launch airsim_ros_pkgs hercules_hero_team.launch.py
ros2 launch hercules_urdf_import robot_description.launch.py \
    settings:=$HOME/Documents/AirSim/settings.json
```

`robot_description.launch.py` starts one `robot_state_publisher` per vehicle that has a
`Urdf.RobotDescription` in the settings file, in the vehicle's namespace (`/Rover1/robot_description`)
with `frame_prefix` `Rover1/`. Pass `vehicles:=Rover1,Rover2` to pick vehicles and
`use_sim_time:=true` when the bridges publish `/clock`.

## Choosing the dynamics: `--drive` and `--sim-mode`

| `--drive` | Robots | Hero mode (`--sim-mode hero`, default) | Native mode (`--sim-mode native`) |
|---|---|---|---|
| `skid` | skid-steer UGVs (Husky, Jackal, tracked bases) | `PhysXCar` with the `UGVPawn` pawn | `SkidVehicle` mode, `CPHusky` |
| `diff` | differential-drive UGVs | `PhysXCar` with the `UGVPawn` pawn | `SkidVehicle` mode, `Pioneer` |
| `ackermann` | car-like robots | `PhysXCar` (SUV pawn) | `Car` mode, `PhysXCar` |
| `multirotor` | quadrotors and other multirotors | `SimpleFlight` | `Multirotor` mode, `SimpleFlight` |
| `static` | arms, sensor rigs, anything without a drive | not available | `ComputerVision` mode |

Hero mode mixes UAVs and UGVs in one simulation. An articulated robot on a mobile base, such as a
mobile manipulator, takes the drive of its base. `--vehicle-type` and `--pawn-path` override the
table, for example to use your own pawn Blueprint.

## The body frame and the spawn pose

HERCULES places sensors and visuals relative to the pawn's origin, and the ROS wrapper's
`<vehicle>/ground_truth/odom_local` frame is that origin. The importer therefore needs to know
which URDF link the pawn origin corresponds to:

* wheeled drives default to **`base_footprint`** (REP-120: the ground point under the robot),
  because wheeled pawns are modelled with their origin on the ground;
* multirotors and static robots default to **`base_link`**;
* falling back to the URDF's root link. Use `--base-link` to choose another link.

If the robot floats above or sinks into the ground, pick the link that matches the pawn's origin.

`--position X Y Z` and `--yaw DEG` set the spawn pose in the settings convention: meters,
north-east-down relative to the `PlayerStart`, yaw clockwise seen from above.

## Sensors

### From the URDF's `<gazebo>` blocks

| Gazebo sensor `type` | HERCULES sensor |
|---|---|
| `camera`, `wideanglecamera` | camera, `Scene` images |
| `depth` (Classic) / `rgbd_camera` | camera, `Scene` + `DepthPlanar` |
| `depth_camera` | camera, `DepthPlanar` |
| `thermal_camera` | camera, `ThermalIR` |
| `segmentation_camera` | camera, `Segmentation` |
| `multicamera` | one camera per `<camera>` |
| `ray`, `lidar` | CPU lidar (`SensorType` 6) |
| `gpu_ray`, `gpu_lidar` | GPU lidar (`SensorType` 8) |
| a `ray` with a single beam, `sonar` | distance sensor |
| `imu` | IMU |
| `gps`, `navsat` | GPS |
| `magnetometer` | magnetometer |
| `altimeter`, `air_pressure` | barometer |

Image size and horizontal field of view, lidar channels, samples, angular windows, range and
update rate, and the sensor's `<pose>` are carried over. Other sensor types are reported and
skipped. `--no-gazebo-sensors` ignores the `<gazebo>` blocks.

### From a sensor file

`--sensors sensors.yaml` (or `.json`) adds sensors, or overrides same-named Gazebo ones key by
key. Every section is optional; entries can be a list or a mapping by name:

```yaml
cameras:
  - name: front_center          # becomes the HERCULES camera name and ROS frame prefix
    link: zed_left_optical      # URDF link it is fixed to
    frame: optical              # "body" (x forward, default) or "optical" (z forward)
    xyz: [0, 0, 0]              # optional offset in the link frame
    rpy: [0, 0, 0]
    width: 1280
    height: 720
    fov_deg: 90                 # horizontal
    image_types: [Scene, DepthPlanar, Segmentation]
lidars:                         # CPU lidar; use "gpulidars" for the GPU lidar
  - name: os1
    link: os_lidar
    ouster_metadata: os1_metadata.json  # beam table, columns and rate from an Ouster sensor
    range: 120
  - name: roof_lidar
    link: lidar_link
    channels: 32
    range: 100
    rotations_per_second: 10
    measurements_per_cycle: 1024
    horizontal_fov_deg: [-180, 180]   # ROS convention: counter-clockwise from x
    vertical_fov_deg: [-15, 15]
distances:
  - {name: sonar_front, link: sonar_link, min: 0.2, max: 4.0}
imus: [{name: imu}]
gps: [{name: gps}]
barometers: [{name: barometer}]
magnetometers: [{name: magnetometer}]
# remove a sensor the URDF declares:
#   cameras: [{name: rear_cam, enabled: false}]
```

`ouster_metadata` reads the metadata JSON the Ouster SDK and ouster-ros save (current and legacy
layouts): the lidar gets the sensor's calibrated beam elevations and azimuth offsets
(see [Calibrated beam tables](lidar.md#calibrated-beam-tables-ouster)), and its columns and rate from
`lidar_mode`; keys given next to it, such as `rotations_per_second`, still win. Paths in a sensor
file are relative to that file.

Image types are the names of AirLib's `ImageType` (`Scene`, `DepthPlanar`, `DepthPerspective`,
`DepthVis`, `DisparityNormalized`, `Segmentation`, `SurfaceNormals`, `Infrared`, `OpticalFlow`,
`OpticalFlowVis`, `Annotation`, `ThermalIR`, `NightVision`).

### Conventions the importer handles for you

* Poses are converted from the URDF's FLU axes to the settings' FRD axes and Euler angles.
* Lidar azimuth windows are mirrored: the simulator measures azimuth clockwise, ROS
  counter-clockwise. A full circle keeps the simulator's default 0-360 degree window.
* The IMU, GPS, barometer and magnetometer are modelled at the vehicle origin, so their URDF
  link only matters for the warning the importer prints when the IMU is offset or rotated:
  its data is reported in the body frame without lever-arm effects.

## Joints

The simulator draws the robot rigidly, so every movable joint is frozen at one position:

* zero, or the middle of the joint's limits when zero is outside them (what
  `joint_state_publisher` does), with `<mimic>` joints following their leader;
* `--joint NAME=VALUE` (repeatable) to pick another pose, for example a gimbal pitch or an arm
  configuration.

The output URDF turns those joints into fixed joints at the same pose, so the TF tree is exactly
what the simulator renders and no `joint_states` publisher is needed. `--keep-joints` keeps them
movable instead, for when you publish `/<vehicle>/joint_states` yourself; the simulator still
draws the frozen pose. Sensors behind a movable joint are reported, since they are fixed at the
import pose too.

## Meshes and appearance

* Mesh formats: STL (binary and ASCII), Wavefront OBJ (MTL diffuse colors), Collada `.dae`
  (`<unit>`, `<up_axis>`, node transforms and material colors are honored). Box, cylinder and
  sphere primitives are tessellated. Other formats (`.glb`, `.fbx`, ...) are skipped with a warning.
* `package://`, `model://`, `file://` and relative paths are resolved like ROS tools do. When a
  package is not installed, the importer looks in the directories above the URDF; otherwise pass
  `--package-path my_robot_description=/path/to/it`.
* Colors: a mesh's own material colors win, then the URDF `<material>`, then grey. Textures are
  not used.
* The output URDF points its meshes at the resolved files (`file://`), so RViz can show the robot
  even when the description package is not installed; `--keep-mesh-uris` leaves them as they were.
* Large meshes are rebuilt at every simulation start; above 500 000 triangles the importer
  suggests decimating them.

## The `Urdf` settings block

```json
"Rover1": {
  "VehicleType": "PhysXCar",
  "PawnPath": "UGVPawn",
  "Urdf": {
    "Visuals": "/home/me/hercules_robots/rover1/Rover1.visuals.json",
    "RobotDescription": "/home/me/hercules_robots/rover1/Rover1.urdf",
    "HideBaseMesh": true,
    "TwoSided": false,
    "SensorFrames": ["front_cam", "os1_64"]
  },
  "Cameras": { ... },
  "Sensors": { ... }
}
```

| Key | Meaning |
|---|---|
| `Visuals` | Visual manifest the Unreal plugin dresses the pawn with. |
| `RobotDescription` | URDF `robot_description.launch.py` publishes. |
| `HideBaseMesh` | Stop drawing the stock pawn's meshes (they keep their collision and physics). `--show-base-mesh` sets it to false. |
| `TwoSided` | Also draw back faces, for meshes that are open or wound inconsistently (`--two-sided`). Leave it off when sensors sit inside closed meshes: back faces would hide their view. |
| `SensorFrames` | Sensors whose TF frames the robot description provides; the ROS wrapper does not publish its own transforms for them. |

Relative paths are relative to the settings file (`--relative-paths` writes them that way).

## ROS 2 frames

For a vehicle `Rover1` imported with `base_footprint` as its body frame:

```text
world                                        (hercules_node: spawn pose from settings)
└── Rover1                                   (hercules_node)
    └── Rover1/ground_truth/odom_local        (hercules_node: odometry, relative to the spawn pose)
        └── Rover1/base_footprint             (robot_state_publisher, from here down)
            └── Rover1/base_link
                ├── Rover1/camera_link
                │   └── Rover1/front_cam_body
                │       └── Rover1/front_cam_optical   <- camera images and camera_info
                └── Rover1/lidar_link
                    └── Rover1/os1_64                  <- lidar point clouds
```

Images and `camera_info` are stamped `Rover1/<camera>_optical`, point clouds `Rover1/<lidar>`,
ranges `Rover1/<distance sensor>`, and IMU data `Rover1/ground_truth/odom_local`, the same frames
the wrapper uses for vehicles that were not imported, so tools work the same for both.

## What is and is not simulated

* Physics, wheel geometry and mass come from the stock pawn of the chosen drive, not from the
  URDF.
* URDF visuals have no collision. CPU lidars, which trace against collision geometry, do not see
  them; cameras and GPU sensors do, as a real robot sees parts of itself.
* The visual meshes copy the pawn mesh's segmentation stencil value, so the robot keeps one
  segmentation ID.
* Joint motion is not simulated (see [Joints](#joints)).

## Tests

`colcon test --packages-select hercules_urdf_import airsim_ros_pkgs` covers the frame math, the
mesh readers (closed, outward-facing geometry), URDF validation and all four robot types. A launch
test runs `robot_state_publisher` on importer output, and another runs the Hero bridges against a
fake simulator (`fake_airsim_server`). Together they check that every sensor frame matches the pose
the wrapper derives from the generated settings. `AirLibUnitTests --urdf-import-dir <output dir>`
checks importer output against AirLib's own settings and Euler conventions. The Unreal side (mesh
construction on the pawn) needs an Unreal Engine build to verify.
