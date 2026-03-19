# HERCULES

**HE**terogeneous **R**obot simulator for **C**oordination, scene **U**nderstanding, **L**arge-scale **E**xploration and **S**LAM

HERCULES is an open-source simulation and experimentation framework for heterogeneous multi-robot autonomy, built on [Unreal Engine 5](https://www.unrealengine.com/) and extending [AirSim](https://github.com/microsoft/AirSim) and [Cosys-AirSim](https://github.com/Cosys-Lab/Cosys-AirSim). It enables UAV–UGV teams to collaboratively explore, map, and perceive large-scale, photorealistic environments with synchronized sensing and ready-to-run research workflows.

HERCULES is developed and maintained by the [Lunar Lab](https://lab-idar.gatech.edu/) at Georgia Institute of Technology.

> **Paper:** *HERCULES: An Open-Source Framework for Heterogeneous Multi-Robot SLAM, Collaborative Perception, and Exploration in Photorealistic Environments*
> Sandilya Sai Garimella, Daniel Chase Butterfield, Sean Wilson, and Lu Gan


---

## Key Features

### Integrated Heterogeneous Autonomy Stack
- **Concurrent UAV–UGV operation** with quadrotors, Jackal-style differential-drive UGVs, and SUV car platforms in a shared physics world
- **Unified high-level control** across aerial and ground platforms — a novel autonomous waypoint-following UGV controller mirrors the UAV command interface for seamless coordination experiments
- **Extensible architecture** — add new platforms (e.g., legged or aquatic robots) by introducing vehicle models and controllers without modifying shared simulation infrastructure

### Multi-Modal Sensing
- **Full Cosys-AirSim sensor suite:** RGB, stereo, depth (DepthPlanar/DepthPerspective), 3D LiDAR (including GPU-accelerated), IMU, GPS, barometer, magnetometer, pulse-echo, UWB ranging, and ground-truth pose/semantic segmentation
- **Thermal infrared (FLIR/LWIR) camera:** Physics-based long-wave infrared simulation integrating Planck-law spectral radiance over the 8–14 µm band with material emissivity and temperature profiles
- **Night-vision (NVG) mode:** Empirical image-intensifier approximation with adaptive gain, gamma correction, sensor noise, and phosphor-style green colormap
- All sensors are perfectly time-synchronized out of the box, with optional per-sensor latency controls for realistic delay modeling
- Hardware profiles provided for Intel RealSense RGB-D, Velodyne/Ouster LiDAR, and others

### Photorealistic Environments
- **Three included environments:** Desert (Australian Outback), Forest, and City — each with distinct challenges (sparse landmarks, perceptual aliasing, dynamic obstacles and occlusions)
- **Geo-registered environments** via [Cesium for Unreal](https://cesium.com/platform/cesium-for-unreal/) — import real-world terrain, satellite imagery, and building models aligned to true lat/long coordinates
- Leverages UE5 **Lumen** global illumination and **Nanite** virtualized geometry for real-time lifelike rendering
- Configurable time-of-day, day/night cycles, and weather via UE5's sky and atmosphere system
- Drop-in UE5 plugin — works with any Unreal project, including Fab marketplace environments

### Dynamic Agents and Environment Phenomena
- **Animal Behavior Agents** (kangaroos, deer) with configurable AI paths
- **Human-Centric Interactive Agents** (MetaHuman pedestrians)
- **Autonomous Road Traffic Agents** (VehicleAI traffic)
- **Wildfire Spread Dynamics** with progressive smoke effects
- **Flood Inundation Modeling** with rising water levels
- **Crop Disease Transmission Dynamics** (SIR-based spread model)
- All phenomena are parameterized, pseudo-randomly seeded for reproducibility, and can coexist in a single simulation

### Planning, Exploration, and Control
- **Ground-truth map generation pipeline:** UE5 geometry → OctoMaps → 2.5D elevation maps with slope layers for terrain-aware navigation (handles overhangs and negative obstacles)
- **Kinodynamic RRT (KRRT) planner** with task-aware sampling and clearance-aware expansion for dynamically feasible trajectories
- **Frontier-based A\* planner** for large-scale coverage-driven exploration
- **Pure-pursuit UGV controller** with proportional steering and speed control
- **Two coordination modes:** Complementary Coverage (viewpoint diversification / exploration) and Leader–Follower (convoy-style overwatch for cooperative perception)
- **Information-driven decentralized coordination** with modular, swappable planning/mapping backends

### Experiment Design and Verification Tools
- **Multi-Robot Trajectory Designer** for scripting team paths and waypoint sequences
- **Wildfire Spread Design Tool** for defining spatial regions of interest
- **Multimodal Ground-Truth Labeler** exporting aligned RGB, depth, instance/semantic segmentation, and LiDAR annotations
- **UE5 Graphical Automation Workflows** (Blueprint automations) for batch scenario generation

### Dataset Collection Workflows
- **Deterministic timing** with fixed-step simulation updates for bitwise-reproducible replays
- **Configurable synchronization** — strictly synchronized or with controlled perturbations to emulate clock drift and network delay
- **Multi-format export:** ROS/ROS 2 bags with synchronized `/clock`, raw PNG/TXT/NPY/CSV dumps, and KITTI-style layouts (`image_02/`, `velodyne/`, `calib/`, `label_2/`) for direct use with existing SLAM and 3D detection pipelines

### Research-Ready Interfaces
- **ROS 2 interface** with multi-threaded publishers and optimized client calls for low-latency, real-time throughput
- **RViz2 visualization** of sensor streams, robot states, and map layers
- **Lightweight Python/C++ API** for fast prototyping and ML workflows
- Both interfaces share a unified simulation clock for deterministic synchronization and can be used concurrently

---

## Benchmarks and Experiments

HERCULES ships with ready-to-run evaluation suites for three core research workflows:

### Collaborative SLAM
Benchmarks for [ROMAN](https://github.com/mit-acl/roman) (object-based multi-robot map alignment with Kimera-RPGO backend) and odometry baselines including OpenVINS, ORB-SLAM3, and LIO-SAM across city, desert, and forest sequences with heterogeneous UAV–UGV teams.

### Collaborative Perception (3D Object Detection)
A vehicle–infrastructure cooperative (VIC) setting compatible with [DAIR-V2X](https://github.com/AIR-THU/DAIR-V2X). Includes 6,000 synchronized UGV–UAV frame pairs (RGB, LiDAR, calibration, ground truth) in DAIR-V2X VIC-Sync format. Demonstrates sim-to-real pretraining with PointPillars late-fusion models.

### Multi-Robot Exploration and Coordination
Scaling and ablation experiments with teams from 1 to 5 UGV–UAV pairs, evaluating coverage, efficiency, and the effects of cross-layer map fusion and within-layer coordination.

All datasets, environment configurations, sensor calibrations, motion trajectories, and experiment code are publicly released for full reproducibility.

---

## Comparison with Related Simulators

| Simulator | Engine | Scope | Concurrent UAV+UGV | Outdoor km-scale | Dynamic agents | Dynamic phenomena | Eval/Dataset tools | Experiment design tools |
|---|---|---|---|---|---|---|---|---|
| **HERCULES (ours)** | UE5/PhysX | Heterogeneous | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| AirSim | UE/PhysX | Aerial/Car | × | △ | × | × | × | × |
| Cosys-AirSim | UE5/PhysX | Aerial/Car/UGV | × | △ | △ | △ | △ | △ |
| Isaac Sim | Omniverse/Flex | Multi-domain | △ | △ | △ | △ | × | △ |
| CARLA | UE/PhysX | Ground (AV) | × | ✓ | ✓ | △ | × | ✓ |
| FastSim (Unity) | Unity/Flexible | Aerial | × | △ | × | × | △ | ✓ |
| ARGoS | Custom | Aerial/Ground | × | × | × | × | × | △ |
| Gazebo/Ignition | ODE/DART+ | Multi-domain | △ | △ | △ | × | × | × |
| Webots | ODE | Multi-domain | △ | × | △ | × | × | ✓ |

✓ = native; △ = with effort/partial; × = absent.

---

## Getting Started

### Requirements
- Unreal Engine 5
- ROS 2 (for ROS interface)
- Python 3.8+ (for Python API)

### Installation

#### From packaged binary (Windows/Linux)
- [Download and run](https://cosys-lab.github.io/run_packaged)

#### From precompiled plugin (Windows/Linux)
- [Download and install](https://cosys-lab.github.io/install_precompiled)

#### Build from source — Windows
- [Build instructions](https://cosys-lab.github.io/install_windows)

#### Build from source — Linux
- [Build instructions](https://cosys-lab.github.io/install_linux)

### Documentation

View our [detailed documentation](https://cosys-lab.github.io/) on all aspects of HERCULES.

---

## Citation

If you use HERCULES in your research, please cite:

```bibtex
@article{garimella2025hercules,
  author  = {Garimella, Sandilya Sai and Butterfield, Daniel Chase and Wilson, Sean and Gan, Lu},
  title   = {{HERCULES}: An Open-Source Framework for Heterogeneous Multi-Robot {SLAM}, Collaborative Perception, and Exploration in Photorealistic Environments},
  journal = {International Journal of Robotics Research},
  year    = {2025},
  note    = {Under review}
}
```

---

## Acknowledgments and Attribution

HERCULES builds on [AirSim](https://github.com/microsoft/AirSim) (Microsoft) and its [Cosys-AirSim](https://github.com/Cosys-Lab/Cosys-AirSim) fork (Cosys-Lab). We gratefully acknowledge both projects for providing the foundational simulation infrastructure that HERCULES extends.

### Cosys-AirSim

HERCULES inherits and extends the following Cosys-AirSim features:

- Updated AirSim for Unreal Engine 5
- Multi-layer annotation for ground-truth label generation (RGB, greyscale, texture) with extensive API integration for camera and GPU-LiDAR sensors
- Instance segmentation
- Echo sensor type for sonar/radar simulation
- GPU-accelerated LiDAR sensor with realistic intensity generation
- Skid-steering SimMode with ClearPath Husky and Pioneer P3DX vehicle types
- MATLAB API client
- Random but deterministic dynamic object types and world configuration
- BoxCar vehicle model for indoor spaces
- Updated ComputerVision mode with full API, sensor attachment, and improved handling
- Updated LiDAR sensor with ground-truth point labels, range-noise, and full-scan API delivery
- External (world-mounted) sensors for cameras, Echo, and GPU-LiDAR
- Sensor ignore filtering via `_MarkedIgnore_` Unreal tag
- Camera distortion features (chromatic aberration, motion blur, lens distortion)
- Updated Python ROS and C++ ROS 2 implementations

Please cite Cosys-AirSim if you use inherited sensor or platform features:

```bibtex
@inproceedings{cosysairsim2023jansen,
  author    = {Jansen, Wouter and Verreycken, Erik and Schenck, Anthony and Blanquart, Jean-Edouard and Verhulst, Connor and Huebel, Nico and Steckel, Jan},
  booktitle = {2023 Annual Modeling and Simulation Conference (ANNSIM)},
  title     = {{COSYS-AIRSIM}: A Real-Time Simulation Framework Expanded for Complex Industrial Applications},
  year      = {2023},
  pages     = {37--48}
}
```

### AirSim

The original AirSim platform provides the core UE plugin architecture, physics-based drone/car simulation, and baseline sensor models:

```bibtex
@inproceedings{airsim2017fsr,
  author    = {Shah, Shital and Dey, Debadeepta and Lovett, Chris and Kapoor, Ashish},
  title     = {{AirSim}: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles},
  year      = {2017},
  booktitle = {Field and Service Robotics},
  eprint    = {arXiv:1705.05065}
}
```

### Other Associated Publications

```bibtex
@inproceedings{lidarsim2022jansen,
  author    = {Jansen, Wouter and Huebel, Nico and Steckel, Jan},
  booktitle = {2022 IEEE Sensors},
  title     = {Physical {LiDAR} Simulation in Real-Time Engine},
  year      = {2022},
  pages     = {1--4},
  doi       = {10.1109/SENSORS52175.2022.9967197}
}

@article{echosim2021schouten,
  author  = {Schouten, Girmi and Jansen, Wouter and Steckel, Jan},
  title   = {Simulation of Pulse-Echo Radar for Vehicle Control and {SLAM}},
  journal = {Sensors},
  volume  = {21},
  number  = {2},
  pages   = {523},
  year    = {2021},
  doi     = {10.3390/s21020523}
}
```

---

## License

This project is released under the MIT License. Please review the [License file](LICENSE) for more details. The [original AirSim MIT license](LICENSE) applies to all native AirSim source files, and the same MIT license applies to all modifications made by Cosys-Lab and by the Lunar Lab at Georgia Tech.
