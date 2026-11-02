# Unitree Go2 and Unitree G1 in HERCULES (MuJoCo locomotion, Unreal rendering + AirSim sensors)

Both robots are AirSim **ComputerVision** vehicles whose pawn is posed by an external MuJoCo process. Unreal does no
locomotion physics for them; it renders them and runs the usual AirSim cameras, lidar, IMU and GPS on them, so the
Python client and the ROS 2 bridge treat them like any other HERCULES vehicle.

| robot | MuJoCo model | walking policy | Unreal pawn | pawn port | AirSim vehicle |
|---|---|---|---|---|---|
| Go2 quadruped | DeepMind menagerie `unitree_go2/go2.xml` | rl_sar HIMLoco (`policy/go2/himloco`) | `AGo2Pawn` (fixed rig from 12 joint angles) | 12345 | `Go2` |
| G1 humanoid | Unitree `unitree_rl_gym` `g1_description/g1_12dof.xml` (official) | Unitree pretrained `deploy/pre_train/g1/motion.pt`, deployed exactly as `deploy_mujoco.py` (official) | `AG1Pawn` (generic link rig: 27 link meshes posed from streamed link poses) | 12346 | `G1` |

```
 go2_mujoco_bridge.py --robot go2|g1 (Python)        Unreal Editor / HERCULES (Hero sim mode)
 ┌──────────────────────────────────────┐           ┌───────────────────────────────────────────────┐
 │ MuJoCo 500 Hz (robot model           │ POSE 80 B │ AMujocoPawn : AComputerVisionPawn (AirSim)    │
 │   + Unreal-fed heightfield)          │ (+LINKS)  │   TCP 127.0.0.1:<port> (GO2_PORT[_<Vehicle>]) │
 │ policy 50 Hz -> PD -> joint torques  │ ────────► │   AGo2Pawn: joint rig  / AG1Pawn: link rig    │
 │ commands: set_velocity / turn / ...  │           │   ground traces -> heightfield window         │
 │                                      │ ◄──────── │   sim time (lockstep)                         │
 └──────────────────────────────────────┘ feedback  │ AirSim cameras / lidar / IMU / GPS on the pawn │
                                                    └───────────────────────────────────────────────┘
```

## Files
| file | role |
|---|---|
| `go2_mujoco_bridge.py` | physics + policy loop, TCP client, heightfield + lockstep handling (`MujocoBridge`, `RobotSim`, `ROBOTS` specs, `LocoPolicy` for the Go2, `G1Policy` for the G1) |
| `go2_commander.py` | `RobotCommander(robot=...)` (`Go2Commander`, `G1Commander`): `forward(m)`, `sidestep(m)`, `turn_left(deg)`, `stand()`, `sit()` (Go2), `get_camera()`, `get_lidar()`, `get_imu()` |
| `mujoco_scene_hfield.xml` | MuJoCo scene template: robot include + heightfield + plane on mocap bodies (+ IMU sensors for the Go2) |
| `tools/g1_meshes_to_obj.py` | Unitree G1 STL link meshes -> cm OBJ (decimated to 4 k tris) for the Unreal importer |
| `tools/ue_import_meshes.py` | editor-side import of the OBJs into `/AirSim/G1/Meshes` + Unitree's dark/light materials |
| `~/Documents/AirSim/settings_go2.json` | example settings: Hero mode, vehicles `Go2` + `G1` (ComputerVision, PawnPaths `Go2` / `G1`) + Husky1 |

Unreal side: `Unreal/Plugins/AirSim/Source/Vehicles/Mujoco/MujocoPawn.{h,cpp}` (socket, anchoring, terrain feedback,
generic link rig), `Vehicles/Go2/Go2Pawn.{h,cpp}` (Go2 rig), `Vehicles/G1/G1Pawn.{h,cpp}` (defaults only).
Meshes live in the AustraliaLandscape plugin copy's Content (`/AirSim/Go2/Meshes`, `/AirSim/G1/Meshes`).

Third-party inputs (not in this repo, all under `~/multi-robot-coordination/go2_thirdparty/`):
`mujoco_menagerie/unitree_go2` (DeepMind, BSD-3; env `GO2_MENAGERIE`), `rl_sar/policy/go2` (HIMLoco TorchScript,
Apache-2.0; env `GO2_POLICY_DIR`), `unitree_rl_gym` (Unitree, BSD-3: `resources/robots/g1_description` meshes + xml,
`deploy/pre_train/g1/motion.pt`, `deploy/deploy_mujoco/configs/g1.yaml`; env `G1_RL_GYM`).
`pip install --user mujoco torch numpy` (+ `open3d` only for the mesh conversion tool).

## Run
1. settings.json: add the pawn paths and vehicles (see `settings_go2.json`), SimMode `Hero`.
2. Play the level. Each pawn logs `listening for the MuJoCo bridge on 127.0.0.1:<port>` (LogMujocoPawn).
3. `python3 go2_mujoco_bridge.py --robot g1 --vx 0.5` (real time) or `--pacing sim` (follows Unreal time; pauses with
   simPause). Scripted: `--script "forward:5;turn:90;forward:3;side:1"` (Go2 also `sit`, `stand`, `pivot:90`).
4. Sensors: `RobotCommander(robot="g1").get_camera()` (AirSim `simGetImages` on vehicle `G1`, CV RPC port 41453),
   ROS 2: `ros2 launch airsim_ros_pkgs airsim_node_hercules.launch.py host_port:=41453` publishes every
   ComputerVision vehicle under `/hercules_node/<Vehicle>/` (imu 100 Hz, odom 20 Hz, gps 20 Hz, lidar 2 Hz, images 1-2 Hz).

## G1 humanoid notes (measured live 2026-11-02, ArenaProcV2s_obstacles)
* Model + policy are Unitree's own, used unmodified: `g1_12dof.xml` (legs actuated, upper body fixed; the mesh
  collision + 4 sole spheres per foot) and `motion.pt` with the `g1.yaml` deployment layout (47 obs, 12 actions,
  kp 100/100/100/150/40/40, kd 2/2/2/4/2/2, action scale 0.25, 0.8 s gait phase, 500 Hz / 50 Hz).
* Walks at 0.47 m/s for `set_velocity(0.5)` (0.87 at 1.0, 1.2 at 1.5), sidesteps at 0.3, backs up at -0.5. Yaw-rate
  commands are tracked (0.5 rad/s -> 0.47 measured, 1.0 -> 0.92), so `turn()` needs no assist: ~45 deg/s at 0.8 rad/s.
  Natural heading drift while walking straight is -0.6 deg/s; `forward()`/`sidestep()` hold the start heading.
* Six consecutive live starts: 6/6 walked 7.2-7.5 m in 16 s, pelvis height 0.765-0.782 m (standing 0.79), 0 recoveries.
  Scripted route (forward 5, turn 90, forward 3, turn -180, forward 3, sidestep 1): pelvis 0.764-0.783 m, 0 recoveries.
* A humanoid cannot be held by joint PD alone (it falls in ~3 s), so there is no "pose" standing mode: the policy
  takes over on the first step and `stand()` = policy at zero command (it marks time in place). No `sit`.
* Rendering: the bridge announces the 27 visual link meshes (MESHES packet) and streams each link's pose relative to
  the pelvis every step (LINKS packet); the pawn creates one static-mesh component per link at runtime. MuJoCo
  re-centres every mesh at compile (`mesh_pos`/`mesh_quat`), so the bridge streams the *original STL frame*
  (`R_o = R_geom R_m^T`, `p_o = p_geom - R_o p_m`, verified equal to body pose o xml offset). The meshes were
  decimated to 4 k triangles each (`tools/g1_meshes_to_obj.py`): the raw torso has 157 k and the hands 70 k, which
  the editor's OBJ importer takes minutes on and a 752x480 camera never resolves.
* Head camera: `front_center` at X 0.06, Z -0.45 (NED) from the pelvis = 1.24 m eye height; lidar at Z -0.40.

## Go2 locomotion notes (measured 2026-10-30)
* Policy: rl_sar `go2/himloco` (TorchScript, 45 obs x 6-step history newest-first). Walks at ~0.4 m/s for
  `set_velocity(0.5)`; commands above ~0.6 make it fall. Stand pose must be the policy default (hips +-0.1 rad).
  `robot_lab` is wired in `POLICY_CONFIGS` but collapses (joint order / obs convention unverified) -- do not use.
* Turning: HIMLoco turns in place at only ~2 deg/s (it skates). `turn()` therefore adds a yaw torque assist on the
  trunk (`GO2_TURN_ASSIST_KP`, default 30 Nm s/rad, clipped +-24 Nm): ~8 deg/s at 0.8 rad/s. For fast turns use
  `turn(deg, mode="pivot")` / script `pivot:90`: a kinematic pivot (legs hold the stand pose, feet slide) at 1 rad/s.
* Six consecutive live starts (2026-11-02): 6/6 walked 6.16 m in 16 s at 0.40 m/s, body height 0.256-0.302 m, 0 recoveries.

## Shared mechanics
* Terrain physics: the heightfield and the plane live on MuJoCo **mocap** bodies (a world geom's `pos` cannot be
  moved at runtime -- that bug cost an hour).
* Contact model (hybrid): a plane always sits at the lowest ground of the central 1.2 m of the window and carries
  everything within 5 cm of it (clean foot contacts); only cells rising more than 5 cm above it stay in the
  heightfield (10 cm cells, +-1.5 m window, ~12 Hz), all other cells are pushed 5 cm below the plane. Reason:
  MuJoCo heightfield multi-contact under the Go2's 2.2 cm foot spheres made the HIMLoco gait collapse on roughly half
  the starts (lattice-phase dependent) and stand tall and stall when starting next to an obstacle. Offline: 20 %
  ramps up and down, walls, starting next to a wall all fine; an 8 cm step still stalls the blind Go2 policy.
* Guards (both robots, logged as "recovery N"): stall (commanded > 0.2 m/s, < 10 cm in 4 s), collapse (body below
  `collapse_z` for 1 s: Go2 0.19 m, G1 0.55 m), topple (not upright for 1 s -> reset upright in place). The policy
  balances at zero command for 1 s before velocity commands apply. Reconnecting bridges re-anchor the MuJoCo frame at
  the robot's current pose (no teleport back to the spawn).
* Lockstep: `--pacing sim` advances MuJoCo to the Unreal time in the feedback packets (simPause stops the robot,
  simContinueForTime advances it).

## Wire protocol (little-endian, TCP)
* bridge -> pawn `POSE` 80 B: `uint32 0x474F3200, float x y z (m), float qw qx qy qz, float joints[12]` (rad; Go2 FL FR RL RR x hip thigh calf, G1 the 12 leg joints), MuJoCo frame. Identical to S2R_Go2Sim, so that bridge also works (flat floor, pawn follows its own ground trace).
* bridge -> pawn `HELLO` 16 B: `uint32 0x474F3202, uint32 flags (1 hfield mode, 2 feedback, 4 grid), float grid_half_m, float grid_cell_m`.
* bridge -> pawn `MESHES` 40 B header `uint32 0x474F3204, uint32 count, char set[32]` + count x `char name[48]`: link meshes to render as `/AirSim/<set>/Meshes/<name>` (sent once after HELLO; G1 only).
* bridge -> pawn `LINKS` 8 B header `uint32 0x474F3205, uint32 count` + count x `float x y z qw qx qy qz`: pose of each announced link relative to the base body, MuJoCo frame (sent after every POSE; G1 only).
* pawn -> bridge `FEEDBACK` 32 B header `uint32 0x474F3201, uint32 grid_n, float sim_time, ground_z, cell, origin_x, origin_y, reserved` + `grid_n*grid_n` floats (row = y, col = x, m above the MuJoCo ground reference). Sent every tick (header only) and once per completed terrain sweep (with grid).

Frames: MuJoCo (right-handed, m) -> Unreal (left-handed, cm): `p_ue = (100x, -100y, 100z)`, `q_ue = (w, -x, y, -z)`;
joint angle about MuJoCo x -> `-theta` about Unreal x, about y -> `+theta`. The MuJoCo world is anchored at the pawn's
placed pose (first POSE packet) and rotated by the placed yaw; MuJoCo z = 0 is the Unreal ground under the placed position.
