# Unitree Go2 in HERCULES (MuJoCo locomotion, Unreal rendering + AirSim sensors)

The Go2 is an AirSim **ComputerVision** vehicle whose pawn (`AGo2Pawn`, `Unreal/Plugins/AirSim/Source/Vehicles/Go2/`)
is posed by an external MuJoCo process. Unreal does no locomotion physics for the dog; it renders it and runs the
usual AirSim cameras, lidar, IMU and GPS on it, so the Python client and the ROS 2 bridge treat it like any other
HERCULES vehicle.

```
 go2_mujoco_bridge.py (Python)                      Unreal Editor / HERCULES (Hero sim mode)
 ┌──────────────────────────────────────┐          ┌──────────────────────────────────────────────┐
 │ MuJoCo 500 Hz (menagerie go2.xml     │  80 B    │ AGo2Pawn : AComputerVisionPawn (AirSim)      │
 │   + Unreal-fed heightfield)          │  pose    │   TCP 127.0.0.1:12345 (GO2_PORT[_<Vehicle>]) │
 │ HIMLoco policy 50 Hz (rl_sar layout) │ ───────► │   static-mesh rig from /AirSim/Go2/Meshes    │
 │ PD -> 12 joint torques               │          │   ground traces -> heightfield window        │
 │ commands: set_velocity / stand / sit │ ◄─────── │   sim time (lockstep)                        │
 └──────────────────────────────────────┘ feedback │ AirSim cameras / lidar / IMU / GPS on the pawn│
                                                   └──────────────────────────────────────────────┘
```

## Files
| file | role |
|---|---|
| `go2_mujoco_bridge.py` | physics + policy loop, TCP client, heightfield + lockstep handling (`Go2Bridge`, `Go2Sim`) |
| `go2_commander.py` | `Go2Commander`: `forward(m)`, `turn_left(deg)`, `stand()`, `sit()`, `get_camera()`, `get_lidar()`, `get_imu()` |
| `go2_scene_hfield.xml` | MuJoCo scene template: menagerie Go2 + heightfield + IMU sensors |
| `~/Documents/AirSim/settings_go2.json` | example settings: Hero mode, vehicle `Go2` (ComputerVision, PawnPath `Go2`) + Husky1 |

Third-party inputs (not in this repo): `~/multi-robot-coordination/go2_thirdparty/mujoco_menagerie/unitree_go2`
(DeepMind, BSD-3; env `GO2_MENAGERIE`) and `~/multi-robot-coordination/go2_thirdparty/rl_sar/policy/go2`
(HIMLoco TorchScript policy, Apache-2.0; env `GO2_POLICY_DIR`). `pip install --user mujoco torch numpy`.

## Run
1. settings.json: add the `Go2` pawn path and vehicle (see `settings_go2.json`), SimMode `Hero`.
2. Play the level. The pawn logs `listening for the MuJoCo bridge on 127.0.0.1:12345` (LogGo2).
3. `python3 go2_mujoco_bridge.py --vx 0.5` (real time) or `--pacing sim` (follows Unreal time; pauses with simPause).
   Scripted: `python3 go2_mujoco_bridge.py --script "forward:2;turn:90;sit;wait:2;stand"`.
4. Sensors: `Go2Commander(...).get_camera()` (AirSim `simGetImages` on vehicle `Go2`, port 41452 or 41453),
   ROS 2: `ros2 launch airsim_ros_pkgs airsim_node_hercules.launch.py host_port:=41453` (CV vehicles, names `Go2*`).

## Locomotion notes (measured 2026-10-30)
* Policy: rl_sar `go2/himloco` (TorchScript, 45 obs x 6-step history newest-first). Walks at ~0.4 m/s for
  `set_velocity(0.5)`; commands above ~0.6 make it fall. Stand pose must be the policy default (hips +-0.1 rad).
  `robot_lab` is wired in `POLICY_CONFIGS` but collapses (joint order / obs convention unverified) -- do not use.
* Turning: HIMLoco turns in place at only ~2 deg/s (it skates). `turn()` therefore adds a yaw torque assist on the
  trunk (`GO2_TURN_ASSIST_KP`, default 30 Nm s/rad, clipped +-24 Nm): ~8 deg/s at 0.8 rad/s. For fast turns use
  `turn(deg, mode="pivot")` / script `pivot:90`: a kinematic pivot (legs hold the stand pose, feet slide) at 1 rad/s.
* Terrain physics: the heightfield lives on a MuJoCo **mocap** body (a world geom's `pos` cannot be moved at
  runtime -- that bug cost an hour). Offline: 20 % ramps are climbed, a 1.5 m wall blocks the dog, an 8 cm step
  stalls the blind policy (crouches against it).
* The MuJoCo `scene.xml` floor is kept 20 m below as a safety net; the window is +-1.5 m at 10 cm (5 cm cells made the
  gait stumble through heightfield multi-contact artifacts under the small foot spheres), refreshed ~12 Hz.

## Wire protocol (little-endian, TCP)
* bridge -> pawn `POSE` 80 B: `uint32 0x474F3200, float x y z (m), float qw qx qy qz, float joints[12]` (rad, FL FR RL RR x hip thigh calf), MuJoCo frame. Identical to S2R_Go2Sim, so that bridge also works (flat floor, pawn follows its own ground trace).
* bridge -> pawn `HELLO` 16 B: `uint32 0x474F3202, uint32 flags (1 hfield mode, 2 feedback, 4 grid), float grid_half_m, float grid_cell_m`.
* pawn -> bridge `FEEDBACK` 32 B header `uint32 0x474F3201, uint32 grid_n, float sim_time, ground_z, cell, origin_x, origin_y, reserved` + `grid_n*grid_n` floats (row = y, col = x, m above the MuJoCo ground reference). Sent every tick (header only) and once per completed terrain sweep (with grid).

Frames: MuJoCo (right-handed, m) -> Unreal (left-handed, cm): `p_ue = (100x, -100y, 100z)`, `q_ue = (w, -x, y, -z)`;
joint angle about MuJoCo x -> `-theta` about Unreal x, about y -> `+theta`. The MuJoCo world is anchored at the pawn's
placed pose (first POSE packet) and rotated by the placed yaw; MuJoCo z = 0 is the Unreal ground under the placed position.
