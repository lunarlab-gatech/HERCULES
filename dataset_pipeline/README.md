# HERCULES Dataset Pipeline

One command to go from recorded trajectories to a finished dataset (raw sensor
data + calibration recording + world-frame odometry + synthetic IMU +
segmentation label maps).

## Prerequisites

1. UE editor open with the environment level, **Play** pressed (AirSim active).
2. Trajectories already recorded (`<name>_trajectory.txt` per vehicle) via the
   teleop recorders in `PythonClient/hero/`.
3. `Content/Python/init_unreal.py` present in the UE project (auto-loads at
   editor startup; it powers the actor-label dump in the `labels` stage).
   A versioned copy lives in `dataset_pipeline/ue/init_unreal.py` — copy it
   into `<UE project>/Content/Python/` for any new environment (it is already
   installed in the SmallTown project). The project needs the **Python Editor
   Script Plugin** enabled (Edit → Plugins); without it, leave out the `labels`
   stage.
4. C++ waypoint controllers built: `build_release/output/bin/{Drone,UGV}WaypointControl`.
5. Python packages for the pipeline's scripts: `numpy`, `scipy`,
   `opencv-python`, `pillow`, `pyyaml` and `rpc-msgpack`.

## Configs

| Config | Environment |
|--------|-------------|
| `configs/blocks_team.yaml` | Blocks with the Hero-mode team of [Running a UAV–UGV Team](../docs/hero_team_quickstart.md): `Drone1`, `Drone2`, `Husky1`, `Husky2` |
| `configs/smalltown.yaml` | SmallTown with `Drone1` and `Husky1` |

Relative paths in a config are relative to the HERCULES repository root, and
`~` and `$VARS` are expanded. Datasets go to `~/hercules_datasets/<sequence>`
and trajectories are read from `trajectory_data/`, where the recorders and the
trajectory planner write them. `paths.cosys_root` defaults to this repository.

## Run

From the repository root, with the editor playing:

```bash
python3 dataset_pipeline/generate_dataset.py dataset_pipeline/configs/blocks_team.yaml
```

Useful variants:

```bash
cd dataset_pipeline
python3 generate_dataset.py configs/smalltown.yaml --dry-run          # check paths, print commands only
python3 generate_dataset.py configs/smalltown.yaml --stages labels    # just label maps
python3 generate_dataset.py configs/smalltown.yaml --stages post,labels
```

For a new sequence: copy the YAML, edit `sequence` (output folder name) and
`trajectory_dir`, then run. Per-vehicle speeds/altitudes and all rates live in
the YAML — nothing is hardcoded in the scripts anymore (the scripts keep their
old values as CLI defaults, so running them by hand still works).

## Stages

| Stage       | What happens |
|-------------|--------------|
| `preflight` | (always runs) checks RPC ports, trajectory files, binaries, output dir |
| `collect`   | starts the synchronized collector, then replays every vehicle's trajectory via the C++ controllers; when replay ends the collector is stopped cleanly (SIGTERM → unpause + close files) |
| `calibrate` | records cam-IMU calibration maneuvers (UAV attitude/velocity excitations, UGV drive patterns) into `<dataset>/calibration/` |
| `post`      | copies `settings.json` + trajectories into the dataset, writes `pose_world_frame.txt` (world NED), generates `synthetic_imu_<rate>Hz_9axis.txt` |
| `labels`    | AirSim segmentation colormap CSV → UE actor Label→Name dump (file-watcher, no console pasting) → merged `label_color_map_<tag>.csv` |

All subprocess output lands in `<dataset>/logs/*.log`.

## Output layout

```
<output_root>/<sequence>/
├── Drone1/  Husky1/            imu.txt odom.txt pose_world_frame.txt
│                               synthetic_imu_{200,500}Hz_9axis.txt
│                               rgb_stereo_left/ rgb_stereo_right/ depth/ seg/ lidar/
├── calibration/<vehicle>/      same layout, calibration maneuvers
├── settings.json               copy of the AirSim settings used
├── trajectory_data/            copy of the replayed trajectories
├── airsim_segmentation_colormap_list_<ts>.csv
├── ue_actor_label_to_name_<tag>.csv
├── label_color_map_<tag>.csv
└── logs/
```

## Troubleshooting

- **"RPC port not reachable"** — the sim isn't playing, or vehicle ports in the
  YAML don't match `settings.json`.
- **"UE label dump timed out"** — the editor was started before
  `init_unreal.py` existed. Restart the editor, or paste the file's contents
  into the UE Python console once, then rerun `--stages labels`.
- **Collector hit its duration before replay finished** — raise
  `collection.duration`; sim-time runs slower than wall-time under stepping.
- **ROS2 bag** — intentionally not part of the pipeline; use
  `PythonClient/hero/data_collection/pack_data_to_ros2bag_multi_vehicle.py <dataset> <bag_out>`.
