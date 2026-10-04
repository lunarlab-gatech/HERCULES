#!/usr/bin/env python3
"""
generate_dataset.py -- end-to-end HERCULES dataset generation.

Given recorded trajectories and a running (playing) UE sim with Cosys-AirSim,
this runs the full pipeline:

  preflight -> collect -> calibrate -> post -> labels

  collect    trajectory replay (C++ waypoint controllers) + synchronized
             multi-vehicle data collector, run concurrently
  calibrate  cam-IMU calibration maneuvers recorded into <dataset>/calibration
  post       settings/trajectory archival, world-frame odometry, synthetic IMU
  labels     AirSim segmentation colormap + UE actor-label dump (via the
             init_unreal.py file-watcher) + merged label_color_map CSV

Usage:
  python3 generate_dataset.py configs/smalltown.yaml
  python3 generate_dataset.py configs/smalltown.yaml --stages labels
  python3 generate_dataset.py configs/smalltown.yaml --dry-run

Relative paths in the config are relative to the HERCULES repository root.
"""

import argparse
import glob
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import yaml

ALL_STAGES = ["collect", "calibrate", "post", "labels"]
REPO_ROOT = Path(__file__).resolve().parent.parent


def log(msg):
    print(f"[pipeline {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def die(msg):
    log(f"ERROR: {msg}")
    sys.exit(1)


def expand(p):
    """Expand ~ and $VARS; resolve relative paths against the repository root."""
    path = Path(os.path.expandvars(os.path.expanduser(str(p))))
    if not path.is_absolute():
        path = REPO_ROOT / path
    return path.resolve()


def tail(path, n=15):
    try:
        lines = Path(path).read_text(errors="replace").splitlines()
        return "\n".join(lines[-n:])
    except OSError:
        return "<no log>"


class Pipeline:
    def __init__(self, cfg, dry_run=False):
        self.cfg = cfg
        self.dry = dry_run

        self.env_name = cfg["environment"]
        self.sequence = cfg["sequence"]
        self.dataset_dir = expand(cfg["output_root"]) / self.sequence
        self.log_dir = self.dataset_dir / "logs"
        self.trajectory_dir = expand(cfg.get("trajectory_dir", "trajectory_data"))
        self.airsim_settings = expand(cfg["airsim_settings"])
        paths = cfg.get("paths") or {}
        # The HERCULES checkout that holds PythonClient/ and build_release/.
        self.cosys_root = expand(paths.get("cosys_root", REPO_ROOT))
        self.ue_saved_dir = expand(paths["ue_saved_dir"]) if paths.get("ue_saved_dir") else None

        pc = self.cosys_root / "PythonClient"
        self.collector = pc / "hero" / "data_collection" / "hercules_multi_vehicle_data_collector.py"
        self.calib_uav = pc / "hero" / "data_collection" / "calibration_camimu_UAV.py"
        self.calib_ugv = pc / "hero" / "data_collection" / "calibration_camimu_UGV.py"
        self.world_translate = pc / "hero" / "util" / "apply_world_translation_odom.py"
        self.imu_synth = pc / "hero" / "data_collection" / "imu_from_GTodometry.py"
        self.seg_dir = pc / "segmentation"
        self.drone_bin = self.cosys_root / "build_release" / "output" / "bin" / "DroneWaypointControl"
        self.ugv_bin = self.cosys_root / "build_release" / "output" / "bin" / "UGVWaypointControl"

        self.drones = cfg.get("drones", []) or []
        self.ugvs = cfg.get("ugvs", []) or []
        self.drone_names = [d["name"] for d in self.drones]
        self.ugv_names = [u["name"] for u in self.ugvs]
        self.ports = cfg.get("ports", {"drone": 41451, "ugv": 41452})
        self.collect_cfg = cfg.get("collection", {})
        self.calib_cfg = cfg.get("calibration", {})
        self.post_cfg = cfg.get("post", {})
        self.labels_cfg = cfg.get("labels", {})

    # ---------- helpers ----------

    def _spawn(self, name, cmd, cwd=None, extra_env=None):
        """Start a subprocess logging to <dataset>/logs/<name>.log; return Popen."""
        log_path = self.log_dir / f"{name}.log"
        if self.dry:
            log(f"DRY-RUN would start [{name}]: {' '.join(map(str, cmd))}"
                + (f" (cwd={cwd})" if cwd else ""))
            return None
        env = os.environ.copy()
        if extra_env:
            env.update(extra_env)
        log(f"start [{name}]: {' '.join(map(str, cmd))}")
        f = open(log_path, "w")
        return subprocess.Popen([str(c) for c in cmd], stdout=f, stderr=subprocess.STDOUT,
                                cwd=str(cwd) if cwd else None, env=env)

    def _run(self, name, cmd, cwd=None, extra_env=None):
        """Run a subprocess to completion; die on failure."""
        proc = self._spawn(name, cmd, cwd=cwd, extra_env=extra_env)
        if proc is None:
            return
        proc.wait()
        if proc.returncode != 0:
            die(f"[{name}] failed (exit {proc.returncode}). Last log lines:\n"
                f"{tail(self.log_dir / (name + '.log'))}")
        log(f"done [{name}]")

    @staticmethod
    def _stop(proc, name, timeout=90):
        if proc is None or proc.poll() is not None:
            return
        log(f"stopping [{name}] (SIGTERM)")
        proc.send_signal(signal.SIGTERM)
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            log(f"[{name}] did not exit in {timeout}s; killing")
            proc.kill()
            proc.wait()

    def _check_port(self, port, what):
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=3):
                pass
        except OSError:
            die(f"{what} RPC port {port} is not reachable. Is the sim playing?")

    def _run_collector(self, outdir, duration):
        cmd = [sys.executable, self.collector,
               "--outdir", outdir,
               "--duration", duration,
               "--imu-rate", self.collect_cfg.get("imu_rate", 200.0),
               "--odom-hz", self.collect_cfg.get("odom_hz", 20.0),
               "--cam-hz", self.collect_cfg.get("cam_hz", 20.0),
               "--lidar-hz", self.collect_cfg.get("lidar_hz", 20.0),
               "--camera", self.collect_cfg.get("camera", "front_center"),
               "--lidar", self.collect_cfg.get("lidar", "LidarSensor1"),
               "--drone-port", self.ports["drone"],
               "--husky-port", self.ports["ugv"],
               "--drones", *self.drone_names,
               "--huskies", *self.ugv_names,
               "--stereo-cameras", *self.collect_cfg.get("stereo_cameras",
                                                         ["stereo_left", "stereo_right"])]
        if self.collect_cfg.get("save_depth_png", False):
            cmd.append("--save-depth-png")
        name = "collector" if Path(outdir).name != "calibration" else "collector_calibration"
        return self._spawn(name, cmd, cwd=self.collector.parent), name

    # ---------- stages ----------

    def preflight(self, stages):
        log(f"dataset dir: {self.dataset_dir}")
        if not self.dry:
            if "collect" in stages or "calibrate" in stages:
                if self.drone_names:
                    self._check_port(self.ports["drone"], "Drone")
                if self.ugv_names:
                    self._check_port(self.ports["ugv"], "UGV")
            if "labels" in stages:
                # segmentation_generate_list.py connects to the multirotor client
                self._check_port(self.ports["drone"], "AirSim (labels stage)")
        problems = []
        if "collect" in stages:
            for name in self.drone_names + self.ugv_names:
                wp = self.trajectory_dir / f"{name}_trajectory.txt"
                if not wp.is_file():
                    problems.append(f"missing trajectory file: {wp}")
            if self.drone_names and not self.drone_bin.is_file():
                problems.append(f"missing binary: {self.drone_bin}")
            if self.ugv_names and not self.ugv_bin.is_file():
                problems.append(f"missing binary: {self.ugv_bin}")
            veh_dirs = [d for d in (self.dataset_dir / n for n in self.drone_names + self.ugv_names)
                        if d.exists()]
            if veh_dirs:
                problems.append(f"dataset already contains vehicle data ({veh_dirs[0]} ...). "
                                "Move it away or change 'sequence' in the config.")
        if "post" in stages and not self.airsim_settings.is_file():
            problems.append(f"missing AirSim settings: {self.airsim_settings}")
        if "labels" in stages:
            if self.ue_saved_dir is None:
                problems.append("set paths.ue_saved_dir to the UE project's Saved folder "
                                "for the labels stage")
            elif not self.ue_saved_dir.is_dir():
                problems.append(f"UE project Saved dir not found: {self.ue_saved_dir}")
        if self.dry:
            for problem in problems:
                log(f"DRY-RUN preflight would stop: {problem}")
            return
        if problems:
            die(problems[0])
        self.dataset_dir.mkdir(parents=True, exist_ok=True)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        log("preflight OK")

    def stage_collect(self):
        duration = self.collect_cfg.get("duration", 1300.0)
        warmup = self.collect_cfg.get("warmup_s", 5.0)
        grace = self.collect_cfg.get("post_replay_grace_s", 10.0)

        collector, coll_name = self._run_collector(self.dataset_dir, duration)

        controllers = []
        if not self.dry:
            time.sleep(warmup)
            if collector.poll() is not None:
                die(f"collector exited during warmup (exit {collector.returncode}). Log:\n"
                    f"{tail(self.log_dir / (coll_name + '.log'))}")

        for d in self.drones:
            cmd = [self.drone_bin, d["name"],
                   self.trajectory_dir / f"{d['name']}_trajectory.txt",
                   d.get("velocity", 0.75), d.get("altitude", -4.0)]
            if d.get("use_waypoint_z", False):
                cmd.append("--use-waypoint-z")
            if d.get("no_return_home", False):
                cmd.append("--no-return-home")
            controllers.append((f"replay_{d['name']}",
                                self._spawn(f"replay_{d['name']}", cmd)))
        for u in self.ugvs:
            cmd = [self.ugv_bin, u["name"], u.get("speed", 0.29),
                   self.trajectory_dir / f"{u['name']}_trajectory.txt",
                   u.get("ctrl_hz", 20)]
            controllers.append((f"replay_{u['name']}",
                                self._spawn(f"replay_{u['name']}", cmd)))

        if self.dry:
            return

        # Wait until every controller finishes or the collector hits its duration.
        while True:
            running = [(n, p) for n, p in controllers if p.poll() is None]
            if collector.poll() is not None:
                if running:
                    log("collector reached its duration before replay finished — "
                        "stopping remaining controllers (consider a longer collection.duration)")
                    for n, p in running:
                        self._stop(p, n, timeout=15)
                break
            if not running:
                log(f"replay finished; waiting {grace}s grace, then stopping collector")
                time.sleep(grace)
                self._stop(collector, coll_name)
                break
            time.sleep(2)

        for n, p in controllers:
            if p.returncode not in (0, None) and p.returncode is not None and p.returncode != 0:
                log(f"WARNING: [{n}] exited with {p.returncode}. Log tail:\n"
                    f"{tail(self.log_dir / (n + '.log'))}")
        if collector.poll() is None:
            collector.wait()
        if collector.returncode != 0:
            die(f"collector failed (exit {collector.returncode}). Log:\n"
                f"{tail(self.log_dir / (coll_name + '.log'))}")
        log("collect stage complete")

    def stage_calibrate(self):
        if not self.calib_cfg.get("enabled", True):
            log("calibration disabled in config; skipping")
            return
        duration = self.calib_cfg.get("duration", 300.0)
        grace = self.calib_cfg.get("grace_s", 5.0)
        outdir = self.dataset_dir / "calibration"

        collector, coll_name = self._run_collector(outdir, duration)
        if not self.dry:
            time.sleep(self.collect_cfg.get("warmup_s", 5.0))
            if collector.poll() is not None:
                die(f"calibration collector exited early. Log:\n"
                    f"{tail(self.log_dir / (coll_name + '.log'))}")

        procs = []
        if self.drone_names:
            procs.append(("calib_uav", self._spawn(
                "calib_uav",
                [sys.executable, self.calib_uav, "--port", self.ports["drone"],
                 "--drones", *self.drone_names],
                cwd=self.calib_uav.parent)))
        if self.ugv_names:
            procs.append(("calib_ugv", self._spawn(
                "calib_ugv",
                [sys.executable, self.calib_ugv, "--port", self.ports["ugv"],
                 "--ugvs", *self.ugv_names],
                cwd=self.calib_ugv.parent)))

        if self.dry:
            return

        for n, p in procs:
            p.wait()
            if p.returncode != 0:
                log(f"WARNING: [{n}] exited with {p.returncode}. Log tail:\n"
                    f"{tail(self.log_dir / (n + '.log'))}")
        log(f"calibration maneuvers finished; waiting {grace}s grace, then stopping collector")
        time.sleep(grace)
        self._stop(collector, coll_name)
        log("calibrate stage complete")

    def stage_post(self):
        # Archive settings.json and the trajectories used
        if self.dry:
            log(f"DRY-RUN would copy {self.airsim_settings} and {self.trajectory_dir} "
                f"into {self.dataset_dir}")
        else:
            shutil.copy2(self.airsim_settings, self.dataset_dir / "settings.json")
            shutil.copytree(self.trajectory_dir, self.dataset_dir / "trajectory_data",
                            dirs_exist_ok=True)
            log("archived settings.json and trajectory_data")

        # Odometry -> world NED frame (writes pose_world_frame.txt per vehicle)
        self._run("world_translate",
                  [sys.executable, self.world_translate, "--base-dir", self.dataset_dir])

        # Synthetic IMU from GT odometry (needs pose_world_frame.txt from above)
        for name in self.drone_names + self.ugv_names:
            odom = self.dataset_dir / name / "odom.txt"
            for rate in self.post_cfg.get("imu_rates", [200, 500]):
                out = self.dataset_dir / name / f"synthetic_imu_{rate}Hz.txt"
                self._run(f"imu_synth_{name}_{rate}",
                          [sys.executable, self.imu_synth, odom, out, "--imu-rate", rate])
        log("post stage complete")

    def stage_labels(self):
        env_tag = self.labels_cfg.get("tag", self.env_name)
        # 1) AirSim segmentation colormap (writes a timestamped CSV into cwd)
        before = set(glob.glob(str(self.dataset_dir / "airsim_segmentation_colormap_list_*.csv")))
        self._run("seg_colormap",
                  [sys.executable, self.seg_dir / "segmentation_generate_list.py"],
                  cwd=self.dataset_dir,
                  extra_env={"PYTHONPATH": str(self.seg_dir) + os.pathsep
                             + os.environ.get("PYTHONPATH", "")})
        if self.dry:
            colors_csv = self.dataset_dir / "airsim_segmentation_colormap_list_<ts>.csv"
        else:
            new = set(glob.glob(str(self.dataset_dir / "airsim_segmentation_colormap_list_*.csv"))) - before
            if not new:
                die("segmentation_generate_list.py produced no colormap CSV")
            colors_csv = Path(max(new, key=os.path.getmtime))

        # 2) UE actor Label->Name dump via the init_unreal.py file-watcher
        labels_csv = self.dataset_dir / f"ue_actor_label_to_name_{env_tag}.csv"
        if self.dry:
            log(f"DRY-RUN would request UE label dump -> {labels_csv}")
        else:
            request = self.ue_saved_dir / "label_dump_request.json"
            result = self.ue_saved_dir / "label_dump_result.json"
            result.unlink(missing_ok=True)
            request.write_text(json.dumps({"out_csv": str(labels_csv)}))
            timeout = self.labels_cfg.get("dump_timeout_s", 60)
            deadline = time.time() + timeout
            while time.time() < deadline and not result.is_file():
                time.sleep(1)
            if not result.is_file():
                die(f"UE label dump timed out after {timeout}s. Is the editor running with "
                    "Content/Python/init_unreal.py loaded? (restart the editor, or paste the "
                    "file into its Python console once)")
            res = json.loads(result.read_text())
            if not res.get("ok"):
                die(f"UE label dump failed: {res.get('error')}")
            log(f"UE label dump: {res['count']} actors -> {labels_csv}")

        # 3) Merge labels with colors
        merged = self.dataset_dir / f"label_color_map_{env_tag}.csv"
        self._run("label_merge",
                  [sys.executable, self.seg_dir / "itemlabel_to_color_csv.py",
                   "--labels_csv", labels_csv,
                   "--colors_csv", colors_csv,
                   "--out_csv", merged])
        log(f"labels stage complete -> {merged}")


def main():
    ap = argparse.ArgumentParser(description="End-to-end HERCULES dataset generation")
    ap.add_argument("config", help="YAML config (see configs/)")
    ap.add_argument("--stages", default=",".join(ALL_STAGES),
                    help=f"Comma-separated subset of: {','.join(ALL_STAGES)}")
    ap.add_argument("--dry-run", action="store_true",
                    help="Print every command without executing anything")
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    stages = [s.strip() for s in args.stages.split(",") if s.strip()]
    for s in stages:
        if s not in ALL_STAGES:
            die(f"unknown stage '{s}' (valid: {ALL_STAGES})")

    pipe = Pipeline(cfg, dry_run=args.dry_run)
    pipe.preflight(stages)
    started = time.time()
    if "collect" in stages:
        pipe.stage_collect()
    if "calibrate" in stages:
        pipe.stage_calibrate()
    if "post" in stages:
        pipe.stage_post()
    if "labels" in stages:
        pipe.stage_labels()
    log(f"all requested stages finished in {time.time() - started:.0f}s "
        f"-> {pipe.dataset_dir}")


if __name__ == "__main__":
    main()
