#!/usr/bin/env python3
"""
open3d_lidar_viewer.py

Load and visualize LiDAR .npy files (Nx3 float arrays in meters) with Open3D.
Designed for data saved by HERCULES/HERCULES collectors where LiDAR points
are stored under ".../<vehicle>/lidar/<timestamp>.npy".

Usage:
  python3 open3d_lidar_viewer.py /path/to/file_or_dir [--voxel 0.05] [--min 0.2] [--max 120]
  python3 open3d_lidar_viewer.py /dataset/root/Drone1/lidar --play
  python3 open3d_lidar_viewer.py /dataset/root --play  # auto-finds */lidar/*.npy

Keys in the viewer:
  [ → ] : next frame     [ ← ] : previous frame
  [ space ] : toggle autoplay
  [ s ] : save current frame as .ply next to the .npy
  [ q ] : quit

Install:
  pip install open3d==0.18.0  # or a compatible version available to you
"""

import argparse
import glob
import os
import time
from typing import List, Optional

import numpy as np
import open3d as o3d


def find_npy_files(path: str) -> List[str]:
    """
    Return a sorted list of LiDAR .npy files.
    Preference order:
      1) If 'lidar' is in the path, only search under that folder.
      2) Otherwise search recursively for '*/lidar/*.npy'.
      3) As a fallback, include any *.npy whose shape matches Nx3 when loaded.
    """
    if os.path.isfile(path) and path.endswith(".npy"):
        return [path]

    paths = []  # candidates
    if os.path.isdir(path):
        # Prefer lidar subfolders
        if "lidar" + os.sep in path or path.endswith("lidar"):
            paths = glob.glob(os.path.join(path, "*.npy"))
        else:
            paths = glob.glob(os.path.join(path, "**", "lidar", "*.npy"), recursive=True)

        # Fallback: allow any *.npy (we'll filter by shape)
        if not paths:
            paths = glob.glob(os.path.join(path, "**", "*.npy"), recursive=True)

    # Natural sort by timestamp-like filename
    paths = sorted(paths)

    # Filter to Nx3 point arrays
    filtered = []
    for p in paths:
        try:
            arr = np.load(p, mmap_mode="r")
            if arr.ndim == 2 and arr.shape[1] == 3:
                filtered.append(p)
        except Exception:
            # Non-numeric or incompatible .npy
            continue

    return filtered


def build_point_cloud(
    pts: np.ndarray,
    voxel: Optional[float] = None,
    min_range: Optional[float] = None,
    max_range: Optional[float] = None,
) -> o3d.geometry.PointCloud:
    """
    Convert Nx3 numpy array to an Open3D point cloud with basic filtering and optional downsampling.
    """
    # Ensure float64 for Open3D
    pts = np.asarray(pts, dtype=np.float64)

    # Remove NaNs/Infs
    finite_mask = np.isfinite(pts).all(axis=1)

    # Range filtering
    ranges = np.linalg.norm(pts, axis=1)
    if min_range is not None:
        finite_mask &= ranges >= float(min_range)
    if max_range is not None:
        finite_mask &= ranges <= float(max_range)

    pts = pts[finite_mask]
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))

    # Optional voxel downsample
    if voxel and voxel > 0:
        pcd = pcd.voxel_down_sample(voxel_size=float(voxel))

    # Optional simple height-based coloring (Z axis)
    if len(pcd.points) > 0:
        z = np.asarray(pcd.points)[:, 2]
        z_min, z_max = np.percentile(z, [5, 95]) if z.size > 100 else (z.min(), z.max() if z.max() > z.min() else z.min() + 1e-6)
        z_clamped = np.clip((z - z_min) / (z_max - z_min + 1e-12), 0.0, 1.0)
        colors = np.stack([z_clamped, 1.0 - z_clamped, 0.5 * np.ones_like(z_clamped)], axis=1)
        pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


def visualize_sequence(
    files: List[str],
    voxel: Optional[float],
    min_range: Optional[float],
    max_range: Optional[float],
    play: bool,
    fps: float,
    frame_size: float,
):
    """
    Visualize a sequence of LiDAR frames with keyboard controls.
    """
    if not files:
        raise SystemExit("No LiDAR .npy files found.")

    idx = 0
    autoplay = play
    dt = 1.0 / max(1e-6, fps)

    # Load first frame
    pts = np.load(files[idx])
    pcd = build_point_cloud(pts, voxel=voxel, min_range=min_range, max_range=max_range)
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size)

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="Open3D LiDAR Viewer", width=1280, height=800)
    vis.add_geometry(pcd)
    vis.add_geometry(frame)

    def set_title():
        vis.get_render_option().point_size = 1.0
        vis.get_view_control().set_zoom(0.8)
        title = f"[{idx+1}/{len(files)}] {os.path.basename(files[idx])}"
        try:
            vis.get_window().window_name = title  # works in newer Open3D
        except Exception:
            pass

    set_title()

    def load_index(new_idx: int):
        nonlocal idx, pcd
        idx = new_idx % len(files)
        pts_local = np.load(files[idx])
        new_pcd = build_point_cloud(pts_local, voxel=voxel, min_range=min_range, max_range=max_range)
        pcd.points = new_pcd.points
        pcd.colors = new_pcd.colors
        vis.update_geometry(pcd)
        set_title()

    # Key callbacks
    def on_next(vis_):
        load_index(idx + 1)
        return False

    def on_prev(vis_):
        load_index(idx - 1)
        return False

    def on_toggle_play(vis_):
        nonlocal autoplay
        autoplay = not autoplay
        return False

    def on_save(vis_):
        ply_path = os.path.splitext(files[idx])[0] + ".ply"
        o3d.io.write_point_cloud(ply_path, pcd, write_ascii=False, compressed=True)
        print(f"Saved: {ply_path}")
        return False

    vis.register_key_callback(ord('S'), on_save)
    vis.register_key_callback(262, on_next)  # Right arrow
    vis.register_key_callback(263, on_prev)  # Left arrow
    vis.register_key_callback(32, on_toggle_play)  # Space

    while True:
        vis.poll_events()
        vis.update_renderer()
        if autoplay:
            time.sleep(dt)
            load_index(idx + 1)
        else:
            time.sleep(0.01)
        # Close if window destroyed
        if not vis.poll_events():
            break

    vis.destroy_window()


def visualize_single(
    file_path: str,
    voxel: Optional[float],
    min_range: Optional[float],
    max_range: Optional[float],
    frame_size: float,
):
    pts = np.load(file_path)
    pcd = build_point_cloud(pts, voxel=voxel, min_range=min_range, max_range=max_range)
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size)
    o3d.visualization.draw_geometries([pcd, frame], window_name=os.path.basename(file_path))


def main():
    parser = argparse.ArgumentParser(description="Open3D viewer for LiDAR .npy files (Nx3).")
    parser.add_argument("path", help="Path to a LiDAR .npy file, a 'lidar' folder, a vehicle folder, or the dataset root.")
    parser.add_argument("--voxel", type=float, default=0.0, help="Optional voxel size for downsampling (meters).")
    parser.add_argument("--min", dest="min_range", type=float, default=None, help="Minimum range filter (meters).")
    parser.add_argument("--max", dest="max_range", type=float, default=None, help="Maximum range filter (meters).")
    parser.add_argument("--play", action="store_true", help="Autoplay through frames when a folder is given.")
    parser.add_argument("--fps", type=float, default=10.0, help="Playback rate when --play is enabled.")
    parser.add_argument("--frame-size", type=float, default=1.0, help="Coordinate frame size in meters.")
    args = parser.parse_args()

    files = find_npy_files(args.path)

    if not files:
        raise SystemExit("No LiDAR .npy files found. Point the script to a file, a 'lidar' folder, a vehicle folder, or the dataset root.")

    if len(files) == 1:
        visualize_single(files[0], args.voxel, args.min_range, args.max_range, args.frame_size)
    else:
        visualize_sequence(files, args.voxel, args.min_range, args.max_range, args.play, args.fps, args.frame_size)


if __name__ == "__main__":
    main()
