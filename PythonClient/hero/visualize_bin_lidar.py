#!/usr/bin/env python3
"""
visualize_bin_lidar.py — Open3D viewer for AirSim/HERCULES LiDAR saved as .npy

Usage:
  python3 visualize_bin_lidar.py /path/to/points.npy --frame ned --voxel 0.05
  python3 visualize_bin_lidar.py /path/to/points.npy --frame enu
Options:
  --frame {ned,enu}   : input coordinate frame (AirSim LiDAR is typically NED: z down)
  --voxel <meters>    : voxel grid size for downsampling (0 = no downsample)
  --color {auto,z,intensity,gray,height}
                      : coloring mode (default auto)
  --no-axes           : hide coordinate axes helper
"""

import argparse
import numpy as np
import open3d as o3d
import os
import sys

def color_by_height_xyz(xyz: np.ndarray) -> np.ndarray:
    z = xyz[:, 2]
    zmin, zmax = np.percentile(z, 1), np.percentile(z, 99)
    span = max(1e-6, (zmax - zmin))
    z01 = np.clip((z - zmin) / span, 0.0, 1.0)
    # simple gradient: low→blue, mid→teal, high→red
    return np.stack([z01, 0.5 * (1.0 - z01), 1.0 - z01], axis=1).astype(np.float32)

def color_from_intensity(intensity: np.ndarray) -> np.ndarray:
    imin, imax = np.percentile(intensity, 1), np.percentile(intensity, 99)
    span = max(1e-6, (imax - imin))
    i01 = np.clip((intensity - imin) / span, 0.0, 1.0).astype(np.float32)
    return np.stack([i01, i01, i01], axis=1)

def load_points_npy(path: str) -> np.ndarray:
    arr = np.load(path, allow_pickle=False)
    if arr.ndim != 2 or arr.shape[1] not in (3, 4):
        raise ValueError(f"Expected Nx3 or Nx4 float array, got shape {arr.shape}")
    return arr.astype(np.float32)

def main():
    ap = argparse.ArgumentParser(description="Open3D viewer for LiDAR .npy (Nx3 or Nx4 [xyz, intensity])")
    ap.add_argument("npy_path", help=".npy file containing Nx3 or Nx4 points")
    ap.add_argument("--frame", choices=["ned", "enu"], default="ned",
                    help="Input frame. Use 'ned' for AirSim (z down), converted to z-up for viewing.")
    ap.add_argument("--voxel", type=float, default=0.0,
                    help="Voxel downsample size in meters (0 disables)")
    ap.add_argument("--color", choices=["auto", "z", "intensity", "gray", "height"], default="auto",
                    help="Coloring mode: auto=intensity if present else height; z=by z; intensity=by intensity; gray=uniform; height=by height")
    ap.add_argument("--no-axes", dest="no_axes", action="store_true",
                    help="Hide coordinate axes")
    args = ap.parse_args()

    if not os.path.exists(args.npy_path):
        print(f"File not found: {args.npy_path}", file=sys.stderr)
        sys.exit(1)

    pts = load_points_npy(args.npy_path)
    xyz = pts[:, :3]

    # Convert NED → viewer-friendly Z-up (simple Z flip keeps +X forward, +Y right)
    if args.frame == "ned":
        xyz = xyz * np.array([1.0, 1.0, -1.0], dtype=np.float32)

    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))

    # Choose colors
    colors = None
    if args.color == "gray":
        colors = np.full_like(xyz, 0.8)
    elif args.color in ("z", "height"):
        colors = color_by_height_xyz(xyz)
    elif args.color == "intensity":
        if pts.shape[1] == 4:
            colors = color_from_intensity(pts[:, 3])
        else:
            print("No intensity channel in data; falling back to height coloring.", file=sys.stderr)
            colors = color_by_height_xyz(xyz)
    elif args.color == "auto":
        if pts.shape[1] == 4:
            colors = color_from_intensity(pts[:, 3])
        else:
            colors = color_by_height_xyz(xyz)
    else:
        colors = color_by_height_xyz(xyz)

    pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float32))

    # Optional voxel downsample
    if args.voxel and args.voxel > 0.0:
        pcd = pcd.voxel_down_sample(voxel_size=float(args.voxel))

    geoms = [pcd]
    if not args.no_axes:
        geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0]))

    o3d.visualization.draw_geometries(
        geoms,
        window_name=f"LiDAR: {os.path.basename(args.npy_path)}",
        width=1280, height=800, left=60, top=60,
    )

if __name__ == "__main__":
    main()
