#!/usr/bin/env python3
"""
Build the two occupancy grid maps the multi-robot trajectory planner
(octomap_server trajectory_planner_OGM_node) plans on:

  <name>_ground.pgm/.yaml          where the UGVs can drive
  <name>_drone_<alt>m.pgm/.yaml    obstacles in the drones' altitude band

The simulator samples the environment into a voxel grid (simCreateVoxelGrid,
a binvox file); this script converts it. Maps use the ROS map_server format in
the planner's frame: x = east (AirSim Y), y = north (AirSim X), metres from the
PlayerStart, which is where the planner expects them.

Ground map: a column's ground is the top of its lowest run of occupied voxels.
Cells are drivable when they are reachable from a UGV start through neighbours
whose ground differs by at most --step, with nothing occupied between --step
and --ugv-height above the ground. Every other cell is an obstacle, so box
tops, walls and holes in the floor are excluded rather than treated as floor;
the planner only avoids obstacle cells.
Drone map: a cell is an obstacle if any voxel lies within --drone-margin of
--drone-altitude (metres above the PlayerStart).

Capture from a running simulator (it writes the .binvox), then convert:
  python make_planning_maps.py --name blocks --size 120 --settings ros2/settings/hero_blocks_team.json
Convert an earlier capture without the simulator:
  python make_planning_maps.py --binvox trajectory_data/maps/blocks.binvox
"""

import argparse
import json
import os
from collections import deque

import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

OCCUPIED, FREE = 0, 254  # PGM grey levels, as ROS map_server reads them


def capture(args, binvox_path):
    """Ask the simulator to write the voxel grid; returns the capture metadata."""
    import setup_path  # noqa: F401  (puts the repo's hercules_cosysairsim on sys.path)
    import hercules_cosysairsim as airsim

    z_size = int(round(args.z_max - args.z_min))
    center = (args.center[0], args.center[1], -(args.z_min + args.z_max) / 2.0)  # NED
    client = airsim.VehicleClient(ip=args.ip, port=args.port)
    client.confirmConnection()
    print(f"Sampling a {args.size} x {args.size} x {z_size} m voxel grid at {args.res} m "
          f"around NED {center}; this can take a few minutes...")
    ok = client.simCreateVoxelGrid(airsim.Vector3r(*center), args.size, args.size, z_size,
                                   args.res, binvox_path)
    if not ok:
        raise SystemExit(f"The simulator could not write {binvox_path}")
    meta = {"center_ned": center, "size_xy": args.size, "size_z": z_size, "res": args.res}
    with open(binvox_path + ".json", "w") as f:
        json.dump(meta, f, indent=2)
    return meta


def read_binvox(path, meta):
    """Decode a simulator voxel grid into occ[i_north, j_east, k_up] (bool)."""
    with open(path, "rb") as f:
        raw = f.read()
    header_end = raw.index(b"data\n") + len(b"data\n")
    dims = None
    for line in raw[:header_end].decode().splitlines():
        if line.startswith("dim "):
            dims = [int(v) for v in line.split()[1:]]
    nx, nz, ny = dims  # the simulator writes "dim nx nz ny"
    pairs = np.frombuffer(raw[header_end:], dtype=np.uint8)
    flat = np.repeat(pairs[0::2], pairs[1::2]).astype(bool)
    if flat.size != nx * ny * nz:
        raise SystemExit(f"{path}: expected {nx * ny * nz} voxels, decoded {flat.size}")
    # The simulator stores voxel (i, j, k) at i + nx * (k + nz * j).
    occ = flat.reshape(ny, nz, nx).transpose(2, 0, 1)  # -> [i, j, k]
    cx, cy, cz = meta["center_ned"]
    res = meta["res"]
    north = cx + (np.arange(nx) - nx // 2) * res
    east = cy + (np.arange(ny) - ny // 2) * res
    up = -cz + (np.arange(nz) - nz // 2) * res
    return occ, north, east, up, res


def ground_map(occ, north, east, up, res, seeds_ne, step, ugv_height):
    nx, ny, nz = occ.shape
    has_ground = occ.any(axis=2)
    lowest = np.argmax(occ, axis=2)
    # Top of the lowest occupied run: first free voxel at or above `lowest`, minus one.
    k = np.arange(nz)[None, None, :]
    free_above = (~occ) & (k > lowest[:, :, None])
    top_k = np.where(free_above.any(axis=2), np.argmax(free_above, axis=2) - 1, nz - 1)
    ground_h = up[0] + top_k * res
    # Body envelope: anything occupied between step and ugv_height above the ground.
    rel = up[None, None, :] - ground_h[:, :, None]
    blocked = (occ & (rel > step) & (rel <= ugv_height)).any(axis=2)
    open_cell = has_ground & ~blocked

    reachable = np.zeros((nx, ny), dtype=bool)
    queue = deque()
    for n, e in seeds_ne:
        i = int(round((n - north[0]) / res))
        j = int(round((e - east[0]) / res))
        if not (0 <= i < nx and 0 <= j < ny) or not open_cell[i, j]:
            raise SystemExit(f"Start N={n} E={e} is outside the map or not drivable; "
                             "check --size/--center, --z-min and the vehicle positions.")
        reachable[i, j] = True
        queue.append((i, j))
    while queue:
        i, j = queue.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if (0 <= a < nx and 0 <= b < ny and not reachable[a, b] and open_cell[a, b]
                    and abs(ground_h[a, b] - ground_h[i, j]) <= step):
                reachable[a, b] = True
                queue.append((a, b))

    grid = np.full((nx, ny), OCCUPIED, dtype=np.uint8)
    grid[reachable] = FREE
    return grid


def drone_map(occ, up, altitude, margin):
    band = (up >= altitude - margin) & (up <= altitude + margin)
    if not band.any():
        raise SystemExit(f"--drone-altitude {altitude} is outside the sampled heights "
                         f"[{up[0]:.1f}, {up[-1]:.1f}]; raise --z-max.")
    grid = np.full(occ.shape[:2], FREE, dtype=np.uint8)
    grid[occ[:, :, band].any(axis=2)] = OCCUPIED
    return grid


def write_map(grid, north, east, res, out_dir, stem):
    """grid[i_north, j_east] -> map_server PGM (top row = north) + YAML."""
    image = np.flipud(grid)  # rows north-to-south; columns west-to-east
    pgm = os.path.join(out_dir, stem + ".pgm")
    with open(pgm, "wb") as f:
        f.write(f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode())
        f.write(np.ascontiguousarray(image).tobytes())
    origin = [float(east[0] - res / 2), float(north[0] - res / 2), 0.0]
    with open(os.path.join(out_dir, stem + ".yaml"), "w") as f:
        f.write(f"image: {stem}.pgm\nresolution: {res}\norigin: {origin}\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    print(f"Wrote {pgm} ({image.shape[1]} x {image.shape[0]} cells, "
          f"{int((grid == OCCUPIED).sum())} occupied)")


def ugv_starts(settings_path):
    """(north, east) of every Husky* vehicle in a settings.json."""
    with open(settings_path) as f:
        vehicles = json.load(f).get("Vehicles", {})
    return [(float(v.get("X", 0.0)), float(v.get("Y", 0.0)))
            for name, v in vehicles.items() if name.startswith("Husky")]


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--binvox", help="convert this capture instead of sampling the simulator")
    p.add_argument("--name", default="map", help="capture name (default: %(default)s)")
    p.add_argument("--out", default=os.path.join(REPO_ROOT, "trajectory_data", "maps"),
                   help="output folder (default: <repo>/trajectory_data/maps)")
    p.add_argument("--ip", default="127.0.0.1")
    p.add_argument("--port", type=int, default=41451)
    p.add_argument("--center", type=float, nargs=2, default=(0.0, 0.0), metavar=("X", "Y"),
                   help="NED centre of the square in metres from the PlayerStart (default: 0 0)")
    p.add_argument("--size", type=int, default=100, help="side of the square in metres (default: %(default)s)")
    p.add_argument("--res", type=float, default=0.5, help="voxel size in metres (default: %(default)s)")
    p.add_argument("--z-min", type=float, default=-5.0,
                   help="lowest height sampled, metres above the PlayerStart; must be below the ground (default: %(default)s)")
    p.add_argument("--z-max", type=float, default=15.0,
                   help="highest height sampled, metres above the PlayerStart (default: %(default)s)")
    p.add_argument("--settings", help="settings.json whose Husky* positions seed the drivable area")
    p.add_argument("--seed", type=float, nargs=2, action="append", metavar=("X", "Y"),
                   help="extra NED start position for the drivable area (repeatable)")
    p.add_argument("--step", type=float, default=0.5, help="ground height change a UGV can climb (default: %(default)s)")
    p.add_argument("--ugv-height", type=float, default=1.0, help="UGV body height (default: %(default)s)")
    p.add_argument("--drone-altitude", type=float, default=10.0,
                   help="planner drone_altitude, metres above the PlayerStart (default: %(default)s)")
    p.add_argument("--drone-margin", type=float, default=2.0,
                   help="clearance above and below the drone altitude (default: %(default)s)")
    args = p.parse_args()

    os.makedirs(args.out, exist_ok=True)
    if args.binvox:
        binvox_path = os.path.abspath(args.binvox)
        with open(binvox_path + ".json") as f:
            meta = json.load(f)
        name = os.path.splitext(os.path.basename(binvox_path))[0]
    else:
        name = args.name
        binvox_path = os.path.join(os.path.abspath(args.out), name + ".binvox")
        meta = capture(args, binvox_path)

    occ, north, east, up, res = read_binvox(binvox_path, meta)
    seeds = ugv_starts(args.settings) if args.settings else []
    seeds += [tuple(s) for s in (args.seed or [])]
    if not seeds:
        seeds = [(0.0, 0.0)]
    write_map(ground_map(occ, north, east, up, res, seeds, args.step, args.ugv_height),
              north, east, res, args.out, f"{name}_ground")
    write_map(drone_map(occ, up, args.drone_altitude, args.drone_margin),
              north, east, res, args.out, f"{name}_drone_{args.drone_altitude:g}m")


if __name__ == "__main__":
    main()
