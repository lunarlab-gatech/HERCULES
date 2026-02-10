#!/usr/bin/env python3

import setup_path
import hercules as airsim
import os
import math
import msgpackrpc.error

# ---------- helpers ----------
def fmt(v: float) -> str:
    """Stable 1-decimal string without the '-0.0' annoyance."""
    s = f"{v:.1f}"
    return "0.0" if s == "-0.0" else s

def generate_voxel_patch(client, center, patch_size, resolution, output_file):
    """
    Create one 100x100x100 m cube centered at `center`.
    Try both common AirSim RPC signatures to dodge std::bad_cast.
    """
    px = int(patch_size)
    res = float(resolution)

    # (center, x, y, z, resolution, filename)
    try:
        client.simCreateVoxelGrid(center, px, px, px, res, output_file)
        print(f"Saved: {output_file}")
        return
    except msgpackrpc.error.RPCError:
        # (center, x, y, z, filename, resolution)
        try:
            client.simCreateVoxelGrid(center, px, px, px, output_file, res)
            print(f"Saved (alt-order): {output_file}")
            return
        except Exception as e:
            print(f"Failed to write {output_file}: {e}")

# ---------- main ----------
def main():
    # ---- User params ----
    world_size   = 1000        # X/Y span (m), same as your working script
    patch_size   = 100        # cube side (m)
    stack_height = 100        # total height to cover (m)
    resolution   = 1.0        # voxel size (m)
    world_center = (0.0, 0.0, 0.0)    # ground (NED Z = 0)
    output_dir   = "/home/sgarimella34/multi-robot-coordination/data_binvox_octomap/ausenv_semanticrag_1mcubed/"
    port         = 41452
    # ----------------------
    os.makedirs(output_dir, exist_ok=True)
    client = airsim.VehicleClient(port=port)

    num_xy = int(world_size / patch_size)
    num_z  = int(math.ceil(stack_height / patch_size))  # number of 100 m layers

    min_x = -world_size / 2
    min_y = -world_size / 2
    ground_z = world_center[2]

    for ix in range(num_xy):
        for iy in range(num_xy):
            # EXACTLY the same XY math as voxel_binvox_tesselation.py
            cx = min_x + ix * patch_size + patch_size / 2
            cy = min_y + iy * patch_size + patch_size / 2

            for iz in range(num_z):
                # Layer 0 center at 0.0; higher layers go negative (up in NED)
                cz = ground_z - iz * patch_size

                center = airsim.Vector3r(cx, cy, cz)

                fname = f"patch_{fmt(cx)}_{fmt(cy)}_layer{iz}.binvox"
                outp  = os.path.join(output_dir, fname)

                print(f"Generating {fname} at center ({fmt(cx)}, {fmt(cy)}, {fmt(cz)})")
                generate_voxel_patch(client, center, patch_size, resolution, outp)

if __name__ == "__main__":
    main()
