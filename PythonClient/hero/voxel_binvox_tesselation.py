#!/usr/bin/env python3

import setup_path
import hercules as airsim
import os

def generate_voxel_patch(client, center, patch_size, resolution, output_file):
    # Create the voxel grid patch centered at "center"
    client.simCreateVoxelGrid(center, patch_size, patch_size, patch_size, resolution, output_file)

def main():
    # Parameters
    world_size = 700       # Total extent in meters for X and Y (world assumed to be square)
    patch_size = 100        # Each patch covers 100m x 100m x 100m
    resolution = 0.5       # Voxel resolution in meters
    
    # The world’s geometric center (from your PlayerStart in UE) in meters
    # world_center = (0, 0, 0) #original
    world_center = (0, 0, 0)

    # For X and Y, we assume the world is centered at 0,0 so:
    min_x = -world_size / 2
    min_y = -world_size / 2
    # We'll fix the Z coordinate to the world_center's Z
    fixed_z = world_center[2]
    
    # Number of patches in X and Y (assuming world_size is a multiple of patch_size)
    num_patches_xy = int(world_size / patch_size)
    
    # Make sure the output directory exists
    output_dir = "/home/sgarimella34/multi-robot-coordination/data_binvox_octomap/customcity_0p5mcubed"
    os.makedirs(output_dir, exist_ok=True)
    
    # Create a single client for efficiency
    client = airsim.VehicleClient(port=41452)
    
    # Loop over the grid of patches in XY only.
    for ix in range(num_patches_xy):
        for iy in range(num_patches_xy):
            # Compute the center for each patch in XY; use fixed_z for Z.
            cx = min_x + (patch_size * ix) + patch_size / 2
            cy = min_y + (patch_size * iy) + patch_size / 2
            cz = fixed_z
            center = airsim.Vector3r(cx, cy, cz)
            
            # Define a unique filename for this patch.
            output_file = os.path.join(output_dir, f"patch_{cx}_{cy}.binvox") #using center of tile to name
            print(f"Generating patch at center ({cx}, {cy}, {cz}) into file {output_file}")
            
            # Generate the voxel grid for this patch.
            generate_voxel_patch(client, center, patch_size, resolution, output_file)

if __name__ == '__main__':
    main()
