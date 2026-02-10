#!/usr/bin/env python3
import argparse
import sys
import random

import numpy as np
import setup_path 
import hercules as airsim

def radiance(absoluteTemperature, emissivity, dx=0.01, response=None):
    """Compute spectral radiance (Planck x emissivity x camera response)."""
    wavelength = np.arange(8, 14, dx)
    c1 = 1.19104e8
    c2 = 1.43879e4

    # Planck’s law x emissivity x camera response (if provided)
    factor = response if response is not None else 1.0
    L = factor * emissivity * (
        c1 / ((wavelength**5) * (np.exp(c2 / (wavelength * absoluteTemperature)) - 1))
    )

    # integrate over lambda to get scalar radiance
    if absoluteTemperature.ndim > 1:
        return L, np.trapz(L, dx=dx, axis=1)
    else:
        return L, np.trapz(L, dx=dx)

def get_new_temp_emiss_from_radiance(tempEmissivity, response):
    """
    Given [[name, T, eps],...], compute a [name, count] array
    where count ∈ [0,255] is the integrated radiance mapped to 0-255.
    """
    # reshape for vectorized radiance
    temps = tempEmissivity[:, 1].astype(float).reshape(-1, 1)
    ems   = tempEmissivity[:, 2].astype(float).reshape(-1, 1)

    # compute radiance → scalar values
    _, values = radiance(temps, ems, response=response)
    counts = ((values / values.max()) * 255).astype(np.uint8)

    # return [[name, count], ...]
    return np.hstack((
        tempEmissivity[:, 0].reshape(-1, 1),
        counts.reshape(-1, 1),
    ))

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--settings", required=True,
                   help="(unused) path to settings.json; kept for compatibility")
    args = p.parse_args()

    # connect to AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # 1) load camera response
    try:
        response = np.load("camera_response.npy")
        print("Loaded camera_response.npy")
    except Exception:
        print("camera_response.npy not found, defaulting to flat response")
        response = None

    # 2) list all scene objects
    object_names = client.simListSceneObjects()

    # 3) build mesh‐type roots by first TWO tokens, skip numeric seconds
    roots = []
    for name in object_names:
        parts = name.split("_")
        if len(parts) >= 2 and not parts[1].isdigit():
            mesh_root = f"{parts[0]}_{parts[1]}"
            if mesh_root not in roots:
                roots.append(mesh_root)

    if not roots:
        print("No meshes found to map. Exiting.")
        sys.exit(1)

    # 4) prepare random T & ε for each mesh‐type
    segIdDict = { root: root.lower() for root in roots }
    tempEmissivity = np.array([
        [root.lower(), random.uniform(285, 310), random.uniform(0.85, 0.98)]
        for root in roots
    ], dtype=object)

    # 5) compute new digital counts
    tempEmissivityNew = get_new_temp_emiss_from_radiance(tempEmissivity, response)

    # 6) reset all IDs to zero
    if not client.simSetSegmentationObjectID(r"[\w]*", 0, True):
        # fallback: clear per‐mesh
        for mesh in object_names:
            client.simSetSegmentationObjectID(mesh, 0, False)

    # 7) assign each mesh‐instance individually
    reassigned = 0
    for root, lower in segIdDict.items():
        # lookup the count for this mesh‐class
        idx = np.where(tempEmissivityNew[:, 0] == lower)[0]
        if idx.size == 0:
            continue
        objectID = int(tempEmissivityNew[idx[0], 1])

        # for every object name that begins with root + "_"
        for mesh_name in object_names:
            if mesh_name.startswith(root + "_"):
                client.simSetSegmentationObjectID(mesh_name, objectID, False)
                reassigned += 1

    print(f"Reassigned {reassigned} object IDs")

if __name__ == "__main__":
    main()
