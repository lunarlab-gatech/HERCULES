#!/usr/bin/env python3
"""Pin segmentation IDs for the RA-L arena (contiguous semantic regions + river).

Run ONCE after each sim start, before the ROS stack, INSTEAD OF set_terrain_seg_ids.py:
    cd PythonClient/hero && python3 pin_arena_ral.py

Three passes:
  1. Terrain static mesh -> ID 321 (ground class, as in the legacy script).
  2. The 10 RAL_* region actors -> class seg IDs, by EXACT runtime name from
     configs/arena_ral_manifest.json (label tried first, runtime name as fallback).
  3. Every remaining StaticMeshActor_* matching a LEGACY patch-disc cluster (position +
     ground-z heuristic copied from set_terrain_seg_ids.py) -> ID 321, so the ~69 old
     CENTERSEQ discs read as plain ground instead of leaking their old semantics into
     the RA-L ground truth. RAL actors already pinned in pass 2 are excluded by name.

    class    seg ID  colour           cost (launch)
    ground   321     (79,159,95)      3.0
    rough    450     (110,255,191)    8.0   <- NEW: launch class_colors[2] updated
    mud      400     (127,110,127)    6.0
    water    410     (191,110,159)    9.0
    grass    420     (159,110,255)    4.5
    debris   430     (223,110,95)     7.0
    leaves   440     (79,110,79)      4.0
"""
import json
import math
import os

import setup_path  # noqa: F401
import hercules_cosysairsim as airsim

MANIFEST = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "../../ros2/src/HERCULES-ROS2-Mapping/configs/arena_ral_manifest.json")
PLAYER_START_UE = (8699.0, 16307.0)
LEGACY_CLUSTERS = [
    ("Mud", 5200, 10500), ("Mud", 12200, 19500), ("Mud", 9800, 22300),
    ("Water", 11800, 9800), ("Water", 4600, 20800), ("Water", 9000, 13600),
    ("Grass", 6800, 12800), ("Grass", 12600, 14500), ("Grass", 5400, 17800),
    ("Debris", 10500, 12200), ("Debris", 8200, 20500),
    ("Mud", 3900, 13800), ("Mud", 13300, 17300), ("Mud", 7600, 23200),
    ("Grass", 9500, 18500), ("Grass", 4200, 9300), ("Grass", 12500, 23000),
    ("Leaves", 8000, 11500), ("Leaves", 5800, 15200), ("Leaves", 11000, 21800),
]
TERRAIN_NAME = "StaticMeshActor_1"
TERRAIN_ID = 321
MATCH_RADIUS_M = 6.0
GROUND_Z_NED = (0.5, 2.5)


def connect():
    c = airsim.VehicleClient(port=41452, timeout_value=60)
    c.confirmConnection()
    return c


def main():
    client = connect()
    man = json.load(open(MANIFEST))

    # ---- pass 1: terrain -> ground ----
    gid = client.simGetSegmentationObjectID(TERRAIN_NAME)
    if gid != TERRAIN_ID:
        ok = client.simSetSegmentationObjectID(TERRAIN_NAME, TERRAIN_ID, False)
        print("terrain %s: ID %s -> %d : %s" % (TERRAIN_NAME, gid, TERRAIN_ID, ok))
        if not ok:
            print("WARNING: terrain pin failed -- ground class broken")
    else:
        print("terrain %s already ID %d" % (TERRAIN_NAME, TERRAIN_ID))

    # ---- pass 2: RAL region actors by exact name ----
    ral_names = set()
    n_ok = 0
    for a in man["actors"]:
        pinned = None
        for name in (a["label"], a["runtime"]):
            if client.simSetSegmentationObjectID(name, a["seg_id"], False):
                pinned = name
                break
        if pinned:
            n_ok += 1
            ral_names.update((a["label"], a["runtime"]))
            got = client.simGetSegmentationObjectID(pinned)
            flag = "" if got == a["seg_id"] else "  VERIFY-MISMATCH(read %s)" % got
            print("  %-24s -> ID %d via '%s'%s" % (a["label"], a["seg_id"], pinned, flag))
        else:
            print("  %-24s -> FAILED (tried %s, %s)" % (a["label"], a["label"], a["runtime"]))
    print("RAL actors pinned: %d/%d" % (n_ok, len(man["actors"])))
    if n_ok < len(man["actors"]):
        print("WARNING: unpinned RAL region(s) -- arena semantics incomplete; re-check "
              "runtime names in the manifest (editor restarts renumber only NEW actors)")

    # ---- pass 3: legacy discs -> ground ----
    ps = PLAYER_START_UE
    centres = [((x - ps[0]) / 100.0, (y - ps[1]) / 100.0) for _, x, y in LEGACY_CLUSTERS]
    legacy, fails = 0, 0
    for name in client.simListSceneObjects("StaticMeshActor.*"):
        if name in ral_names or name == TERRAIN_NAME:
            continue
        try:
            p = client.simGetObjectPose(name)
        except Exception:
            continue
        px, py, pz = p.position.x_val, p.position.y_val, p.position.z_val
        if math.isnan(px):
            continue
        d = min(math.hypot(px - cx, py - cy) for cx, cy in centres)
        if d <= MATCH_RADIUS_M and GROUND_Z_NED[0] < pz < GROUND_Z_NED[1]:
            if client.simSetSegmentationObjectID(name, TERRAIN_ID, False):
                legacy += 1
            else:
                fails += 1
    print("legacy discs neutralised to ground: %d | failures: %d" % (legacy, fails))
    if legacy < 60:
        print("NOTE: expected ~69 legacy discs; matched %d (RAL actors near old clusters "
              "are excluded by name, so a small shortfall is normal)" % legacy)


if __name__ == "__main__":
    main()
