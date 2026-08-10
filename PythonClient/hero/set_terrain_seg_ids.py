#!/usr/bin/env python3
"""Pin segmentation IDs for the semantic terrain patches (mud/water/grass/debris/leaves).

Run ONCE after each sim start (editor Play or packaged), before/at ROS stack launch:
    cd PythonClient/hero && python3 set_terrain_seg_ids.py

Why: the Dirichlet pipeline classifies seg pixels by EXACT colour match (class_colors in
mapping.launch.py, K=8). The patch discs were spawned in the editor with LABELS Sem<Type>_* but
their runtime OBJECT names are anonymous StaticMeshActor_NN (labels are editor-only), and this
Cosys build's simSetSegmentationObjectID only matches EXACT names (the regex flag returns False,
even for ".*"). So the patches are identified BY POSITION (cluster centres are fixed level
constants) and pinned by exact object name. The terrain static mesh (StaticMeshActor_1 -- the
level's "landscape" is a baked static mesh; the ALandscape actor is not in the seg registry) is
verified/pinned to ID 321 so the ground class (79,159,95) survives scene edits.

    class    seg ID  colour           cost (launch)
    Mud      400     (127,110,127)    6.0
    Water    410     (191,110,159)    9.0
    Grass    420     (159,110,255)    4.5
    Debris   430     (223,110,95)     7.0
    Leaves   440     (79,110,79)      4.0
    ground   321     (79,159,95)      3.0
"""
import math

import setup_path  # noqa: F401
import hercules_cosysairsim as airsim

# cluster centres in UE cm (level constants; see the SemanticTerrain patch layout)
PLAYER_START_UE = (8699.0, 16307.0)
CLUSTERS = [
    ("Mud", 5200, 10500), ("Mud", 12200, 19500), ("Mud", 9800, 22300),
    ("Water", 11800, 9800), ("Water", 4600, 20800), ("Water", 9000, 13600),
    ("Grass", 6800, 12800), ("Grass", 12600, 14500), ("Grass", 5400, 17800),
    ("Debris", 10500, 12200), ("Debris", 8200, 20500),
    ("Mud", 3900, 13800), ("Mud", 13300, 17300), ("Mud", 7600, 23200),
    ("Grass", 9500, 18500), ("Grass", 4200, 9300), ("Grass", 12500, 23000),
    ("Leaves", 8000, 11500), ("Leaves", 5800, 15200), ("Leaves", 11000, 21800),
]
IDS = {"Mud": 400, "Water": 410, "Grass": 420, "Debris": 430, "Leaves": 440}
TERRAIN_NAME = "StaticMeshActor_1"
TERRAIN_ID = 321
MATCH_RADIUS_M = 6.0            # patch discs lie within ~4.5 m of their cluster centre
GROUND_Z_NED = (0.5, 2.5)       # discs sit at ground level: z_ned ~ +1.5 below PlayerStart


def connect():
    """Hero SimMode: drone RPC 41451, car RPC 41452; with no drone the 41451 server is absent."""
    try:
        c = airsim.VehicleClient(timeout_value=10)
        c.confirmConnection()
        return c
    except Exception:
        print("41451 not responding, falling back to car port 41452")
        c = airsim.VehicleClient(port=41452, timeout_value=60)
        c.confirmConnection()
        return c


def main():
    client = connect()
    ps = PLAYER_START_UE
    centres = [(t, (x - ps[0]) / 100.0, (y - ps[1]) / 100.0) for t, x, y in CLUSTERS]

    # ---- terrain ground class ----
    gid = client.simGetSegmentationObjectID(TERRAIN_NAME)
    if gid == TERRAIN_ID:
        print("terrain %s already ID %d (ground class intact)" % (TERRAIN_NAME, TERRAIN_ID))
    else:
        ok = client.simSetSegmentationObjectID(TERRAIN_NAME, TERRAIN_ID, False)
        print("terrain %s: ID %s -> %d : %s" % (TERRAIN_NAME, gid, TERRAIN_ID, ok))
        if not ok:
            print("WARNING: could not pin the terrain -- ground class (79,159,95) may be broken; "
                  "re-record class_colors if the map stays empty")

    # ---- patches by position, pinned by exact name ----
    assigned = {t: 0 for t in IDS}
    fails = 0
    for name in client.simListSceneObjects("StaticMeshActor.*"):
        try:
            p = client.simGetObjectPose(name)
        except Exception:
            continue
        px, py, pz = p.position.x_val, p.position.y_val, p.position.z_val
        if math.isnan(px):
            continue
        t, d = min(((t, math.hypot(px - cx, py - cy)) for t, cx, cy in centres),
                   key=lambda v: v[1])
        if d <= MATCH_RADIUS_M and GROUND_Z_NED[0] < pz < GROUND_Z_NED[1]:
            if client.simSetSegmentationObjectID(name, IDS[t], False):
                assigned[t] += 1
            else:
                fails += 1
    total = sum(assigned.values())
    print("patches pinned per type:", assigned, "| failures:", fails)
    if total < 60:
        print("WARNING: expected ~69 patch discs, pinned %d -- did PlayerStart move or the level "
              "change? (cluster centres are level constants)" % total)


if __name__ == "__main__":
    main()
