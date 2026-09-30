#!/usr/bin/env python3
"""Pin segmentation IDs for ArenaProcV1 (procedural arena, 7 ISM class layers).

Must run once per sim Play (seg IDs are per-session). Same class->ID map as the frozen
arena (pin_arena_ral.py) so the extractor's launch class_colors keep working, plus road,
which the frozen arena never had:

    class    seg ID  colour (palette)
    ground   321     (79,159,95)    <- the flat terrain plane StaticMeshActor_1
    road     460     (read from simGetSegmentationColorMap at pin time)
    rough    450     (110,255,191)
    mud      400     (127,110,127)
    water    410     (191,110,159)
    grass    420     (159,110,255)
    debris   430     (223,110,95)
    leaves   440     (79,110,79)

The 7 layer actors are InstancedStaticMesh (one actor per class = one ID for every
instance), addressable by exact name because the editor actors were renamed to their
labels. The Landscape actor cannot be annotated in this build (docs/annotation.md:15) but
sits below the pinned plane, so cameras never see it inside the arena.

Prints "PROCV1 actors pinned: N/7" as the run_experiment.sh gate sentinel.
"""
import sys

import setup_path  # noqa: F401
import hercules_cosysairsim as airsim

TERRAIN_NAME = "StaticMeshActor_1"
TERRAIN_ID = 321
# The ISM visual layers are INVISIBLE to the segmentation pass (verified live: facing the
# largest water body, zero water-colour pixels; the seg camera saw the ground plane through
# them). The pinned semantic geometry is therefore the SemSheet_* static meshes placed 2 cm
# under the ISM quads: scene cameras see only the textured quads, seg cameras only the
# sheets.
LAYER_IDS = {
    "SemSheet_road": 460,
    "SemSheet_rough": 450,
    "SemSheet_mud": 400,
    "SemSheet_water": 410,
    "SemSheet_grass": 420,
    "SemSheet_debris": 430,
    "SemSheet_leaves": 440,
}


def main():
    c = airsim.VehicleClient(port=41452, timeout_value=60)
    c.confirmConnection()

    ok = c.simSetSegmentationObjectID(TERRAIN_NAME, TERRAIN_ID, False)
    print("terrain %s -> %d : %s" % (TERRAIN_NAME, TERRAIN_ID, ok))
    if not ok:
        print("WARNING: terrain pin failed -- ground class broken")

    n = 0
    for name, sid in sorted(LAYER_IDS.items()):
        if c.simSetSegmentationObjectID(name, sid, False):
            got = c.simGetSegmentationObjectID(name)
            flag = "" if got == sid else "  MISMATCH got=%d" % got
            print("  %-22s -> ID %d%s" % (name, sid, flag))
            n += 1
        else:
            print("  %-22s -> ID %d FAILED" % (name, sid))

    # report road's palette colour so the launch class_colors[1] can be kept in sync
    try:
        cmap = c.simGetSegmentationColorMap()
        r, g, b = (int(x) for x in cmap[460])
        print("road ID 460 palette colour: (%d,%d,%d)" % (r, g, b))
    except Exception as e:
        print("colormap unavailable (%s) -- verify road colour by capture" % type(e).__name__)

    print("PROCV1 actors pinned: %d/7" % n)
    sys.exit(0 if n == 7 else 1)


if __name__ == "__main__":
    main()
