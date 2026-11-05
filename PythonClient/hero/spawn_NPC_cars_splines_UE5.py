import unreal
import random
import math

# ─── CONFIG ─────────────────────────────────────────────────────────────────────
# Folder holding the BP_VehicleAI* classes: the released Vehicle Behavior package installs them under
# /Game/HERCULES/Agents/Vehicles; the original HERCULES city project kept them in /Game/CustomHumanPedestrians.
VEHICLE_FOLDER       = next((f for f in ("/Game/HERCULES/Agents/Vehicles", "/Game/CustomHumanPedestrians")
                             if unreal.EditorAssetLibrary.does_directory_exist(f)), "/Game/HERCULES/Agents/Vehicles")
SPLINE_KEYWORD       = "SplineRoad"
INSTANCES_PER_SPLINE = 10
MIN_SPAWN_DISTANCE   = 500.0   # cm
MAX_SPAWN_ATTEMPTS   = 100     # per spline

# Blueprints to skip (base names only)
EXCLUDE_LIST = {"BP_VehicleAI_van2", "BP_VehicleAI_Van"}

asset_lib = unreal.EditorAssetLibrary
actor_sys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)

# ─── 1) Record existing vehicle locations (across runs) ────────────────────────
existing_vehicle_locations = []
for actor in actor_sys.get_all_level_actors():
    lbl = actor.get_actor_label()
    if lbl.startswith("BP_VehicleAI_"):
        existing_vehicle_locations.append(actor.get_actor_location())

# ─── 2) Discover & load vehicle Blueprint classes ──────────────────────────────
all_paths = asset_lib.list_assets(VEHICLE_FOLDER, recursive=False, include_folder=False)
bp_classes = {}
for path in all_paths:
    pkg, name = path.split(".", 1)  # "/Game/.../BP_XXX.BP_XXX"
    if not name.startswith("BP_VehicleAI") or name in EXCLUDE_LIST:
        continue
    bp_cls = asset_lib.load_blueprint_class(pkg)
    if bp_cls:
        key = pkg.rsplit("/", 1)[-1]
        bp_classes[key] = bp_cls
    else:
        unreal.log_error(f"[SpawnCars] Failed to load Blueprint class at '{pkg}'")

if not bp_classes:
    unreal.log_error("[SpawnCars] No valid vehicle Blueprint classes found; aborting.")
    raise SystemExit

# ─── 3) Find all spline components ──────────────────────────────────────────────
splines = []
for actor in actor_sys.get_all_level_actors():
    lbl      = actor.get_actor_label()
    cls_name = actor.get_class().get_name()
    if SPLINE_KEYWORD in lbl or SPLINE_KEYWORD in cls_name:
        comps = actor.get_components_by_class(unreal.SplineComponent)
        if comps:
            splines.append(comps[0])

if not splines:
    unreal.log_error(f"[SpawnCars] No spline actors containing '{SPLINE_KEYWORD}'; aborting.")
    raise SystemExit

# ─── helper: Euclidean distance between two FVectors ───────────────────────────
def distance(v1, v2):
    return math.sqrt((v1.x - v2.x)**2 + (v1.y - v2.y)**2 + (v1.z - v2.z)**2)

# ─── 4) Spawn & rename vehicles with correct orientation and no collisions ──────
used_labels = {a.get_actor_label() for a in actor_sys.get_all_level_actors()}

for spline in splines:
    length        = spline.get_spline_length()
    new_distances = []
    attempts      = 0

    # Poisson‐disc–like sampling along the spline
    while len(new_distances) < INSTANCES_PER_SPLINE and attempts < MAX_SPAWN_ATTEMPTS:
        attempts += 1
        d   = random.uniform(0.0, length)
        loc = spline.get_location_at_distance_along_spline(
            d, unreal.SplineCoordinateSpace.WORLD
        )

        # Collision check against all existing + new locations
        collision = any(
            distance(loc, other) < MIN_SPAWN_DISTANCE
            for other in existing_vehicle_locations +
                         [spline.get_location_at_distance_along_spline(nd, unreal.SplineCoordinateSpace.WORLD)
                          for nd in new_distances]
        )
        if not collision:
            new_distances.append(d)

    if not new_distances:
        unreal.log_warning(f"[SpawnCars] No free points on spline after {attempts} attempts.")
        continue

    # Spawn & orient each vehicle
    for idx, d in enumerate(new_distances):
        loc     = spline.get_location_at_distance_along_spline(
            d, unreal.SplineCoordinateSpace.WORLD
        )
        tangent = spline.get_tangent_at_distance_along_spline(
            d, unreal.SplineCoordinateSpace.WORLD
        )
        # Use Kismet Math Library to build a Rotator from tangent :contentReference[oaicite:0]{index=0}
        rotator = unreal.MathLibrary.make_rot_from_x(tangent)

        bp_name, bp_cls = random.choice(list(bp_classes.items()))
        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
            bp_cls, loc, rotator
        )
        if not actor:
            unreal.log_error(f"[SpawnCars] Failed to spawn {bp_name}")
            continue

        # Record for collision checks in future runs
        existing_vehicle_locations.append(loc)

        # Generate a unique, dot-free label :contentReference[oaicite:1]{index=1}
        base_label = bp_name.replace(".", "_")
        label      = f"{base_label}_{idx}"
        suffix     = 1
        while label in used_labels:
            label = f"{base_label}_{idx}_{suffix}"
            suffix += 1
        used_labels.add(label)

        actor.set_actor_label(label)
        unreal.log(f"[SpawnCars] Spawned {label} at {loc}")
