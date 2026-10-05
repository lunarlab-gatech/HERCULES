"""Build the HERCULES environment levels in the HerculesEnvs project.

Run headless from the Unreal Editor's Python commandlet, for example on Windows:

    UnrealEditor-Cmd.exe HerculesEnvs.uproject -run=pythonscript -script="Scripts/build_levels.py blank cesium"

Levels are written to /Game/Hercules/Maps. Each level gets a PlayerStart at the origin, where the
HERCULES settings files place the robots relative to, and uses the AirSim game mode from
Config/DefaultEngine.ini. Rebuilding a level clears and repopulates it.
"""
import sys

import unreal

MAPS = "/Game/Hercules/Maps"

level_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)


def log(msg):
    unreal.log(f"[build_levels] {msg}")


def new_level(name):
    """Open an empty level at /Game/Hercules/Maps/<name>, reusing and clearing an existing one."""
    path = f"{MAPS}/{name}"
    # The commandlet starts before the asset registry has scanned the project
    unreal.AssetRegistryHelpers.get_asset_registry().scan_paths_synchronous([MAPS], True)
    if unreal.EditorAssetLibrary.does_asset_exist(path):
        if not level_subsystem.load_level(path):
            raise RuntimeError(f"could not open {path}")
        for actor in actor_subsystem.get_all_level_actors():
            if not isinstance(actor, unreal.WorldSettings):
                actor_subsystem.destroy_actor(actor)
        log(f"cleared {path}")
    else:
        if not level_subsystem.new_level(path):
            raise RuntimeError(f"could not create {path}")
        log(f"created {path}")
    return path


def spawn(cls, location=(0, 0, 0), rotation=(0, 0, 0), label=None):
    actor = actor_subsystem.spawn_actor_from_class(cls, unreal.Vector(*location), unreal.Rotator(*rotation))
    if label:
        actor.set_actor_label(label)
    return actor


def add_sky(sun_pitch=-40.0):
    sun = spawn(unreal.DirectionalLight, (0, 0, 1000), (0, sun_pitch, 30), "Sun")
    sun.light_component.set_editor_property("atmosphere_sun_light", True)
    sun.light_component.set_intensity(8.0)
    spawn(unreal.SkyAtmosphere, label="SkyAtmosphere")
    sky = spawn(unreal.SkyLight, (0, 0, 500), label="SkyLight")
    sky.light_component.set_editor_property("real_time_capture", True)
    spawn(unreal.ExponentialHeightFog, label="HeightFog")
    spawn(unreal.VolumetricCloud, label="Clouds")


def add_player_start(z_cm=50.0):
    spawn(unreal.PlayerStart, (0, 0, z_cm), label="PlayerStart")


def add_spawn_pad(size_m=40.0, z_cm=0.0):
    """Invisible floor under the spawn point that only robots collide with.

    Streamed worlds (Cesium) have no ground until tiles load. The pad holds the robots until then;
    it ignores the visibility and camera channels, so cameras and LiDAR do not see it. The
    HerculesSpawnPad tag lets the Cesium subsystem remove it once tiles are under the robots."""
    pad = spawn(unreal.StaticMeshActor, (0, 0, z_cm - 5.0), label="HerculesSpawnPad")
    comp = pad.static_mesh_component
    comp.set_static_mesh(unreal.EditorAssetLibrary.load_asset("/Engine/BasicShapes/Cube.Cube"))
    comp.set_editor_property("relative_scale3d", unreal.Vector(size_m, size_m, 0.1))
    comp.set_visibility(False)
    comp.set_editor_property("hidden_in_game", True)
    comp.set_editor_property("cast_shadow", False)
    comp.set_collision_profile_name("Custom")
    comp.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
    comp.set_collision_response_to_all_channels(unreal.CollisionResponseType.ECR_BLOCK)
    comp.set_collision_response_to_channel(unreal.CollisionChannel.ECC_VISIBILITY, unreal.CollisionResponseType.ECR_IGNORE)
    comp.set_collision_response_to_channel(unreal.CollisionChannel.ECC_CAMERA, unreal.CollisionResponseType.ECR_IGNORE)
    pad.tags = ["InstanceSegmentation_disable", "HerculesSpawnPad"]
    comp.component_tags = ["InstanceSegmentation_disable"]


def save():
    if not level_subsystem.save_current_level():
        raise RuntimeError("could not save the level")


def build_blank():
    """Flat 1 km test world."""
    new_level("HerculesBlank")
    floor = spawn(unreal.StaticMeshActor, (0, 0, -50), label="Floor")
    floor.static_mesh_component.set_static_mesh(unreal.EditorAssetLibrary.load_asset("/Engine/BasicShapes/Cube.Cube"))
    floor.static_mesh_component.set_editor_property("relative_scale3d", unreal.Vector(1000, 1000, 1))
    floor.static_mesh_component.set_material(0, unreal.EditorAssetLibrary.load_asset("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"))
    add_sky()
    add_player_start()
    save()


def build_cesium():
    """Georeferenced world streamed from Cesium ion (Google Photorealistic 3D Tiles by default).

    The level stores no tileset and no token: the HerculesCesiumSubsystem spawns the ion tilesets
    at play time for georeferences tagged HerculesIonTilesets, once -CesiumIonToken=<token> (or
    CESIUM_ION_TOKEN) is given. -CesiumOrigin=<lat>,<lon>,<ellipsoid height m> chooses the place;
    the default origin is the Caltech campus in Pasadena."""
    new_level("HerculesCesium")
    georef = spawn(unreal.CesiumGeoreference, label="CesiumGeoreference")
    georef.set_editor_property("origin_latitude", 34.137658)
    georef.set_editor_property("origin_longitude", -118.125269)
    georef.set_editor_property("origin_height", 198.0)  # ~232 m above sea level, geoid -34.5 m
    georef.tags = ["HerculesIonTilesets"]
    sun_sky = spawn(unreal.CesiumSunSky, label="CesiumSunSky")
    # CesiumSunSky defaults to a physical 111000 lux sun, which saturates the HERCULES cameras'
    # auto exposure; use the brightness of the other HERCULES worlds instead
    sun_sky.get_editor_property("directional_light").set_intensity(8.0)
    add_spawn_pad()
    add_player_start()
    # Cesium recommends this off: the georeferenced world is far larger than Unreal's bounds
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    world.get_world_settings().set_editor_property("enable_world_bounds_checks", False)
    save()


BUILDERS = {"blank": build_blank, "cesium": build_cesium}


def main(argv):
    names = [a for a in argv if not a.startswith("-")] or list(BUILDERS)
    for name in names:
        if name not in BUILDERS:
            raise SystemExit(f"unknown level '{name}', choose from {sorted(BUILDERS)}")
        log(f"building {name}")
        BUILDERS[name]()
    log("done")


main(sys.argv[1:])
