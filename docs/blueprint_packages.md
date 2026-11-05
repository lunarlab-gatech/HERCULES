# Using HERCULES Blueprint Packages

The [dynamic phenomena](dynamic_phenomena.md) and dynamic-agent Blueprints are distributed as **Blueprint packages**: zip archives of an Unreal `Content/` subfolder containing the HERCULES-authored Blueprints (and any HERCULES-authored support assets such as materials, curves, or data tables), published on the [HERCULES releases page](https://github.com/lunarlab-gatech/HERCULES/releases).

## What is (and is not) in a package

* **Included** — everything authored by the HERCULES team: the Blueprint classes, animation Blueprints, behavior logic, and self-made support assets. These are redistributable and covered by the repository license.
* **Not included** — third-party marketplace/Fab assets (meshes, skeletons, animation packs, VFX, character packs). Marketplace licenses allow *using* these in a project but not *redistributing* them, so each Blueprint page lists the required packs with links; you install them into your project yourself (most are free).

The agent Blueprints reference their third-party meshes and animations by **content path** (hard references on the mesh component of each variant). The package therefore loads before the packs are installed, but with empty meshes and a *Failed to load* warning per missing asset; once the packs sit at the documented `/Game/<Pack>/...` paths everything resolves without any reassignment. Every package ships a `MANIFEST.json` listing, per Blueprint, exactly which third-party asset paths it expects.

## Installing a package

1. Download the package zip from the [releases page](https://github.com/lunarlab-gatech/HERCULES/releases) and unzip it.
2. Copy the contained folder into your Unreal project's `Content/` directory (keep the folder name — Blueprint references are by content path).
3. Install the third-party asset packs listed on the Blueprint's page from their store links. If the page documents an expected content path (e.g. `/Game/AnimalVarietyPack/...`), install/move them to that path so hard references resolve.
4. Restart the Unreal Editor so the asset registry picks everything up.
5. Open the Blueprint and check the **asset slot variables** in the Details panel — assign any that are unset (mesh, animation Blueprint, Niagara system, etc.).
6. Drag the Blueprint actor into your level and configure its parameters (see the per-Blueprint page).

!!! tip "Broken references?"
    If the Blueprint opens with blank meshes or compile warnings about missing assets, the third-party pack is either not installed or installed at a different content path than the Blueprint expects. Either move it to the documented path (Fix Up Redirectors afterwards) or assign the assets manually via the exposed slots.

## Making Blueprints visible to the HERCULES sensors

Dynamic actors interact with the HERCULES ground-truth and synthetic sensors through the [instance segmentation](instance_segmentation.md) system:

* **Editor-placed actors** are registered automatically at simulation start (static and skeletal meshes only — see the [limitations](instance_segmentation.md#limitations)).
* **Runtime-spawned actors** (which is what most dynamic phenomena/agent Blueprints do) must be registered explicitly: call `ASimModeBase::AddNewActorToInstanceSegmentation(Actor, UpdateAnnotation)` (BlueprintCallable on the SimMode actor; `DeleteActorFromInstanceSegmentation` and `ForceUpdateInstanceSegmentation` are its companions) from the Blueprint/C++ after spawning, or over RPC call `simAddSegmentationActor` followed by `simSetSegmentationObjectID`. The HERCULES dynamic-agent Blueprints are editor-placed, so they need none of this; the wildfire placement scripts spawn their fire actors before the simulation starts for the same reason.

Registration also drives the [synthetic thermal and night-vision cameras](synthetic_cameras.md): the [LWIR ThermalIR camera](lwir_camera.md) classifies every registered mesh by **name keywords** (`fire`, `kangaroo`, `animal`, `person`, `car`, ...) to assign temperature and emissivity. Two consequences for Blueprint authors and users:

1. **Name your spawned actors/meshes meaningfully** (`BP_Kangaroo23`, `FireFront_...`) so they classify correctly.
2. For anything that doesn't fit the keyword table, set thermal properties explicitly in settings.json — no renaming or recompiling needed:

```json
"SyntheticCameraSettings": {
  "ThermalIR": {
    "Overrides": [
      { "Match": "floodwater", "TempK": 288.0, "Emissivity": 0.96 },
      { "Match": "elephant", "TempK": 309.0, "Emissivity": 0.98, "IsAnimal": true }
    ]
  }
}
```

See the [synthetic cameras page](synthetic_cameras.md) for the full keyword table, settings reference, and troubleshooting ("object not showing up").

## How the HERCULES agent packages are built

The three dynamic-agent packages ([animals](blueprint_animal_behavior.md), [pedestrians](blueprint_pedestrian_behavior.md), [vehicles](blueprint_vehicle_behavior.md)) are produced by `PythonClient/hero/package_agent_blueprints_UE5.py`, run headless against a throwaway Blueprint-only project that contains the source Blueprints plus the third-party packs:

```bash
UnrealEditor-Cmd <PackagingProject>.uproject -run=pythonscript \
    -script=PythonClient/hero/package_agent_blueprints_UE5.py -unattended -nopause -nosplash -NullRHI
```

With `HPKG_MODE=package` it renames every Blueprint into `/Game/HERCULES/Agents/{Animals,Pedestrians,Vehicles}` (base classes first), force-saves them so their import tables carry the final paths, deletes the redirectors left behind, and runs a byte-level scan of every saved asset for the old paths. With `HPKG_MODE=verify` it loads and compiles each Blueprint, classifies every dependency as HERCULES / engine / third-party pack and writes the manifest (`HPKG_OUT`) that becomes each zip's `MANIFEST.json`. The zips are then built from that folder only, and loaded once more in a clean project **without** any pack to check the documented behaviour (empty meshes and one *Failed to load* warning per missing asset; `BP_VehicleAI` additionally reports a compile error until City Sample Vehicles provides its parent class).

## Authoring and sharing your own Blueprint package

If you build your own phenomenon on top of HERCULES and want to share it the same way:

1. **Audit dependencies** — right-click the Blueprint → *Reference Viewer* / *Size Map* and classify every referenced asset: yours (ship it), engine/starter content (users already have it), third-party (link it, don't ship it).
2. **Break hard third-party references** — replace direct mesh/VFX references with instance-editable variables (`Static Mesh`, `Skeletal Mesh`, `Niagara System` object types, ideally soft references), assigned per-instance or on BeginPlay. This is what keeps the package loadable without the store content.
3. **Migrate** — right-click the Blueprint → *Asset Actions → Migrate* into a **clean project's** `Content/`, and verify there that it opens without the third-party packs installed.
4. **Package** — zip the migrated folder, attach it to a GitHub release, and document: required UE version, required plugins (e.g. Niagara, Cesium), each third-party pack with store link and expected content path, and the exposed parameters.
5. Optionally paste the graph to [blueprintUE](https://blueprintue.com/) and link it from your docs so readers can inspect the logic without downloading anything.
