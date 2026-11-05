# Animal Behavior Blueprint

The **Animal Behavior** package (AnimalAI) is the set of HERCULES-authored Unreal Engine Blueprints that put wandering wildlife — kangaroos and deer — into the natural HERCULES worlds. Each animal is a `Character` driven by Unreal's navigation system: it picks a random reachable point around itself, walks there, pauses, and repeats, so robots see moving warm bodies without any scripting on your side.

![Kangaroo wildlife roaming the outback](media/auskangaroo1.gif)

## Download

[:material-download: HERCULES_AnimalBehavior_Blueprints_v1.0.zip](https://github.com/lunarlab-gatech/HERCULES/releases){ .md-button .md-button--primary }

The zip is attached to the [HERCULES releases page](https://github.com/lunarlab-gatech/HERCULES/releases). It contains **only the HERCULES Blueprints** (plus `README.md`, `MANIFEST.json` and the MIT `LICENSE`); the animal meshes, skeletons and animations are third-party Fab content that is **linked, not shipped** — see [Using Blueprint Packages](blueprint_packages.md).

| Blueprint | Parent | Mesh / AnimBP it expects | Walk speed |
|---|---|---|---|
| `BP_Animal` | `Character` | `SK_DeerStag` + `ABP_DeerStag` (Animal Variety Pack) | 150 cm/s |
| `BP_DeerStag` | `BP_Animal` | `SK_DeerStag` + `ABP_DeerStag` | 120 cm/s |
| `BP_DeerDoe`, `BP_DeerDoe1` | `BP_Animal` | `SK_DeerDoe` + `ABP_DeerDoe` | 120 cm/s |
| `BP_Kangaroo` | `Character` (same logic as `BP_Animal`, own copy) | `SM_kangaroo` (skeletal mesh) + `ABP_Kangaroo` (Kangaroo Animated) | 300 cm/s |

## Requirements

| Requirement | Notes |
|---|---|
| Unreal Engine 5.2.1 | Version HERCULES is built against (engine modules `AIModule` and `NavigationSystem` are always available; no extra plugin) |
| HERCULES AirSim plugin | Only needed to see the animals in the robots' sensor streams ([install](install_precompiled.md)) |
| [Animal Variety Pack](https://www.fab.com/listings/2dd7964c-a601-4264-a53d-465dcae1644c) (PROTOFACTOR, **free**) | Deer stag + doe. Expected at `/Game/AnimalVarietyPack/DeerStagAndDoe/` (the default *Add to project* location) |
| [Kangaroo Animated](https://www.fab.com/listings/876c0654-dadc-47a6-ac14-fb58eac3cccf) (low-poly kangaroo, 11 animations) | Expected at `/Game/Kangaroo/` with `Meshes/SM_kangaroo` and `Animations/ABP_Kangaroo`. The listing was identified from the asset names the Blueprint references (`SK_kangaroo`, `Anim_kangaroo_idle/walk/run/boxing/kick/doublekick/eat/die/hit`) — confirm it against your Fab library before buying |
| NavMesh in your level | Add a `NavMeshBoundsVolume` covering the roam area (a `RecastNavMesh` actor is created automatically). The Australia level uses ten of them. |

## Install into your Unreal project

1. Unzip the package and copy its `Content` folder onto your project root, so the Blueprints land in `<Project>/Content/HERCULES/Agents/Animals/`.
2. Add the Animal Variety Pack and the kangaroo pack to the same project from Fab / the Epic Games Launcher. Keep their default folder names (`AnimalVarietyPack`, `Kangaroo`); the Blueprints reference the meshes by content path.
3. Open the project. If a pack is missing you get one *Failed to load* warning per missing asset and that species shows an empty mesh — everything else still works.
4. Make sure the level has a NavMesh (press `P` in the viewport to visualise it).

## How it works

`BP_Animal` is a `Character` with `AIControllerClass = DetourCrowdAIController` and *Auto Possess AI = Placed in World*. Its Event Graph runs one loop from `BeginPlay`:

```
GetRandomReachablePointInRadius (origin = own location, radius)
  -> AI MoveTo (acceptance radius)        # DetourCrowd steers around other agents
  -> on Completed / Aborted -> Delay -> loop
```

The deer species are children that only change the mesh, animation Blueprint and walk speed; the does add a `Yaw Speed` variable (default 0) that applies a constant turn every tick on top of the navigation. `BP_Kangaroo` is a standalone copy of the same loop on the kangaroo mesh with `BlockAll` collision.

## Usage

1. Drag `BP_Kangaroo`, `BP_DeerStag`, `BP_DeerDoe` or `BP_DeerDoe1` into the level, as many as you want — there is no spawner: herd size is the number of actors you place (the HERCULES Australia world ships two kangaroos, the Spruce Forest test level two stags, four does and two `BP_Animal`).
2. Keep the actors inside the NavMesh, otherwise `GetRandomReachablePointInRadius` finds nothing and the animal stands still.
3. Play / run the simulation — the animals start wandering immediately and are observable by every robot's sensors.

### Parameters

| Parameter | Where | Default | Description |
|---|---|---|---|
| `Max Walk Speed` | CharacterMovement component (per instance or per Blueprint) | 150 / 120 / 300 cm/s | Walking speed |
| `Yaw Speed` | `BP_DeerDoe`, `BP_DeerDoe1` (instance-editable) | 0 deg/s | Extra constant turning while walking |
| Wander radius, acceptance radius, pause | Literals on the `GetRandomReachablePointInRadius`, `AI MoveTo` and `Delay` nodes in the Event Graph | — | Open `BP_Animal` / `BP_Kangaroo` and edit the pin values (or promote them to variables) |
| Roam area | `NavMeshBoundsVolume`(s) in the level | — | Animals never leave the NavMesh |

### Adding a species

Duplicate `BP_DeerStag`, set the *Mesh* component's skeletal mesh and *Anim Class* to your animal's assets, and adjust `Max Walk Speed`. Nothing else is needed; the wander loop is inherited from `BP_Animal`.

## Sensor integration

Animals are the flagship targets for the [synthetic thermal camera](synthetic_cameras.md):

* **No registration call is needed.** Editor-placed actors are added to [instance segmentation](instance_segmentation.md) automatically when the simulation starts; only actors you spawn yourself at runtime need `AddNewActorToInstanceSegmentation` — see [Using Blueprint Packages](blueprint_packages.md#making-blueprints-visible-to-the-hercules-sensors).
* **Classification is by name, case-insensitive.** The [LWIR camera](lwir_camera.md) looks for `kangaroo`, `deer` or `animal` in the segmentation name, which for a single-mesh actor is the actor's own name (`BP_Kangaroo_C_...`, `BP_DeerStag_...`). The default names therefore classify as warm bodies (315 K, brightness-floored) out of the box, and kangaroos get the orange highlight recolor. Keep those words in the name if you rename actors.
* Other species need only an override — no renaming:

```json
"SyntheticCameraSettings": {
  "ThermalIR": { "Overrides": [ { "Match": "elephant", "TempK": 309.0, "Emissivity": 0.98, "IsAnimal": true } ] }
}
```

* For [object detection](object_detection.md) labels, add a detection filter on the camera, e.g. `simAddDetectionFilterMeshName(camera, ImageType.Scene, "BP_Kangaroo*")`; the 2-D/3-D boxes then follow the moving animals.

See the [Dynamic Environmental Phenomena](dynamic_phenomena.md) overview and the [Environments](index.md#environments) for these agents in context.
