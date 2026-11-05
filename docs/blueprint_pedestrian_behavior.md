# Pedestrian Behavior Blueprint

The **Pedestrian Behavior** package is the set of HERCULES-authored Blueprints that populate the city world with walking pedestrians. They are **spline followers**: each pedestrian carries (or is assigned) a spline and walks along it with a looping walk animation, which makes them deterministic and cheap — 20 to 150 of them run in the HERCULES city levels. The character meshes come from scanned-people, mannequin and character packs (not MetaHumans), linked below.

## Download

[:material-download: HERCULES_PedestrianBehavior_Blueprints_v1.0.zip](https://github.com/lunarlab-gatech/HERCULES/releases){ .md-button .md-button--primary }

The zip on the [releases page](https://github.com/lunarlab-gatech/HERCULES/releases) contains **only the HERCULES Blueprints** (22 of them, plus `README.md`, `MANIFEST.json` and the MIT `LICENSE`). Meshes, animations and materials are third-party content — **linked, not shipped** — see [Using Blueprint Packages](blueprint_packages.md).

| Blueprint | What it is | Character pack |
|---|---|---|
| `BP_SplineHuman` | Base class: own `Spline` component + `Character` skeletal mesh playing one walk animation, moved along the spline every tick (loops) | Scanned 3D People Pack (`rp_eric`, walk `rp_nathan`) |
| `BP_SplineHuman_Type2` … `_Type5` | Children: Renderpeople scans `rp_claudia`, `rp_carla`, `rp_manuel`, `rp_sophia` | Scanned 3D People Pack (**free**) |
| `BP_SplineHuman_Type6` … `_Type9` | Children: `SK_Military_Survivalist_1..4` | "Military" pack |
| `BP_SplineHuman_Type10` | Child: `SK_Survival_Character` | "Survival Character" pack (+ 1 City Sample Crowd material) |
| `BP_SplineHuman_Type11` | Child: `SM_Military_4` | "Military" + MyGirl12 materials |
| `BP_SplineHuman_Type12` … `_Type14` | Children: `SK_Skin_1/2/4` | Male 3 (+ MyGirl12 / Military assets) |
| `BP_SplineHuman_Girl1`, `_Girl2` | Children: `SKM_MyGirl12` | MyGirl12 (Girl2 also Male 3 + City Sample Crowd materials) |
| `BP_SplineHuman_Mannequin1`, `_Mannequin3` | Children: UE5 `SKM_Quinn_Simple` / `SKM_Manny_Simple` | "Survival Character" pack demo folder (+ Military material) |
| `BP_SplineHuman_Mannequin2` | Child: UE4 `SK_Mannequin` + `ThirdPersonWalk` | **In the repo**: `Unreal/Environments/DynamicObjects/Content/GroupedAI` (+ Military material) |
| `BP_SplinePath` | Path actor: just a `Spline` component | — |
| `BP_SplineMotionHuman`, `BP_SplineMotionHumanGirl` | Follow an external `BP_SplinePath` (variable *Spline To Follow*) with a `T_Move` Timeline; `Reverse` / `ReverseFromEnd` make them ping-pong | "Survival Character" + "Military" walk / MyGirl12 |

!!! tip "Free subset"
    `BP_SplineHuman` and `Type2`–`Type5` need only the free Renderpeople pack; `Mannequin2` needs only the repo's `GroupedAI` folder (and renders with a default body material without the Military pack). Install the paid packs only for the variants you want.

## Requirements

| Requirement | Notes |
|---|---|
| Unreal Engine 5.2.1 | Version HERCULES is built against; no extra plugin |
| HERCULES AirSim plugin | Only needed to see the pedestrians in the robots' sensor streams ([install](install_precompiled.md)) |
| [Scanned 3D People Pack](https://www.fab.com/listings/2e872a96-686e-4955-8256-82fa19904f92) (Renderpeople, **free**) | Expected at `/Game/Scanned3DPeoplePack/RP_Character/` |
| `GroupedAI` folder from this repository (**free**) | Copy `Unreal/Environments/DynamicObjects/Content/GroupedAI` into `Content/` — it is also the [Dynamic Objects](dynamic_objects.md) minimal walking-AI example |
| [City Sample Crowds](https://www.fab.com/search?q=city%20sample%20crowds) (Epic, **free, optional**) | Four materials used by `Girl2`, `Type8`, `Type10`; without it those variants render with default materials |
| "Military" character pack (`SK_Military_Survivalist_1..4`, `SM_Military_4`, `Demo_animations/Walk`) | Expected at `/Game/Military/`. Also supplies the walk animation of most non-Renderpeople variants |
| "Survival Character" pack (`SK_Survival_Character` + demo mannequins `SKM_Manny_Simple`, `SKM_Quinn_Simple`, `MF_Walk_Fwd`) | Expected at `/Game/Survival_Character/` |
| "MyGirl12" character (`SKM_MyGirl12`, `AS_MyGirl14_Walk`) | Expected at `/Game/MyGirl12/` |
| Male 3 (legacy Unreal Marketplace character; the seller's Fab listings [Male 4](https://www.fab.com/listings/725d8824-274f-4f8d-ab66-1afca5464a85) / Male 5 link to it) | Expected at `/Game/Male_3/` |

!!! warning "Store links being confirmed"
    The exact Fab listings for the Military, Survival Character, MyGirl12 and Male 3 packs are being confirmed from the lab's purchase history; the asset names above are what the Blueprints reference, and `MANIFEST.json` in the zip lists every expected path. No NavMesh is needed — the pedestrians follow splines, not the navigation system.

## Install into your Unreal project

1. Unzip and copy the `Content` folder onto your project root, so the Blueprints land in `<Project>/Content/HERCULES/Agents/Pedestrians/`.
2. Add the packs you need from Fab / the Epic Games Launcher (keep the default folder names), and copy the repo's `GroupedAI` folder into `Content/` for `Mannequin2`.
3. Open the project. Variants whose pack is missing load with an empty mesh and one *Failed to load* warning each; the rest work.

## Usage

**Self-contained pedestrians (`BP_SplineHuman` family)**

1. Drag a variant into the level and select its `Spline` component; add / move spline points to draw the walk path (default: a 1 m straight segment). The pedestrian walks the spline from start to end and jumps back to the start — close the loop by ending near where it starts.
2. Set `Spline Speed` (cm/s, default 150) and `Distance Along Spline` (start offset, default 0) per instance so a crowd on the same path does not walk in lock-step.
3. Play.

**Path-following pedestrians (`BP_SplineMotionHuman` family)**

1. Place `BP_SplinePath` actors and shape their splines (sidewalks, crossings).
2. Place `BP_SplineMotionHuman` / `BP_SplineMotionHumanGirl` actors and set *Spline To Follow* on each to one of the paths. `Reverse` / `ReverseFromEnd` play the Timeline backwards / start from the far end, so two actors on one path can walk toward each other.
3. Play. The HERCULES level `CustomCityHercules_DynamicObjects` uses 124 paths and 20 of these actors; `CustomCityHercules_NOAI` uses 19 self-contained `BP_SplineHuman` variants.

### Parameters

| Parameter | Where | Default | Description |
|---|---|---|---|
| `Spline Speed` | `BP_SplineHuman` + children (instance-editable) | 150 cm/s | Walking speed along the own spline |
| `Distance Along Spline` | `BP_SplineHuman` + children | 0 cm | Start offset |
| spline points | `Spline` component of each instance | 2 points, 100 cm | The walk path |
| `Spline To Follow` | `BP_SplineMotionHuman(Girl)` | None | `BP_SplinePath` actor to follow |
| `Reverse`, `ReverseFromEnd` | `BP_SplineMotionHuman(Girl)` | false | Direction / ping-pong |
| mesh, `Anim to Play` | `Character` component | per variant | Swap the character or the walk cycle; the component runs in *Animation Single Node* mode, so any looping walk animation for that skeleton works |

Crowd density is simply the number of actors you place; there is no spawner. To add a look, duplicate `BP_SplineHuman_Type2` and change the mesh and animation on its `Character` component.

## Sensor integration

* **No registration call is needed** for placed pedestrians: they are added to [instance segmentation](instance_segmentation.md) automatically at simulation start. Only actors spawned at runtime need `AddNewActorToInstanceSegmentation` — see [Using Blueprint Packages](blueprint_packages.md#making-blueprints-visible-to-the-hercules-sensors).
* The [LWIR thermal camera](synthetic_cameras.md) classifies warm bodies by the (case-insensitive) keywords `human` or `person` in the segmentation name, which for these single-mesh actors is the actor name — `BP_SplineHuman_Type3_...` and `BP_SplineMotionHuman_...` already match. If you rename actors, keep `Human` in the name or add an `Overrides` entry matching your name.
* For [object detection](object_detection.md), filter on the actor names, e.g. `simAddDetectionFilterMeshName(camera, ImageType.Scene, "BP_SplineHuman*")`. The `Debug_Object_Detection_Level` of the city project is a small ready-made scene (one car, two pedestrians, six paths) for testing this.
* Pedestrians at night are a natural target for the [night-vision camera](night_vision.md) in search-and-rescue style scenarios.

See the [Dynamic Environmental Phenomena](dynamic_phenomena.md) overview and the [Environments](index.md#environments) for these agents in context.
