# Vehicle Behavior Blueprint

The **Vehicle Behavior** package (VehicleAI) is the set of HERCULES-authored Blueprints that fill the city world with autonomous road traffic. `BP_VehicleAI` is a physically simulated Chaos wheeled vehicle that steers itself along `BP_SplineRoad` lanes you draw in the level: it finds the nearest road, aims at a look-ahead point on it, brakes into bends and keeps its speed inside a band. The HERCULES city levels run 85 to 152 of them.

![Vehicle traffic through the urban world](media/citycarspeds1.gif)

## Download

[:material-download: HERCULES_VehicleBehavior_Blueprints_v1.0.zip](https://github.com/lunarlab-gatech/HERCULES/releases){ .md-button .md-button--primary }

The zip on the [releases page](https://github.com/lunarlab-gatech/HERCULES/releases) contains **only the HERCULES Blueprints** (14 of them, plus `README.md`, `MANIFEST.json` and the MIT `LICENSE`). The vehicle meshes, wheels and the `BP_Vehicle` base class come from Epic's free City Sample Vehicles pack — **linked, not shipped** — see [Using Blueprint Packages](blueprint_packages.md).

| Blueprint | What it is | City Sample vehicle |
|---|---|---|
| `BP_VehicleAI` | Base traffic agent: child of City Sample `BP_Vehicle` (`WheeledVehiclePawn`, Chaos wheeled movement, 4 wheels) with the steering / braking / speed logic | `vehicle03_Car` |
| `BP_VehicleAI_sedan2` | child | `vehicle07_Car` |
| `BP_VehicleAI_sportscar` | child | `vehicle06_Car` |
| `BP_VehicleAI_SUV1`, `BP_VehicleAI_SUV1_og` | child / older copy parented directly to `BP_Vehicle` | `vehicle02_Car` |
| `BP_VehicleAI_SUV2` | child | `vehicle05_Car` |
| `BP_VehicleAI_taxi` | child | `vehicle12_Car` body on the `vehicle03` rig |
| `BP_VehicleAI_policecar` | child | `vehicle13_Car` body on the `vehicle03` rig |
| `BP_VehicleAI_Van`, `BP_VehicleAI_van2` | children | `vehicle01_Van`, `vehicle09_Van` |
| `BP_VehicleAI_pickup`, `BP_VehicleAI_pickup1` | child / older copy parented to `BP_Vehicle` | `vehicle04_Truck` |
| `BP_SplineRoad` | Lane actor: just a `Spline` component; vehicles follow the nearest one | — |
| `BP_SplineVehicle` | Kinematic spline follower (same logic as the pedestrians' `BP_SplineHuman`, mesh set per instance) for non-physics traffic such as trains or parked-to-moving props | — |

## Requirements

| Requirement | Notes |
|---|---|
| Unreal Engine 5.2.1 | Version HERCULES is built against |
| **ChaosVehiclesPlugin** | Enable it in your `.uproject` (Edit → Plugins → *Chaos Vehicles*); it provides `WheeledVehiclePawn`. Enhanced Input (enabled by default) is also used by the City Sample base class |
| HERCULES AirSim plugin | Only needed to see the traffic in the robots' sensor streams ([install](install_precompiled.md)) |
| [City Sample Vehicles](https://www.fab.com/search?q=city%20sample%20vehicles) (Epic Games, **free**) | Expected at `/Game/CitySampleVehicles/`: `Blueprint/BP_Vehicle` (parent class), `Rig/ABP_Vehicle`, the `vehicleNN_*` meshes and `Physics/CVW_*` wheel setups. Only this pack is needed; City Sample's MassTraffic is **not** used |
| Road splines in your level | `BP_SplineRoad` actors along the lanes (the HERCULES city levels ship 24); vehicles with no road in reach stand still |

## Install into your Unreal project

1. Enable the Chaos Vehicles plugin and restart the editor.
2. Unzip and copy the `Content` folder onto your project root, so the Blueprints land in `<Project>/Content/HERCULES/Agents/Vehicles/`.
3. Add City Sample Vehicles to the project from Fab / the Epic Games Launcher (keep the default `CitySampleVehicles` folder name).
4. Open the project; the vehicles compile against `BP_Vehicle`.

!!! warning "Install City Sample Vehicles before opening the vehicle Blueprints"
    Without the pack the `BP_VehicleAI*` assets still load (empty mesh, one *Failed to load* warning for `BP_Vehicle`), but they have no parent class, so the editor reports *missing or NULL parent class* and cannot place them — and pressing **Compile** on such a Blueprint can crash UE 5.2. Install the pack first, or delete the vehicle Blueprints you do not need.

## How it works

`BP_VehicleAI` has `AIControllerClass = AIController` and *Auto Possess AI = Placed in World or Spawned*, so a placed or spawned vehicle drives itself:

```
BeginPlay : GetAllActorsOfClass(BP_SplineRoad) -> nearest spline (FindNearestSpline)
Tick      : target = road location at (closest distance along spline + AdditionalOffset)
            CalculateSteeringInput : angle to target, MapRangeClamped -> SetSteeringInput
            CalculateBrakeInput    : RuntimeFloatCurve over the steering angle -> SetBrakeInput
            speed band             : GetForwardSpeedMPH < Min Speed -> throttle; > Max Speed -> release
            Debug Steering / Debug Brake -> debug lines
```

## Usage

1. Lay out `BP_SplineRoad` actors along your lanes (one spline per lane and direction; vehicles snap to the nearest one, so keep opposite lanes a car width apart).
2. Place vehicles by hand, or run `PythonClient/hero/spawn_NPC_cars_splines_UE5.py` from the editor's Python console: it spawns a random mix of `BP_VehicleAI_*` along every `BP_SplineRoad` (10 per spline, 5 m minimum spacing, oriented along the lane). It looks in `/Game/HERCULES/Agents/Vehicles` first and falls back to the original `/Game/CustomHumanPedestrians` folder of the HERCULES city project.
3. Turn `Debug Steering` / `Debug Brake` off before collecting data.
4. Play. The HERCULES level `CustomCityHercules_DynamicObjects` runs 152 vehicles on 24 roads; `CustomCityHercules_CSLAM` 85.

### Parameters

| Parameter | Where | Default | Description |
|---|---|---|---|
| `Min Speed`, `Max Speed` | `BP_VehicleAI` + children (instance-editable) | 5 / 25 mph | Throttle below *Min*, coast above *Max* (compared against `GetForwardSpeedMPH`) |
| `AdditionalOffset` | `BP_VehicleAI` + children | see graph | Look-ahead distance (cm) along the road spline used for steering; larger = smoother, cuts corners |
| `Debug Steering`, `Debug Brake` | `BP_VehicleAI` + children | true | Draw the steering target / brake debug lines |
| brake curve | `CalculateBrakeInput` function (RuntimeFloatCurve) | — | Brake amount versus steering angle |
| `Spline Speed`, `Distance Along Spline` | `BP_SplineVehicle` | 150 cm/s, 0 | Kinematic follower speed and start offset |

Traffic density is the number of vehicles you place or spawn; there is no spawner actor. To add a model, duplicate `BP_VehicleAI_sedan2` and swap the `VehicleMesh` skeletal mesh, the four `SM_Wheel_*` static meshes, `SM_Frame`, `SM_All_Trans` and the wheel classes in the Chaos movement component — any City Sample vehicle (or your own `BP_Vehicle`-compatible rig) works.

## Sensor integration

* **No registration call is needed** for placed or spawned-before-play vehicles: they are added to [instance segmentation](instance_segmentation.md) automatically at simulation start. Only actors spawned at runtime need `AddNewActorToInstanceSegmentation` — see [Using Blueprint Packages](blueprint_packages.md#making-blueprints-visible-to-the-hercules-sensors).
* A vehicle has several mesh components, so its segmentation entries are named after the meshes (`SKM_vehCar_vehicle03...`, `SM_Wheel_Front_L_vehCar_vehicle03...`). The [LWIR thermal camera](synthetic_cameras.md) matches `car`, `truck` and `vehicle` case-insensitively, so every City Sample vehicle classifies as engine-warmed metal (305 K) by default; tune per-model signatures with `Overrides` entries if needed.
* For [object detection](object_detection.md), filter on the mesh names, e.g. `simAddDetectionFilterMeshName(camera, ImageType.Scene, "*vehicle*")`.

See the [Dynamic Environmental Phenomena](dynamic_phenomena.md) overview and the [Environments](index.md#environments) for these agents in context.
