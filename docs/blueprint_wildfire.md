# Wildfire Blueprint

The **Wildfire** Blueprint is a custom, parameterized Unreal Engine Blueprint that simulates fire igniting and spreading through an environment at runtime. This page covers how to download it and use it in your own Unreal project.

![Wildfire spreading through a forest](media/forestfire1.gif)

## Download

!!! note "Package being finalized"
    The Wildfire Blueprint will be published as a Blueprint package on the [HERCULES releases page](https://github.com/lunarlab-gatech/HERCULES/releases). Until the zip lands, this page already documents the requirements, install procedure, and sensor integration.
<!-- TODO: replace with the actual release asset link once published -->

The package contains the HERCULES-authored Blueprints and support assets only. Third-party VFX content is **not redistributed** (marketplace licensing) — you install it from the links below. See [Using Blueprint Packages](blueprint_packages.md) for the full packaging and licensing model.

## Requirements

| Requirement | Notes |
|---|---|
| Unreal Engine 5.2.1 | Version HERCULES is built against |
| HERCULES AirSim plugin | For sensing integration ([install](install_precompiled.md)) |
| Niagara plugin | Enabled by default in UE5; used for flame/smoke VFX |
| Fire VFX pack | e.g. [Vefects Free Fire VFX](https://www.fab.com/search?q=vefects%20free%20fire) (free) — the pack used in the HERCULES AustraliaLandscape world. Assign via the Blueprint's exposed VFX slots or install at the documented content path. |

<!-- TODO: confirm final third-party asset list + expected content paths when the package is cut -->

## Install into your Unreal project

Follow the general steps in [Using Blueprint Packages](blueprint_packages.md#installing-a-package):

1. Download and unzip the Blueprint package, and copy the contained folder into your project's `Content/`.
2. Install the fire VFX pack from its store link.
3. Restart the Unreal Editor, open the Blueprint, and assign any unset asset slots (flame Niagara system, smoke, burnt material) in the Details panel.

## Usage

1. Drag the wildfire Blueprint actor into your level.
2. Set the ignition source(s) and configure the spread behavior.
3. Play / run the simulation — the fire updates the shared world state at runtime and is observable by all robots.

## Parameters

<!-- TODO: fill in the real exposed parameters from the packaged Blueprint -->
| Parameter | Description | Default |
|---|---|---|
| Ignition point(s) | Where the fire starts | — |
| Spread rate | How fast the fire front advances | — |
| Fuel / extent | Region the fire can consume | — |

## Sensor integration

Fire is the flagship use case for the [synthetic thermal camera](synthetic_cameras.md):

* Fire actors whose mesh names contain `fire`, `flame`, or `torch` are automatically classified **hot** (1000 K, forced to maximum thermal brightness) by the [LWIR ThermalIR camera](lwir_camera.md), provided they are registered with the [instance segmentation](instance_segmentation.md) system — runtime-spawned actors need `AddNewActorToInstanceSegmentation`, see [Using Blueprint Packages](blueprint_packages.md#making-blueprints-visible-to-the-hercules-sensors).
* Anything named differently can be classified explicitly via a settings.json override:

```json
"SyntheticCameraSettings": {
  "ThermalIR": { "Overrides": [ { "Match": "burnfront", "TempK": 1000.0, "Emissivity": 0.98, "IsFire": true } ] }
}
```

* Fires observed at night pair naturally with the [night-vision camera](night_vision.md) — bright flames drive its bloom/halo stage.

![Fire over a Cesium model of Pasadena at night](media/caltech_fire_1drone_night.gif)

See the [Dynamic Environmental Phenomena](dynamic_phenomena.md) overview for all phenomena in context.
