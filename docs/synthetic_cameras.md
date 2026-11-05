# Synthetic Thermal (FLIR) and Night Vision Cameras

HERCULES provides two synthetic camera image types that are composed **server-side inside the AirSim plugin**, with no ROS2 involvement:

| ImageType | Value | Look | Composed from |
|---|---|---|---|
| `ThermalIR` | 11 | FLIR-style LWIR thermal, Inferno colormap, hot animals/fire highlighted | Segmentation + DepthPlanar |
| `NightVision` | 12 | Green-phosphor light intensifier (AGC, CLAHE, grain, bloom, vignette) | Scene + Segmentation |

Neither type has a scene capture component. When you request one, the plugin silently renders the underlying captures it needs **in the same `simGetImages` render batch** and composes the result on CPU. This gives a hard guarantee: the synthetic image, and any normal captures requested alongside it, come from the **same frame** (identical timestamps), which makes them safe to use under lockstep simulation.

The pixel pipelines are faithful ports of the two former ROS2 post-processing nodes (`thermal_image_node.cpp` "nightvision" mode and `thermal_image_segmentation_based_node.cpp` "flir" mode), with the label→temperature mapping now built natively from the [instance segmentation](instance_segmentation.md) system instead of a CSV file.

## Quick start

```python
import hercules_cosysairsim as airsim

client = airsim.MultirotorClient()
client.confirmConnection()

responses = client.simGetImages([
    airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False),
    airsim.ImageRequest("front_center", airsim.ImageType.ThermalIR, False, False),
    airsim.ImageRequest("front_center", airsim.ImageType.NightVision, False, False),
])
# All three responses share the same time_stamp (same rendered frame).
# Synthetic responses are uncompressed interleaved RGB8 (width*height*3 bytes).
```

Mixed batches, synthetic-only batches, and single-image `simGetImage` all work. Ready-made scripts:

* `PythonClient/computer_vision/live_ir_nvg_viewer.py` — live side-by-side FLIR + NVG OpenCV window.
* `PythonClient/computer_vision/test_synthetic_ir_nvg.py` — smoke test (saves PNGs, asserts sanity).

## Making it work

Both types work out of the box with an unmodified settings.json, but for good results check the following.

### Requirements for ThermalIR (FLIR)

1. **Instance segmentation must be active** (`InitialInstanceSegmentation` not set to false — it is on by default). The thermal look is driven entirely by the segmentation image and the mesh-name list.
2. **Object mesh names decide their temperature.** At the first image request the plugin zips `simListInstanceSegmentationObjects()` with `simGetInstanceSegmentationColorMap()` and classifies every mesh name with a keyword search (case-insensitive substring match):

    | Keywords in mesh name | Temp (K) | Emissivity | Flags |
    |---|---|---|---|
    | tree, bush, grass, plant, vegetation | 293 | 0.97 | |
    | road, ground, dirt, rock, soil | 300 | 0.93 | |
    | car, truck, vehicle, husky | 305 | 0.90 | |
    | fire, flame, torch | 1000 | 0.98 | fire (forced to max brightness) |
    | animal, kangaroo, deer, human, person | 315 | 0.98 | animal (brightness floor 220); kangaroo also gets the orange recolor |
    | *anything else* | 295 | 0.90 | neutral |

3. **DepthPlanar drives distance attenuation** (`1 / (1 + 0.01 * depth_m)`). For it to apply, the DepthPlanar capture must have the **same resolution as the Segmentation capture**; otherwise attenuation is silently skipped.

### Requirements for NightVision (NVG)

1. For the proper fused look (scene luminance + thermal hint), the **Scene capture and the Segmentation capture must have the same resolution** in settings.json:

    ```json
    "CaptureSettings": [
      { "ImageType": 0, "Width": 752, "Height": 480 },
      { "ImageType": 5, "Width": 752, "Height": 480 }
    ]
    ```

2. If the resolutions differ, NVG still works but **falls back to the pure thermal map** (no scene texture) — flat green regions per object. This mirrors the old ROS node's behavior on mismatched topics. The output resolution always follows the Segmentation capture.

## Object not showing up in the thermal image?

Work through these in order:

1. **Is the object in the instance segmentation at all?** Check with `client.simListInstanceSegmentationObjects()`. If it is missing:
    * Only static and skeletal meshes are supported (no landscape, foliage, brush, decals — see [instance segmentation limitations](instance_segmentation.md#limitations)).
    * Objects **spawned at runtime** (Blueprint/C++ extensions, dynamic agents) must be registered: call `ASimModeBase::AddNewActorToInstanceSegmentation(Actor, UpdateAnnotation)` from C++/Blueprints, or via RPC first `simAddSegmentationActor` then `simSetSegmentationObjectID`.
    * Components tagged `InstanceSegmentation_disable` are deliberately excluded.
    * An unregistered object renders with a color that is not in the color map and classifies as **neutral** (295 K) — visible but never hot.
2. **Does its mesh name match a keyword?** The classifier only sees the mesh name (e.g. `BP_Kangaroo23_...` matches "kangaroo"). If your object is named `SM_Blob42` it classifies as neutral. Fix it either by renaming the mesh to contain a keyword, **or set its thermal properties explicitly with an override** (no rename, no recompile):

    ```json
    "SyntheticCameraSettings": {
      "ThermalIR": {
        "Overrides": [
          { "Match": "blob", "TempK": 320.0, "Emissivity": 0.95, "IsAnimal": true },
          { "Match": "campfire", "TempK": 900.0, "Emissivity": 0.98, "IsFire": true }
        ]
      }
    }
    ```

    `Match` is a case-insensitive substring of the mesh name; the first matching entry wins and **takes priority over the keyword table**. `IsAnimal` gives the brightness floor, `IsFire` forces max brightness, `IsKangaroo` additionally applies the orange recolor. Settings changes require a sim restart.
3. **Did the object appear after the first thermal request?** Handled automatically: the radiance LUTs grow incrementally as new segmentation colors/labels are encountered (the old ROS nodes froze the LUT on the first frame; this implementation renormalizes instead — expect a subtle global brightness shift on the frame where a new hottest object appears).
4. **Is everything dim/black far away?** That is depth attenuation; lower `DepthAttenuation` (or set it to 0) in the settings block.

## Settings reference

Everything is optional; defaults shown. The block sits at the **top level** of settings.json.

```json
"SyntheticCameraSettings": {
  "ThermalIR": {
    "TempMin": 280.0,
    "TempMax": 1300.0,
    "EpsMin": 0.80,
    "EpsMax": 0.99,
    "DepthAttenuation": 0.01,
    "Overrides": []
  },
  "NightVision": {
    "TempMin": 285.0,
    "TempMax": 310.0,
    "EpsMin": 0.85,
    "EpsMax": 0.98,
    "BlendAlpha": 0.25,
    "NvgGain": 1.0,
    "Seed": 42
  }
}
```

* `TempMin/TempMax`, `EpsMin/EpsMax` — clamp range for ThermalIR profiles; random draw range for the per-label NightVision LUT.
* `DepthAttenuation` — `k` in `1 / (1 + k * depth_m)`; 0 disables attenuation.
* `Overrides` — per-object thermal properties, see above.
* `BlendAlpha` — thermal weight when blending with the scene grayscale in NVG.
* `NvgGain` — multiplies the automatic gain; higher = brighter and grainier.
* `Seed` — fixed RNG seed for the NVG per-label temperature LUT and grain stream, for reproducible runs.

## Common failure modes

| Symptom | Cause | Fix |
|---|---|---|
| Empty image, message `"...the segmentation capture failed"` | Segmentation render returned no data | Verify a plain Segmentation request works; under lockstep in heavy scenes, empty segmentation frames are a known sim-side issue independent of the synthetic cameras |
| NVG has no scene detail (flat green blobs) | Scene resolution ≠ Segmentation resolution | Set ImageType 0 and 5 to the same `Width`/`Height` in settings.json |
| ThermalIR uniform / attenuation missing | DepthPlanar resolution ≠ Segmentation resolution | Set ImageType 1 to the same resolution (or accept no attenuation) |
| Everything neutral, no hot animals | Instance segmentation empty or object names carry no keywords | See "Object not showing up" above |
| Old behavior after editing plugin sources | Editor compiled the plugin at launch, before the edit | Restart the editor (it rebuilds the plugin on open), or prebuild with UBT |
| `AttributeError`/`out_of_range` on type 11/12 from clients | Stale Python client or ROS wrapper build | Reinstall `PythonClient` package / rebuild `airsim_ros_pkgs` (name maps include 11/12) |

## Implementation notes

* Source: `Unreal/Plugins/AirSim/Source/SyntheticCameraProcessor.{h,cpp}` (pixel pipelines) and the interception in `UnrealImageCapture::getImages` (hidden sub-request batching, dedup against user requests, 1:1 response ordering).
* Output is RGB8; `compress=true` is honored via the same PNG path normal captures use; `pixels_as_float` is ignored (always uint8).
* The plugin-local `Source/AirLib` copies (enum in `ImageCaptureBase.hpp`, settings in `AirSimSettings.hpp`) are gitignored build artifacts — when recreating a plugin copy, re-sync those files from an existing copy.
