# Dynamic Environmental Phenomena

HERCULES exposes parameterized **dynamic-environment modules** and **dynamic-agent Blueprints** through plug-and-play interfaces, so the shared world state evolves at runtime for procedural scenario generation. These go beyond the static [dynamic objects](dynamic_objects.md) inherited from Cosys-AirSim.

## Phenomena

Each phenomenon is packaged as a downloadable, parameterized Unreal **Blueprint** you can drop into your own project — see the per-phenomenon pages below for download and usage, and [Using Blueprint Packages](blueprint_packages.md) for the packaging/licensing model, install steps, and how to make Blueprints visible to the HERCULES sensors.

=== "Wildfire spread"

    Fire propagates through the environment over time, coupling naturally with the [LWIR thermal](lwir_camera.md) and [night-vision](night_vision.md) sensors for detection and monitoring scenarios.

    ![Wildfire spreading through a forest](media/forestfire1.gif)
    ![Fire over a Cesium model of Pasadena at night](media/caltech_fire_1drone_night.gif)

    [:material-download: Wildfire Blueprint — download & usage](blueprint_wildfire.md){ .md-button .md-button--primary }

=== "Flood inundation"

    Water level rises and spreads across terrain, including over georeferenced [Cesium for Unreal](https://cesium.com/platform/cesium-for-unreal/) city models.

    ![Flood inundation over a Cesium model of Atlanta](media/Atlanta_hercules_flooding_demo1.gif)
    ![Flood inundation in a dense jungle](media/jungleenv_flooding_1.gif)

    [:material-download: Flood Blueprint — download & usage](blueprint_flood.md){ .md-button .md-button--primary }

=== "Crop-disease transmission"

    Disease spreads across agricultural terrain, driving precision-agriculture and monitoring scenarios.

    ![Crop disease transmission across farmland](media/phenom_cropdisease.png)

    [:material-download: Crop Disease Blueprint — download & usage](blueprint_crop_disease.md){ .md-button .md-button--primary }

## Dynamic agents

Dynamic-agent Blueprints populate the world with moving entities that update the shared state at runtime — each family is a downloadable Blueprint package (HERCULES-authored Blueprints only; the meshes come from the linked free/paid packs):

- **[Animal behavior](blueprint_animal_behavior.md)** (AnimalAI) — kangaroos and deer that wander the natural worlds on the navigation mesh.
- **[Pedestrian behavior](blueprint_pedestrian_behavior.md)** — spline-following walking pedestrians (scanned-people, mannequin and character-pack variants).
- **[Vehicle behavior](blueprint_vehicle_behavior.md)** (VehicleAI) — Chaos-vehicle traffic that steers along spline roads in the city world.

[:material-download: Animal Behavior](blueprint_animal_behavior.md){ .md-button .md-button--primary }
[:material-download: Pedestrian Behavior](blueprint_pedestrian_behavior.md){ .md-button .md-button--primary }
[:material-download: Vehicle Behavior](blueprint_vehicle_behavior.md){ .md-button .md-button--primary }

These integrate with the [environments](index.md#environments) and are designed to stress perception and planning under dynamic obstacles.
