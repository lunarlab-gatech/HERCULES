#!/usr/bin/env python3
"""Build the three HERCULES dynamic-agent Blueprint packages (zip) from the migrated /Game/HERCULES/Agents folder.

Each zip = Content/HERCULES/Agents/<Family>/*.uasset + README.md + MANIFEST.json + LICENSE. Only HERCULES-authored
Blueprints are included; the manifest lists every third-party asset path each Blueprint expects (nothing third-party
is copied). Refuses to build if any file outside the HERCULES folder would be included or if the verify manifest
reports stale paths / load failures.
"""
import json
import os
import sys
import time
import zipfile

SRC = os.path.expanduser("~/Documents/Unreal Projects/HerculesAgentPkg/Content/HERCULES/Agents")
MANIFEST = sys.argv[1]
OUT = os.path.expanduser("~/Documents/hercules_issue8/packages")
LICENSE = os.path.expanduser("~/multi-robot-coordination/Cosys-AirSim/LICENSE")
VERSION = "1.0"
DOCS = "https://lunarlab-gatech.github.io/HERCULES"
REPO = "https://github.com/lunarlab-gatech/HERCULES"

PACKS = {
    "AnimalVarietyPack": dict(title="Animal Variety Pack (PROTOFACTOR)", url="https://www.fab.com/listings/2dd7964c-a601-4264-a53d-465dcae1644c", cost="free",
                              path="/Game/AnimalVarietyPack/DeerStagAndDoe/", note="deer stag + doe skeletal meshes and animation Blueprints"),
    "Kangaroo": dict(title="Kangaroo Animated (low-poly kangaroo, 11 animations)", url="https://www.fab.com/listings/876c0654-dadc-47a6-ac14-fb58eac3cccf", cost="see listing",
                     path="/Game/Kangaroo/", note="SM_kangaroo (skeletal mesh) + ABP_Kangaroo; listing identified from the asset names, confirm against your Fab library"),
    "CitySampleVehicles": dict(title="City Sample Vehicles (Epic Games)", url="https://www.fab.com/search?q=city%20sample%20vehicles", cost="free",
                               path="/Game/CitySampleVehicles/", note="BP_Vehicle base class, ABP_Vehicle, vehicle meshes and Chaos wheel setups"),
    "CitySampleCrowd": dict(title="City Sample Crowds (Epic Games)", url="https://www.fab.com/search?q=city%20sample%20crowds", cost="free, optional",
                            path="/Game/CitySampleCrowd/", note="only four materials (head LODs, hair helmet, shoes) used by three pedestrian variants; without it those variants render with default materials"),
    "Scanned3DPeoplePack": dict(title="Scanned 3D People Pack (Renderpeople)", url="https://www.fab.com/listings/2e872a96-686e-4955-8256-82fa19904f92", cost="free",
                                path="/Game/Scanned3DPeoplePack/RP_Character/", note="rp_eric / rp_claudia / rp_carla / rp_manuel / rp_sophia rigged scans + rp_nathan walking animation"),
    "Military": dict(title="'Military' character pack (SK_Military_Survivalist_1..4, SM_Military_4, Demo_animations/Walk)", url="store listing being confirmed, see docs", cost="paid",
                     path="/Game/Military/", note="also provides the Walk animation used by most pedestrian variants"),
    "Survival_Character": dict(title="'Survival Character' pack (SK_Survival_Character + Demo mannequins SKM_Manny_Simple / SKM_Quinn_Simple, MF_Walk_Fwd)", url="store listing being confirmed, see docs", cost="paid",
                               path="/Game/Survival_Character/", note="the Demo mannequins are Epic's UE5 third-person template characters bundled with the pack"),
    "MyGirl12": dict(title="'MyGirl12' character (SKM_MyGirl12, AS_MyGirl14_Walk)", url="store listing being confirmed, see docs", cost="paid",
                     path="/Game/MyGirl12/", note=""),
    "Male_3": dict(title="Male 3 (legacy Unreal Marketplace character, product id 8880da4175524facabb58a051c70d26b)", url="https://www.fab.com/listings/725d8824-274f-4f8d-ab66-1afca5464a85 (seller's 'Male 4' listing links to Male 3)", cost="paid",
                   path="/Game/Male_3/", note="SK_Skin_1/2/4 skeletal meshes + material instances"),
    "GroupedAI": dict(title="HERCULES repository: Unreal/Environments/DynamicObjects/Content/GroupedAI", url=REPO + "/tree/main/Unreal/Environments/DynamicObjects", cost="free (in the repo)",
                      path="/Game/GroupedAI/", note="SK_Mannequin + ThirdPersonWalk; copy the GroupedAI folder into your project's Content/"),
}

FAMILIES = {
    "Animals": dict(zip="HERCULES_AnimalBehavior_Blueprints_v%s" % VERSION, title="HERCULES Animal Behavior Blueprints", docs=DOCS + "/blueprint_animal_behavior/",
                    plugins=["AIModule, NavigationSystem (engine modules, always present)"],
                    blurb="Wildlife agents (kangaroo, deer stag, deer doe) that wander a NavMesh: BP_Animal picks a random reachable point within a radius, walks there with the DetourCrowd AI controller, waits, and repeats. BP_Kangaroo is the same loop on the kangaroo mesh.",
                    params=[("CharacterMovement > Max Walk Speed", "cm/s", "150 (BP_Animal), 120 (deer), 300 (kangaroo)", "walking speed of the animal"),
                            ("Yaw Speed (BP_DeerDoe / BP_DeerDoe1 only)", "deg/s", "0", "extra constant turning applied every tick on top of the navigation"),
                            ("wander radius / acceptance radius", "cm", "literals on the GetRandomReachablePointInRadius and AI MoveTo nodes", "edit in the Event Graph of BP_Animal / BP_Kangaroo")],
                    usage=["Add a NavMeshBoundsVolume (and let the RecastNavMesh build) over the area the animals may roam.",
                           "Drag BP_Kangaroo, BP_DeerStag, BP_DeerDoe or BP_DeerDoe1 into the level (Auto Possess AI is 'Placed in World', so nothing else is needed).",
                           "Press Play: every animal starts its wander loop immediately."]),
    "Pedestrians": dict(zip="HERCULES_PedestrianBehavior_Blueprints_v%s" % VERSION, title="HERCULES Pedestrian Behavior Blueprints", docs=DOCS + "/blueprint_pedestrian_behavior/",
                        plugins=["none beyond the engine defaults"],
                        blurb="Spline-following walking pedestrians. BP_SplineHuman carries its own spline and moves a skeletal mesh (single walk animation) along it, looping; the Type/Girl/Mannequin children only swap the mesh, animation and materials. BP_SplineMotionHuman(Girl) follows an external BP_SplinePath actor with a Timeline and can ping-pong.",
                        params=[("Spline Speed (BP_SplineHuman + children)", "cm/s", "150", "walking speed along the actor's own spline"),
                                ("Distance Along Spline", "cm", "0", "start offset along the spline"),
                                ("Spline To Follow (BP_SplineMotionHuman / Girl)", "BP_SplinePath actor", "None", "the path actor to walk along (set per instance)"),
                                ("Reverse / ReverseFromEnd (BP_SplineMotionHuman / Girl)", "bool", "false", "play the Timeline backwards / start from the far end (ping-pong)")],
                        usage=["Drag a BP_SplineHuman variant into the level and edit the points of its Spline component to draw the walk path (it loops).",
                               "Or place BP_SplinePath actors, shape their splines, then place BP_SplineMotionHuman actors and set 'Spline To Follow' on each.",
                               "Press Play."]),
    "Vehicles": dict(zip="HERCULES_VehicleBehavior_Blueprints_v%s" % VERSION, title="HERCULES Vehicle Behavior Blueprints", docs=DOCS + "/blueprint_vehicle_behavior/",
                     plugins=["ChaosVehiclesPlugin (enable it in your .uproject; it provides WheeledVehiclePawn)", "Enhanced Input (enabled by default in UE 5.2)"],
                     install_note=" **Install City Sample Vehicles before opening the vehicle Blueprints**: without it `BP_VehicleAI*` have no parent class (`BP_Vehicle`), the editor cannot place them, and pressing Compile on such a Blueprint can crash UE 5.2.",
                     blurb="VehicleAI traffic: BP_VehicleAI is a child of the City Sample BP_Vehicle (Chaos wheeled vehicle). On BeginPlay it finds the nearest BP_SplineRoad, then every tick steers toward a look-ahead point on that spline, brakes from a float curve and keeps its speed between Min Speed and Max Speed. The _taxi/_Van/_pickup/... children only swap the vehicle meshes and wheels. BP_SplineVehicle is a kinematic spline follower for non-physics vehicles.",
                     params=[("Min Speed / Max Speed (BP_VehicleAI + children)", "mph (compared against GetForwardSpeedMPH)", "5 / 25", "throttle is applied below Min Speed and released above Max Speed"),
                             ("AdditionalOffset", "cm", "see graph", "look-ahead distance along the road spline used for steering"),
                             ("Debug Steering / Debug Brake", "bool", "true", "draw the steering target / brake debug lines (turn off for data collection)"),
                             ("Spline Speed / Distance Along Spline (BP_SplineVehicle)", "cm/s / cm", "150 / 0", "kinematic follower speed and start offset")],
                     usage=["Place BP_SplineRoad actors and shape their splines along the lanes (vehicles follow the nearest road spline).",
                            "Drag BP_VehicleAI children onto the roads (Auto Possess AI: Placed in World or Spawned), or run PythonClient/hero/spawn_NPC_cars_splines_UE5.py in the editor to sprinkle random vehicles along every BP_SplineRoad.",
                            "Press Play."]),
}


def main():
    man = json.load(open(MANIFEST))
    assets = man["assets"]
    if man.get("file_audit_stale"):
        sys.exit("manifest reports stale paths: %s" % man["file_audit_stale"])
    bad = [k for k, e in assets.items() if not e.get("load_ok") or "compile_error" in e or "load_error" in e]
    if bad:
        sys.exit("manifest reports problems: %s" % bad)
    os.makedirs(OUT, exist_ok=True)
    for fam, info in FAMILIES.items():
        folder = os.path.join(SRC, fam)
        files = sorted(f for f in os.listdir(folder) if f.endswith(".uasset"))
        if not files:
            sys.exit("no assets in " + folder)
        bps = {}
        packs_used = {}
        for f in files:
            pkg = "/Game/HERCULES/Agents/%s/%s" % (fam, f[:-7])
            e = assets[pkg]
            tp = sorted(d for d, c in list(e["deps_hard"].items()) + list(e["deps_soft"].items()) if c.startswith("thirdparty"))
            herc = sorted(d for d, c in e["deps_hard"].items() if c == "hercules")
            if any(not d.startswith("/Game/HERCULES/Agents/%s/" % fam) for d in herc):
                sys.exit("%s depends on another family: %s" % (pkg, herc))
            mesh = next((c.get("skeletal_mesh_asset") or c.get("static_mesh") for c in e.get("components", []) if c.get("skeletal_mesh_asset") or c.get("static_mesh")), None)
            bps[f[:-7]] = {"parent_class": e.get("ParentClass"), "native_parent_class": e.get("NativeParentClass"),
                           "hercules_dependencies": herc, "third_party_assets": tp,
                           "engine_dependencies": sorted(d for d, c in e["deps_hard"].items() if c == "engine"),
                           "main_mesh": mesh, "class_defaults": {k: v for k, v in e.get("cdo_values", {}).items() if k != "_error" and not k.startswith("ai_") and k != "auto_possess_ai"}}
            for d in tp:
                packs_used.setdefault(d.split("/")[2], set()).add(f[:-7])
        manifest = {"package": info["zip"], "version": VERSION, "generated": time.strftime("%Y-%m-%d"), "unreal_engine": "5.2.1",
                    "install_path": "Content/HERCULES/Agents/%s" % fam, "required_plugins": info["plugins"],
                    "third_party_packs": {p: dict(PACKS[p], used_by=sorted(packs_used[p])) for p in sorted(packs_used)},
                    "blueprints": bps, "docs": info["docs"], "repository": REPO, "license": "MIT (HERCULES Blueprints only; third-party packs keep their own licenses and are not included)"}
        readme = build_readme(fam, info, bps, packs_used)
        zpath = os.path.join(OUT, info["zip"] + ".zip")
        top = info["zip"] + "/"
        with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
            z.writestr(top + "README.md", readme)
            z.writestr(top + "MANIFEST.json", json.dumps(manifest, indent=1, sort_keys=True))
            z.write(LICENSE, top + "LICENSE")
            for f in files:
                z.write(os.path.join(folder, f), top + "Content/HERCULES/Agents/%s/%s" % (fam, f))
        names = [n.filename for n in zipfile.ZipFile(zpath).infolist()]
        outside = [n for n in names if n.endswith(".uasset") and "/Content/HERCULES/Agents/" not in n]
        if outside:
            sys.exit("non-HERCULES content in zip: %s" % outside)
        print("%s: %d blueprints, %d files, %.1f KB, packs %s" % (os.path.basename(zpath), len(files), len(names), os.path.getsize(zpath) / 1024.0, sorted(packs_used)))
        open(os.path.join(OUT, info["zip"] + ".MANIFEST.json"), "w").write(json.dumps(manifest, indent=1, sort_keys=True))
        open(os.path.join(OUT, info["zip"] + ".README.md"), "w").write(readme)


def build_readme(fam, info, bps, packs_used):
    L = ["# %s (v%s)" % (info["title"], VERSION), "",
         info["blurb"], "",
         "Part of [HERCULES](%s). Full documentation: <%s>." % (REPO, info["docs"]), "",
         "## What is in the zip", "",
         "`Content/HERCULES/Agents/%s/` with %d HERCULES-authored Blueprints (listed below), this README, `MANIFEST.json` "
         "(per-Blueprint dependency list) and the MIT `LICENSE`. **No third-party content is included**: the meshes, "
         "animations and materials come from the packs listed under Requirements, which you install yourself." % (fam, len(bps)), "",
         "| Blueprint | Parent | Mesh it expects | Third-party pack(s) |", "|---|---|---|---|"]
    for n in sorted(bps):
        b = bps[n]
        par = (b["parent_class"] or "?").split("/")[-1].rstrip("'").replace(".", " ").split(" ")[-1]
        mesh = (b["main_mesh"] or "-").split("/")[-1].split(".")[0]
        packs = sorted(set(d.split("/")[2] for d in b["third_party_assets"]))
        L.append("| `%s` | %s | %s | %s |" % (n, par, mesh, ", ".join(packs) or "-"))
    L += ["", "## Requirements", "",
          "* Unreal Engine **5.2.1** (the version HERCULES is built against; newer 5.x should open the Blueprints after the usual asset upgrade).",
          "* Plugins: %s." % "; ".join(info["plugins"]),
          "* The HERCULES AirSim plugin if you want the agents in the robots' sensor streams (ground-truth segmentation, LWIR, NVG): <%s/install_precompiled/>." % DOCS,
          "* Third-party packs, installed at the **content paths below** (the Blueprints reference them by path):", ""]
    L += ["| Pack folder | What it is | Where to get it | Expected content path | Needed by |", "|---|---|---|---|---|"]
    for p in sorted(packs_used):
        d = PACKS[p]
        L.append("| `%s` | %s%s | %s (%s) | `%s` | %s |" % (p, d["title"], (" — " + d["note"]) if d["note"] else "", d["url"], d["cost"], d["path"], ", ".join(sorted(packs_used[p]))))
    L += ["", "## Install", "",
          "1. Unzip; copy the `Content` folder onto your project root so the Blueprints land in `<Project>/Content/HERCULES/Agents/%s/`." % fam,
          "2. Install the packs from the table into the same project (Fab / Epic Games Launcher → *Add to project*). They must end up at the content paths listed above, which is where the launcher puts them by default.",
          "3. Open the project. If a pack is missing you get one *Failed to load* warning per missing asset and the affected variant shows an empty mesh; everything else works." + info.get("install_note", ""),
          "", "## Usage", ""]
    L += ["%d. %s" % (i + 1, u) for i, u in enumerate(info["usage"])]
    L += ["", "## Parameters", "", "| Parameter | Unit | Default | Meaning |", "|---|---|---|---|"]
    L += ["| %s | %s | %s | %s |" % p for p in info["params"]]
    L += ["", "## Sensor integration (HERCULES)", "",
          "Editor-placed actors are registered with the instance-segmentation system automatically at simulation start, "
          "so these agents appear in the segmentation, LWIR and NVG streams without extra calls. The LWIR camera classifies "
          "by name keywords (`kangaroo`, `deer`, `animal`, `human`, `person` → warm body; `car`, `truck`, `vehicle` → engine-warm "
          "metal), which the default actor labels (`BP_Kangaroo2`, `BP_SplineHuman_Type3_4`, `BP_VehicleAI_taxi_7`, ...) already satisfy. "
          "Only actors you spawn at runtime need `AddNewActorToInstanceSegmentation` (BlueprintCallable on the SimMode actor) or the "
          "`simAddSegmentationActor` RPC. See <%s/blueprint_packages/>." % DOCS,
          "", "## License", "",
          "The Blueprints in this package are released under the MIT license of the HERCULES repository (see `LICENSE`). "
          "The third-party packs are **not** redistributed and keep their own Fab / Marketplace licenses.", ""]
    return "\n".join(L)


if __name__ == "__main__":
    main()
