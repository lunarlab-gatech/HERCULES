# Packaged HERCULES Environments

Packaged (cooked) builds of the HERCULES environments run without Unreal Engine installed. Each package is a standalone game with the HERCULES plugin built in. Download the zip for your platform, unzip it, and connect to it with the Python client or the ROS 2 bridges from this repository.

## Environments

| Package | World | Default robots |
| --- | --- | --- |
| `HERCULES-Blocks` | Cosys-AirSim's Blocks test world | `Drone1`, `Drone2`, `Husky1`, `Husky2` with `settings/hero_blocks_team.json` |
| `HERCULES-Cesium` | Anywhere on Earth, streamed from [Cesium ion](https://cesium.com/platform/cesium-ion/) (Google Photorealistic 3D Tiles); see [Cesium world](#cesium-world) | same |

The packages are built with Unreal Engine 5.8.3 from the `hercules-envs` branch of HERCULES. Each zip holds a `VERSION.txt` with the exact commit.

## Requirements

- Windows 10/11 (x64) with DirectX 12, or Linux x86-64 (Ubuntu 22.04 or newer) with a GPU and driver that support Vulkan at Shader Model 6, for example a current NVIDIA or AMD driver. Software Vulkan (lavapipe) and WSL 2 do not meet this; there the Linux build runs only without rendering (`-nullrhi`), so cameras return no images.
- A GPU with 8 GB of video memory or more. The larger worlds benefit from 16 GB.
- The Python client and ROS 2 bridges from this repository, on the same machine or another one on the network.

## Run

Each package unzips to a folder with the game, a `settings` folder of ready settings files, and `VERSION.txt`. Start it with `run_hero_team.bat` (Windows) or `./run_hero_team.sh` (Linux), which open the game in a window with the two-drone, two-Husky team of `settings/hero_blocks_team.json`. Arguments you add are passed to the game. To use another settings file, start the game directly:

```bat
:: Windows (Blocks.exe in the Blocks package, HerculesEnvs.exe in the others)
Blocks.exe -windowed -ResX=1280 -ResY=720 -settings="%CD%\settings\hero_blocks_team_network.json"
```

```bash
# Linux
./Blocks.sh -windowed -ResX=1280 -ResY=720 -settings="$PWD/settings/hero_blocks_team_network.json"
```

Without `-settings`, the game reads `Documents/AirSim/settings.json` (see [settings](settings.md#where-are-settings-stored)), and without that it asks which vehicle type to simulate. In the team settings (`"SimMode": "Hero"`) the game serves drones on RPC port 41451 and ground vehicles on 41452.

Useful options:

| Option | Effect |
| --- | --- |
| `-windowed -ResX=1280 -ResY=720` | Run in a window instead of full screen |
| `-RenderOffscreen` | Run without a window, for example on a server; cameras still render |
| `-log` | Open a log console (Windows); the log is also written to `<Project>/Saved/Logs` |
| `-settings=<file or JSON>` | HERCULES settings to use |

## Connect

From the same machine, the Python client connects to `127.0.0.1`:

```python
import hercules_cosysairsim as airsim
drones = airsim.MultirotorClient(port=41451)
ugvs = airsim.CarClient(port=41452)
print(drones.listVehicles())
```

and the ROS 2 bridges start with `ros2 launch airsim_ros_pkgs hercules_hero_team.launch.py`; see [Running a UAV–UGV Team](hero_team_quickstart.md) for flying, driving, recording and planning.

From another machine, or from WSL 2 when the game runs on Windows:

- Use the settings files ending in `_network.json`. They set `"LocalHostIp": "0.0.0.0"` so the game accepts connections from other addresses; the regular files only accept connections from the same machine.
- Allow the game through the firewall. On Windows, answer **Allow** when Windows Firewall asks the first time the game starts. If you answered **Block**, change the inbound rules for the game's `.exe` under *Windows Defender Firewall → Advanced settings*.
- Pass the game machine's address, for example `MultirotorClient(ip="172.28.160.1", port=41451)` or `host_ip:=172.28.160.1` for the ROS 2 launch file. From WSL 2 with default (NAT) networking, the Windows address is the default gateway: `ip route show default`.

## Cesium world

The Cesium package streams photorealistic 3D tiles of a real place at run time. It needs an internet connection and a free [Cesium ion](https://ion.cesium.com/signup) account. No access token is built into the package; supply your own:

1. In Cesium ion, add **Google Photorealistic 3D Tiles** (asset 2275207) to *My Assets* from the Asset Depot, and create an access token under *Access Tokens*.
2. Start the game. A window asks where the robots should start (latitude and longitude, with presets for Caltech and Georgia Tech) and for the token, unless the `CESIUM_ION_TOKEN` environment variable or `-CesiumIonToken=<token>` gives it. The location is remembered for the next launch; the token is not.

For scripted runs, give the place on the command line, which skips the window:

```bash
CESIUM_ION_TOKEN=<token> ./HerculesEnvs.sh -CesiumOrigin=33.775620,-84.396285 -settings="$PWD/settings/hero_blocks_team.json"
```

| Option | Effect |
| --- | --- |
| `-CesiumOrigin=<lat>,<lon>[,<height m>]` | Where the robots start; without a height, the ground height is looked up |
| `-CesiumIonToken=<token>` | Cesium ion access token (or set `CESIUM_ION_TOKEN`); prefer the environment variable, since the game logs its command line |
| `-CesiumIonAssets=<id>[,<id>]` | Other ion assets to stream, for example `1,96188` for Cesium World Terrain with OSM Buildings |
| `-CesiumSpawnSearchRadius=<m>` | How far to look for open ground for the robots (default 150, `0` to stay at the given place) |
| `-CesiumMaxScreenSpaceError=<px>` | Tile detail; lower is finer (default 2, Cesium's own default is 16). Raise it on slow connections or small GPUs |
| `-CesiumMaxSimultaneousTileLoads=<n>` | Tile requests in flight (default 64, Cesium's own default is 20) |
| `-CesiumNoPrompt` | Never show the location window (it is also skipped with `-unattended` and `-RenderOffscreen`) |
| `-CesiumLogSelectionStats` | Log Cesium's tile statistics every frame they change (tiles visited, rendered and waiting to load) |

The robots appear before the tiles have loaded, held up by an invisible floor that only they collide with. Once the tiles around them have loaded, the world is shifted so that the nearest flat, open, ground-level area that fits the whole team lies under the robots, typically within 100–150 m of the requested place; the game logs `Robots placed at lat ..., lon ...`. When tiles are under every robot, the invisible floor stops colliding and the robots settle onto the ground, at most 50 cm below (`the spawn pad no longer holds them`). Wait for that message before commanding the robots. Their start poses are unchanged, and their GPS reports the place they were moved to.

Tiles are refined for the game window and for every robot camera that captures images (a camera starts capturing when its images are first requested), starting with the ground around the robots. Every such camera adds work each frame, so request images only from the cameras you use. The memory budgets for the tiles (the ray tracing geometry pool and the tile cache) are sized to the machine's GPU and system memory at launch. Vehicle physics is substepped, so the robots move in real time down to 7.5 frames per second. With four robots each streaming five image types from one camera at the default detail, the Cesium world ran at about 12 frames per second in the editor on an RTX 5090 workstation, limited by the CPU's work on the tiles, and used about 18 GB of system memory.

In [instance segmentation](instance_segmentation.md), the 3D tiles are one object, separate from the sky and the robots; its name in `simListInstanceSegmentationObjects()` contains `CesiumPhotorealisticTiles` (the rest varies between runs). They are a single photogrammetry mesh, so buildings, trees and roads are not told apart, and the thermal camera treats them as neutral surfaces. Tile detail depends on Google's coverage of the place.

## Licenses

The HERCULES plugin and the Blocks world are MIT-licensed. Worlds built from Fab content are distributed in packaged form only, as the Fab license permits; the raw assets are not included. To open or modify a world in the Unreal Editor, download the packs it uses yourself (see [Downloading the HERCULES Environments from Fab](downloading_hercules_environments.md)).
