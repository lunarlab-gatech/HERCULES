# Install or Build HERCULES on Linux

HERCULES targets **Unreal Engine 5.8**. The ROS 2 wrappers target Ubuntu 24.04 (ROS 2 Jazzy) and Ubuntu 26.04 (ROS 2 Lyrical).


## Install Unreal Engine
Download Unreal Engine 5.8 from the [official download page](https://www.unrealengine.com/en-US/linux). 
This will require an Epic Games account. Once the zip archive is downloaded you can extract it to where you want to install the Unreal Engine.
```bash
unzip Linux_Unreal_Engine_5.8.3.zip -d destination_folder
```
If you chose a folder such as for example `/opt/UnrealEngine` make sure to provide permissions and to set the owner, otherwise you might run into issues:
```bash
sudo chmod -R 777 /opt/UnrealEngine
sudo chown -R yourusername /opt/UnrealEngine
```
From where you install Unreal Engine, you can run `Engine/Binaries/Linux/UnrealEditor` from the terminal to launch Unreal Engine.
For more information you can read the [quick start guide](https://dev.epicgames.com/documentation/en-us/unreal-engine/linux-development-quickstart-for-unreal-engine?application_version=5.8).

You can alternatively install Unreal Engine from source if you do not use a Ubuntu distribution, see the documentation linked above for more information. 

## Build HERCULES
- Clone HERCULES and build it:
   ```bash
   # go to the folder where you clone GitHub projects
   git clone https://github.com/lunarlab-gatech/HERCULES.git
   cd HERCULES
   ./setup.sh
   ./build.sh
   ```
- On Ubuntu 24.04 and newer (the releases for ROS 2 Jazzy and Lyrical), `clang-12` is no longer packaged. Build with Unreal Engine's bundled toolchain instead, passing the folder that contains `Engine/`:
   ```bash
   ./build.sh --ue-root /path/to/UnrealEngine
   ```
   You can also `export UE_ROOT=/path/to/UnrealEngine` once and run `./build.sh`. This builds AirLib with the same compiler, sysroot and libc++ as the plugin, which avoids link errors such as `undefined symbol: __isoc23_strtol` from the newer system glibc.
- Without a Linux Unreal Engine install, for example to build the Linux libraries for packaging a Linux game from an engine installed on Windows, download Epic's native Linux toolchain for your engine version (`native-linux-v26_clang-20.1.8-rockylinux8.tar.gz` for Unreal Engine 5.8, from `https://cdn.unrealengine.com/Toolchain_Linux/`) and pass its `x86_64-unknown-linux-gnu` folder:
   ```bash
   ./build.sh --ue-toolchain /path/to/v26_clang-20.1.8-rockylinux8/x86_64-unknown-linux-gnu
   ```
   This writes `AirLib/lib/libAirLib.a`, `AirLib/deps/rpclib/lib/librpc.a` and `AirLib/deps/MavLinkCom/lib/libMavLinkCom.a`, which the plugin links when Unreal Engine builds a Linux target.

## Build Unreal Environment

Finally, you will need an Unreal project that hosts the environment for your vehicles. HERCULES includes the scaffolding of the "Blocks Environment", but its `Blocks.uproject` and `Content/` are not in the repository; [Running a UAV–UGV team](hero_team_quickstart.md) shows how to fetch them. You can also create your own environment; see [setting up Unreal Environment](unreal_proj.md).

## How to Use HERCULES

Once HERCULES is setup:
- Navigate to the environment folder (for example for BLocks it is `./Unreal/Environments/Blocks`), and run `update_from_git.sh`.
- Go to `UnrealEngine` installation folder and start Unreal by running `./Engine/Binaries/Linux/UnrealEditor`.
- When Unreal Engine prompts for opening or creating project, select Browse and choose `HERCULES/Unreal/Environments/Blocks` (or your [custom](unreal_custenv.md) Unreal project).
- Alternatively, the project file can be passed as a commandline argument. For Blocks: `./Engine/Binaries/Linux/UnrealEditor <Cosys-AirSim_path>/Unreal/Environments/Blocks/Blocks.uproject`
- If you get prompts to convert project, look for More Options or Convert-In-Place option. If you get prompted to build, choose Yes. If you get prompted to disable Cosys-AirSim plugin, choose No.
- After Unreal Editor loads, press Play button.

See [Using APIs](apis.md) and [settings.json](settings.md) for various options available for HERCULES usage.

!!! tip
Go to 'Edit->Editor Preferences', in the 'Search' box type 'CPU' and ensure that the 'Use Less CPU when in Background' is unchecked.

### [Optional] Setup Remote Control (Multirotor Only)

A remote control is required if you want to fly manually. See the [remote control setup](remote_control.md) for more details.

Alternatively, you can use [APIs](apis.md) for programmatic control or use the so-called [Computer Vision mode](image_apis.md) to move around using the keyboard.
