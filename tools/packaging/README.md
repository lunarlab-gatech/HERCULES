# Building the HERCULES Release Packages

These scripts build the packaged worlds described in [Packaged HERCULES Environments](../../docs/packaged_environments.md) with Unreal Engine 5.8 on Windows, for Windows and Linux. The `.bat` scripts run in a Windows command prompt; `assemble_release.sh` runs in WSL or Linux.

All scripts read their locations from [`env.bat`](env.bat); set any of these first to override the defaults:

| Variable | Default | Meaning |
| --- | --- | --- |
| `UE_DIR` | `C:\Program Files\Epic Games\UE_5.8` | Unreal Engine 5.8, with the Linux target platform installed (Epic Games Launcher → UE 5.8 → Options) |
| `HERCULES_OUT` | the folder containing the repository | gets `packages\`, `staging\`, `release\` and `toolchains\` |
| `VCVARS` | Visual Studio 2026 Community's `vcvars64.bat` | Visual Studio with an MSVC that UE 5.8 accepts |
| `LINUX_MULTIARCH_ROOT` | `%HERCULES_OUT%\toolchains\v26_clang-20.1.8-rockylinux8\` if present | Epic's Linux cross-toolchain |

## 1. One-time setup

- **Linux cross-toolchain.** Download `https://cdn.unrealengine.com/CrossToolchain_Linux/v26_clang-20.1.8-rockylinux8.exe`. Run it, or, without administrator rights, unpack it with 7-Zip (`7zz x -tNsis`) into `%HERCULES_OUT%\toolchains\v26_clang-20.1.8-rockylinux8`.
- **Linux libraries for the plugin.** In WSL or on Linux, download Epic's native toolchain (`https://cdn.unrealengine.com/Toolchain_Linux/native-linux-v26_clang-20.1.8-rockylinux8.tar.gz`), then from a clone of this repository run `./setup.sh` (or fetch rpclib 2.3.1 and Eigen 3.4.1r as it does) and `./build.sh --ue-toolchain <toolchain>/x86_64-unknown-linux-gnu`. Copy `AirLib/lib/libAirLib.a`, `AirLib/deps/rpclib/lib/librpc.a` and `AirLib/deps/MavLinkCom/lib/libMavLinkCom.a` into the same places in the Windows checkout. Rebuild them whenever AirLib changes (`build_airlib.bat` keeps them in place).
- **Cesium for Unreal**, installed into UE 5.8 from Fab. `copy_cesium.bat` copies it into the HerculesEnvs project, so that it is built from source for Linux as well, and patches it (`patch_cesium.ps1`) for Linux builds and headless (`-nullrhi`) games:

  ```bat
  copy_cesium.bat ..\..\Unreal\Environments\HerculesEnvs
  ```

- **Blocks content.** `Blocks.uproject` and `Content/` come from Cosys-AirSim 5.8-v3.5.0; see [Running a UAV–UGV Team](../../docs/hero_team_quickstart.md#2-create-the-blocks-project).

## 2. Build

```bat
build_airlib.bat
build_editor.bat ..\..\Unreal\Environments\Blocks Blocks
build_editor.bat ..\..\Unreal\Environments\HerculesEnvs HerculesEnvs
build_levels.bat blank cesium
```

`build_levels.bat` regenerates the HerculesEnvs levels in `Content/Hercules/Maps` with [`Scripts/build_levels.py`](../../Unreal/Environments/HerculesEnvs/Scripts/build_levels.py).

## 3. Package

```bat
package.bat ..\..\Unreal\Environments\Blocks Blocks Blocks Win64
package.bat ..\..\Unreal\Environments\Blocks Blocks Blocks Linux
package.bat ..\..\Unreal\Environments\HerculesEnvs HerculesEnvs Cesium Win64 /Game/Hercules/Maps/HerculesCesium
package.bat ..\..\Unreal\Environments\HerculesEnvs HerculesEnvs Cesium Linux /Game/Hercules/Maps/HerculesCesium
```

With a map, a package cooks only that map and opens it by default. Packages go to `%HERCULES_OUT%\packages\<name>_<platform>`.

## 4. Test

Start a package with the team settings (`-settings=...\ros2\settings\hero_blocks_team.json`, or the `_network` variant for a client in WSL or on another machine) and run [`PythonClient/hero/smoke_test_team.py`](../../PythonClient/hero/smoke_test_team.py) against it. A Linux package can be checked without a GPU by starting it with `-nullrhi` and passing `--no-images`.

## 5. Assemble

From WSL or Linux:

```bash
export HERCULES_OUT=/mnt/d/hercules   # the same folder as on Windows
tools/packaging/assemble_release.sh Blocks Win64 "$HERCULES_OUT/packages/Blocks_Win64" ros2/settings/hero_blocks_team.json
tools/packaging/assemble_release.sh Cesium Linux "$HERCULES_OUT/packages/Cesium_Linux" ros2/settings/hero_blocks_team.json
```

Each zip in `$HERCULES_OUT/release` holds the game without its `Saved/` folder (test logs can contain command lines and tokens) or Cesium's request cache (streamed tiles, which can't be redistributed), the settings files plus `_network` copies, `run_hero_team` launchers, the packaged-environments guide as `README.md`, and `VERSION.txt` with the engine version and the commit it was built from. Build from a clean checkout so `VERSION.txt` doesn't report uncommitted changes.
