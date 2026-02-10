# Building and Packaging HERCULES Plugin and/or Unreal Projects

This document describes how to build and package the HERCULES plugin as a standalone plugin as well as
packaging an entire project including the plugin. 

## Building the HERCULES Unreal Plugin

### Build AirLib
First you need to build the library. 
On Windows:

* Install Visual Studio 2022. Make sure to select Desktop Development with C++ and Windows 10/11 SDK **10.0.X (choose latest)** and select the latest .NET Framework SDK under the 'Individual Components' tab while installing VS 2022. More info [here](https://dev.epicgames.com/documentation/en-us/unreal-engine/setting-up-visual-studio-development-environment-for-cplusplus-projects-in-unreal-engine?application_version=5.2).
* Start `Developer Command Prompt for VS 2022`. 
* Clone the repo: `git clone https://github.com/Cosys-Lab/HERCULES.git`, and go the AirSim directory by `cd HERCULES`. 
* Run `build.cmd` from the command line. This will create ready to use plugin bits in the `Unreal\Plugins` folder.

On Linux:

* Clone the repo: `git clone https://github.com/Cosys-Lab/HERCULES.git`, and go the AirSim directory by `cd HERCULES`. 
* Run `./setup.sh` and `./build.sh` from the command line. This will create ready to use plugin bits in the `Unreal/Plugins` folder.

### Build and package Unreal plugin
Then you can package the plugin as a standalone plugin from a Unreal Project like the provided sample Blocks environment.
On Windows:

* Open the Blocks project in Unreal Engine `cd HERCULES/Unreal/Environments/Blocks` and pull the latest plugin files by running `update_from_git.bat`.
* Go to your Unreal Engine installation folder and run the build script while pointing at the Blocks project: `./RunUAT.bat BuildPlugin -Plugin=....\HERCULES\Unreal\Environments\Blocks\Plugins\AirSim\AirSim.uplugin -Package=....\airsimpluginpackagewin -Rocket -TargetPlatforms=Win64`

On Linux:

* Open the Blocks project in Unreal Engine `cd HERCULES/Unreal/Environments/Blocks` and pull the latest plugin files by running `update_from_git.sh`.
* Go to your Unreal Engine installation folder and run the build script while pointing at the Blocks project: `./RunUAT.sh BuildPlugin -Plugin=..../HERCULES/Unreal/Environments/Blocks/Plugins/AirSim/AirSim.uplugin -Package=..../airsimpluginpackagelinux -Rocket -TargetPlatforms=Linux`

## Building an Unreal Project with HERCULES Plugin

### Build AirLib
First you need to build the library. 
On Windows:

* Install Visual Studio 2022. Make sure to select Desktop Development with C++ and Windows 10/11 SDK **10.0.X (choose latest)** and select the latest .NET Framework SDK under the 'Individual Components' tab while installing VS 2022. More info [here](https://dev.epicgames.com/documentation/en-us/unreal-engine/setting-up-visual-studio-development-environment-for-cplusplus-projects-in-unreal-engine?application_version=5.2).
* Start `Developer Command Prompt for VS 2022`. 
* Clone the repo: `git clone https://github.com/Cosys-Lab/HERCULES.git`, and go the AirSim directory by `cd HERCULES`. 
* Run `build.cmd` from the command line. This will create ready to use plugin bits in the `Unreal\Plugins` folder.

On Linux:

* Clone the repo: `git clone https://github.com/Cosys-Lab/HERCULES.git`, and go the AirSim directory by `cd HERCULES`. 
* Run `./setup.sh` and `./build.sh` from the command line. This will create ready to use plugin bits in the `Unreal/Plugins` folder.

### Build and package Unreal Project
Then you can package the plugin as a standalone plugin from a Unreal Project like the provided sample Blocks environment.
On Windows:

* Open the Blocks project in Unreal Engine `cd HERCULES/Unreal/Environments/Blocks` and pull the latest plugin files by running `update_from_git.bat`.
* Go to your Unreal Engine installation folder and run the build script while pointing at the Blocks project: `./RunUAT.bat BuildCookRun -cook -noP4 -build -stage -noiterate -archive -project=....\HERCULES\Unreal\Environments\Blocks\Blocks.uproject -archivedirectory=....\blockswin -Rocket -TargetPlatforms=Win64 -configuration=Development`

On Linux:

* Open the Blocks project in Unreal Engine `cd HERCULES/Unreal/Environments/Blocks` and pull the latest plugin files by running `update_from_git.sh`.
* Go to your Unreal Engine installation folder and run the build script while pointing at the Blocks project: `./RunUAT.sh BuildCookRun -nop4 -utf8output -nocompileeditor -skipbuildeditor -cook -project="..../HERCULES/Unreal/Environments/Blocks/Blocks.uproject" -target=Blocks -platform=Linux -installed -stage -archive -package -build -pak -iostore -compressed -prereqs -archivedirectory="..../blockslinux/" -clientconfig=Development -nocompile -nocompileuat`




