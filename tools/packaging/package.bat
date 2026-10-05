@echo off
REM usage: package.bat <project folder> <project name> <package name> <Win64|Linux> [<map>]
REM e.g.   package.bat ..\..\Unreal\Environments\Blocks Blocks Blocks Win64
REM        package.bat ..\..\Unreal\Environments\HerculesEnvs HerculesEnvs Cesium Linux /Game/Hercules/Maps/HerculesCesium
REM Output: %HERCULES_OUT%\packages\<package name>_<platform>. With a map, only that map is cooked and it
REM opens by default (desktop builds ignore a staged command line, so the project config is pointed at
REM the map for the build and restored afterwards).
setlocal
call "%~dp0env.bat"
set "PROJ=%~f1"
set "MAPARG="
if "%~5"=="" goto :build
set "MAPARG=-map=%~5+/AirSim/AirSimAssets"
copy /y "%PROJ%\Config\DefaultEngine.ini" "%PROJ%\Config\DefaultEngine.ini.release-backup" >nul
copy /y "%PROJ%\Config\DefaultGame.ini" "%PROJ%\Config\DefaultGame.ini.release-backup" >nul
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0set_default_map.ps1" "%PROJ%" "%~5"

:build
call "%~dp0update_plugin.bat" "%PROJ%"
call "%UE_DIR%\Engine\Build\BatchFiles\RunUAT.bat" BuildCookRun -project="%PROJ%\%2.uproject" -noP4 -platform=%4 -clientconfig=Development -build -cook %MAPARG% -stage -pak -compressed -prereqs -archive -archivedirectory="%HERCULES_OUT%\packages\%3_%4" -stagingdirectory="%HERCULES_OUT%\staging\%3_%4" -utf8output
set "RESULT=%ERRORLEVEL%"
if "%~5"=="" goto :done
move /y "%PROJ%\Config\DefaultEngine.ini.release-backup" "%PROJ%\Config\DefaultEngine.ini" >nul
move /y "%PROJ%\Config\DefaultGame.ini.release-backup" "%PROJ%\Config\DefaultGame.ini" >nul

:done
exit /b %RESULT%
