@echo off
REM usage: build_levels.bat <level> [<level>...]   (levels: blank cesium)
REM Generates HerculesEnvs levels headless with Scripts/build_levels.py.
setlocal
call "%~dp0env.bat"
set "P=%HERCULES_ROOT%\Unreal\Environments\HerculesEnvs"
"%UE_DIR%\Engine\Binaries\Win64\UnrealEditor-Cmd.exe" "%P%\HerculesEnvs.uproject" -run=pythonscript -script="%P%\Scripts\build_levels.py %*" -unattended -nop4 -nosplash -stdout -FullStdOutLogOutput
