@echo off
REM usage: update_plugin.bat <project folder>
REM Copies the plugins (AirSim, HerculesRayTracing) and AirLib into the project, keeping the project's compiled binaries.
setlocal
call "%~dp0env.bat"
robocopy /MIR "%HERCULES_ROOT%\Unreal\Plugins\AirSim" "%~1\Plugins\AirSim" /XD temp Binaries Intermediate /njh /njs /ndl /np /nfl >nul
robocopy /MIR "%HERCULES_ROOT%\AirLib" "%~1\Plugins\AirSim\Source\AirLib" /XD temp /njh /njs /ndl /np /nfl >nul
robocopy /MIR "%HERCULES_ROOT%\Unreal\Plugins\HerculesRayTracing" "%~1\Plugins\HerculesRayTracing" /XD Binaries Intermediate /njh /njs /ndl /np /nfl >nul
exit /b 0
