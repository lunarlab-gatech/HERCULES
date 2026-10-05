@echo off
REM usage: copy_cesium.bat <project folder>
REM Copies Cesium for Unreal from the engine's Marketplace plugins into the project, so it is compiled
REM from source for every target platform (the Fab install only carries Win64 binaries), and patches it
REM with patch_cesium.ps1 for Linux builds and headless (-nullrhi) games.
setlocal
call "%~dp0env.bat"
set "SRC="
for /d %%D in ("%UE_DIR%\Engine\Plugins\Marketplace\Cesium*") do set "SRC=%%~fD"
if not defined SRC (echo Cesium for Unreal is not installed in %UE_DIR% & exit /b 1)
robocopy "%SRC%" "%~f1\Plugins\CesiumForUnreal" /E /XD Binaries Intermediate Android-aarch64-Release Darwin-universal-Release iOS-arm64-Release /njh /njs /ndl /np /nfl >nul
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0patch_cesium.ps1" -Plugin "%~f1\Plugins\CesiumForUnreal" || exit /b 1
exit /b 0
