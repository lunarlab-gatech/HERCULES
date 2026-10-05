@echo off
REM Builds AirLib, rpclib and MavLinkCom (Release) for Windows with Visual Studio, like build.cmd
REM from a developer prompt. The Linux libraries come from build.sh --ue-toolchain (see README.md).
setlocal
call "%~dp0env.bat"
call "%VCVARS%" >nul || exit /b 1
cd /d "%HERCULES_ROOT%"
REM build.cmd mirrors the Windows MavLinkCom libraries over AirLib\deps\MavLinkCom\lib, which would
REM delete the Linux library copied there: set it aside and put it back
set "LINUX_MAVLINK=AirLib\deps\MavLinkCom\lib\libMavLinkCom.a"
if exist "%LINUX_MAVLINK%" copy /y "%LINUX_MAVLINK%" "%TEMP%\hercules-libMavLinkCom.a" >nul
call build.cmd --Release
set "RESULT=%ERRORLEVEL%"
if exist "%TEMP%\hercules-libMavLinkCom.a" move /y "%TEMP%\hercules-libMavLinkCom.a" "%LINUX_MAVLINK%" >nul
exit /b %RESULT%
