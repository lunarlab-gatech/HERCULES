@echo off
REM Defaults for the release scripts; set any of these before calling a script to override them.
REM   HERCULES_ROOT  this repository (default: two folders up from this script)
REM   UE_DIR         Unreal Engine 5.8 install
REM   HERCULES_OUT   where packages, staging folders, toolchains and release zips go (default: next to the repository)
REM   VCVARS         Visual Studio's vcvars64.bat
REM   LINUX_MULTIARCH_ROOT  Epic's Linux cross-toolchain, for Linux packages
for %%I in ("%~dp0..\..") do if not defined HERCULES_ROOT set "HERCULES_ROOT=%%~fI"
if not defined UE_DIR set "UE_DIR=C:\Program Files\Epic Games\UE_5.8"
for %%I in ("%HERCULES_ROOT%\..") do if not defined HERCULES_OUT set "HERCULES_OUT=%%~fI"
if not defined VCVARS set "VCVARS=C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
if not defined LINUX_MULTIARCH_ROOT if exist "%HERCULES_OUT%\toolchains\v26_clang-20.1.8-rockylinux8\x86_64-unknown-linux-gnu" set "LINUX_MULTIARCH_ROOT=%HERCULES_OUT%\toolchains\v26_clang-20.1.8-rockylinux8\"
