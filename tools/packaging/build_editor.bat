@echo off
REM usage: build_editor.bat <project folder> <project name>
REM e.g.   build_editor.bat ..\..\Unreal\Environments\HerculesEnvs HerculesEnvs
setlocal
call "%~dp0env.bat"
call "%~dp0update_plugin.bat" "%~f1"
call "%UE_DIR%\Engine\Build\BatchFiles\Build.bat" %2Editor Win64 Development -Project="%~f1\%2.uproject" -WaitMutex -NoHotReload
