@echo off
setlocal
pushd %~dp0

rem install Visual Studio 2022 Community

if "%VSINSTALLDIR%"=="" call "%ProgramFiles%\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

msbuild NavServer.sln -p:Configuration=Release
rem msbuild NavServer.sln -target:NavServer -p:Configuration=Release

pause
