@echo off
setlocal
pushd %~dp0

luajit.exe NavServerFixCode.lua

pause
