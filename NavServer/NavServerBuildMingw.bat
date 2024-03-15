@echo off
setlocal
pushd %~dp0

rem install mingw-gcc 4.4+ and append PATH with mingw\bin

set COMMON_FILES=^
..\Detour\Source\DetourAlloc.cpp ^
..\Detour\Source\DetourAssert.cpp ^
..\Detour\Source\DetourCommon.cpp ^
..\Detour\Source\DetourNavMesh.cpp ^
..\Detour\Source\DetourNavMeshBuilder.cpp ^
..\Detour\Source\DetourNavMeshQuery.cpp ^
..\Detour\Source\DetourNode.cpp ^
..\Recast\Source\Recast.cpp ^
..\Recast\Source\RecastAlloc.cpp ^
..\Recast\Source\RecastArea.cpp ^
..\Recast\Source\RecastAssert.cpp ^
..\Recast\Source\RecastContour.cpp ^
..\Recast\Source\RecastFilter.cpp ^
..\Recast\Source\RecastLayers.cpp ^
..\Recast\Source\RecastMesh.cpp ^
..\Recast\Source\RecastMeshDetail.cpp ^
..\Recast\Source\RecastRasterization.cpp ^
..\Recast\Source\RecastRegion.cpp ^
NavServer.cpp

set DLL_FILES=^
NavServerJNI.cpp

set TEST_FILES=^
NavServerTest.cpp

set COMPILE=-std=c++11 -DNDEBUG -D__USE_MINGW_ANSI_STDIO=1 -I. -I..\Detour\Include -I..\Recast\Include -I%JAVA_HOME%\include -I%JAVA_HOME%\include\win32 -O3 -pipe -static -Wall

rem set COMPILE32=i686-w64-mingw32-g++.exe -m32 -march=i686 %COMPILE% -Wl,--enable-stdcall-fixup
set COMPILE64=x86_64-w64-mingw32-g++.exe -m64 %COMPILE%

rem %COMPILE32% -s -shared -Wl,--image-base,0x10000000 -Wl,--kill-at -Wl,-soname -Wl,recastjni32.dll -o recastjni32.dll %COMMON_FILES% %DLL_FILES%
%COMPILE64% -shared -Wl,--image-base,0x10000000 -Wl,--kill-at -Wl,-soname -Wl,recastjni64_mingw.dll -o recastjni64_mingw.dll %COMMON_FILES% %DLL_FILES%

echo ===============================================================================

rem %COMPILE32% -o NavServerTest_mingw32.exe %COMMON_FILES% %TEST_FILES%
%COMPILE64% -o NavServerTest_mingw64.exe %COMMON_FILES% %TEST_FILES%

pause
