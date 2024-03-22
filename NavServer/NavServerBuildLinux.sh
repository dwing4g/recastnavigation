#!/bin/sh

cd `dirname $0`

if [ -z $JAVA_HOME ]; then JAVA_HOME=/usr/java/default; fi

JAVA_VER=$($JAVA_HOME/bin/java -version 2>&1 | awk -F '"' '/version/ {print $2}' | awk -F '.' '{sub("^$", "0", $2); print $1$2}')
if [[ "$JAVA_VER" -le 18 ]]; then echo 'ERROR: JDK 1.8 or later is required!'; exit 1; fi

COMMON_FILES="\
../Detour/Source/DetourAlloc.cpp \
../Detour/Source/DetourAssert.cpp \
../Detour/Source/DetourCommon.cpp \
../Detour/Source/DetourNavMesh.cpp \
../Detour/Source/DetourNavMeshBuilder.cpp \
../Detour/Source/DetourNavMeshQuery.cpp \
../Detour/Source/DetourNode.cpp \
../Recast/Source/Recast.cpp \
../Recast/Source/RecastAlloc.cpp \
../Recast/Source/RecastArea.cpp \
../Recast/Source/RecastAssert.cpp \
../Recast/Source/RecastContour.cpp \
../Recast/Source/RecastFilter.cpp \
../Recast/Source/RecastLayers.cpp \
../Recast/Source/RecastMesh.cpp \
../Recast/Source/RecastMeshDetail.cpp \
../Recast/Source/RecastRasterization.cpp \
../Recast/Source/RecastRegion.cpp \
NavServer.cpp \
"

SO_FILES="\
NavServerJNI.cpp \
"

TEST_FILES="\
NavServerTest.cpp \
"

# for C++11
COMPILE="g++ -std=c++11 -DDT_POLYREF64=1 -DNDEBUG -D__STDC_LIMIT_MACROS -I. -I../Detour/Include -I../Recast/Include -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -m64 -O3 -fPIC -pipe -Wall"
# COMPILE="g++ -std=c++0x -DDT_POLYREF64=1 -DNDEBUG -D__STDC_LIMIT_MACROS -Dconstexpr=const -I. -I../Detour/Include -I../Recast/Include -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -m64 -O3 -fPIC -pipe -Wall"

echo building librecastjni64.so ...
$COMPILE -shared -fvisibility=hidden -Wl,-soname -Wl,librecastjni64.so -o librecastjni64.so $COMMON_FILES $SO_FILES

echo building NavServerTest ...
$COMPILE -o NavServerTest $COMMON_FILES $TEST_FILES
