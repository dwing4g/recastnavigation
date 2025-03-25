#include <float.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <deque>
#include <unordered_set>
#include <algorithm>
#include "NavServer.internal.h"
#ifdef _MSC_VER
#include <io.h>
#else
#include <unistd.h>
#define _access access
#endif

static clock_t t;

static void resetTimer()
{
    t = clock();
}

static int getTime()
{
    return static_cast<int>((static_cast<long long>(clock()) - t) * 1000 / CLOCKS_PER_SEC);
}

static void printMem()
{
    printf("-------- mem = %lld/%lld, %lld bytes\n",
        static_cast<long long>(navGetAllocCount()),
        static_cast<long long>(navGetFreeCount()),
        static_cast<long long>(navGetMemSize()));
}

void printCenterPos(const dtNavMesh& navMesh, dtPolyRef polyRef)
{
    const dtMeshTile* tile;
    const dtPoly* poly;
    navMesh.getTileAndPolyByRefUnsafe(polyRef, &tile, &poly);
    float cx = 0, cy = 0, cz = 0;
    for (int i = 0, n = poly->vertCount; i < n; ++i)
    {
        const int vertIdx = poly->verts[i];
        const float* const vert = tile->verts + 3 * vertIdx;
        cx += vert[0];
        cy = std::max(cy, vert[1]);
        cz += vert[2];
    }
    cx /= poly->vertCount;
    cz /= poly->vertCount;
    const dtPolyRef baseRef = navMesh.getPolyRefBase(tile);
    printf("%9.3f,%9.3f,%9.3f, tile:%2d,%2d, poly:%lld", cx, cy, cz, tile->header->x, tile->header->y, (long long)(polyRef - baseRef));
}

void markPoly(const dtNavMesh& navMesh, dtPolyRef polyRef, std::unordered_set<dtPolyRef>& markedPolyRefs)
{
    std::deque<dtPolyRef> queue;
    queue.emplace_back(polyRef);
    markedPolyRefs.insert(polyRef);
    while (!queue.empty())
    {
        polyRef = queue.front();
        queue.pop_front();
        const dtMeshTile* tile;
        const dtPoly* poly;
        navMesh.getTileAndPolyByRefUnsafe(polyRef, &tile, &poly);
        for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
        {
            const dtPolyRef nextRef = tile->links[k].ref;
            if (nextRef && markedPolyRefs.find(nextRef) == markedPolyRefs.end())
            {
                queue.emplace_back(nextRef);
                markedPolyRefs.insert(nextRef);
            }
        }
    }
}

void traceNavMesh(const dtNavMesh& navMesh)
{
    printf("traceNavMesh begin:\n");
    std::unordered_set<dtPolyRef> markedPolyRefs;
    int group = 0, markedCount = 0;
    for (int i = 0, n = navMesh.getMaxTiles(); i < n; ++i)
    {
        const dtMeshTile* const tile = navMesh.getTile(i);
        if (!tile || !tile->header)
            continue;
        const dtPolyRef baseRef = navMesh.getPolyRefBase(tile);
        for (int j = 0, m = tile->header->polyCount; j < m; ++j)
        {
            const dtPolyRef polyRef = baseRef + j;
            if (markedPolyRefs.find(polyRef) == markedPolyRefs.end())
            {
                printf("group:%4d, ", group++);
                markPoly(navMesh, polyRef, markedPolyRefs);
                const int count = static_cast<int>(markedPolyRefs.size());
                printf("polyCount:%5d, pos:", count - markedCount);
                printCenterPos(navMesh, polyRef);
                printf("\n");
                markedCount = count;
            }
        }
    }
    printf("traceNavMesh end\n");
}

void checkMap(const dtNavMesh& navMesh, const dtNavMeshQuery& /*navQuery*/)
{
    float a[3] = {0}, b[3] = {0};
    for (int i = 0, n = navMesh.getMaxTiles(); i < n; ++i)
    {
        const dtMeshTile* tile = navMesh.getTile(i);
        if (!tile || !tile->header) continue;
        dtPolyRef baseRef = navMesh.getPolyRefBase(tile);
        for (int j = 0, m = tile->header->polyCount; j < m; ++j)
        {
            dtPolyRef fromRef = baseRef + j;
            const dtMeshTile* fromTile = 0;
            const dtPoly* fromPoly = 0;
            navMesh.getTileAndPolyByRefUnsafe(fromRef, &fromTile, &fromPoly);
            for (unsigned int k = fromPoly->firstLink; k != DT_NULL_LINK; k = fromTile->links[k].next)
            {
                dtPolyRef toRef = fromTile->links[k].ref;
                if (!toRef) continue;
                const dtMeshTile* toTile = 0;
                const dtPoly* toPoly = 0;
                navMesh.getTileAndPolyByRefUnsafe(toRef, &toTile, &toPoly);
                // if (dtStatusFailed(navQuery.getPortalPoints(fromRef, fromPoly, fromTile, toRef, toPoly, toTile, a, b)))
                //     printf("ERROR1");
                if (isnan(a[0]) || isnan(b[0]))
                    printf("ERROR2");
            }
        }
    }
    printf("OK!");
}

bool isPointIn0(const float* vertXZs, int vertCount, float x, float z)
{
    bool res = false;
    const int n = vertCount * 2;
    float ax = vertXZs[n - 2];
    float az = vertXZs[n - 1];
    for (int i = 0; i < n; i += 2)
    {
        const float bx = vertXZs[i];
        const float bz = vertXZs[i + 1];
        if ((bx < x) == (x <= ax))
            res ^= ((ax < bx) == ((z - az) * (bx - ax) < (x - ax) * (bz - az)));
        ax = bx;
        az = bz;
    }
    return res;
}

static float segSegFactor(float ax, float az, float bx, float bz, float cx, float cz, float dx, float dz)
{
    // aabb判断在这个环境下意义不大, 因为都是临近多边形的位置
    // if (Math.min(ax, bx) > Math.max(cx, dx) || Math.min(cx, dx) > Math.max(ax, bx) ||
    //     Math.min(az, bz) > Math.max(cz, dz) || Math.min(cz, dz) > Math.max(az, bz))
    //     return 2;
    const float bax = bx - ax;
    const float baz = bz - az;
    const float dcx = dx - cx;
    const float dcz = dz - cz;
    const float baXdc = bax * dcz - baz * dcx;
    // if (fabsf(baXdc) < 1e-6f)
    // 	return 3;
    const float baXdcInv = 1 / baXdc;
    const float cax = cx - ax;
    const float caz = cz - az;
    const float u = (cax * dcz - caz * dcx) * baXdcInv;
    if (u < 0 || u > 1)
        return 4;
    const float v = (cax * baz - caz * bax) * baXdcInv;
    if (v < 0 || v > 1)
        return 5;
    return u;
}

float segPolyFactor(const float* vertXZs, int vertCount, float ax, float az, float bx, float bz)
{
    const int n = vertCount * 2;
    float x = vertXZs[n - 2];
    float z = vertXZs[n - 1];
    float res = 2;
    for (int i = 0; i < n; i += 2)
    {
        const float xx = vertXZs[i];
        const float zz = vertXZs[i + 1];
        const float f = segSegFactor(ax, az, bx, bz, x, z, xx, zz);
        if (res > f)
            res = f;
        x = xx;
        z = zz;
    }
    return res < 2 ? res : 0;
}

bool fixLine(const float* vertXZs, int vertCount, const float* posFrom, float* posTo)
{
    if (isPointIn0(vertXZs, vertCount, posTo[0], posTo[2]))
        return false;
    const float f = segPolyFactor(vertXZs, vertCount, posFrom[0], posFrom[2], posTo[0], posTo[2]);
    posTo[0] = posFrom[0] + (posTo[0] - posFrom[0]) * f;
    posTo[1] = posFrom[1] + (posTo[1] - posFrom[1]) * f;
    posTo[2] = posFrom[2] + (posTo[2] - posFrom[2]) * f;
    return true;
}

static inline float dist2(float dx, float dz)
{
    return dx * dx + dz * dz;
}

static void updateNearerPos(float x, float z, float x0, float z0, float x1, float z1, float& bestX, float& bestZ, float& minDist2)
{
    const float dx = x1 - x0;
    const float dz = z1 - z0;
    const float pdx = x - x0;
    const float pdz = z - z0;
    const float u = (pdx * dx + pdz * dz) / (dx * dx + dz * dz);
    if (u > 0)
    {
        if (u < 1)
        {
            const float ux = u * dx;
            const float uz = u * dz;
            const float d2 = dist2(pdx - ux, pdz - uz);
            if (minDist2 > d2)
            {
                minDist2 = d2;
                bestX = x0 + ux;
                bestZ = z0 + uz;
            }
        }
        else
        {
            const float d2 = dist2(x - x1, z - z1);
            if (minDist2 > d2)
            {
                minDist2 = d2;
                bestX = x1;
                bestZ = z1;
            }
        }
    }
    else
    {
        const float d2 = dist2(pdx, pdz);
        if (minDist2 > d2)
        {
            minDist2 = d2;
            bestX = x0;
            bestZ = z0;
        }
    }
}

void moveToEdge(const float* vertXZs, int vertCount, float& x, float& z)
{
    const float px = x;
    const float pz = z;
    float minDist2 = FLT_MAX;
    const int n = vertCount * 2;
    float ax = vertXZs[n - 2];
    float az = vertXZs[n - 1];
    for (int i = 0; i < n; i += 2)
    {
        const float bx = vertXZs[i];
        const float bz = vertXZs[i + 1];
        updateNearerPos(px, pz, ax, az, bx, bz, x, z, minDist2);
        ax = bx;
        az = bz;
    }
}

int main1()
{
    const float vertXZs[] =
    {
        320.43377685546877,98.67929077148438,
        313.78375244140627,98.23928833007813,
        298.4437561035156,99.96929168701172,
        290.8837585449219,96.32928466796875,
        294.7337646484375,83.6192855834961,
        297.43377685546877,74.6492919921875,
        317.3237609863281,73.3992919921875,
        323.2337646484375,87.3992919921875,
    };

    float posFrom[] =
    {
//		305,0,89
        300.0,61.96218,74.488014
    };

    float posTo[] =
    {
        299.26703,61.93002,71.39106
    };

    const int vertCount = sizeof(vertXZs) / sizeof(*vertXZs) / 2;
    bool r = isPointIn0(vertXZs, vertCount, posFrom[0], posFrom[2]);
    printf("%d\n", r ? 1 : 0);
    moveToEdge(vertXZs, vertCount, posFrom[0], posFrom[2]);
    printf("%d, %f, %f\n", r ? 1 : 0, posFrom[0], posFrom[2]);

    r = fixLine(vertXZs, vertCount, posFrom, posTo);
    printf("%d, %f, %f\n", r ? 1 : 0, posTo[0], posTo[2]);

#if defined(_MSC_VER) && defined(_DEBUG)
    printf("PRESS ENTER TO EXIT ... ");
    getchar();
#endif
    return 0;
}

class rcContextTest : public rcContext
{
    clock_t time[256];
    int total[256];
public:
    rcContextTest()
    {
        memset(time, 0, sizeof(time));
        memset(total, 0, sizeof(total));
    }
protected:
    virtual void doLog(const rcLogCategory category, const char* message, const int /*len*/)
    {
        if (category == RC_LOG_PROGRESS)
            printf(" INFO: %s\n", message);
        else if (category == RC_LOG_WARNING)
            printf(" WARN: %s\n", message);
        else if (category == RC_LOG_ERROR)
            printf("ERROR: %s\n", message);
        else
            printf("%5d: %s\n", category, message);
    }

    virtual void doStartTimer(const rcTimerLabel label)
    {
        time[label] = clock();
    }

    virtual void doStopTimer(const rcTimerLabel label)
    {
        total[label] += static_cast<int>((static_cast<long long>(clock()) - time[label]) * 1000 / CLOCKS_PER_SEC);
    }

    virtual int doGetAccumulatedTime(const rcTimerLabel label) const
    {
        return total[label];
    }
};

int main(int argc, const char** argv)
{
    navInit();
    FILE* fp = fopen("~$dump.navmeshes", "rb");
    if (!fp)
    {
        printf("ERR1");
        return 0;
    }
    fseek(fp, 0, SEEK_END);
    int size = (int)ftell(fp);
    fseek(fp, 0, SEEK_SET);
    unsigned char* buf = new unsigned char[size];
    fread(buf, 1, size, fp);
    fclose(fp);
    dtNavMeshParams params;
    memset(&params, 0, sizeof(params));
    params.tileWidth = 16;
    params.tileHeight = 16;
    params.maxTiles = 512 * 512;
    params.maxPolys = 0x7fff;
    dtNavMesh* navMesh;
    NavStatus r = navCreateNavMesh(&params, &navMesh);
    if (r != 0)
    {
        printf("ERR2 %lld", (long long)r);
        return 0;
    }
    for (int i = 0; i < size;)
    {
        int len = *(int*)(buf + i);
        navReplaceTile(navMesh, buf + i + 4, len);
        i += 4 + len;
    }
    dtNavMeshQueryEx* navQuery = 0;
    r = navAllocNavQuery(navMesh, 4096, &navQuery);
    if (r != 0)
    {
        printf("ERR3 %lld", (long long)r);
        return 0;
    }
    float sp[3] = { 4960 + 87, 300, 4480 + 20 };
    float tp[3] = { 4960 + 135, 300, 4480 + 20 };
    float poses[1024 * 3];
    r = navFindPath(navQuery, sp, tp, poses, 0, 1024, false);
    if (r < 0)
    {
        printf("ERR4 %lld", (long long)r);
        return 0;
    }
    for (int i = 0; i < r; i++)
        printf("%3d: %f, %f, %f,\n", i, poses[i*3], poses[i*3+1], poses[i*3+2]);
    printf("end");
    return 0;
}

int main2(int argc, const char** argv)
{
    const char* serverMapRootPath = "ark_resource/resource/develop/server";
    char serverMapRootDir[512];
    for (int i = 0;; i++)
    {
        strcpy(serverMapRootDir + i * 3, serverMapRootPath);
        if (!_access(serverMapRootDir, 0))
            break;
        if (i >= 10)
        {
            printf("ERROR: not found '%s' in parent path\n", serverMapRootPath);
            return -1;
        }
        serverMapRootDir[i * 3] = '.';
        serverMapRootDir[i * 3 + 1] = '.';
        serverMapRootDir[i * 3 + 2] = '/';
    }

    const int mapId = argc > 1 ? atoi(argv[argc - 1]) : 3010;
    char defPath[256];
    sprintf(defPath, "%s/map/map%d/navmesh/%d.rcnavmesh", serverMapRootDir, mapId, mapId);
    const char* const navFileName = argc > 1 ? argv[1] : defPath;
    NavStatus s;

    navInit();
    printMem();

    if (argc > 1 && !strcmp(argv[1], "buildall"))
    {
        printf("navBuildAllNavMesh(%d) running ...\n", mapId);
        rcContextTest rcCtx;
        DebugParam dp;
        dp.ctx = &rcCtx;
        dp.beginTileX = 0;
        dp.beginTileZ = 0;
        dp.endTileX = 99;
        dp.endTileZ = 99;
        dp.saveDebugFile = false;
        dp.saveHeightfield = false;
        dp.saveContourSet = false;
        dp.onlyBuildSomeTiles = true;
        s = navBuildAllNavMesh(serverMapRootDir, mapId, &dp);
        if (s)
        {
            printf("ERROR: navBuildAllNavMesh failed(%lld,0x%llX): %s, %d\n",
                static_cast<long long>(s), static_cast<long long>(s), serverMapRootDir, mapId);
            return -1;
        }
        printf("navBuildAllNavMesh OK! (%d ms)\n", rcCtx.getAccumulatedTime(RC_TIMER_TOTAL));
        printf("RC_TIMER_AUTO_OFFMESH_LINK = %d ms\n", rcCtx.getAccumulatedTime(static_cast<rcTimerLabel>(RC_TIMER_AUTO_OFFMESH_LINK)));
        printf("RC_TIMER_FIX_POLY_FLAGS    = %d ms\n", rcCtx.getAccumulatedTime(static_cast<rcTimerLabel>(RC_TIMER_FIX_POLY_FLAGS)));
        printMem();
        return 0;
    }

    resetTimer();
    FILE* fp = fopen(navFileName, "rb");
    if (!fp)
    {
        printf("ERROR: open file failed: %s\n", navFileName);
        return -1;
    }
    dtNavMeshEx* navMesh;
    s = navLoadNavMesh(fp, &navMesh);
    fclose(fp);
    if (s)
    {
        printf("ERROR: navLoadNavMesh failed(0x%llX): %s\n", static_cast<long long>(s), navFileName);
        return -1;
    }
    printf("navMesh  = %p (%d ms)\n", navMesh, getTime());
    printMem();

    // traceNavMesh(*static_cast<dtNavMesh*>(navMesh));

    resetTimer();
    dtNavMeshQueryEx* navQuery = 0;
    s = navAllocNavQuery(navMesh, 65535, &navQuery);
    if (s)
    {
        printf("ERROR: navAllocNavQuery failed(0x%llX): %s\n", static_cast<long long>(s), navFileName);
        return -1;
    }
    printf("navQuery = %p (%d ms)\n", navQuery, getTime());
    printMem();

    navSetFindPathFlags(navQuery, 7);
    for (int i = 0; i < 64; ++i)
        navSetFindPathWeight(navQuery, i, 1);

    resetTimer();
    const float pi = 3.14159265f;
    const float cx = 71.0f, cz = 41.5f;
    const float cr = 2.f;
    const float a0 = pi * 0.0f;
    const float a1 = pi * 0.5f;
    const float x0 = cx + cr * (float)cos(a0);
    const float z0 = cz + cr * (float)sin(a0);
    const float x1 = cx + cr * (float)cos(a1);
    const float z1 = cz + cr * (float)sin(a1);
    s = navCheckArcCollision(navQuery, cx, cz, cr, x0, z0, x1, z1);
    if (s < 0)
        printf("ERROR: navCheckArcCollision failed(0x%llX): %s\n", static_cast<long long>(s), navFileName);
    else
        printf("navCheckArcCollision = %lld (%d ms)\n", static_cast<long long>(s), getTime());
    printMem();

    resetTimer();
    const int floatBufCount = 4096;
    float* const floatBuf = static_cast<float*>(navAlloc(4 * 3 * floatBufCount));
    printf("floatBuf = %p (%d ms)\n", floatBuf, getTime());
    if (!floatBuf)
    {
        printf("ERROR: navAlloc failed\n");
        return -1;
    }
    printMem();

    resetTimer();
    const float sp[3] = { -216.99872,100.4,-376.29297  };
    const float tp[3] = { -211.35783,100.62807,-372.62265 };
    s = navFindPath(navQuery, sp, tp, floatBuf, 0, floatBufCount, false);
    if (s < 0)
        printf("ERROR: navFindPath failed(0x%llX): %s\n", static_cast<long long>(s), navFileName);
    else
    {
        printf("navFindPath = %lld (%d ms)\n", static_cast<long long>(s), getTime());
        for (int i = 0, n = static_cast<int>(s); i < n; ++i)
            printf("  %4d: %5.3f %5.3f %5.3f\n", i, floatBuf[i * 3], floatBuf[i * 3 + 1], floatBuf[i * 3 + 2]);
    }
    printMem();

    resetTimer();
    NavFieldCtx* ctx = 0;
    s = navAllocCircleCtx(-198.4f, -385.8f, 30, &ctx);
    if (s < 0)
        printf("ERROR: navAllocCircleCtx failed(0x%llX): %s\n", static_cast<long long>(s), navFileName);
    else
        printf("navAllocCircleCtx = %lld (%d ms)\n", static_cast<long long>(s), getTime());
    s = navFindPathInField(navQuery, sp, tp, floatBuf, floatBufCount, false, ctx);
    if (s < 0)
        printf("ERROR: navFindPath failed(0x%llX): %s\n", static_cast<long long>(s), navFileName);
    else
    {
        printf("navFindPath = %lld (%d ms)\n", static_cast<long long>(s), getTime());
        for (int i = 0, n = static_cast<int>(s); i < n; ++i)
            printf("  %4d: %5.3f %5.3f %5.3f\n", i, floatBuf[i * 3], floatBuf[i * 3 + 1], floatBuf[i * 3 + 2]);
    }
    if (ctx)
    {
        navFreeFieldCtx(ctx);
        ctx = 0;
    }
    printMem();

    resetTimer();
    s = navFindPathInRange(navQuery, sp, tp, floatBuf, floatBufCount, 211.4f, -263.2f, 30.0f * 30.0f);
    if (s < 0)
        printf("ERROR: navFindPathInRange failed(0x%llX): %s\n", static_cast<long long>(s), navFileName);
    else
    {
        printf("navFindPathInRange = %lld (%d ms)\n", static_cast<long long>(s), getTime());
        for (int i = 0, n = static_cast<int>(s); i < n; ++i)
            printf("  %4d: %5.3f %5.3f %5.3f\n", i, floatBuf[i * 3], floatBuf[i * 3 + 1], floatBuf[i * 3 + 2]);
    }
    printMem();

    resetTimer();
    s = navFindWater(navQuery, sp, 1000, floatBuf, floatBufCount);
    if (s < 0)
        printf("ERROR: navFindWater failed(0x%llX): %s\n", static_cast<long long>(s), navFileName);
    else
    {
        printf("navFindWater = %lld (%d ms)\n", static_cast<long long>(s), getTime());
        for (int i = 0, n = static_cast<int>(s); i < n; ++i)
            printf("  %4d: %5.3f %5.3f %5.3f\n", i, floatBuf[i * 3], floatBuf[i * 3 + 1], floatBuf[i * 3 + 2]);
    }
    printMem();

    resetTimer();
    memcpy(floatBuf, sp, sizeof(sp));
    // floatBuf[0] = -136.88683f;
    // floatBuf[1] = 47.2f;
    // floatBuf[2] = -371.21887f;
    floatBuf[0] = -139.58017f;
    floatBuf[1] = 63.0f;
    floatBuf[2] = 53.8633f;
    s = navFindPos(navQuery, floatBuf, 0);
    if (s != 0)
        printf("ERROR: navFindPos failed(0x%llX): %s\n", static_cast<long long>(s), navFileName);
    else
    {
        printf("navFindPos = %lld (%d ms)\n", static_cast<long long>(s), getTime());
        printf("  %5.3f %5.3f %5.3f\n", floatBuf[0], floatBuf[1], floatBuf[2]);
    }
    printMem();

    resetTimer();
    memcpy(floatBuf, sp, sizeof(sp));
    s = navRandomPos(navQuery, floatBuf, 30);
    if (s != 0)
        printf("ERROR: navRandomPos failed(0x%llX): %s\n", static_cast<long long>(s), navFileName);
    else
    {
        printf("navRandomPos = %lld (%d ms)\n", static_cast<long long>(s), getTime());
        printf("  %5.3f %5.3f %5.3f\n", floatBuf[0], floatBuf[1], floatBuf[2]);
    }
    printMem();

    resetTimer();
    dtNavMeshEx* forkNavMesh;
    s = navForkNavMesh(navMesh, &forkNavMesh);
    if (s)
        printf("ERROR: navForkNavMesh failed(0x%llX): %s\n", static_cast<long long>(s), navFileName);
    else
        printf("forkNavMesh = %p (%d ms)\n", forkNavMesh, getTime());
    printMem();

    resetTimer();
    char outFileName[1024];
    strcpy(outFileName, navFileName);
    strcat(outFileName, ".save");
    fp = fopen(outFileName, "wb");
    if (!fp)
        printf("ERROR: create file failed: %s\n", outFileName);
    else
        s = navSaveNavMesh(navMesh, fp);
    fclose(fp);
    if (s)
        printf("ERROR: navSaveNavMesh failed(0x%llX): %s\n", static_cast<long long>(s), outFileName);
    else
        printf("navSaveNavMesh OK (%d ms)\n", getTime());
    printMem();

    //TODO: navBuildTileBegin, navBuildTileLink, navBuildTileEnd, navReplaceTile, navRemoveTile
    //TODO: navSetFindPathFlags, navSetFindPathWeight, navFindPos, navFindWater, navRandomPos

    printf("free forkNavMesh\n");
    navFreeNavMesh(forkNavMesh);
    printMem();
    printf("free floatBuf\n");
    navFree(floatBuf);
    printMem();
    printf("free floatBuf\n");
    navFreeNavQuery(navQuery);
    printMem();
    printf("free floatBuf\n");
    navFreeNavMesh(navMesh);
    printMem();
    printf("end\n");

#if defined(_MSC_VER) && defined(_DEBUG)
    printf("PRESS ENTER TO EXIT ... ");
    getchar();
#endif
    return 0;
}
