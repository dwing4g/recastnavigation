#include <math.h>
#include <float.h>
#include <malloc.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <time.h>
#include "DetourMath.h" // fix 'isfinite' for linux gcc 4.8.5
#include <new>
#include <vector>
#include <deque>
#include <unordered_map>
#include <functional>
#include <algorithm>
#include "Recast.h"
#include "RecastAlloc.h"
#include "DetourStatus.h"
#include "DetourAlloc.h"
#include "DetourCommon.h"
#include "DetourNode.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourNavMeshBuilder.h"
#ifdef _MSC_VER
#pragma warning(push, 4)
#endif
#include "NavServer.internal.h"
#ifdef USE_JEMALLOC
// #include "jemalloc/jemalloc.h"
// extern "C" void* je_malloc(size_t size);
// extern "C" void je_free(void* p);
// #define malloc je_malloc
// #define free je_free
#elif USE_MIMALLOC
// #include "mimalloc/mimalloc.h"
extern "C" void* mi_malloc(size_t size);
extern "C" void mi_free(void* p);
#define malloc mi_malloc
#define free mi_free
#endif

#ifdef _MSC_VER
#include <atomic>
static std::atomic<int64_t> allocCount, freeCount, memSize;
inline static void addMemSize(int64_t delta)
{
    allocCount++;
    memSize += delta;
}
inline static void subMemSize(int64_t delta)
{
    freeCount++;
    memSize -= delta;
}
inline static int64_t getAllocCount()
{
    return allocCount;
}
inline static int64_t getFreeCount()
{
    return freeCount;
}
inline static int64_t getMemSize()
{
    return memSize;
}
#else
static volatile int64_t allocCount, freeCount, memSize;
inline static void addMemSize(int64_t delta)
{
    __sync_fetch_and_add(&allocCount, 1);
    __sync_fetch_and_add(&memSize, delta);
}
inline static void subMemSize(int64_t delta)
{
    __sync_fetch_and_add(&freeCount, 1);
    __sync_fetch_and_sub(&memSize, delta);
}
inline static int64_t getAllocCount()
{
    __sync_synchronize();
    return allocCount;
}
inline static int64_t getFreeCount()
{
    __sync_synchronize();
    return freeCount;
}
inline static int64_t getMemSize()
{
    __sync_synchronize();
    return memSize;
}
#endif

inline void* recastRcAlloc(size_t size, rcAllocHint)
{
    const size_t realSize = sizeof(size_t) + size;
    size_t* const p = static_cast<size_t*>(malloc(realSize));
    if (!p)
        return 0;
    addMemSize(static_cast<int64_t>(realSize));
    *p = realSize;
    return p + 1;
}

void* recastDtAlloc(size_t size, dtAllocHint)
{
    return recastRcAlloc(size, RC_ALLOC_PERM);
}

void recastFree(void* ptr)
{
    if (!ptr)
        return;
    size_t* const p = static_cast<size_t*>(ptr) - 1;
    subMemSize(static_cast<int64_t>(*p));
    free(p);
}

extern "C" bool navInit()
{
    rcAllocSetCustom(recastRcAlloc, recastFree);
    dtAllocSetCustom(recastDtAlloc, recastFree);
    srand(static_cast<unsigned>(clock()));
    return true;
}

extern "C" int64_t navGetAllocCount()
{
    return getAllocCount();
}

extern "C" int64_t navGetFreeCount()
{
    return getFreeCount();
}

extern "C" int64_t navGetMemSize()
{
    return getMemSize();
}

extern "C" void* navAlloc(size_t size)
{
    return dtAlloc(size, DT_ALLOC_PERM);
}

extern "C" void navFree(void* ptr)
{
    dtFree(ptr);
}

int fixNavMeshTileInvalidPoly(void* navTileData) // fix after addTile
{
    dtMeshHeader* const header = static_cast<dtMeshHeader*>(navTileData);
    const int vertCount = header->vertCount;
    const int polyCount = header->polyCount;
    float* const verts = reinterpret_cast<float*>(header + 1);
    dtPoly* const polys = reinterpret_cast<dtPoly*>(verts + vertCount * 3);
    float lastX = 0, lastZ = 0;
    int fixCount = 0;
    for (int i = 0; i < polyCount; i++)
    {
        dtPoly& poly = polys[i];
        if (!poly.flags)
            continue;
        const int vn = poly.vertCount;
        if (vn < 3)
            continue;
        bool okX = false, okZ = false;
        for (int j = 0; j < vn; j++)
        {
            const int vertIdx = static_cast<int>(poly.verts[j]) * 3;
            const float x = verts[vertIdx];
            const float z = verts[vertIdx + 2];
            if (j > 0)
            {
                if (x != lastX)
                    okX = true;
                if (z != lastZ)
                    okZ = true;
            }
            lastX = x;
            lastZ = z;
        }
        if (!okX || !okZ)
        {
            poly.flags = 0; // invalidate this poly
            fixCount++;
        }
    }
    return fixCount;
}

// s = abs{[(x0*y1-x1*y0)+(x1*y2-x2*y1)+......+(xn*y0-x0*yn)] / 2}
inline static float calcPolyAreaSize(const dtMeshTile& tile, const dtPoly& poly)
{
    const int n = poly.vertCount;
    if (n <= 2)
        return 0;
    const float* const verts = tile.verts;
    float area = 0;
    const float* vert = verts + poly.verts[n - 1] * 3;
    float xx = vert[0], zz = vert[2];
    for (int i = 0; i < n; i++)
    {
        vert = verts + poly.verts[i] * 3;
        const float x = vert[0], z = vert[2];
        area += xx * z - x * zz;
        xx = x; zz = z;
    }
    return fabs(area * 0.5f);
}

// 从poly开始广度优先遍历,同时标记MARK2并统计遍历的poly总面积,达到minUndergroundRegionArea时返回已统计的总面积
inline static float markPolys(const dtNavMeshEx& navMesh, const dtMeshTile& tile, dtPoly& poly, const float minUndergroundRegionArea)
{
    std::deque<dtPolyRef>& queue = navMesh.tempQueue;
    queue.clear();
    const dtMeshTile* curTile = &tile;
    dtPoly* curPoly = &poly;
    float markedArea = 0;
    goto inner;
    while (!queue.empty())
    {
        navMesh.getTileAndPolyByRefUnsafe(queue.front(), &curTile, const_cast<const dtPoly**>(&curPoly));
        queue.pop_front();
inner:  if (curPoly->flags & NAVMESH_POLYFLAGS_TEMP_MARK2)
            continue;
        curPoly->flags |= NAVMESH_POLYFLAGS_TEMP_MARK2;
        markedArea += calcPolyAreaSize(*curTile, *curPoly);
        if (markedArea >= minUndergroundRegionArea && !(curPoly->flags & NAVMESH_POLYFLAGS_TEMP_MARK1)) // 有MARK1的不限制搜索,提高性能
            return markedArea;
        for (unsigned int k = curPoly->firstLink; k != DT_NULL_LINK; k = curTile->links[k].next)
            queue.emplace_back(curTile->links[k].ref);
    }
    return markedArea;
}
/*
inline static bool checkCeiling(const dtMeshTile& tile, const dtPoly& poly, const rcHeightfield& hf)
{
    const float* const verts = tile.verts;
    const int vn = poly.vertCount;
    if (vn <= 2)
        return true;
    float x = 0, y = 0, z = 0; // 计算poly中心坐标(x,z)和最高值y
    for (int i = 0; i < vn; i++)
    {
        const int vertIdx = static_cast<int>(poly.verts[i]) * 3;
        x += verts[vertIdx];
        z += verts[vertIdx + 2];
        y = std::max(y, verts[vertIdx + 1]);
    }
    float f = 1.0f / vn;
    x *= f;
    z *= f;
    f = 1.0f / hf.cs;
    const int spanX = (int)((x - hf.bmin[0]) * f);
    const int spanZ = (int)((z - hf.bmin[2]) * f);
    if (static_cast<unsigned>(spanX) < static_cast<unsigned>(hf.width) &&
        static_cast<unsigned>(spanZ) < static_cast<unsigned>(hf.height))
    {
        const int spanY = (int)((y + 1.0f - hf.bmin[1]) / hf.ch); // +1.0确保更高的位置有障碍
        const rcSpan* s = hf.spans[hf.width * spanZ + spanX];
        for (; s; s = s->next)
            if (static_cast<int>(s->smax) > spanY)
                break;
        if (!s)
            return false;
    }
    return true;
}
*/
inline static bool checkCeiling(const dtPoly& poly) // 返回上方是否有遮挡(桥只返回true)
{
    return poly.vertCount <= 2 || (poly.getArea() & NAVMESH_POLYAREA_HAS_CEILING) != 0;
}

// 从poly开始广度优先遍历,清除MARK2(同时清除MARK1),hf用于判断并返回是否带MARK2的poly都有高层障碍
inline static bool cleanPolys(const dtNavMeshEx& navMesh, const dtMeshTile& tile, dtPoly& poly) // const rcHeightfield* hf
{
    std::deque<dtPolyRef>& queue = navMesh.tempQueue;
    queue.clear();
    const dtMeshTile* curTile = &tile;
    dtPoly* curPoly = &poly;
    bool hasCeiling = true;
    goto inner;
    while (!queue.empty())
    {
        navMesh.getTileAndPolyByRefUnsafe(queue.front(), &curTile, const_cast<const dtPoly**>(&curPoly));
        queue.pop_front();
inner:  if (!(curPoly->flags & NAVMESH_POLYFLAGS_TEMP_MARK2))
            continue;
        curPoly->flags &= ~(NAVMESH_POLYFLAGS_TEMP_MARK1 | NAVMESH_POLYFLAGS_TEMP_MARK2);
        if (hasCeiling && !checkCeiling(*curPoly)) // !checkCeiling(*curTile, *curPoly, *hf)
            hasCeiling = false;
        for (unsigned int k = curPoly->firstLink; k != DT_NULL_LINK; k = curTile->links[k].next)
            queue.emplace_back(curTile->links[k].ref);
    }
    return hasCeiling;
}

// 从poly开始广度优先遍历,修改所有遍历的poly.flags为0,返回修改的poly数量
inline static int fixPolys(const dtNavMeshEx& navMesh, const dtMeshTile& tile, dtPoly& poly)
{
    std::deque<dtPolyRef>& queue = navMesh.tempQueue;
    queue.clear();
    const dtMeshTile* curTile = &tile;
    dtPoly* curPoly = &poly;
    int fixCount = 0;
    goto inner;
    while (!queue.empty())
    {
        navMesh.getTileAndPolyByRefUnsafe(queue.front(), &curTile, const_cast<const dtPoly**>(&curPoly));
        queue.pop_front();
inner:  if (!curPoly->flags)
            continue;
        curPoly->flags = 0;
        fixCount++;
        for (unsigned int k = curPoly->firstLink; k != DT_NULL_LINK; k = curTile->links[k].next)
            queue.emplace_back(curTile->links[k].ref);
    }
    return fixCount;
}

// 修正某tile内所有无露天的孤岛polys(设置flags=0),必须在周围tiles都已经加到navmesh里才能正确修正,返回修正的poly数量
int fixNavMeshTileIsland(const dtNavMeshEx& navMesh, const BuildParam& bp, int tileX, int tileZ)
{
    const dtMeshTile* const tile = navMesh.getTileAt(tileX, tileZ, 0);
    if (!tile)
        return -1;
    if (!tile->header)
        return -2;
    const TileCtx* const tileCtx = navMesh.getTileCtx(tileX, tileZ);
    if (!tileCtx)
        return -3;
    const float minUndergroundRegionArea = bp.minUndergroundRegionArea;
    const int polyCount = tile->header->polyCount;
    dtPoly* const polys = tile->polys;
    int fixCount = 0;
    for (int i = 0; i < polyCount; i++)
        polys[i].flags |= NAVMESH_POLYFLAGS_TEMP_MARK1;
    for (int i = 0; i < polyCount; i++)
    {
        dtPoly& poly = polys[i];
        if (!(poly.flags & NAVMESH_POLYFLAGS_TEMP_MARK1))
            continue;
        const float markedArea = markPolys(navMesh, *tile, poly, minUndergroundRegionArea);
        if (markedArea >= minUndergroundRegionArea)
            cleanPolys(navMesh, *tile, poly);
        else if (cleanPolys(navMesh, *tile, poly))
            fixCount += fixPolys(navMesh, *tile, poly);
    }
    return fixCount;
}

extern "C" NavStatus navLoadNavMesh(FILE* fp, dtNavMeshEx** outNavMesh)
{
    if (!fp)
        return -1;
    if (!outNavMesh)
        return -2;
    int version;
    if (fread(&version, sizeof(version), 1, fp) != 1)
        return -3;
    if (version != RC_NAVMESH_FILE_MAGIC_VERSION)
        return -4;
    rcConfig cfg;
    if (fread(&cfg, 1, sizeof(cfg), fp) != sizeof(cfg)) // 92 bytes
        return -5;
    const int tileSize = cfg.tileSize;
    if (cfg.tileSize <= 0 || cfg.cs <= 0)
        return -6;

    void* const mem = dtAlloc(sizeof(dtNavMeshEx), DT_ALLOC_PERM);
    if (!mem)
        return -7;
    dtNavMeshEx* const navMesh = new(mem) dtNavMeshEx;
    const int tileW = (cfg.width  + tileSize - 1) / tileSize;
    const int tileH = (cfg.height + tileSize - 1) / tileSize;
    const int tileCount = tileW * tileH;
    if (tileCount > MAX_TILES)
    {
        navFreeNavMesh(navMesh);
        return -8;
    }
    dtNavMeshParams params;
    rcVcopy(params.orig, cfg.bmin);
    params.tileWidth = params.tileHeight = tileSize * cfg.cs;
    params.maxTiles = tileCount;
    params.maxPolys = 1 << (32 - 10 - dtIlog2(dtNextPow2(static_cast<unsigned int>(tileCount))));
    dtStatus s = navMesh->init(&params, cfg, tileW, tileH);
    if (dtStatusFailed(s))
    {
        navFreeNavMesh(navMesh);
        return NAVSTATUS_HIGH_BIT | s;
    }
    for (int i = 0; i < tileCount; ++i)
    {
        int dataSize;
        if (fread(&dataSize, sizeof(dataSize), 1, fp) != 1)
        {
            navFreeNavMesh(navMesh);
            return -9;
        }
        if (dataSize == 0)
            continue;
        if (dataSize < 0)
        {
            navFreeNavMesh(navMesh);
            return -10;
        }
        void* const data = dtAlloc(static_cast<size_t>(dataSize), DT_ALLOC_PERM);
        if (!data)
        {
            navFreeNavMesh(navMesh);
            return -11;
        }
        if (fread(data, 1, static_cast<size_t>(dataSize), fp) != static_cast<size_t>(dataSize))
        {
            dtFree(data);
            navFreeNavMesh(navMesh);
            return -12;
        }
        const dtMeshHeader* const header = static_cast<const dtMeshHeader*>(data);
        if (header->polyCount > params.maxPolys)
        {
            dtFree(data);
            navFreeNavMesh(navMesh);
            return -13;
        }
        s = navMesh->addTile(static_cast<unsigned char*>(data), dataSize, DT_TILE_FREE_DATA, 0, 0);
        if (dtStatusFailed(s))
        {
            dtFree(data);
            navFreeNavMesh(navMesh);
            return NAVSTATUS_HIGH_BIT | (static_cast<NavStatus>(i) + 1) << 32 | s;
        }
        fixNavMeshTileInvalidPoly(data);
        navMesh->initLink(header->x, header->y);
    }
    *outNavMesh = navMesh;
    return 0;
}

extern "C" NavStatus navForkNavMesh(const dtNavMeshEx* navMesh, dtNavMeshEx** outNavMesh)
{
    if (!navMesh)
        return -1;
    if (!outNavMesh)
        return -2;
    void* const mem = dtAlloc(sizeof(dtNavMeshEx), DT_ALLOC_PERM);
    if (!mem)
        return -3;
    dtNavMeshEx* const newNavMesh = new(mem) dtNavMeshEx;
    dtStatus s = newNavMesh->init(*navMesh);
    if (dtStatusFailed(s))
    {
        navFreeNavMesh(newNavMesh);
        return NAVSTATUS_HIGH_BIT | s;
    }
    for (int i = 0, n = navMesh->getMaxTiles(); i < n; ++i)
    {
        const dtMeshTile* const tile = navMesh->getTile(i);
        const int dataSize = tile->dataSize;
        if (dataSize > 0)
        {
            void* const data = dtAlloc(static_cast<size_t>(dataSize), DT_ALLOC_PERM);
            if (!data)
            {
                navFreeNavMesh(newNavMesh);
                return -4;
            }
            memcpy(data, tile->data, dataSize);
            s = newNavMesh->addTile(static_cast<unsigned char*>(data), dataSize, DT_TILE_FREE_DATA, 0, 0);
            if (dtStatusFailed(s))
            {
                dtFree(data);
                navFreeNavMesh(newNavMesh);
                return NAVSTATUS_HIGH_BIT | (static_cast<NavStatus>(i) + 1) << 32 | s;
            }
            const dtMeshHeader* const header = static_cast<const dtMeshHeader*>(data);
            newNavMesh->initLink(header->x, header->y);
        }
    }
    *outNavMesh = newNavMesh;
    return 0;
}

extern "C" void navFreeNavMesh(dtNavMeshEx* navMesh)
{
    if (!navMesh)
        return;
    navMesh->~dtNavMeshEx();
    dtFree(navMesh);
}

static void customMarkHeightfield(rcHeightfield& hf, const BuildParam& bp, const BHMapConfig* waterCfg, const BHMapConfig* roadCfg)
{
    const int w = hf.width;
    const int h = hf.height;
    for (int z = 0; z < h; z++)
    {
        rcSpan** const spans = hf.spans + z * w;
        for (int x = 0; x < w; x++)
        {
            for (rcSpan* s = spans[x]; s; s = s->next)
            {
                if (s->area == NAVMESH_POLYAREA_IMPASSABLE)
                {
                    s->area = RC_NULL_AREA;
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        const int newZ = z + dz;
                        if (static_cast<unsigned>(newZ) < static_cast<unsigned>(h))
                        {
                            rcSpan** const newSpans = hf.spans + newZ * w;
                            for (int dx = -1; dx <= 1; dx++)
                            {
                                const int newX = x + dx;
                                if (static_cast<unsigned>(newX) < static_cast<unsigned>(w))
                                    for (rcSpan* p = newSpans[newX]; p; p = p->next)
                                        if (p->area == RC_WALKABLE_AREA && s->smin - 10 < p->smax && p->smax < s->smax + 10)
                                            p->area = RC_NULL_AREA;
                            }
                        }
                    }
                }
            }
        }
    }
    for (int z = 0; z < h; z++)
    {
        rcSpan** const spans = hf.spans + z * w;
        const float worldZ = (z + 0.5f) * hf.cs + hf.bmin[2];
        for (int x = 0; x < w; x++)
        {
            rcSpan* s = spans[x];
            if (!s)
                continue;
            const float worldX = (x + 0.5f) * hf.cs + hf.bmin[0];
            const float waterHeight = waterCfg ? waterCfg->getValue(worldX, worldZ, -1) : -1;
            for (; s; s = s->next)
            {
                if (s->area == RC_NULL_AREA)
                    continue;
                if (roadCfg && s->area == NAVMESH_POLYAREA_TERRAIN && roadCfg->getValue(worldX, worldZ) >= bp.roadAreaMinWeight)
                    s->area = NAVMESH_POLYAREA_ROAD;
                if (waterHeight >= 0)
                {
                    const float worldY = s->smax * hf.ch + hf.bmin[1];
                    if (worldY < waterHeight - bp.waterAreaMaxHeight)
                        s->area = RC_NULL_AREA; // not walkable
                    else if (worldY < waterHeight)
                        s->area = NAVMESH_POLYAREA_WATER;
                }
                if (s->area == NAVMESH_POLYAREA_TERRAIN || s->area == RC_WALKABLE_AREA)
                    s->area = NAVMESH_POLYAREA_GROUND;
            }
        }
    }
    for (int z = 0; z < h; z++)
    {
        rcSpan** const spans = hf.spans + z * w;
        for (int x = 0; x < w; x++)
        {
            int yMax = -1;
            for (rcSpan* s = spans[x]; s; s = s->next)
            {
                if (yMax < static_cast<int>(s->smax))
                    yMax = static_cast<int>(s->smax);
            }
            for (rcSpan* s = spans[x]; s; s = s->next)
            {
                if (s->area != RC_NULL_AREA && static_cast<int>(s->smax) < yMax)
                    s->area |= NAVMESH_POLYAREA_HAS_CEILING;
            }
        }
    }
}

static NavStatus createNavMesh(const rcConfig& cfg, const BuildParam& bp, rcContext* rcCtx, const rcPolyMesh* pmesh, const rcPolyMeshDetail* dmesh,
    const std::vector<OffMeshLink>& offMeshLinks, int tileX, int tileZ, unsigned char** outNavTileData, int* outNavTileDataSize)
{
    if (!pmesh || !dmesh)
        return -99;
    const int tileSize = cfg.tileSize;
    const int tileW = (cfg.width  + tileSize - 1) / tileSize;
    const int tileH = (cfg.height + tileSize - 1) / tileSize;
    const int tileCount = tileW * tileH;
    if (tileCount > MAX_TILES)
        return -98;
    const int maxPolys = 1 << (32 - 10 - dtIlog2(dtNextPow2(static_cast<unsigned int>(tileCount))));
    if (pmesh->npolys > maxPolys)
    {
        if (rcCtx)
            rcCtx->log(RC_LOG_ERROR, "createNavMesh: too many npolys in pmesh: %d > %d @ tile(%d,%d)", pmesh->npolys, maxPolys, tileX, tileZ);
        return -97;
    }
//  if (pmesh->npolys + static_cast<int>(offMeshLinks.size()) > maxPolys)
//  {
//      if (rcCtx)
//          rcCtx->log(RC_LOG_ERROR, "createNavMesh: too many npolys + offMeshLinks in pmesh: %d + %d > %d @ tile(%d,%d)",
//              pmesh->npolys, count, maxPolys, tileX, tileZ);
//      return -96;
//  }

    dtNavMeshCreateParams params = {0};
    params.verts     = pmesh->verts;
    params.vertCount = pmesh->nverts;
    params.polys     = pmesh->polys;
    params.polyFlags = pmesh->flags;
    params.polyAreas = pmesh->areas;
    params.polyCount = pmesh->npolys;
    params.nvp       = pmesh->nvp;
    params.detailMeshes     = dmesh->meshes;
    params.detailVerts      = dmesh->verts;
    params.detailVertsCount = dmesh->nverts;
    params.detailTris       = dmesh->tris;
    params.detailTriCount   = dmesh->ntris;
    // params.userId = 0;
    params.tileX = tileX;
    params.tileY = tileZ;
    // params.tileLayer = 0;
    rcVcopy(params.bmin, pmesh->bmin);
    rcVcopy(params.bmax, pmesh->bmax);
    params.walkableHeight = bp.agentHeight;   // cfg.ch * cfg.walkableHeight;
    params.walkableRadius = bp.agentRadius;   // cfg.ch * cfg.walkableRadius;
    params.walkableClimb  = bp.agentMaxClimb; // cfg.ch * cfg.walkableClimb;
    params.cs = cfg.cs;
    params.ch = cfg.ch;
    params.buildBvTree = true;

    unsigned char* navData;
    int navDataSize;
    const int count = std::min(static_cast<int>(offMeshLinks.size()), maxPolys - pmesh->npolys);
    if (count > 0)
    {
        float* const offMeshConVerts = static_cast<float*>(dtAlloc(count * sizeof(float) * 6, DT_ALLOC_PERM));
        float* const offMeshConRad = static_cast<float*>(dtAlloc(count * sizeof(float), DT_ALLOC_PERM));
        unsigned char* const offMeshConDir = static_cast<unsigned char*>(dtAlloc(count * sizeof(unsigned char), DT_ALLOC_PERM));
        unsigned char* const offMeshConAreas = static_cast<unsigned char*>(dtAlloc(count * sizeof(unsigned char), DT_ALLOC_PERM));
        unsigned short* const offMeshConFlags = static_cast<unsigned short*>(dtAlloc(count * sizeof(unsigned short), DT_ALLOC_PERM));
        float jumpNearDistance2 = bp.jumpNearDistance * bp.jumpNearDistance;
        float jumpMediumDistance2 = bp.jumpMediumDistance * bp.jumpMediumDistance;
        const int64_t jstep = (static_cast<int64_t>(offMeshLinks.size()) << 32) / count;
        int64_t j = 0;
        for (int i = 0; i < count; i++, j += jstep)
        {
            const OffMeshLink& link = offMeshLinks[j >> 32];
            offMeshConVerts[i * 6    ] = link.sx;
            offMeshConVerts[i * 6 + 1] = link.sy;
            offMeshConVerts[i * 6 + 2] = link.sz;
            offMeshConVerts[i * 6 + 3] = link.tx;
            offMeshConVerts[i * 6 + 4] = link.ty;
            offMeshConVerts[i * 6 + 5] = link.tz;
            offMeshConRad[i] = bp.agentRadius;
            offMeshConDir[i] = 1;
            const float offsetX = link.tx - link.sx;
            const float offsetZ = link.tz - link.sz;
            const float deltaY = fabs(link.ty - link.sy);
            const float deltaXZ2 = offsetX * offsetX + offsetZ * offsetZ;
            if (deltaY <= bp.jumpNearDistance && deltaXZ2 <= jumpNearDistance2)
            {
                offMeshConAreas[i] = NAVMESH_POLYAREA_JUMP_NEAR;
                offMeshConFlags[i] = NAVMESH_POLYFLAGS_JUMP_NEAR;
            }
            else if (deltaY <= bp.jumpMediumDistance && deltaXZ2 <= jumpMediumDistance2)
            {
                offMeshConAreas[i] = NAVMESH_POLYAREA_JUMP_MEDIUM;
                offMeshConFlags[i] = NAVMESH_POLYFLAGS_JUMP_MEDIUM;
            }
            else
            {
                offMeshConAreas[i] = NAVMESH_POLYAREA_JUMP_FAR;
                offMeshConFlags[i] = NAVMESH_POLYFLAGS_JUMP_FAR;
            }
        }
        params.offMeshConVerts = offMeshConVerts;
        params.offMeshConRad   = offMeshConRad;
        params.offMeshConFlags = offMeshConFlags;
        params.offMeshConAreas = offMeshConAreas;
        params.offMeshConDir   = offMeshConDir;
        // params.offMeshConUserID = 0;
        params.offMeshConCount = count;
        const bool r = dtCreateNavMeshData(&params, &navData, &navDataSize);
        dtFree(offMeshConFlags);
        dtFree(offMeshConAreas);
        dtFree(offMeshConDir);
        dtFree(offMeshConRad);
        dtFree(offMeshConVerts);
        if (!r)
            return -95; // could not build Detour navmesh
    }
    else if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
        return -94; // could not build Detour navmesh

    *outNavTileData = navData;
    *outNavTileDataSize = navDataSize;
    return 0;
}

extern "C" NavStatus navBuildTileBegin(dtNavMeshEx* navMesh, const void* buildParam, rcContext* rcCtx,
    int tileX, int tileZ, const void* const* meshDataArray, const void* const* meshVertPatchArray, int arraySize,
    const void* waterData, const void* roadData, unsigned char** outNavTileData, int* outNavTileDataSize)
{
    if (!navMesh)
        return -1;
    if (!meshDataArray && arraySize > 0)
        return -2;
    if (arraySize < 0)
        return -3;
    if (!outNavTileData)
        return -4;
    if (!outNavTileDataSize)
        return -5;
    TileCtx* const tileCtx = navMesh->getTileCtx(tileX, tileZ);
    if (!tileCtx)
        return -6;
    tileCtx->freeMesh();

    // allocate array that can hold triangle flags
    // if you have multiple meshes you need to process, allocate an array which can hold the max number of triangles you need to process
    int maxTriCount = 0;
    for (int i = 0; i < arraySize; ++i)
    {
        const int* const meshData = reinterpret_cast<const int* const*>(meshDataArray)[i];
        const int vertCount = meshData[0];
        const int triCount = vertCount >= 0 ? meshData[1] : 12;
        if (maxTriCount < triCount)
            maxTriCount = triCount;
    }
#ifdef _MSC_VER
    unsigned char* const areas = static_cast<unsigned char*>(_alloca(static_cast<size_t>(maxTriCount)));
    if (!areas)
        return -7;
#else
    unsigned char areas[maxTriCount];
#endif

    const rcConfig& cfg = navMesh->getCfg();
    if (cfg.maxVertsPerPoly > DT_VERTS_PER_POLYGON)
        return -8;
    const int tileSize = cfg.tileSize + cfg.borderSize * 2;
    const float bmin[3] = { cfg.bmin[0] + (tileX * cfg.tileSize - cfg.borderSize) * cfg.cs, cfg.bmin[1],
                            cfg.bmin[2] + (tileZ * cfg.tileSize - cfg.borderSize) * cfg.cs };
    const float bmax[3] = { cfg.bmin[0] + ((tileX + 1) * cfg.tileSize + cfg.borderSize) * cfg.cs, cfg.bmax[1],
                            cfg.bmin[2] + ((tileZ + 1) * cfg.tileSize + cfg.borderSize) * cfg.cs };
    rcContext rcCtxDef(false);
    if (!rcCtx)
        rcCtx = &rcCtxDef;
    rcHeightfield* const hf = rcAllocHeightfield(); // allocate voxel heightfield where we rasterize our input data to
    if (!hf)
        return -9;
    tileCtx->heightField = hf;
    if (!rcCreateHeightfield(rcCtx, *hf, tileSize, tileSize, bmin, bmax, cfg.cs, cfg.ch))
        return -10; // could not create solid heightfield

    for (int i = 0; i < arraySize; ++i)
    {
        const int* const meshData = reinterpret_cast<const int* const*>(meshDataArray)[i];
        int vertCount = meshData[0];
        int triCount;
        const float* verts = (meshVertPatchArray ? reinterpret_cast<const float* const*>(meshVertPatchArray)[i] : 0);
        const float* const patch = verts;
        const int* tris;
        const int* meshDataEnd;
        if (vertCount >= 0)
        {
            triCount = meshData[1];
            if (!verts)
                verts = reinterpret_cast<const float*>(meshData + 2);
            tris = meshData + 2 + 3 * vertCount;
            meshDataEnd = tris + 3 * triCount;
        }
        else // for box
        {
            vertCount = 8;
            triCount = 12;
            if (!verts)
                verts = reinterpret_cast<const float*>(meshData + 1);
            tris = BOX_TRIS;
            meshDataEnd = meshData + 1 + 3 * vertCount;
        }
        // find triangles which are walkable based on their slope and rasterize them
        // if your input data is multiple meshes, you can transform them here
        memset(areas, 0, triCount * sizeof(unsigned char));
        rcMarkWalkableTriangles(rcCtx, cfg.walkableSlopeAngle, verts, vertCount, tris, triCount, areas);
        if (!patch) // static model
        {
            const int area = *meshDataEnd; // navMeshPolyArea
            if (area != 0)
            {
                for (int k = 0; k < triCount; k++)
                {
                    if (areas[k] != RC_NULL_AREA)
                        areas[k] = static_cast<unsigned char>(area);
                }
            }
        }
        if (!rcRasterizeTriangles(rcCtx, verts, vertCount, tris, areas, triCount, *hf, cfg.walkableClimb))
            return -11; // could not rasterize triangles
    }

    // once all geometry is rasterized, we do initial pass of filtering to remove unwanted overhangs caused by the conservative rasterization
    // as well as filter spans where the character cannot possibly stand
    rcFilterLowHangingWalkableObstacles(rcCtx, cfg.walkableClimb, *hf);
    rcFilterLedgeSpans(rcCtx, cfg.walkableHeight, cfg.walkableClimb, *hf);
    rcFilterWalkableLowHeightSpans(rcCtx, cfg.walkableHeight, *hf);

    const BuildParam bp(buildParam);
    if (waterData)
    {
        BHMapConfig waterCfg(static_cast<const char*>(waterData));
        if (roadData)
        {
            BHMapConfig roadCfg(static_cast<const char*>(roadData));
            customMarkHeightfield(*hf, bp, &waterCfg, &roadCfg);
        }
        else
            customMarkHeightfield(*hf, bp, &waterCfg, 0);
    }
    else
    {
        if (roadData)
        {
            BHMapConfig roadCfg(static_cast<const char*>(roadData));
            customMarkHeightfield(*hf, bp, 0, &roadCfg);
        }
        else
            customMarkHeightfield(*hf, bp, 0, 0);
    }

    // compact the heightfield so that it is faster to handle from now on
    // this will result more cache coherent data as well as the neighbours between walkable cells will be calculated
    new(tileCtx->compactHeightfield = static_cast<rcCompactHeightfield*>(rcAlloc(sizeof(rcCompactHeightfield), RC_ALLOC_PERM))) rcCompactHeightfield;
    rcCompactHeightfield& chf = *tileCtx->compactHeightfield;
    if (!rcBuildCompactHeightfield(rcCtx, cfg.walkableHeight, cfg.walkableClimb, *hf, chf))
        return -12; // could not build compact data
    if (!rcErodeWalkableArea(rcCtx, cfg.walkableRadius, chf)) // erode the walkable area by agent radius
        return -13; // could not erode

#if 0 // m_partitionType == SAMPLE_PARTITION_WATERSHED
    if (!rcBuildDistanceField(rcCtx, chf)) // prepare for region partitioning, by calculating distance field along the walkable surface
        return -14; // could not build distance field
    if (!rcBuildRegions(rcCtx, chf, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea)) // partition the walkable surface into simple regions without holes
        return -15; // could not build watershed regions
#elif 1 // m_partitionType == SAMPLE_PARTITION_MONOTONE
    // Partition the walkable surface into simple regions without holes.
    // Monotone partitioning does not need distancefield.
    if (!rcBuildRegionsMonotone(rcCtx, chf, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea))
        return -16;
#else // m_partitionType == SAMPLE_PARTITION_LAYERS
    // Partition the walkable surface into simple regions without holes.
    if (!rcBuildLayerRegions(rcCtx, chf, cfg.borderSize, cfg.minRegionArea))
        return -17;
#endif

    new(tileCtx->contourSet = static_cast<rcContourSet*>(rcAlloc(sizeof(rcContourSet), RC_ALLOC_PERM))) rcContourSet;
    rcContourSet& cset = *tileCtx->contourSet;
    if (!rcBuildContours(rcCtx, chf, cfg.maxSimplificationError, cfg.maxEdgeLen, cset))
        return -18; // could not create contours
    if (cset.nconts == 0)
    {
        *outNavTileData = 0;
        *outNavTileDataSize = 0;
        return 0;
    }

    rcPolyMesh* const pmesh = rcAllocPolyMesh(); // build polygon navmesh from the contours
    if (!pmesh)
        return -19;
    tileCtx->polyMesh = pmesh;

    if (!rcBuildPolyMesh(rcCtx, cset, cfg.maxVertsPerPoly, *pmesh))
        return -20; // could not triangulate contours
    if (pmesh->nverts >= 0xffff) // the vertex indices are ushorts, and cannot point to more than 0xffff vertices
    {
        rcCtx->log(RC_LOG_ERROR, "navBuildTileBegin: too many vertices per tile: %d >= 0xffff @ tile(%d,%d)", pmesh->nverts, tileX, tileZ);
        return -21;
    }
    for (int i = 0; i < pmesh->npolys; ++i)
    {
        const unsigned int area = pmesh->areas[i];
        switch (area & NAVMESH_POLYAREA_MASK) // update poly flags from areas
        {
        case NAVMESH_POLYAREA_GROUND:
        case NAVMESH_POLYAREA_TERRAIN:
            pmesh->flags[i] = NAVMESH_POLYFLAGS_GROUND;
            break;
        case NAVMESH_POLYAREA_WATER:
            pmesh->flags[i] = NAVMESH_POLYFLAGS_WATER;
            break;
        case NAVMESH_POLYAREA_ROAD:
            pmesh->flags[i] = NAVMESH_POLYFLAGS_ROAD;
            break;
        case NAVMESH_POLYAREA_PLANT:
            pmesh->flags[i] = NAVMESH_POLYFLAGS_PLANT;
            break;
        default:
            if (area == RC_WALKABLE_AREA)
                pmesh->flags[i] = NAVMESH_POLYFLAGS_GROUND;
            else
                rcCtx->log(RC_LOG_ERROR, "navBuildTileBegin: unknown poly area: %d @ tile(%d,%d)", area, tileX, tileZ);
        }
    }

    rcPolyMeshDetail* const dmesh = rcAllocPolyMeshDetail(); // build detail mesh
    if (!dmesh)
        return -22;
    tileCtx->detailMesh = dmesh;
    if (!rcBuildPolyMeshDetail(rcCtx, *pmesh, chf, cfg.detailSampleDist, cfg.detailSampleMaxError, *dmesh))
        return -23; // could build polymesh detail

    return createNavMesh(cfg, bp, rcCtx, tileCtx->polyMesh, tileCtx->detailMesh, tileCtx->offMeshLinks, tileX, tileZ, outNavTileData, outNavTileDataSize);
}

bool checkHeightfieldCollision(const rcHeightfield* const hf, float x, float ymin, float ymax, float z)
{
    if (!hf || hf->cs == 0)
        return true;

    const float* const orig = hf->bmin;
    const float ics = 1 / hf->cs;
    const int ix = static_cast<int>(floorf((x - orig[0]) * ics));
    const int iz = static_cast<int>(floorf((z - orig[2]) * ics));
    if (static_cast<unsigned>(ix) >= static_cast<unsigned>(hf->width) || static_cast<unsigned>(iz) >= static_cast<unsigned>(hf->height))
        return false;
    const float by = orig[1];
    const float ch = hf->ch;
    for (const rcSpan* s = hf->spans[iz * hf->width + ix]; s; s = s->next)
        if (ymin < by + s->smax * ch && ymax > by + s->smin * ch) // overlap
            return true;
    return false;
}

bool checkHeightfieldCollision(const dtNavMeshEx& navMesh, float x, float ymin, float ymax, float z)
{
    const TileCtx* const tileCtx = navMesh.getTileCtx(navMesh.getTileX(x), navMesh.getTileZ(z));
    return tileCtx && checkHeightfieldCollision(tileCtx->heightField, x, ymin, ymax, z);
}

bool dropDownBlocked(const dtNavMeshEx& navMesh, const rcConfig& cfg, const BuildParam& bp, const float* startPos, const float* endPos)
{
    float xzStep[3] = { endPos[0] - startPos[0], 0, endPos[2] - startPos[2] };
    const float linkXZLength = rcSqrt(xzStep[0] * xzStep[0] + xzStep[2] * xzStep[2]);
    if (linkXZLength == 0)
        return false;
    const float stepSize = cfg.cs * 0.5f;
    const float d = stepSize / linkXZLength;
    xzStep[0] *= d;
    xzStep[2] *= d;
    const float perpStep[3] = { xzStep[2], 0, -xzStep[0] }; // xzStep顺时针旋转90度
    const float verticalOffset = (cfg.walkableClimb + 1) * cfg.ch;
    const float radius = cfg.walkableRadius * cfg.cs;
    float scanHeight = bp.agentHeight;
    float centerScanner[3] = { startPos[0], startPos[1] + verticalOffset, startPos[2] };
    bool overEdgeFlag = false;
    for (float scannedLen = 0; scannedLen < linkXZLength;)
    {
        if (checkHeightfieldCollision(navMesh, centerScanner[0], centerScanner[1], centerScanner[1] + scanHeight, centerScanner[2]))
            return true;
        for (float perpScannedLen = stepSize;;)
        {
            float perpScanner[3];
            rcVmad(perpScanner, centerScanner, perpStep, perpScannedLen);
            if (checkHeightfieldCollision(navMesh, perpScanner[0], perpScanner[1], perpScanner[1] + scanHeight, perpScanner[2]))
                return true;
            rcVmad(perpScanner, centerScanner, perpStep, -perpScannedLen);
            if (checkHeightfieldCollision(navMesh, perpScanner[0], perpScanner[1], perpScanner[1] + scanHeight, perpScanner[2]))
                return true;
            if ((perpScannedLen += stepSize) > radius)
                break;
        }
        scannedLen += stepSize;
        rcVadd(centerScanner, centerScanner, xzStep);
        if (!overEdgeFlag && linkXZLength - scannedLen < radius)
        {
            overEdgeFlag = true;
            centerScanner[1] = endPos[1] + verticalOffset;
            scanHeight += startPos[1] - endPos[1];
        }
    }
    return false;
}

void findValidDropDownForStartPosAndDir(const dtNavMeshEx& navMesh, const dtNavMeshQuery& navQuery, const dtQueryFilter& filter,
    const rcConfig& cfg, const BuildParam& bp, const float* startPos, const float* normal)
{
    const float minDistance = bp.agentRadius * 2 + cfg.cs * 4;
    const float maxDistance = std::max({minDistance, bp.jumpNearDistance, bp.jumpMediumDistance, bp.jumpFarDistance});
    const float ledgeDropHeight = bp.jumpFarDistance; // old version: 4.1f
    const float ledgeDropFactor = 0.8f; // horizon / vertical
    const float minDropHeight = cfg.walkableClimb * cfg.ch;
    const float extents[3] = { 0, ledgeDropHeight * 0.5f, 0 };
    float bestEndPos[3] = { FLT_MAX };
    for (float dropDownOffset = minDistance;;)
    {
        float endPos[3], samplePos[3];
        rcVmad(endPos, startPos, normal, dropDownOffset);
        rcVcopy(samplePos, endPos);
        const float center[3] = { endPos[0], endPos[1] - ledgeDropHeight * 0.5f, endPos[2] };
        int polyCount = 0;
        dtPolyRef polys[128];
        navQuery.queryPolygons(center, extents, &filter, polys, &polyCount, 128);
        float nearestDist = FLT_MAX;
        for (int i = 0; i < polyCount; i++)
        {
            float height;
            if (dtStatusSucceed(navQuery.getPolyHeight(polys[i], samplePos, &height)))
            {
                const float distance = endPos[1] - height;
                if (distance < nearestDist && distance > 0)
                    nearestDist = distance;
            }
        }
        if (nearestDist <= ledgeDropHeight && nearestDist > minDropHeight)
        {
            const bool isBest = nearestDist <= dropDownOffset * ledgeDropFactor; // old version: isBest = true;
            if (isBest || bestEndPos[0] == FLT_MAX)
            {
                endPos[1] -= nearestDist;
                if (dropDownBlocked(navMesh, cfg, bp, startPos, endPos)) // fast break if blocked (from unity)
                    break;
                rcVcopy(bestEndPos, endPos);
                if (isBest)
                    break;
            }
        }
        if ((dropDownOffset += bp.agentRadius) > maxDistance)
            break;
    }
    if (bestEndPos[0] != FLT_MAX)
    {
        navMesh.tempLinks.emplace_back(startPos, bestEndPos, navMesh.getTileX(startPos[0]), navMesh.getTileZ(startPos[2]),
            navMesh.getTileX(bestEndPos[0]), navMesh.getTileZ(bestEndPos[2]));
    }
}

void findValidDropDownsForMeshEdgeSegment(const dtNavMeshEx& navMesh, const dtNavMeshQuery& navQuery, const dtQueryFilter& filter,
    const rcConfig& cfg, const BuildParam& bp, const float* segStart, const float* segEnd)
{
    float step[3], segMid[3];
    rcVsub(step, segEnd, segStart);
    if (step[0] == 0 && step[2] == 0)
        return;
    const float segLen = rcSqrt(rcVdot(step, step));
    const float segHalfLen = segLen * 0.5f;
    float normal[3] = { -step[2], 0, step[0] }; // normal逆时针旋转90度
    rcVnormalize(normal);
    const int stepSize = 2;
    const float d = stepSize / segLen;
    step[0] *= d;
    step[1] *= d;
    step[2] *= d;
    rcVadd(segMid, segStart, segEnd);
    segMid[0] *= 0.5f;
    segMid[1] *= 0.5f;
    segMid[2] *= 0.5f;

    findValidDropDownForStartPosAndDir(navMesh, navQuery, filter, cfg, bp, segMid, normal);
    for (int i = 1; stepSize * i < segHalfLen; i++)
    {
        float pos[3];
        rcVmad(pos, segMid, step, static_cast<float>(i));
        findValidDropDownForStartPosAndDir(navMesh, navQuery, filter, cfg, bp, pos, normal);
        rcVmad(pos, segMid, step, static_cast<float>(-i));
        findValidDropDownForStartPosAndDir(navMesh, navQuery, filter, cfg, bp, pos, normal);
    }
}

extern "C" NavStatus navBuildTileLink(dtNavMeshEx* navMesh, const dtNavMeshQueryEx* navQuery, const void* buildParam,
    rcContext* rcCtx, int tileX, int tileZ, unsigned char** outNavTileData, int* outNavTileDataSize)
{
    if (!navMesh)
        return -1;
    if (!navQuery)
        return -2;
    if (!outNavTileData)
        return -3;
    if (!outNavTileDataSize)
        return -4;

    TileCtx* const tileCtx = navMesh->getTileCtx(tileX, tileZ);
    if (!tileCtx)
        return -5;
    const dtMeshTile* const tile = navMesh->getTileAt(tileX, tileZ, 0);
    if (!tile || !tile->header)
    {
        *outNavTileData = 0;
        *outNavTileDataSize = 0;
        return 0;
    }
    const rcConfig& cfg = navMesh->getCfg();
    const BuildParam bp(buildParam);
    const dtPolyRef baseRef = navMesh->getPolyRefBase(tile);
    const int kNavMeshVertsPerPoly = 6;
    const int kMaxSegsPerPoly = kNavMeshVertsPerPoly * 3;
    float segs[kMaxSegsPerPoly * 3 * 2];
    navMesh->tempLinks.clear();
    for (int i = 0, n = tile->header->polyCount; i < n; ++i)
    {
        if (!tile->polys[i].flags)
            continue;
        int nsegs = 0;
        navQuery->getPolyWallSegments(baseRef | static_cast<dtPolyRef>(i), navQuery, segs, 0, &nsegs, kMaxSegsPerPoly);
        for (int k = 0; k < nsegs; k++)
            findValidDropDownsForMeshEdgeSegment(*navMesh, *navQuery, *navQuery, cfg, bp, segs + k * 6, segs + k * 6 + 3);
    }

    tileCtx->offMeshLinks.clear();
    for (auto it = navMesh->tempLinks.cbegin(), ie = navMesh->tempLinks.cend(); it != ie; ++it)
    {
        const TileCtx* const sTileCtx = navMesh->getTileCtx(it->stx, it->stz);
        const TileCtx* const tTileCtx = navMesh->getTileCtx(it->ttx, it->ttz);
        if (sTileCtx && tTileCtx && sTileCtx->heightField && tTileCtx->heightField)
            tileCtx->offMeshLinks.emplace_back(*it);
    }
    for (auto it = tileCtx->neighbourTileOffMeshLinks.cbegin(), ie = tileCtx->neighbourTileOffMeshLinks.cend(); it != ie; ++it)
    {
        const TileCtx* const sTileCtx = navMesh->getTileCtx(it->stx, it->stz);
        const TileCtx* const tTileCtx = navMesh->getTileCtx(it->ttx, it->ttz);
        // 注意此处neighbourTileOffMeshLinkDatas与offMeshLinkDatas对于heightField是否为空的判断正好相反
        // 此处的意图是，如果sourceTile和targetTile都有体素数据，那么会计算得到一个桥在offMeshLinkDatas里面，所以不需要旧的sourceTile->targetTile的桥数据
        // 如果sourceTile和targetTile其中一个体素数据为空，说明，这两个Tile之间的桥没有发生变化，所以可以并且需要使用neighbourTileOffMeshLinkDatas中存的旧有的桥的数据
        // 当sourceTile和targetTile其中一个tile没有体素数据的时候，是无法计算sourceTile和targetTile之间的桥的
        if (sTileCtx && tTileCtx && sTileCtx != tTileCtx && (!sTileCtx->heightField || !tTileCtx->heightField))
            tileCtx->offMeshLinks.emplace_back(*it);
    }
    tileCtx->neighbourTileOffMeshLinks.clear();
    if (tileCtx->offMeshLinks.empty())
    {
        *outNavTileData = 0;
        *outNavTileDataSize = 0;
        return 0;
    }
    for (auto it = tileCtx->offMeshLinks.cbegin(), ie = tileCtx->offMeshLinks.cend(); it != ie; ++it)
    {
        if (it->stx != it->ttx || it->stz != it->ttz)
            tileCtx->neighbourTileOffMeshLinks.emplace_back(*it);
    }
    return createNavMesh(cfg, bp, rcCtx, tileCtx->polyMesh, tileCtx->detailMesh, tileCtx->offMeshLinks, tileX, tileZ, outNavTileData, outNavTileDataSize);
}

extern "C" void navBuildTileEnd(dtNavMeshEx* navMesh, int tileX, int tileZ)
{
    if (!navMesh)
        return;
    TileCtx* const tileCtx = navMesh->getTileCtx(tileX, tileZ);
    if (!tileCtx)
        return;
    tileCtx->freeMesh();
}

extern "C" NavStatus navReplaceTile(dtNavMesh* navMesh, unsigned char* navTileData, int navTileDataSize)
{
    if (!navMesh)
        return -1;
    if (!navTileData)
        return -2;
    if (navTileDataSize < static_cast<int>(sizeof(dtMeshHeader))) // 100 bytes
        return -3;
    const dtMeshHeader* const header = reinterpret_cast<const dtMeshHeader*>(navTileData);
    const dtTileRef tileRef = navMesh->getTileRefAt(header->x, header->y, 0);
    dtStatus s;
    if (tileRef)
    {
        s = navMesh->removeTile(tileRef, 0, 0);
        if (dtStatusFailed(s))
            return NAVSTATUS_HIGH_BIT | s;
    }
    s = navMesh->addTile(navTileData, navTileDataSize, DT_TILE_FREE_DATA, 0, 0);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | 1LL << 32 | s;
    return 0;
}

extern "C" NavStatus navRemoveTile(dtNavMesh* navMesh, int tileX, int tileZ)
{
    if (!navMesh)
        return -1;
    const dtTileRef tileRef = navMesh->getTileRefAt(tileX, tileZ, 0);
    if (!tileRef)
        return 1;
    dtStatus s = navMesh->removeTile(tileRef, 0, 0);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | s;
    return 0;
}

extern "C" NavStatus navFixTile(const dtNavMeshEx* navMesh, const void* buildParam, int tileX, int tileZ)
{
    if (!navMesh)
        return -1;
    const dtMeshTile* const tile = navMesh->getTileAt(tileX, tileZ, 0);
    if (!tile)
        return -2;
    const BuildParam bp(buildParam);
    const int r = fixNavMeshTileIsland(*navMesh, bp, tileX, tileZ);
    if (r < 0)
        return r - 50;
    return r + fixNavMeshTileInvalidPoly(tile->data);
}

extern "C" NavStatus navGetTileData(const dtNavMesh* navMesh, int tileX, int tileZ, const dtMeshTile** tilePtr)
{
    if (!tilePtr)
        return -1;
    if (!navMesh)
    {
        *tilePtr = 0;
        return -2;
    }
    const dtMeshTile* tile = navMesh->getTileAt(tileX, tileZ, 0);
    *tilePtr = tile;
    return tile ? 0 : -3;
}

extern "C" NavStatus navAllocNavQuery(const dtNavMesh* navMesh, int maxNodes, dtNavMeshQueryEx** outNavQuery)
{
    if (!navMesh)
        return -1;
    if (!outNavQuery)
        return -2;
    dtNavMeshQueryEx* navQuery = *outNavQuery;
    if (!navQuery)
    {
        void* const mem = dtAlloc(sizeof(dtNavMeshQueryEx), DT_ALLOC_PERM);
        if (!mem)
            return -3;
        navQuery = new(mem) dtNavMeshQueryEx(maxNodes);
    }
    const dtStatus s = navQuery->init(navMesh, maxNodes);
    if (dtStatusFailed(s))
    {
        if (!*outNavQuery)
            navFreeNavQuery(navQuery);
        return NAVSTATUS_HIGH_BIT | s;
    }
    *outNavQuery = navQuery;
    return 0;
}

extern "C" void navFreeNavQuery(dtNavMeshQueryEx* navQuery)
{
    if (!navQuery)
        return;
    navQuery->~dtNavMeshQueryEx();
    dtFree(navQuery);
}

extern "C" void navSetHalfExtents(dtNavMeshQueryEx* navQuery, float x, float y, float z)
{
    if (navQuery)
        navQuery->setHalfExtents(x, y, z);
}

extern "C" void navSetFindPathFlags(dtNavMeshQueryEx* navQuery, int areaFlags)
{
    if (navQuery)
        navQuery->setIncludeFlags(static_cast<unsigned short>(areaFlags));
}

extern "C" void navSetFindPathWeight(dtNavMeshQueryEx* navQuery, int areaIdx, float weight)
{
    if (navQuery && static_cast<unsigned>(areaIdx) < static_cast<unsigned>(DT_MAX_AREAS))
    {
        navQuery->setAreaCost(areaIdx, weight);
        navQuery->setAreaCost(areaIdx ^ NAVMESH_POLYAREA_HAS_CEILING, weight);
    }
}

// from dtFindNearestPolyQuery
class NavServerFindNearestPolyQuery : public dtPolyQuery
{
    const dtNavMeshQuery* m_query;
    const float* m_center;
    float m_nearestDistanceSqr;
    dtPolyRef m_nearestRef;
    float m_nearestPoint[3];

public:
    NavServerFindNearestPolyQuery(const dtNavMeshQuery* query, const float* center)
        : m_query(query), m_center(center), m_nearestDistanceSqr(FLT_MAX), m_nearestRef(0), m_nearestPoint() {}
    dtPolyRef nearestRef() const { return m_nearestRef; }
    const float* nearestPoint() const { return m_nearestPoint; }

    void process(const dtMeshTile* /*tile*/, dtPoly** /*polys*/, dtPolyRef* refs, int count)
    {
        for (int i = 0; i < count; ++i)
        {
            const dtPolyRef ref = refs[i];
            float closestPtPoly[3];
            // bool posOverPoly = false;
            if (dtStatusFailed(m_query->closestPointOnPoly(ref, m_center, closestPtPoly, 0 /*&posOverPoly*/)))
                continue;
            // float diff[3];
            // dtVsub(diff, m_center, closestPtPoly);
            const float d = dtVdistSqr(m_center, closestPtPoly);
            // if (posOverPoly || (d = diff[0]*diff[0] + diff[2]*diff[2]) < 0.0001f) // 有时因计算误差导致在poly的边缘仍然posOverPoly=false
            // {
            //     d = dtAbs(diff[1]) - tile->header->walkableClimb;
            //     d = d > 0 ? d * d : 0;
            // }
            // else
            //     d += diff[1]*diff[1] + 1000000; // 增加垂直范围不在navmesh上的代价,避免跨层的可能性
            if (d < m_nearestDistanceSqr)
            {
                dtVcopy(m_nearestPoint, closestPtPoly);
                m_nearestDistanceSqr = d;
                m_nearestRef = ref;
            }
        }
    }
};
class NavServerFindFloorNearestPolyQuery : public dtPolyQuery
{
    const dtNavMeshQuery* m_query;
    const float* m_center;
    double m_nearestDistanceSqr;
    dtPolyRef m_nearestRef;
    float m_nearestPoint[3];

public:
    NavServerFindFloorNearestPolyQuery(const dtNavMeshQuery* query, const float* center)
        : m_query(query), m_center(center), m_nearestDistanceSqr(DBL_MAX), m_nearestRef(0), m_nearestPoint() {
    }
    dtPolyRef nearestRef() const { return m_nearestRef; }
    const float* nearestPoint() const { return m_nearestPoint; }

    void process(const dtMeshTile* /*tile*/, dtPoly** /*polys*/, dtPolyRef* refs, int count)
    {
        for (int i = 0; i < count; ++i)
        {
            const dtPolyRef ref = refs[i];
            float closestPtPoly[3];
            if (dtStatusFailed(m_query->closestPointOnPoly(ref, m_center, closestPtPoly, 0)))
                continue;
            float diff[3];
            dtVsub(diff, m_center, closestPtPoly);
            const double dxz2 = (double)diff[0] * diff[0] + (double)diff[2] * diff[2];
            double d2 = dxz2 + (double)diff[1] * diff[1];
            if (dxz2 > 0.0001 || diff[1] < -0.01)
                d2 += 1000000; // 增加非向下垂直不在navmesh上的代价
            if (d2 < m_nearestDistanceSqr)
            {
                dtVcopy(m_nearestPoint, closestPtPoly);
                m_nearestDistanceSqr = d2;
                m_nearestRef = ref;
            }
        }
    }
};

dtStatus dtNavMeshQueryEx::findNearestPoly(const float* center, const float* pHalfExtents,
    const dtQueryFilter* filter, dtPolyRef* nearestRef, float* nearestPt, int method) const
{
    if (method == 1)
    {
        NavServerFindFloorNearestPolyQuery query(this, center);
        const dtStatus status = queryPolygons(center, pHalfExtents, filter, &query);
        if (dtStatusFailed(status))
            return status;
        const dtPolyRef bestRef = query.nearestRef();
        if (nearestRef)
            *nearestRef = bestRef;
        if (nearestPt && bestRef)
            dtVcopy(nearestPt, query.nearestPoint());
        return DT_SUCCESS;
    }

    NavServerFindNearestPolyQuery query(this, center);
    const dtStatus status = queryPolygons(center, pHalfExtents, filter, &query);
    if (dtStatusFailed(status))
        return status;
    const dtPolyRef bestRef = query.nearestRef();
    if (nearestRef)
        *nearestRef = bestRef;
    if (nearestPt && bestRef)
        dtVcopy(nearestPt, query.nearestPoint());
    return DT_SUCCESS;
}

extern "C" NavStatus navFindPath(const dtNavMeshQueryEx* navQuery, const float* sp, const float* tp,
    float* outFloatBuf, unsigned char* outAreaBuf, int maxPosCount, bool onlyRay)
{
    if (!navQuery)
        return -1;
    if (!sp)
        return -2;
    if (!tp)
        return -3;
    if (!onlyRay)
    {
        if (!outFloatBuf)
            return -4;
        if (maxPosCount <= 0)
            return -5;
    }
    else if (!outFloatBuf && maxPosCount > 0)
        return -6;
    dtPolyRef sRef;
    float np[3];
    rcVcopy(np, sp);
    dtStatus s = navQuery->findNearestPoly(sp, navQuery->getHalfExtents(), navQuery, &sRef, np);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | s;
    if (!sRef)
        return -7; // invalid source pos
    dtPolyRef polys[FIND_PATH_MAX_POLYS];
    dtRaycastHit rh;
    rh.path = polys;
    rh.maxPath = FIND_PATH_MAX_POLYS;
    s = navQuery->raycast(sRef, np, tp, navQuery, 0, &rh);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | 1LL << 32 | s;
    // if (rh.pathCount <= 0)
    //     return -8;
    float p[3];
    if (onlyRay)
    {
        if (rh.t < 1) // 有碰撞
        {
            dtVlerp(p, np, tp, rh.t);
            s = navQuery->closestPointOnPolyBoundary(rh.pathCount > 0 ? polys[rh.pathCount - 1] : sRef, p, p);
            if (dtStatusFailed(s))
                return NAVSTATUS_HIGH_BIT | 2LL << 32 | s;
            if (maxPosCount == 1 || (maxPosCount > 1 && rh.pathCount <= 0)) // 有碰撞但只需终点
            {
                dtVcopy(outFloatBuf, p);
                return 1;
            }
            if (maxPosCount <= 0) // 直接返回有碰撞
                return 1;
        }
        else if (maxPosCount == 1 || (maxPosCount > 1 && rh.pathCount <= 0)) // 没碰撞但只需终点
        {
            s = navQuery->closestPointOnPoly(rh.pathCount > 0 ? polys[rh.pathCount - 1] : sRef, tp, outFloatBuf, 0);
            if (dtStatusFailed(s))
                return NAVSTATUS_HIGH_BIT | 3LL << 32 | s;
            return 1;
        }
        else if (maxPosCount <= 0) // 直接返回没碰撞
            return 0;
        else
            dtVcopy(p, tp); // 没碰撞但需要所有位置
    }
    else
    {
        if (rh.t < 1) // 有碰撞
        {
            dtPolyRef tRef;
            s = navQuery->findNearestPoly(tp, navQuery->getHalfExtents(), navQuery, &tRef, 0);
            if (dtStatusFailed(s))
                return NAVSTATUS_HIGH_BIT | 4LL << 32 | s;
            if (!tRef)
                return -9; // invalid target pos
            s = navQuery->findPath(sRef, tRef, np, tp, navQuery, polys, &rh.pathCount, FIND_PATH_MAX_POLYS);
            if (dtStatusFailed(s))
                return NAVSTATUS_HIGH_BIT | 5LL << 32 | s;
        }
        dtVcopy(p, tp);
    }
#ifdef _MSC_VER
    dtPolyRef* const resPolys = static_cast<dtPolyRef*>(_alloca(sizeof(dtPolyRef) * maxPosCount));
    if (!resPolys)
        return -10;
#else
    dtPolyRef resPolys[maxPosCount];
#endif
    if (!dtVisfinite(np))
        return -11;
    if (!dtVisfinite(p))
        return -12;
    if (rh.pathCount <= 0)
        return -13;
    if (!polys[0])
        return -14;
    int posCount = 0;
    s = navQuery->findStraightPath(np, p, polys, rh.pathCount, outFloatBuf, 0, resPolys, &posCount, maxPosCount, DT_STRAIGHTPATH_AREA_CROSSINGS);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | 6LL << 32 | s;
    const dtNavMesh& navMesh = *navQuery->getAttachedNavMesh();
    const dtMeshTile* tile;
    const dtPoly* poly = 0;
    for (int i = 0; i < posCount; ++i)
    {
        float* const buf = outFloatBuf + 3 * i;
        dtPolyRef ref = resPolys[i];
        if (!ref && i + 1 == posCount)
            ref = polys[rh.pathCount - 1];
        if (ref)
            navQuery->closestPointOnPoly(ref, buf, buf, 0);
        else
            navQuery->findNearestPoly(buf, navQuery->getHalfExtents(), navQuery, &ref, buf);
        if (outAreaBuf)
        {
            if (ref)
                navMesh.getTileAndPolyByRefUnsafe(ref, &tile, &poly);
            outAreaBuf[i] = (poly ? poly->getArea() : 0);
        }
    }
    return posCount;
}

inline static dtStatus getPathToNode(dtNodePool& nodePool, const dtNode* endNode, dtPolyRef* path, int* pathCount)
{
    // Find the length of the entire path.
    const dtNode* curNode = endNode;
    int length = 0;
    do
    {
        length++;
        curNode = nodePool.getNodeAtIdx(curNode->pidx);
    } while (curNode);

    // If the path cannot be fully stored then advance to the last node we will be able to store.
    curNode = endNode;
    int writeCount;
    for (writeCount = length; writeCount > FIND_PATH_MAX_POLYS; writeCount--)
        curNode = nodePool.getNodeAtIdx(curNode->pidx);

    for (int i = writeCount - 1; i >= 0; i--)
    {
        path[i] = curNode->id; // Write path
        curNode = nodePool.getNodeAtIdx(curNode->pidx);
    }

    *pathCount = dtMin(length, FIND_PATH_MAX_POLYS);
    return length <= FIND_PATH_MAX_POLYS ? DT_SUCCESS : DT_SUCCESS | DT_BUFFER_TOO_SMALL;
}

inline static bool checkDist(const float* p, float cx, float cz, float dist2)
{
    const float dx = p[0] - cx;
    const float dz = p[2] - cz;
    return dx * dx + dz * dz <= dist2;
}

inline static dtStatus findPath(const dtNavMeshQueryEx& navQuery, dtPolyRef startRef, dtPolyRef endRef,
    const float* startPos, const float* endPos, dtPolyRef* path, int* pathCount,
    std::function<bool(dtPolyRef, dtPolyRef, const float*, const float*)> canPass)
{
    if (!pathCount)
        return DT_FAILURE | DT_INVALID_PARAM | (201u << 16);
    *pathCount = 0;

    const dtNavMesh* navMesh = navQuery.getAttachedNavMesh();
    dtNodePool* const nodePool = navQuery.getNodePool();
    dtNodeQueue& openList = navQuery.getOpenList();
    if (!navMesh || !nodePool || !navMesh->isValidPolyRef(startRef) || !navMesh->isValidPolyRef(endRef) ||
        !startPos || !dtVisfinite(startPos) || !endPos || !dtVisfinite(endPos) || !path)
        return DT_FAILURE | DT_INVALID_PARAM | (202u << 16);

    if (startRef == endRef)
    {
        path[0] = startRef;
        *pathCount = 1;
        return DT_SUCCESS;
    }

    nodePool->clear();
    openList.clear();

    static const float H_SCALE = 0.999f; // search heuristic scale
    dtNode* const startNode = nodePool->getNode(startRef);
    dtVcopy(startNode->pos, startPos);
    startNode->pidx = 0;
    startNode->cost = 0;
    startNode->total = dtVdist(startPos, endPos) * H_SCALE;
    startNode->id = startRef;
    startNode->flags = DT_NODE_OPEN;
    openList.push(startNode);

    dtNode* lastBestNode = startNode;
    float lastBestNodeCost = startNode->total;
    bool outOfNodes = false;

    while (!openList.empty())
    {
        dtNode* const bestNode = openList.pop(); // remove node from open list and put it in closed list
        bestNode->flags &= ~DT_NODE_OPEN;
        bestNode->flags |= DT_NODE_CLOSED;
        if (bestNode->id == endRef) // reached the goal, stop searching
        {
            lastBestNode = bestNode;
            break;
        }

        const dtPolyRef parentRef = bestNode->pidx ? nodePool->getNodeAtIdx(bestNode->pidx)->id : 0;
        const dtPolyRef bestRef = bestNode->id;
        const dtMeshTile* bestTile = 0;
        const dtPoly* bestPoly = 0;
        navMesh->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);

        for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
        {
            const dtPolyRef neighbourRef = bestTile->links[i].ref;
            if (!neighbourRef || neighbourRef == parentRef)
                continue; // skip invalid ids and do not expand back to where we came from

            const dtMeshTile* neighbourTile;
            const dtPoly* neighbourPoly;
            navMesh->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
            if ((neighbourPoly->flags & navQuery.getIncludeFlags()) == 0 || (neighbourPoly->flags & navQuery.getExcludeFlags()) != 0)
                continue;

            // deal explicitly with crossing tile boundaries
            const unsigned char crossSide = bestTile->links[i].side != 0xff ? bestTile->links[i].side >> 1 : 0;
            dtNode* const neighbourNode = nodePool->getNode(neighbourRef, crossSide);
            if (!neighbourNode)
            {
                outOfNodes = true;
                continue;
            }

            if (neighbourNode->flags == 0) // visited the first time
            {
                float left[3], right[3];
                if (dtStatusSucceed(getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, left, right)))
                {
                    if (!canPass(bestRef, neighbourRef, left, right))
                        continue;
                    neighbourNode->pos[0] = (left[0] + right[0]) * 0.5f;
                    neighbourNode->pos[1] = (left[1] + right[1]) * 0.5f;
                    neighbourNode->pos[2] = (left[2] + right[2]) * 0.5f;
                }
            }

            float cost, heuristic;
            if (neighbourRef == endRef) // special case for last node
            {
                const float curCost = dtVdist(bestNode->pos, neighbourNode->pos) * navQuery.getAreaCost(bestPoly->getArea());
                const float endCost = dtVdist(neighbourNode->pos, endPos) * navQuery.getAreaCost(neighbourPoly->getArea());
                cost = bestNode->cost + curCost + endCost;
                heuristic = 0;
            }
            else
            {
                const float curCost = dtVdist(bestNode->pos, neighbourNode->pos) * navQuery.getAreaCost(bestPoly->getArea());
                cost = bestNode->cost + curCost;
                heuristic = dtVdist(neighbourNode->pos, endPos) * H_SCALE;
            }
            const float total = cost + heuristic;

            if ((neighbourNode->flags & (DT_NODE_OPEN | DT_NODE_CLOSED)) && total >= neighbourNode->total)
                continue; // skip the node that is already in open list or visited, and the new result is worse

            neighbourNode->pidx   = nodePool->getNodeIdx(bestNode);
            neighbourNode->id     = neighbourRef;
            neighbourNode->flags &= ~DT_NODE_CLOSED;
            neighbourNode->cost   = cost;
            neighbourNode->total  = total;
            if (neighbourNode->flags & DT_NODE_OPEN) // already in open, update node location
                openList.modify(neighbourNode);
            else // put the node in open list
            {
                neighbourNode->flags |= DT_NODE_OPEN;
                openList.push(neighbourNode);
            }

            if (heuristic < lastBestNodeCost) // update nearest node to target so far
            {
                lastBestNodeCost = heuristic;
                lastBestNode = neighbourNode;
            }
        }
    }

    dtStatus status = getPathToNode(*nodePool, lastBestNode, path, pathCount);
    if (lastBestNode->id != endRef)
        status |= DT_PARTIAL_RESULT;
    if (outOfNodes)
        status |= DT_OUT_OF_NODES;
    return status;
}

inline static void calcLineCollision(float x0, float z0, float x1, float z1, float x, float z, float r2, float* xyz)
{
    float dx = x1 - x0;
    float dz = z1 - z0;
    const float d = sqrtf(dx * dx + dz * dz);
    if (d != 0)
    {
        const float f = 1 / d;
        dx *= f;
        dz *= f;
        float ex = x - x0;
        float ez = z - z0;
        const float t = dx * ex + dz * ez;
        ex = t * dx - ex;
        ez = t * dz - ez;
        const float d2 = ex * ex + ez * ez;
        if (r2 >= d2)
        {
            const float dt = t - sqrtf(r2 - d2);
            if (dt <= 0)
            {
                xyz[0] = x0;
                xyz[2] = z0;
                return;
            }
            if (dt < d)
            {
                xyz[0] = dt * dx + x0;
                xyz[2] = dt * dz + z0;
                return;
            }
        }
    }
    xyz[0] = x1;
    xyz[2] = z1;
}

static inline float dist2(float dx, float dz)
{
    return dx * dx + dz * dz;
}

extern "C" NavStatus navFindPathInRange(const dtNavMeshQueryEx* navQuery, const float* sp, const float* tp,
    float* outFloatBuf, int maxPosCount, float cx, float cz, float radius2)
{
    if (!navQuery)
        return -1;
    if (!sp)
        return -2;
    if (!tp)
        return -3;
    if (!outFloatBuf)
        return -4;
    if (maxPosCount <= 0)
        return -5;
    dtPolyRef sRef;
    float np[3];
    rcVcopy(np, sp);
    dtStatus s = navQuery->findNearestPoly(sp, navQuery->getHalfExtents(), navQuery, &sRef, np);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | s;
    if (!sRef)
        return -6; // invalid source pos
    dtPolyRef polys[FIND_PATH_MAX_POLYS];
    dtRaycastHit rh;
    rh.path = polys;
    rh.maxPath = FIND_PATH_MAX_POLYS;
    s = navQuery->raycast(sRef, np, tp, navQuery, 0, &rh);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | 1LL << 32 | s;
    if (rh.t < 1) // 有碰撞
    {
        dtPolyRef tRef;
        s = navQuery->findNearestPoly(tp, navQuery->getHalfExtents(), navQuery, &tRef, 0);
        if (dtStatusFailed(s))
            return NAVSTATUS_HIGH_BIT | 2LL << 32 | s;
        if (!tRef)
            return -7; // invalid target pos
        s = findPath(*navQuery, sRef, tRef, sp, tp, polys, &rh.pathCount,
            [=](dtPolyRef /*leftRef*/, dtPolyRef /*rightRef*/, const float* left, const float* right)
            {
                return checkDist(left, cx, cz, radius2) || checkDist(right, cx, cz, radius2);
            });
        if (dtStatusFailed(s))
            return NAVSTATUS_HIGH_BIT | 3LL << 32 | s;
    }
    float p[3];
    dtVcopy(p, tp);
#ifdef _MSC_VER
    dtPolyRef* const resPolys = static_cast<dtPolyRef*>(_alloca(sizeof(dtPolyRef) * maxPosCount));
    if (!resPolys)
        return -8;
#else
    dtPolyRef resPolys[maxPosCount];
#endif
    int posCount = 0;
    s = navQuery->findStraightPath(sp, p, polys, rh.pathCount, outFloatBuf, 0, resPolys, &posCount, maxPosCount, DT_STRAIGHTPATH_AREA_CROSSINGS);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | 4LL << 32 | s;
    for (int i = 0; i < posCount; ++i)
    {
        float* const buf = outFloatBuf + 3 * i;
        dtPolyRef ref = resPolys[i];
        if (!ref && i + 1 == posCount)
            ref = polys[rh.pathCount - 1];
        if (ref)
            navQuery->closestPointOnPoly(ref, buf, buf, 0);
        else
            navQuery->findNearestPoly(buf, navQuery->getHalfExtents(), navQuery, &ref, buf);
    }
    if (posCount > 1) // 再验证一下结尾部分是否在范围内并做修正,在raycast成功的情况下很需要修正
    {
        float* buf = outFloatBuf + 3 * (posCount - 1);
        if (dist2(buf[0] - cx, buf[2] - cz) > radius2)
        {
            for (; posCount > 1; buf -= 3, posCount--)
            {
                if (dist2(buf[-3] - cx, buf[-1] - cz) <= radius2)
                {
                    calcLineCollision(buf[0], buf[2], buf[-3], buf[-1], cx, cz, radius2, buf);
                    dtPolyRef ref;
                    navQuery->findNearestPoly(buf, navQuery->getHalfExtents(), navQuery, &ref, buf);
                    break;
                }
            }
        }
    }
    return posCount;
}

class UnCopyable
{
    UnCopyable(UnCopyable&&) = delete;
    UnCopyable(const UnCopyable&) = delete;
    UnCopyable& operator=(const UnCopyable&) = delete;
public:
    UnCopyable() {}
};

struct PolyRefPair
{
    const dtPolyRef a, b;
    PolyRefPair(dtPolyRef a, dtPolyRef b) : a(a), b(b) {}
    bool operator==(const PolyRefPair& p) const { return a == p.a && b == p.b; }
};

namespace std
{
    template<> struct hash<PolyRefPair>
    {
        size_t operator()(const PolyRefPair& p) const
        {
#ifdef DT_POLYREF64
            return static_cast<size_t>((((p.a ^ (p.a >> 32)) * C0)
                                      ^ ((p.b ^ (p.b >> 32)) * C1)) >> 32);
#else
            return static_cast<size_t>(((p.a * C0) ^ (p.b * C1)) >> 32);
#endif
        }
    private:
        static const uint64_t C0 = 0x65d200ce55b19ad8L;
        static const uint64_t C1 = 0x4f2162926e40c299L;
    };
}

class NavFieldCtx : private UnCopyable
{
    std::unordered_map<PolyRefPair, bool> linkResCache; // key的高位是dtPolyRef(from),低位是dtPolyRef(to); value是能否通过

public:
    virtual ~NavFieldCtx() {}
    virtual void getCenter(float& cx, float& cz) const = 0;
    virtual bool isPointIn(float x, float z) const = 0;
    virtual void moveToEdge(float& x, float& z) const = 0; // 移动坐标(x,z)到最近的区域边缘
    virtual bool canPass(dtPolyRef fromRef, dtPolyRef toRef, const float* left, const float* right) = 0;
    virtual int fixPath(const dtNavMeshQueryEx* navQuery, float* posBuf, int posCount) const = 0;
    virtual bool fixLine(const dtNavMeshQueryEx* navQuery, const float* posFrom, float* posTo) const = 0;
    bool tryGetLinkResCache(dtPolyRef fromRef, dtPolyRef toRef, bool& res) const
    {
        auto it = linkResCache.find(PolyRefPair(fromRef, toRef));
        if (it == linkResCache.end())
            return false;
        res = it->second;
        return true;
    }
    void addLinkResCache(dtPolyRef fromRef, dtPolyRef toRef, bool res)
    {
        linkResCache[PolyRefPair(fromRef, toRef)] = res;
    }
    bool findPos(const dtNavMeshQueryEx* navQuery, float* pos) const
    {
        if (!isPointIn(pos[0], pos[2]))
            moveToEdge(pos[0], pos[2]);
        const float tx = pos[0];
        const float tz = pos[2];
        float cx = tx, cz = tz;
        getCenter(cx, cz);
        const float dx = tx - cx;
        const float dz = tz - cz;
        for (float f = 1; f >= 0; f -= 1.0f / 16)
        {
            pos[0] = cx + dx * f;
            pos[2] = cz + dz * f;
            dtPolyRef sRef = 0;
            const dtStatus s = navQuery->findNearestPoly(pos, navQuery->getHalfExtents(), navQuery, &sRef, pos);
            if (dtStatusSucceed(s) && sRef && isPointIn(pos[0], pos[2]))
                return true;
        }
        return false;
    }
};

class NavCircleCtx : public NavFieldCtx
{
    const float cx, cz, radius, radius2;

    static float pointToLineDist2(float x, float z, float x0, float z0, float x1, float z1)
    {
        const float dx = x1 - x0;
        const float dz = z1 - z0;
        const float pdx = x - x0;
        const float pdz = z - z0;
        const float u = (pdx * dx + pdz * dz) / (dx * dx + dz * dz);
        if (u > 0)
        {
            if (u < 1)
                return dist2(pdx - u * dx, pdz - u * dz);
            return dist2(x - x1, z - z1);
        }
        return dist2(pdx, pdz);
    }

    bool isPointIn0(float x, float z) const
    {
        return dist2(x - cx, z - cz) <= radius2;
    }
public:
    NavCircleCtx(float cx_, float cz_, float radius_) : cx(cx_), cz(cz_), radius(radius_), radius2(radius_ * radius_) {}
    virtual ~NavCircleCtx() override {}

    virtual void getCenter(float& x, float& z) const override
    {
        x = cx;
        z = cz;
    }

    virtual bool isPointIn(float x, float z) const override
    {
        return isPointIn0(x, z);
    }

    virtual void moveToEdge(float& x, float& z) const override
    {
        const float dx = x - cx;
        const float dz = z - cz;
        const float d = sqrtf(dist2(dx, dz));
        if (d > 0)
        {
            const float f = radius / d;
            x = cx + dx * f;
            z = cz + dz * f;
        }
    }

    virtual bool canPass(dtPolyRef fromRef, dtPolyRef toRef, const float* left, const float* right) override
    {
        if (checkDist(left, cx, cz, radius2) || checkDist(right, cx, cz, radius2)) // 有一端在圆内即可穿过
            return true;
        bool res;
        if (tryGetLinkResCache(fromRef, toRef, res)) // 先尝试从上下文缓存中得到结果
            return res;
        res = pointToLineDist2(cx, cz, left[0], left[2], right[0], right[2]) < radius2; // 两端不在圆内但中间部分在圆内也可穿过
        addLinkResCache(fromRef, toRef, res); // 结果加入上下文缓存
        return res;
    }

    virtual int fixPath(const dtNavMeshQueryEx* navQuery, float* posBuf, int posCount) const override
    {
        if (posCount > 1)
        {
            float* buf = posBuf + 3 * (posCount - 1);
            if (dist2(buf[0] - cx, buf[2] - cz) > radius2)
            {
                for (; posCount > 1; buf -= 3, posCount--)
                {
                    if (dist2(buf[-3] - cx, buf[-1] - cz) <= radius2)
                    {
                        calcLineCollision(buf[0], buf[2], buf[-3], buf[-1], cx, cz, radius2, buf);
                        dtPolyRef ref;
                        navQuery->findNearestPoly(buf, navQuery->getHalfExtents(), navQuery, &ref, buf);
                        break;
                    }
                }
            }
        }
        return posCount;
    }

    virtual bool fixLine(const dtNavMeshQueryEx* navQuery, const float* posFrom, float* posTo) const
    {
        if (isPointIn0(posTo[0], posTo[2]))
            return false;
        calcLineCollision(posTo[0], posTo[2], posFrom[0], posFrom[2], cx, cz, radius2, posTo);
        dtPolyRef ref;
        navQuery->findNearestPoly(posTo, navQuery->getHalfExtents(), navQuery, &ref, posTo);
        return true;
    }
};

class NavPolygonCtx : public NavFieldCtx
{
public:
    static constexpr int MIN_VERT_COUNT = 3;
private:
    // const float minX, minZ, maxX, maxZ;
    const float cx, cz;
    const int vertCount;
    float vertXZs[MIN_VERT_COUNT * 2]; // [vertCount * 2]

    static float calcCenter(const float* vertXZs, int n)
    {
        float a = 0;
        n *= 2;
        for (int i = 0; i < n; i += 2)
            a += vertXZs[i];
        return a / n;
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

    static float mult(float x0, float z0, float x1, float z1, float x2, float z2)
    {
        return (x0 - x1) * (z0 - z2) - (z0 - z1) * (x0 - x2);
    }

    static bool segSegCollision(float ax, float az, float bx, float bz, float cx, float cz, float dx, float dz)
    {
        // aabb判断在这个环境下意义不大, 因为都是临近多边形的位置
        // if(Math.max(ax, bx) < Math.min(cx, dx)) return false;
        // if(Math.max(az, bz) < Math.min(cz, dz)) return false;
        // if(Math.max(cx, dx) < Math.min(ax, bx)) return false;
        // if(Math.max(cz, dz) < Math.min(az, bz)) return false;
        return (mult(ax, az, cx, cz, bx, bz) * mult(ax, az, bx, bz, dx, dz) >= 0) &&
                mult(cx, cz, ax, az, dx, dz) * mult(cx, cz, dx, dz, bx, bz) >= 0;
    }

    bool segPolyCollision(float ax, float az, float bx, float bz) const
    {
        const int n = vertCount * 2;
        float x = vertXZs[n - 2];
        float z = vertXZs[n - 1];
        for (int i = 0; i < n; i += 2)
        {
            const float xx = vertXZs[i];
            const float zz = vertXZs[i + 1];
            if (segSegCollision(ax, az, bx, bz, x, z, xx, zz))
                return true;
            x = xx;
            z = zz;
        }
        return false;
    }

    bool isPointIn0(float x, float z) const
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

    float segPolyFactor(float ax, float az, float bx, float bz) const
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
public:
    // 注意不能直接new来构造,需分配足够的扩充空间才能构造
    NavPolygonCtx(const float* vertXZs_, int vertCount_) : cx(calcCenter(vertXZs_, vertCount_)),
        cz(calcCenter(vertXZs_ + 1, vertCount_)), vertCount(vertCount_)
    {
        memcpy(const_cast<float*>(vertXZs), vertXZs_, vertCount_ * 2 * sizeof(float));
    }

    virtual ~NavPolygonCtx() override {}

    virtual void getCenter(float& x, float& z) const override
    {
        x = cx;
        z = cz;
    }

    virtual bool isPointIn(float x, float z) const override
    {
        return isPointIn0(x, z);
    }

    virtual void moveToEdge(float& x, float& z) const override
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

    virtual bool canPass(dtPolyRef fromRef, dtPolyRef toRef, const float* left, const float* right) override
    {
        bool res;
        if (tryGetLinkResCache(fromRef, toRef, res)) // 先尝试从上下文缓存中得到结果
            return res;
        res = isPointIn0(left[0], left[2]) || isPointIn0(right[0], right[2]) // 有一端在多边形内即可穿过
            || segPolyCollision(left[0], left[2], right[0], right[2]); // 两端不在多边形内但中间部分在多边形内也可穿过
        addLinkResCache(fromRef, toRef, res); // 结果加入上下文缓存
        return res;
    }

    virtual int fixPath(const dtNavMeshQueryEx* navQuery, float* posBuf, int posCount) const override
    {
        if (posCount > 1)
        {
            float* buf = posBuf + 3 * (posCount - 1);
            if (!isPointIn0(buf[0], buf[2]))
            {
                for (; posCount > 1; buf -= 3, posCount--)
                {
                    if (isPointIn0(buf[-3], buf[-1]))
                    {
                        const float f = segPolyFactor(buf[-3], buf[-1], buf[0], buf[2]);
                        buf[0] = buf[-3] + (buf[0] - buf[-3]) * f;
                        buf[1] = buf[-2] + (buf[1] - buf[-2]) * f;
                        buf[2] = buf[-1] + (buf[2] - buf[-1]) * f;
                        dtPolyRef ref;
                        navQuery->findNearestPoly(buf, navQuery->getHalfExtents(), navQuery, &ref, buf);
                        break;
                    }
                }
            }
        }
        return posCount;
    }

    virtual bool fixLine(const dtNavMeshQueryEx* navQuery, const float* posFrom, float* posTo) const
    {
        if (isPointIn0(posTo[0], posTo[2]))
            return false;
        const float f = segPolyFactor(posFrom[0], posFrom[2], posTo[0], posTo[2]);
        posTo[0] = posFrom[0] + (posTo[0] - posFrom[0]) * f;
        posTo[1] = posFrom[1] + (posTo[1] - posFrom[1]) * f;
        posTo[2] = posFrom[2] + (posTo[2] - posFrom[2]) * f;
        dtPolyRef ref;
        navQuery->findNearestPoly(posTo, navQuery->getHalfExtents(), navQuery, &ref, posTo);
        return true;
    }
};

extern "C" NavStatus navAllocCircleCtx(float cx, float cz, float radius, NavFieldCtx** outFieldCtx)
{
    if (!outFieldCtx)
        return -1;
    void* const mem = dtAlloc(sizeof(NavCircleCtx), DT_ALLOC_PERM);
    if (!mem)
        return -2;
    *outFieldCtx = new(mem) NavCircleCtx(cx, cz, radius);
    return 0;
}

extern "C" NavStatus navAllocPolygonCtx(const float* vertXZs, int vertCount, NavFieldCtx** outFieldCtx)
{
    if (!vertXZs)
        return -1;
    if (vertCount < NavPolygonCtx::MIN_VERT_COUNT)
        return -2;
    if (!outFieldCtx)
        return -3;
    void* const mem = dtAlloc(sizeof(NavPolygonCtx) + (vertCount - NavPolygonCtx::MIN_VERT_COUNT) * 2 * sizeof(float), DT_ALLOC_PERM);
    if (!mem)
        return -4;
    *outFieldCtx = new(mem) NavPolygonCtx(vertXZs, vertCount);
    return 0;
}

extern "C" void navFreeFieldCtx(NavFieldCtx* fieldCtx)
{
    if (!fieldCtx)
        return;
    fieldCtx->~NavFieldCtx();
    dtFree(fieldCtx);
}

extern "C" NavStatus navFindPathInField(const dtNavMeshQueryEx* navQuery, const float* sp, const float* tp,
    float* outFloatBuf, int maxPosCount, bool onlyRay, NavFieldCtx* fieldCtx)
{
    if (!navQuery)
        return -1;
    if (!sp)
        return -2;
    if (!tp)
        return -3;
    if (!onlyRay)
    {
        if (!outFloatBuf)
            return -4;
        if (maxPosCount <= 0)
            return -5;
    }
    else if (!outFloatBuf && maxPosCount > 0)
        return -6;
    dtPolyRef sRef;
    float np[3];
    rcVcopy(np, sp);
    dtStatus s = navQuery->findNearestPoly(sp, navQuery->getHalfExtents(), navQuery, &sRef, np);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | s;
    if (!sRef)
        return -7; // invalid source pos
    dtPolyRef polys[FIND_PATH_MAX_POLYS];
    dtRaycastHit rh;
    rh.path = polys;
    rh.maxPath = FIND_PATH_MAX_POLYS;
    s = navQuery->raycast(sRef, np, tp, navQuery, 0, &rh);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | 1LL << 32 | s;
    // if (rh.pathCount <= 0)
    //     return -8;
    float p[3];
    if (onlyRay)
    {
        if (rh.t < 1) // 有碰撞
        {
            dtVlerp(p, np, tp, rh.t);
            s = navQuery->closestPointOnPolyBoundary(rh.pathCount > 0 ? polys[rh.pathCount - 1] : sRef, p, p);
            if (dtStatusFailed(s))
                return NAVSTATUS_HIGH_BIT | 2LL << 32 | s;
            if (maxPosCount == 1) // 有碰撞但只需终点
            {
                dtVcopy(outFloatBuf, p);
                if (fieldCtx)
                    fieldCtx->fixLine(navQuery, sp, outFloatBuf);
                return 1;
            }
            if (maxPosCount <= 0) // 直接返回有碰撞
                return 1;
        }
        else if (maxPosCount == 1) // 没碰撞但只需终点
        {
            s = navQuery->closestPointOnPoly(rh.pathCount > 0 ? polys[rh.pathCount - 1] : sRef, tp, outFloatBuf, 0);
            if (dtStatusFailed(s))
                return NAVSTATUS_HIGH_BIT | 3LL << 32 | s;
            if (fieldCtx)
                fieldCtx->fixLine(navQuery, sp, outFloatBuf);
            return 1;
        }
        else if (maxPosCount <= 0) // 直接返回没碰撞
            return 0;
        else
            dtVcopy(p, tp); // 没碰撞但需要所有位置
    }
    else
    {
        if (rh.t < 1) // 有碰撞
        {
            dtPolyRef tRef;
            s = navQuery->findNearestPoly(tp, navQuery->getHalfExtents(), navQuery, &tRef, 0);
            if (dtStatusFailed(s))
                return NAVSTATUS_HIGH_BIT | 4LL << 32 | s;
            if (!tRef)
                return -8; // invalid target pos
            if (fieldCtx)
            {
                s = findPath(*navQuery, sRef, tRef, sp, tp, polys, &rh.pathCount,
                    [=](dtPolyRef leftRef, dtPolyRef rightRef, const float* left, const float* right)
                {
                    return fieldCtx->canPass(leftRef, rightRef, left, right);
                });
                if (dtStatusFailed(s))
                    return NAVSTATUS_HIGH_BIT | 5LL << 32 | s;
            }
            else
            {
                s = navQuery->findPath(sRef, tRef, sp, tp, navQuery, polys, &rh.pathCount, FIND_PATH_MAX_POLYS);
                if (dtStatusFailed(s))
                    return NAVSTATUS_HIGH_BIT | 6LL << 32 | s;
            }
        }
        dtVcopy(p, tp);
    }
#ifdef _MSC_VER
    dtPolyRef* const resPolys = static_cast<dtPolyRef*>(_alloca(sizeof(dtPolyRef) * maxPosCount));
    if (!resPolys)
        return -9;
#else
    dtPolyRef resPolys[maxPosCount];
#endif
    int posCount = 0;
    s = navQuery->findStraightPath(sp, p, polys, rh.pathCount, outFloatBuf, 0, resPolys, &posCount, maxPosCount, DT_STRAIGHTPATH_AREA_CROSSINGS);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | 7LL << 32 | s;
    for (int i = 0; i < posCount; ++i)
    {
        float* const buf = outFloatBuf + 3 * i;
        dtPolyRef ref = resPolys[i];
        if (!ref && i + 1 == posCount)
            ref = polys[rh.pathCount - 1];
        if (ref)
            navQuery->closestPointOnPoly(ref, buf, buf, 0);
        else
            navQuery->findNearestPoly(buf, navQuery->getHalfExtents(), navQuery, &ref, buf);
    }
    return fieldCtx ? fieldCtx->fixPath(navQuery, outFloatBuf, posCount) : posCount; // 再验证一下结尾部分是否在范围内并做修正,在raycast成功的情况下很需要修正
}

extern "C" NavStatus navFindPos(const dtNavMeshQueryEx* navQuery, float* p, int method, float xzRange)
{
    if (!navQuery)
        return -1;
    if (!p)
        return -2;
    const float* pHalfExtents = navQuery->getHalfExtents();
    float halfExtents[3];
    if (xzRange >= 0)
    {
        halfExtents[0] = xzRange;
        halfExtents[1] = pHalfExtents[1];
        halfExtents[2] = xzRange;
        pHalfExtents = halfExtents;
    }
    dtPolyRef sRef = 0;
    const dtStatus s = navQuery->findNearestPoly(p, pHalfExtents, navQuery, &sRef, p, method);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | s;
    if (!sRef)
        return -3; // invalid source pos
    return 0;
}

extern "C" NavStatus navFindPosInField(const dtNavMeshQueryEx* navQuery, float* p, const NavFieldCtx* fieldCtx)
{
    if (!navQuery)
        return -1;
    if (!p)
        return -2;
    if (!fieldCtx)
    {
        dtPolyRef sRef = 0;
        const dtStatus s = navQuery->findNearestPoly(p, navQuery->getHalfExtents(), navQuery, &sRef, p);
        if (dtStatusFailed(s))
            return NAVSTATUS_HIGH_BIT | s;
        if (!sRef)
            return -3; // invalid source pos
    }
    else if (!fieldCtx->findPos(navQuery, p))
        return -4;
    return 0;
}

// Returns portal points between two polygons.
dtStatus getPortalPoints(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
    dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile, float* left, float* right)
{
    // Find the link that points to the 'to' polygon.
    const dtLink* link = 0;
    for (unsigned i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromTile->links[i].next)
    {
        if (fromTile->links[i].ref == to)
        {
            link = &fromTile->links[i];
            break;
        }
    }
    if (!link)
        return DT_FAILURE | DT_INVALID_PARAM | (203u << 16);

    // Handle off-mesh connections.
    if (fromPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
    {
        // Find link that points to first vertex.
        for (unsigned i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromTile->links[i].next)
        {
            if (fromTile->links[i].ref == to)
            {
                const int v = fromTile->links[i].edge;
                dtVcopy(left , &fromTile->verts[fromPoly->verts[v] * 3]);
                dtVcopy(right, &fromTile->verts[fromPoly->verts[v] * 3]);
                return DT_SUCCESS;
            }
        }
        return DT_FAILURE | DT_INVALID_PARAM | (204u << 16);
    }

    if (toPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
    {
        for (unsigned i = toPoly->firstLink; i != DT_NULL_LINK; i = toTile->links[i].next)
        {
            if (toTile->links[i].ref == from)
            {
                const int v = toTile->links[i].edge;
                dtVcopy(left , &toTile->verts[toPoly->verts[v] * 3]);
                dtVcopy(right, &toTile->verts[toPoly->verts[v] * 3]);
                return DT_SUCCESS;
            }
        }
        return DT_FAILURE | DT_INVALID_PARAM | (205u << 16);
    }

    // Find portal vertices.
    const int v0 = fromPoly->verts[link->edge];
    const int v1 = fromPoly->verts[(link->edge + 1) % static_cast<int>(fromPoly->vertCount)];
    dtVcopy(left , &fromTile->verts[v0 * 3]);
    dtVcopy(right, &fromTile->verts[v1 * 3]);

    // If the link is at tile boundary, dtClamp the vertices to
    // the link width.
    if (link->side != 0xff)
    {
        // Unpack portal limits.
        if (link->bmin != 0 || link->bmax != 255)
        {
            const float s = 1.0f / 255.0f;
            const float tmin = link->bmin * s;
            const float tmax = link->bmax * s;
            dtVlerp(left , &fromTile->verts[v0 * 3], &fromTile->verts[v1 * 3], tmin);
            dtVlerp(right, &fromTile->verts[v0 * 3], &fromTile->verts[v1 * 3], tmax);
        }
    }

    return DT_SUCCESS;
}

extern "C" NavStatus navFindWater(const dtNavMeshQueryEx* navQuery, const float* sp, float r, float* outFloatBuf, int maxPosCount)
{
    if (!navQuery)
        return -1;
    if (!sp)
        return -2;
    if (!outFloatBuf)
        return -3;
    if (maxPosCount <= 0)
        return -4;
    dtPolyRef sRef;
    static const dtQueryFilter defaultQueryFilter;
    dtStatus s = navQuery->findNearestPoly(sp, navQuery->getHalfExtents(), &defaultQueryFilter, &sRef, 0);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | s;
    if (!sRef)
        return -5; // invalid source pos

    const dtNavMesh& navMesh = *navQuery->getAttachedNavMesh();
    dtNodePool* const nodePool = navQuery->getNodePool();
    nodePool->clear();
    dtNode* const startNode = nodePool->getNode(sRef);
    dtVcopy(startNode->pos, sp);
    startNode->pidx = 0;
    startNode->cost = 0;
    startNode->total = 0;
    startNode->id = sRef;
    startNode->flags = DT_NODE_OPEN;
    dtNodeQueue& openList = navQuery->getOpenList();
    openList.clear();
    openList.push(startNode);
    const float radiusSqr = dtSqr(r);

    while (!openList.empty())
    {
        dtNode* const bestNode = openList.pop();
        bestNode->flags &= ~DT_NODE_OPEN;
        bestNode->flags |= DT_NODE_CLOSED;
        // Get poly and tile. The API input has been cheked already, skip checking internal data.
        const dtPolyRef bestRef = bestNode->id;
        const dtMeshTile* bestTile = 0;
        const dtPoly* bestPoly = 0;
        navMesh.getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
        // Get parent poly and tile.
        const dtMeshTile* parentTile = 0;
        const dtPoly* parentPoly = 0;
        dtPolyRef parentRef = 0;
        if (bestNode->pidx)
        {
            parentRef = nodePool->getNodeAtIdx(bestNode->pidx)->id;
            if (parentRef)
                navMesh.getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);
        }

        for (unsigned i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
        {
            const dtLink* const link = &bestTile->links[i];
            const dtPolyRef neighbourRef = link->ref;
            // Skip invalid neighbours and do not follow back to parent.
            if (!neighbourRef || neighbourRef == parentRef)
                continue;
            // Expand to neighbour
            const dtMeshTile* neighbourTile = 0;
            const dtPoly* neighbourPoly = 0;
            navMesh.getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
            // Do not advance if the polygon is excluded by the filter.
            // if (!navQuery->passFilter(neighbourRef, neighbourTile, neighbourPoly))
            if ((neighbourPoly->flags & ~(NAVMESH_POLYFLAGS_JUMP_NEAR | NAVMESH_POLYFLAGS_JUMP_MEDIUM | NAVMESH_POLYFLAGS_JUMP_FAR)) != 0)
                continue;
            // Find edge and calc distance to the edge.
            float va[3], vb[3];
            if (dtStatusFailed(getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb)))
                continue;

            if ((neighbourPoly->flags & NAVMESH_POLYFLAGS_WATER) != 0)
            {
                const float tp[3] = { (va[0] + vb[0]) * 0.5f, (va[1] + vb[1]) * 0.5f, (va[2] + vb[2]) * 0.5f };
                if (maxPosCount == 1)
                {
                    dtVcopy(outFloatBuf, tp);
                    return 1;
                }
                int n = 0;
                for (const dtNode* node = nodePool->getNode(bestRef); node && node->id; node = nodePool->getNodeAtIdx(node->pidx))
                    n++;
                if (n > maxPosCount)
                    return -6;
#ifdef _MSC_VER
                dtPolyRef* const polys = static_cast<dtPolyRef*>(_alloca(sizeof(dtPolyRef) * n));
                dtPolyRef* const resPolys = static_cast<dtPolyRef*>(_alloca(sizeof(dtPolyRef) * maxPosCount));
                if (!polys || !resPolys)
                    return -7;
#else
                dtPolyRef polys[n], resPolys[maxPosCount];
#endif
                int j = n;
                for (const dtNode* node = nodePool->getNode(bestRef); node && node->id; node = nodePool->getNodeAtIdx(node->pidx))
                    polys[--j] = node->id;
                int posCount = 0;
                s = navQuery->findStraightPath(sp, tp, polys, n, outFloatBuf, 0, resPolys, &posCount, maxPosCount, DT_STRAIGHTPATH_AREA_CROSSINGS);
                if (dtStatusFailed(s))
                    return NAVSTATUS_HIGH_BIT | 1LL << 32 | s;
                for (int k = 0; k < posCount; ++k)
                {
                    float* const buf = outFloatBuf + 3 * k;
                    dtPolyRef ref = resPolys[k];
                    if (!ref && k + 1 == posCount)
                        ref = polys[n - 1];
                    if (ref)
                        navQuery->closestPointOnPoly(ref, buf, buf, 0);
                    else
                        navQuery->findNearestPoly(buf, navQuery->getHalfExtents(), navQuery, &ref, buf);
                }
                return posCount;
            }

            // If the circle is not touching the next polygon, skip it.
            float tseg;
            if (dtDistancePtSegSqr2D(sp, va, vb, tseg) > radiusSqr)
                continue;
            dtNode* const neighbourNode = nodePool->getNode(neighbourRef);
            if (!neighbourNode) // DT_OUT_OF_NODES
                continue;
            if (neighbourNode->flags & DT_NODE_CLOSED)
                continue;
            if (neighbourNode->flags == 0)
                dtVlerp(neighbourNode->pos, va, vb, 0.5f);
            // const float cost = navQuery->getCost(bestNode->pos, neighbourNode->pos, parentRef, parentTile, parentPoly,
            //    bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly);
            const float cost = dtVdist(bestNode->pos, neighbourNode->pos);
            const float total = bestNode->total + cost;
            // The node is already in open list and the new result is worse, skip.
            if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
                continue;
            neighbourNode->id = neighbourRef;
            neighbourNode->pidx = nodePool->getNodeIdx(bestNode);
            neighbourNode->total = total;
            if (neighbourNode->flags & DT_NODE_OPEN)
                openList.modify(neighbourNode);
            else
            {
                neighbourNode->flags = DT_NODE_OPEN;
                openList.push(neighbourNode);
            }
        }
    }
    return 0;
}

float frand()
{
    return rand() / (RAND_MAX + 1.f);
}

extern "C" NavStatus navRandomPos(const dtNavMeshQueryEx* navQuery, float* p, float r)
{
    if (!navQuery)
        return -1;
    if (!p)
        return -2;
    dtPolyRef sRef;
    dtStatus s = navQuery->findNearestPoly(p, navQuery->getHalfExtents(), navQuery, &sRef, p);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | s;
    if (!sRef)
        return -3; // invalid source pos
    s = navQuery->findRandomPointAroundCircle(sRef, p, r, navQuery, frand, &sRef, p);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | 1LL << 32 | s;
    return 0;
}

extern "C" NavStatus navSaveNavMesh(const dtNavMeshEx* navMesh, FILE* fp)
{
    if (!navMesh)
        return -1;
    if (!fp)
        return -2;

    if (fwrite(&RC_NAVMESH_FILE_MAGIC_VERSION, sizeof(RC_NAVMESH_FILE_MAGIC_VERSION), 1, fp) != 1)
        return -3;
    if (fwrite(&navMesh->getCfg(), sizeof(rcConfig), 1, fp) != 1)
        return -4;
    const int w = navMesh->getTileW();
    const int h = navMesh->getTileH();
    const int zero = 0;
    for (int z = 0; z < h; z++)
    {
        for (int x = 0; x < w; x++)
        {
            const dtMeshTile* const tile = navMesh->getTileAt(x, z, 0);
            if (tile && tile->header)
            {
                if (fwrite(&tile->dataSize, sizeof(tile->dataSize), 1, fp) != 1)
                    return -5;
                if (fwrite(tile->data, 1, static_cast<size_t>(tile->dataSize), fp) != static_cast<size_t>(tile->dataSize))
                    return -6;
            }
            else if (fwrite(&zero, sizeof(zero), 1, fp) != 1)
                return -7;
        }
    }
    return 0;
}

int circleLineCross(float cx, float cz, float r2, float x0, float z0, float x1, float z1, float res[4])
{
    const float dx = x1 - x0;
    const float dz = z1 - z0;
    const float invLen = 1 / sqrtf(dist2(dx, dz));
    const float fx = dx * invLen;
    const float fz = dz * invLen;
    const float t = fx * (cx - x0) + fz * (cz - z0);
    const float ex = t * fx + x0; // 圆心垂直投射到直线上的坐标
    const float ez = t * fz + z0;
    const float ec2 = dist2(ex - cx, ez - cz); // 圆心到直线距离平方
    if (ec2 > r2)
        return 0;
    if (ec2 < r2)
    {
        const float dt = sqrtf(r2 - ec2);
        const float tx = fx * dt;
        const float tz = fz * dt;
        float x = ex - tx;
        float z = ez - tz;
        int i = 0;
        if ((x0 < x) == (x < x1) && (z0 < z) == (z < z1))
        {
            res[0] = x;
            res[1] = z;
            i = 2;
        }
        x = ex + tx;
        z = ez + tz;
        if ((x0 < x) == (x < x1) && (z0 < z) == (z < z1))
        {
            res[i] = x;
            res[i + 1] = z;
            i += 2;
        }
        return i >> 1;
    }
    if ((x0 < ex) == (ex < x1) && (z0 < ez) == (ez < z1))
    {
        res[0] = ex;
        res[1] = ez;
        return 1;
    }
    return 0;
}

extern "C" NavStatus navCheckArcCollision(const dtNavMeshQueryEx* navQuery, float cx, float cz, float r, float x0, float z0, float x1, float z1)
{
    if (!navQuery)
        return -1;

    const float center[3] = { cx, 0, cz };
    const float extend[3] = { r, 1000.f, r };
    const int MAX_POLY_COUNT = 64;
    dtPolyRef polys[MAX_POLY_COUNT];
    int polyCount;
    dtStatus s = navQuery->queryPolygons(center, extend, navQuery, polys, &polyCount, MAX_POLY_COUNT);
    if (dtStatusFailed(s))
        return -2;

    const float r2 = r * r, dx = x1 - x0, dz = z1 - z0;
    const int kNavMeshVertsPerPoly = 6;
    const int kMaxSegsPerPoly = kNavMeshVertsPerPoly * 3;
    float segs[kMaxSegsPerPoly * 3 * 2];
    for (int i = 0; i < polyCount; i++)
    {
        int segCount;
        s = navQuery->getPolyWallSegments(polys[i], navQuery, segs, 0, &segCount, kMaxSegsPerPoly);
        if (dtStatusFailed(s))
            continue;
        for (int j = 0, n = segCount * 6; j < n; j += 6)
        {
            const int cn = circleLineCross(cx, cz, r2, segs[j], segs[j + 2], segs[j + 3], segs[j + 5], segs);
            if (cn > 0 && (pointInArc(segs[0], segs[1], x0, z0, dx, dz) || (cn == 2 && pointInArc(segs[2], segs[3], x0, z0, dx, dz))))
                return 1;
        }
    }
    return 0;
}

template<typename T>
class AutoDtFreer : private UnCopyable
{
    T* obj;
public:
    explicit AutoDtFreer(T* o) : obj(o) {}
    ~AutoDtFreer() { close(); }
    void close() { if (obj) { obj->~T(); dtFree(obj); obj = 0; } }
};

template<typename T>
class AutoArrayDeleter : private UnCopyable
{
    T* buf;
public:
    explicit AutoArrayDeleter(T* b) : buf(b) {}
    ~AutoArrayDeleter() { close(); }
    void close() { if (buf) { delete[] buf; buf = 0; } }
};

class AutoFileCloser : private UnCopyable
{
    FILE* fp;
public:
    explicit AutoFileCloser(FILE* f) : fp(f) {}
    ~AutoFileCloser() { close(); }
    void close() { if (fp) { fclose(fp); fp = 0; } }
};

class AutoStopTimer : private UnCopyable
{
    rcContext* rcCtx;
    const rcTimerLabel timerLabel;
public:
    explicit AutoStopTimer(rcContext* c, rcTimerLabel label) : rcCtx(c), timerLabel(label) {}
    ~AutoStopTimer() { close(); }
    void close() { if (rcCtx) { rcCtx->stopTimer(timerLabel); rcCtx = 0; } }
};

/*
class Defer : private UnCopyable
{
    std::function<void()> fun;
public:
    explicit Defer(std::function<void()>&& f) : fun(std::move(f)) {}
    ~Defer() { close(); }
    void close() { if (fun) { fun(); fun = 0; } }
};
*/

static int readAllFile(const char* filename, char** outBuf)
{
    if (!filename)
        return -1;
    if (!outBuf)
        return -2;
    FILE* const fp = fopen(filename, "rb");
    if (!fp)
        return -3;
    fseek(fp, 0, SEEK_END);
    const int n = static_cast<int>(ftell(fp));
    fseek(fp, 0, SEEK_SET);
    char* buf = new char[n];
    const int n2 = static_cast<int>(fread(buf, 1, n, fp));
    fclose(fp);
    if (n != n2)
    {
        delete[] buf;
        return -4;
    }
    *outBuf = buf;
    return n;
}

#define CHECK_FWRITE_N(x, n) { if (fwrite((x), sizeof(*(x)), static_cast<size_t>(n), fp) != static_cast<size_t>(n)) return -99; }
#define CHECK_FWRITE(x) CHECK_FWRITE_N(x, 1)

static int saveFullPolyMeshDebugData(const char* navMeshFileName, const dtNavMesh& navMesh, int tileW, int tileH)
{
    if (!navMeshFileName)
        return -1;
    char debugFileName[512];
    sprintf(debugFileName, "%s.debug", navMeshFileName);
    FILE* const fp = fopen(debugFileName, "wb");
    if (!fp)
        return -2;
    AutoFileCloser closer(fp);
    std::vector<float> vertices;
    CHECK_FWRITE(&tileW);
    CHECK_FWRITE(&tileH);
    int totalPolyCount = 0;
    const int zero = 0;
    for (int x = 0; x < tileW; x++)
    {
        for (int z = 0; z < tileH; z++)
        {
            CHECK_FWRITE(&x);
            CHECK_FWRITE(&z)
            const dtMeshTile* const tile = navMesh.getTileAt(x, z, 0);
            if (!tile)
            {
                CHECK_FWRITE(&zero);
                continue;
            }
            const int polyCount = tile->header->polyCount;
            totalPolyCount += polyCount;
            CHECK_FWRITE(&polyCount)
            for (int i = 0; i < polyCount; ++i)
            {
                const dtPoly& poly = tile->polys[i];
                const unsigned char type = poly.getType();
                CHECK_FWRITE(&type);
                CHECK_FWRITE(&poly.flags);
                CHECK_FWRITE(&poly.vertCount);
                for (int j = 0; j < poly.vertCount; j++)
                    CHECK_FWRITE_N(tile->verts + poly.verts[j] * 3, 3);
                if (poly.vertCount <= 2)
                    continue;
                const dtPolyDetail& dm = tile->detailMeshes[i];
                const unsigned char triCount = dm.triCount;
                CHECK_FWRITE(&triCount);
                for (int j = 0; j < triCount; ++j)
                {
                    const unsigned char* const dt = &tile->detailTris[(dm.triBase + j) * 4];
                    for (int k = 0; k < 3; ++k)
                    {
                        const float* vertex;
                        if (dt[k] < poly.vertCount)
                            vertex = tile->verts + poly.verts[dt[k]] * 3;
                        else
                            vertex = tile->detailVerts + (dm.vertBase + dt[k] - poly.vertCount) * 3;
                        CHECK_FWRITE_N(vertex, 3);
                    }
                }
            }
        }
    }
    return 0;
}

static int saveHeightfield(const char* path, int mapId, const rcCompactHeightfield* chf, int tileX, int tileZ)
{
    if (!path)
        return -1;
    char fileName[512];
    sprintf(fileName, "%s/map/map%d/navmesh/tile_%d_%d.chf", path, mapId, tileX, tileZ);
    FILE* const fp = fopen(fileName, "wb");
    if (!fp)
        return -2;
    AutoFileCloser closer(fp);
    if (!chf)
        return 0;
    CHECK_FWRITE(&chf->width);
    CHECK_FWRITE(&chf->height);
    CHECK_FWRITE(&chf->spanCount);
    CHECK_FWRITE(&chf->walkableHeight);
    CHECK_FWRITE(&chf->walkableClimb);
    CHECK_FWRITE(&chf->borderSize);
    CHECK_FWRITE(&chf->maxDistance);
    CHECK_FWRITE(&chf->maxRegions);
    CHECK_FWRITE_N(chf->bmin, 3);
    CHECK_FWRITE_N(chf->bmax, 3);
    CHECK_FWRITE(&chf->cs);
    CHECK_FWRITE(&chf->ch);
    CHECK_FWRITE_N(chf->cells, chf->width * chf->height);
    CHECK_FWRITE_N(chf->spans, chf->spanCount);
    if (!chf->dist)
    {
        const int zero = 0;
        CHECK_FWRITE(&zero);
    }
    else
    {
        CHECK_FWRITE(&chf->spanCount);
        CHECK_FWRITE_N(chf->dist, chf->spanCount);
    }
    CHECK_FWRITE_N(chf->areas, chf->spanCount);
    return 0;
}

static int saveContourSet(const char* path, int mapId, const rcContourSet* cset, int tileX, int tileZ)
{
    if (!path)
        return -1;
    char fileName[512];
    sprintf(fileName, "%s/map/map%d/navmesh/tile_%d_%d.cset", path, mapId, tileX, tileZ);
    FILE* const fp = fopen(fileName, "wb");
    if (!fp)
        return -2;
    AutoFileCloser closer(fp);
    if (!cset)
        return 0;
    CHECK_FWRITE(&cset->nconts);
    CHECK_FWRITE_N(cset->bmin, 3);
    CHECK_FWRITE_N(cset->bmax, 3);
    CHECK_FWRITE(&cset->cs);
    CHECK_FWRITE(&cset->ch);
    CHECK_FWRITE(&cset->width);
    CHECK_FWRITE(&cset->height);
    CHECK_FWRITE(&cset->borderSize);
    CHECK_FWRITE(&cset->maxError);
    for (int i = 0; i < cset->nconts; i++)
    {
        const rcContour& contour = cset->conts[i];
        CHECK_FWRITE(&contour.nverts);
        CHECK_FWRITE(&contour.nrverts);
        CHECK_FWRITE(&contour.reg);
        CHECK_FWRITE(&contour.area);
        CHECK_FWRITE_N(contour.verts, 4 * contour.nverts);
        CHECK_FWRITE_N(contour.rverts, 4 * contour.nrverts);
    }
    return 0;
}

#undef CHECK_FWRITE_N
#undef CHECK_FWRITE

class MiniJsonParser
{
    char* buf;
public:
    MiniJsonParser(const char* filename) : buf(0)
    {
        const int bufSize = readAllFile(filename, &buf);
        if (bufSize > 0)
            buf[bufSize - 1] = 0;
        else
        {
            delete[] buf;
            buf = 0;
        }
    }
    ~MiniJsonParser() { delete[] buf; }

    int parseIntField(const char* key, int def)
    {
        if (!buf) return def;
        const size_t n = strlen(key);
        for (const char* p = buf + 1; (p = strstr(p, key)) != 0; p += n)
            if (p[-1] == '"' && p[n] == '"')
                return (p = strchr(p + n + 1, ':')) != 0 ? atoi(p + 1) : def;
        return def;
    }

    float parseFloatField(const char* key, float def)
    {
        if (!buf) return def;
        const size_t n = strlen(key);
        for (const char* p = buf + 1; (p = strstr(p, key)) != 0; p += n)
            if (p[-1] == '"' && p[n] == '"')
                return (p = strchr(p + n + 1, ':')) != 0 ? static_cast<float>(atof(p + 1)) : def;
        return def;
    }
};

extern "C" NavStatus navBuildAllNavMesh(const char* path, int mapId, const DebugParam* debugParam)
{
    rcContext* const rcCtx = debugParam ? debugParam->ctx : 0;
    if (!path)
        return -1;
    if (mapId <= 0)
        return -2;
    char rcNavInputFileName[512];
    char rcNavMeshFileName[512];
    char worldTplFileName[512];
    char waterFileName[512];
    char roadFileName[512];
    sprintf(rcNavInputFileName, "%s/map/map%d/navmesh/%d.rcnavinput", path, mapId, mapId);
    sprintf(rcNavMeshFileName,  "%s/map/map%d/navmesh/%d.rcnavmesh",  path, mapId, mapId);
    sprintf(worldTplFileName,   "%s/map/map%d/world_template.json",   path, mapId);
    MiniJsonParser jp(worldTplFileName);
    const int sceneId = jp.parseIntField("sceneID", mapId / 10);
    sprintf(waterFileName,      "%s/scene/scene%d/water.bhmap",       path, sceneId);
    sprintf(roadFileName,       "%s/scene/scene%d/road.bhmap",        path, sceneId);

    char* rcNavInputBuf = 0;
    const int rcNavInputSize = readAllFile(rcNavInputFileName, &rcNavInputBuf);
    AutoArrayDeleter<char> deleter0(rcNavInputBuf);
    if (rcNavInputSize < 0)
    {
        if (rcCtx)
            rcCtx->log(RC_LOG_ERROR, "navBuildAllNavMesh: can not read file(%d): %s", rcNavInputSize, rcNavInputFileName);
        return rcNavInputSize - 100;
    }
    if (rcNavInputSize < 8)
        return -3;

    char* waterBuf = 0;
    const int waterSize = readAllFile(waterFileName, &waterBuf);
    AutoArrayDeleter<char> deleter1(waterBuf);
    if (waterSize != -3) // -3: file not found
    {
        if (waterSize < 0)
        {
            if (rcCtx)
                rcCtx->log(RC_LOG_ERROR, "navBuildAllNavMesh: can not read file(%d): %s", waterSize, waterFileName);
            return waterSize - 200;
        }
        const int r = BHMapConfig::checkData(waterBuf, waterSize);
        if (r < 0)
        {
            if (rcCtx)
                rcCtx->log(RC_LOG_ERROR, "navBuildAllNavMesh: checkData failed(%d): %s", r, waterFileName);
            return r - 300;
        }
    }

    char* roadBuf = 0;
    const int roadSize = readAllFile(roadFileName, &roadBuf);
    AutoArrayDeleter<char> deleter2(roadBuf);
    if (roadSize != -3) // -3: file not found
    {
        if (roadSize < 0)
        {
            if (rcCtx)
                rcCtx->log(RC_LOG_ERROR, "navBuildAllNavMesh: can not read file(%d): %s", roadSize, roadFileName);
            return roadSize - 400;
        }
        const int r = BHMapConfig::checkData(roadBuf, roadSize);
        if (r < 0)
        {
            if (rcCtx)
                rcCtx->log(RC_LOG_ERROR, "navBuildAllNavMesh: checkData failed(%d): %s", r, roadFileName);
            return r - 500;
        }
    }

    if (*reinterpret_cast<int*>(rcNavInputBuf) != RC_NAVINPUT_FILE_MAGIC_VERSION)
        return -4;
    const int meshCount = *reinterpret_cast<int*>(rcNavInputBuf + 4);
    struct MeshIndex
    {
        int offset;
        unsigned char minTileX;
        unsigned char minTileZ;
        unsigned char maxTileX;
        unsigned char maxTileZ;
    };
    const int headerSize = 4 + 4 + sizeof(MeshIndex) * meshCount + sizeof(rcConfig); // rcConfig: 92 bytes
    if (rcNavInputSize < headerSize)
        return -5;
    const MeshIndex* const meshIdxes = reinterpret_cast<MeshIndex*>(rcNavInputBuf + 4 + 4);
    const rcConfig& cfg = *reinterpret_cast<rcConfig*>(rcNavInputBuf + headerSize - sizeof(rcConfig));
    if (rcCtx)
    {
        rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: mapWidth=%d=%gm, mapHeight=%d=%gm",
            cfg.width, cfg.width* cfg.cs, cfg.height, cfg.height* cfg.cs);
        rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: ch=%gm, bmin={%g,%g,%g}",
            cfg.ch, cfg.bmin[0], cfg.bmin[1], cfg.bmin[2]);
        rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: cs=%gm, bmax={%g,%g,%g}",
            cfg.cs, cfg.bmax[0], cfg.bmax[1], cfg.bmax[2]);
    }
    const int tileSize = cfg.tileSize;
    if (tileSize <= 0 || cfg.cs <= 0)
        return -6;
    const int tileW = (cfg.width  + tileSize - 1) / tileSize;
    const int tileH = (cfg.height + tileSize - 1) / tileSize;
    const int tileCount = tileW * tileH;
    if (rcCtx)
        rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: tileSize=%d=%gm, tileCount=%d*%d=%d",
            tileSize, tileSize * cfg.cs, tileW, tileH, tileCount);
    if (tileCount > MAX_TILES)
        return -7;

    void* const mem = dtAlloc(sizeof(dtNavMeshEx), DT_ALLOC_PERM);
    if (!mem)
        return -8;
    dtNavMeshEx* const navMesh = new(mem) dtNavMeshEx;
    AutoDtFreer<dtNavMeshEx> freer0(navMesh);
    dtNavMeshParams params;
    rcVcopy(params.orig, cfg.bmin);
    params.tileWidth = params.tileHeight = tileSize * cfg.cs;
    params.maxTiles = tileCount;
    params.maxPolys = 1 << (32 - 10 - dtIlog2(dtNextPow2(static_cast<unsigned int>(tileCount))));
    if (rcCtx)
        rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: tileBits=%d, polyBits=%d, maxPolys=%d",
            dtIlog2(dtNextPow2(static_cast<unsigned int>(tileCount))), dtIlog2(params.maxPolys), params.maxPolys);
    const dtStatus s = navMesh->init(&params, cfg, tileW, tileH);
    if (dtStatusFailed(s))
        return NAVSTATUS_HIGH_BIT | s;

    std::vector<const void*>* const inputMeshOffsetLists = new std::vector<const void*>[tileCount];
    AutoArrayDeleter<std::vector<const void*>> deleter3(inputMeshOffsetLists);
    int minOffset = rcNavInputSize;
    for (int i = 0; i < meshCount; i++)
    {
        const MeshIndex& meshIdx = meshIdxes[i];
        const int xb = std::max(static_cast<int>(meshIdx.minTileX), 0);
        const int xe = std::min(static_cast<int>(meshIdx.maxTileX), tileW - 1);
        const int ze = std::min(static_cast<int>(meshIdx.maxTileZ), tileH - 1);
        for ( int z  = std::max(static_cast<int>(meshIdx.minTileZ), 0); z <= ze; z++)
        {
            const int idxBase = z * tileW;
            for (int x = xb; x <= xe; x++)
                inputMeshOffsetLists[idxBase + x].emplace_back(rcNavInputBuf + meshIdx.offset);
        }
        if (meshIdx.offset >= rcNavInputSize)
            return -9;
        if (minOffset > meshIdx.offset)
            minOffset = meshIdx.offset;
    }
    const int extraSize = minOffset - headerSize;
    const BuildParam* const bp = (extraSize > 0 ? reinterpret_cast<const BuildParam*>(rcNavInputBuf + headerSize) : 0);
    if (bp && (extraSize < static_cast<int>(sizeof(bp->buildParamSize)) || bp->buildParamSize > extraSize))
        return -10;
    if (rcCtx)
    {
        if (bp)
        {
            if (extraSize >= 0x04) rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: bp.buildParamSize           = %d", bp->buildParamSize);
            if (extraSize >= 0x08) rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: bp.agentRadius              = %g", bp->agentRadius);
            if (extraSize >= 0x0C) rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: bp.agentHeight              = %g", bp->agentHeight);
            if (extraSize >= 0x10) rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: bp.agentMaxClimb            = %g", bp->agentMaxClimb);
            if (extraSize >= 0x14) rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: bp.jumpNearDistance         = %g", bp->jumpNearDistance);
            if (extraSize >= 0x18) rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: bp.jumpMediumDistance       = %g", bp->jumpMediumDistance);
            if (extraSize >= 0x1C) rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: bp.jumpFarDistance          = %g", bp->jumpFarDistance);
            if (extraSize >= 0x20) rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: bp.waterAreaMaxHeight       = %g", bp->waterAreaMaxHeight);
            if (extraSize >= 0x24) rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: bp.roadAreaMinWeight        = %g", bp->roadAreaMinWeight);
            if (extraSize >= 0x28) rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: bp.minUndergroundRegionArea = %g", bp->minUndergroundRegionArea);
            if (extraSize >= 0x34) rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: bp.mapBoundsMin             = {%g,%g,%g}", bp->mapBoundsMin[0], bp->mapBoundsMin[1], bp->mapBoundsMin[2]);
        }
        else
            rcCtx->log(RC_LOG_PROGRESS, "navBuildAllNavMesh: not found BuildParam, use default values");
    }

    NavStatus ns;
    dtNavMeshQueryEx* navQuery = 0;
    ns = navAllocNavQuery(navMesh, 2048, &navQuery);
    AutoDtFreer<dtNavMeshQueryEx> freer1(navQuery);
    if (ns < 0)
        return ns >= -99 ? ns - 600 : (ns | 1LL << 48);
    if (rcCtx)
        rcCtx->startTimer(RC_TIMER_TOTAL);
    AutoStopTimer defer1(rcCtx, RC_TIMER_TOTAL);
/*
    Defer defer1([&]()
    {
        if (rcCtx)
            rcCtx->stopTimer(RC_TIMER_TOTAL);
    });
*/
    int beginTileX, beginTileZ, endTileX, endTileZ;
    if (debugParam && debugParam->onlyBuildSomeTiles)
    {
        beginTileX = std::max(debugParam->beginTileX - 1, -1);
        beginTileZ = std::max(debugParam->beginTileZ - 1, -1);
        endTileX   = std::min(debugParam->endTileX + 1, tileW);
        endTileZ   = std::min(debugParam->endTileZ + 1, tileH);
    }
    else
    {
        beginTileX = -1;
        beginTileZ = -1;
        endTileX   = tileW;
        endTileZ   = tileH;
    }
    for (int tileZ = beginTileZ; tileZ <= endTileZ; tileZ++)
    {
        for (int tileX = beginTileX; tileX <= endTileX; tileX++)
        {
            int tx = tileX + 1, tz = tileZ + 1;
            if (beginTileX < tx && tx < endTileX && beginTileZ < tz && tz < endTileZ)
            {
                const clock_t t = clock();
                const std::vector<const void*>& inputMeshOffsetList = inputMeshOffsetLists[tz * tileW + tx];
                unsigned char* navTileData = 0;
                int navTileSize = 0;
                ns = navBuildTileBegin(navMesh, bp, rcCtx, tx, tz, inputMeshOffsetList.data(),
                    0, static_cast<int>(inputMeshOffsetList.size()), waterBuf, roadBuf, &navTileData, &navTileSize);
                if (ns < 0)
                    return ns - 700;
                if (navTileSize > 0)
                    ns = navReplaceTile(navMesh, navTileData, navTileSize);
                else
                    ns = navRemoveTile(navMesh, tx, tz);
                if (ns < 0)
                {
                    dtFree(navTileData);
                    return ns >= -99 ? ns - (navTileSize > 0 ? 800 : 900) : (ns | (2LL << 48));
                }
                if (rcCtx)
                    rcCtx->log(static_cast<rcLogCategory>(RC_LOG_TRACE), "navBuildTileBegin:%3d,%3d (%3d ms) (%d bytes)",
                        tx, tz, static_cast<int>((clock() - t) * 1000 / CLOCKS_PER_SEC), navTileSize);
            }
            if (beginTileX < tileX && tileX < endTileX && beginTileZ < tileZ && tileZ < endTileZ)
            {
                const clock_t t = clock();
                unsigned char* navTileData = 0;
                int navTileSize = 0;
                if (rcCtx)
                    rcCtx->startTimer(static_cast<rcTimerLabel>(RC_TIMER_AUTO_OFFMESH_LINK));
                ns = navBuildTileLink(navMesh, navQuery, bp, rcCtx, tileX, tileZ, &navTileData, &navTileSize);
                if (rcCtx)
                    rcCtx->stopTimer(static_cast<rcTimerLabel>(RC_TIMER_AUTO_OFFMESH_LINK));
                if (ns < 0)
                    return ns - 1000;
                if (navTileData)
                {
                    if (navTileSize > 0)
                        ns = navReplaceTile(navMesh, navTileData, navTileSize);
                    else
                        ns = navRemoveTile(navMesh, tileX, tileZ);
                    if (ns < 0)
                    {
                        dtFree(navTileData);
                        return ns >= -99 ? ns - (navTileSize > 0 ? 1100 : 1200) : (ns | (3LL << 48));
                    }
                }
                if (rcCtx)
                    rcCtx->log(static_cast<rcLogCategory>(RC_LOG_TRACE), "navBuildTileLink :%3d,%3d (%3d ms) (%d bytes)",
                        tileX, tileZ, static_cast<int>((clock() - t) * 1000 / CLOCKS_PER_SEC), navTileSize);
            }
            tx = tileX - 1, tz = tileZ - 1;
            if (beginTileX < tx && tx < endTileX && beginTileZ < tz && tz < endTileZ)
            {
                if (debugParam && debugParam->beginTileX <= tx && tx <= debugParam->endTileX
                               && debugParam->beginTileZ <= tz && tz <= debugParam->endTileZ)
                {
                    const TileCtx* const tileCtx = navMesh->getTileCtx(tx, tz);
                    if (debugParam->saveHeightfield)
                    {
                        const int r = saveHeightfield(path, mapId, tileCtx ? tileCtx->compactHeightfield : 0, tx, tz);
                        if (r)
                            return r - 1300;
                    }
                    if (debugParam->saveContourSet)
                    {
                        const int r = saveContourSet(path, mapId, tileCtx ? tileCtx->contourSet : 0, tx, tz);
                        if (r)
                            return r - 1400;
                    }
                }
                navBuildTileEnd(navMesh, tx, tz);
            }
        }
    }
    if (rcCtx)
        rcCtx->startTimer(static_cast<rcTimerLabel>(RC_TIMER_FIX_POLY_FLAGS));
    int totalVertCount = 0;
    int totalPolyCount = 0;
    int totalLinkCount = 0;
    int totalDetailMeshCount = 0;
    int totalDetailVertCount = 0;
    int totalDetailTriCount = 0;
    int totalBvNodeCount = 0;
    int totalOffMeshConCount = 0;
    for (int tileZ = 0; tileZ < tileH; ++tileZ)
    {
        for (int tileX = 0; tileX < tileW; ++tileX)
        {
            ns = navFixTile(navMesh, bp, tileX, tileZ);
            if (ns && rcCtx)
                rcCtx->log(static_cast<rcLogCategory>(RC_LOG_TRACE), "navFixTile:%3d,%3d (%lld)", tileX, tileZ, static_cast<long long>(ns));
            const dtMeshTile* const tile = navMesh->getTileAt(tileX, tileZ, 0);
            if (tile && tile->header)
            {
                totalVertCount       += tile->header->vertCount;
                totalPolyCount       += tile->header->polyCount;
                totalLinkCount       += tile->header->maxLinkCount;
                totalDetailMeshCount += tile->header->detailMeshCount;
                totalDetailVertCount += tile->header->detailVertCount;
                totalDetailTriCount  += tile->header->detailTriCount;
                totalBvNodeCount     += tile->header->bvNodeCount;
                totalOffMeshConCount += tile->header->offMeshConCount;
            }
        }
    }
    if (rcCtx)
    {
        rcCtx->stopTimer(static_cast<rcTimerLabel>(RC_TIMER_FIX_POLY_FLAGS));
        rcCtx->log(static_cast<rcLogCategory>(RC_LOG_TRACE), "totalVertCount       =%7d * 12 =%6d KB", totalVertCount,       totalVertCount       * 12 / 1024);
        rcCtx->log(static_cast<rcLogCategory>(RC_LOG_TRACE), "totalPolyCount       =%7d * 32 =%6d KB", totalPolyCount,       totalPolyCount       * 32 / 1024);
        rcCtx->log(static_cast<rcLogCategory>(RC_LOG_TRACE), "totalLinkCount       =%7d * 12 =%6d KB", totalLinkCount,       totalLinkCount       * 12 / 1024);
        rcCtx->log(static_cast<rcLogCategory>(RC_LOG_TRACE), "totalDetailMeshCount =%7d * 12 =%6d KB", totalDetailMeshCount, totalDetailMeshCount * 12 / 1024);
        rcCtx->log(static_cast<rcLogCategory>(RC_LOG_TRACE), "totalDetailVertCount =%7d * 12 =%6d KB", totalDetailVertCount, totalDetailVertCount * 12 / 1024);
        rcCtx->log(static_cast<rcLogCategory>(RC_LOG_TRACE), "totalDetailTriCount  =%7d *  4 =%6d KB", totalDetailTriCount,  totalDetailTriCount  *  4 / 1024);
        rcCtx->log(static_cast<rcLogCategory>(RC_LOG_TRACE), "totalBvNodeCount     =%7d * 16 =%6d KB", totalBvNodeCount,     totalBvNodeCount     * 16 / 1024);
        rcCtx->log(static_cast<rcLogCategory>(RC_LOG_TRACE), "totalOffMeshConCount =%7d * 36 =%6d KB", totalOffMeshConCount, totalOffMeshConCount * 36 / 1024);
    }

    defer1.close();
    if (debugParam && debugParam->saveDebugFile)
    {
        const int r = saveFullPolyMeshDebugData(rcNavMeshFileName, *navMesh, tileW, tileH);
        if (r)
            return r - 1500;
    }
    FILE* const fp = fopen(rcNavMeshFileName, "wb");
    if (!fp)
        return -11;
    ns = navSaveNavMesh(navMesh, fp);
    if (rcCtx)
    {
        const int totalSize =
            totalVertCount       * 12 +
            totalPolyCount       * 32 +
            totalLinkCount       * 12 +
            totalDetailMeshCount * 12 +
            totalDetailVertCount * 12 +
            totalDetailTriCount  *  4 +
            totalBvNodeCount     * 16 +
            totalOffMeshConCount * 36;
        rcCtx->log(static_cast<rcLogCategory>(RC_LOG_TRACE), "totalSize =%7d KB", totalSize / 1024);
        rcCtx->log(static_cast<rcLogCategory>(RC_LOG_TRACE), " fileSize =%7d KB", static_cast<int>(ftell(fp) / 1024));
    }
    fclose(fp);
    if (ns < 0)
        return ns - 1600;
    return 0;
}

extern "C" NavStatus navBuildNavMesh(dtNavMeshCreateParams* params, unsigned char** outNavMeshData, int* outNavMeshDataSize)
{
    if (!params)
        return -1;
    if (!outNavMeshData)
        return -2;
    if (!outNavMeshDataSize)
        return -3;
    const bool r = dtCreateNavMeshData(params, outNavMeshData, outNavMeshDataSize);
    if (!r)
        return -4;
    return 0;
}

extern "C" NavStatus navCreateNavMesh(const dtNavMeshParams* params, dtNavMesh** outNavMesh)
{
    if (!params)
        return -1;
    if (!outNavMesh)
        return -2;
    void* const mem = dtAlloc(sizeof(dtNavMesh), DT_ALLOC_PERM);
    if (!mem)
        return -3;
    dtNavMesh* const navMesh = new(mem) dtNavMesh;
    const dtStatus s = navMesh->init(params);
    if (dtStatusFailed(s))
    {
        navMesh->~dtNavMesh();
        dtFree(navMesh);
        return NAVSTATUS_HIGH_BIT | s;
    }
    *outNavMesh = navMesh;
    return 0;
}

extern "C" void navDestroyNavMesh(dtNavMesh* navMesh)
{
    if (!navMesh)
        return;
    navMesh->~dtNavMesh();
    dtFree(navMesh);
}

extern "C" NavStatus navDumpNavMesh(const dtNavMesh* navMesh, FILE* fp)
{
    if (!navMesh)
        return -1;
    if (!fp)
        return -2;

    int64_t r = 0;
    for (int i = 0, n = navMesh->getMaxTiles(); i < n; i++)
    {
        const dtMeshTile* const tile = navMesh->getTile(i);
        if (tile && tile->header)
        {
            if (fwrite(&tile->dataSize, sizeof(tile->dataSize), 1, fp) != 1)
                return -3;
            if (fwrite(tile->data, 1, static_cast<size_t>(tile->dataSize), fp) != static_cast<size_t>(tile->dataSize))
                return -4;
            r++;
        }
    }
    return r;
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif
