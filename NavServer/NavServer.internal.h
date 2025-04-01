#ifndef NAVSERVER_INTERNAL_H
#define NAVSERVER_INTERNAL_H

#include <string.h>
#include <cmath>
#include <vector>
#include <deque>
#include <algorithm>
#include "Recast.h"
#include "RecastAlloc.h"
#include "DetourNode.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "NavServer.h"

constexpr NavStatus NAVSTATUS_HIGH_BIT = static_cast<NavStatus>(1) << (sizeof(NavStatus) * 8 - 1);

constexpr int RC_NAVMESH_FILE_MAGIC_VERSION  = 1;    // from PathfindingSystem::RC_NAVMESH_FILE_MAGIC_VERSION
constexpr int RC_NAVINPUT_FILE_MAGIC_VERSION = 1;    // from RecastInputData::RC_NAVINPUT_FILE_MAGIC_VERSION
constexpr int MAX_TILES                      = 4096; // from PathfindingSystem.cpp
constexpr int FIND_PATH_MAX_POLYS            = 4096; // see PathfindingSystem::MAX_POLYS

constexpr int BOX_TRIS[36] = { 0, 1, 2, 0, 2, 3, 1, 4, 5, 1, 5, 2, 4, 6, 7, 4, 7, 5, 6, 0, 3, 6, 3, 7, 3, 2, 5, 3, 5, 7, 6, 4, 1, 6, 1, 0 };

struct BuildParam
{
    const int buildParamSize;       // [+00]构建参数结构体的字节大小(包括此字段),用于兼容后续扩展
    float agentRadius;              // [+04]移动单位的半径(米) 默认: 0.25
    float agentHeight;              // [+08]移动单位的高度(米) 默认: 2.0
    float agentMaxClimb;            // [+0C]移动单位移动高度差最大值(米) 默认: 0.6
    float jumpNearDistance;         // [+10]小桥最大距离(米) 默认: 2.0
    float jumpMediumDistance;       // [+14]中桥最大距离(米) 默认: 4.0
    float jumpFarDistance;          // [+18]大桥最大距离(米) 默认: 6.0
    float waterAreaMaxHeight;       // [+1C]可移动的水域最大深度(米) 默认: 0.7
    float roadAreaMinWeight;        // [+20]道路区域判定最小权重 默认: 0.8
    float minUndergroundRegionArea; // [+24]无露天的孤立空间允许的最小面积(平方米)(小于此面积会在构建时把相关poly的flags清0) 默认: 1000
    float mapBoundsMin[3];          // [+28]地图左下角的基准坐标(x,y,z)(米) 默认NaN. 该坐标表示x和z的最小值,其中y暂无意义
    // [+34]

    BuildParam(const void* buildParam = 0) : buildParamSize(sizeof(BuildParam)),
        agentRadius(0.25f),
        agentHeight(2.0f),
        agentMaxClimb(0.6f),
        jumpNearDistance(2.0f),
        jumpMediumDistance(4.0f),
        jumpFarDistance(6.0f),
        waterAreaMaxHeight(0.7f),
        roadAreaMinWeight(0.8f),
        minUndergroundRegionArea(1000.0f),
        mapBoundsMin{NAN, NAN, NAN}
    {
        const BuildParam* const bp = static_cast<const BuildParam*>(buildParam);
        if (bp)
            memcpy(&agentRadius, &bp->agentRadius, std::min(static_cast<size_t>(bp->buildParamSize), sizeof(BuildParam)) - sizeof(buildParamSize));
    }
};

struct OffMeshLink
{
    const float sx, sy, sz; // startPosition
    const float tx, ty, tz; // endPosition
    const int stx; // sourceTileX
    const int stz; // sourceTileZ
    const int ttx; // targetTileX
    const int ttz; // targetTileZ

    OffMeshLink(const float* s, const float* t, int _stx, int _stz, int _ttx, int _ttz) :
        sx(s[0]), sy(s[1]), sz(s[2]), tx(t[0]), ty(t[1]), tz(t[2]),
        stx(_stx), stz(_stz), ttx(_ttx), ttz(_ttz) {}

    OffMeshLink(const dtOffMeshConnection& conn, int _stx, int _stz, int _ttx, int _ttz) :
        sx(conn.pos[0]), sy(conn.pos[1]), sz(conn.pos[2]),
        tx(conn.pos[3]), ty(conn.pos[4]), tz(conn.pos[5]),
        stx(_stx), stz(_stz), ttx(_ttx), ttz(_ttz) {}

    OffMeshLink& operator=(const OffMeshLink& o) // for gcc 4.4
    {
        memcpy(this, &o, sizeof(*this));
        return *this;
    }
};

struct TileCtx
{
    std::vector<OffMeshLink> neighbourTileOffMeshLinks;
    std::vector<OffMeshLink> offMeshLinks;
    rcHeightfield* heightField;
    rcPolyMesh* polyMesh;
    rcPolyMeshDetail* detailMesh;
    rcCompactHeightfield* compactHeightfield;
    rcContourSet* contourSet;

    TileCtx() : heightField(0), polyMesh(0), detailMesh(0), compactHeightfield(0), contourSet(0) {}
    ~TileCtx() { freeMesh(); }

    void freeMesh()
    {
        if (contourSet)
        {
            contourSet->~rcContourSet();
            rcFree(contourSet);
            contourSet = 0;
        }
        if (compactHeightfield)
        {
            compactHeightfield->~rcCompactHeightfield();
            rcFree(compactHeightfield);
            compactHeightfield = 0;
        }
        if (detailMesh)
        {
            rcFreePolyMeshDetail(detailMesh);
            detailMesh = 0;
        }
        if (polyMesh)
        {
            rcFreePolyMesh(polyMesh);
            polyMesh = 0;
        }
        if (heightField)
        {
            rcFreeHeightField(heightField);
            heightField = 0;
        }
    }
};

class dtNavMeshEx : public dtNavMesh
{
    rcConfig cfg;
    int tileW, tileH;
    float tileSizeInv;
    TileCtx* tiles;

    dtNavMeshEx(const dtNavMeshEx&) = delete;
    dtNavMeshEx& operator=(const dtNavMeshEx&) = delete;
public:
    mutable std::vector<OffMeshLink> tempLinks;
    mutable std::deque<dtPolyRef> tempQueue;

    dtNavMeshEx() : tileW(0), tileH(0), tileSizeInv(0), tiles(0)
    {
        memset(&cfg, 0, sizeof(cfg));
    }

    ~dtNavMeshEx()
    {
        if (tiles)
        {
            for (int i = tileW * tileH; i > 0;)
                tiles[--i].~TileCtx();
            dtFree(tiles);
        }
    }

    dtStatus init(const dtNavMeshParams* params, const rcConfig& rcCfg, int tw, int th)
    {
        const dtStatus s = dtNavMesh::init(params);
        if (dtStatusFailed(s))
            return s;
        memcpy(&cfg, &rcCfg, sizeof(cfg));
        tileW = tw;
        tileH = th;
        tileSizeInv = 1 / (rcCfg.tileSize * rcCfg.cs);
        const int tileCount = tw * th;
        tiles = static_cast<TileCtx*>(dtAlloc(sizeof(TileCtx) * tileCount, DT_ALLOC_PERM));
        if (!tiles)
            return DT_FAILURE | DT_OUT_OF_MEMORY | (255u << 16);
        for (int i = 0; i < tileCount; ++i)
            new(tiles + i) TileCtx;
        return s;
    }

    dtStatus init(const dtNavMeshEx& navMesh)
    {
        return init(navMesh.getParams(), navMesh.cfg, navMesh.tileW, navMesh.tileH);
    }

    void initLink(int x, int z)
    {
        TileCtx* const tileCtx = getTileCtx(x, z);
        if (!tileCtx)
            return;
        tileCtx->neighbourTileOffMeshLinks.clear();
        const dtMeshTile* const tile = getTileByRef(getTileRefAt(x, z, 0));
        if (!tile || !tile->header)
            return;
        for (int i = 0, n = tile->header->offMeshConCount; i < n; i++)
        {
            const dtOffMeshConnection& conn = tile->offMeshCons[i];
            const int stx = getTileX(conn.pos[0]);
            const int stz = getTileZ(conn.pos[2]);
            const int ttx = getTileX(conn.pos[3]);
            const int ttz = getTileZ(conn.pos[5]);
            if (stx != ttx || stz != ttz)
                tileCtx->neighbourTileOffMeshLinks.emplace_back(conn, stx, stz, ttx, ttz);
        }
    }

    const rcConfig& getCfg() const { return cfg; }
    int getTileW() const { return tileW; }
    int getTileH() const { return tileH; }

    int getTileX(float x) const
    {
        return static_cast<int>(floorf((x - cfg.bmin[0]) * tileSizeInv));
    }

    int getTileZ(float z) const
    {
        return static_cast<int>(floorf((z - cfg.bmin[2]) * tileSizeInv));
    }

    TileCtx* getTileCtx(int x, int z)
    {
        if (static_cast<unsigned>(x) >= static_cast<unsigned>(tileW) || static_cast<unsigned>(z) >= static_cast<unsigned>(tileH))
            return 0;
        return tiles + z * tileW + x;
    }

    const TileCtx* getTileCtx(int x, int z) const
    {
        if (static_cast<unsigned>(x) >= static_cast<unsigned>(tileW) || static_cast<unsigned>(z) >= static_cast<unsigned>(tileH))
            return 0;
        return tiles + z * tileW + x;
    }
};

class dtNavMeshQueryEx : public dtNavMeshQuery, public dtQueryFilter
{
    mutable dtNodeQueue openList;
    float halfExtents[3];

    dtNavMeshQueryEx(const dtNavMeshQueryEx&) = delete;
    dtNavMeshQueryEx& operator=(const dtNavMeshQueryEx&) = delete;
public:
    explicit dtNavMeshQueryEx(int maxNodes) : openList(maxNodes) {
        halfExtents[0] = 10;
        halfExtents[1] = 1000;
        halfExtents[2] = 10;
    }
    dtNodeQueue& getOpenList() const { return openList; }
    const float* getHalfExtents() const { return halfExtents; }
    void setHalfExtents(float x, float y, float z) {
        halfExtents[0] = x;
        halfExtents[1] = y;
        halfExtents[2] = z;
    }
    dtStatus findNearestPoly(const float* center, const float* halfExtents,
        const dtQueryFilter* filter, dtPolyRef* nearestRef, float* nearestPt, int method = 0) const;
};

enum
{
    kHMapHeaderMinSize = 0x28,
    kHMapHeaderVersion = 1,
    kBHMapHeaderMinSize = 0x30,
    kBHMapHeaderVersion = 1,
    kBlockIndexRange = 0x4000,
    kInvalidValueKey = 0xffff,
};

struct HMapHeader // ark_resource/resource/develop/server/scene/scene*/*.hmap
{
    int headerSize; // 文件头大小(包括此字段,用于保持前后兼容,目前大小至少是0x28)
    int version; // 文件格式版本号(目前只用1)
    int resolutionX; // 网格数据在X轴方向的采样数量
    int resolutionZ; // 网格数据在Z轴方向的采样数量
    float sizeX; // 网格映射区域在X轴方向的尺寸(米)
    float sizeZ; // 网格映射区域在Z轴方向的尺寸(米)
    float minX; // 网格映射区域在X轴方向的最小位置(米)
    float minZ; // 网格映射区域在Z轴方向的最小位置(米)
    float maxValue; // 网格采样数据的最大值(如最大高度(米),最大权重值等)
    float minValue; // 网格采样数据的最小值(如最小高度(米),最小权重值等)
    // unsigned short allData[resolutionX*resolutionZ]; // 所有的采样数据. 外层从下到上(z轴从小到大),内层从左到右(x轴从小到大)

    HMapHeader(const char* data)
    {
        const HMapHeader* header = reinterpret_cast<const HMapHeader*>(data);
        const size_t dataHeaderSize = static_cast<size_t>(header->headerSize);
        if (dataHeaderSize < sizeof(*this))
        {
            memset(this, 0, sizeof(*this));
            memcpy(this, header, dataHeaderSize);
        }
        else
            memcpy(this, header, sizeof(*this));
    }
};

struct HMapConfig
{
    const HMapHeader header;
    const float resFactorX;
    const float resFactorZ;
    const unsigned short* const mapData;
    const float unitY;

    HMapConfig(const HMapConfig&) = delete;
    HMapConfig& operator=(const HMapConfig&) = delete;

    // data是*.hmap文件的完整数据,HMapConfig会持有data指针,在销毁HMapConfig之前不能释放data
    HMapConfig(const char* data) :
        header(data),
        resFactorX((header.resolutionX - 1) / header.sizeX),
        resFactorZ((header.resolutionZ - 1) / header.sizeZ),
        mapData(reinterpret_cast<const unsigned short*>(data + header.headerSize)),
        unitY((header.maxValue - header.minValue) / 0xffff) // [0,0xffff]
    {
    }

    // 校验*.hmap文件数据的合法性. 返回0表示校验成功,否则返回负值
    static int checkData(const char* data, size_t size) // return 0 for OK
    {
        if (size < kHMapHeaderMinSize)
            return -1;
        const HMapHeader* const header = reinterpret_cast<const HMapHeader*>(data);
        if (header->headerSize < kHMapHeaderMinSize)
            return -2;
        if (header->version != kHMapHeaderVersion)
            return -3;
        if (!std::isfinite(header->sizeX) ||
            !std::isfinite(header->sizeZ) ||
            !std::isfinite(header->minX) ||
            !std::isfinite(header->minZ) ||
            !std::isfinite(header->minValue) ||
            !std::isfinite(header->maxValue))
            return -4;
        if (header->resolutionX <= 0 ||
            header->resolutionZ <= 0)
            return -5;
        if (static_cast<unsigned>(header->resolutionX) > 0x8000 || // 尺寸不能太大,避免resolutionX*resolutionZ溢出
            static_cast<unsigned>(header->resolutionZ) > 0x8000)
            return -8;
        if (header->sizeX < 0 ||
            header->sizeZ < 0)
            return -6;
        if (header->maxValue < header->minValue)
            return -9;
        if (size != header->headerSize + sizeof(unsigned short) * header->resolutionX * header->resolutionZ)
            return -12;
        return 0;
    }

    int getSampleValue(int x, int z) const
    {
        if (x < 0)
            x = 0;
        if (x > header.resolutionX - 1)
            x = header.resolutionX - 1;
        if (z < 0)
            z = 0;
        if (z > header.resolutionZ - 1)
            z = header.resolutionZ - 1;
        return mapData[z * header.resolutionX + x];
    }

    // 传入世界坐标,返回该点的地形高度值
    float getValue(float x, float z) const
    {
        x = (x - header.minX) * resFactorX;
        z = (z - header.minZ) * resFactorZ;
        const int ix = static_cast<int>(floor(x));
        const int iz = static_cast<int>(floor(z));
        // y10+-+y11
        //    |/|
        // y00+-+y01
        const int y00 = getSampleValue(ix, iz);
        const int y11 = getSampleValue(ix + 1, iz + 1);
        if ((x -= ix) > (z -= iz)) // right-bottom
        {
            const int y01 = getSampleValue(ix + 1, iz);
            return (y00 + (y01 - y00) * x + (y11 - y01) * z) * unitY + header.minValue;
        } // left-top
        const int y10 = getSampleValue(ix, iz + 1);
        return (y00 + (y11 - y10) * x + (y10 - y00) * z) * unitY + header.minValue;
    }
};

struct BHMapHeader // ark_resource/resource/develop/server/scene/scene*/*.bhmap
{
    int headerSize; // 文件头大小(包括此字段,用于保持前后兼容,目前大小至少是0x30)
    int version; // 文件格式版本号(目前只用1)
    int resolutionX; // 网格数据在X轴方向的采样数量
    int resolutionZ; // 网格数据在Z轴方向的采样数量
    float sizeX; // 网格映射区域在X轴方向的尺寸(米)
    float sizeZ; // 网格映射区域在Z轴方向的尺寸(米)
    float minX; // 网格映射区域在X轴方向的最小位置(米)
    float minZ; // 网格映射区域在Z轴方向的最小位置(米)
    float maxValue; // 网格采样数据的最大值(如最大高度(米),最大权重值等)
    float minValue; // 网格采样数据的最小值(如最小高度(米),最小权重值等)
    float invalidValue; // 无效值(有时也用于默认值)
    int blockSizeShift; // 每块采样数量的移位值(即每块的X轴和Z轴方向都有1<<blockSizeShift个采样数量)
    // unsigned short idxData[blockCountX*blockCountZ]; // 各分块的索引/单一采样值/默认值. 分块排列是外层从下到上(z轴从小到大),内层从左到右(x轴从小到大)
    // unsigned short blockData[blockDataCount<<(blockSizeShift*2)]; // 各分块的采样数据. 每块的采样数据排列都是外层从下到上(z轴从小到大),内层从左到右(x轴从小到大)

    BHMapHeader(const char* data)
    {
        const BHMapHeader* header = reinterpret_cast<const BHMapHeader*>(data);
        const size_t dataHeaderSize = static_cast<size_t>(header->headerSize);
        if (dataHeaderSize < sizeof(*this))
        {
            memset(this, 0, sizeof(*this));
            memcpy(this, header, dataHeaderSize);
        }
        else
            memcpy(this, header, sizeof(*this));
    }
};

struct BHMapConfig
{
    const BHMapHeader header;
    const int blockCountX; // 网格数据在X轴方向的分块数量(等于resolutionX>>blockSizeShift向上取整)
    const int blockCountZ; // 网格数据在Z轴方向的分块数量(等于resolutionZ>>blockSizeShift向上取整)
    const float resFactorX;
    const float resFactorZ;
    const unsigned short* const idxData;
    const unsigned short* const blockData;
    const float idxUnitY;
    const float blockUnitY;

    BHMapConfig(const BHMapConfig&) = delete;
    BHMapConfig& operator=(const BHMapConfig&) = delete;

    // data是*.bhmap文件的完整数据,BHMapHeader会持有data指针,在销毁BHMapHeader之前不能释放data
    BHMapConfig(const char* data) :
        header(data),
        blockCountX((header.resolutionX + (1 << header.blockSizeShift) - 1) >> header.blockSizeShift),
        blockCountZ((header.resolutionZ + (1 << header.blockSizeShift) - 1) >> header.blockSizeShift),
        resFactorX((header.resolutionX - 1) / header.sizeX),
        resFactorZ((header.resolutionZ - 1) / header.sizeZ),
        idxData(reinterpret_cast<const unsigned short*>(data + header.headerSize)),
        blockData(reinterpret_cast<const unsigned short*>(data + header.headerSize + sizeof(unsigned short) * blockCountX * blockCountZ)),
        idxUnitY((header.maxValue - header.minValue) / (kInvalidValueKey - kBlockIndexRange - 1)), // [0x4000,0xfffe]
        blockUnitY((header.maxValue - header.minValue) / (kInvalidValueKey - 1)) // [0,0xfffe]
    {
    }

    // 校验*.bhmap文件数据的合法性. 返回0表示校验成功,否则返回负值
    static int checkData(const char* data, size_t size) // return 0 for OK
    {
        if (size < kBHMapHeaderMinSize)
            return -1;
        const BHMapHeader* const header = reinterpret_cast<const BHMapHeader*>(data);
        if (header->headerSize < kBHMapHeaderMinSize)
            return -2;
        if (header->version != kBHMapHeaderVersion)
            return -3;
        if (!std::isfinite(header->sizeX) ||
            !std::isfinite(header->sizeZ) ||
            !std::isfinite(header->minX) ||
            !std::isfinite(header->minZ) ||
            !std::isfinite(header->minValue) ||
            !std::isfinite(header->maxValue) ||
            !std::isfinite(header->invalidValue))
            return -4;
        if (header->resolutionX <= 0 ||
            header->resolutionZ <= 0)
            return -5;
        if (header->sizeX < 0 ||
            header->sizeZ < 0)
            return -6;
        if (static_cast<unsigned>(header->blockSizeShift) > 16)
            return -7;
        const int blockCountX = (header->resolutionX + (1 << header->blockSizeShift) - 1) >> header->blockSizeShift;
        const int blockCountZ = (header->resolutionZ + (1 << header->blockSizeShift) - 1) >> header->blockSizeShift;
        if (static_cast<unsigned>(blockCountX) > 0x8000 || // 分块宽不能太大,避免blockCountX*blockCountZ溢出
            static_cast<unsigned>(blockCountZ) > 0x8000)
            return -8;
        if (header->maxValue < header->minValue)
            return -9;
        if (header->minValue <= header->invalidValue && header->invalidValue <= header->maxValue && header->minValue != header->maxValue)
            return -10;
        const int blockCount = blockCountX * blockCountZ;
        if (size < header->headerSize + sizeof(unsigned short) * blockCount)
            return -11;
        const unsigned short* const idxData = reinterpret_cast<const unsigned short*>(data + header->headerSize);
        int blockDataCount = 0;
        for (int i = 0; i < blockCount; i++)
        {
            const int block = idxData[i];
            if (block < kBlockIndexRange && block >= blockDataCount)
                blockDataCount = block + 1;
        }
        if (size != header->headerSize + sizeof(unsigned short) *
            (blockCount + (static_cast<unsigned long long>(blockDataCount) << (header->blockSizeShift * 2))))
            return -12;
        return 0;
    }

    float getSampleValue(int x, int z) const
    {
        if (x < 0)
            x = 0;
        if (x > header.resolutionX - 1)
            x = header.resolutionX - 1;
        if (z < 0)
            z = 0;
        if (z > header.resolutionZ - 1)
            z = header.resolutionZ - 1;
        const int inBlockMask = (1 << header.blockSizeShift) - 1;
        const int bx = x >> header.blockSizeShift;
        const int bz = z >> header.blockSizeShift;
        const int block = idxData[bz * blockCountX + bx];
        if (block < kBlockIndexRange)
        {
            const int xInBlock = x & inBlockMask;
            const int zInBlock = z & inBlockMask;
            const int v = blockData[(block << (header.blockSizeShift * 2)) + (zInBlock << header.blockSizeShift) + xInBlock];
            return v != kInvalidValueKey ? header.minValue + blockUnitY * v : header.invalidValue;
        }
        return block != kInvalidValueKey ? header.minValue + idxUnitY * (block - kBlockIndexRange) : header.invalidValue;
    }

    // 传入世界坐标,返回该点的高度/权重值,其中invalidValue表示无效值(可调getHeader().invalidValue获得)
    float getValue(float x, float z) const
    {
        x = (x - header.minX) * resFactorX;
        z = (z - header.minZ) * resFactorZ;
        const int ix = static_cast<int>(floor(x));
        const int iz = static_cast<int>(floor(z));
        // y10+-+y11
        //    |/|
        // y00+-+y01
        float y00, y01, y10, y11;
        const float invalidValue = header.invalidValue;
        if ((y00 = getSampleValue(ix, iz)) == invalidValue)
            return invalidValue;
        if ((y11 = getSampleValue(ix + 1, iz + 1)) == invalidValue)
            return invalidValue;
        if ((x -= ix) > (z -= iz)) // right-bottom
        {
            if ((y01 = getSampleValue(ix + 1, iz)) == invalidValue)
                return invalidValue;
            return y00 + (y01 - y00) * x + (y11 - y01) * z;
        } // left-top
        if ((y10 = getSampleValue(ix, iz + 1)) == invalidValue)
            return invalidValue;
        return y00 + (y11 - y10) * x + (y10 - y00) * z;
    }

    // 传入世界坐标,返回该点的高度/权重值,如果结果是无效值,则返回defValue参数值
    float getValue(float x, float z, float defValue) const
    {
        const float v = getValue(x, z);
        return v != header.invalidValue ? v : defValue;
    }
};

void* recastRcAlloc(size_t size, rcAllocHint);
void* recastDtAlloc(size_t size, dtAllocHint);
void recastFree(void* ptr);
float frand();

NavStatus createNavMesh(const rcConfig& cfg, const BuildParam& bp, const rcPolyMesh* pmesh, const rcPolyMeshDetail* dmesh,
    const std::vector<OffMeshLink>& offMeshLinks, int tileX, int tileZ, unsigned char** outNavTileData, int* outNavTileDataSize);

// 修正tileData里面积为0的poly(设置其flag为0)
int fixNavMeshTileInvalidPoly(void* navTileData); // fix after addTile
// 修正某tile内所有无露天的孤岛polys(设置flags=0),必须在周围tiles都已经加到navmesh里才能正确修正,返回修正的poly数量
int fixNavMeshTileIsland(const dtNavMeshEx& navMesh, const BuildParam& bp, int tileX, int tileZ);

void customMarkHeightfield(rcHeightfield& solid, const rcConfig& cfg, const BuildParam& bp, const BHMapConfig* waterCfg, const BHMapConfig* roadCfg);
bool checkHeightfieldCollision(const rcHeightfield* const hf, float x, float ymin, float ymax, float z);
bool checkHeightfieldCollision(const dtNavMeshEx& navMesh, float x, float ymin, float ymax, float z);
bool dropDownBlocked(const dtNavMeshEx& navMesh, const rcConfig& cfg, const BuildParam& bp, const float* startPos, const float* endPos);
void findValidDropDownForStartPosAndDir(const dtNavMeshEx& navMesh, const dtNavMeshQuery& navQuery, const dtQueryFilter& filter,
    const rcConfig& cfg, const BuildParam& bp, const float* startPos, const float* normal);
void findValidDropDownsForMeshEdgeSegment(const dtNavMeshEx& navMesh, const dtNavMeshQuery& navQuery, const dtQueryFilter& filter,
    const rcConfig& cfg, const BuildParam& bp, const float* segStart, const float* segEnd);

// Returns portal points between two polygons.
dtStatus getPortalPoints(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
    dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile, float* left, float* right);

int circleLineCross(float cx, float cz, float r2, float x0, float z0, float x1, float z1, float res[4]);

// 点(x,z)是否在弧(x0,z0)逆时针到(x0+dx,z0+dz)的范围内
inline bool pointInArc(float x, float z, float x0, float z0, float dx, float dz)
{
    return (x - x0) * dz - (z - z0) * dx >= 0;
}

#endif
