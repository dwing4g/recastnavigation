#ifndef NAVSERVER_H
#define NAVSERVER_H

#include <stddef.h> // for size_t
#include <stdint.h> // for int64_t
#include <string.h> // for memset
#include <stdio.h>  // for FILE

#ifdef __cplusplus
extern "C" {
class dtNavMesh;
class dtNavMeshEx;
class dtNavMeshQueryEx;
class rcContext;
class NavFieldCtx;
#else
struct dtNavMesh;
struct dtNavMeshEx;
struct dtNavMeshQueryEx;
struct rcContext;
struct NavFieldCtx;
typedef char bool;
#endif
struct dtMeshTile;
struct dtNavMeshCreateParams;
struct dtNavMeshParams;

typedef int64_t NavStatus;

typedef enum // from NavMeshBuildPipeline.h
{
    NAVMESH_POLYAREA_NONE        = 0,
    NAVMESH_POLYAREA_GROUND      = 1,
    NAVMESH_POLYAREA_WATER       = 2,
    NAVMESH_POLYAREA_ROAD        = 3,
    NAVMESH_POLYAREA_JUMP_NEAR   = 4,
    NAVMESH_POLYAREA_JUMP_FAR    = 5,
    NAVMESH_POLYAREA_PLANT       = 6,
    NAVMESH_POLYAREA_TERRAIN     = 7,
    NAVMESH_POLYAREA_JUMP_MEDIUM = 8,
    NAVMESH_POLYAREA_IMPASSABLE  = 15,

    NAVMESH_POLYAREA_MASK        = 0x1f,
    NAVMESH_POLYAREA_HAS_CEILING = 0x20,
} NavMeshPolyAreas;

typedef enum // from NavMeshBuildPipeline.h
{
    NAVMESH_POLYFLAGS_GROUND      = 0x01,
    NAVMESH_POLYFLAGS_WATER       = 0x02,
    NAVMESH_POLYFLAGS_ROAD        = 0x04,
    NAVMESH_POLYFLAGS_JUMP_NEAR   = 0x08,
    NAVMESH_POLYFLAGS_JUMP_FAR    = 0x10,
    NAVMESH_POLYFLAGS_PLANT       = 0x20,
    NAVMESH_POLYFLAGS_JUMP_MEDIUM = 0x40,
    NAVMESH_POLYFLAGS_TEMP_MARK1  = 0x4000, // only for internal use
    NAVMESH_POLYFLAGS_TEMP_MARK2  = 0x8000, // only for internal use
} NavMeshPolyFlags;

typedef enum
{
    RC_LOG_TRACE = 0,
} rcLogCategoryEx;

typedef enum // from NavMeshBuildPipeline.h
{
    RC_TIMER_AUTO_OFFMESH_LINK = 61,
    RC_TIMER_FIX_POLY_FLAGS    = 62,
} rcTimerLabelEx;

/// 初始化NavServer库.必须在其它调用之前调用过一次
/// @return 返回true才能调用其它接口
bool navInit();

/// @return 当前进程此NavServer库的总分配次数
int64_t navGetAllocCount();
/// @return 当前进程此NavServer库的总释放次数
int64_t navGetFreeCount();
/// @return 当前进程此NavServer库分配的字节大小
int64_t navGetMemSize();

/// 使用此NavServer库分配内存
/// @param size 分配的字节大小
/// @return 分配的指针. 0表示失败
void* navAlloc(size_t size);

/// 使用此NavServer库释放已分配的内存
/// @param ptr 之前使用navAlloc分配的指针. 无效指针(0)会被忽略,释放错误指针会导致未知后果,严重时会进程崩溃
void navFree(void* ptr);

/// 加载指定文件名的navmesh文件(后缀名是.rcnavmesh). 其中包括整张地图所有tile的导航网格数据
/// @param fp 输入文件. 必须用二进制可读模式打开, 此方法不负责关闭文件
/// @param outNavMesh 输出的dtNavMeshEx结构指针. 加载成功后通过navFreeNavMesh来释放此指针
/// @return 加载结果的状态. 0表示成功,<0表示失败
NavStatus navLoadNavMesh(FILE* fp, dtNavMeshEx** outNavMesh);

/// 复制指定的navmesh. 其中包括整张地图所有tile的导航网格数据
/// @param navMesh 用于复制的源navmesh. 通过navLoadNavMesh或navForkNavMesh加载得到的
/// @param outNavMesh 输出的dtNavMeshEx结构指针. 加载成功后通过navFreeNavMesh来释放此指针
/// @return 加载结果的状态. 0表示成功,<0表示失败
NavStatus navForkNavMesh(const dtNavMeshEx* navMesh, dtNavMeshEx** outNavMesh);

/// 释放通过navLoadNavMesh或navForkNavMesh加载得到的dtNavMeshEx结构的指针
void navFreeNavMesh(dtNavMeshEx* navMesh);

/// 根据指定的若干模型数据构建一个tile区域内的导航网格数据(需要该tile的写锁,可与寻路等读操作并发)
/// @param navMesh 通过navLoadNavMesh或navForkNavMesh得到的有效指针
/// @param buildParam 构建参数. 从rcnavinput文件中获得,0表示全用默认值
/// @param rcCtx 构建日志接口. 可传0表示不关心
/// @param tileX tile的X坐标
/// @param tileZ tile的Z坐标
/// @param meshDataArray 模型数据指针的数组指针. 其中模型数据指针指向:
///                      [(int)顶点数量(-1表示box模型,顶点数量固定为8,此时没有三角形数量及其顶点索引数据字段) +
///                      (int)三角形数量 + (float[顶点数量*3])顶点坐标数据 + (int[triCount*3])三角形顶点索引数据]
/// @param meshVertPatchArray 模型顶点坐标数据补丁指针的数组指针,用于替代上个参数中的模型顶点坐标. 0表示无任何补丁.
///                           如果有补丁则数组大小必须同上,数组中的指针为0时表示无对应模型的补丁,
///                           每个补丁指针指向: [(float[顶点数量*3])顶点坐标数据]
/// @param arraySize meshDataArray数组中的指针数量
/// @param waterData 水域高度图文件的数据指针(water.whmap)(0表示没有)
/// @param roadData 道路图文件的数据指针(*.rcmask)(0表示没有)
/// @param outNavTileData 输出构建结果的tile数据指针. 成功且输出0表示构建出空数据
/// @param outNavTileDataSize 输出构建结果的tile数据字节大小. 成功且输出0表示构建出空数据
/// @return 构建结果的状态. 0表示成功,<0表示失败
NavStatus navBuildTileBegin(dtNavMeshEx* navMesh, const void* buildParam, rcContext* rcCtx,
    int tileX, int tileZ, const void* const* meshDataArray, const void* const* meshVertPatchArray, int arraySize,
    const void* waterData, const void* roadData, unsigned char** outNavTileData, int* outNavTileDataSize);

/// 构建一个tile区域内的offmesh link数据(需要该map的写锁,可与寻路等读操作并发,该tile及周围tiles需要之前调过navBuildTileBegin且还没释放)
/// @param navMesh 通过navLoadNavMesh或navForkNavMesh得到的有效指针
/// @param navQuery 通过navAllocNavQuery得到的有效指针. 必须是navMesh绑定的
/// @param buildParam 构建参数. 从rcnavinput文件中获得,0表示全用默认值
/// @param rcCtx 构建日志接口. 可传0表示不关心
/// @param tileX tile的X坐标
/// @param tileZ tile的Z坐标
/// @param outNavTileData 输出构建结果的tile数据指针. 成功且输出0表示构建出空数据
/// @param outNavTileDataSize 输出构建结果的tile数据字节大小. 成功且输出0表示构建出空数据
/// @return 构建结果的状态. 0表示成功,<0表示失败
NavStatus navBuildTileLink(dtNavMeshEx* navMesh, const dtNavMeshQueryEx* navQuery, const void* buildParam,
    rcContext* rcCtx, int tileX, int tileZ, unsigned char** outNavTileData, int* outNavTileDataSize);

/// 释放navBuildTileBegin及navBuildTileLink分配的临时空间(需要该tile的写锁,可与寻路等读操作并发)
/// @param navMesh 通过navLoadNavMesh或navForkNavMesh得到的有效指针
/// @param tileX tile的X坐标
/// @param tileZ tile的Z坐标
void navBuildTileEnd(dtNavMeshEx* navMesh, int tileX, int tileZ);

/// 替换navMesh中的某个tile的导航网格数据. 对navMesh是写操作(读读并发,读写和写写不能并发)
/// @param navMesh 通过navLoadNavMesh或navForkNavMesh或navCreateNavMesh得到的有效指针
/// @param navTileData 通过navBuildTile或navAlloc(需要填充之前navBuildTile得到的数据)得到的tile数据指针.
///                    成功后所有权交给navMesh,调用者不能自行释放,navMesh释放时会自动释放所有的tile数据,被替换掉的tile数据也会自动释放
/// @param navTileDataSize navTileData指向的数据字节大小
/// @return 替换结果的状态. 0表示成功; <0表示失败
NavStatus navReplaceTile(dtNavMesh* navMesh, unsigned char* navTileData, int navTileDataSize);

/// 移除navMesh中的某个tile的导航网格数据. 对navMesh是写操作(读读并发,读写和写写不能并发)
/// @param navMesh 通过navLoadNavMesh或navForkNavMesh或navCreateNavMesh得到的有效指针
/// @param tileX tile的X坐标
/// @param tileZ tile的Z坐标
/// @return 移除结果的状态. 0表示移除成功; 1表示未找到该tile数据; <0表示失败
NavStatus navRemoveTile(dtNavMesh* navMesh, int tileX, int tileZ);

/// 修正navMesh中某个tile的不正常poly的flags为0. 对navMesh是写操作(读读并发,读写和写写不能并发)
/// @param navMesh 通过navLoadNavMesh或navForkNavMesh得到的有效指针
/// @param buildParam 构建参数. 从rcnavinput文件中获得,0表示全用默认值
/// @param tileX tile的X坐标
/// @param tileZ tile的Z坐标
/// @return 修正的poly数量. <0表示失败
NavStatus navFixTile(const dtNavMeshEx* navMesh, const void* buildParam, int tileX, int tileZ);

/// 获取navMesh中的某个tile的导航网格数据. 对navMesh是读操作(读读并发,读写和写写不能并发)
/// @param navMesh 通过navLoadNavMesh或navForkNavMesh或navCreateNavMesh得到的有效指针
/// @param tileX tile的X坐标
/// @param tileZ tile的Z坐标
/// @param tilePtr 输出tile的内部数据结构指针(dtMeshTile). 仅在移除该tile或整个navMesh之前有效
/// @return 获取结果的状态. 0表示获取成功; <0表示失败
NavStatus navGetTileData(const dtNavMesh* navMesh, int tileX, int tileZ, const dtMeshTile** tilePtr);

/// 分配绑定某个navMesh的dtNavMeshQueryEx结构指针. 此方法是线程安全的,但获得的指针后续不能并发访问
/// @param navMesh 通过navLoadNavMesh或navForkNavMesh或navCreateNavMesh得到的有效指针
/// @param maxNodes 最大的搜索节点数量. 必须在(0,65535]的范围内
/// @param outNavQuery 输出dtNavMeshQueryEx结构的指针. 如果指向非空指针,则可以复用dtNavMeshQueryEx
/// @return 分配结果的状态. 0表示成功; <0表示失败
NavStatus navAllocNavQuery(const dtNavMesh* navMesh, int maxNodes, dtNavMeshQueryEx** outNavQuery);

/// 释放某个dtNavMeshQueryEx结构指针
/// @param navQuery 通过navAllocNavQuery得到的dtNavMeshQueryEx结构的指针
void navFreeNavQuery(dtNavMeshQueryEx* navQuery);

/// 设置哪些区域可以通过
/// @param navQuery 通过navAllocNavQuery得到的dtNavMeshQueryEx结构的指针
/// @param areaFlags 区域flags的集合. 见NavMeshPolyFlags枚举定义. 默认:0xffff
void navSetFindPathFlags(dtNavMeshQueryEx* navQuery, int areaFlags);

/// 设置某种区域的寻路权重
/// @param navQuery 通过navAllocNavQuery得到的dtNavMeshQueryEx结构的指针
/// @param areaIdx 区域索引. 见NavMeshPolyAreas枚举定义,有效范围:[0,63(DT_MAX_AREAS-1)]
/// @param weight 权值. 默认为1.0f
void navSetFindPathWeight(dtNavMeshQueryEx* navQuery, int areaIdx, float weight);

/// 执行寻路操作. 同一navQuery不能并发,对绑定的navMesh是只读操作(读读并发,读写和写写不能并发)
/// @param navQuery 通过navAllocNavQuery得到的dtNavMeshQueryEx结构的指针
/// @param sp 起点坐标指针. 指向float x,y,z
/// @param tp 终点坐标指针. 指向float x,y,z
/// @param outFloatBuf 输出的坐标数组. 格式为:float[maxPosCount*3].
///                    如果maxPosCount==1且onlyRay则只输出最终坐标(终点或navmesh的边缘);
///                    如果maxPosCount<=0且onlyRay则返回0/1表示从起点到终点的直线是否碰撞navmesh的边缘
/// @param outAreaBuf 输出的区域类型数组(0表示不输出). 格式为:unsigned char[maxPosCount]. 类型定义见NavMeshPolyAreas枚举
/// @param maxPosCount floatBuf指向空间最多可存坐标的数量(每个坐标均为float[3]存放xyz)
/// @param onlyRay 是否仅直线路线(性能会高很多)
/// @return 实际输出的坐标数量. <0表示失败
NavStatus navFindPath(const dtNavMeshQueryEx* navQuery, const float* sp, const float* tp, float* outFloatBuf, unsigned char* outAreaBuf, int maxPosCount, bool onlyRay);

/// 类似navFindPath. 但限定在以(cx,cz)为圆心,radius2为半径平方的范围内寻路. 不支持navFindPath的outAreaBuf和onlyRay参数. 此方法已过时,最好使用navFindPathInField
NavStatus navFindPathInRange(const dtNavMeshQueryEx* navQuery, const float* sp, const float* tp, float* outFloatBuf, int maxPosCount, float cx, float cz, float radius2);

/// 分配一个圆形区域上下文用于之后navFindPathInField所用. 不再用时需调用navFreeFieldCtx释放该上下文
/// @param cx,cz 圆心坐标
/// @param radius 圆形半径
/// @param outFieldCtx 输出新分配的区域上下文指针
/// @return 是否成功创建. 0表示分配成功, <0表示失败
NavStatus navAllocCircleCtx(float cx, float cz, float radius, NavFieldCtx** outFieldCtx);

/// 分配一个多边形区域上下文用于之后navFindPathInField所用. 不再用时需调用navFreeFieldCtx释放该上下文. 多边形只支持凸多边形
/// @param vertXZs 输入多边形按连线顺序的各顶点坐标数组的基址,以x,z,x,z,...的顺序排列,float的数量必须是顶点数量的2倍. 该数组会被复制到区域上下文中
/// @param vertCount 顶点数量. 最少为3
/// @param outFieldCtx 输出新分配的区域上下文指针
/// @return 是否成功创建. 0表示分配成功, <0表示失败
NavStatus navAllocPolygonCtx(const float* vertXZs, int vertCount, NavFieldCtx** outFieldCtx);

/// 释放一个区域上下文
void navFreeFieldCtx(NavFieldCtx* fieldCtx);

/// 类似navFindPath. 但限定在fieldCtx指定的区域内寻路. 相同的fieldCtx不能并发寻路. fieldCtx可为0表示不限制区域. 不支持navFindPath的outAreaBuf参数
NavStatus navFindPathInField(const dtNavMeshQueryEx* navQuery, const float* sp, const float* tp, float* outFloatBuf, int maxPosCount, bool onlyRay, NavFieldCtx* fieldCtx);

/// 根据指定坐标查找位于navMesh上的最近坐标. 同一navQuery不能并发,对绑定的navMesh是只读操作(读读并发,读写和写写不能并发)
/// @param navQuery 通过navAllocNavQuery得到的dtNavMeshQueryEx结构的指针
/// @param p 指定的坐标指针. 指向float x,y,z. 也用于输出坐标(0表示不需要输出)
/// @param method 寻找方式. 0:NavMesh上三维最近的点; 1:优先垂直向下的点
/// @return 查找结果的状态. 0表示查找成功, <0表示查找失败(通常是指定的X和Z坐标离navMesh太远)
NavStatus navFindPos(const dtNavMeshQueryEx* navQuery, float* p, int method);

/// 类似navFindPos. 但限定在fieldCtx指定的区域内寻找. fieldCtx可为0表示不限制区域
NavStatus navFindPosInField(const dtNavMeshQueryEx* navQuery, float* p, const NavFieldCtx* fieldCtx);

/// 执行附近找水操作. 同一navQuery不能并发,对绑定的navMesh是只读操作(读读并发,读写和写写不能并发)
/// @param navQuery 通过navAllocNavQuery得到的dtNavMeshQueryEx结构的指针
/// @param sp 起点坐标指针. 指向float x,y,z
/// @param r 找水的范围半径
/// @param outFloatBuf 输出的坐标数组. 格式为:float[maxPosCount*3]. 如果maxPosCount==1则只输出最终坐标
/// @param maxPosCount floatBuf指向空间最多可存坐标的数量(每个坐标均为float[3]存放xyz)
/// @return 实际输出的坐标数量. 0表示附近找不到水, <0表示失败
NavStatus navFindWater(const dtNavMeshQueryEx* navQuery, const float* sp, float r, float* outFloatBuf, int maxPosCount);

/// 根据指定坐标在周围半径r内随机查找一个NavMesh上的坐标位置. 同一navQuery不能并发,对绑定的navMesh是只读操作(读读并发,读写和写写不能并发)
/// @param navQuery 通过navAllocNavQuery得到的dtNavMeshQueryEx结构的指针
/// @param p 指定的坐标指针. 指向float x,y,z. 也用于输出坐标
/// @param r 随机的范围半径(仅用于判断周围Nav网格的距离,只要有一部分在半径内即可在该网格内任意位置随机找)
/// @return 查找结果的状态. 0表示查找成功, <0表示查找失败(通常是指定的X和Z坐标离navMesh太远)
NavStatus navRandomPos(const dtNavMeshQueryEx* navQuery, float* p, float r);

/// 保存指定navMesh数据到指定文件中
/// @param navMesh 通过navLoadNavMesh或navForkNavMesh得到的有效指针
/// @param fp 输出文件. 必须用二进制可写模式打开, 此方法不负责关闭文件
/// @return 保存结果的状态. 0表示保存成功, <0表示保存失败
NavStatus navSaveNavMesh(const dtNavMeshEx* navMesh, FILE* fp);

/// 检测圆心(cx,cz),半径r,端点(x0,z0)逆时针到(x1,z1)的圆弧是否跟NavMesh中的边线有碰撞
/// @return 0表示没有碰撞, 1表示有碰撞, <0表示其它错误
NavStatus navCheckArcCollision(const dtNavMeshQueryEx* navQuery, float cx, float cz, float r, float x0, float z0, float x1, float z1);

typedef struct DebugParam
{
    rcContext* ctx; // for log, can be 0
    int beginTileX; // for saveHeightfield or saveContourSet or onlyBuildSomeTiles
    int beginTileZ; // for saveHeightfield or saveContourSet or onlyBuildSomeTiles
    int endTileX;   // for saveHeightfield or saveContourSet or onlyBuildSomeTiles
    int endTileZ;   // for saveHeightfield or saveContourSet or onlyBuildSomeTiles
    bool saveDebugFile;
    bool saveHeightfield;
    bool saveContourSet;
    bool onlyBuildSomeTiles;
#ifdef __cplusplus
    DebugParam() { memset(this, 0, sizeof(*this)); }
#endif
} DebugParam;

/// 输入地图资源根目录(ark_resource/resource/develop/server)及地图ID构建整个NavMesh并保存. 路径字符串编码只支持本地编码
NavStatus navBuildAllNavMesh(const char* path, int mapId, const DebugParam* debugParam);

/// 根据dtNavMeshCreateParams中的数据构建NavMesh数据
/// @param params 传入指向dtNavMeshCreateParams数据的指针
/// @param outNavMeshData 输出构建结果的NavMesh数据指针,只有构建成功时有效
/// @param navMeshDataSize 长度至少为1,索引0的位置输出构建结果的NavMesh数据字节大小,只有构建成功时有效
/// @return 构建结果的状态. 0表示构建成功, <0表示构建失败
NavStatus navBuildNavMesh(dtNavMeshCreateParams* params, unsigned char** outNavMeshData, int* outNavMeshDataSize);

/// 根据dtNavMeshParams中的配置创建一个空的dtNavMesh
/// @param params 传入指向dtNavMeshParams数据的指针
/// @param outNavMesh 输出创建的dtNavMesh指针,只有构建成功时有效
/// @return 创建结果的状态. 0表示创建成功, <0表示创建失败
NavStatus navCreateNavMesh(const dtNavMeshParams* params, dtNavMesh** outNavMesh);

/// 释放navCreateNavMesh创建的dtNavMesh
/// @param navMesh 传入有效的dtNavMesh指针
void navDestroyNavMesh(dtNavMesh* navMesh);

/// 保存指定navMesh数据到指定文件中. 对navMesh是读操作(读读并发,读写和写写不能并发)
/// @param navMesh 通过navLoadNavMesh或navForkNavMesh或navCreateNavMesh得到的有效指针
/// @param fp 输出文件. 必须用二进制可写模式打开, 此方法不负责关闭文件
/// @return 保存tile的数量. >=0表示保存成功, <0表示保存失败
NavStatus navDumpNavMesh(const dtNavMesh* navMesh, FILE* fp);

#ifdef __cplusplus
}
#endif

#endif
