#include <malloc.h>
#include <string.h>
#include <jni.h>
#include "NavServer.h"

// private static native bool nativeInit();
extern "C" JNIEXPORT jboolean JNICALL Java_recastnavigation_RecastAPI_nativeInit(JNIEnv*, jclass)
{
    return static_cast<jboolean>(navInit() ? JNI_TRUE : JNI_FALSE);
}

// public static native long nativeAllocCount();
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeAllocCount(JNIEnv*, jclass)
{
    return static_cast<jlong>(navGetAllocCount());
}
extern "C" JNIEXPORT jlong JNICALL JavaCritical_recastnavigation_RecastAPI_nativeAllocCount()
{
    return static_cast<jlong>(navGetAllocCount());
}

// public static native long nativeFreeCount();
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeFreeCount(JNIEnv*, jclass)
{
    return static_cast<jlong>(navGetFreeCount());
}
extern "C" JNIEXPORT jlong JNICALL JavaCritical_recastnavigation_RecastAPI_nativeFreeCount()
{
    return static_cast<jlong>(navGetFreeCount());
}

// public static native long nativeMemSize();
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeMemSize(JNIEnv*, jclass)
{
    return static_cast<jlong>(navGetMemSize());
}
extern "C" JNIEXPORT jlong JNICALL JavaCritical_recastnavigation_RecastAPI_nativeMemSize()
{
    return static_cast<jlong>(navGetMemSize());
}

// public static native long nativeAlloc(int size);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeAlloc(JNIEnv*, jclass, jint size)
{
    return size >= 0 ? reinterpret_cast<jlong>(navAlloc(static_cast<size_t>(size))) : 0;
}
extern "C" JNIEXPORT jlong JNICALL JavaCritical_recastnavigation_RecastAPI_nativeAlloc(jint size)
{
    return size >= 0 ? reinterpret_cast<jlong>(navAlloc(static_cast<size_t>(size))) : 0;
}

// public static native void nativeFree(long ptr);
extern "C" JNIEXPORT void JNICALL Java_recastnavigation_RecastAPI_nativeFree(JNIEnv*, jclass, jlong ptr)
{
    if (ptr > 0)
        navFree(reinterpret_cast<void*>(ptr));
}
extern "C" JNIEXPORT void JNICALL JavaCritical_recastnavigation_RecastAPI_nativeFree(jlong ptr)
{
    if (ptr > 0)
        navFree(reinterpret_cast<void*>(ptr));
}

// public static native long nativeLoadNavMesh(String filename);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeLoadNavMesh(JNIEnv* const jenv, jclass, jstring filename)
{
    if (!filename)
        return -101;
#ifdef _WIN32
    const jchar* const pfn = jenv->GetStringChars(filename, 0);
    if (!pfn)
        return -102;
    FILE* const fp = _wfopen(reinterpret_cast<const wchar_t*>(pfn), L"rb");
    jenv->ReleaseStringChars(filename, pfn);
#else
    const char* const pfn = jenv->GetStringUTFChars(filename, 0);
    if (!pfn)
        return -102;
    FILE* const fp = fopen(pfn, "rb");
    jenv->ReleaseStringUTFChars(filename, pfn);
#endif
    if (!fp)
        return -103;

    dtNavMeshEx* navMesh;
    const NavStatus s = navLoadNavMesh(fp, &navMesh);
    fclose(fp);
    if (s)
        return static_cast<jlong>(s);
    return reinterpret_cast<jlong>(navMesh);
}

// public static native long nativeForkNavMesh(long navMesh);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeForkNavMesh(JNIEnv*, jclass, jlong navMesh)
{
    if (navMesh <= 0)
        return -101;
    dtNavMeshEx* newNavMesh;
    const NavStatus s = navForkNavMesh(reinterpret_cast<dtNavMeshEx*>(navMesh), &newNavMesh);
    if (s)
        return static_cast<jlong>(s);
    return reinterpret_cast<jlong>(newNavMesh);
}

// public static native void nativeFreeNavMesh(long navMesh);
extern "C" JNIEXPORT void JNICALL Java_recastnavigation_RecastAPI_nativeFreeNavMesh(JNIEnv*, jclass, jlong navMesh)
{
    if (navMesh > 0)
        navFreeNavMesh(reinterpret_cast<dtNavMeshEx*>(navMesh));
}
extern "C" JNIEXPORT void JNICALL JavaCritical_recastnavigation_RecastAPI_nativeFreeNavMesh(jlong navMesh)
{
    if (navMesh > 0)
        navFreeNavMesh(reinterpret_cast<dtNavMeshEx*>(navMesh));
}

// public static native long nativeBuildTileBegin(long navMesh, long buildParam, int tileX, int tileZ, long meshDataArray, long meshVertPatchArray, int arraySize, long waterData, long roadData, int[] navTileDataSize);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeBuildTileBegin
    (JNIEnv* const jenv, jclass, jlong navMesh, jlong buildParam, jint tileX, jint tileZ, jlong meshDataArray, jlong meshVertPatchArray, jint arraySize, jlong waterData, jlong roadData, jintArray navTileDataSize)
{
    if (navMesh <= 0)
        return -101;
    if (buildParam < 0)
        return -102;
    if (meshDataArray <= 0)
        return -103;
    if (meshVertPatchArray < 0)
        return -104;
    if (arraySize < 0)
        return -105;
    if (waterData < 0)
        return -106;
    if (roadData < 0)
        return -107;
    if (!navTileDataSize)
        return -108;
    if (jenv->GetArrayLength(navTileDataSize) < 1)
        return -109;

    unsigned char* navData;
    int navDataSize;
    const NavStatus s = navBuildTileBegin(reinterpret_cast<dtNavMeshEx*>(navMesh), reinterpret_cast<const void*>(buildParam), 0/*TODO*/,
        tileX, tileZ, reinterpret_cast<const void* const*>(meshDataArray), reinterpret_cast<const void* const*>(meshVertPatchArray),
        arraySize, reinterpret_cast<void*>(waterData), reinterpret_cast<void*>(roadData), &navData, &navDataSize);
    if (s)
        return static_cast<jlong>(s);
    jenv->SetIntArrayRegion(navTileDataSize, 0, 1, reinterpret_cast<jint*>(&navDataSize));
    return reinterpret_cast<jlong>(navData);
}

// public static native long nativeBuildTileLink(long navMesh, long navQuery, long buildParam, int tileX, int tileZ, int[] navTileDataSize);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeBuildTileLink
    (JNIEnv* const jenv, jclass, jlong navMesh, jlong navQuery, jlong buildParam, jint tileX, jint tileZ, jintArray navTileDataSize)
{
    if (navMesh <= 0)
        return -101;
    if (navQuery <= 0)
        return -102;
    if (buildParam < 0)
        return -103;
    if (!navTileDataSize)
        return -104;
    if (jenv->GetArrayLength(navTileDataSize) < 1)
        return -105;

    unsigned char* navData;
    int navDataSize;
    const NavStatus s = navBuildTileLink(reinterpret_cast<dtNavMeshEx*>(navMesh), reinterpret_cast<dtNavMeshQueryEx*>(navQuery),
        reinterpret_cast<const void*>(buildParam), 0/*TODO*/, tileX, tileZ, &navData, &navDataSize);
    if (s)
        return static_cast<jlong>(s);
    jenv->SetIntArrayRegion(navTileDataSize, 0, 1, reinterpret_cast<jint*>(&navDataSize));
    return reinterpret_cast<jlong>(navData);
}

// public static native void nativeBuildTileEnd(long navMesh, int tileX, int tileZ);
extern "C" JNIEXPORT void JNICALL Java_recastnavigation_RecastAPI_nativeBuildTileEnd
    (JNIEnv*, jclass, jlong navMesh, jint tileX, jint tileZ)
{
    if (navMesh > 0)
        navBuildTileEnd(reinterpret_cast<dtNavMeshEx*>(navMesh), tileX, tileZ);
}

// public static native long nativeReplaceTile(long navMesh, long navTileData, int navTileDataSize);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeReplaceTile
    (JNIEnv*, jclass, jlong navMesh, jlong navTileData, jint navTileDataSize)
{
    if (navMesh <= 0)
        return -101;
    if (navTileData <= 0)
        return -102;
    const NavStatus s = navReplaceTile(reinterpret_cast<dtNavMesh*>(navMesh), reinterpret_cast<unsigned char*>(navTileData), navTileDataSize);
    return static_cast<jlong>(s);
}

// public static native long nativeRemoveTile(long navMesh, int tileX, int tileZ);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeRemoveTile
    (JNIEnv*, jclass, jlong navMesh, jint tileX, jint tileZ)
{
    if (navMesh <= 0)
        return -101;
    const NavStatus s = navRemoveTile(reinterpret_cast<dtNavMesh*>(navMesh), tileX, tileZ);
    return static_cast<jlong>(s);
}

// public static native long nativeFixTile(long navMesh, long buildParam, int tileX, int tileZ);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeFixTile
    (JNIEnv*, jclass, jlong navMesh, jlong buildParam, jint tileX, jint tileZ)
{
    if (navMesh <= 0)
        return -101;
    if (buildParam < 0)
        return -102;
    const NavStatus s = navFixTile(reinterpret_cast<dtNavMeshEx*>(navMesh), reinterpret_cast<const void*>(buildParam), tileX, tileZ);
    return static_cast<jlong>(s);
}

// public static native long nativeGetTileData(long navMesh, int tileX, int tileZ);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeGetTileData
    (JNIEnv*, jclass, jlong navMesh, jint tileX, jint tileZ)
{
    if (navMesh <= 0)
        return -101;
    const dtMeshTile* dataPtr = 0;
    const NavStatus s = navGetTileData(reinterpret_cast<const dtNavMesh*>(navMesh), tileX, tileZ, &dataPtr);
    return s < 0 ? s : reinterpret_cast<jlong>(dataPtr);
}

// public static native long nativeAllocNavQuery(long navMesh, int maxNodes);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeAllocNavQuery(JNIEnv*, jclass, jlong navMesh, jint maxNodes)
{
    if (navMesh <= 0)
        return -101;
    dtNavMeshQueryEx* navQuery = 0;
    const NavStatus s = navAllocNavQuery(reinterpret_cast<const dtNavMesh*>(navMesh), maxNodes, &navQuery);
    return s < 0 ? static_cast<jlong>(s) : reinterpret_cast<jlong>(navQuery);
}
extern "C" JNIEXPORT jlong JNICALL JavaCritical_recastnavigation_RecastAPI_nativeAllocNavQuery(jlong navMesh, jint maxNodes)
{
    if (navMesh <= 0)
        return -101;
    dtNavMeshQueryEx* navQuery = 0;
    const NavStatus s = navAllocNavQuery(reinterpret_cast<const dtNavMesh*>(navMesh), maxNodes, &navQuery);
    return s < 0 ? static_cast<jlong>(s) : reinterpret_cast<jlong>(navQuery);
}

// public static native void nativeFreeNavQuery(long navQuery);
extern "C" JNIEXPORT void JNICALL Java_recastnavigation_RecastAPI_nativeFreeNavQuery(JNIEnv*, jclass, jlong navQuery)
{
    if (navQuery > 0)
        navFreeNavQuery(reinterpret_cast<dtNavMeshQueryEx*>(navQuery));
}
extern "C" JNIEXPORT void JNICALL JavaCritical_recastnavigation_RecastAPI_nativeFreeNavQuery(jlong navQuery)
{
    if (navQuery > 0)
        navFreeNavQuery(reinterpret_cast<dtNavMeshQueryEx*>(navQuery));
}

// public static native void nativeSetFindPathFlags(long navQuery, int areaFlags);
extern "C" JNIEXPORT void JNICALL Java_recastnavigation_RecastAPI_nativeSetFindPathFlags
    (JNIEnv*, jclass, jlong navQuery, jint areaFlags)
{
    if (navQuery > 0)
        navSetFindPathFlags(reinterpret_cast<dtNavMeshQueryEx*>(navQuery), areaFlags);
}
extern "C" JNIEXPORT void JNICALL JavaCritical_recastnavigation_RecastAPI_nativeSetFindPathFlags
    (jlong navQuery, jint areaFlags)
{
    if (navQuery > 0)
        navSetFindPathFlags(reinterpret_cast<dtNavMeshQueryEx*>(navQuery), areaFlags);
}

// public static native void nativeSetFindPathWeight(long navQuery, int areaIdx, float weight);
extern "C" JNIEXPORT void JNICALL Java_recastnavigation_RecastAPI_nativeSetFindPathWeight
    (JNIEnv*, jclass, jlong navQuery, jint areaIdx, jfloat weight)
{
    if (navQuery > 0)
        navSetFindPathWeight(reinterpret_cast<dtNavMeshQueryEx*>(navQuery), areaIdx, weight);
}
extern "C" JNIEXPORT void JNICALL JavaCritical_recastnavigation_RecastAPI_nativeSetFindPathWeight
    (jlong navQuery, jint areaIdx, jfloat weight)
{
    if (navQuery > 0)
        navSetFindPathWeight(reinterpret_cast<dtNavMeshQueryEx*>(navQuery), areaIdx, weight);
}

// public static native long nativeFindPath(long navQuery, float sx, float sy, float sz, float tx, float ty, float tz, long floatBuf, long areaBuf, int maxPosCount, boolean onlyRay);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeFindPath
    (JNIEnv*, jclass, jlong navQuery, jfloat sx, jfloat sy, jfloat sz, jfloat tx, jfloat ty, jfloat tz, jlong floatBuf, jlong areaBuf, jint maxPosCount, jboolean onlyRay)
{
    if (navQuery <= 0)
        return -101;
    if (floatBuf < 0 || (floatBuf == 0 && maxPosCount > 0))
        return -102;
    if (areaBuf < 0)
        return -103;
    const float sp[3] = { sx, sy, sz };
    const float tp[3] = { tx, ty, tz };
    const NavStatus s = navFindPath(reinterpret_cast<const dtNavMeshQueryEx*>(navQuery), sp, tp,
        reinterpret_cast<float*>(floatBuf), reinterpret_cast<unsigned char*>(areaBuf), maxPosCount, onlyRay == JNI_TRUE);
    return static_cast<jlong>(s);
}

// public static native long nativeFindPathInRange(long navQuery, float sx, float sy, float sz, float tx, float ty, float tz, long floatBuf, int maxPosCount, float cx, float cz, float radius2);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeFindPathInRange
    (JNIEnv*, jclass, jlong navQuery, jfloat sx, jfloat sy, jfloat sz, jfloat tx, jfloat ty, jfloat tz, jlong floatBuf, jint maxPosCount, jfloat cx, jfloat cz, jfloat radius2)
{
    if (navQuery <= 0)
        return -101;
    if (floatBuf < 0 || (floatBuf == 0 && maxPosCount > 0))
        return -102;
    const float sp[3] = { sx, sy, sz };
    const float tp[3] = { tx, ty, tz };
    const NavStatus s = navFindPathInRange(reinterpret_cast<const dtNavMeshQueryEx*>(navQuery), sp, tp,
        reinterpret_cast<float*>(floatBuf), maxPosCount, cx, cz, radius2);
    return static_cast<jlong>(s);
}

// public static native long nativeAllocCircleCtx(float cx, float cz, float radius);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeAllocCircleCtx
    (JNIEnv*, jclass, jfloat cx, jfloat cz, jfloat radius)
{
    NavFieldCtx* ctx = 0;
    const NavStatus s = navAllocCircleCtx(cx, cz, radius, &ctx);
    return s < 0 ? static_cast<jlong>(s) : reinterpret_cast<jlong>(ctx);
}
extern "C" JNIEXPORT jlong JNICALL JavaCritical_recastnavigation_RecastAPI_nativeAllocCircleCtx
    (jfloat cx, jfloat cz, jfloat radius)
{
    NavFieldCtx* ctx = 0;
    const NavStatus s = navAllocCircleCtx(cx, cz, radius, &ctx);
    return s < 0 ? static_cast<jlong>(s) : reinterpret_cast<jlong>(ctx);
}

// public static native long nativeAllocPolygonCtx(float[] vertXZs);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeAllocPolygonCtx
    (JNIEnv* const jenv, jclass, jfloatArray vertXZs)
{
    if (!vertXZs)
        return -101;
    jsize arrayLen = jenv->GetArrayLength(vertXZs);
#ifdef _MSC_VER
    float* const vertBuf = static_cast<float*>(_alloca(sizeof(float) * arrayLen));
    if (!vertBuf)
        return -102;
#else
    float vertBuf[arrayLen];
#endif
    jenv->GetFloatArrayRegion(vertXZs, 0, arrayLen, static_cast<jfloat*>(vertBuf));
    NavFieldCtx* ctx = 0;
    const NavStatus s = navAllocPolygonCtx(vertBuf, arrayLen / 2, &ctx);
    return s < 0 ? static_cast<jlong>(s) : reinterpret_cast<jlong>(ctx);
}
extern "C" JNIEXPORT jlong JNICALL JavaCritical_recastnavigation_RecastAPI_nativeAllocPolygonCtx
    (jint vertXZsLen, jfloat* vertXZs)
{
    if (!vertXZs)
        return -101;
    NavFieldCtx* ctx = 0;
    const NavStatus s = navAllocPolygonCtx(static_cast<const float*>(vertXZs), vertXZsLen / 2, &ctx);
    return s < 0 ? static_cast<jlong>(s) : reinterpret_cast<jlong>(ctx);
}

// public static native void nativeFreeFieldCtx(long fieldCtx);
extern "C" JNIEXPORT void JNICALL Java_recastnavigation_RecastAPI_nativeFreeFieldCtx
    (JNIEnv*, jclass, jlong fieldCtx)
{
    if (fieldCtx > 0)
        navFreeFieldCtx(reinterpret_cast<NavFieldCtx*>(fieldCtx));
}
extern "C" JNIEXPORT void JNICALL JavaCritical_recastnavigation_RecastAPI_nativeFreeFieldCtx
    (jlong fieldCtx)
{
    if (fieldCtx > 0)
        navFreeFieldCtx(reinterpret_cast<NavFieldCtx*>(fieldCtx));
}

// public static native long nativeFindPathInField(long navQuery, float sx, float sy, float sz, float tx, float ty, float tz, long floatBuf, int maxPosCount, boolean onlyRay, long fieldCtx);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeFindPathInField
    (JNIEnv*, jclass, jlong navQuery, jfloat sx, jfloat sy, jfloat sz, jfloat tx, jfloat ty, jfloat tz, jlong floatBuf, jint maxPosCount, jboolean onlyRay, jlong fieldCtx)
{
    if (navQuery <= 0)
        return -101;
    if (floatBuf < 0 || (floatBuf == 0 && maxPosCount > 0))
        return -102;
    if (fieldCtx < 0)
        return -103;
    const float sp[3] = { sx, sy, sz };
    const float tp[3] = { tx, ty, tz };
    const NavStatus s = navFindPathInField(reinterpret_cast<const dtNavMeshQueryEx*>(navQuery), sp, tp,
        reinterpret_cast<float*>(floatBuf), maxPosCount, onlyRay == JNI_TRUE, reinterpret_cast<NavFieldCtx*>(fieldCtx));
    return static_cast<jlong>(s);
}

// public static native long nativeFindPos(long navQuery, float x, float y, float z, long floatBuf);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeFindPos
    (JNIEnv*, jclass, jlong navQuery, jfloat x, jfloat y, jfloat z, jlong floatBuf)
{
    if (navQuery <= 0)
        return -101;
    if (floatBuf < 0)
        return -102;
    float p[3] = { x, y, z };
    const NavStatus s = navFindPos(reinterpret_cast<const dtNavMeshQueryEx*>(navQuery), p);
    if (floatBuf > 0)
        memcpy(reinterpret_cast<float*>(floatBuf), p, sizeof(p));
    return static_cast<jlong>(s);
}
extern "C" JNIEXPORT jlong JNICALL JavaCritical_recastnavigation_RecastAPI_nativeFindPos
    (jlong navQuery, jfloat x, jfloat y, jfloat z, jlong floatBuf)
{
    if (navQuery <= 0)
        return -101;
    if (floatBuf < 0)
        return -102;
    float p[3] = { x, y, z };
    const NavStatus s = navFindPos(reinterpret_cast<const dtNavMeshQueryEx*>(navQuery), p);
    if (floatBuf > 0)
        memcpy(reinterpret_cast<float*>(floatBuf), p, sizeof(p));
    return static_cast<jlong>(s);
}

// public static native long nativeFindPosInField(long navQuery, float x, float y, float z, long floatBuf, long fieldCtx);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeFindPosInField
    (JNIEnv*, jclass, jlong navQuery, jfloat x, jfloat y, jfloat z, jlong floatBuf, jlong fieldCtx)
{
    if (navQuery <= 0)
        return -101;
    if (floatBuf < 0)
        return -102;
    if (fieldCtx < 0)
        return -103;
    float p[3] = { x, y, z };
    const NavStatus s = navFindPosInField(reinterpret_cast<const dtNavMeshQueryEx*>(navQuery), p, reinterpret_cast<const NavFieldCtx*>(fieldCtx));
    if (floatBuf > 0)
        memcpy(reinterpret_cast<float*>(floatBuf), p, sizeof(p));
    return static_cast<jlong>(s);
}
extern "C" JNIEXPORT jlong JNICALL JavaCritical_recastnavigation_RecastAPI_nativeFindPosInField
    (jlong navQuery, jfloat x, jfloat y, jfloat z, jlong floatBuf, jlong fieldCtx)
{
    if (navQuery <= 0)
        return -101;
    if (floatBuf < 0)
        return -102;
    if (fieldCtx < 0)
        return -103;
    float p[3] = { x, y, z };
    const NavStatus s = navFindPosInField(reinterpret_cast<const dtNavMeshQueryEx*>(navQuery), p, reinterpret_cast<const NavFieldCtx*>(fieldCtx));
    if (floatBuf > 0)
        memcpy(reinterpret_cast<float*>(floatBuf), p, sizeof(p));
    return static_cast<jlong>(s);
}

// public static native long nativeFindWater(long navQuery, float sx, float sy, float sz, float r, long floatBuf, int maxPosCount);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeFindWater
    (JNIEnv*, jclass, jlong navQuery, jfloat sx, jfloat sy, jfloat sz, jfloat r, jlong floatBuf, jint maxPosCount)
{
    if (navQuery <= 0)
        return -101;
    if (floatBuf <= 0)
        return -102;
    const float sp[3] = { sx, sy, sz };
    const NavStatus s = navFindWater(reinterpret_cast<const dtNavMeshQueryEx*>(navQuery), sp, r, reinterpret_cast<float*>(floatBuf), maxPosCount);
    return static_cast<jlong>(s);
}

// public static native long nativeRandomPos(long navQuery, float x, float y, float z, float r, long floatBuf);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeRandomPos
    (JNIEnv*, jclass, jlong navQuery, jfloat x, jfloat y, jfloat z, jfloat r, jlong floatBuf)
{
    if (navQuery <= 0)
        return -101;
    if (floatBuf <= 0)
        return -102;
    float* p = reinterpret_cast<float*>(floatBuf);
    p[0] = x;
    p[1] = y;
    p[2] = z;
    const NavStatus s = navRandomPos(reinterpret_cast<const dtNavMeshQueryEx*>(navQuery), p, r);
    return static_cast<jlong>(s);
}

// public static native long nativeSaveNavMesh(long navMesh, String filename);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeSaveNavMesh(JNIEnv* const jenv, jclass, jlong navMesh, jstring filename)
{
    if (navMesh <= 0)
        return -101;
    if (!filename)
        return -102;
#ifdef _WIN32
    const jchar* const pfn = jenv->GetStringChars(filename, 0);
    if (!pfn)
        return -103;
    FILE* const fp = _wfopen(reinterpret_cast<const wchar_t*>(pfn), L"wb");
    jenv->ReleaseStringChars(filename, pfn);
#else
    const char* const pfn = jenv->GetStringUTFChars(filename, 0);
    if (!pfn)
        return -103;
    FILE* const fp = fopen(pfn, "wb");
    jenv->ReleaseStringUTFChars(filename, pfn);
#endif
    if (!fp)
        return -104;

    const NavStatus s = navSaveNavMesh(reinterpret_cast<const dtNavMeshEx*>(navMesh), fp);
    fclose(fp);
    return static_cast<jlong>(s);
}

// public static native long nativeCheckArcCollision(long navQuery, float cx, float cz, float r, float x0, float z0, float x1, float z1);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeCheckArcCollision
    (JNIEnv*, jclass, jlong navQuery, jfloat cx, jfloat cz, jfloat r, jfloat x0, jfloat z0, jfloat x1, jfloat z1)
{
    if (navQuery <= 0)
        return -101;
    const NavStatus s = navCheckArcCollision(reinterpret_cast<const dtNavMeshQueryEx*>(navQuery), cx, cz, r, x0, z0, x1, z1);
    return static_cast<jlong>(s);
}
extern "C" JNIEXPORT jlong JNICALL JavaCritical_recastnavigation_RecastAPI_nativeCheckArcCollision
    (jlong navQuery, jfloat cx, jfloat cz, jfloat r, jfloat x0, jfloat z0, jfloat x1, jfloat z1)
{
    if (navQuery <= 0)
        return -101;
    const NavStatus s = navCheckArcCollision(reinterpret_cast<const dtNavMeshQueryEx*>(navQuery), cx, cz, r, x0, z0, x1, z1);
    return static_cast<jlong>(s);
}

// public static native long nativeBuildNavMesh(long navMeshCreateParams, int[] navMeshDataSize);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeBuildNavMesh
    (JNIEnv* const jenv, jclass, jlong navMeshCreateParams, jintArray navMeshDataSize)
{
    if (navMeshCreateParams <= 0)
        return -101;
    if (!navMeshDataSize)
        return -102;
    if (jenv->GetArrayLength(navMeshDataSize) < 1)
        return -103;

    unsigned char* navData;
    int navDataSize;
    const NavStatus s = navBuildNavMesh(reinterpret_cast<dtNavMeshCreateParams*>(navMeshCreateParams), &navData, &navDataSize);
    if (s)
        return static_cast<jlong>(s);
    jenv->SetIntArrayRegion(navMeshDataSize, 0, 1, reinterpret_cast<jint*>(&navDataSize));
    return reinterpret_cast<jlong>(navData);
}

// public static native long nativeCreateNavMesh(long navMeshParams);
extern "C" JNIEXPORT jlong JNICALL Java_recastnavigation_RecastAPI_nativeCreateNavMesh
    (JNIEnv*, jclass, jlong navMeshParams)
{
    if (navMeshParams <= 0)
        return -101;
    dtNavMesh* navMesh;
    const NavStatus s = navCreateNavMesh(reinterpret_cast<dtNavMeshParams*>(navMeshParams), &navMesh);
    if (s)
        return static_cast<jlong>(s);
    return reinterpret_cast<jlong>(navMesh);
}
extern "C" JNIEXPORT jlong JNICALL JavaCritical_recastnavigation_RecastAPI_nativeCreateNavMesh
    (jlong navMeshParams)
{
    if (navMeshParams <= 0)
        return -101;
    dtNavMesh* navMesh;
    const NavStatus s = navCreateNavMesh(reinterpret_cast<dtNavMeshParams*>(navMeshParams), &navMesh);
    if (s)
        return static_cast<jlong>(s);
    return reinterpret_cast<jlong>(navMesh);
}

// public static native void nativeDestroyNavMesh(long navMesh);
extern "C" JNIEXPORT void JNICALL Java_recastnavigation_RecastAPI_nativeDestroyNavMesh
    (JNIEnv*, jclass, jlong navMesh)
{
    if (navMesh > 0)
        navDestroyNavMesh(reinterpret_cast<dtNavMesh*>(navMesh));
}
extern "C" JNIEXPORT void JNICALL JavaCritical_recastnavigation_RecastAPI_nativeDestroyNavMesh
    (jlong navMesh)
{
    if (navMesh > 0)
        navDestroyNavMesh(reinterpret_cast<dtNavMesh*>(navMesh));
}
