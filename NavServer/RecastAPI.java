package recastnavigation;

import java.io.File;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.net.JarURLConnection;
import java.net.URL;
import java.net.URLConnection;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.nio.file.StandardOpenOption;
import java.util.Enumeration;
import java.util.jar.JarEntry;
import java.util.zip.CRC32;
import sun.misc.Unsafe;

public final class RecastAPI {
	/**
	 * 初始化native库. 必须在其它调用之前调用过一次
	 *
	 * @return 返回true才能调用其它接口
	 */
	private static native boolean nativeInit();

	/**
	 * @return 当前进程此native库的总分配次数
	 */
	public static native long nativeAllocCount();

	/**
	 * @return 当前进程此native库的总释放次数
	 */
	public static native long nativeFreeCount();

	/**
	 * @return 当前进程此native库分配的字节大小
	 */
	public static native long nativeMemSize();

	/**
	 * 使用此native库分配内存
	 *
	 * @param size 分配的字节大小. 必须>=0
	 * @return 分配的指针. <=0表示失败
	 */
	public static native long nativeAlloc(int size);

	/**
	 * 使用此native库释放已分配的内存
	 *
	 * @param ptr 之前使用nativeAlloc分配的指针. 无效指针(<=0)会被忽略,释放错误指针会导致未知后果,严重时会进程崩溃
	 */
	public static native void nativeFree(long ptr);

	/**
	 * 加载指定文件名的navmesh文件(后缀名是.rcnavmesh). 其中包括整张地图所有tile的导航网格数据
	 *
	 * @param filename 输入文件名
	 * @return dtNavMesh结构的指针. <=0表示失败, 成功后通过nativeFreeNavMesh来释放
	 */
	public static native long nativeLoadNavMesh(String filename);

	/**
	 * 复制指定的navmesh. 其中包括整张地图所有tile的导航网格数据
	 *
	 * @param navMesh 用于复制的源navmesh
	 * @return dtNavMesh结构的指针. <=0表示失败, 成功后通过nativeFreeNavMesh来释放
	 */
	public static native long nativeForkNavMesh(long navMesh);

	/**
	 * 释放通过nativeLoadNavMesh或nativeForkNavMesh加载得到的dtNavMesh结构的指针
	 */
	public static native void nativeFreeNavMesh(long navMesh);

	/**
	 * 根据指定的若干模型数据构建一个tile区域内的导航网格数据(需要该tile的写锁,可与寻路等读操作并发)
	 *
	 * @param navMesh            通过nativeLoadNavMesh或nativeForkNavMesh得到的有效指针
	 * @param buildParam         构建参数. 从rcnavinput文件中获得,0表示全用默认值
	 * @param tileX              tile的X坐标
	 * @param tileZ              tile的Z坐标
	 * @param meshDataArray      模型数据指针的数组指针. 需要nativeAlloc来分配, 其中模型数据指针指向:
	 *                           [(int)顶点数量(-1表示box模型,顶点数量固定为8,此时没有三角形数量及其顶点索引数据字段) +
	 *                           (int)三角形数量 + (float[顶点数量*3])顶点坐标数据 + (int[triCount*3])三角形顶点索引数据]
	 * @param meshVertPatchArray 模型顶点坐标数据补丁指针的数组指针,用于替代上个参数中的模型顶点坐标.
	 *                           需要nativeAlloc来分配, 0表示无任何补丁.
	 *                           如果有补丁则数组大小必须同上,数组中的指针为0时表示无对应模型的补丁,
	 *                           每个补丁指针指向: [(float[顶点数量*3])顶点坐标数据]
	 * @param arraySize          meshDataArray数组中的指针数量
	 * @param waterData          水域高度图文件的数据指针(water.whmap)(0表示没有)
	 * @param roadData           道路图文件的数据指针(*.rcmask)(0表示没有)
	 * @param navTileDataSize    长度至少为1,索引0的位置输出构建结果的tile数据字节大小
	 * @return navTileData 构建结果的tile数据指针. <0表示失败, 0表示构建出空数据
	 */
	public static native long nativeBuildTileBegin(long navMesh, long buildParam, int tileX, int tileZ,
												   long meshDataArray, long meshVertPatchArray, int arraySize,
												   long waterData, long roadData, int[] navTileDataSize);

	/**
	 * 构建一个tile区域内的offmesh link数据(需要该map的写锁,可与寻路等读操作并发,该tile及周围tiles需要之前调过navBuildTileBegin且还没释放)
	 *
	 * @param navMesh         通过nativeLoadNavMesh或nativeForkNavMesh得到的有效指针
	 * @param navQuery        通过nativeAllocNavQuery得到的有效指针. 必须是navMesh绑定的
	 * @param buildParam      构建参数. 从rcnavinput文件中获得,0表示全用默认值
	 * @param tileX           tile的X坐标
	 * @param tileZ           tile的Z坐标
	 * @param navTileDataSize 长度至少为1,索引0的位置输出构建结果的tile数据字节大小
	 * @return navTileData 构建结果的tile数据指针. 0表示没有offmesh link数据构建出来,无需更新该tile; <0表示失败
	 */
	public static native long nativeBuildTileLink(long navMesh, long navQuery, long buildParam, int tileX, int tileZ,
												  int[] navTileDataSize);

	/**
	 * 释放navBuildTileBegin及navBuildTileLink分配的临时空间(需要该tile的写锁,可与寻路等读操作并发)
	 *
	 * @param navMesh 通过nativeLoadNavMesh或nativeForkNavMesh得到的有效指针
	 * @param tileX   tile的X坐标
	 * @param tileZ   tile的Z坐标
	 */
	public static native void nativeBuildTileEnd(long navMesh, int tileX, int tileZ);

	/**
	 * 替换navMesh中的某个tile的导航网格数据. 对navMesh是写操作(读读并发,读写和写写不能并发)
	 *
	 * @param navMesh         通过nativeLoadNavMesh或nativeForkNavMesh或nativeCreateNavMesh得到的有效指针
	 * @param navTileData     通过nativeBuildTile或nativeAlloc(需要填充之前nativeBuildTile得到的数据)得到的tile数据指针.
	 *                        成功后所有权交给navMesh,调用者不能自行释放,navMesh释放时会自动释放所有的tile数据,被替换掉的tile数据也会自动释放
	 * @param navTileDataSize navTileData指向的数据字节大小
	 * @return 0表示成功; <0表示失败
	 */
	public static native long nativeReplaceTile(long navMesh, long navTileData, int navTileDataSize);

	/**
	 * 移除navMesh中的某个tile的导航网格数据. 对navMesh是写操作(读读并发,读写和写写不能并发)
	 *
	 * @param navMesh 通过nativeLoadNavMesh或nativeForkNavMesh或nativeCreateNavMesh得到的有效指针
	 * @param tileX   tile的X坐标
	 * @param tileZ   tile的Z坐标
	 * @return 0表示移除成功; 1表示未找到该tile数据; <0表示失败
	 */
	public static native long nativeRemoveTile(long navMesh, int tileX, int tileZ);

	/**
	 * 修正navMesh中某个tile的不正常poly的flags为0. 对navMesh是写操作(读读并发,读写和写写不能并发)
	 *
	 * @param navMesh    通过nativeLoadNavMesh或nativeForkNavMesh得到的有效指针
	 * @param buildParam 构建参数. 从rcnavinput文件中获得,0表示全用默认值
	 * @param tileX      tile的X坐标
	 * @param tileZ      tile的Z坐标
	 * @return 修正的poly数量. <0表示失败
	 */
	public static native long nativeFixTile(long navMesh, long buildParam, int tileX, int tileZ);

	/**
	 * 获取navMesh中的某个tile的导航网格数据. 对navMesh是读操作(读读并发,读写和写写不能并发)
	 *
	 * @param navMesh 通过nativeLoadNavMesh或nativeForkNavMesh或nativeCreateNavMesh得到的有效指针
	 * @param tileX   tile的X坐标
	 * @param tileZ   tile的Z坐标
	 * @return tile的内部数据结构指针(dtMeshTile). 仅在移除该tile或整个navMesh之前有效; <=0表示失败
	 */
	public static native long nativeGetTileData(long navMesh, int tileX, int tileZ);

	/**
	 * 分配绑定某个navMesh的navQuery指针. 此方法是线程安全的,但获得的指针后续不能并发访问
	 *
	 * @param navMesh  通过nativeLoadNavMesh或nativeForkNavMesh或nativeCreateNavMesh得到的有效指针
	 * @param maxNodes 最大的搜索节点数量. 必须在(0,65535]的范围内
	 * @return dtNavMeshQueryEx结构的指针, <=0表示失败
	 */
	public static native long nativeAllocNavQuery(long navMesh, int maxNodes);

	/**
	 * 释放某个navQuery指针
	 *
	 * @param navQuery 通过nativeAllocNavQuery得到的有效指针
	 */
	public static native void nativeFreeNavQuery(long navQuery);

	/**
	 * 设置哪些区域可以通过
	 *
	 * @param navQuery  通过nativeAllocNavQuery得到的有效指针
	 * @param areaFlags 区域flags的集合. 见NavMeshPolyFlags枚举定义. 默认:0xffff
	 */
	public static native void nativeSetFindPathFlags(long navQuery, int areaFlags);

	/**
	 * 设置某种区域的寻路权重
	 *
	 * @param navQuery 通过nativeAllocNavQuery得到的有效指针
	 * @param areaIdx  区域索引. 见NavMeshPolyAreas枚举定义,有效范围:[0,63(DT_MAX_AREAS-1)]
	 * @param weight   权值. 默认为1.0f
	 */
	public static native void nativeSetFindPathWeight(long navQuery, int areaIdx, float weight);

	/**
	 * 执行寻路操作. 同一navQuery不能并发,对绑定的navMesh是只读操作(读读并发,读写和写写不能并发)
	 *
	 * @param navQuery    通过nativeAllocNavQuery得到的有效指针
	 * @param sx          起点X坐标
	 * @param sy          起点Y坐标
	 * @param sz          起点Z坐标
	 * @param tx          终点X坐标
	 * @param ty          终点Y坐标
	 * @param tz          终点Z坐标
	 * @param floatBuf    输出的坐标数组. 格式为:float[maxPosCount*3], 需要nativeAlloc来分配.
	 *                    如果maxPosCount==1且onlyRay则只输出最终坐标(终点或navmesh的边缘);
	 *                    如果maxPosCount<=0且onlyRay则返回0/1表示从起点到终点的直线是否碰撞navmesh的边缘
	 * @param areaBuf     输出的区域类型数组(0表示不输出). 格式为:unsigned char[maxPosCount], 需要nativeAlloc来分配. 类型定义见NavMeshPolyAreas枚举
	 * @param maxPosCount floatBuf指向空间最多可存坐标的数量(每个坐标均为float[3]存放xyz)
	 * @param onlyRay     是否仅直线路线(性能会高很多)
	 * @return 实际输出的坐标数量. <0表示失败
	 */
	public static native long nativeFindPath(long navQuery, float sx, float sy, float sz, float tx, float ty, float tz,
											 long floatBuf, long areaBuf, int maxPosCount, boolean onlyRay);

	/**
	 * 类似nativeFindPath. 但限定在以(cx,cz)为圆心,radius2为半径平方的范围内寻路. 不支持nativeFindPath的areaBuf和onlyRay参数
	 */
	public static native long nativeFindPathInRange(long navQuery, float sx, float sy, float sz, float tx, float ty, float tz,
													long floatBuf, int maxPosCount, float cx, float cz, float radius2);

	/**
	 * 分配一个圆形区域上下文用于之后nativeFindPathInField所用. 不再用时需调用nativeFreeFieldCtx释放该上下文
	 *
	 * @param cx,cz  圆心坐标
	 * @param radius 圆形半径
	 * @return 输出新分配的区域上下文指针. >0表示分配成功, <=0表示失败
	 */
	public static native long nativeAllocCircleCtx(float cx, float cz, float radius);

	/**
	 * 分配一个多边形区域上下文用于之后nativeFindPathInField所用. 不再用时需调用nativeFreeFieldCtx释放该上下文. 多边形只支持凸多边形
	 *
	 * @param vertXZs 输入多边形按连线顺序的各顶点坐标数组的基址,以x,z,x,z,...的顺序排列,float的数量必须是顶点数量的2倍. 该数组会被复制到区域上下文中
	 * @return 输出新分配的区域上下文指针. >0表示分配成功, <=0表示失败
	 */
	public static native long nativeAllocPolygonCtx(float[] vertXZs);

	/**
	 * 释放一个区域上下文
	 */
	public static native void nativeFreeFieldCtx(long fieldCtx);

	/**
	 * 类似nativeFindPath. 但限定在fieldCtx指定的区域内寻路. 相同的fieldCtx不能并发寻路. fieldCtx可为0表示不限制区域. 不支持nativeFindPath的areaBuf参数
	 */
	public static native long nativeFindPathInField(long navQuery, float sx, float sy, float sz, float tx, float ty, float tz,
													long floatBuf, int maxPosCount, boolean onlyRay, long fieldCtx);

	/**
	 * 根据指定坐标查找位于navMesh上的最近坐标. 同一navQuery不能并发,对绑定的navMesh是只读操作(读读并发,读写和写写不能并发)
	 *
	 * @param navQuery 通过nativeAllocNavQuery得到的有效指针
	 * @param x        指定的X坐标
	 * @param y        指定的Y坐标
	 * @param z        指定的Z坐标
	 * @param floatBuf 输出坐标的指针(需要float[3]大小的空间). 如果<=0则忽略输出, 需要nativeAlloc来分配
	 * @return 0表示查找成功, <0表示查找失败(通常是指定的X和Z坐标离navMesh太远)
	 */
	public static native long nativeFindPos(long navQuery, float x, float y, float z, long floatBuf);

	/**
	 * 类似nativeFindPos. 但限定在fieldCtx指定的区域内寻找. fieldCtx可为0表示不限制区域
	 */
	public static native long nativeFindPosInField(long navQuery, float x, float y, float z, long floatBuf, long fieldCtx);

	/**
	 * 执行附近找水操作. 同一navQuery不能并发,对绑定的navMesh是只读操作(读读并发,读写和写写不能并发)
	 *
	 * @param navQuery    通过nativeAllocNavQuery得到的有效指针
	 * @param sx          起点X坐标
	 * @param sy          起点Y坐标
	 * @param sz          起点Z坐标
	 * @param r           找水的范围半径
	 * @param floatBuf    输出的坐标数组. 格式为:float[maxPosCount*3], 需要nativeAlloc来分配. 如果maxPosCount==1则只输出最终坐标
	 * @param maxPosCount floatBuf指向空间最多可存坐标的数量(每个坐标均为float[3]存放xyz)
	 * @return 实际输出的坐标数量. 0表示附近找不到水, <0表示失败
	 */
	public static native long nativeFindWater(long navQuery, float sx, float sy, float sz, float r, long floatBuf,
											  int maxPosCount);

	/**
	 * 根据指定坐标在周围半径r内随机查找一个NavMesh上的坐标位置. 同一navQuery不能并发,对绑定的navMesh是只读操作(读读并发,读写和写写不能并发)
	 *
	 * @param navQuery 通过nativeAllocNavQuery得到的有效指针
	 * @param x        指定的X坐标
	 * @param y        指定的Y坐标
	 * @param z        指定的Z坐标
	 * @param r        随机的范围半径(仅用于判断周围Nav网格的距离,只要有一部分在半径内即可在该网格内任意位置随机找)
	 * @param floatBuf 输出的坐标. 格式为:float[3], 需要nativeAlloc来分配
	 * @return 查找结果的状态. 0表示查找成功, <0表示查找失败(通常是指定的X和Z坐标离navMesh太远)
	 */
	public static native long nativeRandomPos(long navQuery, float x, float y, float z, float r, long floatBuf);

	/**
	 * 保存指定navMesh数据到指定文件中
	 *
	 * @param navMesh  通过nativeLoadNavMesh或nativeForkNavMesh得到的有效指针
	 * @param filename 输出文件名
	 * @return 保存结果的状态. 0表示保存成功, <0表示保存失败
	 */
	public static native long nativeSaveNavMesh(long navMesh, String filename);

	/**
	 * 检测圆心(cx,cz),半径r,端点(x0,z0)逆时针到(x1,z1)的圆弧是否跟NavMesh中的边线有碰撞
	 *
	 * @return 0表示没有碰撞, 1表示有碰撞, <0表示其它错误
	 */
	public static native long nativeCheckArcCollision(long navQuery, float cx, float cz, float r,
													  float x0, float z0, float x1, float z1);

	/**
	 * 根据dtNavMeshCreateParams中的数据构建NavMesh数据
	 *
	 * @param navMeshCreateParams 传入指向dtNavMeshCreateParams数据的指针
	 * @param navMeshDataSize     长度至少为1,索引0的位置输出构建结果的NavMesh数据字节大小,只有构建成功时有效
	 * @return 构建结果的NavMesh数据指针. <0表示失败, 0表示构建出空数据
	 */
	public static native long nativeBuildNavMesh(long navMeshCreateParams, int[] navMeshDataSize);

	/**
	 * 根据dtNavMeshParams中的配置创建一个空的dtNavMesh
	 *
	 * @param navMeshParams 传入指向dtNavMeshParams数据的指针
	 * @return 输出创建的dtNavMesh指针. <0表示创建失败
	 */
	public static native long nativeCreateNavMesh(long navMeshParams);

	/**
	 * 释放nativeCreateNavMesh创建的dtNavMesh
	 *
	 * @param navMesh 传入有效的dtNavMesh指针
	 */
	public static native void nativeDestroyNavMesh(long navMesh);

	/**
	 * 保存指定navMesh数据到指定文件中
	 *
	 * @param navMesh  通过nativeLoadNavMesh或nativeForkNavMesh或nativeCreateNavMesh得到的有效指针
	 * @param filename 输出文件名
	 * @return 保存tile的数量. >=0表示保存成功, <0表示保存失败
	 */
	public static native long nativeDumpNavMesh(long navMesh, String filename);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	static {
		loadNativeLib(RecastAPI.class.getClassLoader(), null, "recastjni");
		if (!nativeInit())
			throw new Error("RecastAPI init failed");
	}

	// {libNamePrefix}64.dll; lib{libNamePrefix}64.so
	public static void loadNativeLib(ClassLoader classLoader, String libPathDefault, String libNamePrefix) {
		String nativeLibName = System.mapLibraryName(libNamePrefix + System.getProperty("sun.arch.data.model"));
		File file = libPathDefault != null ? new File(libPathDefault, nativeLibName) : null;
		if (file == null || !file.exists()) {
			String tmpPath = System.getProperty("java.io.tmpdir");
			try {
				long crc = -1;
				String fileName = null, filePath = null;
				Enumeration<URL> urls = classLoader.getResources(nativeLibName);
				URLConnection urlConn = urls.hasMoreElements() ? urls.nextElement().openConnection() : null;
				if (urlConn instanceof JarURLConnection) {
					JarEntry je = ((JarURLConnection)urlConn).getJarEntry();
					if (je != null) {
						crc = je.getCrc() & 0xffff_ffffL;
						fileName = String.format("%08x_%s", crc, nativeLibName);
						filePath = tmpPath + File.separatorChar + fileName;
						file = new File(filePath);
						if (file.length() == je.getSize())
							urlConn = null;
					}
				}
				try (InputStream is = urlConn != null ? urlConn.getInputStream() : null) {
					if (is != null) {
						byte[] data = is.readAllBytes();
						if (crc < 0) {
							CRC32 crc32 = new CRC32();
							crc32.update(data, 0, data.length);
							fileName = String.format("%08x_%s", crc32.getValue(), nativeLibName);
							filePath = tmpPath + File.separatorChar + fileName;
							file = new File(filePath);
							if (file.length() == data.length)
								data = null;
						}
						if (data != null) {
							Path tmpFilePath = Files.createTempFile(fileName, ".tmp");
							Files.write(tmpFilePath, data, StandardOpenOption.CREATE,
									StandardOpenOption.TRUNCATE_EXISTING, StandardOpenOption.WRITE);
							Files.move(tmpFilePath, Paths.get(filePath),
									StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
						}
					}
				}
			} catch (Exception e) {
				throw new Error("create library failed: tmpPath=" + tmpPath + ", nativeLibName=" + nativeLibName, e);
			}
		}
		if (file == null)
			file = new File(nativeLibName);
		System.load(file.getAbsolutePath());
	}

	private RecastAPI() {
	}

	private static Unsafe unsafe;

	private static void initUnsafe() throws ReflectiveOperationException {
		Field theUnsafeField = Class.forName("sun.misc.Unsafe").getDeclaredField("theUnsafe");
		theUnsafeField.setAccessible(true);
		unsafe = (Unsafe)theUnsafeField.get(null);
	}

	private static long t;

	private static void resetTimer() {
		t = System.nanoTime();
	}

	private static int getTime() {
		return (int)((System.nanoTime() - t) / 1_000_000);
	}

	private static void printMem() {
		System.out.format("-------- mem = %d/%d, %d bytes%n", RecastAPI.nativeAllocCount(), RecastAPI.nativeFreeCount(),
				RecastAPI.nativeMemSize());
	}

	public static void main(String[] args) throws ReflectiveOperationException {
		initUnsafe();
		printMem();

		resetTimer();
		final int defMapId = 3061;
		final String filename = args.length == 0
				? "C:/svn/ark_resource/resource/develop/server/map/map" + defMapId + "/navmesh/" + defMapId + ".rcnavmesh"
				: args[0];
		final long navMesh = RecastAPI.nativeLoadNavMesh(filename);
		System.out.format("navMesh  = %016X (%d ms)%n", navMesh, getTime());
		if (navMesh <= 0)
			throw new RuntimeException();
		printMem();

		resetTimer();
		final long navQuery = RecastAPI.nativeAllocNavQuery(navMesh, 65535);
		System.out.format("navQuery = %016X (%d ms)%n", navQuery, getTime());
		if (navQuery <= 0)
			throw new RuntimeException();
		printMem();

		RecastAPI.nativeSetFindPathFlags(navQuery, 0xffff);
		for (int i = 0; i < 64; ++i)
			RecastAPI.nativeSetFindPathWeight(navQuery, i, 1);

		resetTimer();
		final float pi = 3.14159265f;
		final float cx = 71.0f, cz = 41.5f;
		final float cr = 2.f;
		final float a0 = pi * 0.0f;
		final float a1 = pi * 0.5f;
		final float x0 = cx + cr * (float)Math.cos(a0);
		final float z0 = cz + cr * (float)Math.sin(a0);
		final float x1 = cx + cr * (float)Math.cos(a1);
		final float z1 = cz + cr * (float)Math.sin(a1);
		long r = RecastAPI.nativeCheckArcCollision(navQuery, cx, cz, cr, x0, z0, x1, z1);
		System.out.format("navCheckArcCollision = %016X (%d ms)%n", r, getTime());
		printMem();

		resetTimer();
		final int floatBufCount = 4096;
		final long floatBuf = RecastAPI.nativeAlloc(4 * 3 * floatBufCount);
		System.out.format("floatBuf = %016X (%d ms)%n", floatBuf, getTime());
		if (floatBuf <= 0)
			throw new RuntimeException();
		printMem();

		resetTimer();
		r = RecastAPI.nativeFindPath(navQuery, 50.8f, 50.1f, -464.2f,
				409.6f, 37.9f, 367.6f, floatBuf, 0, floatBufCount, false);
		System.out.format("navFindPath = %016X (%d ms)%n", r, getTime());
		if (r < 0)
			throw new RuntimeException();
		for (long i = 0, n = r; i < n; ++i)
			System.out.format("  %4d: %5.3f %5.3f %5.3f%n", i,
					unsafe.getFloat(floatBuf + i * 3 * 4),
					unsafe.getFloat(floatBuf + i * 3 * 4 + 4),
					unsafe.getFloat(floatBuf + i * 3 * 4 + 8));
		printMem();

		resetTimer();
		r = RecastAPI.nativeFindWater(navQuery, 280, 46, -460, 1000, floatBuf, floatBufCount);
		System.out.format("nativeFindWater = %016X (%d ms)%n", r, getTime());
		if (r < 0)
			throw new RuntimeException();
		for (long i = 0, n = r; i < n; ++i)
			System.out.format("  %4d: %5.3f %5.3f %5.3f%n", i,
					unsafe.getFloat(floatBuf + i * 3 * 4),
					unsafe.getFloat(floatBuf + i * 3 * 4 + 4),
					unsafe.getFloat(floatBuf + i * 3 * 4 + 8));
		printMem();

		resetTimer();
		r = RecastAPI.nativeFindPos(navQuery, 280, 46, -460, floatBuf);
		System.out.format("nativeFindPos = %016X (%d ms)%n", r, getTime());
		if (r != 0)
			throw new RuntimeException();
		System.out.format("  %5.3f %5.3f %5.3f%n",
				unsafe.getFloat(floatBuf),
				unsafe.getFloat(floatBuf + 4),
				unsafe.getFloat(floatBuf + 8));
		printMem();

		resetTimer();
		r = RecastAPI.nativeRandomPos(navQuery, 280, 46, -460, 30, floatBuf);
		System.out.format("nativeRandomPos = %016X (%d ms)%n", r, getTime());
		if (r != 0)
			throw new RuntimeException();
		System.out.format("  %5.3f %5.3f %5.3f%n",
				unsafe.getFloat(floatBuf),
				unsafe.getFloat(floatBuf + 4),
				unsafe.getFloat(floatBuf + 8));
		printMem();

		resetTimer();
		final long forkNavMesh = RecastAPI.nativeForkNavMesh(navMesh);
		System.out.format("forkNavMesh = %016X (%d ms)%n", forkNavMesh, getTime());
		if (forkNavMesh <= 0)
			throw new RuntimeException();
		printMem();

		resetTimer();
		r = RecastAPI.nativeSaveNavMesh(navMesh, filename + ".save");
		System.out.format("navSaveNavMesh = %016X (%d ms)%n", r, getTime());
		if (r < 0)
			throw new RuntimeException();
		printMem();

		//TODO: nativeBuildTileBegin, nativeBuildTileLink, nativeBuildTileEnd, nativeReplaceTile, nativeRemoveTile

		System.out.println("free forkNavMesh");
		RecastAPI.nativeFreeNavMesh(forkNavMesh);
		printMem();
		System.out.println("free floatBuf");
		RecastAPI.nativeFree(floatBuf);
		printMem();
		System.out.println("free navQuery");
		RecastAPI.nativeFreeNavQuery(navQuery);
		printMem();
		System.out.println("free navMesh");
		RecastAPI.nativeFreeNavMesh(navMesh);
		printMem();
		System.out.println("end");
	}
}
/* Windows OUTPUT:
mem = 0/0, 0 bytes
navMesh  = 000002320F9DFDC8 (30 ms)
mem = 3603/0, 10120128 bytes
navQuery = 000002320F9E06C8 (0 ms)
mem = 3614/0, 10201256 bytes
floatBuf = 0000023210544E28 (0 ms)
mem = 3615/0, 10213552 bytes
r = 000000000000000C (0 ms)
mem = 3615/0, 10213552 bytes
free floatBuf
mem = 3615/1, 10201256 bytes
free navQuery
mem = 3615/12, 10120128 bytes
free navMesh
mem = 3615/3615, 0 bytes
end
*/
/* Linux OUTPUT:
mem = 0/0, 0 bytes
navMesh  = 00007F9B700BC838 (39 ms)
mem = 3603/0, 10120128 bytes
navQuery = 00007F9B700BC5B8 (0 ms)
mem = 3614/0, 10201256 bytes
floatBuf = 00007F9B70A28568 (0 ms)
mem = 3615/0, 10213552 bytes
r = 000000000000000C (0 ms)
mem = 3615/0, 10213552 bytes
free floatBuf
mem = 3615/1, 10201256 bytes
free navQuery
mem = 3615/12, 10120128 bytes
free navMesh
mem = 3615/3615, 0 bytes
end
*/
