#include <string.h>
#include <stdio.h>
#include <io.h>
#include <string>
#include "NavServer.h"
#include "Recast.h"
#include <stdio.h>
#include <ctime>

class rcDebugContext : public rcContext
{
public:
	rcDebugContext()
	{
		m_timerEnabled = true;
		memset(total, 0, sizeof(total));
	}

protected:

	virtual void doLog(const rcLogCategory category, const char* message, const int /*len*/) override
	{
		if (category == 1)
		{
			printf("INFO,  %s\n", message);
		}
		else if (category == 2)
		{
			printf("WARNING,  %s\n", message);
		}
		else if (category == 3)
		{
			printf("ERROR,  %s\n", message);
			//exit(0);
		}
		else
		{
			printf("doLog %d, message : %s\n", category, message);
		}
	}

	/// Starts the specified performance timer.
	///  @param[in]		label	The category of timer.
	virtual void doStartTimer(const rcTimerLabel label) override
	{
		time[label] = clock();
		//printf("doStartTimer label : %d\n", label);
	}

	/// Stops the specified performance timer.
	///  @param[in]		label	The category of the timer.
	virtual void doStopTimer(const rcTimerLabel label) override
	{
		int deltaClock = clock() - time[label];
		total[label] += deltaClock;
		//printf("doStopTimer label : %d, deltaClock : %d\n", label, deltaClock);
	}

	virtual int doGetAccumulatedTime(const rcTimerLabel label) const override
	{
		return total[label];
	}

private:
	clock_t time[256];
	int total[256];
};

int main(int argc, char** argv)
{
	printf("main argc : %d\n", argc);
	for (int i = 0; i < argc; i++)
		printf("argv[%d] : %s\n", i, argv[i]);

	const char* serverMapRootPath1 = "ark_resource/resource/develop/server";
	const char* serverMapRootPath2 = "tang_resource/resource/develop/server";
	char serverMapRootDir[512];
	for (int i = 0;; i++)
	{
		strcpy(serverMapRootDir + i * 3, serverMapRootPath1);
		if (!_access(serverMapRootDir, 0))
			break;
		strcpy(serverMapRootDir + i * 3, serverMapRootPath2);
		if (!_access(serverMapRootDir, 0))
			break;
		if (i >= 10)
		{
			printf("ERROR: not found '%s' or '%s' in parent path\n", serverMapRootPath1, serverMapRootPath2);
			return -1;
		}
		serverMapRootDir[i * 3] = '.';
		serverMapRootDir[i * 3 + 1] = '.';
		serverMapRootDir[i * 3 + 2] = '/';
	}

	int mapID = 6010;
	bool saveHeightfieldAndContourSet = true;
	if ((argc >= 2 && argv[1][0] == 'r')) // recast
	{
		if (argc == 3)
		{
			sscanf(argv[2], "%d", &mapID);
			saveHeightfieldAndContourSet = false;
		}
		printf("navBuildAllNavMesh start %s, mapID : %d\n", serverMapRootDir, mapID);
		DebugParam param;
		param.saveDebugFile = true;
		param.beginTileX = param.beginTileX = -1;
		param.endTileX = param.endTileZ = 10000;
		param.saveHeightfield = saveHeightfieldAndContourSet;
		param.saveContourSet = saveHeightfieldAndContourSet;
		param.ctx = new rcDebugContext();
		NavStatus status = navBuildAllNavMesh(serverMapRootDir, mapID, &param);
		printf("status : %lld\n", (long long)status);
		if (status != 0)
		{
			exit(-1);
		}
	}
	return 0;
}
