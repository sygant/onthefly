#pragma once

#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

class CLog final
{
public:
	CLog();
	~CLog();
	static void Log(const std::string& logInfo);

private:
	static std::atomic_bool isStop;
	static std::atomic_size_t currentIndex;
	static tbb::concurrent_vector<std::string> logInfos;
	std::thread outputLogThread;

	static void Output();
};