
#include <iostream>
#include "Log.h"
using namespace std;

atomic_bool CLog::isStop = false;
atomic_size_t CLog::currentIndex = 0;
tbb::concurrent_vector<string> CLog::logInfos;

CLog::CLog() : outputLogThread(&CLog::Output)
{
    isStop = false;
    currentIndex = 0;
}

CLog::~CLog()
{
    isStop = true;
    if (outputLogThread.joinable())
    {
        outputLogThread.join();
    }
}

void CLog::Log(const string& logInfo)
{
    logInfos.push_back(logInfo);
}

void CLog::Output()
{
    while (!isStop)
    {
        this_thread::sleep_for(chrono::milliseconds(10));
        if (currentIndex >= logInfos.size())
        {
            continue;
        }
        cout << logInfos[currentIndex] << endl;
        currentIndex++;
    }
}