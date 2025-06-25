#pragma once
#include <thread>
#include <atomic>
#include <string>


#include <tbb/concurrent_queue.h>
#include "../Scene/Database.h"
#include "../Base/Options.h"
#include "../Feature/FeatureExtractor.h"

enum CImageStatusFlag : char
{
	CUnread = 0,
	CRead = 1,
	CExtracting = 2,
	CExtracted = 3,
	CMatching = 4,
	CMatched = 5,
	CTwoViewGeometryEstimating = 6,
	CTwoViewGeometryEstimated = 7
};

class CThreadPool final
{
public:
	CThreadPool(CDatabase& database, COptions& options);
	~CThreadPool();
	void Stop();
	void AddReadImageTask(const std::string& imagePath);
	void AddSIFTExtractionTask(const std::string& imagePath);
	void AddSIFTMatchingTask(const std::string& imagePath, const std::vector<size_t>& matchingImageIDs = std::vector<size_t>(0));


private:
	CDatabase* const database;
	COptions* const options;
	std::atomic_bool isStop = false;
	tbb::concurrent_vector<CImageStatusFlag> imageStatus;



	size_t numReadImageThreads = 2;
	tbb::concurrent_queue<std::string> readImageTasks;
	std::vector<std::thread> readImageThreads;

	size_t numGPUThreads = 1;
	tbb::concurrent_queue<std::string> extractionSIFTGPUTasks;
	tbb::concurrent_queue<std::pair<size_t, size_t>> waitingMatchingSIFTGPUTasks;
	std::vector<std::thread> GPUThreads;

	size_t numExtractionSIFTCPUThreads = 2;
	tbb::concurrent_queue<std::string> extractionSIFTCPUTasks;
	std::vector<std::thread> extractionSIFTCPUThreads;

	tbb::concurrent_queue<std::pair<size_t, size_t>> matchingTasks;
	tbb::concurrent_queue<std::pair<size_t, size_t>> waitingMatchingSIFTCPUTasks;
	std::vector<std::thread> findMatchPairsThreads;

	void GPUTask();
	void ReadImage();
	void Extraction();
	void FindMatchPairs();


};
