#pragma once
#include <string>
#include <queue>
#include <mutex>
#include "../Scene/Database.h"
#include "../Feature/FeatureExtractor.h"
#include "../Feature/ImageReader.h"
#include "../Feature/FeatureMatcher.h"
#include "Reconstructor.h"

#ifndef _DEBUG
#undef slots
#undef emit
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_map.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_unordered_set.h>
#define slots Q_SLOTS
#define emit Q_EMIT
#endif // !_DEBUG


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

class CWorkflow final
{
public:

	~CWorkflow();

	//static void Initialize(Database* database, COptions* options, CReconstructor* reconstructor, std::size_t numReadThreads = 1, std::size_t numGPUThreads = 1, std::size_t numSIFTExtractCPUThreads = 2, std::size_t numSIFTMatchCPUThreads = 4, std::size_t numEstimateThreads = 4);
	static void Initialize(Database* database, Reconstructor* reconstructor, std::size_t numReadThreads = 1, std::size_t numGPUThreads = 1, std::size_t numSIFTExtractCPUThreads = 2, std::size_t numSIFTMatchCPUThreads = 4, std::size_t numEstimateThreads = 4);
	static void SetDatabase(Database* database);
	//static void SetOptions(COptions* options);
	//static void SetReconstructor(CReconstructor* reconstructor);

	static void Stop();

	static void AddImageProcess(const std::string& imagePath);

	static PyLoader* pyloader;
	static void LoadPython();
	static bool SIFTExtract(const std::string& imagePath, const CSIFTExtractionOptions& options, Database& database);
	static bool SIFTMatch(Database& database, std::size_t imageID1, std::size_t imageID2, const CSIFTMatchingOptions& options);
	static bool PyMatch(Database& database , std::size_t imageID1 , std::size_t imageID2 );
	//static bool EstimateTwoViewGeometry(Database& database, std::size_t imageID1, std::size_t imageID2, const CTwoViewGeometryOptions& options);
	static bool EstimateTwoViewGeometry(Database& database, std::size_t imageID1, std::size_t imageID2);

	static void OutputTimeFiles(const std::string& outputDir);

private:
	static std::mutex outputMutex;
	static ImageReader imageReader;
	static thread_local CSIFTCPUExtractor* SIFTCPUExtractor;
	static thread_local CSIFTGPUExtractor* SIFTGPUExtractor;
	static thread_local CSIFTCPUMatcher* SIFTCPUMatcher;
	static thread_local CSIFTGPUMatcher* SIFTGPUMatcher;
	static CPythonExtractor* PYTHONExtractor;
	static CPythonMatcher* PYTHONMatcher;
	static CGlobalFeatureExtractor* globalFeatureExtractor;
	static Reconstructor* reconstructor;
	static CGlobalFeatureRetriever* globalFeatureRetriever;
	static std::atomic_bool isBusy;
	static size_t retrievalTopN;

	static Database* database;
	//static COptions* options;

	static std::size_t numReadThreads;
	static std::size_t numGPUThreads;
	static std::size_t numSIFTExtractCPUThreads;
	static std::size_t numSIFTMatchCPUThreads;
	static std::size_t numEstimateThreads;

	static std::vector<std::thread> readThreads;
	static std::vector<std::thread> GPUThreads;
	static std::vector<std::thread> SIFTExtractCPUThreads;
	static std::vector<std::thread> SIFTMatchCPUThreads;
	static std::vector<std::thread> estimateThreads;
	static std::thread loadPythonThread;
	static std::thread retrievalThread;
	static std::thread reconstructThread;
	static std::thread writeRegThread;

	static std::atomic_bool isStop;
	static std::size_t reconstructNextImageIndex;

	static std::unordered_map<std::size_t, std::string> allImages;
	static std::unordered_map<std::string, std::size_t> allImagePaths;


	static tbb::concurrent_map<std::size_t, std::atomic_size_t> SIFTExtractionTimes;
	static tbb::concurrent_map<std::size_t, std::atomic_size_t> globalFeatureExtractionTimes;
	static tbb::concurrent_map<std::size_t, std::atomic_size_t> globalFeatureRetrievalTimes;
	static tbb::concurrent_map<std::size_t, std::atomic_size_t> SIFTMatchingTimes;
	static tbb::concurrent_map<std::size_t, std::atomic_size_t> geometricVerificationTimes;


#ifdef _DEBUG

	// 任务队列
	static std::queue<std::string> readTasks;
	static std::queue<std::pair<Bitmap, std::string>> SIFTExtractCPUTasks;
	static std::queue<std::pair<Bitmap, std::string>> SIFTExtractGPUTasks;
	static std::queue<std::pair<std::size_t, std::size_t>> SIFTMatchCPUTasks;
	static std::queue<std::pair<std::size_t, std::size_t>> SIFTMatchGPUTasks;
	static std::queue<std::pair<std::size_t, std::size_t>> estimateTasks;
	static std::unordered_set<std::size_t> reconstructionTasks;
	static std::unordered_map<std::string, CImageStatusFlag> imagesStatus;
	static std::unordered_map<std::size_t, std::unordered_set<std::size_t>> matchingPairs;


	// 互斥锁
	static std::mutex readTasks_Mu;
	static std::mutex SIFTExtractCPUTasks_Mu;
	static std::mutex SIFTExtractGPUTasks_Mu;
	static std::mutex SIFTMatchCPUTasks_Mu;
	static std::mutex SIFTMatchGPUTasks_Mu;
	static std::mutex estimateTasks_Mu;
	static std::mutex imagesStatus_Mu;
	static std::mutex matchingPairs_Mu;
	static std::mutex reconstructionTasks_Mu;
#else
	static tbb::concurrent_queue<std::string> readTasks;
	static tbb::concurrent_queue<std::pair<Bitmap, std::string>> SIFTExtractCPUTasks;
	static tbb::concurrent_queue<std::pair<Bitmap, std::string>> SIFTExtractGPUTasks;
	static tbb::concurrent_queue<std::string> globalFeatureExtractionTasks;
	static tbb::concurrent_queue<std::pair<std::size_t, std::size_t>> SIFTMatchCPUTasks;
	static tbb::concurrent_queue<std::pair<std::size_t, std::size_t>> SIFTMatchGPUTasks;
	static tbb::concurrent_queue<std::pair<std::size_t, std::size_t>> estimateTasks;
	static tbb::concurrent_unordered_set<std::size_t> reconstructionTasks;
	static tbb::concurrent_queue<std::size_t> retrievalTasks;

	static tbb::concurrent_unordered_map<std::string, CImageStatusFlag> imagesStatus;
	static tbb::concurrent_unordered_map<std::size_t, std::atomic_size_t> numSIFTMatchingPairs;
	static tbb::concurrent_unordered_map<std::size_t, std::atomic_size_t> numMatchingPairs;
	static tbb::concurrent_unordered_map<std::size_t, std::vector<image_t>> RetrivalPairs;

#endif
	static void Read();
	static void GPU();
	static void PyGPU();
	static void SIFTExtractCPU();
	static void SIFTMatchCPU();
	static void Estimate();
	static void Reconstruct();
	static void WriteReg();
	static void WriteReconstructStatus(bool status);
	static void Retrieval();
};