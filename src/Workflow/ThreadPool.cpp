#include <iostream>

#include "ThreadPool.h"
#include "Workflow.h"
#include "../Base/Base.h"

using namespace std;

CThreadPool::CThreadPool(CDatabase& database, COptions& options) :database(&database), options(&options)
{
	isStop = false;

	readImageThreads.reserve(numReadImageThreads);
	for (size_t i = 0; i < numReadImageThreads; i++)
	{
		readImageThreads.emplace_back(&CThreadPool::ReadImage, this);
		readImageThreads.back().detach();
	}

	GPUThreads.reserve(numGPUThreads);
	for (size_t i = 0; i < numGPUThreads; i++)
	{
		GPUThreads.emplace_back(&CThreadPool::GPUTask, this);
		GPUThreads.back().detach();
	}

	extractionSIFTCPUThreads.reserve(numExtractionSIFTCPUThreads);
	for (size_t i = 0; i < numExtractionSIFTCPUThreads; i++)
	{
		extractionSIFTCPUThreads.emplace_back(&CThreadPool::Extraction, this);
		extractionSIFTCPUThreads.back().detach();
	}

	const size_t numFindMatchPairsThreads = numGPUThreads + numExtractionSIFTCPUThreads + 1;
	findMatchPairsThreads.reserve(numFindMatchPairsThreads);
	for (size_t i = 0; i < numFindMatchPairsThreads; i++)
	{
		findMatchPairsThreads.emplace_back(&CThreadPool::FindMatchPairs, this);
		findMatchPairsThreads.back().detach();
	}




}
CThreadPool::~CThreadPool()
{
	isStop = true;
}
void CThreadPool::Stop()
{
	isStop = true;
	readImageTasks.clear();
	extractionSIFTGPUTasks.clear();
	extractionSIFTCPUTasks.clear();
}
void CThreadPool::AddReadImageTask(const string& imagePath)
{
	readImageTasks.push(imagePath);
	imageStatus.push_back(CUnread);
}
void CThreadPool::AddSIFTExtractionTask(const string& imagePath)
{
	extractionSIFTCPUTasks.push(imagePath);
}
void CThreadPool::AddSIFTMatchingTask(const string& imagePath, const vector<size_t>& matchingImageIDs)
{

	//matchingTasks.push({ imagePath, matchingImageIDs });
}

void CThreadPool::GPUTask()
{
	while (!isStop)
	{
		this_thread::sleep_for(chrono::milliseconds(GetRandomUniformInteger<size_t>(50, 80)));
		if (!database)
		{
			continue;
		}
		if (!extractionSIFTGPUTasks.empty())
		{
			string imagePath = "";
			size_t numTries = 0;
			while (!extractionSIFTGPUTasks.try_pop(imagePath) && numTries <= 5)
			{
				this_thread::sleep_for(chrono::milliseconds(10));
				numTries++;
			}
			if (!imagePath.empty())
			{
				imageStatus[database->GetImageID(GetFileName(imagePath))] = CExtracting;
				CWorkflow::SIFTExtract(imagePath, options->SIFTExtractionOptions, *database);
				imageStatus[database->GetImageID(GetFileName(imagePath))] = CExtracted;
			}
			continue;
		}
		if (!waitingMatchingSIFTGPUTasks.empty())
		{
			constexpr size_t invalidIndex = numeric_limits<size_t>::max();
			pair<size_t, size_t> imagePair({ invalidIndex,invalidIndex });
			size_t numTries = 0;
			while (!waitingMatchingSIFTGPUTasks.try_pop(imagePair) && numTries <= 5)
			{
				this_thread::sleep_for(chrono::milliseconds(10));
				numTries++;
			}
			if (imagePair.first != invalidIndex && imagePair.second != invalidIndex)
			{

			}


		}
	}
}
void CThreadPool::ReadImage()
{
	while (!isStop)
	{
		this_thread::sleep_for(chrono::milliseconds(GetRandomUniformInteger<size_t>(20, 50)));
		if (readImageTasks.empty() || !database)
		{
			continue;
		}
		string imagePath = "";
		if (!readImageTasks.try_pop(imagePath) || imagePath.empty())
		{
			continue;
		}
		CWorkflow::ReadImage(imagePath, *database);
		imageStatus[database->GetImageID(GetFileName(imagePath))] = CRead;
	}
}
void CThreadPool::Extraction()
{
	while (!isStop)
	{
		this_thread::sleep_for(chrono::milliseconds(GetRandomUniformInteger<size_t>(100, 300)));
		if (extractionSIFTCPUTasks.empty() || !database)
		{
			continue;
		}

		string imagePath = "";
		if (!extractionSIFTCPUTasks.try_pop(imagePath) || imagePath.empty())
		{
			continue;
		}
		if (options->SIFTExtractionOptions.isUseGPU)
		{
			extractionSIFTGPUTasks.push(imagePath);
		}
		else
		{
			imageStatus[database->GetImageID(GetFileName(imagePath))] = CExtracting;
			CWorkflow::SIFTExtract(imagePath, options->SIFTExtractionOptions, *database);
			imageStatus[database->GetImageID(GetFileName(imagePath))] = CExtracted;
		}
	}
}
void CThreadPool::FindMatchPairs()
{
	while (!isStop)
	{
		this_thread::sleep_for(chrono::milliseconds(GetRandomUniformInteger<size_t>(10, 30)));
		if (matchingTasks.empty() || !database)
		{
			continue;
		}
	}
}



//void CThreadPool::FindingSIFTMatchingPairs()
//{
//	while (!isStop)
//	{
//		this_thread::sleep_for(chrono::milliseconds(GetRandomUniformInteger<size_t>(10, 30)));
//		if (matchingSIFTCPUTasks.empty() || !database)
//		{
//			continue;
//		}
//		string imagePath = "";
//		if (!matchingSIFTCPUTasks.try_pop(imagePath) || imagePath.empty())
//		{
//			continue;
//		}
//		const string imageName = GetFileName(imagePath);
//		if (!database->IsImageExists(imageName))
//		{
//			continue;
//		}
//		const size_t imageID = database->GetImageID(imageName);
//
//		unordered_set<size_t> extractingImages;
//		extractingImages.reserve(database->GetImagesNum());
//		for(size_t matchedImageID = 0; matchedImageID < imageID; matchedImageID++)
//		{
//			if (database->GetImageDescriptorsNum(matchedImageID) == 0)
//			{
//				extractingImages.insert(matchedImageID);
//				continue;
//			}
//			if (options->SIFTMatchingOptions.isUseGPU)
//			{
//				waitingMatchingSIFTGPUTasks.push({ matchedImageID ,imageID });
//			}
//			else
//			{
//				waitingMatchingSIFTCPUTasks.push({ matchedImageID ,imageID });
//			}
//		}
//
//		auto startWait = chrono::high_resolution_clock::now();
//		while (!isStop && !extractingImages.empty())
//		{
//			if (chrono::duration_cast<chrono::seconds>(chrono::high_resolution_clock::now() - startWait).count() >= 5)
//			{
//				CLog::Log((boost::format("[FindingSIFTMatchingPairs] Waiting timed out!")).str());
//				break;
//			}
//			for (const size_t matchedImageID : extractingImages)
//			{
//				if (database->GetImageDescriptorsNum(matchedImageID) > 0)
//				{
//					extractingImages.erase(matchedImageID);
//					if (options->SIFTMatchingOptions.isUseGPU)
//					{
//						waitingMatchingSIFTGPUTasks.push({ matchedImageID ,imageID });
//					}
//					else
//					{
//						waitingMatchingSIFTCPUTasks.push({ matchedImageID ,imageID });
//					}
//					startWait = chrono::high_resolution_clock::now();
//					break;
//				}
//			}
//		}
//
//	}
//}