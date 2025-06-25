#include "Reconstructor.h"
#include "../Base/timer.h"
#include "../Base/misc.h"
#include "../Base/macros.h"

#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <windows.h>
#include <algorithm>

using namespace std;

std::atomic_int Reconstructor::lastRegImageID = -1;
std::atomic_int Reconstructor::lastRegModelID = -1;

size_t GetIntersectionSize(std::vector<image_t>& v1, std::vector<image_t>& v2) {
	// 首先对两个向量进行排序
	std::sort(v1.begin(), v1.end());
	std::sort(v2.begin(), v2.end());

	// 使用std::set_intersection算法计算交集
	std::vector<image_t> intersection;
	std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(intersection));

	// 返回交集的大小
	return intersection.size();
}

Reconstructor::Reconstructor(Database* const database, ReconstructionManager* const modelManager) : database(database), modelManager(modelManager)
{
	CHECK(database);
	CHECK(modelManager);

	RegisterCallback(INITIAL_IMAGE_PAIR_REG_CALLBACK);
	RegisterCallback(NEXT_IMAGE_REG_CALLBACK);
	RegisterCallback(LAST_IMAGE_REG_CALLBACK);
	RegisterCallback(CHANGE_CURRENT_MODEL_CALLBACK);

	regFailedImages.clear();
}
void Reconstructor::ReconstructImage(size_t imageID)
{
	const size_t numImages = database->NumImages();
	regFailedImages.insert(imageID);

#ifdef DEMO_MODE
	const size_t minNumImages = 5;
#else
	const size_t minNumImages = 20;
#endif // DEMO_MODE

	if (numImages >= minNumImages)
	{
		lastRegImageID = imageID;
		TryReconstructingImages(imageID);
	}
}
int Reconstructor::TryReconstructingImages(size_t imageID)
{
	const vector<size_t> modelIDs = ChooseModel(imageID);
	atomic_bool isNeedToMerge = false;

//#pragma omp parallel for
	for (int i = 0; i < modelIDs.size(); i++)
	{
		const size_t modelID = modelIDs[i];
		const size_t originNumRegImages = modelManager->Get(modelID)->NumRegImages();
		const size_t originNumPoints3D = modelManager->Get(modelID)->NumRegImages();
		if (originNumRegImages == database->NumImages())
		{
			continue;
		}

		Reconstruct(modelID, imageID);

		isNeedToMerge = (isNeedToMerge || modelManager->Get(modelID)->NumRegImages() != originNumRegImages || modelManager->Get(modelID)->NumRegImages() != originNumPoints3D);
	}

	bool isContinue = true;
	while (true)
	{
		isContinue = false;
		for (size_t i = 0; i < modelManager->Size(); i++)
		{
			if (modelManager->Get(i)->NumRegImages() < 4)
			{
				modelManager->Delete(i);
				isContinue = true;
				break;
			}
		}
		if (!isContinue)
		{
			break;
		}
	}

#ifndef NO_MERGE_ON_RECONSTRUCTION
	if (isNeedToMerge)
	{
		mergeCount++;
		if (mergeCount >= 6)
		{
			TryMergeModels();

			mergeCount = 0;
		}
	}
#endif
	return 0;
}
void Reconstructor::TryMergeModels()
{
#ifdef NO_MERGE
	return;
#endif
	cout << "try to merge models..." << endl;
	constexpr double maxReprojectionError = 16.0;
	bool isFinished = true;
	while (true)
	{
		isFinished = true;
		for (size_t i = 0; i < modelManager->Size(); i++)
		{
			const size_t numRegImages1 = modelManager->Get(i)->NumRegImages();
			vector<image_t> regImageIDs1 = modelManager->Get(i)->RegImageIds();
			for (size_t j = 0; j < modelManager->Size(); j++)
			{
				if (i == j)
				{
					continue;
				}
				const size_t numRegImages2 = modelManager->Get(j)->NumRegImages();
				vector<image_t> regImageIDs2 = modelManager->Get(j)->RegImageIds();
				if (GetIntersectionSize(regImageIDs1, regImageIDs2) < 3)
				{
					continue;
				}

				if (numRegImages1 > numRegImages2)
				{
					if (MergeReconstructions(maxReprojectionError, *modelManager->Get(j), modelManager->Get(i).get()))
					{
						cout << "Merge model " << j + 1 << " into model " << i + 1 << " succeed! Delete model " << j + 1 << "!" << endl;
						modelManager->Delete(j);
						isFinished = false;
						Callback(NEXT_IMAGE_REG_CALLBACK);
						break;
					}
				}

				if (MergeReconstructions(maxReprojectionError, *modelManager->Get(i), modelManager->Get(j).get()))
				{
					cout << "Merge model " << i + 1 << " into model " << j + 1 << " succeed! Delete model " << i + 1 << "!" << endl;
					modelManager->Delete(i);
					isFinished = false;
					Callback(NEXT_IMAGE_REG_CALLBACK);
					break;
				}
			}
			if (!isFinished)
			{
				break;
			}
		}
		if (isFinished)
		{
			break;
		}
	}
	cout << "model merged completed!" << endl;
}
void Reconstructor::FinalGlobalBA()
{
	if (!LoadDatabase())
	{
		return;
	}
	for (size_t i = 0; i < modelManager->Size(); i++)
	{
		IncrementalMapper mapper(database_cache_);
		mapper.BeginReconstruction(modelManager->Get(i));

		IncrementalMapper::Options options;
		GlobalBundleAdjustment(options, mapper);
	}
}
bool Reconstructor::LoadDatabase()
{
#ifdef _DEBUG
	cout << "Loading database..." << endl;
#endif // DEBUG

	Timer timer;
	timer.Start();
	const size_t min_num_matches = 10;
	database_cache_ = DatabaseCache::Create(*database, min_num_matches, true, {});

#ifdef _DEBUG
	std::cout << std::endl;
	timer.PrintMinutes();
	std::cout << std::endl;
#endif // DEBUG

	if (database_cache_->NumImages() == 0) {
		std::cout << "WARNING: No images with matches found in the database."
			<< std::endl
			<< std::endl;
		return false;
	}

	return true;
}
void Reconstructor::Reconstruct(size_t modelID, size_t imageID)
{
	Timer reconstructionTimer;
	reconstructionTimer.Start();

	shared_ptr<Reconstruction> model = modelManager->Get(modelID);
	if (!LoadDatabase())
	{
		model->reconstructTime += (reconstructionTimer.ElapsedMicroSeconds() / 1000);
		return;
	}
	IncrementalMapper mapper(database_cache_);
	mapper.BeginReconstruction(model, &regFailedImages);

	IncrementalMapper::Options init_mapper_options;

	for (size_t numInitTrials = 0; numInitTrials < 2; numInitTrials++)
	{
		if (model->NumRegImages() > 0)
		{
			break;
		}

		// 只对空模型尝试初始化
		if (InitializeEmptyModel(init_mapper_options, *model, mapper))
		{
			lastRegModelID = modelID;
			Callback(CHANGE_CURRENT_MODEL_CALLBACK);
			break;
		}

		init_mapper_options.init_min_num_inliers /= 2;
		if (InitializeEmptyModel(init_mapper_options, *model, mapper))
		{
			lastRegModelID = modelID;
			Callback(CHANGE_CURRENT_MODEL_CALLBACK);
			break;
		}

		init_mapper_options.init_min_tri_angle /= 2;
	}
	if (model->NumRegImages() == 0)
	{
		mapper.EndReconstruction(true);
		return;
	}
	model->reconstructTime += (reconstructionTimer.ElapsedMicroSeconds() / 1000);

	bool isContinue = true;
	while (isContinue)
	{
		reconstructionTimer.Restart();
		isContinue = false;
		size_t registerTime = 0;
		const vector<image_t> nextImageIDs = mapper.FindNextImages(init_mapper_options);
		registerTime += reconstructionTimer.ElapsedMicroSeconds() / 1000;
		model->reconstructTime += (reconstructionTimer.ElapsedMicroSeconds() / 1000);

		if (nextImageIDs.empty())
		{
			break;
		}
		size_t numRegNextImageTrials = 0;
		for (size_t nextImageID : nextImageIDs)
		{
			numRegNextImageTrials++;
			if (numRegNextImageTrials > 10)
			{
				break;
			}

			reconstructionTimer.Restart();
			const string nextImageName = database->ReadImage(nextImageID).Name();
			if (!mapper.RegisterNextImage(init_mapper_options, nextImageID))
			{
				regFailedImages.insert(nextImageID);

				model->reconstructTime += (reconstructionTimer.ElapsedMicroSeconds() / 1000);
				cout << (boost::format("[model %1%] image %2% registered failed!") % (modelID + 1) % nextImageName).str() << endl;
				continue;
			}
			if (regFailedImages.find(nextImageID) != regFailedImages.end())
			{
				regFailedImages.erase(nextImageID);
			}
			model->reconstructTime += (reconstructionTimer.ElapsedMicroSeconds() / 1000);
			registerTime += reconstructionTimer.ElapsedMicroSeconds() / 1000;
			model->registerTimes.push_back(registerTime);
			cout << (boost::format("[model %1%] image %2% registered successfully!") % (modelID + 1) % nextImageName).str() << endl;


			reconstructionTimer.Restart();
			TriangulateImage(init_mapper_options, nextImageID, mapper);
			model->reconstructTime += (reconstructionTimer.ElapsedMicroSeconds() / 1000);
			model->triangulateTimes.push_back(reconstructionTimer.ElapsedMicroSeconds() / 1000);

			reconstructionTimer.Restart();
			LocalBundleAdjustment(init_mapper_options, nextImageID, mapper);
			ExtractColors(init_mapper_options, nextImageID, *model);
			lastRegModelID = modelID;
			Callback(CHANGE_CURRENT_MODEL_CALLBACK);
			model->reconstructTime += (reconstructionTimer.ElapsedMicroSeconds() / 1000);

#ifdef GaussianSplatting_Output
			const size_t numRegImages = modelManager->Get(modelID)->NumRegImages();
			if (modelID == 0 && numRegImages % 20 == 0 && numRegImages != 0)
			{
				std::thread exportProjectThread([&]() {
					const std::string basePath = StringReplace(EnsureTrailingSlash(GetParentDir(Database::exportPath)), "\\", "/");

					const std::string modelExportDir = basePath + "models/" + to_string(numRegImages);
					CreateDirIfNotExists(modelExportDir + "/bin", true);
					CreateDirIfNotExists(modelExportDir + "/text", true);
					modelManager->Get(modelID)->WriteBinary(modelExportDir + "/bin");
					modelManager->Get(modelID)->WriteText(modelExportDir + "/text");
					modelManager->Get(modelID)->OutputDebugResult(modelExportDir);

					cout << (boost::format("Export %1% models completed!") % numRegImages).str() << endl;
					});
				exportProjectThread.detach();
			}
#endif

			isContinue = true;
			break;
		}
	}
	mapper.EndReconstruction(false);
	// TODO: 给当前影像上色
}
bool Reconstructor::InitializeEmptyModel(const IncrementalMapper::Options& options, Reconstruction& model, IncrementalMapper& mapper)
{
	cout << "Finding good initial image pair..." << endl;

	image_t initImageID1 = numeric_limits<image_t>::max();
	image_t initImageID2 = numeric_limits<image_t>::max();
	const bool isFindInitSuccess = mapper.FindInitialImagePair(options, &initImageID1, &initImageID2);
	if (!isFindInitSuccess)
	{
		cout << "No good initial image pair found!" << endl;
		return false;
	}
	if (!database->ExistsImage(initImageID1) || !database->ExistsImage(initImageID2))
	{
		cout << (boost::format("Initial image pair %1% and %2% do not exist!") % initImageID1 % initImageID2).str() << endl;
		return false;
	}

	cout << (boost::format("Initializing with image pair %1% and %2%...") % initImageID1 % initImageID2).str() << endl;
	const bool isRegInitSuccess = mapper.RegisterInitialImagePair(options, initImageID1, initImageID2);
	if (!isRegInitSuccess)
	{
		regFailedImages.insert(initImageID1);
		regFailedImages.insert(initImageID2);
		cout << "Initialization failed!" << endl;
		return false;
	}
	if (regFailedImages.find(initImageID1) != regFailedImages.end())
	{
		regFailedImages.erase(initImageID1);
	}
	if (regFailedImages.find(initImageID2) != regFailedImages.end())
	{
		regFailedImages.erase(initImageID2);
	}
	cout << "Initialization successful!" << endl;

	GlobalBundleAdjustment(options, mapper);
	FilterPoints(options, mapper);
	FilterImages(options, mapper);

	if (model.NumRegImages() == 0 || model.NumPoints3D() == 0)
	{
		model.DeRegisterImage(initImageID1);
		model.DeRegisterImage(initImageID2);
		if (regFailedImages.find(initImageID1) != regFailedImages.end())
		{
			regFailedImages.erase(initImageID1);
		}
		if (regFailedImages.find(initImageID2) != regFailedImages.end())
		{
			regFailedImages.erase(initImageID2);
		}
		cout << "After bundle adjustment, the registered images or 3D points is empty!" << endl;
		return false;
	}
	ExtractColors(options, initImageID1, model);
	Callback(INITIAL_IMAGE_PAIR_REG_CALLBACK);
	return true;
}
vector<size_t> Reconstructor::ChooseModel(size_t imageID) const
{
	const size_t numModels = modelManager->Size();
	vector<size_t> modelIDs;
	modelIDs.reserve(numModels);
	for (size_t modelID = 0; modelID < numModels; modelID++)
	{
		if (IsImageInModel(imageID, modelID))
		{
			modelIDs.push_back(modelID);
		}
	}

	if (modelIDs.empty())
	{
		cout << "[Reconstructor] Add a new model!" << endl;
		const size_t newModelID = modelManager->Add();
		return { newModelID };
	}
	return modelIDs;
}
bool Reconstructor::IsImageInModel(size_t imageID, size_t modelID) const
{
	shared_ptr<Reconstruction> model = modelManager->Get(modelID);
	const Image image = database->ReadImage(imageID);

	const vector<image_t> regImageIDs = model->RegImageIds();
	size_t numValidConnectedImages = 0;
	for (size_t regImageID : regImageIDs)
	{
		const Image regImage = database->ReadImage(regImageID);
		
		const size_t numCorrespondences = database->ExistsInlierMatches(imageID, regImageID) ? database->ReadTwoViewGeometry(imageID, regImageID).inlier_matches.size() : 0;
		if (numCorrespondences > 10)
		{
			numValidConnectedImages++;
		}
	}
	return numValidConnectedImages >= 1;
}

void Reconstructor::GlobalBundleAdjustment(const IncrementalMapper::Options& options, IncrementalMapper& mapper)
{
	Timer globalBATimer;
	globalBATimer.Start();

	BundleAdjustmentOptions custom_ba_options;
	IncrementalMapper::Options globalBundleAdjustOptions = options;
	if (mapper.GetReconstruction()->NumRegImages() < 10)
	{
		custom_ba_options.solver_options.function_tolerance /= 10;
		custom_ba_options.solver_options.gradient_tolerance /= 10;
		custom_ba_options.solver_options.parameter_tolerance /= 10;
		custom_ba_options.solver_options.max_num_iterations *= 2;
		custom_ba_options.solver_options.max_linear_solver_iterations = 200;
	}
	custom_ba_options.solver_options.function_tolerance = 0;
	custom_ba_options.solver_options.gradient_tolerance = 1.0;
	custom_ba_options.solver_options.parameter_tolerance = 0.0;
	custom_ba_options.solver_options.max_num_iterations = 50;
	custom_ba_options.solver_options.max_linear_solver_iterations = 100;
	custom_ba_options.solver_options.minimizer_progress_to_stdout = true;

#ifndef _DEBUG
	custom_ba_options.solver_options.minimizer_progress_to_stdout = true;
	custom_ba_options.solver_options.logging_type = ceres::LoggingType::SILENT;
#endif // DEBUG

	double finalCost = 0;
	mapper.AdjustGlobalBundle(globalBundleAdjustOptions, custom_ba_options, finalCost);
	cout << "global BA final cost: " << finalCost << "px" << endl;

	mapper.GetReconstruction()->finalGlobalBATime = globalBATimer.ElapsedMicroSeconds() / 1000;
	mapper.GetReconstruction()->finalGloablBAError = finalCost;
}
void Reconstructor::LocalBundleAdjustment(const IncrementalMapper::Options& options, size_t imageID, IncrementalMapper& mapper)
{
	BundleAdjustmentOptions custom_ba_options;
	IncrementalMapper::Options localBundleAdjustOptions = options;
	IncrementalTriangulator::Options triOptions;

#ifdef USE_HIERARCHICAL_WEIGHT_BA
	const size_t numLocalBAIters = 2;
#else
	const size_t numLocalBAIters = 2;
#endif
	
	vector<double> localBAErrors;
	vector<size_t> localBATimes;

	for (size_t i = 0; i < numLocalBAIters; i++)
	{
		Timer localBATimer;
		localBATimer.Start();

		double finalCost = 0;
		const IncrementalMapper::LocalBundleAdjustmentReport report = mapper.AdjustLocalBundle(localBundleAdjustOptions, custom_ba_options, triOptions, imageID,mapper.GetModifiedPoints3D(), finalCost);
		double changed = 0;
		if (report.num_adjusted_observations > 0)
		{
			changed = (report.num_merged_observations + report.num_completed_observations + report.num_filtered_observations) * 1.0 / report.num_adjusted_observations;
		}

#ifdef _DEBUG
		cout << (boost::format("[Reconstructor] Merged observations: %1%") % report.num_merged_observations).str() << endl;
		cout << (boost::format("[Reconstructor] Completed observations: %1%") % report.num_completed_observations).str() << endl;
		cout << (boost::format("[Reconstructor] Filtered observations: %1%") % report.num_filtered_observations).str() << endl;
		cout << (boost::format("[Reconstructor] Changed observations: %1%") % changed).str() << endl;
#endif

		const size_t localBATime = localBATimer.ElapsedMicroSeconds() / 1000;
		if (changed < 0.001f)
		{
			localBATimes.push_back(localBATime);
			localBAErrors.push_back(finalCost);
			break;
		}
		custom_ba_options.loss_function_type = BundleAdjustmentOptions::LossFunctionType::TRIVIAL;

		localBATimes.push_back(localBATime);
		localBAErrors.push_back(finalCost);
	}
	mapper.ClearModifiedPoints3D();

	mapper.GetReconstruction()->localBAErrors.push_back(localBAErrors);
	mapper.GetReconstruction()->localBATimes.push_back(localBATimes);
}
size_t Reconstructor::TriangulateImage(const IncrementalMapper::Options& options, size_t imageID, IncrementalMapper& mapper)
{
	IncrementalTriangulator::Options triOptions;
	const size_t numTris = mapper.TriangulateImage(triOptions, imageID);
#ifdef _DEBUG
	cout << (boost::format("[Reconstructor] Added observations: %1%") % numTris).str() << endl;
#endif
	
	return numTris;
}
size_t Reconstructor::FilterPoints(const IncrementalMapper::Options& options, IncrementalMapper& mapper)
{
	const size_t numFilterPoints = mapper.FilterPoints(options);
#ifdef _DEBUG
	cout << (boost::format("[Reconstructor] Filtered observations: %1%") % numFilterPoints).str() << endl;
#endif
	
	return numFilterPoints;
}
size_t Reconstructor::FilterImages(const IncrementalMapper::Options& options, IncrementalMapper& mapper)
{
	const size_t numFilterImages = mapper.FilterImages(options);
#ifdef _DEBUG
	cout << (boost::format("[Reconstructor] Filtered images: %1%") % numFilterImages).str() << endl;
#endif
	
	return numFilterImages;
}
void Reconstructor::ExtractColors(const IncrementalMapper::Options& options, size_t imageID, Reconstruction& model)
{
	const string imageDir = EnsureTrailingSlash(StringReplace(Database::imageDir, "\\", "/"));
	const string imagePath = imageDir + database->ReadImage(imageID).Name();
	if (model.ExtractColorsForImage(imageID, imageDir))
	{
		return;
	}
	cout << (boost::format("[Reconstructor] Could not read image %1%") % imagePath).str() << endl;
}

void Reconstructor::Run()
{

}



