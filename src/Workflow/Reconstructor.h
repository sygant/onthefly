#pragma once
#include <corecrt_math.h>
#include "../Base/threading.h"
#include "../Scene/ReconstructionManager.h"
#include "../Estimator/IncrementalMapper.h"

class Reconstructor final : public Thread
{
public:
	enum
	{
		INITIAL_IMAGE_PAIR_REG_CALLBACK,
		NEXT_IMAGE_REG_CALLBACK,
		LAST_IMAGE_REG_CALLBACK,
		CHANGE_CURRENT_MODEL_CALLBACK
	};
	static std::atomic_int lastRegImageID;
	static std::atomic_int lastRegModelID;

	Reconstructor(Database* const database, ReconstructionManager* const modelManager);
	void ReconstructImage(size_t imageID);
	void TryMergeModels();
	void FinalGlobalBA();

private:
	Database* const database;
	ReconstructionManager* const modelManager;
	std::shared_ptr<DatabaseCache> database_cache_ = nullptr;
	size_t mergeCount = 0;
	std::unordered_set<image_t> regFailedImages;

	int TryReconstructingImages(size_t imageID);
	
	bool LoadDatabase();

	void Reconstruct(size_t modelID, size_t imageID);
	bool InitializeEmptyModel(const IncrementalMapper::Options& options, Reconstruction& model, IncrementalMapper& mapper);
	std::vector<size_t> ChooseModel(size_t imageID) const;
	bool IsImageInModel(size_t imageID, size_t modelID) const;


	void GlobalBundleAdjustment(const IncrementalMapper::Options& options, IncrementalMapper& mapper);
	void LocalBundleAdjustment(const IncrementalMapper::Options& options, size_t imageID, IncrementalMapper& mapper);
	size_t TriangulateImage(const IncrementalMapper::Options& options, size_t imageID, IncrementalMapper& mapper);
	size_t FilterPoints(const IncrementalMapper::Options& options, IncrementalMapper& mapper);
	size_t FilterImages(const IncrementalMapper::Options& options, IncrementalMapper& mapper);
	void ExtractColors(const IncrementalMapper::Options& options, size_t imageID, Reconstruction& model);
	

	void Run();
};






















