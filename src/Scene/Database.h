#pragma once
#include "../Base/types.h"
#include "../Geometry/TwoViewGeometry.h"
#include "Camera.h"
#include "Image.h"
#include <unordered_map>
#include <unordered_set>

#ifndef _DEBUG
#undef slots
#undef emit
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_unordered_set.h>
#include <tbb/concurrent_vector.h>
#define slots Q_SLOTS
#define emit Q_EMIT
#endif // !_DEBUG

#include <faiss/IndexHNSW.h>

class Database final
{
public:
	// The maximum number of images, that can be stored in the database.
	// This limitation arises due to the fact, that we generate unique IDs for
	// image pairs manually. Note: do not change this to
	// another type than `size_t`.
	const static size_t kMaxNumImages;
	static std::string imageDir;
	static std::string exportPath;

	Database();
	~Database();

	bool ExistsCamera(camera_t camera_id) const;
	bool ExistsCamera(const Camera& camera, camera_t& cameraID)const;
	bool ExistsImage(image_t image_id) const;
	bool ExistsImageWithName(const std::string& name) const;
	bool ExistsKeypoints(image_t image_id) const;
	bool ExistsGlobalFeature(image_t image_id) const;
	bool ExistsDescriptors(image_t image_id) const;
	bool ExistsMatches(image_t image_id1, image_t image_id2) const;
	bool ExistsInlierMatches(image_t image_id1, image_t image_id2) const;

	// Number of rows in `cameras` table.
	size_t NumCameras() const;

	//  Number of rows in `images` table.
	size_t NumImages() const;

	// Number of descriptors for specific image.
	size_t NumKeypointsForImage(image_t image_id) const;

	// Number of descriptors for specific image.
	size_t NumDescriptorsForImage(image_t image_id) const;

	// Sum of `rows` column in `matches` table, i.e. number of total matches.
	size_t NumMatches() const;

	// Sum of `rows` column in `two_view_geometries` table,
	// i.e. number of total inlier matches.
	size_t NumInlierMatches() const;

	// Number of rows in `matches` table.
	size_t NumMatchedImagePairs() const;

	// Number of rows in `two_view_geometries` table.
	size_t NumVerifiedImagePairs() const;

	Eigen::Vector4f CalculatePlaneColor(image_t imageID);


	// Each image pair is assigned an unique ID in the `matches` and
	// `two_view_geometries` table. We intentionally avoid to store the pairs in a
	// separate table by using e.g. AUTOINCREMENT, since the overhead of querying
	// the unique pair ID is significant.
	inline static image_pair_t ImagePairToPairId(image_t image_id1, image_t image_id2)
	{
		CHECK_LT(image_id1, kMaxNumImages);
		CHECK_LT(image_id2, kMaxNumImages);
		if (SwapImagePair(image_id1, image_id2)) {
			return static_cast<image_pair_t>(kMaxNumImages) * image_id2 + image_id1;
		}
		else {
			return static_cast<image_pair_t>(kMaxNumImages) * image_id1 + image_id2;
		}
	}

	inline static void PairIdToImagePair(image_pair_t pair_id, image_t* image_id1, image_t* image_id2)
	{
		*image_id2 = static_cast<image_t>(pair_id % kMaxNumImages);
		*image_id1 = static_cast<image_t>((pair_id - *image_id2) / kMaxNumImages);
		CHECK_LT(*image_id1, kMaxNumImages);
		CHECK_LT(*image_id2, kMaxNumImages);
	}

	// Return true if image pairs should be swapped. Used to enforce a specific
	// image order to generate unique image pair identifiers independent of the
	// order in which the image identifiers are used.
	inline static bool SwapImagePair(image_t image_id1, image_t image_id2)
	{
		return image_id1 > image_id2;
	}

	// Read an existing entry in the database. The user is responsible for making
	// sure that the entry actually exists. For image pairs, the order of
	// `image_id1` and `image_id2` does not matter.
	Camera ReadCamera(camera_t camera_id) const;
	std::vector<Camera> ReadAllCameras() const;

	Image ReadImage(image_t image_id) const;
	Image ReadImageWithName(const std::string& name) const;
	std::vector<Image> ReadAllImages() const;

	FeatureKeypoints ReadKeypoints(image_t image_id) const;
	FeatureDescriptors ReadDescriptors(image_t image_id) const;
	GlobalFeature ReadGlobalFeature(image_t image_id) const;

	FeatureMatches ReadMatches(image_t image_id1, image_t image_id2) const;
	std::vector<std::pair<image_pair_t, FeatureMatches>> ReadAllMatches() const;

	TwoViewGeometry ReadTwoViewGeometry(image_t image_id1, image_t image_id2) const;
	void ReadTwoViewGeometries(std::vector<image_pair_t>* image_pair_ids, std::vector<TwoViewGeometry>* two_view_geometries) const;

	// Read all image pairs that have an entry in the `NumVerifiedImagePairs`
	// table with at least one inlier match and their number of inlier matches.
	void ReadTwoViewGeometryNumInliers(
		std::vector<std::pair<image_t, image_t>>* image_pairs,
		std::vector<int>* num_inliers) const;

	// Add new camera and return its database identifier. If `use_camera_id`
	// is false a new identifier is automatically generated.
	camera_t WriteCamera(const Camera& camera, bool use_camera_id = false);

	// Add new image and return its database identifier. If `use_image_id`
	// is false a new identifier is automatically generated.
	image_t WriteImage(const Image& image, bool use_image_id = false);

	// Write a new entry in the database. The user is responsible for making sure
	// that the entry does not yet exist. For image pairs, the order of
	// `image_id1` and `image_id2` does not matter.
	void WriteKeypoints(image_t image_id,
		const FeatureKeypoints& keypoints);
	void WriteGlobalFeature(image_t image_id, const GlobalFeature& globalFeature);

	void WriteDescriptors(image_t image_id,
		const FeatureDescriptors& descriptors);
	void WriteMatches(image_t image_id1,
		image_t image_id2,
		const FeatureMatches& matches);
	void WriteTwoViewGeometry(image_t image_id1,
		image_t image_id2,
		const TwoViewGeometry& two_view_geometry);

	void Exhaustive(const GlobalFeature& vec, size_t targetN, std::vector<float>& D, std::vector<faiss::idx_t>& I) const;
	void HNSW(const GlobalFeature& vec, size_t targetN, std::vector<float>& D, std::vector<faiss::idx_t>& I) const;

	// Update an existing camera in the database. The user is responsible for
	// making sure that the entry already exists.
	void UpdateCamera(const Camera& camera);

	// Update an existing image in the database. The user is responsible for
	// making sure that the entry already exists.
	void UpdateImage(const Image& image);

	// Delete matches of an image pair.
	void DeleteMatches(image_t image_id1, image_t image_id2);

	// Delete inlier matches of an image pair.
	void DeleteInlierMatches(image_t image_id1, image_t image_id2);

	// Clear all database tables
	void ClearAllTables();

	// Clear the entire cameras table
	void ClearCameras();

	// Clear the entire images, keypoints, and descriptors tables
	void ClearImages();

	// Clear the entire descriptors table
	void ClearDescriptors();

	// Clear the entire keypoints table
	void ClearKeypoints();

	// Clear the entire matches table.
	void ClearMatches();

	// Clear the entire inlier matches table.
	void ClearTwoViewGeometries();

	// Merge two databases into a single, new database.
	static void Merge(const Database& database1, const Database& database2, Database* merged_database);

	void Export(const std::string& exportPath);

private:
#ifdef _DEBUG
	std::unordered_map<camera_t, Camera> cameras;
	std::unordered_map<image_t, Image> images;
	std::unordered_map<image_t, FeatureKeypoints> keypoints;
	std::unordered_map<image_t, FeatureDescriptors> descriptors;
	std::unordered_map<image_pair_t, FeatureMatches> matches;
	std::unordered_map<image_pair_t, TwoViewGeometry> twoViewGeometries;
#else
	tbb::concurrent_unordered_map<camera_t, Camera> cameras;
	tbb::concurrent_unordered_map<image_t, Image> images;
	tbb::concurrent_unordered_map<image_t, FeatureKeypoints> keypoints;
	tbb::concurrent_unordered_map<image_t, FeatureDescriptors> descriptors;
	tbb::concurrent_vector<GlobalFeature> globalFeatures;
	tbb::concurrent_unordered_map<image_pair_t, FeatureMatches> matches;
	tbb::concurrent_unordered_map<image_pair_t, TwoViewGeometry> twoViewGeometries;
#endif

	const int d = 2048; // 向量的维数
	const int M = 32; // HNSW的M参数，控制图的连通性
	const int efConstruction = 200; // 构建过程中的ef参数，影响索引的构建时间和质量
	mutable std::mutex faissMu;
	faiss::IndexHNSWFlat* indexHNSW = nullptr;
	//faiss::IndexFlatL2* indexL2 = nullptr;
};






















