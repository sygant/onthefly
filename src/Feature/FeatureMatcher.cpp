#include "FeatureMatcher.h"
#include "../Scene/Database.h"
#include "PyLoader.h"
using namespace std;

Eigen::MatrixXi ComputeSiftDistanceMatrix(
	const FeatureKeypoints* keypoints1,
	const FeatureKeypoints* keypoints2,
	const FeatureDescriptors& descriptors1,
	const FeatureDescriptors& descriptors2,
	const std::function<bool(float, float, float, float)>& guided_filter)
{
	if (guided_filter != nullptr) 
	{
		CHECK_NOTNULL(keypoints1);
		CHECK_NOTNULL(keypoints2);
		CHECK_EQ(keypoints1->size(), descriptors1.rows());
		CHECK_EQ(keypoints2->size(), descriptors2.rows());
	}

	const Eigen::Matrix<int, Eigen::Dynamic, 128> descriptors1_int =
		descriptors1.cast<int>();
	const Eigen::Matrix<int, Eigen::Dynamic, 128> descriptors2_int =
		descriptors2.cast<int>();

	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dists(
		descriptors1.rows(), descriptors2.rows());

	for (FeatureDescriptors::Index i1 = 0; i1 < descriptors1.rows(); ++i1) {
		for (FeatureDescriptors::Index i2 = 0; i2 < descriptors2.rows(); ++i2) {
			if (guided_filter != nullptr && guided_filter((*keypoints1)[i1].x,
				(*keypoints1)[i1].y,
				(*keypoints2)[i2].x,
				(*keypoints2)[i2].y)) {
				dists(i1, i2) = 0;
			}
			else {
				dists(i1, i2) = descriptors1_int.row(i1).dot(descriptors2_int.row(i2));
			}
		}
	}

	return dists;
}
size_t FindBestMatchesOneWayBruteForce(const Eigen::MatrixXi& dists,
	const float max_ratio,
	const float max_distance,
	std::vector<int>* matches) {
	// SIFT descriptor vectors are normalized to length 512.
	const float kDistNorm = 1.0f / (512.0f * 512.0f);

	size_t num_matches = 0;
	matches->resize(dists.rows(), -1);

	for (Eigen::Index i1 = 0; i1 < dists.rows(); ++i1) {
		int best_i2 = -1;
		int best_dist = 0;
		int second_best_dist = 0;
		for (Eigen::Index i2 = 0; i2 < dists.cols(); ++i2) {
			const int dist = dists(i1, i2);
			if (dist > best_dist) {
				best_i2 = i2;
				second_best_dist = best_dist;
				best_dist = dist;
			}
			else if (dist > second_best_dist) {
				second_best_dist = dist;
			}
		}

		// Check if any match found.
		if (best_i2 == -1) {
			continue;
		}

		const float best_dist_normed =
			std::acos(std::min(kDistNorm * best_dist, 1.0f));

		// Check if match distance passes threshold.
		if (best_dist_normed > max_distance) {
			continue;
		}

		const float second_best_dist_normed =
			std::acos(std::min(kDistNorm * second_best_dist, 1.0f));

		// Check if match passes ratio test. Keep this comparison >= in order to
		// ensure that the case of best == second_best is detected.
		if (best_dist_normed >= max_ratio * second_best_dist_normed) {
			continue;
		}

		num_matches += 1;
		(*matches)[i1] = best_i2;
	}

	return num_matches;
}
void FindBestMatchesBruteForce(const Eigen::MatrixXi& dists,
	const float max_ratio,
	const float max_distance,
	const bool cross_check,
	FeatureMatches* matches) {
	matches->clear();

	std::vector<int> matches12;
	const size_t num_matches12 = FindBestMatchesOneWayBruteForce(
		dists, max_ratio, max_distance, &matches12);

	if (cross_check) {
		std::vector<int> matches21;
		const size_t num_matches21 = FindBestMatchesOneWayBruteForce(
			dists.transpose(), max_ratio, max_distance, &matches21);
		matches->reserve(std::min(num_matches12, num_matches21));
		for (size_t i1 = 0; i1 < matches12.size(); ++i1) {
			if (matches12[i1] != -1 && matches21[matches12[i1]] != -1 &&
				matches21[matches12[i1]] == static_cast<int>(i1)) {
				FeatureMatch match;
				match.point2D_idx1 = i1;
				match.point2D_idx2 = matches12[i1];
				matches->push_back(match);
			}
		}
	}
	else {
		matches->reserve(num_matches12);
		for (size_t i1 = 0; i1 < matches12.size(); ++i1) {
			if (matches12[i1] != -1) {
				FeatureMatch match;
				match.point2D_idx1 = i1;
				match.point2D_idx2 = matches12[i1];
				matches->push_back(match);
			}
		}
	}
}

void FindNearestNeighborsFlann(
	const FeatureDescriptors& query,
	const FeatureDescriptors& index,
	const flann::Index<flann::L2<uint8_t>>& flann_index,
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>*
	indices,
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>*
	distances) {
	if (query.rows() == 0 || index.rows() == 0) {
		return;
	}

	constexpr size_t kNumNearestNeighbors = 2;
	constexpr size_t kNumLeafsToVisit = 128;

	const size_t num_nearest_neighbors =
		std::min(kNumNearestNeighbors, static_cast<size_t>(index.rows()));

	indices->resize(query.rows(), num_nearest_neighbors);
	distances->resize(query.rows(), num_nearest_neighbors);
	const flann::Matrix<uint8_t> query_matrix(
		const_cast<uint8_t*>(query.data()), query.rows(), 128);

	flann::Matrix<int> indices_matrix(
		indices->data(), query.rows(), num_nearest_neighbors);
	std::vector<float> distances_vector(query.rows() * num_nearest_neighbors);
	flann::Matrix<float> distances_matrix(
		distances_vector.data(), query.rows(), num_nearest_neighbors);
	flann_index.knnSearch(query_matrix,
		indices_matrix,
		distances_matrix,
		num_nearest_neighbors,
		flann::SearchParams(kNumLeafsToVisit));

	for (Eigen::Index query_idx = 0; query_idx < indices->rows(); ++query_idx) {
		for (Eigen::Index k = 0; k < indices->cols(); ++k) {
			const Eigen::Index index_idx = indices->coeff(query_idx, k);
			(*distances)(query_idx, k) = query.row(query_idx).cast<int>().dot(
				index.row(index_idx).cast<int>());
		}
	}
}
size_t FindBestMatchesOneWayFLANN(
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>&
	indices,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>&
	distances,
	const float max_ratio,
	const float max_distance,
	std::vector<int>* matches) {
	// SIFT descriptor vectors are normalized to length 512.
	const float kDistNorm = 1.0f / (512.0f * 512.0f);

	size_t num_matches = 0;
	matches->resize(indices.rows(), -1);

	for (int d1_idx = 0; d1_idx < indices.rows(); ++d1_idx) {
		int best_i2 = -1;
		int best_dist = 0;
		int second_best_dist = 0;
		for (int n_idx = 0; n_idx < indices.cols(); ++n_idx) {
			const int d2_idx = indices(d1_idx, n_idx);
			const int dist = distances(d1_idx, n_idx);
			if (dist > best_dist) {
				best_i2 = d2_idx;
				second_best_dist = best_dist;
				best_dist = dist;
			}
			else if (dist > second_best_dist) {
				second_best_dist = dist;
			}
		}

		// Check if any match found.
		if (best_i2 == -1) {
			continue;
		}

		const float best_dist_normed =
			std::acos(std::min(kDistNorm * best_dist, 1.0f));

		// Check if match distance passes threshold.
		if (best_dist_normed > max_distance) {
			continue;
		}

		const float second_best_dist_normed =
			std::acos(std::min(kDistNorm * second_best_dist, 1.0f));

		// Check if match passes ratio test. Keep this comparison >= in order to
		// ensure that the case of best == second_best is detected.
		if (best_dist_normed >= max_ratio * second_best_dist_normed) {
			continue;
		}

		num_matches += 1;
		(*matches)[d1_idx] = best_i2;
	}

	return num_matches;
}
void FindBestMatchesFlann(
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>&
	indices_1to2,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>&
	distances_1to2,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>&
	indices_2to1,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>&
	distances_2to1,
	const float max_ratio,
	const float max_distance,
	const bool cross_check,
	FeatureMatches* matches) {
	matches->clear();

	std::vector<int> matches12;
	const size_t num_matches12 = FindBestMatchesOneWayFLANN(
		indices_1to2, distances_1to2, max_ratio, max_distance, &matches12);

	if (cross_check && indices_2to1.rows()) {
		std::vector<int> matches21;
		const size_t num_matches21 = FindBestMatchesOneWayFLANN(
			indices_2to1, distances_2to1, max_ratio, max_distance, &matches21);
		matches->reserve(std::min(num_matches12, num_matches21));
		for (size_t i1 = 0; i1 < matches12.size(); ++i1) {
			if (matches12[i1] != -1 && matches21[matches12[i1]] != -1 &&
				matches21[matches12[i1]] == static_cast<int>(i1)) {
				FeatureMatch match;
				match.point2D_idx1 = i1;
				match.point2D_idx2 = matches12[i1];
				matches->push_back(match);
			}
		}
	}
	else {
		matches->reserve(num_matches12);
		for (size_t i1 = 0; i1 < matches12.size(); ++i1) {
			if (matches12[i1] != -1) {
				FeatureMatch match;
				match.point2D_idx1 = i1;
				match.point2D_idx2 = matches12[i1];
				matches->push_back(match);
			}
		}
	}
}

CSIFTCPUMatcher::CSIFTCPUMatcher(const CSIFTMatchingOptions& options)
{
	options.CheckOptions();
	this->options = options;
}
FeatureMatches CSIFTCPUMatcher::Match(const FeatureDescriptors& descriptors1, const FeatureDescriptors& descriptors2)
{
	CHECK(descriptors1.cols() == 128 && descriptors2.cols() == 128);
	CHECK(descriptors1.rows() != 0 && descriptors2.rows() != 0);

	FlannIndexType flannIndex1 = BuildFlannIndex(descriptors1);
	FlannIndexType flannIndex2 = BuildFlannIndex(descriptors2);

	FeatureMatches matches;
	if (!options.isUseFLANN)
	{
		const Eigen::MatrixXi distances = ComputeSiftDistanceMatrix(nullptr, nullptr, descriptors1, descriptors2, nullptr);
		FindBestMatchesBruteForce(distances, options.maxRatio, options.maxDistance, options.doCrossCheck, &matches);
		return matches;
	}
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> indices_1to2;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> distances_1to2;

	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> indices_2to1;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> distances_2to1;
	FindNearestNeighborsFlann(descriptors1, descriptors2, flannIndex2, &indices_1to2, &distances_1to2);
	if (options.doCrossCheck)
	{
		FindNearestNeighborsFlann(descriptors2, descriptors1, flannIndex1, &indices_2to1, &distances_2to1);
	}
	FindBestMatchesFlann(indices_1to2, distances_1to2, indices_2to1, distances_2to1, options.maxRatio, options.maxDistance, options.doCrossCheck, &matches);
	return matches;
}
CSIFTCPUMatcher::FlannIndexType CSIFTCPUMatcher::BuildFlannIndex(const FeatureDescriptors& descriptors)
{
	CHECK_EQ(descriptors.cols(), 128);
	const flann::Matrix<uint8_t> descriptors_matrix(
		const_cast<uint8_t*>(descriptors.data()), descriptors.rows(), 128);
	constexpr size_t kNumTreesInForest = 4;
	FlannIndexType index(descriptors_matrix, flann::KDTreeIndexParams(kNumTreesInForest));
	index.buildIndex();
	return index;
}

size_t CSIFTGPUMatcher::lastDescriptors1Index = std::numeric_limits<size_t>::max();
size_t CSIFTGPUMatcher::lastDescriptors2Index = std::numeric_limits<size_t>::max();
bool CSIFTGPUMatcher::isUploadDescriptors1 = true;
bool CSIFTGPUMatcher::isUploadDescriptors2 = true;
CSIFTGPUMatcher::CSIFTGPUMatcher(const CSIFTMatchingOptions& options)
{
	options.CheckOptions();
	this->options = options;

	SiftGPU siftGPU;
	siftGPU.SetVerbose(0);
	siftMatchGPU = SiftMatchGPU(options.maxNumMatches);

	int gpuIndex = SetBestCudaDevice();
	siftMatchGPU.SetLanguage(SiftMatchGPU::SIFTMATCH_CUDA_DEVICE0 + gpuIndex);
	CHECK(siftMatchGPU.VerifyContextGL() != 0);

	CHECK(siftMatchGPU.Allocate(options.maxNumMatches, options.doCrossCheck), "Allocate ERROR: Not enough GPU memory!");
	siftMatchGPU.gpu_index = gpuIndex;

	lastDescriptors1Index = std::numeric_limits<size_t>::max();
	lastDescriptors2Index = std::numeric_limits<size_t>::max();
	isUploadDescriptors1 = true;
	isUploadDescriptors2 = true;
}
FeatureMatches CSIFTGPUMatcher::Match(const FeatureDescriptors& descriptors1, const FeatureDescriptors& descriptors2)
{
	CHECK(descriptors1.cols() == 128 && descriptors2.cols() == 128);
	CHECK(descriptors1.rows() != 0 && descriptors2.rows() != 0);

	if (isUploadDescriptors1)
	{
		siftMatchGPU.SetDescriptors(0, descriptors1.rows(), descriptors1.data());
	}
	if (isUploadDescriptors2)
	{
		siftMatchGPU.SetDescriptors(1, descriptors2.rows(), descriptors2.data());
	}
	isUploadDescriptors1 = isUploadDescriptors2 = true; // 每Match一次之后都要求外部程序重新判断是否要上传新的descriptors

	FeatureMatches matches;
	matches.resize(static_cast<size_t>(options.maxNumMatches));
	const int num_matches = siftMatchGPU.GetSiftMatch(
		options.maxNumMatches,
		reinterpret_cast<uint32_t(*)[2]>(matches.data()),
		static_cast<float>(options.maxDistance),
		static_cast<float>(options.maxRatio),
		options.doCrossCheck);
	if (num_matches < 0) {
		std::cerr << "ERROR: Feature matching failed. This is probably caused by "
			"insufficient GPU memory. Consider reducing the maximum "
			"number of features and/or matches."
			<< std::endl;
		matches.clear();
	}
	else 
	{
		CHECK_LE(num_matches, matches.size());
		matches.resize(num_matches);
	}
	return matches;
}

CPythonMatcher::CPythonMatcher() {
	
}
FeatureMatches CPythonMatcher::Match(const std::size_t imageID1 , const std::size_t imageID2){
	PyObject* Arg1 = PyTuple_Pack(1 , PyUnicode_FromString(to_string(imageID1).c_str()));
	PyObject* feat1 = PyObject_CallObject(PyLoader::LocalFeatureReadFunc , Arg1);
	PyObject* Arg2 = PyTuple_Pack(1 , PyUnicode_FromString(to_string(imageID2).c_str()));
	PyObject* feat2 = PyObject_CallObject(PyLoader::LocalFeatureReadFunc , Arg2);
	PyObject* Arg3 = PyTuple_Pack(2,feat1,feat2);
	PyObject* matchesResultPtr = PyObject_CallObject(PyLoader::LocalMatcherFunc , Arg3);
	Py_XDECREF(Arg1); Py_XDECREF(Arg2); Py_XDECREF(Arg3);
	FeatureMatches matches;
	if (matchesResultPtr) {
		PyArrayObject* npArray = reinterpret_cast<PyArrayObject*>( matchesResultPtr );
		npy_intp* shape = PyArray_SHAPE(npArray);
		const size_t num_matches = shape[0];
		matches.resize(num_matches);
		/*cout << "match number: " << num_matches << endl;*/
		for (npy_intp i = 0; i < shape[0]; ++i) {
			int x_value = *reinterpret_cast<int*>( PyArray_GETPTR2(npArray , i , 0) );
			int y_value = *reinterpret_cast<int*>( PyArray_GETPTR2(npArray , i , 1) );
			matches[i] = FeatureMatch(x_value,y_value);
		}
		Py_XDECREF(npArray);
	}
	//Py_XDECREF(feat1);Py_XDECREF(feat2);
	return matches;
}

bool CPythonMatcher::SimilarStructureFilter(const std::string image1, const std::string image2, const std::size_t imagePairId)
{
	PyObject* Arg1 = PyTuple_Pack(3, PyUnicode_FromString(image1.c_str()), PyUnicode_FromString(image2.c_str()),PyLong_FromSize_t(imagePairId));
	PyObject* isSimilar = PyObject_CallObject(PyLoader::SimilarityMatcher, Arg1);
	Py_XDECREF(Arg1);
	return PyObject_IsTrue(isSimilar);
}

CGlobalFeatureRetriever::CGlobalFeatureRetriever(Database* database, size_t topN)
{
	CHECK(database);
	this->database = database;
	this->topN = topN;
}
std::vector<image_t> CGlobalFeatureRetriever::Retrieve(image_t imageID)
{
	const GlobalFeature globalFeature = database->ReadGlobalFeature(imageID);

	std::vector<float> distances;
	std::vector<faiss::idx_t> indices;
	database->HNSW(globalFeature, topN + 1, distances, indices);
	
	std::vector<image_t> result(0);
	result.reserve(topN);
	for (faiss::idx_t index : indices)
	{
		if (index == -1)
		{
			continue;
		}
		index++; // 数据库中的imageID是从1开始的, 检索结果indices中的序号是从0开始的
		if (index != imageID)
		{
			result.push_back(index);
		}
	}
	return result;
}
