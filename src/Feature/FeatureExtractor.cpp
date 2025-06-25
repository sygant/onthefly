#include "FeatureExtractor.h"
#include <GL/GL.h>
#include <glog/logging.h>
#include <cuda_runtime.h>
#include <filesystem>
#include <iostream>
#include "../Base/misc.h"
#include "PyLoader.h"
using namespace std;


// VLFeat uses a different convention to store its descriptors. This transforms
// the VLFeat format into the original SIFT format that is also used by SiftGPU.
FeatureDescriptors TransformVLFeatToUBCFeatureDescriptors(const FeatureDescriptors& vlfeat_descriptors)
{
    FeatureDescriptors ubc_descriptors(vlfeat_descriptors.rows(),
        vlfeat_descriptors.cols());
    const std::array<int, 8> q{ {0, 7, 6, 5, 4, 3, 2, 1} };
    for (FeatureDescriptors::Index n = 0; n < vlfeat_descriptors.rows(); ++n) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                for (int k = 0; k < 8; ++k) {
                    ubc_descriptors(n, 8 * (j + 4 * i) + q[k]) =
                        vlfeat_descriptors(n, 8 * (j + 4 * i) + k);
                }
            }
        }
    }
    return ubc_descriptors;
}

CSIFTCPUExtractor::CSIFTCPUExtractor(const CSIFTExtractionOptions& options) :sift(nullptr, &vl_sift_delete)
{
	this->options = options;
}
bool CSIFTCPUExtractor::Extract(const Bitmap& bitmap, FeatureKeypoints& keypoints, FeatureDescriptors& descriptors)
{
	CHECK(bitmap.IsGrey());

    if (sift == nullptr || sift->width != bitmap.Width() || sift->height != bitmap.Height())
    {
        sift = VlSiftType(vl_sift_new(bitmap.Width(), bitmap.Height(), options.numOctaves, options.octaveResolution, -1), &vl_sift_delete);
        if (!sift)
        {
            return false; // 创建失败, 返回false
        }
    }

    // 设置sift的峰值和边缘阈值
    vl_sift_set_peak_thresh(sift.get(), options.peakThreshold);
    vl_sift_set_edge_thresh(sift.get(), options.edgeThreshold);

    // Iterate through octaves.
    std::vector<size_t> level_num_features;
    std::vector<FeatureKeypoints> level_keypoints;
    std::vector<FeatureDescriptors> level_descriptors;
    bool first_octave = true;
    while (true) {
        if (first_octave) {
            const std::vector<uint8_t> data_uint8 = bitmap.ConvertToRowMajorArray();
            std::vector<float> data_float(data_uint8.size());
            for (size_t i = 0; i < data_uint8.size(); ++i) {
                data_float[i] = static_cast<float>(data_uint8[i]) / 255.0f;
            }
            if (vl_sift_process_first_octave(sift.get(), data_float.data())) {
                break;
            }
            first_octave = false;
        }
        else {
            if (vl_sift_process_next_octave(sift.get())) {
                break;
            }
        }

        // Detect keypoints.
        vl_sift_detect(sift.get());

        // Extract detected keypoints.
        const VlSiftKeypoint* vl_keypoints = vl_sift_get_keypoints(sift.get());
        const int num_keypoints = vl_sift_get_nkeypoints(sift.get());
        if (num_keypoints == 0) {
            continue;
        }

        // Extract features with different orientations per DOG level.
        size_t level_idx = 0;
        int prev_level = -1;
        FeatureDescriptorsFloat desc(1, 128);
        for (int i = 0; i < num_keypoints; ++i)
        {
            if (vl_keypoints[i].is != prev_level)
            {
                if (i > 0)
                {
                    // Resize containers of previous DOG level.
                    level_keypoints.back().resize(level_idx);
                    level_descriptors.back().conservativeResize(level_idx, 128);
                }

                // Add containers for new DOG level.
                level_idx = 0;
                level_num_features.push_back(0);
                level_keypoints.emplace_back(options.maxNumOrientations * num_keypoints);
                level_descriptors.emplace_back(options.maxNumOrientations * num_keypoints, 128);
            }

            level_num_features.back() += 1;
            prev_level = vl_keypoints[i].is;

            // Extract feature orientations.
            double angles[4];
            int num_orientations;
            if (options.isUpRight)
            {
                num_orientations = 1;
                angles[0] = 0.0;
            }
            else
            {
                num_orientations = vl_sift_calc_keypoint_orientations(
                    sift.get(), angles, &vl_keypoints[i]);
            }

            // Note that this is different from SiftGPU, which selects the top
            // global maxima as orientations while this selects the first two
            // local maxima. It is not clear which procedure is better.
            const int num_used_orientations = std::min(num_orientations, (int)options.maxNumOrientations);

            for (int o = 0; o < num_used_orientations; ++o) {
                level_keypoints.back()[level_idx] =
                    FeatureKeypoint(vl_keypoints[i].x + 0.5f,
                        vl_keypoints[i].y + 0.5f,
                        vl_keypoints[i].sigma,
                        angles[o]);
                vl_sift_calc_keypoint_descriptor(sift.get(), desc.data(), &vl_keypoints[i], angles[o]);
                if (options.normalizationType == CSIFTNormalizationType::L2)
                {
                    L2NormalizeFeatureDescriptors(&desc);
                }
                else if (options.normalizationType == CSIFTNormalizationType::L1_ROOT)
                {
                    L1RootNormalizeFeatureDescriptors(&desc);
                }
                else
                {
                    LOG(FATAL) << "Normalization type not supported";
                }

                level_descriptors.back().row(level_idx) = FeatureDescriptorsToUnsignedByte(desc);

                level_idx += 1;
            }
        }

        // Resize containers for last DOG level in octave.
        level_keypoints.back().resize(level_idx);
        level_descriptors.back().conservativeResize(level_idx, 128);
    }

    // Determine how many DOG levels to keep to satisfy max_num_features option.
    int first_level_to_keep = 0;
    int num_features = 0;
    int num_features_with_orientations = 0;
    for (int i = level_keypoints.size() - 1; i >= 0; --i) {
        num_features += level_num_features[i];
        num_features_with_orientations += level_keypoints[i].size();
        if (num_features > options.maxNumFeatures) 
        {
            first_level_to_keep = i;
            break;
        }
    }

    // Extract the features to be kept.
    {
        size_t k = 0;
        keypoints.resize(num_features_with_orientations);
        for (size_t i = first_level_to_keep; i < level_keypoints.size(); ++i) {
            for (size_t j = 0; j < level_keypoints[i].size(); ++j) {
                keypoints[k] = level_keypoints[i][j];
                k += 1;
            }
        }
    }

    // Compute the descriptors for the detected keypoints.
    size_t k = 0;
    descriptors.resize(num_features_with_orientations, 128);
    for (size_t i = first_level_to_keep; i < level_keypoints.size(); ++i) {
        for (size_t j = 0; j < level_keypoints[i].size(); ++j) {
            descriptors.row(k) = level_descriptors[i].row(j);
            k += 1;
        }
    }
    descriptors = TransformVLFeatToUBCFeatureDescriptors(descriptors);

    return true;
}

CSIFTGPUExtractor::CSIFTGPUExtractor(const CSIFTExtractionOptions& options)
{
    this->options = options;

    vector<const char*> siftGPU_Args;
    siftGPU_Args.push_back("./sift_gpu");
    siftGPU_Args.push_back("-cuda");

    size_t cudaDeviceIndex = SetBestCudaDevice();

    string cudaDeviceIndex_String = to_string(cudaDeviceIndex);
    string maxImageSize_String = to_string(options.maxImageSize * 2);
    string maxNumFeatures_String = to_string(options.maxNumFeatures);
    string octaveResolution_String = to_string(options.octaveResolution);
    string peakThreshold_String = to_string(options.peakThreshold);
    string edgeThreshold_String = to_string(options.edgeThreshold);
    string maxNumOrientations_String = to_string(options.maxNumOrientations);


    siftGPU_Args.push_back(cudaDeviceIndex_String.c_str());

    siftGPU_Args.push_back("-v");
    siftGPU_Args.push_back("0");
    siftGPU_Args.push_back("-maxd");
    siftGPU_Args.push_back(maxImageSize_String.c_str());
    siftGPU_Args.push_back("-tc2");
    siftGPU_Args.push_back(maxNumFeatures_String.c_str());
    siftGPU_Args.push_back("-fo");
    siftGPU_Args.push_back("-1");
    siftGPU_Args.push_back("-d");
    siftGPU_Args.push_back(octaveResolution_String.c_str());
    siftGPU_Args.push_back("-t");
    siftGPU_Args.push_back(peakThreshold_String.c_str());
    siftGPU_Args.push_back("-e");
    siftGPU_Args.push_back(edgeThreshold_String.c_str());

    if (options.isUpRight)
    {
        siftGPU_Args.push_back("-ofix");
        siftGPU_Args.push_back("-mo");
        siftGPU_Args.push_back("1");
    }
    else
    {
        siftGPU_Args.push_back("-mo");
        siftGPU_Args.push_back(maxNumOrientations_String.c_str());
    }
    siftGPU.ParseParam(siftGPU_Args.size(), siftGPU_Args.data());
    siftGPU.gpu_index = cudaDeviceIndex;

    CHECK(siftGPU.VerifyContextGL() == SiftGPU::SIFTGPU_FULL_SUPPORTED);
}
bool CSIFTGPUExtractor::Extract(const Bitmap& bitmap, FeatureKeypoints& keypoints, FeatureDescriptors& descriptors)
{
    CHECK(bitmap.IsGrey());

    // Note the max dimension of SiftGPU is the maximum dimension of the
    // first octave in the pyramid (which is the 'first_octave').
    CHECK_EQ(options.maxImageSize * 2, siftGPU.GetMaxDimension());

    // Note, that this produces slightly different results than using SiftGPU
    // directly for RGB->GRAY conversion, since it uses different weights.
    const std::vector<uint8_t> bitmap_raw_bits = bitmap.ConvertToRawBits();
    const int code = siftGPU.RunSIFT(bitmap.ScanWidth(),
        bitmap.Height(),
        bitmap_raw_bits.data(),
        GL_LUMINANCE,
        GL_UNSIGNED_BYTE);

    const int kSuccessCode = 1;
    if (code != kSuccessCode) {
        return false;
    }

    const size_t num_features = static_cast<size_t>(siftGPU.GetFeatureNum());
    std::vector<SiftKeypoint> keypoints_buffer_;
    keypoints_buffer_.resize(num_features);

    FeatureDescriptorsFloat descriptors_float(num_features, 128);

    // Download the extracted keypoints and descriptors.
    siftGPU.GetFeatureVector(keypoints_buffer_.data(), descriptors_float.data());

    keypoints.resize(num_features);
    for (size_t i = 0; i < num_features; ++i)
    {
        keypoints[i] = FeatureKeypoint(keypoints_buffer_[i].x, keypoints_buffer_[i].y, keypoints_buffer_[i].s, keypoints_buffer_[i].o);
    }

    // Save and normalize the descriptors.
    if (options.normalizationType == CSIFTNormalizationType::L2) {
        L2NormalizeFeatureDescriptors(&descriptors_float);
    }
    else if (options.normalizationType == CSIFTNormalizationType::L1_ROOT) {
        L1RootNormalizeFeatureDescriptors(&descriptors_float);
    }
    else {
        LOG(FATAL) << "Normalization type not supported";
    }

    descriptors = FeatureDescriptorsToUnsignedByte(descriptors_float);

    return true;
}

CGlobalFeatureExtractor::CGlobalFeatureExtractor(const CGlobalFeatureExtractonOptions& options)
{
    const std::filesystem::path currentPath = std::filesystem::current_path();
    if (!ExistsFile(options.pthPath) || !ExistsFile(options.HDF5Path) || !ExistsFile(options.PCAModelPath) || !ExistsFile(options.checkPointPath))
    {
        const std::filesystem::path targetFolder = currentPath / "GlobalFeature";
        if (std::filesystem::exists(targetFolder) && std::filesystem::is_directory(targetFolder))
        {
            const std::string baseDir = StringReplace(EnsureTrailingSlash(currentPath.string()), "\\", "/");
            this->options.pthPath = baseDir + "GlobalFeature/vgg16-397923af.pth";
            this->options.HDF5Path = baseDir + "GlobalFeature/VGG16_64_desc_cen.hdf5";
            this->options.checkPointPath = baseDir + "GlobalFeature/VGG16_NetVlad_NoSplit.pth.tar";
            this->options.PCAModelPath = baseDir + "GlobalFeature/PCA_dims32768to2048.model";
            /*std::thread initializeThread(&CGlobalFeatureExtractor::Initialize, this);
            initializeThread.detach();*/
            Initialize();
            return;
        }
        cout << "The file does not exist or the file path is incorrect!" << endl;
        return;
    }
    this->options = options;

    /*std::thread initializeThread(&CGlobalFeatureExtractor::Initialize, this);
    initializeThread.detach();*/
    Initialize();
}
CGlobalFeatureExtractor::~CGlobalFeatureExtractor()
{
    /*if (extractFunc)
    {
        if (!PyGILState_Check())
        {
            state = PyGILState_Ensure();
        }
        Py_DECREF(extractFunc);
        PyGILState_Release(state);
    }
    Py_Finalize();*/
}
bool CGlobalFeatureExtractor::Extract(const std::string& imagePath, GlobalFeature& globalFeature)
{
    if (isInitializeError)
    {
        cout << "Initialization error!" << endl;
        return false;
    }
    //cout << "start extract global features: "<<imagePath<<endl;
    PyObject* Arg = PyTuple_Pack(1 , PyUnicode_FromString(imagePath.c_str()));
    //PyObject* globalFeatureResultPtr = PyObject_CallObject(PyLoader::GlobalExtractorFunc, Arg);
    PyObject* globalFeatureResultPtr = PyObject_CallOneArg(PyLoader::GlobalExtractorFunc, PyUnicode_FromString(imagePath.c_str()));
    Py_DECREF(Arg);
    if (globalFeatureResultPtr)
    {
        PyArrayObject* array = reinterpret_cast<PyArrayObject*>( globalFeatureResultPtr );
        double* data = reinterpret_cast<double*>( PyArray_DATA(array) );
        globalFeature = GlobalFeature(globalFeatureDim);
        for (int i = 0; i < globalFeatureDim; i++)
        {
            globalFeature[i] = static_cast<float>( data[i] );
        }
        Py_DECREF(globalFeatureResultPtr);
        return true;
    }
    cout << "Unable to get return value" << endl;
    PyErr_Print();
    return false;
}
void CGlobalFeatureExtractor::Initialize()
{
    if (PyLoader::GlobalExtractorFunc) {
        isInitializeError = false;
    }
    else {
        isInitializeError = true;
    }
}
CPythonExtractor::CPythonExtractor(){
    
}
PyObject* CPythonExtractor::Extract(
    const std::string& imagePath , 
    FeatureKeypoints& keypoints , 
    FeatureDescriptors& descriptors)
{
    //////////  Superpoint    //////////
    //cout<<"superpoint extract image: "<< imagePath <<endl;
    PyObject* Arg = PyTuple_Pack(1 , PyUnicode_FromString(imagePath.c_str()));
    PyObject* featureDictPtr = PyObject_CallObject(PyLoader::LocalExtractorFunc , Arg);
    Py_XDECREF(Arg);
    if (featureDictPtr) {
        PyObject* keypointsPyPtr = PyDict_GetItemString(featureDictPtr , "keypoints");
        PyObject* descriptorsPyPtr = PyDict_GetItemString(featureDictPtr , "descriptors");
        if(keypointsPyPtr){
            PyArrayObject* npArray = reinterpret_cast<PyArrayObject*>( keypointsPyPtr );
            npy_intp* shape = PyArray_SHAPE(npArray);
            const size_t num_features = shape[0];
            keypoints.resize(num_features);
            //cout << "features number: " << num_features << endl;
            for (npy_intp i = 0; i < shape[0]; ++i) {
                float x_value = *reinterpret_cast<float*>( PyArray_GETPTR2(npArray , i , 0) );
                float y_value = *reinterpret_cast<float*>( PyArray_GETPTR2(npArray , i , 1) );
                keypoints[i] = FeatureKeypoint(x_value,y_value);
            }
        }
        if (descriptorsPyPtr) {
            PyArrayObject* npArray = reinterpret_cast<PyArrayObject*>( descriptorsPyPtr );
            npy_intp* shape = PyArray_SHAPE(npArray);
            const size_t feature_len = shape[0];
            const size_t num_features = shape[1];
            descriptors.resize(num_features , feature_len);
            for (npy_intp i = 0; i < shape[1]; ++i) {
                for (npy_intp j = 0; j < shape[0]; ++j) {
                    float value = *reinterpret_cast<float*>( PyArray_GETPTR2(npArray , j , i) );
                    descriptors(i , j) = value;
                }
            }
            
        }
    }
    else {
        cout<<"features null\n";
        return nullptr;
    }
    return featureDictPtr;
}
void CPythonExtractor::SaveFeatures(const std::string& imageName , PyObject* feat) {
    PyObject* Arg = PyTuple_New(2);
    PyTuple_SetItem(Arg , 0 , Py_BuildValue("s" , imageName.c_str()));
    PyTuple_SetItem(Arg , 1 , feat);
    PyObject_Call(PyLoader::LocalFeatureSaveFunc , Arg , nullptr);
    Py_XDECREF(Arg);
}