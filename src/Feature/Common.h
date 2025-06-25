#pragma once
#include "../Base/types.h"
#include <GL/GL.h>
#include <glog/logging.h>
#include <cuda_runtime.h>
#include <windows.h>


// Convert feature keypoints to vector of points.
std::vector<Eigen::Vector2d> FeatureKeypointsToPointsVector(
    const FeatureKeypoints& keypoints);

// L2-normalize feature descriptor, where each row represents one feature.
void L2NormalizeFeatureDescriptors(FeatureDescriptorsFloat* descriptors);

// L1-Root-normalize feature descriptors, where each row represents one feature.
// See "Three things everyone should know to improve object retrieval",
// Relja Arandjelovic and Andrew Zisserman, CVPR 2012.
void L1RootNormalizeFeatureDescriptors(FeatureDescriptorsFloat* descriptors);

// Convert normalized floating point feature descriptor to unsigned byte
// representation by linear scaling from range [0, 0.5] to [0, 255]. Truncation
// to a maximum value of 0.5 is used to avoid precision loss and follows the
// common practice of representing SIFT vectors.
FeatureDescriptors FeatureDescriptorsToUnsignedByte(
    const Eigen::Ref<const FeatureDescriptorsFloat>& descriptors);

// Extract the descriptors corresponding to the largest-scale features.
void ExtractTopScaleFeatures(FeatureKeypoints* keypoints,
    FeatureDescriptors* descriptors,
    size_t num_features);

// 当前计算机有多少个支持CUDA的显卡
inline size_t GetNumCudaDevices()
{
    int numCudaDevices;
    cudaGetDeviceCount(&numCudaDevices);
    return numCudaDevices;
}
// 选择并设置最好的CUDA设备
inline bool CompareCudaDevice(const cudaDeviceProp& d1, const cudaDeviceProp& d2)
{
    bool result = (d1.major > d2.major) || ((d1.major == d2.major) && (d1.minor > d2.minor)) || ((d1.major == d2.major) && (d1.minor == d2.minor) && (d1.multiProcessorCount > d2.multiProcessorCount));
    return result;
}
inline int SetBestCudaDevice()
{
    size_t numCudaDevices = GetNumCudaDevices();
    std::vector<cudaDeviceProp> allDevices(numCudaDevices);
    for (size_t deviceID = 0; deviceID < numCudaDevices; deviceID++)
    {
        cudaGetDeviceProperties(&allDevices[deviceID], deviceID);
    }
    std::sort(allDevices.begin(), allDevices.end(), CompareCudaDevice);
    int selectedGPUIndex = -1;
    cudaChooseDevice(&selectedGPUIndex, allDevices.data());

    CHECK(selectedGPUIndex >= 0 && selectedGPUIndex < numCudaDevices);
    cudaDeviceProp device;
    cudaGetDeviceProperties(&device, selectedGPUIndex);
    cudaSetDevice(selectedGPUIndex);
    return selectedGPUIndex;
}