#include "Common.h"
#include "../Base/math.h"

std::vector<Eigen::Vector2d> FeatureKeypointsToPointsVector(
    const FeatureKeypoints& keypoints) {
    std::vector<Eigen::Vector2d> points(keypoints.size());
    for (size_t i = 0; i < keypoints.size(); ++i) {
        points[i] = Eigen::Vector2d(keypoints[i].x, keypoints[i].y);
    }
    return points;
}

void L2NormalizeFeatureDescriptors(FeatureDescriptorsFloat* descriptors) {
    descriptors->rowwise().normalize();
}

void L1RootNormalizeFeatureDescriptors(FeatureDescriptorsFloat* descriptors) {
    for (Eigen::MatrixXf::Index r = 0; r < descriptors->rows(); ++r) {
        descriptors->row(r) *= 1 / descriptors->row(r).lpNorm<1>();
        descriptors->row(r) = descriptors->row(r).array().sqrt();
    }
}

FeatureDescriptors FeatureDescriptorsToUnsignedByte(
    const Eigen::Ref<const FeatureDescriptorsFloat>& descriptors) {
    FeatureDescriptors descriptors_unsigned_byte(descriptors.rows(),
        descriptors.cols());
    for (Eigen::MatrixXf::Index r = 0; r < descriptors.rows(); ++r) {
        for (Eigen::MatrixXf::Index c = 0; c < descriptors.cols(); ++c) {
            const float scaled_value = std::round(512.0f * descriptors(r, c));
            descriptors_unsigned_byte(r, c) =
                TruncateCast<float, uint8_t>(scaled_value);
        }
    }
    return descriptors_unsigned_byte;
}

void ExtractTopScaleFeatures(FeatureKeypoints* keypoints,
    FeatureDescriptors* descriptors,
    const size_t num_features) {
    CHECK_EQ(keypoints->size(), descriptors->rows());
    CHECK_GT(num_features, 0);

    if (static_cast<size_t>(descriptors->rows()) <= num_features) {
        return;
    }

    std::vector<std::pair<size_t, float>> scales;
    scales.reserve(keypoints->size());
    for (size_t i = 0; i < keypoints->size(); ++i) {
        scales.emplace_back(i, (*keypoints)[i].ComputeScale());
    }

    std::partial_sort(scales.begin(),
        scales.begin() + num_features,
        scales.end(),
        [](const std::pair<size_t, float>& scale1,
            const std::pair<size_t, float>& scale2) {
                return scale1.second > scale2.second;
        });

    FeatureKeypoints top_scale_keypoints(num_features);
    FeatureDescriptors top_scale_descriptors(num_features, descriptors->cols());
    for (size_t i = 0; i < num_features; ++i) {
        top_scale_keypoints[i] = (*keypoints)[scales[i].first];
        top_scale_descriptors.row(i) = descriptors->row(scales[i].first);
    }

    *keypoints = std::move(top_scale_keypoints);
    *descriptors = std::move(top_scale_descriptors);
}