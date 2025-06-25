#include "SupportMeasurer.h"
#include "../Base/logging.h"
#include <unordered_set>


InlierSupportMeasurer::Support InlierSupportMeasurer::Evaluate(
    const std::vector<double>& residuals, const double max_residual) {
    Support support;
    support.num_inliers = 0;
    support.residual_sum = 0;

    for (const auto residual : residuals) {
        if (residual <= max_residual) {
            support.num_inliers += 1;
            support.residual_sum += residual;
        }
    }

    return support;
}

bool InlierSupportMeasurer::Compare(const Support& support1,
    const Support& support2) {
    if (support1.num_inliers > support2.num_inliers) {
        return true;
    }
    else {
        return support1.num_inliers == support2.num_inliers &&
            support1.residual_sum < support2.residual_sum;
    }
}

UniqueInlierSupportMeasurer::Support UniqueInlierSupportMeasurer::Evaluate(
    const std::vector<double>& residuals, const double max_residual) {
    CHECK_EQ(residuals.size(), unique_sample_ids_.size());
    Support support;
    support.num_inliers = 0;
    support.num_unique_inliers = 0;
    support.residual_sum = 0;

    std::unordered_set<size_t> inlier_point_ids;
    for (size_t idx = 0; idx < residuals.size(); ++idx) {
        if (residuals[idx] <= max_residual) {
            support.num_inliers += 1;
            inlier_point_ids.insert(unique_sample_ids_[idx]);
            support.residual_sum += residuals[idx];
        }
    }
    support.num_unique_inliers = inlier_point_ids.size();
    return support;
}

bool UniqueInlierSupportMeasurer::Compare(const Support& support1,
    const Support& support2) {
    if (support1.num_unique_inliers > support2.num_unique_inliers) {
        return true;
    }
    else if (support1.num_unique_inliers == support2.num_unique_inliers) {
        if (support1.num_inliers > support2.num_inliers) {
            return true;
        }
        else {
            return support1.num_inliers == support2.num_inliers &&
                support1.residual_sum < support2.residual_sum;
        }
    }
    else {
        return false;
    }
}

MEstimatorSupportMeasurer::Support MEstimatorSupportMeasurer::Evaluate(
    const std::vector<double>& residuals, const double max_residual) {
    Support support;
    support.num_inliers = 0;
    support.score = 0;

    for (const auto residual : residuals) {
        if (residual <= max_residual) {
            support.num_inliers += 1;
            support.score += residual;
        }
        else {
            support.score += max_residual;
        }
    }

    return support;
}

bool MEstimatorSupportMeasurer::Compare(const Support& support1,
    const Support& support2) {
    return support1.score < support2.score;
}