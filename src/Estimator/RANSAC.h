#pragma once
#include "Sampler.h"
#include "SupportMeasurer.h"
#include "../Base/logging.h"

#include <cfloat>
#include <random>
#include <stdexcept>
#include <vector>

struct RANSACOptions 
{
    // Maximum error for a sample to be considered as an inlier. Note that
    // the residual of an estimator corresponds to a squared error.
    double max_error = 0.0;

    // A priori assumed minimum inlier ratio, which determines the maximum number
    // of iterations. Only applies if smaller than `max_num_trials`.
    double min_inlier_ratio = 0.1;

    // Abort the iteration if minimum probability that one sample is free from
    // outliers is reached.
    double confidence = 0.99;

    // The num_trials_multiplier to the dynamically computed maximum number of
    // iterations based on the specified confidence value.
    double dyn_num_trials_multiplier = 3.0;

    // Number of random trials to estimate model from random subset.
    int min_num_trials = 0;
    int max_num_trials = std::numeric_limits<int>::max();

    void Check() const {
        CHECK_GT(max_error, 0);
        CHECK_GE(min_inlier_ratio, 0);
        CHECK_LE(min_inlier_ratio, 1);
        CHECK_GE(confidence, 0);
        CHECK_LE(confidence, 1);
        CHECK_LE(min_num_trials, max_num_trials);
    }
};


template <typename Estimator,
    typename SupportMeasurer = InlierSupportMeasurer,
    typename Sampler = RandomSampler>
class RANSAC {
public:
    struct Report {
        // Whether the estimation was successful.
        bool success = false;

        // The number of RANSAC trials / iterations.
        size_t num_trials = 0;

        // The support of the estimated model.
        typename SupportMeasurer::Support support;

        // Boolean mask which is true if a sample is an inlier.
        std::vector<char> inlier_mask;

        // The estimated model.
        typename Estimator::M_t model;
    };

    explicit RANSAC(const RANSACOptions& options) : sampler(Sampler(Estimator::kMinNumSamples)), options_(options)
    {
        options.Check();

        // Determine max_num_trials based on assumed `min_inlier_ratio`.
        const size_t kNumSamples = 100000;
        const size_t dyn_max_num_trials = ComputeNumTrials(
            static_cast<size_t>(options_.min_inlier_ratio * kNumSamples),
            kNumSamples,
            options_.confidence,
            options_.dyn_num_trials_multiplier);
        options_.max_num_trials =
            std::min<size_t>(options_.max_num_trials, dyn_max_num_trials);
    }

    // Determine the maximum number of trials required to sample at least one
    // outlier-free random set of samples with the specified confidence,
    // given the inlier ratio.
    //
    // @param num_inliers				The number of inliers.
    // @param num_samples				The total number of samples.
    // @param confidence				Confidence that one sample is
    //								outlier-free.
    // @param num_trials_multiplier   Multiplication factor to the computed
    //							    number of trials.
    //
    // @return               The required number of iterations.
    static size_t ComputeNumTrials(size_t num_inliers,
        size_t num_samples,
        double confidence,
        double num_trials_multiplier)
    {
        const double inlier_ratio = num_inliers / static_cast<double>(num_samples);

        const double nom = 1 - confidence;
        if (nom <= 0) {
            return std::numeric_limits<size_t>::max();
        }

        const double denom = 1 - std::pow(inlier_ratio, Estimator::kMinNumSamples);
        if (denom <= 0) {
            return 1;
        }
        // Prevent divide by zero below.
        if (denom == 1.0) {
            return std::numeric_limits<size_t>::max();
        }

        return static_cast<size_t>(
            std::ceil(std::log(nom) / std::log(denom) * num_trials_multiplier));
    }

    // Robustly estimate model with RANSAC (RANdom SAmple Consensus).
    //
    // @param X              Independent variables.
    // @param Y              Dependent variables.
    //
    // @return               The report with the results of the estimation.
    Report Estimate(const std::vector<typename Estimator::X_t>& X,
        const std::vector<typename Estimator::Y_t>& Y)
    {
        CHECK_EQ(X.size(), Y.size());

        const size_t num_samples = X.size();

        Report report;
        report.success = false;
        report.num_trials = 0;

        if (num_samples < Estimator::kMinNumSamples) {
            return report;
        }

        typename SupportMeasurer::Support best_support;
        typename Estimator::M_t best_model;

        bool abort = false;

        const double max_residual = options_.max_error * options_.max_error;

        std::vector<double> residuals(num_samples);

        std::vector<typename Estimator::X_t> X_rand(Estimator::kMinNumSamples);
        std::vector<typename Estimator::Y_t> Y_rand(Estimator::kMinNumSamples);

        sampler.Initialize(num_samples);

        size_t max_num_trials =
            std::min<size_t>(options_.max_num_trials, sampler.MaxNumSamples());
        size_t dyn_max_num_trials = max_num_trials;
        const size_t min_num_trials = options_.min_num_trials;

        for (report.num_trials = 0; report.num_trials < max_num_trials;
            ++report.num_trials) {
            if (abort) {
                report.num_trials += 1;
                break;
            }

            sampler.SampleXY(X, Y, &X_rand, &Y_rand);

            // Estimate model for current subset.
            const std::vector<typename Estimator::M_t> sample_models =
                estimator.Estimate(X_rand, Y_rand);

            // Iterate through all estimated models.
            for (const auto& sample_model : sample_models) {
                estimator.Residuals(X, Y, sample_model, &residuals);
                CHECK_EQ(residuals.size(), num_samples);

                const auto support = support_measurer.Evaluate(residuals, max_residual);

                // Save as best subset if better than all previous subsets.
                if (support_measurer.Compare(support, best_support)) {
                    best_support = support;
                    best_model = sample_model;

                    dyn_max_num_trials =
                        ComputeNumTrials(best_support.num_inliers,
                            num_samples,
                            options_.confidence,
                            options_.dyn_num_trials_multiplier);
                }

                if (report.num_trials >= dyn_max_num_trials &&
                    report.num_trials >= min_num_trials) {
                    abort = true;
                    break;
                }
            }
        }

        report.support = best_support;
        report.model = best_model;

        // No valid model was found.
        if (report.support.num_inliers < estimator.kMinNumSamples) {
            return report;
        }

        report.success = true;

        // Determine inlier mask. Note that this calculates the residuals for the
        // best model twice, but saves to copy and fill the inlier mask for each
        // evaluated model. Some benchmarking revealed that this approach is faster.

        estimator.Residuals(X, Y, report.model, &residuals);
        CHECK_EQ(residuals.size(), num_samples);

        report.inlier_mask.resize(num_samples);
        for (size_t i = 0; i < residuals.size(); ++i) {
            report.inlier_mask[i] = residuals[i] <= max_residual;
        }

        return report;
    }

    // Objects used in RANSAC procedure. Access useful to define custom behavior
    // through options or e.g. to compute residuals.
    Estimator estimator;
    Sampler sampler;
    SupportMeasurer support_measurer;

protected:
    RANSACOptions options_;
};


// Implementation of LO-RANSAC (Locally Optimized RANSAC).
//
// "Locally Optimized RANSAC" Ondrej Chum, Jiri Matas, Josef Kittler, DAGM 2003.
template <typename Estimator,
    typename LocalEstimator,
    typename SupportMeasurer = InlierSupportMeasurer,
    typename Sampler = RandomSampler>
class LORANSAC : public RANSAC<Estimator, SupportMeasurer, Sampler> {
public:
    using typename RANSAC<Estimator, SupportMeasurer, Sampler>::Report;

    explicit LORANSAC(const RANSACOptions& options) : RANSAC<Estimator, SupportMeasurer, Sampler>(options)
    {

    }

    // Robustly estimate model with RANSAC (RANdom SAmple Consensus).
    //
    // @param X              Independent variables.
    // @param Y              Dependent variables.
    //
    // @return               The report with the results of the estimation.
    Report Estimate(const std::vector<typename Estimator::X_t>& X,
        const std::vector<typename Estimator::Y_t>& Y)
    {
        CHECK_EQ(X.size(), Y.size());

        const size_t num_samples = X.size();

        typename RANSAC<Estimator, SupportMeasurer, Sampler>::Report report;
        report.success = false;
        report.num_trials = 0;

        if (num_samples < Estimator::kMinNumSamples) {
            return report;
        }

        typename SupportMeasurer::Support best_support;
        typename Estimator::M_t best_model;
        bool best_model_is_local = false;

        bool abort = false;

        const double max_residual = options_.max_error * options_.max_error;

        std::vector<double> residuals;
        std::vector<double> best_local_residuals;

        std::vector<typename LocalEstimator::X_t> X_inlier;
        std::vector<typename LocalEstimator::Y_t> Y_inlier;

        std::vector<typename Estimator::X_t> X_rand(Estimator::kMinNumSamples);
        std::vector<typename Estimator::Y_t> Y_rand(Estimator::kMinNumSamples);

        sampler.Initialize(num_samples);

        size_t max_num_trials =
            std::min<size_t>(options_.max_num_trials, sampler.MaxNumSamples());
        size_t dyn_max_num_trials = max_num_trials;
        const size_t min_num_trials = options_.min_num_trials;

        for (report.num_trials = 0; report.num_trials < max_num_trials;
            ++report.num_trials) {
            if (abort) {
                report.num_trials += 1;
                break;
            }

            sampler.SampleXY(X, Y, &X_rand, &Y_rand);

            // Estimate model for current subset.
            const std::vector<typename Estimator::M_t> sample_models =
                estimator.Estimate(X_rand, Y_rand);

            // Iterate through all estimated models
            for (const auto& sample_model : sample_models) {
                estimator.Residuals(X, Y, sample_model, &residuals);
                CHECK_EQ(residuals.size(), num_samples);

                const auto support = support_measurer.Evaluate(residuals, max_residual);

                // Do local optimization if better than all previous subsets.
                if (support_measurer.Compare(support, best_support)) {
                    best_support = support;
                    best_model = sample_model;
                    best_model_is_local = false;

                    // Estimate locally optimized model from inliers.
                    if (support.num_inliers > Estimator::kMinNumSamples &&
                        support.num_inliers >= LocalEstimator::kMinNumSamples) {
                        // Recursive local optimization to expand inlier set.
                        const size_t kMaxNumLocalTrials = 10;
                        for (size_t local_num_trials = 0;
                            local_num_trials < kMaxNumLocalTrials;
                            ++local_num_trials) {
                            X_inlier.clear();
                            Y_inlier.clear();
                            X_inlier.reserve(num_samples);
                            Y_inlier.reserve(num_samples);
                            for (size_t i = 0; i < residuals.size(); ++i) {
                                if (residuals[i] <= max_residual) {
                                    X_inlier.push_back(X[i]);
                                    Y_inlier.push_back(Y[i]);
                                }
                            }

                            const std::vector<typename LocalEstimator::M_t> local_models =
                                local_estimator.Estimate(X_inlier, Y_inlier);

                            const size_t prev_best_num_inliers = best_support.num_inliers;

                            for (const auto& local_model : local_models) {
                                local_estimator.Residuals(X, Y, local_model, &residuals);
                                CHECK_EQ(residuals.size(), num_samples);

                                const auto local_support =
                                    support_measurer.Evaluate(residuals, max_residual);

                                // Check if locally optimized model is better.
                                if (support_measurer.Compare(local_support, best_support)) {
                                    best_support = local_support;
                                    best_model = local_model;
                                    best_model_is_local = true;
                                    std::swap(residuals, best_local_residuals);
                                }
                            }

                            // Only continue recursive local optimization, if the inlier set
                            // size increased and we thus have a chance to further improve.
                            if (best_support.num_inliers <= prev_best_num_inliers) {
                                break;
                            }

                            // Swap back the residuals, so we can extract the best inlier
                            // set in the next recursion of local optimization.
                            std::swap(residuals, best_local_residuals);
                        }
                    }

                    dyn_max_num_trials =
                        RANSAC<Estimator, SupportMeasurer, Sampler>::ComputeNumTrials(
                            best_support.num_inliers,
                            num_samples,
                            options_.confidence,
                            options_.dyn_num_trials_multiplier);
                }

                if (report.num_trials >= dyn_max_num_trials &&
                    report.num_trials >= min_num_trials) {
                    abort = true;
                    break;
                }
            }
        }

        report.support = best_support;
        report.model = best_model;

        // No valid model was found
        if (report.support.num_inliers < estimator.kMinNumSamples) {
            return report;
        }

        report.success = true;

        // Determine inlier mask. Note that this calculates the residuals for the
        // best model twice, but saves to copy and fill the inlier mask for each
        // evaluated model. Some benchmarking revealed that this approach is faster.

        if (best_model_is_local) {
            local_estimator.Residuals(X, Y, report.model, &residuals);
        }
        else {
            estimator.Residuals(X, Y, report.model, &residuals);
        }

        CHECK_EQ(residuals.size(), num_samples);

        report.inlier_mask.resize(num_samples);
        for (size_t i = 0; i < residuals.size(); ++i) {
            report.inlier_mask[i] = residuals[i] <= max_residual;
        }

        return report;
    }

    // Objects used in RANSAC procedure.
    using RANSAC<Estimator, SupportMeasurer, Sampler>::estimator;
    LocalEstimator local_estimator;
    using RANSAC<Estimator, SupportMeasurer, Sampler>::sampler;
    using RANSAC<Estimator, SupportMeasurer, Sampler>::support_measurer;

private:
    using RANSAC<Estimator, SupportMeasurer, Sampler>::options_;
};