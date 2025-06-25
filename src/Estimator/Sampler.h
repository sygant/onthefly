#pragma once
#include "../Base/logging.h"
#include <cstddef>
#include <vector>


// Abstract base class for sampling methods.
class Sampler 
{
public:
	Sampler() = default;
	explicit Sampler(size_t num_samples);
	virtual ~Sampler() = default;

	// Initialize the sampler, before calling the `Sample` method.
	virtual void Initialize(size_t total_num_samples) = 0;

	// Maximum number of unique samples that can be generated.
	virtual size_t MaxNumSamples() = 0;

	// Sample `num_samples` elements from all samples.
	virtual void Sample(std::vector<size_t>* sampled_idxs) = 0;

	// Sample elements from `X` into `X_rand`.
	//
	// Note that `X.size()` should equal `num_total_samples` and `X_rand.size()`
	// should equal `num_samples`.
	template <typename X_t>
	void SampleX(const X_t& X, X_t* X_rand) {
		thread_local std::vector<size_t> sampled_idxs;
		Sample(&sampled_idxs);
		for (size_t i = 0; i < X_rand->size(); ++i) {
			(*X_rand)[i] = X[sampled_idxs[i]];
		}
	}

	// Sample elements from `X` and `Y` into `X_rand` and `Y_rand`.
	//
	// Note that `X.size()` should equal `num_total_samples` and `X_rand.size()`
	// should equal `num_samples`. The same applies for `Y` and `Y_rand`.
	template <typename X_t, typename Y_t>
	void SampleXY(const X_t& X, const Y_t& Y, X_t* X_rand, Y_t* Y_rand) {
		CHECK_EQ(X.size(), Y.size());
		CHECK_EQ(X_rand->size(), Y_rand->size());
		thread_local std::vector<size_t> sampled_idxs;
		Sample(&sampled_idxs);
		for (size_t i = 0; i < X_rand->size(); ++i) {
			(*X_rand)[i] = X[sampled_idxs[i]];
			(*Y_rand)[i] = Y[sampled_idxs[i]];
		}
	}
};

// Random sampler for RANSAC-based methods that generates unique samples.
//
// Note that a separate sampler should be instantiated per thread and it assumes
// that the input data is shuffled in advance.
class CombinationSampler : public Sampler 
{
public:
	explicit CombinationSampler(size_t num_samples);

	void Initialize(size_t total_num_samples) override;

	size_t MaxNumSamples() override;

	void Sample(std::vector<size_t>* sampled_idxs) override;

private:
	const size_t num_samples_;
	std::vector<size_t> total_sample_idxs_;
};


// Random sampler for PROSAC (Progressive Sample Consensus), as described in:
//
//    "Matching with PROSAC - Progressive Sample Consensus".
//        Ondrej Chum and Matas, CVPR 2005.
//
// Note that a separate sampler should be instantiated per thread and that the
// data to be sampled from is assumed to be sorted according to the quality
// function in descending order, i.e., higher quality data is closer to the
// front of the list.
class ProgressiveSampler : public Sampler {
public:
	explicit ProgressiveSampler(size_t num_samples);

	void Initialize(size_t total_num_samples) override;

	size_t MaxNumSamples() override;

	void Sample(std::vector<size_t>* sampled_idxs) override;

private:
	const size_t num_samples_;
	size_t total_num_samples_;

	// The number of generated samples, i.e. the number of calls to `Sample`.
	size_t t_;
	size_t n_;

	// Variables defined in equation 3.
	double T_n_;
	double T_n_p_;
};


// Random sampler for RANSAC-based methods.
//
// Note that a separate sampler should be instantiated per thread.
class RandomSampler : public Sampler {
public:
	explicit RandomSampler(size_t num_samples);

	void Initialize(size_t total_num_samples) override;

	size_t MaxNumSamples() override;

	void Sample(std::vector<size_t>* sampled_idxs) override;

private:
	const size_t num_samples_;
	std::vector<size_t> sample_idxs_;
};