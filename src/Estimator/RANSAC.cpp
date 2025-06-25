#include "RANSAC.h"

using namespace std;


CRANSAC::CRANSAC(const CRANSACOptions& options, CEstimator* estimator, CSupportMeasurer* supportMeasurer, CSampler* sampler)
{
	CHECK(estimator);
	options.Check();

	this->estimator = estimator;
	this->options = options;
	const size_t numSamples = 100000;
	const size_t dynamicMaxNumTrials = GetNumTrials(estimator->minNumSamples, (size_t)(options.minInlierRatio * numSamples), numSamples, options.confidence, options.maxIterNumTrialsMultiplier);
	this->options.maxNumTrials = min(this->options.maxNumTrials, dynamicMaxNumTrials);

	this->~CRANSAC();
	if (!supportMeasurer)
	{
		this->supportMeasurer = new CInlierSupportMeasurer();
		isSupportMeasurerInitialNull = true;
	}
	else
	{
		this->supportMeasurer = supportMeasurer;
		isSupportMeasurerInitialNull = false;
	}

	if (!sampler)
	{
		this->sampler = new CRandomSampler(estimator->minNumSamples);
		isSamplerInitialNull = true;
	}
	else
	{
		this->sampler = sampler;
		isSamplerInitialNull = false;
	}
}
CRANSAC::~CRANSAC()
{
	if (isSupportMeasurerInitialNull && supportMeasurer)
	{
		delete supportMeasurer;
	}
	if (isSamplerInitialNull && sampler)
	{
		delete sampler;
	}
}
size_t CRANSAC::GetNumTrials(size_t minNumSamples, size_t numInliers, size_t numSamples, double confidence, double numTrialsMultiplier)
{
	const double nom = 1 - confidence;
	if (nom <= 0)
	{
		return numeric_limits<size_t>::max();
	}
	const double inlierRatio = numInliers * 1.0 / numSamples;
	const double denom = 1 - pow(inlierRatio, minNumSamples);
	if (denom <= 0) 
	{
		return 1;
	}
	if (abs(denom - 1) < 1e-6) // ·ÀÖ¹ÏÂÃæ³ýÒÔ0
	{
		return numeric_limits<size_t>::max();
	}

	return static_cast<size_t>(ceil(log(nom) / log(denom) * numTrialsMultiplier));
}
size_t CRANSAC::GetNumTrials(size_t numInliers, size_t numSamples, double confidence, double numTrialsMultiplier)
{
	CHECK(estimator);
	return GetNumTrials(estimator->minNumSamples, numInliers, numSamples, confidence, numTrialsMultiplier);
}

CLORANSAC::CLORANSAC(const CRANSACOptions& options, CEstimator* estimator, CEstimator* localEstimator, CSupportMeasurer* supportMeasurer, CSampler* sampler) :CRANSAC(options, estimator, supportMeasurer, sampler)
{
	CHECK(localEstimator);
	this->localEstimator = localEstimator;
}
