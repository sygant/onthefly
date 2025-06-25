#pragma once
#include "logging.h"
#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <list>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>


#include <chrono>
#include <memory>
#include <random>
#include <thread>


#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

// Return 1 if number is positive, -1 if negative, and 0 if the number is 0.
template <typename T>
int SignOfNumber(const T val) {
	return (T(0) < val) - (val < T(0));
}

// Clamp the given value to a low and maximum value.
template <typename T>
T Clamp(const T& value, const T& low, const T& high) {
	return std::max(low, std::min(value, high));
}

// Convert angle in degree to radians.
inline float DegToRad(float deg)
{
	return deg * 0.0174532925199432954743716805978692718781530857086181640625f;
}
inline double DegToRad(double deg)
{
	return deg * 0.0174532925199432954743716805978692718781530857086181640625;
}

// Convert angle in radians to degree.
inline float RadToDeg(float rad)
{
	return rad * 57.29577951308232286464772187173366546630859375f;
}
inline double RadToDeg(double rad)
{
	return rad * 57.29577951308232286464772187173366546630859375;
}

// Determine median value in vector. Returns NaN for empty vectors.
template <typename T>
double Median(const std::vector<T>& elems)
{
	CHECK(!elems.empty());

	const size_t mid_idx = elems.size() / 2;

	std::vector<T> ordered_elems = elems;
	std::nth_element(ordered_elems.begin(),
		ordered_elems.begin() + mid_idx,
		ordered_elems.end());

	if (elems.size() % 2 == 0) {
		const T mid_element1 = ordered_elems[mid_idx];
		const T mid_element2 = *std::max_element(ordered_elems.begin(),
			ordered_elems.begin() + mid_idx);
		return 0.5 * mid_element1 + 0.5 * mid_element2;
	}
	else {
		return ordered_elems[mid_idx];
	}
}

// Determine mean value in a vector.
template <typename T>
double Mean(const std::vector<T>& elems)
{
	CHECK(!elems.empty());
	double sum = 0;
	for (const auto el : elems) {
		sum += static_cast<double>(el);
	}
	return sum / elems.size();
}

// Determine sample variance in a vector.
template <typename T>
double Variance(const std::vector<T>& elems)
{
	const double mean = Mean(elems);
	double var = 0;
	for (const auto el : elems) {
		const double diff = el - mean;
		var += diff * diff;
	}
	return var / (elems.size() - 1);
}

// Determine sample standard deviation in a vector.
template <typename T>
double StdDev(const std::vector<T>& elems)
{
	return std::sqrt(Variance(elems));
}



template <class Iterator>
bool NextCombination(Iterator first1,
	Iterator last1,
	Iterator first2,
	Iterator last2) {
	if ((first1 == last1) || (first2 == last2)) {
		return false;
	}
	Iterator m1 = last1;
	Iterator m2 = last2;
	--m2;
	while (--m1 != first1 && *m1 >= *m2) {
	}
	bool result = (m1 == first1) && *first1 >= *m2;
	if (!result) {
		while (first2 != m2 && *m1 >= *first2) {
			++first2;
		}
		first1 = m1;
		std::iter_swap(first1, first2);
		++first1;
		++first2;
	}
	if ((first1 != last1) && (first2 != last2)) {
		m1 = last1;
		m2 = first2;
		while ((m1 != first1) && (m2 != last2)) {
			std::iter_swap(--m1, m2);
			++m2;
		}
		std::reverse(first1, m1);
		std::reverse(first1, last1);
		std::reverse(m2, last2);
		std::reverse(first2, last2);
	}
	return !result;
}
// Generate N-choose-K combinations.
//
// Note that elements in range [first, last) must be in sorted order,
// according to `std::less`.
template <class Iterator>
bool NextCombination(Iterator first, Iterator middle, Iterator last)
{
	return NextCombination(first, middle, middle, last);
}

// Sigmoid function.
template <typename T>
T Sigmoid(T x, T alpha = 1)
{
	return T(1) / (T(1) + std::exp(-x * alpha));
}

// Scale values according to sigmoid transform.
//
//   x \in [0, 1] -> x \in [-x0, x0] -> sigmoid(x, alpha) -> x \in [0, 1]
//
// @param x        Value to be scaled in the range [0, 1].
// @param x0       Spread that determines the range x is scaled to.
// @param alpha    Exponential sigmoid factor.
//
// @return         The scaled value in the range [0, 1].
template <typename T>
T ScaleSigmoid(T x, T alpha = 1, T x0 = 10)
{
	const T t0 = Sigmoid(-x0, alpha);
	const T t1 = Sigmoid(x0, alpha);
	x = (Sigmoid(2 * x0 * x - x0, alpha) - t0) / (t1 - t0);
	return x;
}

// Binomial coefficient or all combinations, defined as n! / ((n - k)! k!).
size_t NChooseK(size_t n, size_t k);

// Cast value from one type to another and truncate instead of overflow, if the
// input value is out of range of the output data type.
template <typename T1, typename T2>
T2 TruncateCast(T1 value)
{
	return static_cast<T2>(std::min(
		static_cast<T1>(std::numeric_limits<T2>::max()),
		std::max(static_cast<T1>(std::numeric_limits<T2>::min()), value)));
}

// Compute the n-th percentile in the given sequence.
template <typename T>
T Percentile(const std::vector<T>& elems, double p) {
	CHECK(!elems.empty());
	CHECK_GE(p, 0);
	CHECK_LE(p, 100);

	const int idx = static_cast<int>(std::round(p / 100 * (elems.size() - 1)));
	const size_t percentile_idx =
		std::max(0, std::min(static_cast<int>(elems.size() - 1), idx));

	std::vector<T> ordered_elems = elems;
	std::nth_element(ordered_elems.begin(),
		ordered_elems.begin() + percentile_idx,
		ordered_elems.end());

	return ordered_elems.at(percentile_idx);
}


template <typename MatrixType>
void DecomposeMatrixRQ(const MatrixType& A, MatrixType* R, MatrixType* Q)
{
	const MatrixType A_flipud_transpose =
		A.transpose().rowwise().reverse().eval();

	const Eigen::HouseholderQR<MatrixType> QR(A_flipud_transpose);
	const MatrixType& Q0 = QR.householderQ();
	const MatrixType& R0 = QR.matrixQR();

	*R = R0.transpose().colwise().reverse().eval();
	*R = R->rowwise().reverse().eval();
	for (int i = 0; i < R->rows(); ++i) {
		for (int j = 0; j < R->cols() && (R->cols() - j) >(R->rows() - i); ++j) {
			(*R)(i, j) = 0;
		}
	}

	*Q = Q0.transpose().colwise().reverse().eval();

	// Make the decomposition unique by requiring that det(Q) > 0.
	if (Q->determinant() < 0) {
		Q->row(1) *= -1.0;
		R->col(1) *= -1.0;
	}
}


// All polynomials are assumed to be the form:
//
//   sum_{i=0}^N polynomial(i) x^{N-i}.
//
// and are given by a vector of coefficients of size N + 1.
//
// The implementation is based on COLMAP's old polynomial functionality and is
// inspired by Ceres-Solver's/Theia's implementation to support complex
// polynomials. The companion matrix implementation is based on NumPy.

// Evaluate the polynomial for the given coefficients at x using the Horner
// scheme. This function is templated such that the polynomial may be evaluated
// at real and/or imaginary points.
template <typename T>
T EvaluatePolynomial(const Eigen::VectorXd& coeffs, const T& x)
{
	T value = 0.0;
	for (Eigen::VectorXd::Index i = 0; i < coeffs.size(); ++i) {
		value = value * x + coeffs(i);
	}
	return value;
}

// Find the root of polynomials of the form: a * x + b = 0.
// The real and/or imaginary variable may be NULL if the output is not needed.
bool FindLinearPolynomialRoots(const Eigen::VectorXd& coeffs,
	Eigen::VectorXd* real,
	Eigen::VectorXd* imag);

// Find the roots of polynomials of the form: a * x^2 + b * x + c = 0.
// The real and/or imaginary variable may be NULL if the output is not needed.
bool FindQuadraticPolynomialRoots(const Eigen::VectorXd& coeffs,
	Eigen::VectorXd* real,
	Eigen::VectorXd* imag);

// Find the roots of a polynomial using the Durand-Kerner method, based on:
//
//    https://en.wikipedia.org/wiki/Durand%E2%80%93Kerner_method
//
// The Durand-Kerner is comparatively fast but often unstable/inaccurate.
// The real and/or imaginary variable may be NULL if the output is not needed.
bool FindPolynomialRootsDurandKerner(const Eigen::VectorXd& coeffs,
	Eigen::VectorXd* real,
	Eigen::VectorXd* imag);

// Find the roots of a polynomial using the companion matrix method, based on:
//
//    R. A. Horn & C. R. Johnson, Matrix Analysis. Cambridge,
//    UK: Cambridge University Press, 1999, pp. 146-7.
//
// Compared to Durand-Kerner, this method is slower but more stable/accurate.
// The real and/or imaginary variable may be NULL if the output is not needed.
bool FindPolynomialRootsCompanionMatrix(const Eigen::VectorXd& coeffs,
	Eigen::VectorXd* real,
	Eigen::VectorXd* imag);


extern thread_local std::unique_ptr<std::mt19937> PRNG;

extern int kDefaultPRNGSeed;


// Initialize the PRNG with the given seed.
//
// @param seed   The seed for the PRNG. If the seed is -1, the current time
//               is used as the seed.
void SetPRNGSeed(unsigned seed = kDefaultPRNGSeed);

// Generate uniformly distributed random integer number.
//
// This implementation is unbiased and thread-safe in contrast to `rand()`.
template <typename T>
T RandomUniformInteger(T min, T max)
{
	if (PRNG == nullptr) {
		SetPRNGSeed();
	}

	std::uniform_int_distribution<T> distribution(min, max);

	return distribution(*PRNG);
}

// Generate uniformly distributed random real number.
//
// This implementation is unbiased and thread-safe in contrast to `rand()`.
template <typename T>
T RandomUniformReal(T min, T max)
{
	if (PRNG == nullptr) {
		SetPRNGSeed();
	}

	std::uniform_real_distribution<T> distribution(min, max);

	return distribution(*PRNG);
}

// Generate Gaussian distributed random real number.
//
// This implementation is unbiased and thread-safe in contrast to `rand()`.
template <typename T>
T RandomGaussian(T mean, T stddev)
{
	if (PRNG == nullptr) {
		SetPRNGSeed();
	}

	std::normal_distribution<T> distribution(mean, stddev);
	return distribution(*PRNG);
}

// Fisher-Yates shuffling.
//
// Note that the vector may not contain more values than UINT32_MAX. This
// restriction comes from the fact that the 32-bit version of the
// Mersenne Twister PRNG is significantly faster.
//
// @param elems            Vector of elements to shuffle.
// @param num_to_shuffle   Optional parameter, specifying the number of first
//                         N elements in the vector to shuffle.
template <typename T>
void Shuffle(uint32_t num_to_shuffle, std::vector<T>* elems)
{
	CHECK_LE(num_to_shuffle, elems->size());
	const uint32_t last_idx = static_cast<uint32_t>(elems->size() - 1);
	for (uint32_t i = 0; i < num_to_shuffle; ++i) {
		const auto j = RandomUniformInteger<uint32_t>(i, last_idx);
		std::swap((*elems)[i], (*elems)[j]);
	}
}













