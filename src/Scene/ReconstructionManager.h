#pragma once
#include "Reconstruction.h"
#include <memory>

class OptionManager;
class ReconstructionManager
{
public:
	ReconstructionManager();
	// The number of reconstructions managed.
	size_t Size() const;

	// Get a reference to a specific reconstruction.
	std::shared_ptr<const Reconstruction> Get(size_t idx) const;
	std::shared_ptr<Reconstruction>& Get(size_t idx);

	// Add a new empty reconstruction and return its index.
	size_t Add();

	// Delete a specific reconstruction.
	void Delete(size_t idx);

	// Delete all reconstructions.
	void Clear();

	// Read and add a new reconstruction and return its index.
	size_t Read(const std::string& path);

	// Write all managed reconstructions into sub-folders "0", "1", "2", ...
	void Write(const std::string& path) const;

private:
	std::vector<std::shared_ptr<Reconstruction>> reconstructions_;
};