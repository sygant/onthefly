#pragma once
#include <string>
#include <stdexcept>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <any>
#include <Windows.h>
#include <math.h>

#include <boost/stacktrace.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>


#include <Eigen/Core>

#include <cuda_runtime.h>

/*
* 各种宏变量：
* FindPolynomialRoots_Fast
* SIFT_Use_OpenCV
*/

using uchar = unsigned char;
namespace Eigen
{
	using Matrix3x4f = Eigen::Matrix<float, 3, 4>;
	using Matrix3x4d = Eigen::Matrix<double, 3, 4>;
	using Matrix6d = Eigen::Matrix<double, 6, 6>;
	using Vector3ub = Eigen::Matrix<uchar, 3, 1>;
	using Vector4ub = Eigen::Matrix<uchar, 4, 1>;
	using Vector6d = Eigen::Matrix<double, 6, 1>;
}

// 让unordered_set, unordered_map支持使用pair作为key
struct MatchPairHash
{
	size_t operator()(const std::pair<size_t, size_t>& p) const
	{
		size_t h1 = std::hash<size_t>{}(p.first);
		size_t h2 = std::hash<size_t>{}(p.second);
		return h1 ^ h2;
	}
};

struct MatchPairEqual
{
	bool operator()(const std::pair<size_t, size_t>& a, const std::pair<size_t, size_t>& b) const
	{
		return (a.first == b.first && a.second == b.second) || (a.first == b.second && a.second == b.first);
	}
};

#define CHECK(condition, ...) CheckFunc((condition), (__FILE__), (__LINE__), __VA_ARGS__)
inline void CheckFunc(bool condition, const char* file, int line, std::string info = "Check failed!")
{
	if (!condition)
	{
		try
		{
			auto now = std::chrono::system_clock::now();
			auto in_time_t = std::chrono::system_clock::to_time_t(now);
			std::stringstream ss;
			ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");

			std::string output = (boost::format("[%1%]%2% %3%: %4%") % ss.str() % std::string(file) % std::to_string(line) % info).str();
			throw std::runtime_error(output);
}
		catch (const std::exception& e)
		{
			std::string exception = "Exception: " + std::string(e.what());
			OutputDebugStringA(exception.c_str());

			boost::stacktrace::stacktrace st;
			std::ostringstream oss;
			oss << st;
			std::string stacktrace = oss.str();
			OutputDebugStringA(stacktrace.c_str());

			std::string messageBoxContent = exception + "\n" + stacktrace;
			const size_t len = messageBoxContent.length() + 1;
			HGLOBAL hMem = GlobalAlloc(GMEM_MOVEABLE, len);
			memcpy(GlobalLock(hMem), messageBoxContent.c_str(), len);
			GlobalUnlock(hMem);
			OpenClipboard(0);
			EmptyClipboard();
			SetClipboardData(CF_TEXT, hMem);
			CloseClipboard();

			messageBoxContent = "错误信息已复制到粘贴板!\n" + messageBoxContent;
			MessageBoxA(NULL, messageBoxContent.c_str(), "Runtime Error", MB_OK | MB_ICONERROR);

			std::exit(EXIT_FAILURE);
		}
	}
}

#if defined(PARALLEL_OPENMP)
#define Parallel_for(start, end, body) \
        _Pragma("omp parallel for") \
        for(long long i = start; i < end; ++i) \
        { \
            body; \
        }
#elif defined(PARALLEL_TBB)
#define Parallel_for(start_, end_, body) \
        tbb::parallel_for(tbb::blocked_range<long long>(start_, end_), [&](const tbb::blocked_range<long long>& r) { \
            for(long long i = r.begin(); i != r.end(); ++i) \
            { \
                body \
            } \
        });
#else
#define Parallel_for(start, end, body) \
        for(long long i = start; i < end; ++i) \
        { \
            body; \
        }
#endif

class CPartedTime
{
public:
	inline CPartedTime(size_t index)
	{
		this->index = index;
		start = std::chrono::high_resolution_clock::now();
	}
	inline ~CPartedTime()
	{
		stop = std::chrono::high_resolution_clock::now();
		std::string timeInfo = "part " + std::to_string(index) + " time consuming: ";
		timeInfo += std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count()) + "ms\n";
		OutputDebugStringA(timeInfo.c_str());
	}

private:
	std::chrono::steady_clock::time_point start, stop;
	size_t index;
};


// 字符串分割
inline std::vector<std::string> StringSplit(const std::string& str, const std::string& delim)
{
	std::vector<std::string> elems;
	boost::split(elems, str, boost::is_any_of(delim), boost::token_compress_on);
	return elems;
}

// 去除string左边的\n, \t, \r, 空格
inline void StringLeftTrim(std::string& str)
{
	auto it = std::find_if(str.begin(), str.end(), [](char ch) {
		return !std::isspace(static_cast<unsigned char>(ch));
		});
	str.erase(str.begin(), it);
}

// 去除string右边的\n, \t, \r, 空格
inline void StringRightTrim(std::string& str)
{
	auto it = std::find_if(str.rbegin(), str.rend(), [](char ch) {
		return !std::isspace(static_cast<unsigned char>(ch));
		});
	str.erase(it.base(), str.end());
}

// 去除string两边的\n, \t, \r, 空格
inline void StringTrim(std::string& str)
{
	StringLeftTrim(str);
	StringRightTrim(str);
}

template <typename T>
std::vector<T> StringToVector(const std::string& str, const std::string& delim = ",;")
{
	std::vector<std::string> split = StringSplit(str, delim);
	std::vector<T> result;
	result.reserve(split.size());
	for (auto& subStr : split)
	{
		StringTrim(subStr);
		if (subStr.empty())
		{
			continue;
		}
		try
		{
			double value = std::stold(subStr);
			result.push_back(static_cast<T>(value));
		}
		catch (const std::invalid_argument&)
		{
			return {};
		}
	}
	return result;
}

template <typename T>
std::string VectorToString(const std::vector<T>& vec, const std::string& delim = ", ")
{
	std::string result = "";
	for (size_t i = 0; i < vec.size(); i++)
	{
		result += std::to_string(vec[i]);
		if (i != vec.size() - 1)
		{
			result += delim;
		}
	}
	return result;
}

// vector中是否包含某个元素
template <typename T>
bool IsVectorContains(const std::vector<T>& vector, const T value) 
{
	return std::find(vector.begin(), vector.end(), value) != vector.end();
}

// vector中是否存在重复元素
template <typename T>
bool IsVectorContainsDuplicateValues(const std::vector<T>& vector)
{
	std::unordered_set<T> uniqueElements;
	uniqueElements.reserve(vector.size());
	for (const T& element : vector)
	{
		if (uniqueElements.find(element) != uniqueElements.end())
		{
			return true;
		}
		uniqueElements.insert(element);
	}
	return false;
}

// 字符串替换
inline std::string StringReplace(const std::string& str, const std::string& oldStr, const std::string& newStr)
{
	std::string result = str;
	boost::replace_all(result, oldStr, newStr);
	return result;
}

// 获取关键字之后的子字符串
inline std::string StringGetAfter(const std::string& str, const std::string& key)
{
	if (key.empty())return str;
	auto pos = str.rfind(key);
	if (pos != std::string::npos) 
	{
		return str.substr(pos + key.length());
	}
	return "";
}

// 检查字符串是否以某个前缀开始
inline bool IsStringStartsWith(const std::string& str, const std::string& prefix)
{
	if (prefix.empty())return false;
	return boost::starts_with(str, prefix);
}

// 字符串转为小写
inline std::string StringToLower(const std::string& str)
{
	return boost::to_lower_copy(str);
}

// 字符串转为大写
inline std::string StringToUpper(const std::string& str)
{
	return boost::to_upper_copy(str);
}

// 字符串是否包含某个子串
inline bool IsStringContains(const std::string& str, const std::string& subStr)
{
	return boost::contains(str, subStr);
}

// 确保给定字符串路径末尾有一个斜杠
inline std::string EnsureTrailingSlash(const std::string& str) noexcept
{
	if (str.empty())
	{
		return "/";
	}
	if (str.back() != '/')
	{
		return str + '/';
	}
	return str;
}

// 检查文件名是否具有指定的文件扩展名(不区分大小写)
inline bool HasFileExtension(const std::string& fileName, const std::string& extension)
{
	CHECK(!extension.empty());
	CHECK(extension[0] == '.');
	std::string extensionLower = StringToLower(extension);
	std::string fileNameLower = StringToLower(fileName);
	return boost::algorithm::ends_with(fileNameLower, extensionLower);
}

// 分割路径为根和扩展名. 例如: "dir/file.jpg"分割为"dir/file"和".jpg"
inline void SplitFileExtension(const std::string& path, std::string& root, std::string& extension)
{
	std::string pathReplaced = StringReplace(path, "\\", "/");
	if (pathReplaced.empty())
	{
		root = extension = "";
		return;
	}
	if (pathReplaced.back() == '.')
	{
		root = pathReplaced;
		root.pop_back();
		extension = "";
		return;
	}
	size_t pos = pathReplaced.find_last_of('.');
	if (pos != std::string::npos)
	{
		root = path.substr(0, pos);
		extension = path.substr(pos);
	}
	else
	{
		root = path;
		extension = "";
	}
}

// 文件复制
enum class CFileCopyType
{
	CCOPY,
	CHARD_LINK,
	CSOFT_LINK
};
inline void FileCopy(const std::string& srcPath, const std::string& dstPath, CFileCopyType copyType = CFileCopyType::CCOPY)
{
	switch (copyType)
	{
	case CFileCopyType::CCOPY:
		boost::filesystem::copy_file(srcPath, dstPath);
		break;
	case CFileCopyType::CHARD_LINK:
		boost::filesystem::create_hard_link(srcPath, dstPath);
		break;
	case CFileCopyType::CSOFT_LINK:
		boost::filesystem::create_symlink(srcPath, dstPath);
		break;
	default:
		CHECK(false, "Unknown file copy type!");
	}
}

// 路径是否是一个存在的文件
inline bool IsFileExists(const std::string& path)
{
	return boost::filesystem::is_regular_file(path);
}

// 路径是否是一个存在的文件夹(目录)
inline bool IsDirExists(const std::string& path) 
{
	return boost::filesystem::is_directory(path);
}

// 路径是否是存在的(文件或文件夹)
inline bool IsPathExists(const std::string& path)
{
	return boost::filesystem::exists(path);
}

// 如果目录(文件夹)不存在, 则创建它
inline void CreateDirIfNotExists(const std::string& path, bool recursive = false)
{
	if (IsDirExists(path)) 
	{
		return;
	}
	if (recursive) 
	{
		CHECK(boost::filesystem::create_directories(path));
	}
	else 
	{
		CHECK(boost::filesystem::create_directory(path));
	}
}

// 提取路径的最后一段文件或目录的名字
inline std::string GetPathBaseName(const std::string& path) 
{
	const std::vector<std::string> names = StringSplit(StringReplace(path, "\\", "/"), "/");
	if (names.size() > 1 && names.back() == "") 
	{
		return names[names.size() - 2];
	}
	else
	{
		return names.back();
	}
}

// 提取文件名
inline std::string GetFileName(const std::string& path)
{
	std::string result = StringReplace(path, "\\", "/");
	size_t pos = result.find_last_of('/');
	CHECK(pos != std::string::npos);
	result = result.substr(pos + 1, result.size() - pos - 1);
	return result;
}

inline std::string GetFileNameNoExtension(const std::string& path)
{
	std::string result = GetFileName(path);
	size_t pos = result.find_last_of('.');
	if (pos == std::string::npos)
	{
		return result;
	}
	return result.substr(0, pos);
}

// 删除文件
inline void RemoveFile(const std::string& path)
{
	CHECK(IsFileExists(path));
	boost::filesystem::path fileToRemove(path);
	boost::filesystem::remove(fileToRemove);
}

// 删除文件夹(即使这个文件夹非空)
inline void RemoveDir(const std::string& path)
{
	CHECK(IsDirExists(path));
	boost::filesystem::path dirToRemove(path);
	boost::filesystem::remove_all(dirToRemove);
}

// 获取父目录路径
inline std::string GetParentDir(const std::string& path) 
{
	return boost::filesystem::path(path).parent_path().string();
}

// 返回目录中的所有文件
inline std::vector<std::string> GetFileList(const std::string& path)
{
	std::vector<std::string> fileList;
	for (auto it = boost::filesystem::directory_iterator(path); it != boost::filesystem::directory_iterator(); it++)
	{
		if (boost::filesystem::is_regular_file(*it)) 
		{
			const boost::filesystem::path filePath = *it;
			fileList.push_back(filePath.string());
		}
	}
	return fileList;
}

inline std::vector<std::string> GetImageFileList(const std::string& path)
{
	std::vector<std::string> imageFileList;
	for (auto it = boost::filesystem::directory_iterator(path); it != boost::filesystem::directory_iterator(); it++)
	{
		if (boost::filesystem::is_regular_file(*it))
		{
			const boost::filesystem::path filePath = *it;
			std::string path = filePath.string();
			if (HasFileExtension(path, ".jpg") || HasFileExtension(path, ".bmp") || HasFileExtension(path, ".png") || HasFileExtension(path, ".jpeg") || HasFileExtension(path, ".tif") || HasFileExtension(path, ".tiff"))
			{
				std::replace(path.begin(), path.end(), '\\', '/');
				imageFileList.push_back(path);
			}
		}
	}
	return imageFileList;
}



// 递归地返回目录中的所有文件(包含所有的子目录)
inline std::vector<std::string> GetRecursiveFileList(const std::string& path)
{
	std::vector<std::string> fileList;
	for (auto it = boost::filesystem::recursive_directory_iterator(path); it != boost::filesystem::recursive_directory_iterator(); it++)
	{
		if (boost::filesystem::is_regular_file(*it)) 
		{
			const boost::filesystem::path filePath = *it;
			fileList.push_back(filePath.string());
		}
	}
	return fileList;
}

// 返回目录中的所有目录(文件夹)
inline std::vector<std::string> GetDirList(const std::string& path)
{
	std::vector<std::string> dirList;
	for (auto it = boost::filesystem::directory_iterator(path); it != boost::filesystem::directory_iterator(); it++)
	{
		if (boost::filesystem::is_directory(*it)) 
		{
			const boost::filesystem::path dirPath = *it;
			dirList.push_back(dirPath.string());
		}
	}
	return dirList;
}

// 递归地返回目录中的所有子目录
inline std::vector<std::string> GetRecursiveDirList(const std::string& path)
{
	std::vector<std::string> dirList;
	for (auto it = boost::filesystem::recursive_directory_iterator(path); it != boost::filesystem::recursive_directory_iterator(); it++)
	{
		if (boost::filesystem::is_directory(*it)) 
		{
			const boost::filesystem::path dirPath = *it;
			dirList.push_back(dirPath.string());
		}
	}
	return dirList;
}

// 获取文件大小, 单位: 字节
inline size_t GetFileSize(const std::string& path) 
{
	std::ifstream file(path, std::ifstream::ate | std::ifstream::binary);
	CHECK(file.is_open());
	return file.tellg();
}

// 从文本文件中读取每一行，并作为字符串的向量返回
inline std::vector<std::string> ReadTextFile(const std::string& path)
{
	CHECK(IsFileExists(path));

	std::vector<std::string> lines;
	boost::iostreams::mapped_file_source mmap(path);

	const char* f = mmap.data();
	const char* l = f + mmap.size();

	std::string line;
	while (f && f != l) 
	{
		if (*f == '\n')
		{
			lines.push_back(line);
			line.clear();
		}
		else 
		{
			line += *f;
		}
		++f;
	}
	if (!line.empty()) 
	{
		lines.push_back(line);
	}
	return lines;
}

// 路径拼接
template <typename... T>
std::string JoinPaths(T const&... paths) 
{
	boost::filesystem::path result;
	int unpack[]{ 0, (result = result / boost::filesystem::path(paths), 0)... };
	static_cast<void>(unpack);
	return result.string();
}

template <typename Type>
std::vector<std::any> TypeVec2AnyVec(const std::vector<Type>& vec)
{
	std::vector<std::any> result;
	result.reserve(vec.size());
	for (const auto& item : vec) 
	{
		result.emplace_back(std::move(item));
	}
	return result;
	//return std::vector<std::any>(vec.begin(), vec.end());
}

template <typename Type>
std::vector<Type> AnyVec2TypeVec(const std::vector<std::any>& vec)
{
	CHECK(vec.empty() || vec[0].type() == typeid(Type));
	std::vector<Type> result(vec.size());
	for (size_t i = 0; i < vec.size(); i++)
	{
		result[i] = std::any_cast<Type>(vec[i]);
	}
	return result;
}

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



