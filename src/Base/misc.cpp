#include "misc.h"
#include <cstdarg>
#include <filesystem>
#include <boost/algorithm/string.hpp>

std::string EnsureTrailingSlash(const std::string& str)
{
    if (str.length() > 0)
    {
        if (str.back() != '/')
        {
            return str + "/";
        }
    }
    else
    {
        return str + "/";
    }
    return str;
}

bool HasFileExtension(const std::string& file_name, const std::string& ext)
{
    CHECK(!ext.empty());
    CHECK_EQ(ext.at(0), '.');
    std::string ext_lower = ext;
    StringToLower(&ext_lower);
    if (file_name.size() >= ext_lower.size() && file_name.substr(file_name.size() - ext_lower.size(), ext_lower.size()) == ext_lower)
    {
        return true;
    }
    return false;
}

void SplitFileExtension(const std::string& path, std::string* root, std::string* ext)
{
    const auto parts = StringSplit(path, ".");
    CHECK_GT(parts.size(), 0);
    if (parts.size() == 1) 
    {
        *root = parts[0];
        *ext = "";
    }
    else 
    {
        *root = "";
        for (size_t i = 0; i < parts.size() - 1; ++i) 
        {
            *root += parts[i] + ".";
        }
        *root = root->substr(0, root->length() - 1);
        if (parts.back() == "") 
        {
            *ext = "";
        }
        else 
        {
            *ext = "." + parts.back();
        }
    }
}
void FileCopy(const std::string& src_path,
    const std::string& dst_path,
    CopyType type) {
    switch (type) {
    case CopyType::COPY:
        boost::filesystem::copy_file(src_path, dst_path);
        break;
    case CopyType::HARD_LINK:
        boost::filesystem::create_hard_link(src_path, dst_path);
        break;
    case CopyType::SOFT_LINK:
        boost::filesystem::create_symlink(src_path, dst_path);
        break;
    }
}

bool ExistsFile(const std::string& path) {
    std::filesystem::path p(path);
    return std::filesystem::exists(p) && std::filesystem::is_regular_file(p);
}

bool ExistsDir(const std::string& path) {
    std::filesystem::path p(path);
    return std::filesystem::exists(p) && std::filesystem::is_directory(p);
}

bool ExistsPath(const std::string& path) {
    std::filesystem::path p(path);
    return std::filesystem::exists(p);

}

void CreateDirIfNotExists(const std::string& path, bool recursive) {
    if (ExistsDir(path)) {
        return;
    }
    if (recursive) {
        CHECK(boost::filesystem::create_directories(path));
    }
    else {
        CHECK(boost::filesystem::create_directory(path));
    }
}

std::string GetPathBaseName(const std::string& path) {
    const std::vector<std::string> names =
        StringSplit(StringReplace(path, "\\", "/"), "/");
    if (names.size() > 1 && names.back() == "") {
        return names[names.size() - 2];
    }
    else {
        return names.back();
    }
}

std::string GetParentDir(const std::string& path) {
    return boost::filesystem::path(path).parent_path().string();
}

std::vector<std::string> GetFileList(const std::string& path) {
    std::vector<std::string> file_list;
    for (auto it = boost::filesystem::directory_iterator(path);
        it != boost::filesystem::directory_iterator();
        ++it) {
        if (boost::filesystem::is_regular_file(*it)) {
            const boost::filesystem::path file_path = *it;
            file_list.push_back(file_path.string());
        }
    }
    return file_list;
}

std::vector<std::string> GetRecursiveFileList(const std::string& path) {
    std::vector<std::string> file_list;
    for (auto it = boost::filesystem::recursive_directory_iterator(path);
        it != boost::filesystem::recursive_directory_iterator();
        ++it) {
        if (boost::filesystem::is_regular_file(*it)) {
            const boost::filesystem::path file_path = *it;
            file_list.push_back(file_path.string());
        }
    }
    return file_list;
}

std::vector<std::string> GetDirList(const std::string& path) {
    std::vector<std::string> dir_list;
    for (auto it = boost::filesystem::directory_iterator(path);
        it != boost::filesystem::directory_iterator();
        ++it) {
        if (boost::filesystem::is_directory(*it)) {
            const boost::filesystem::path dir_path = *it;
            dir_list.push_back(dir_path.string());
        }
    }
    return dir_list;
}

std::vector<std::string> GetRecursiveDirList(const std::string& path) {
    std::vector<std::string> dir_list;
    for (auto it = boost::filesystem::recursive_directory_iterator(path);
        it != boost::filesystem::recursive_directory_iterator();
        ++it) {
        if (boost::filesystem::is_directory(*it)) {
            const boost::filesystem::path dir_path = *it;
            dir_list.push_back(dir_path.string());
        }
    }
    return dir_list;
}

size_t GetFileSize(const std::string& path) {
    std::ifstream file(path, std::ifstream::ate | std::ifstream::binary);
    CHECK(file.is_open()) << path;
    return file.tellg();
}

void PrintHeading1(const std::string& heading) {
    std::cout << std::endl << std::string(78, '=') << std::endl;
    std::cout << heading << std::endl;
    std::cout << std::string(78, '=') << std::endl << std::endl;
}

void PrintHeading2(const std::string& heading) {
    std::cout << std::endl << heading << std::endl;
    std::cout << std::string(std::min<int>(heading.size(), 78), '-') << std::endl;
}

template <>
std::vector<std::string> CSVToVector(const std::string& csv) {
    auto elems = StringSplit(csv, ",;");
    std::vector<std::string> values;
    values.reserve(elems.size());
    for (auto& elem : elems) {
        StringTrim(&elem);
        if (elem.empty()) {
            continue;
        }
        values.push_back(elem);
    }
    return values;
}

template <>
std::vector<int> CSVToVector(const std::string& csv) {
    auto elems = StringSplit(csv, ",;");
    std::vector<int> values;
    values.reserve(elems.size());
    for (auto& elem : elems) {
        StringTrim(&elem);
        if (elem.empty()) {
            continue;
        }
        try {
            values.push_back(std::stoi(elem));
        }
        catch (const std::invalid_argument&) {
            return std::vector<int>(0);
        }
    }
    return values;
}

template <>
std::vector<float> CSVToVector(const std::string& csv) {
    auto elems = StringSplit(csv, ",;");
    std::vector<float> values;
    values.reserve(elems.size());
    for (auto& elem : elems) {
        StringTrim(&elem);
        if (elem.empty()) {
            continue;
        }
        try {
            values.push_back(std::stod(elem));
        }
        catch (const std::invalid_argument&) {
            return std::vector<float>(0);
        }
    }
    return values;
}

template <>
std::vector<double> CSVToVector(const std::string& csv) {
    auto elems = StringSplit(csv, ",;");
    std::vector<double> values;
    values.reserve(elems.size());
    for (auto& elem : elems) {
        StringTrim(&elem);
        if (elem.empty()) {
            continue;
        }
        try {
            values.push_back(std::stold(elem));
        }
        catch (const std::invalid_argument&) {
            return std::vector<double>(0);
        }
    }
    return values;
}

std::vector<std::string> ReadTextFileLines(const std::string& path) {
    std::ifstream file(path);
    CHECK(file.is_open()) << path;

    std::string line;
    std::vector<std::string> lines;
    while (std::getline(file, line)) {
        StringTrim(&line);

        if (line.empty()) {
            continue;
        }

        lines.push_back(line);
    }

    return lines;
}

void RemoveCommandLineArgument(const std::string& arg, int* argc, char** argv) {
    for (int i = 0; i < *argc; ++i) {
        if (argv[i] == arg) {
            for (int j = i + 1; j < *argc; ++j) {
                argv[i] = argv[j];
            }
            *argc -= 1;
            break;
        }
    }
}
