#include "Receiver.h"
#include "../Base/macros.h"
#include "../Base/misc.h"

Receiver::Receiver(CallbackFunction callback) :callback(callback)
{
	running = false;
    ClearRegistryValue();
}
void Receiver::Start()
{
    running = true; // 设置运行标志为真
    loopThread = std::thread([&]() 
        {
        while (running)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            const std::string value = ReadRegistryValue();
            if (!value.empty())
            {
                if (callback)
                {
#ifdef DEMO_MODE
                    if (ExistsFile(value))
                    {
                        callback(value);
                    }
                    else
                    {
                        callback("D:/Code/On-the-fly SfM Server/SfMServer/" + value); // 调用回调函数
                    }
                    
#else
                    callback(value); // 调用回调函数
#endif // DEMO_MODE
                }
                ClearRegistryValue();
            }
        }
        });
}
void Receiver::Stop()
{
    running = false; // 设置运行标志为假，使循环结束
    if (loopThread.joinable())
    {
        loopThread.join(); // 等待线程完成
    }
}
std::string Receiver::ReadRegistryValue()
{
    HKEY hKey;
    if (RegOpenKeyEx(HKEY_CURRENT_USER, TEXT("SOFTWARE\\RTPS\\CamFiTransmit"), 0, KEY_READ, &hKey) == ERROR_SUCCESS) {
        DWORD type, size = 1024 * sizeof(wchar_t); // 预留足够的空间
        std::vector<wchar_t> value(size / sizeof(wchar_t)); // 使用vector动态数组以便处理不同长度的字符串
        if (RegQueryValueEx(hKey, TEXT("NewImage"), NULL, &type, reinterpret_cast<LPBYTE>(value.data()), &size) == ERROR_SUCCESS && type == REG_SZ) {
            RegCloseKey(hKey);
            // 计算实际的字符串长度，确保不超过返回的大小
            std::wstring wsValue(value.begin(), std::find(value.begin(), value.end(), L'\0'));
            // 转换宽字符串到多字节字符串
            int len = WideCharToMultiByte(CP_UTF8, 0, wsValue.c_str(), -1, NULL, 0, NULL, NULL);
            std::vector<char> mbStr(len);
            WideCharToMultiByte(CP_UTF8, 0, wsValue.c_str(), -1, &mbStr[0], len, NULL, NULL);
            return std::string(mbStr.begin(), mbStr.end() - 1); // 移除多余的终止字符
        }
        RegCloseKey(hKey);
    }
    return "";
}
void Receiver::ClearRegistryValue()
{
    HKEY hKey;
    if (RegOpenKeyEx(HKEY_CURRENT_USER, TEXT("SOFTWARE\\RTPS\\CamFiTransmit"), 0, KEY_WRITE, &hKey) == ERROR_SUCCESS)
    {
        RegSetValueEx(hKey, TEXT("NewImage"), 0, REG_SZ, (LPBYTE)"", 1);
        RegCloseKey(hKey);
    }
}
