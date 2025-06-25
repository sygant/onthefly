#pragma once
#include <windows.h>
#include <string>
#include <chrono>
#include <thread>
#include <iostream>
#include <functional>
#include <atomic>



class Receiver
{
public:
	using CallbackFunction = std::function<void(const std::string&)>;
	Receiver(CallbackFunction callback);
	void Start();
	void Stop();






private:
	CallbackFunction callback;
	std::string ReadRegistryValue();
	void ClearRegistryValue();
	std::thread loopThread;
	std::atomic<bool> running = false;
};

























