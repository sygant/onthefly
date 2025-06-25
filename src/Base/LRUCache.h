#pragma once
#include <list>
#include <unordered_map>
#include <functional>

#include "Base.h"


// LRU缓存(最近最少使用缓存)
template <typename CKeyType, typename CValueType>
class CLRUCache
{
public:
	CLRUCache(size_t maxSize, const std::function<CValueType(const CKeyType&)>& getterFunction): maxSize(maxSize), getterFunction(getterFunction)
	{
		CHECK(!!getterFunction);
		CHECK(maxSize > 0);
	}
	~CLRUCache() = default;
	size_t GetNumElements() const noexcept
	{
		return elementsMap.size();
	}
	size_t GetMaxNumElements() const noexcept
	{
		return maxSize;
	}
	bool IsExists(const CKeyType& key) const
	{
		return elementsMap.find(key) != elementsMap.end();
	}

	// 从Cache中弹出最近最少使用的元素
	virtual void Pop()
	{
		if (elements.empty())return;
		auto last = elements.rbegin();
		elementsMap.erase(last->first);
		elements.pop_back();
	}
	virtual void Set(const CKeyType& key, const CValueType& value)
	{
		auto it = elementsMap.find(key);
		elements.emplace_front(key, value);
		if (it != elementsMap.end())
		{
			elements.erase(it->second);
			elementsMap.erase(it);
		}
		elementsMap[key] = elements.begin();
		if (elementsMap.size() > maxSize)
		{
			Pop();
		}
	}
	const CValueType& Get(const CKeyType& key)
	{
		return GetRefer(key);
	}
	CValueType& GetRefer(const CKeyType& key)
	{
		const auto it = elementsMap.find(key);
		if (it == elementsMap.end())
		{
			Set(key, std::move(getterFunction(key)));
			return elementsMap[key]->second;
		}
		elements.splice(elements.begin(), elements, it->second);
		return it->second->second;
	}
	virtual void Clear() noexcept
	{
		elements.clear();
		elementsMap.clear();
	}

protected:
	using CListIter = typename std::list<std::pair<CKeyType, CValueType>>::iterator;

	const size_t maxSize;
	std::list<std::pair<CKeyType, CValueType>> elements;
	std::unordered_map<CKeyType, CListIter> elementsMap;
	const std::function<CValueType(const CKeyType&)> getterFunction; // 如果key不在Cache中的时候, 计算新value的函数
};
