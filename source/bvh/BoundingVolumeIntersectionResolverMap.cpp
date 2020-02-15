#include "pch.h"
#include "BoundingVolumeIntersectionResolverMap.h"

namespace NPhysics
{
void BoundingVolumeIntersectionResolverMap::AddEntry(const std::string& type1, const std::string& type2, OverlappingFunctionPtr overlappingFunction, MergeFunctionPtr mergeFunction, bool isSymmetric)
{
	auto key = std::make_pair(type1, type2);
	BoundingVolumeIntersectionResolverMap::ResolverEntryMap::iterator mapEntry = mBoundingVolumeIntersectionResolvers.find(key);
	if (mapEntry == mBoundingVolumeIntersectionResolvers.end())
	{
		ResolverEntry resolverEntry;
		resolverEntry.overlappingFunction = overlappingFunction;
		resolverEntry.mergeFunction = mergeFunction;

		mBoundingVolumeIntersectionResolvers[key] = resolverEntry;
		if (isSymmetric)
		{
			mBoundingVolumeIntersectionResolvers[std::make_pair(type2, type1)] = resolverEntry;
		}
	}
}

void BoundingVolumeIntersectionResolverMap::RemoveEntry(const std::string & type1, const std::string & type2)
{
}

bool BoundingVolumeIntersectionResolverMap::FindResolverEntry(const std::string& type1, const std::string& type2, ResolverEntry& entry)
{
	auto key = std::make_pair(type1, type2);
	ResolverEntryMap::iterator mapEntry = mBoundingVolumeIntersectionResolvers.find(key);
	if (mapEntry != mBoundingVolumeIntersectionResolvers.end())
	{
		entry = mBoundingVolumeIntersectionResolvers[key];
		return true;
	}
	else
	{
		return false;
	}
}

BoundingVolumeIntersectionResolverMap::OverlappingFunctionPtr BoundingVolumeIntersectionResolverMap::LookupOverlappingFunction(const std::string & type1, const std::string & type2)
{
	ResolverEntry entry;
	if (FindResolverEntry(type1, type2, entry))
	{
		return entry.overlappingFunction;
	}
	else
	{
		return nullptr;
	}
}

BoundingVolumeIntersectionResolverMap::MergeFunctionPtr BoundingVolumeIntersectionResolverMap::LookupMergeFunction(const std::string& type1, const std::string& type2)
{
	ResolverEntry entry;
	if (FindResolverEntry(type1, type2, entry))
	{
		return entry.mergeFunction;
	}
	else
	{
		return nullptr;
	}
}

BoundingVolumeIntersectionResolverMap& BoundingVolumeIntersectionResolverMap::GetInstance()
{
	static BoundingVolumeIntersectionResolverMap mBVIntersectionResolver;
	return mBVIntersectionResolver;
}
}
