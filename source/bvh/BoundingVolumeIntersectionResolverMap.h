#pragma once

#include <string>
#include <map>
#include <functional>
#include <memory>


namespace NPhysics
{
	class IBoundingVolume;
	class Contact;

	class BoundingVolumeIntersectionResolverMap
	{
	public:
		using OverlappingFunctionPtr = std::function<bool(const IBoundingVolume&, const IBoundingVolume&)>;
		using MergeFunctionPtr = std::function<std::shared_ptr<IBoundingVolume>(const IBoundingVolume&, const IBoundingVolume&)>;
		using ContainsFunctionPtr = std::function<bool(const IBoundingVolume&, const IBoundingVolume&)>;
		using CollisionResolverFunctionPtr = std::function<std::shared_ptr<Contact>(const IBoundingVolume&, const IBoundingVolume&)>;
	private:
		typedef struct ResolverEntry
		{
			OverlappingFunctionPtr overlappingFunction;
			MergeFunctionPtr mergeFunction;
			ContainsFunctionPtr containsFunction;
			CollisionResolverFunctionPtr collisionResolverFunction;
		}ResolverEntry;

	public:
		void AddEntry(
			const std::string& type1, 
			const std::string& type2, 
			OverlappingFunctionPtr collisionFunction, 
			MergeFunctionPtr mergeFunction, 
			ContainsFunctionPtr containsFunction, 
			CollisionResolverFunctionPtr collisionResolverFunction,
			bool symmetric = true); // see below
		void RemoveEntry(const std::string& type1, const std::string& type2);
		
		OverlappingFunctionPtr LookupOverlappingFunction(const std::string& type1, const std::string& type2);
		MergeFunctionPtr LookupMergeFunction(const std::string& type1, const std::string& type2);
		ContainsFunctionPtr LookupContainsFunction(const std::string& type1, const std::string& type2);
		CollisionResolverFunctionPtr LookupCollisionResolverFunction(const std::string& type1, const std::string& type2);
		static BoundingVolumeIntersectionResolverMap& GetInstance();

	private:
		// these functions are private to prevent the creation
		// of multiple maps — see Item 26
		BoundingVolumeIntersectionResolverMap() = default;
		//BoundingVolumeIntersectionResolverMap(const BoundingVolumeIntersectionResolverMap&);
		bool FindResolverEntry(const std::string& type1, const std::string& type2, ResolverEntry& entry);
		
		using ResolverEntryMap = std::map<std::pair<std::string, std::string>, ResolverEntry>;
		ResolverEntryMap mBoundingVolumeIntersectionResolvers;
	};
};

