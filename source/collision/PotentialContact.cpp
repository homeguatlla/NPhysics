#include "pch.h"
#include "PotentialContact.h"
#include "../bvh/BoundingVolumeIntersectionResolverMap.h"

namespace NPhysics
{
	PotentialContact::PotentialContact(const ContactData& data1, const ContactData& data2)
	{
		mObjects.push_back(data1);
		mObjects.push_back(data2);
	}
	std::shared_ptr<Contact> PotentialContact::Resolve()
	{
		auto resolveCollisionFunction = BoundingVolumeIntersectionResolverMap::GetInstance().LookupCollisionResolverFunction(
			typeid(mObjects[0].second).name(),
			typeid(mObjects[1].second).name());

		assert(resolveCollisionFunction);
		return resolveCollisionFunction(*mObjects[0].second.get(), *mObjects[1].second.get());
	}
}