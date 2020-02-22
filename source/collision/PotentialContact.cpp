#include "pch.h"
#include "PotentialContact.h"
#include "../bvh/BoundingVolumeIntersectionResolverMap.h"
#include "../bvh/boundingVolumes/IBoundingVolume.h"

namespace NPhysics
{
	PotentialContact::PotentialContact(const ContactData& data1, const ContactData& data2)
	{
		mObjects.push_back(data1);
		mObjects.push_back(data2);
	}
	std::shared_ptr<Contact> PotentialContact::Resolve() const
	{
		auto volume1 = mObjects[0].second;
		auto volume2 = mObjects[1].second;

		auto resolveCollisionFunction = BoundingVolumeIntersectionResolverMap::GetInstance().LookupCollisionResolverFunction(
			typeid(*volume1).name(),
			typeid(*volume2).name());

		assert(resolveCollisionFunction);
		return resolveCollisionFunction(*volume1.get(), *volume2.get());
	}
}