#include "pch.h"
#include "PotentialContact.h"
#include "../bvh/BoundingVolumeIntersectionResolverMap.h"
#include "../bvh/boundingVolumes/IBoundingVolume.h"
#include "../rigidbody/RigidBody.h"
#include "Contact.h"

namespace NPhysics
{
	PotentialContact::PotentialContact(const ContactData& data1, const ContactData& data2)
	{
		mObjects.push_back(data1);
		mObjects.push_back(data2);
	}

	void PotentialContact::Resolve(std::vector<std::shared_ptr<Contact>>& contacts) const
	{
		auto volume1 = mObjects[0].second;
		auto volume2 = mObjects[1].second;

		auto resolveCollisionFunction = BoundingVolumeIntersectionResolverMap::GetInstance().LookupCollisionResolverFunction(
			typeid(*volume1).name(),
			typeid(*volume2).name());

		assert(resolveCollisionFunction);
		auto contact = resolveCollisionFunction(*volume1.get(), *volume2.get());
		if (contact != nullptr)
		{
			contact->SetBodies(mObjects[0].first, mObjects[1].first);
			auto rigidbody1 = std::dynamic_pointer_cast<RigidBody>(mObjects[0].first);
			auto rigidbody2 = std::dynamic_pointer_cast<RigidBody>(mObjects[1].first);
			float cor = (rigidbody1->GetRestitution() + rigidbody2->GetRestitution()) * 0.5f;
			contact->SetRestitution(cor);
			contact->SetFriction(0.0f);

			contacts.push_back(contact);
		}
	}
}