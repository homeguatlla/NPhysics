#include "pch.h"
#include "PotentialContact.h"

namespace NPhysics
{
	PotentialContact::PotentialContact(std::shared_ptr<PhysicsObject>& obj1, std::shared_ptr<PhysicsObject>& obj2)
	{
		mObjects.push_back(obj1);
		mObjects.push_back(obj2);
	}
}