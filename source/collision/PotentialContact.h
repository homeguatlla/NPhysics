#pragma once
#include "../PhysicsObject.h"
#include <memory>
#include <vector>

namespace NPhysics
{
	class PotentialContact
	{
		explicit PotentialContact(std::shared_ptr<PhysicsObject>& obj1, std::shared_ptr<PhysicsObject>& obj2);

	private:
		//Holds the physics objects that might be in contact
		std::vector<std::shared_ptr<PhysicsObject>> mObjects;
	};
};