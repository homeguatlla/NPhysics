#pragma once
#include <memory>
#include <vector>

namespace NPhysics
{
	class PhysicsObject;

	class PotentialContact
	{
	public:
		explicit PotentialContact(std::shared_ptr<PhysicsObject> obj1, std::shared_ptr<PhysicsObject> obj2);

	private:
		//Holds the physics objects that might be in contact
		std::vector<std::shared_ptr<PhysicsObject>> mObjects;
	};
};