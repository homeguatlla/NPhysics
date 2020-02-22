#pragma once
#include <memory>
#include <vector>
#include <utility>

namespace NPhysics
{
	class PhysicsObject;
	class IBoundingVolume;
	class Contact;

	class PotentialContact
	{
	public:
		using ContactData = std::pair<std::shared_ptr<PhysicsObject>, std::shared_ptr<IBoundingVolume>>;

		explicit PotentialContact(const ContactData& obj1, const ContactData& obj2);
		std::shared_ptr<Contact> Resolve() const;

	private:
		//Holds the physics objects that might be in contact
		std::vector<ContactData> mObjects;
	};
};