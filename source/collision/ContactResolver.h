#pragma once
#include<glm/glm.hpp>
#include <vector>
#include <memory>

namespace NPhysics
{
	class PotentialContact;
	class Contact;

	class ContactResolver
	{
	public:
		ContactResolver(std::vector<std::shared_ptr<PotentialContact>>& potentialContacts);
		virtual ~ContactResolver() = default;

		void Resolve(float elapsedTime);

	private:
		void ResolveInterpenetration(const std::shared_ptr<Contact>& contact, float elapsedTime);
		void ResolveVelocities(const std::shared_ptr<Contact>& contact, float elapsedTime);

	private:
		std::vector<std::shared_ptr<PotentialContact>> mPotentialContacts;
		std::vector<std::shared_ptr<Contact>> mContacts;
	};
};

