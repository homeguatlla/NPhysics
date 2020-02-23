#pragma once
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

		void Resolve();

	private:
		std::vector<std::shared_ptr<PotentialContact>> mPotentialContacts;
		std::vector<std::shared_ptr<Contact>> mContacts;
	};
};

