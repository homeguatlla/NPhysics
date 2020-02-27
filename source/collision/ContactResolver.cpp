#include "pch.h"
#include "ContactResolver.h"
#include "PotentialContact.h"
#include "Contact.h"


#include <iostream>

namespace NPhysics
{
	ContactResolver::ContactResolver(std::vector<std::shared_ptr<PotentialContact>>& potentialContacts) :
		mPotentialContacts(potentialContacts)
	{
	}

	void ContactResolver::Resolve(float elapsedTime)
	{
		for (const auto& potentialContact : mPotentialContacts)
		{
			potentialContact->Resolve(mContacts);
		}

		for (const auto& contact : mContacts)
		{
			contact->Calculate(elapsedTime);

			ResolveInterpenetration(contact, elapsedTime);

			ResolveVelocities(contact, elapsedTime);
		}
	}

	void ContactResolver::ResolveInterpenetration(const std::shared_ptr<Contact>& contact, float elapsedTime)
	{

	}

	void ContactResolver::ResolveVelocities(const std::shared_ptr<Contact>& contact, float elapsedTime)
	{

	}
}