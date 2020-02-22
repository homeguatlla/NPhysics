#include "pch.h"
#include "ContactResolver.h"
#include "PotentialContact.h"

namespace NPhysics
{
	ContactResolver::ContactResolver(std::vector<std::shared_ptr<PotentialContact>>& potentialContacts) :
		mPotentialContacts(potentialContacts)
	{
	}

	void ContactResolver::Resolve()
	{
		for (const auto& potentialContact : mPotentialContacts)
		{
			potentialContact->Resolve();
		}
	}
}