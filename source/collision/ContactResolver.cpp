#include "pch.h"
#include "ContactResolver.h"
#include "PotentialContact.h"
#include "Contact.h"

#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <iostream>

namespace NPhysics
{
	ContactResolver::ContactResolver(
		std::vector<std::shared_ptr<PotentialContact>>& potentialContacts, 
		int numIterationsPerContactWhenResolvingInterpenetration,
		int numIterationsPerContactWhenResolvingVelocity) :
		mPotentialContacts(potentialContacts),
		mNumIterationsPerContactWhenResolvingInterpenetration(numIterationsPerContactWhenResolvingInterpenetration),
		mNumIterationsPerContactWhenResolvingVelocity(numIterationsPerContactWhenResolvingVelocity)
	{
	}

	void ContactResolver::Resolve(float elapsedTime)
	{
		for (const auto& potentialContact : mPotentialContacts)
		{
			potentialContact->Resolve(mContacts);
		}

		PerformActionOnEachContact(
			[&elapsedTime](const std::shared_ptr<Contact>& contact) {
				contact->Calculate(elapsedTime);
				contact->NotifyCollisionEnter();
			});

		if (!mContacts.empty())
		{
			ResolveInterpenetration(elapsedTime);
			ResolveVelocities(elapsedTime);
		}

		//Notify about a collision
		PerformActionOnEachContact(
			[&elapsedTime](const std::shared_ptr<Contact>& contact) {
				//notify both objects about a collision
				contact->NotifyCollisionExit();
			});
	}

	void ContactResolver::ResolveInterpenetration(float elapsedTime)
	{
		int iterations = 0;

		while (iterations < mNumIterationsPerContactWhenResolvingInterpenetration)
		{
			auto contactToResolve = FindContactWithLargerPenetration();
			if (contactToResolve->GetPenetration() <= 0.0f /*
				este c�digo se comenta debido a la modificacion de la velocidad para evitar las microcolisiones,
				si lo dejamos, sigue haciendo microcolisiones.
				|| 
				glm::epsilonEqual(contactToResolve->GetPenetration(), 0.0f, EPSILON2)*/)
			{
				//All interpenetrations have been resolved. No objects colliding.
				break;
			}

			contactToResolve->ApplyPositionChanges();

			PerformActionOnEachContact(
				[&contactToResolve](const std::shared_ptr<Contact>& contact) {
					contact->UpdatePenetration(contactToResolve);
				});

			iterations++;
		}
	}
	
	void ContactResolver::ResolveVelocities(float elapsedTime)
	{
		int iterations = 0;

		while (iterations < mNumIterationsPerContactWhenResolvingVelocity)
		{
			auto contactToResolve = FindContactWithLargerDesiredDeltaVelocity();
			if (contactToResolve->GetDesiredDeltaVelocity() < 0.0f || 
				glm::epsilonEqual(contactToResolve->GetDesiredDeltaVelocity(), 0.0f, EPSILON2))
			{
				//All velocities have been resolved. No objects colliding.
				break;
			}

			contactToResolve->ApplyVelocityChange();

			PerformActionOnEachContact(
				[&contactToResolve, &elapsedTime](const std::shared_ptr<Contact>& contact) {
					contact->UpdateLocalVelocity(contactToResolve, elapsedTime);
				});

			iterations++;
		}
	}

	void ContactResolver::PerformActionOnEachContact(std::function<void(const std::shared_ptr<Contact>& c)> action)
	{
		for (const auto& contact : mContacts)
		{
			action(contact);
		}
	}

	std::shared_ptr<Contact> ContactResolver::FindContactWithLargerPenetration()
	{
		auto it = std::max_element(mContacts.begin(), mContacts.end(),
			[](const std::shared_ptr<Contact>& lhs, const std::shared_ptr<Contact>& rhs) {
				return lhs->GetPenetration() < rhs->GetPenetration();
			});

		return *it;
	}

	std::shared_ptr<Contact> ContactResolver::FindContactWithLargerDesiredDeltaVelocity()
	{
		auto it = std::max_element(mContacts.begin(), mContacts.end(),
			[](const std::shared_ptr<Contact>& lhs, const std::shared_ptr<Contact>& rhs) {
				return lhs->GetDesiredDeltaVelocity() < rhs->GetDesiredDeltaVelocity();
			});

		return *it;
	}
}