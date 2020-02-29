#pragma once
#include<glm/glm.hpp>
#include <vector>
#include <memory>
#include <functional>

namespace NPhysics
{
	class PotentialContact;
	class Contact;

	class ContactResolver
	{
	public:
		ContactResolver(
			std::vector<std::shared_ptr<PotentialContact>>& potentialContacts, 
			int numIterationsPerContactWhenResolvingInterpenetration = 4,
			int numIterationsPerContactWhenResolvingVelocity = 4);
		virtual ~ContactResolver() = default;

		void Resolve(float elapsedTime);

	private:
		void ResolveInterpenetration(float elapsedTime);
		void ResolveVelocities(float elapsedTime);
		std::shared_ptr<Contact> FindContactWithLargerPenetration();
		std::shared_ptr<Contact> FindContactWithLargerDesiredDeltaVelocity();
		void PerformActionOnEachContact(std::function<void(const std::shared_ptr<Contact> & c)> action);

	private:
		std::vector<std::shared_ptr<PotentialContact>> mPotentialContacts;
		std::vector<std::shared_ptr<Contact>> mContacts;

		int mNumIterationsPerContactWhenResolvingInterpenetration;
		int mNumIterationsPerContactWhenResolvingVelocity;
	};
};

