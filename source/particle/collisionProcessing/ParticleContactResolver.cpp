#include "pch.h"
#include "ParticleContactResolver.h"
#include "ParticleContact.h"

namespace NPhysics
{
	ParticleContactResolver::ParticleContactResolver(unsigned int iterations) :
		mIterations{ iterations }
	{
	}

	void ParticleContactResolver::SetIterations(unsigned int iterations)
	{
		mIterations = iterations;
	}

	void ParticleContactResolver::ResolvesContacts(ParticleContactsList& contacts, real duration)
	{
		mIterationsUsed = 0;
		while (mIterationsUsed < mIterations)
		{
			//Find the contact with the largest closing velocity to resolve first
			real max = 0.0f;
			unsigned int maxIndex = contacts.size();

			for (unsigned int i = 0; i < contacts.size(); ++i)
			{
				real sepVel = contacts[i]->CalculateSeparitingVelocity();
				if (sepVel < max)
				{
					max = sepVel;
					maxIndex = i;
				}
			}

			//Resolve the max sep vel contact
			contacts[maxIndex]->Resolve(duration);

			mIterationsUsed++;
		}
	}
}