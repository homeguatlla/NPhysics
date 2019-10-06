#pragma once
#include <memory>
#include <vector>
#include "../../../framework.h"

namespace NPhysics
{
	class ParticleContact;
}

namespace NPhysics {

	class ParticleContactResolver
	{
	public:
		using ParticleContactsList = std::vector<std::shared_ptr<ParticleContact>>;
		//Creates a new ParticleContactResolver
		ParticleContactResolver(unsigned int iterations);

		// Sets the number of iterations that can be used
		void SetIterations(unsigned int iterations);

		//Resolves a set of particle contacts for both penetration and velocity
		void ResolvesContacts(ParticleContactsList& contacts, real duration);

	private:
		//Holds the number of iterations allowed
		unsigned int mIterations;

		//Performance tracking, keep how many iterations were used
		unsigned int mIterationsUsed;
	};
};

