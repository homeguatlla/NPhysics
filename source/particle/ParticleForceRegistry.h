#pragma once
#include "Particle.h"
#include "IParticleForceGenerator.h"

#include <vector>

namespace NPhysics {

	class ParticleForceRegistry
	{
		struct ParticleForceRegistration
		{
			ParticleForceRegistration(std::shared_ptr<Particle>& p, std::shared_ptr<IParticleForceGenerator>& fg) :
				particle(p),
				forceRegistrator(fg) {};

			std::shared_ptr<Particle> particle;
			std::shared_ptr<IParticleForceGenerator> forceRegistrator;
		};

		using Registry = std::vector<ParticleForceRegistration>;

	public:
		void Add(std::shared_ptr<Particle>& particle, std::shared_ptr<IParticleForceGenerator>& forceGenerator);
		void Remove(std::shared_ptr<Particle>& particle, std::shared_ptr<IParticleForceGenerator>& forceGenerator);
		void Clear();
		void UpdateForces(real duration);

	private:
		Registry mRegistrations;
	};
};

