#pragma once
#include "source/particle/ParticleForceRegistry.h"

namespace NPhysics
{
	class NPhysicsEngine
	{
	public:
		void AddParticle(std::shared_ptr<Particle>& particle);
		void RegisterParticleForceGenerator(std::shared_ptr<Particle>& particle, std::shared_ptr<IParticleForceGenerator>& forceGenerator);
		void Update(real duration);

	private:
		std::vector<std::shared_ptr<Particle>> mParticles;
		ParticleForceRegistry mRegistry;
	};
};

