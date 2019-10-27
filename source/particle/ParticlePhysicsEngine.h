#pragma once

#include "../ForceRegistry.h"
#include "Particle.h"
#include <vector>
#include <memory>

namespace NPhysics
{
	class ParticlePhysicsEngine
	{
		std::vector<std::shared_ptr<Particle>> mParticles;
		ForceRegistry<Particle> mRegistry;

	public:
		void AddParticle(std::shared_ptr<Particle>& particle);
		void RegisterParticleForceGenerator(std::shared_ptr<Particle>& particle, std::shared_ptr<IForceGenerator<Particle>>& forceGenerator);
		void Update(real duration);
	};
};

