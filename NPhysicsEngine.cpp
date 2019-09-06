// NPhysics.cpp : Define las funciones de la biblioteca estática.
//


#include "pch.h"
#include "NPhysicsEngine.h"

#include <glm/glm.hpp>
#include <algorithm>

namespace NPhysics
{
	void NPhysicsEngine::AddParticle(std::shared_ptr<Particle>& particle)
	{
		bool found = std::find(mParticles.begin(), mParticles.end(), particle) != mParticles.end();

		if (!found)
		{
			mParticles.push_back(particle);
		}
	}

	void NPhysicsEngine::RegisterParticleForceGenerator(std::shared_ptr<Particle>& particle, std::shared_ptr<IParticleForceGenerator>& forceGenerator)
	{
		AddParticle(particle);
		mRegistry.Add(particle, forceGenerator);
	}

	void NPhysicsEngine::Update(real duration)
	{
		mRegistry.UpdateForces(duration);

		for (auto particle : mParticles)
		{
			particle->Integrate(duration);
		}
	}
}