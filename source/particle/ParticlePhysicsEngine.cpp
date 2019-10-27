#include "pch.h"
#include "ParticlePhysicsEngine.h"

namespace NPhysics
{
	void ParticlePhysicsEngine::AddParticle(std::shared_ptr<Particle>& particle)
	{
		bool found = std::find(mParticles.begin(), mParticles.end(), particle) != mParticles.end();

		if (!found)
		{
			mParticles.push_back(particle);
		}
	}

	void ParticlePhysicsEngine::RegisterParticleForceGenerator(std::shared_ptr<Particle>& particle, std::shared_ptr<IForceGenerator<Particle>>& forceGenerator)
	{
		assert(particle);

		AddParticle(particle);
		mRegistry.Add(particle, forceGenerator);
	}

	void ParticlePhysicsEngine::Update(real duration)
	{
		mRegistry.UpdateForces(duration);

		for (auto particle : mParticles)
		{
			particle->Integrate(duration);
			//std::cout << "velocity: " << particle->GetVelocity().x << ", " << particle->GetVelocity().y << ", " << particle->GetVelocity().z << "\n";
		}
	}
}