#include "pch.h"
#include "ParticleImpulse.h"

namespace NPhysics
{
	ParticleImpulse::ParticleImpulse(const glm::vec3& force) : mForce(force)
	{
	}

	void ParticleImpulse::UpdateForce(std::shared_ptr<Particle> particle, real duration)
	{
		if (!particle->HasFiniteMass()) return;

		particle->AddForce(mForce * particle->GetMass());
	}
}
