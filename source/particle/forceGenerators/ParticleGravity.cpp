#include "pch.h"
#include "ParticleGravity.h"
#include "../Particle.h"

namespace NPhysics
{
	ParticleGravity::ParticleGravity(const glm::vec3& gravity) : mGravity(gravity)
	{
	}

	void ParticleGravity::UpdateForce(std::shared_ptr<Particle> particle, real duration)
	{
		if (!particle->HasFiniteMass()) return;

		particle->AddForce(mGravity * particle->GetMass());
	}
}
