#include "pch.h"
#include "ParticleGravity.h"

namespace NPhysics
{
	ParticleGravity::ParticleGravity(const glm::vec3& gravity) : mGravity(gravity)
	{
	}

	void ParticleGravity::UpdateForce(std::shared_ptr<Particle> particle, real duration)
	{
		if (!particle->HasInfinteMass()) return;

		particle->AddForce(mGravity * particle->GetMass());
	}
}
