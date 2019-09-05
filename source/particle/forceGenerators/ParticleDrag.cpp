#include "pch.h"
#include "ParticleDrag.h"

namespace NPhysics
{
	ParticleDrag::ParticleDrag(real k1, real k2) : mK1(k1), mK2(k2)
	{
	}

	void ParticleDrag::UpdateForce(std::shared_ptr<Particle> particle, real duration)
	{
		glm::vec3 force(particle->GetVelocity());

		real dragCoeff = glm::length(force);
		dragCoeff = mK1 * dragCoeff + mK2 * dragCoeff * dragCoeff;
		
		force = glm::normalize(force);
		force *= -dragCoeff;

		particle->AddForce(force);
	}
}
