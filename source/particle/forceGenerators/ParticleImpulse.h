#pragma once
#include "../../IForceGenerator.h"
#include "../Particle.h"

namespace NPhysics
{
	class ParticleImpulse : public IForceGenerator<Particle>
	{
	public:
		explicit ParticleImpulse(const glm::vec3& force);
		virtual ~ParticleImpulse() = default;

		// Heredado vía IParticleForceGenerator
		void UpdateForce(std::shared_ptr<Particle> particle, real duration) override;

	private:
		glm::vec3 mForce;
	};
};

