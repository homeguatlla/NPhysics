#pragma once
#include "../IParticleForceGenerator.h"

namespace NPhysics
{
	class ParticleGravity : public IParticleForceGenerator
	{
	public:
		explicit ParticleGravity(const glm::vec3& gravity);

		// Heredado vía IParticleForceGenerator
		void UpdateForce(std::shared_ptr<Particle> particle, real duration) override;

	private:
		glm::vec3 mGravity;
	};
};

