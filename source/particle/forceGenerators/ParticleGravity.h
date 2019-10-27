#pragma once
#include "../../IForceGenerator.h"
#include <glm/glm.hpp>

namespace NPhysics
{
	class Particle;

	class ParticleGravity : public IForceGenerator<Particle>
	{
	public:
		explicit ParticleGravity(const glm::vec3& gravity);
		virtual ~ParticleGravity() = default;

		// Heredado vía IParticleForceGenerator
		void UpdateForce(std::shared_ptr<Particle> particle, real duration) override;

	private:
		glm::vec3 mGravity;
	};
};

