#pragma once
#include "../../framework.h"
#include "../PhysicsObject.h"
#include <glm/glm.hpp>

namespace NPhysics {

	class Particle : public PhysicsObject
	{
	public:
		explicit Particle(const glm::vec3& initialPosition, const glm::vec3& initialVelocity, bool isStatic);
		~Particle() = default;

		void Integrate(real duration);
		void AddForce(const glm::vec3& force);

	private:
		void DoResetForceAccumulated() override;
	};
};
