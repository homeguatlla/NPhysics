#pragma once
#include "../../IForceGenerator.h"
#include "../RigidBody.h"

namespace NPhysics
{
	class RigidBodyGravity : public IForceGenerator<RigidBody>
	{
	public:
		explicit RigidBodyGravity(const glm::vec3& gravity);
		virtual ~RigidBodyGravity() = default;

		// Heredado vía IRigidBodyForceGenerator
		void UpdateForce(std::shared_ptr<RigidBody> body, real duration) override;

	private:
		glm::vec3 mGravity;
	};
};

