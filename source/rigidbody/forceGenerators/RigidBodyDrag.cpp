#include "pch.h"
#include "RigidBodyDrag.h"

namespace NPhysics
{
	RigidBodyDrag::RigidBodyDrag(real k1, real k2) : mK1(k1), mK2(k2)
	{
	}

	void RigidBodyDrag::UpdateForce(std::shared_ptr<RigidBody> body, real duration)
	{
		glm::vec3 force(body->GetVelocity());

		real dragCoeff = glm::length(force);
		dragCoeff = mK1 * dragCoeff + mK2 * dragCoeff * dragCoeff;

		//We can not normalize a force = 0.0f vector
		if (glm::length2(force) >= EPSILON * EPSILON)
		{
			force = glm::normalize(force);
		}

		force *= -dragCoeff;

		body->AddForce(force);
	}
}
