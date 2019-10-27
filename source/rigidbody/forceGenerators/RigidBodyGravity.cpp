#include "pch.h"
#include "RigidBodyGravity.h"

namespace NPhysics
{
	RigidBodyGravity::RigidBodyGravity(const glm::vec3& gravity) : mGravity(gravity)
	{
	}

	void RigidBodyGravity::UpdateForce(std::shared_ptr<RigidBody> body, real duration)
	{
		if (!body->HasFiniteMass()) return;

		body->AddForce(mGravity * body->GetMass());
	}
}
