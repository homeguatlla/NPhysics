#include "pch.h"
#include "RigidBodyPhysicsEngine.h"
#include "RigidBody.h"

namespace NPhysics
{
	void NPhysics::RigidBodyPhysicsEngine::AddRigidBody(std::shared_ptr<RigidBody>& body)
	{
		bool found = std::find(mBodies.begin(), mBodies.end(), body) != mBodies.end();

		if (!found)
		{
			mBodies.push_back(body);
		}
	}

	void RigidBodyPhysicsEngine::RegisterForceGenerator(std::shared_ptr<RigidBody> & body, std::shared_ptr<IForceGenerator<RigidBody>> & forceGenerator)
	{
		assert(body);

		AddRigidBody(body);
		mRegistry.Add(body, forceGenerator);
	}

	void RigidBodyPhysicsEngine::Update(real duration)
	{
		mRegistry.UpdateForces(duration);

		for (auto body : mBodies)
		{
			body->Integrate(duration);
			//std::cout << "velocity: " << particle->GetVelocity().x << ", " << particle->GetVelocity().y << ", " << particle->GetVelocity().z << "\n";
		}
	}
}
