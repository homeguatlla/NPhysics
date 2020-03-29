#include "pch.h"
#include "RigidBodyPhysicsEngine.h"
#include "RigidBody.h"
#include "../bvh/boundingVolumes/IBoundingVolume.h"
#include "../utils/Math.h"
#include <iostream>

namespace NPhysics
{
	void NPhysics::RigidBodyPhysicsEngine::AddRigidBody(std::shared_ptr<RigidBody> body, const std::shared_ptr<IBoundingVolume> volume)
	{
		if (body->IsStatic())
		{
			AddRigidBody(mStaticBodies, body, volume);
		}
		else
		{
			AddRigidBody(mDynamicBodies, body, volume);
		}
	}

	void NPhysics::RigidBodyPhysicsEngine::AddRigidBody(BodiesVector& bodiesVector, std::shared_ptr<RigidBody> body, const std::shared_ptr<IBoundingVolume> volume)
	{
		auto found = std::find_if(
			bodiesVector.begin(),
			bodiesVector.end(),
			[&body](const std::pair<std::shared_ptr<RigidBody>, std::shared_ptr<IBoundingVolume>>& element)
			{ return element.first == body; }) != bodiesVector.end();

		if (!found)
		{
			bodiesVector.push_back(std::make_pair(body, volume));
			mCollisionResolver.AddCollider(body, volume);
		}
	}

	void RigidBodyPhysicsEngine::RegisterForceGenerator(std::shared_ptr<RigidBody> body, const std::shared_ptr<IBoundingVolume> volume, std::shared_ptr<IForceGenerator<RigidBody>> forceGenerator)
	{
		assert(body);

		AddRigidBody(body, volume);
		mRegistry.Add(body, forceGenerator);
	}

	void RigidBodyPhysicsEngine::Update(real duration)
	{
		mRegistry.UpdateForces(duration);

		/*for (auto body : mStaticBodies)
		{
			body.first->Integrate(duration);
			//std::cout << "velocity: " << particle->GetVelocity().x << ", " << particle->GetVelocity().y << ", " << particle->GetVelocity().z << "\n";
		}*/

		for (auto body : mDynamicBodies)
		{
			body.first->Integrate(duration);
		}
		UpdateBoundingVolumeHierarchy();

		mCollisionResolver.Update(duration);

		UpdateBoundingVolumeHierarchy();
	}

	void RigidBodyPhysicsEngine::UpdateBoundingVolumeHierarchy()
	{
		for (auto body : mDynamicBodies)
		{
			glm::vec3 position1 = body.second->GetPosition();
			glm::vec3 position2 = body.first->GetPosition();
			
			bool hasChanged = !NMath::IsNearlyEqual(position1, position2, glm::epsilon<float>());
			if (hasChanged)
			{
				body.second->SetPosition(body.first->GetPosition());
				mCollisionResolver.UpdateCollider(body.first, body.second);
			}
		}
	}
}
