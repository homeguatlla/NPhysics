#include "pch.h"
#include "RigidBodyPhysicsEngine.h"
#include "RigidBody.h"
#include "../bvh/BoundingVolumeHierarchyNode.h"
#include "../bvh/BoundingVolumeIntersectionResolverMap.h"
#include "../utils/Math.h"

namespace NPhysics
{
	RigidBodyPhysicsEngine::RigidBodyPhysicsEngine()
	{
		BoundingVolumeIntersectionResolverMap::GetInstance().AddEntry(
			std::string("SphereBoundingVolume"), 
			std::string("SphereBouningVolume"),
			std::bind<bool>(&NMath::IsOverlapping, std::placeholders::_1, std::placeholders::_2),
			std::bind<std::shared_ptr<IBoundingVolume>>(&NMath::MergeBoundingVolumes, std::placeholders::_1, std::placeholders::_2));
		mBoundingVolumeHierarchyRoot = std::make_shared<BoundingVolumeHierarchyNode>();

	}

	void NPhysics::RigidBodyPhysicsEngine::AddRigidBody(std::shared_ptr<RigidBody> body, const std::shared_ptr<IBoundingVolume> volume)
	{
		bool found = std::find(mBodies.begin(), mBodies.end(), body) != mBodies.end();

		if (!found)
		{
			mBodies.push_back(body);
			mBoundingVolumeHierarchyRoot->Insert(body, volume);
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

		for (auto body : mBodies)
		{
			body->Integrate(duration);
			//std::cout << "velocity: " << particle->GetVelocity().x << ", " << particle->GetVelocity().y << ", " << particle->GetVelocity().z << "\n";
		}
	}
}
