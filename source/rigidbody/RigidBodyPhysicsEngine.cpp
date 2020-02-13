#include "pch.h"
#include "RigidBodyPhysicsEngine.h"
#include "RigidBody.h"
#include "../bvh/BoundingVolumeHierarchyNode.h"
#include "../bvh/BoundingVolumeIntersectionResolverMap.h"
#include "../bvh/boundingVolumes/SphereBoundingVolume.h"


namespace NPhysics
{
	RigidBodyPhysicsEngine::RigidBodyPhysicsEngine()
	{
		BoundingVolumeIntersectionResolverMap::GetInstance().AddEntry(
			typeid(SphereBoundingVolume).name(), 
			typeid(SphereBoundingVolume).name(),
			std::bind<bool>(&RigidBodyPhysicsEngine::IsOverlapping, this, std::placeholders::_1, std::placeholders::_2),
			std::bind<std::shared_ptr<IBoundingVolume>>(&RigidBodyPhysicsEngine::MergeBoundingVolumes, this, std::placeholders::_1, std::placeholders::_2));
			
		mBoundingVolumeHierarchyRoot = std::make_shared<BoundingVolumeHierarchyNode>();
	}

	bool RigidBodyPhysicsEngine::IsOverlapping(std::shared_ptr<const IBoundingVolume> volume1, std::shared_ptr<const IBoundingVolume> volume2) const
	{
		auto sphere1 = std::dynamic_pointer_cast<const SphereBoundingVolume>(volume1);
		auto sphere2 = std::dynamic_pointer_cast<const SphereBoundingVolume>(volume2);
		real distance = glm::distance(sphere1->GetCenter(), sphere2->GetCenter());
		return distance < sphere1->GetRadius() + sphere2->GetRadius();
	}

	std::shared_ptr<IBoundingVolume> RigidBodyPhysicsEngine::MergeBoundingVolumes(std::shared_ptr<const IBoundingVolume> volume1, std::shared_ptr<const IBoundingVolume> volume2) const
	{
		auto sphere1 = std::dynamic_pointer_cast<const SphereBoundingVolume>(volume1);
		auto sphere2 = std::dynamic_pointer_cast<const SphereBoundingVolume>(volume2);
		auto newVolume = SphereBoundingVolume(*sphere1.get(), *sphere2.get());

		return std::make_shared<SphereBoundingVolume>(newVolume);
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
