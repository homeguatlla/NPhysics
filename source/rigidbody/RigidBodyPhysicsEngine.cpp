#include "pch.h"
#include "RigidBodyPhysicsEngine.h"
#include "RigidBody.h"
#include "../bvh/BoundingVolumeHierarchyNode.h"
#include "../bvh/BoundingVolumeIntersectionResolverMap.h"
#include "../bvh/boundingVolumes/SphereBoundingVolume.h"
#include "../bvh/boundingVolumes/BoxBoundingVolume.h"
#include "../utils/Math.h"

#include <iostream>

namespace NPhysics
{
	RigidBodyPhysicsEngine::RigidBodyPhysicsEngine()
	{
		RegisterCollisionBoundingVolumesSupported();
		mBoundingVolumeHierarchyRoot = std::make_shared<BoundingVolumeHierarchyNode>();
	}

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

		CheckCollisions();
	}

	void RigidBodyPhysicsEngine::UpdateBoundingVolumeHierarchy()
	{
		for (auto body : mDynamicBodies)
		{
			body.second->SetPosition(body.first->GetPosition());
			mBoundingVolumeHierarchyRoot->UpdateBoundingVolume(body.first, body.second);
		}
	}

	void RigidBodyPhysicsEngine::CheckCollisions()
	{
		std::vector<std::shared_ptr<PotentialContact>> potentialContacts;
		mBoundingVolumeHierarchyRoot->GetPotentialContacts(potentialContacts, MAX_CONTACTS);

		//std::cout << "Num potential contacts " << potentialContacts.size() << "\n";
	}

	void RigidBodyPhysicsEngine::RegisterCollisionBoundingVolumesSupported()
	{
		BoundingVolumeIntersectionResolverMap::GetInstance().AddEntry(
			typeid(SphereBoundingVolume).name(),
			typeid(SphereBoundingVolume).name(),
			[](const IBoundingVolume& volume1, const IBoundingVolume& volume2) {
				auto sphere1 = dynamic_cast<const SphereBoundingVolume&>(volume1);
				auto sphere2 = dynamic_cast<const SphereBoundingVolume&>(volume2);
				return NMath::IsOverlapping(sphere1, sphere2);
			},
			[](const IBoundingVolume& volume1, const IBoundingVolume& volume2) {
				auto sphere1 = dynamic_cast<const SphereBoundingVolume&>(volume1);
				auto sphere2 = dynamic_cast<const SphereBoundingVolume&>(volume2);

				return NMath::MergeBoundingVolumes(sphere1, sphere2);
			},
				[](const IBoundingVolume& volume1, const IBoundingVolume& volume2) {
				auto sphere1 = dynamic_cast<const SphereBoundingVolume&>(volume1);
				auto sphere2 = dynamic_cast<const SphereBoundingVolume&>(volume2);
				return NMath::Contains(sphere1, sphere2);
			});

		BoundingVolumeIntersectionResolverMap::GetInstance().AddEntry(
			typeid(BoxBoundingVolume).name(),
			typeid(BoxBoundingVolume).name(),
			[](const IBoundingVolume& volume1, const IBoundingVolume& volume2) {
				auto box1 = dynamic_cast<const BoxBoundingVolume&>(volume1);
				auto box2 = dynamic_cast<const BoxBoundingVolume&>(volume2);
				return NMath::IsOverlapping(box1, box2);
			},
			[](const IBoundingVolume& volume1, const IBoundingVolume& volume2) {
				auto box1 = dynamic_cast<const BoxBoundingVolume&>(volume1);
				auto box2 = dynamic_cast<const BoxBoundingVolume&>(volume2);

				return NMath::MergeBoundingVolumes(box1, box2);
			},
				[](const IBoundingVolume& volume1, const IBoundingVolume& volume2) {
				auto box1 = dynamic_cast<const BoxBoundingVolume&>(volume1);
				auto box2 = dynamic_cast<const BoxBoundingVolume&>(volume2);
				return NMath::Contains(box1, box2);
			});

		BoundingVolumeIntersectionResolverMap::GetInstance().AddEntry(
			typeid(BoxBoundingVolume).name(),
			typeid(SphereBoundingVolume).name(),
			[](const IBoundingVolume& volume1, const IBoundingVolume& volume2) {
				auto box = dynamic_cast<const BoxBoundingVolume&>(volume1);
				auto sphere = dynamic_cast<const SphereBoundingVolume&>(volume2);
				return NMath::IsOverlapping(box, sphere);
			},
			[](const IBoundingVolume& volume1, const IBoundingVolume& volume2) {
				auto box = dynamic_cast<const BoxBoundingVolume&>(volume1);
				auto sphere = dynamic_cast<const SphereBoundingVolume&>(volume2);

				return NMath::MergeBoundingVolumes(box, sphere);
			},
				[](const IBoundingVolume& volume1, const IBoundingVolume& volume2) {
				auto box = dynamic_cast<const BoxBoundingVolume&>(volume1);
				auto sphere = dynamic_cast<const SphereBoundingVolume&>(volume2);
				return NMath::Contains(box, sphere);
			}, true);
	}
}
