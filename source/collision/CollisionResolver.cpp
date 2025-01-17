#include "pch.h"
#include "CollisionResolver.h"
#include "../bvh/BoundingVolumeHierarchyNode.h"
#include "../bvh/BoundingVolumeIntersectionResolverMap.h"
#include "../bvh/boundingVolumes/SphereBoundingVolume.h"
#include "../bvh/boundingVolumes/BoxBoundingVolume.h"
#include "../utils/Math.h"
#include "../PhysicsObject.h"
#include "ContactResolver.h"

#include <iostream>

namespace NPhysics
{
	CollisionResolver::CollisionResolver()
	{
		RegisterCollisionBoundingVolumesSupported();
		mBoundingVolumeHierarchyRoot = std::make_shared<BoundingVolumeHierarchyNode>();
	}

	void CollisionResolver::AddCollider(
		const std::shared_ptr<PhysicsObject> object, 
		const std::shared_ptr<IBoundingVolume> collider)
	{
		mBoundingVolumeHierarchyRoot->Insert(object, collider);
	}

	void CollisionResolver::Update(real duration)
	{
		std::vector<std::shared_ptr<PotentialContact>> potentialContacts;
		mBoundingVolumeHierarchyRoot->GetPotentialContacts(potentialContacts, MAX_CONTACTS);
		//std::cout << "Num potential contacts " << potentialContacts.size() << "\n";

		if (!potentialContacts.empty())
		{
			ContactResolver contactResolver(potentialContacts);
			contactResolver.Resolve(duration);
		}
	}

	void CollisionResolver::UpdateCollider(std::shared_ptr<PhysicsObject> body, std::shared_ptr<IBoundingVolume> collider)
	{
		std::shared_ptr<BoundingVolumeHierarchyNode> foundNode;
		//Find node in the bvh with its collider value (if not won't be found)
		mBoundingVolumeHierarchyRoot->Find(body, collider, foundNode);
		//once found or not, update collider with the changed position of the rigidbody
		collider->SetPosition(body->GetPosition());

		if (foundNode)
		{
			//finally if found, update the node with both body and collider updated.
			mBoundingVolumeHierarchyRoot->UpdateBoundingVolume(foundNode, body, collider);
		}
	}

	void CollisionResolver::RegisterCollisionBoundingVolumesSupported()
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
			},
				[](const IBoundingVolume& volume1, const IBoundingVolume& volume2) {
				auto sphere1 = dynamic_cast<const SphereBoundingVolume&>(volume1);
				auto sphere2 = dynamic_cast<const SphereBoundingVolume&>(volume2);
				return NMath::ResolveCollision(sphere1, sphere2);
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
			},
				[](const IBoundingVolume& volume1, const IBoundingVolume& volume2) {
				auto box1 = dynamic_cast<const BoxBoundingVolume&>(volume1);
				auto box2 = dynamic_cast<const BoxBoundingVolume&>(volume2);
				return NMath::ResolveCollision(box1, box2);
			});

		BoundingVolumeIntersectionResolverMap::GetInstance().AddEntry(
			typeid(BoxBoundingVolume).name(),
			typeid(SphereBoundingVolume).name(),
			[](const IBoundingVolume& volume1, const IBoundingVolume& volume2) {
				BoxBoundingVolume box;
				SphereBoundingVolume sphere;

				if (typeid(volume1) == typeid(BoxBoundingVolume))
				{
					box = dynamic_cast<const BoxBoundingVolume&>(volume1);
					sphere = dynamic_cast<const SphereBoundingVolume&>(volume2);
				}
				else
				{
					box = dynamic_cast<const BoxBoundingVolume&>(volume2);
					sphere = dynamic_cast<const SphereBoundingVolume&>(volume1);
				}
				
				return NMath::IsOverlapping(box, sphere);
			},
			[](const IBoundingVolume& volume1, const IBoundingVolume& volume2) {
				BoxBoundingVolume box;
				SphereBoundingVolume sphere;

				if (typeid(volume1) == typeid(BoxBoundingVolume))
				{
					box = dynamic_cast<const BoxBoundingVolume&>(volume1);
					sphere = dynamic_cast<const SphereBoundingVolume&>(volume2);
				}
				else
				{
					box = dynamic_cast<const BoxBoundingVolume&>(volume2);
					sphere = dynamic_cast<const SphereBoundingVolume&>(volume1);
				}

				return NMath::MergeBoundingVolumes(box, sphere);
			},
				[](const IBoundingVolume& volume1, const IBoundingVolume& volume2) {
				BoxBoundingVolume box;
				SphereBoundingVolume sphere;

				if (typeid(volume1) == typeid(BoxBoundingVolume))
				{
					box = dynamic_cast<const BoxBoundingVolume&>(volume1);
					sphere = dynamic_cast<const SphereBoundingVolume&>(volume2);
				}
				else
				{
					box = dynamic_cast<const BoxBoundingVolume&>(volume2);
					sphere = dynamic_cast<const SphereBoundingVolume&>(volume1);
				}
				return NMath::Contains(box, sphere);
			},
				[](const IBoundingVolume& volume1, const IBoundingVolume& volume2) {
				BoxBoundingVolume box;
				SphereBoundingVolume sphere;

				float sign = 1;
				if (typeid(volume1) == typeid(BoxBoundingVolume))
				{
					box = dynamic_cast<const BoxBoundingVolume&>(volume1);
					sphere = dynamic_cast<const SphereBoundingVolume&>(volume2);
					//box collides with sphere
				}
				else
				{
					box = dynamic_cast<const BoxBoundingVolume&>(volume2);
					sphere = dynamic_cast<const SphereBoundingVolume&>(volume1);
					//sphere collides with box
					sign = -1;
				}
				
				return NMath::ResolveCollision(box, sphere, sign);
			}, true);
	}
}