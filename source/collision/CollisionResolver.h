#pragma once
#include <memory>

namespace NPhysics
{
	class IBoundingVolume;
	class PhysicsObject;
	class BoundingVolumeHierarchyNode;

	class CollisionResolver
	{
		const unsigned int MAX_CONTACTS{ 1000 };

	public:
		CollisionResolver();
		virtual ~CollisionResolver() = default;

		void AddCollider(const std::shared_ptr<PhysicsObject> object, const std::shared_ptr<IBoundingVolume> collider);
		void Update(real duration);
		void UpdateCollider(std::shared_ptr<PhysicsObject> body, std::shared_ptr<IBoundingVolume> collider);

	private:
		void RegisterCollisionBoundingVolumesSupported();
		void CheckCollisions();

	private:
		std::shared_ptr<BoundingVolumeHierarchyNode> mBoundingVolumeHierarchyRoot;
	};
};

