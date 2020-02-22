#pragma once
#include "../ForceRegistry.h"
#include "../collision/CollisionResolver.h"
#include <memory>
#include <vector>


namespace NPhysics
{
	class RigidBody;
	class BoundingVolumeHierarchyNode;
	class IBoundingVolume;

	class RigidBodyPhysicsEngine
	{
		using BodiesVector = std::vector<std::pair<std::shared_ptr<RigidBody>, std::shared_ptr<IBoundingVolume>>>;

	public:
		RigidBodyPhysicsEngine() = default;

		void AddRigidBody(std::shared_ptr<RigidBody> body, const std::shared_ptr<IBoundingVolume> volume);
		void RegisterForceGenerator(std::shared_ptr<RigidBody> body, const std::shared_ptr<IBoundingVolume> volume, std::shared_ptr<IForceGenerator<RigidBody>> forceGenerator);
		void Update(real duration);

	private:
		void AddRigidBody(BodiesVector& bodiesVector, std::shared_ptr<RigidBody> body, const std::shared_ptr<IBoundingVolume> volume);
		void UpdateBoundingVolumeHierarchy();

	private:
		
		BodiesVector mStaticBodies;
		BodiesVector mDynamicBodies;

		ForceRegistry<RigidBody> mRegistry;
		CollisionResolver mCollisionResolver;
	};
};
