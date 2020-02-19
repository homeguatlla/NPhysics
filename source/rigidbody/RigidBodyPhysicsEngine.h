#pragma once
#include "../ForceRegistry.h"
#include <memory>
#include <vector>

namespace NPhysics
{
	class RigidBody;
	class BoundingVolumeHierarchyNode;
	class IBoundingVolume;

	class RigidBodyPhysicsEngine
	{
		const unsigned int MAX_CONTACTS { 1000 };
		using BodiesVector = std::vector<std::pair<std::shared_ptr<RigidBody>, std::shared_ptr<IBoundingVolume>>>;

	public:
		RigidBodyPhysicsEngine();

		void AddRigidBody(std::shared_ptr<RigidBody> body, const std::shared_ptr<IBoundingVolume> volume);
		void RegisterForceGenerator(std::shared_ptr<RigidBody> body, const std::shared_ptr<IBoundingVolume> volume, std::shared_ptr<IForceGenerator<RigidBody>> forceGenerator);
		void Update(real duration);

	private:
		void AddRigidBody(BodiesVector& bodiesVector, std::shared_ptr<RigidBody> body, const std::shared_ptr<IBoundingVolume> volume);
		void UpdateBoundingVolumeHierarchy();
		void CheckCollisions();

	private:
		
		BodiesVector mStaticBodies;
		BodiesVector mDynamicBodies;

		ForceRegistry<RigidBody> mRegistry;
		std::shared_ptr<BoundingVolumeHierarchyNode> mBoundingVolumeHierarchyRoot;
	};
};
