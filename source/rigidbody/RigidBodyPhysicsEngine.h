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
	public:
		RigidBodyPhysicsEngine();

		void AddRigidBody(std::shared_ptr<RigidBody> body, const std::shared_ptr<IBoundingVolume> volume);
		void RegisterForceGenerator(std::shared_ptr<RigidBody> body, const std::shared_ptr<IBoundingVolume> volume, std::shared_ptr<IForceGenerator<RigidBody>> forceGenerator);
		void Update(real duration);

	private:
		void CheckCollisions();

	private:
		std::vector<std::shared_ptr<RigidBody>> mBodies;
		ForceRegistry<RigidBody> mRegistry;
		std::shared_ptr<BoundingVolumeHierarchyNode> mBoundingVolumeHierarchyRoot;
	};
};
