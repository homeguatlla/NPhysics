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
	public:
		RigidBodyPhysicsEngine();

		void AddRigidBody(std::shared_ptr<RigidBody> body, const std::shared_ptr<IBoundingVolume> volume);
		void RegisterForceGenerator(std::shared_ptr<RigidBody> body, const std::shared_ptr<IBoundingVolume> volume, std::shared_ptr<IForceGenerator<RigidBody>> forceGenerator);
		void Update(real duration);

		bool IsOverlapping(std::shared_ptr<const IBoundingVolume> volume1, std::shared_ptr<const IBoundingVolume> volume2) const;
		std::shared_ptr<IBoundingVolume> MergeBoundingVolumes(std::shared_ptr<const IBoundingVolume> volume1, std::shared_ptr<const IBoundingVolume> volume2) const;


	private:
		std::vector<std::shared_ptr<RigidBody>> mBodies;
		ForceRegistry<RigidBody> mRegistry;
		std::shared_ptr<BoundingVolumeHierarchyNode> mBoundingVolumeHierarchyRoot;
	};
};
