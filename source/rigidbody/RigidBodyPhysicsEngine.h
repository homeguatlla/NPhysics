#pragma once
#include "../ForceRegistry.h"
#include <memory>
#include <vector>

namespace NPhysics
{
	class RigidBody;

	class RigidBodyPhysicsEngine
	{
	public:
		void AddRigidBody(std::shared_ptr<RigidBody>& body);
		void RegisterForceGenerator(std::shared_ptr<RigidBody>& body, std::shared_ptr<IForceGenerator<RigidBody>>& forceGenerator);
		void Update(real duration);

	private:
		std::vector<std::shared_ptr<RigidBody>> mBodies;
		ForceRegistry<RigidBody> mRegistry;
	};
};
