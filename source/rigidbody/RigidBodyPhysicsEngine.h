#pragma once
#include "RigidBody.h"
#include <memory>

namespace NPhysics
{
	class RigidBodyPhysicsEngine
	{
	public:
		void AddRigidBody(std::shared_ptr<RigidBody>& body);
	};
};
