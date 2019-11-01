#pragma once
#include "../framework.h"
#include <memory>

namespace NPhysics 
{
	template<class PhysicsObject>
	class IForceGenerator
	{
	public:
		virtual void UpdateForce(std::shared_ptr<PhysicsObject> object, real duration) = 0;
	};
};

