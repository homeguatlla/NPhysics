#pragma once
#include "Particle.h"
#include <memory>

namespace NPhysics {

	class IParticleForceGenerator
	{
	public:
		virtual void UpdateForce(std::shared_ptr<Particle> particle, real duration) = 0;
	};
};

