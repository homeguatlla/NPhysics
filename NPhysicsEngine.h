#pragma once
#include "source/particle/ParticlePhysicsEngine.h"
#include "source/rigidbody/RigidBodyPhysicsEngine.h"
#include "source/particle/Particle.h"
#include "source/rigidbody/RigidBody.h"

namespace NPhysics
{
	class NPhysicsEngine
	{
	public:
		void AddParticle(std::shared_ptr<Particle>& particle);
		void RegisterParticleForceGenerator(std::shared_ptr<Particle>& particle, std::shared_ptr<IForceGenerator<Particle>>& forceGenerator);
		void Update(real duration);

		void AddRigidBody(std::shared_ptr<RigidBody>& body);

	private:
		ParticlePhysicsEngine mParticlePhysicsEngine;
		RigidBodyPhysicsEngine mRigidBodyPhysicsEngine;
	};
};

