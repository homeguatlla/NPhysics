// NPhysics.cpp : Define las funciones de la biblioteca estática.
//
#include "pch.h"
#include "NPhysicsEngine.h"

namespace NPhysics
{
	void NPhysicsEngine::AddParticle(std::shared_ptr<Particle>& particle)
	{
		mParticlePhysicsEngine.AddParticle(particle);
	}

	void NPhysicsEngine::RegisterParticleForceGenerator(std::shared_ptr<Particle>& particle, std::shared_ptr<IForceGenerator<Particle>>& forceGenerator)
	{
		mParticlePhysicsEngine.RegisterParticleForceGenerator(particle, forceGenerator);
	}

	void NPhysicsEngine::Update(real duration)
	{
		mParticlePhysicsEngine.Update(duration);
	}
	void NPhysicsEngine::AddRigidBody(std::shared_ptr<RigidBody>& body)
	{
	}
}