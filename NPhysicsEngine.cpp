// NPhysics.cpp : Define las funciones de la biblioteca estática.
//
#include "pch.h"
#include "NPhysicsEngine.h"
#include "source/InstantiableObject.h"
#include "source/bvh/boundingVolumes/SphereBoundingVolume.h"

namespace NPhysics
{
	NPhysicsEngine::NPhysicsEngine()
	{
		InstantiableObject::RegisterBoundingVolume<SphereBoundingVolume>();
	}

	void NPhysicsEngine::AddParticle(std::shared_ptr<Particle> particle)
	{
		mParticlePhysicsEngine.AddParticle(particle);
	}

	void NPhysicsEngine::RegisterParticleForceGenerator(std::shared_ptr<Particle> particle, std::shared_ptr<IForceGenerator<Particle>> forceGenerator)
	{
		mParticlePhysicsEngine.RegisterForceGenerator(particle, forceGenerator);
	}

	void NPhysicsEngine::Update(real duration)
	{
		mParticlePhysicsEngine.Update(duration);
		mRigidBodyPhysicsEngine.Update(duration);
	}

	void NPhysicsEngine::AddRigidBody(std::shared_ptr<RigidBody> body, std::shared_ptr<IBoundingVolume> volume)
	{
		mRigidBodyPhysicsEngine.AddRigidBody(body, volume);
	}

	void NPhysicsEngine::RegisterRigidBodyForceGenerator(std::shared_ptr<RigidBody> body, std::shared_ptr<IBoundingVolume> volume, std::shared_ptr<IForceGenerator<RigidBody>> forceGenerator)
	{
		mRigidBodyPhysicsEngine.RegisterForceGenerator(body, volume, forceGenerator);
	}
}