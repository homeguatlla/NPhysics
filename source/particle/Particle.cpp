#include "pch.h"
#include "Particle.h"

#include <iostream>

namespace NPhysics
{
	NPhysics::Particle::Particle(const glm::vec3& initialPosition, const glm::vec3& initialVelocity) :
		mPosition(initialPosition),
		mVelocity(initialVelocity),
		mAcceleration(0.0f),
		mDamping(0.995f),
		mInverseMass(0.0f) //by default inmovable particle
	{
	}

	void Particle::SetAcceleration(const glm::vec3& acceleration)
	{
		mAcceleration = acceleration;
	}

	void Particle::SetDamping(real damping)
	{
		mDamping = damping;
	}

	void Particle::SetMass(real mass)
	{
		assert(mass != 0.0f);

		mInverseMass = 1.0f / mass;
	}

	//inmovable particle
	void Particle::SetInfiniteMass()
	{
		mInverseMass = 0.0f;
	}

	void Particle::SetPosition(const glm::vec3& position)
	{
		mPosition = position;
	}

	void Particle::SetInitialVelocity(const glm::vec3& velocity)
	{
		mVelocity = velocity;
	}

	void Particle::Integrate(real duration)
	{
		assert(duration > 0.0f);

		if (!HasFiniteMass()) return;
	
		//Update linear position
		mPosition += mVelocity * duration;

		//Work out the acceleration from the force
		glm::vec3 resultingAcceleration = mAcceleration;
		resultingAcceleration += mForceAccumulated * mInverseMass;

		//Update linear velocity from the acceleration
		mVelocity += resultingAcceleration * duration;

		//Impose drag
		//WARNING this line can be removed or replaced directly by mVelocity *= mDamping;
		//Refer to book: Game Physics Engine Development (GPED) page 52
		mVelocity *= glm::pow(mDamping, duration);

		ResetForceAccumulated();
	}

	void Particle::AddForce(const glm::vec3& force)
	{
		mForceAccumulated += force;
	}

	real Particle::GetMass() const
	{
		if (mInverseMass == 0.0f) 
		{
			return MAX_REAL;
		}
		else
		{
			return 1.0f / mInverseMass; 
		}
	}

	void Particle::ResetForceAccumulated()
	{
		mForceAccumulated = glm::vec3(0.0f);
	}
}