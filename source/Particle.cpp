#include "pch.h"
#include "Particle.h"

namespace NPhysics
{
	NPhysics::Particle::Particle(const glm::vec3& initialPosition, const glm::vec3& initialVelocity) :
		mPosition(initialPosition),
		mVelocity(initialVelocity),
		mAcceleration(0.0f),
		mDamping(0.995f)
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

	void Particle::Integrate(real duration)
	{
		assert(duration > 0.0f);

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
	}
}