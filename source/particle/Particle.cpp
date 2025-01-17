#include "pch.h"
#include "Particle.h"

#include <iostream>

namespace NPhysics
{
	NPhysics::Particle::Particle(const glm::vec3& initialPosition, const glm::vec3& initialVelocity) :
		PhysicsObject(initialPosition, initialVelocity, PhysicsType::kDynamic)
	{
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

		DoResetForceAccumulated();
	}

	void Particle::AddForce(const glm::vec3& force)
	{
		mForceAccumulated += force;
	}

	std::shared_ptr<PhysicsObject> Particle::Clone()
	{
		auto object = std::make_shared<Particle>(mPosition, mVelocity);
		object->SetAcceleration(mAcceleration);
		object->SetDamping(mDamping);
		if (HasFiniteMass())
		{
			object->SetMass(GetMass());
		}
		else
		{
			object->SetInfiniteMass();
		}
		object->SetRotation(GetRotation());

		return object;
	}

	void Particle::DoResetForceAccumulated()
	{
		mForceAccumulated = glm::vec3(0.0f);
	}
}