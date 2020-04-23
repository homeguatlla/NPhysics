#include "pch.h"
#include "PhysicsObject.h"

namespace NPhysics
{
	unsigned int PhysicsObject::IDCounter = 0;

	PhysicsObject::PhysicsObject(const glm::vec3& initialPosition, const glm::vec3& initialVelocity, PhysicsType type) :
		mPosition(initialPosition),
		mVelocity(initialVelocity),
		mAcceleration(0.0f),
		mDamping(0.995f),
		mInverseMass(0.0f), //by default inmovable particle
		mType(type)
	{
		mId = IDCounter++;
	}

	void PhysicsObject::SetAcceleration(const glm::vec3& acceleration)
	{
		mAcceleration = acceleration;
	}

	void PhysicsObject::SetDamping(real damping)
	{
		mDamping = damping;
	}

	void PhysicsObject::SetMass(real mass)
	{
		assert(mass != 0.0f);

		mInverseMass = 1.0f / mass;
	}

	//inmovable particle
	void PhysicsObject::SetInfiniteMass()
	{
		mInverseMass = 0.0f;
	}

	void PhysicsObject::SetPosition(const glm::vec3& position)
	{
		mPosition = position;
		//in order the rigidbody calculate matrices
		DoSetPosition(position);
	}

	void PhysicsObject::SetRotation(const glm::vec3& rotation)
	{
		DoSetRotation(rotation);
	}

	void PhysicsObject::AddRotation(const glm::vec3& rotation)
	{
		DoAddRotation(rotation);
	}

	void PhysicsObject::SetInitialVelocity(const glm::vec3& velocity)
	{
		mVelocity = velocity;
	}

	void PhysicsObject::AddVelocity(const glm::vec3& velocity)
	{
		mVelocity += velocity;
	}

	real PhysicsObject::GetMass() const
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

	real PhysicsObject::GetInverseMass() const
	{
		return mInverseMass;
	}
}