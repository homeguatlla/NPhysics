#include "pch.h"
#include "RigidBody.h"
#include <glm/gtx/transform.hpp>

namespace NPhysics
{
	void RigidBody::AddForce(const glm::vec3& force)
	{
		mForceAccumulated += force;
	}

	void RigidBody::AddForceAtBodyPoint(const glm::vec3& force, const glm::vec3& point)
	{
		glm::vec3 pointInWorldSpace = GetPointInWorldSpace(point);
		AddForceAtPoint(force, pointInWorldSpace);
	}

	void RigidBody::AddForceAtPoint(const glm::vec3& force, const glm::vec3& point)
	{
		glm::vec3 pt = point;
		pt -= mPosition;

		mForceAccumulated += force;
		mTorqueAccumulated += glm::dot(pt, force);

		//mIsAwake = true;
	}

	glm::vec3 RigidBody::GetPointInWorldSpace(const glm::vec3& point)
	{
		return mTransformationMatrix * glm::vec4(point, 1.0f);
	}

	void RigidBody::ResetForceAndTorqueAccumulated()
	{
		mForceAccumulated = glm::vec3(0.0f);
		mTorqueAccumulated = glm::vec3(0.0f);
	}

	real RigidBody::GetMass() const
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

	void NPhysics::RigidBody::CalculateDerivedData()
	{
		CalculateTransformationMatrix(mTransformationMatrix, mPosition, mOrientation);
		glm::mat3 inverseInertiaTensorWorld;
		CalculateTransformInertiaTensor(inverseInertiaTensorWorld, mOrientation, mInverseInertiaTensor, mTransformationMatrix);
	}

	void RigidBody::CalculateTransformationMatrix(glm::mat4& matrix, const glm::vec3& position, const glm::quat& orientation)
	{
		glm::mat4 rotationMatrix = glm::toMat4(orientation);
		glm::mat4 translationMatrix = glm::translate(glm::mat4(), position);
		matrix = translationMatrix * rotationMatrix;
	}

	void RigidBody::SetInertiaTensor(const glm::mat3& inertiaTensor)
	{
		mInverseInertiaTensor = glm::inverse(inertiaTensor);
		//Falta un check, que no tengo claro que hace
	}

	void RigidBody::CalculateTransformInertiaTensor(glm::mat3& iiWorld, const glm::quat& q, const glm::mat3& iiBody, glm::mat4& rotmat)
	{
		//change of base, R * B * R(Transposed)
		glm::mat3 rotMat = glm::mat3(rotmat);

		iiWorld = rotMat * iiBody * glm::transpose(rotMat);
	}

	void RigidBody::Integrate(real duration)
	{
		//Calculate the linear accelareation frome force inputs
		mLastFrameAcceleration = mAcceleration;
		mLastFrameAcceleration += mForceAccumulated * mInverseMass;

		//Calculate angular acceleration from torque inputs
		glm::vec3 angularAcceleration = mInverseInertiaTensorWorld * mTorqueAccumulated;

		//Adjust velocities
		//Update linear velocity from both acceleration and impulse.
		mVelocity += mLastFrameAcceleration * duration;

		//Update angular velocity from both acceleration and impulse
		mRotation += angularAcceleration * duration;

		//Impose drag
		mVelocity *= glm::pow(mLinearDamping, duration);
		mRotation *= glm::pow(mAngularDamping, duration);

		//Adjust positions
		//Update linear position
		mPosition += mVelocity * duration;

		//Update angular position
		//TODO validate this operation is OK
		mOrientation *= glm::quat(mRotation * duration);

		//Normalise the orientation, and update the matrices with the new position and orientation
		CalculateDerivedData();

		ResetForceAndTorqueAccumulated();

		//TODO missing code not in use now
	}
}
