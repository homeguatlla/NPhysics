#include "pch.h"
#include "RigidBody.h"
#include <glm/gtx/transform.hpp>

namespace NPhysics
{
	RigidBody::RigidBody(const glm::vec3& position, const glm::vec3& rotation, const glm::vec3& initialVelocity) : 
		PhysicsObject(position, initialVelocity),
		mRotation(rotation),
		mAngularDamping(0.995f),
		mTransformationMatrix(1.0f)
	{
		CalculateDerivedData();
	}

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
		mTorqueAccumulated += glm::cross(pt, force);

		//mIsAwake = true;
	}

	glm::vec3 RigidBody::GetPointInWorldSpace(const glm::vec3& point)
	{
		return mTransformationMatrix * glm::vec4(point, 1.0f);
	}

	void RigidBody::DoResetForceAccumulated()
	{
		ResetForceAndTorqueAccumulated();
	}

	void RigidBody::SetRotation(const glm::vec3& initialRotation)
	{
		mRotation = initialRotation;
		CalculateDerivedData();
	}

	void RigidBody::ResetForceAndTorqueAccumulated()
	{
		mForceAccumulated = glm::vec3(0.0f);
		mTorqueAccumulated = glm::vec3(0.0f);
	}

	void RigidBody::SetInertiaTensorMatrix(const glm::mat3& matrix)
	{
		mInverseInertiaTensor = glm::inverse(matrix);
	}

	void RigidBody::DoSetPosition(const glm::vec3& position)
	{
		CalculateDerivedData();
	}

	void NPhysics::RigidBody::CalculateDerivedData()
	{
		mTransformationMatrix = CalculateTransformationMatrix(mPosition, mOrientation);
		mInverseInertiaTensorWorld = CalculateTransformInertiaTensor(mOrientation, mInverseInertiaTensor, mTransformationMatrix);
	}

	glm::mat4 RigidBody::CalculateTransformationMatrix(const glm::vec3& position, const glm::quat& orientation)
	{
		glm::mat4 rotationMatrix = glm::toMat4(orientation);
		glm::mat4 translationMatrix = glm::translate(glm::mat4(), position);
		return translationMatrix * rotationMatrix;
	}

	void RigidBody::SetInertiaTensor(const glm::mat3& inertiaTensor)
	{
		mInverseInertiaTensor = glm::inverse(inertiaTensor);
		//Falta un check, que no tengo claro que hace
	}

	glm::mat3 RigidBody::CalculateTransformInertiaTensor(const glm::quat& q, const glm::mat3& iiBody, glm::mat4& rotmat)
	{
		//change of base, R * B * R(Transposed)
		glm::mat3 rotMat = glm::mat3(rotmat);

		return rotMat * iiBody * glm::transpose(rotMat);
	}

	void RigidBody::Integrate(real duration)
	{
		assert(duration > 0.0f);

		if (!HasFiniteMass()) return;

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
		mVelocity *= glm::pow(mDamping, duration);
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
