#include "pch.h"
#include "RigidBody.h"
#include <glm/gtx/transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include "../utils/Math.h"

namespace NPhysics
{
	RigidBody::RigidBody(const glm::vec3& position, const glm::vec3& angularVelocity, const glm::vec3& initialVelocity, PhysicsType type) :
		PhysicsObject(position, initialVelocity, type),
		mAngularVelocity(angularVelocity),
		mAngularDamping(0.995f),
		mTransformationMatrix(1.0f),
		mCoefficientOfRestitution(DEFAULT_COR),
		mCollisionEnterHandler(nullptr),
		mCollisionExitHandler(nullptr)
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

		if (glm::abs(mTorqueAccumulated.x) < NMath::FLOAT_TOLERANCE)
		{
			mTorqueAccumulated.x = 0.0f;
		}
		if (glm::abs(mTorqueAccumulated.y) < NMath::FLOAT_TOLERANCE)
		{
			mTorqueAccumulated.y = 0.0f;
		}
		if (glm::abs(mTorqueAccumulated.z) < NMath::FLOAT_TOLERANCE)
		{
			mTorqueAccumulated.z = 0.0f;
		}
	}

	glm::vec3 RigidBody::GetPointInWorldSpace(const glm::vec3& point)
	{
		return mTransformationMatrix * glm::vec4(point, 1.0f);
	}

	void RigidBody::DoResetForceAccumulated()
	{
		ResetForceAndTorqueAccumulated();
	}

	void RigidBody::SetOrientation(const glm::quat& orientation)
	{
		mOrientation = orientation;
		CalculateDerivedData();
	}

	void RigidBody::SetRotation(const glm::vec3& initialRotation)
	{
		mOrientation = NMath::FromEulerAnglesToQuaternion(initialRotation);
		CalculateDerivedData();
	}

	void RigidBody::DoAddRotation(const glm::vec3& rotation)
	{
		mOrientation = mOrientation * NMath::FromEulerAnglesToQuaternion(rotation);
	}

	glm::vec3 RigidBody::GetRotation() const
	{
		return NMath::FromQuatToEulerAngles(mOrientation);
	}

	void RigidBody::SetAngularDamping(float damping)
	{
		mAngularDamping = damping;
	}

	void RigidBody::OnCollisionEnter(const Contact& contact)
	{
		if (mCollisionEnterHandler)
		{
			mCollisionEnterHandler(contact);
		}
	}

	void RigidBody::OnCollisionExit(const Contact& contact)
	{
		if (mCollisionExitHandler)
		{
			mCollisionExitHandler(contact);
		}
	}

	std::shared_ptr<PhysicsObject> RigidBody::Clone()
	{
		auto object = std::make_shared<RigidBody>(mPosition, mAngularVelocity, mVelocity, mType);
		object->SetAcceleration(mAcceleration);
		object->SetDamping(mDamping);
		object->SetAngularDamping(mAngularDamping);
		object->SetInertiaTensorMatrix(glm::inverse(mInverseInertiaTensor));
		if (HasFiniteMass())
		{
			object->SetMass(GetMass());
		}
		else
		{
			object->SetInfiniteMass();
		}
		object->SetOrientation(mOrientation);
		object->SetResitution(mCoefficientOfRestitution);
		object->SetRotation(GetRotation());

		return object;
	}

	void RigidBody::ResetForceAndTorqueAccumulated()
	{
		mForceAccumulated = glm::vec3(0.0f);
		mTorqueAccumulated = glm::vec3(0.0f);
	}

	glm::vec3 RigidBody::DoGetRotation() const
	{
		return GetRotation();
	}

	void RigidBody::SetInertiaTensorMatrix(const glm::mat3& matrix)
	{
		mInverseInertiaTensor = glm::inverse(matrix);
		CalculateDerivedData();
	}

	void RigidBody::DoSetPosition(const glm::vec3& position)
	{
		CalculateDerivedData();
	}

	void RigidBody::DoSetRotation(const glm::vec3& rotation)
	{
		SetRotation(rotation);
	}

	void NPhysics::RigidBody::CalculateDerivedData()
	{
		mOrientation = glm::normalize(mOrientation);
		mTransformationMatrix = CalculateTransformationMatrix(mPosition, mOrientation);
		mInverseInertiaTensorWorld = CalculateTransformInertiaTensor(mOrientation, mInverseInertiaTensor, mTransformationMatrix);
	}

	glm::mat4 RigidBody::CalculateTransformationMatrix(const glm::vec3& position, const glm::quat& orientation)
	{
		glm::vec3 eulerAngles = NMath::FromQuatToEulerAngles(orientation);
		glm::mat4 rotationMatrix = glm::eulerAngleXYZ(eulerAngles.x, eulerAngles.y, eulerAngles.z);
		/*
		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				rotationMatrix[i][j] = glm::abs(rotationMatrix[i][j]) < NMath::FLOAT_TOLERANCE ? 0.0f : rotationMatrix[i][j];
			}
		}*/

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
		mAngularVelocity += angularAcceleration * duration;

		//Impose drag
		mVelocity *= glm::pow(mDamping, duration);
		mAngularVelocity *= glm::pow(mAngularDamping, duration);

		//Adjust positions
		//Update linear position
		mPosition += mVelocity * duration;

		//Update angular position
		mOrientation  *= NMath::FromEulerAnglesToQuaternion(mAngularVelocity * duration);
		//mOrientation *= glm::quat(mAngularVelocity * duration);

		//Normalise the orientation, and update the matrices with the new position and orientation
		CalculateDerivedData();

		ResetForceAndTorqueAccumulated();

		//TODO missing code not in use now
	}
}
