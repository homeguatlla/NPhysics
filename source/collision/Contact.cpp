#include "pch.h"
#include "Contact.h"
#include "../utils/Math.h"
#include "../PhysicsObject.h"
#include "../rigidbody/RigidBody.h"

namespace NPhysics
{
	Contact::Contact(const glm::vec3& point, const glm::vec3& normal, real penetration) :
		mPoint(point),
		mNormal(normal),
		mPenetration(penetration),
		mBodies { nullptr, nullptr }
	{
	}

	void Contact::SetBodies(std::shared_ptr<PhysicsObject> body1, std::shared_ptr<PhysicsObject> body2)
	{
		mBodies[0] = body1;
		mBodies[1] = body2;
	}

	std::shared_ptr<PhysicsObject> Contact::GetBody(unsigned int index)
	{
		assert(index >= 0 && index < 2);

		return mBodies[index];
	}

	glm::vec3 Contact::GetRelativeContactPositionForBody(int body) const
	{
		assert(body >= 0 && body < 2);
		return mRelativeContactPosition[body];
	}

	void Contact::Calculate(float elapsedTime)
	{
		mContactLocalMatrix = NMath::CreateOrthonormalBasis(mNormal);
		mWorldToContactMatrix = glm::transpose(mContactLocalMatrix);

		CalculateFrictionLessImpulse();

		//desired velocity
		mDesiredDeltaVelocity = CalculateDesiredDeltaVelocity();
	}

	real Contact::CalculateDesiredDeltaVelocity()
	{
		return -mLocalVelocity.x * (1.0f + mRestitution);
	}

	real Contact::CalculateFrictionLessImpulse()
	{
		real velocity = 0.0f;

		for (int i = 0; i < 2; ++i)
		{
			std::shared_ptr<RigidBody> body = std::static_pointer_cast<RigidBody>(mBodies[i]);
			if (body)
			{
				mRelativeContactPosition[i] = mPoint - body->GetPosition();

				/*velocity += CalculateFrictionLessImpulsePerBody(
					mRelativeContactPosition[i],
					mNormal,
					body->GetInverseInertiaTensorWorldMatrix(),
					body->GetInverseMass());*/

				//vel body1 - vel body2
				//local velocity
				mLocalVelocity += (1.0f - 2.0f * i) * CalculateLocalVelocity(
					mRelativeContactPosition[i],
					body->GetVelocity(),
					body->GetRotation(),
					mWorldToContactMatrix);
			}
		}

		return velocity;
	}

	real Contact::CalculateFrictionLessImpulsePerBody(
		const glm::vec3& relativeContactPosition,
		const glm::vec3& normal,
		const glm::mat3& inverseTensorMatrix,
		real inverseMass)
	{
		glm::vec3 deltaVelocityWorld = glm::cross(relativeContactPosition, normal);
		deltaVelocityWorld = inverseTensorMatrix * deltaVelocityWorld;
		deltaVelocityWorld = glm::cross(deltaVelocityWorld, relativeContactPosition);

		//TODO we should use the transform matrix for contact basis
		//work out the change in velocity in contact coordinates
		real deltaVelocity = glm::dot(deltaVelocityWorld, normal);

		//Add linear component of velocity change
		deltaVelocity += inverseMass;

		return deltaVelocity;
	}
	
	glm::vec3 Contact::CalculateLocalVelocity(
		const glm::vec3& relativeContactPosition,
		const glm::vec3& bodyVelocity,
		const glm::vec3& bodyRotation,
		const glm::mat3& fromWorldToContactMatrix)
	{
		glm::vec3 velocity = glm::cross(bodyRotation, relativeContactPosition); //due to angular component
		velocity += bodyVelocity; //due to linear component

		//transform velocity to a contact basis
		velocity = fromWorldToContactMatrix * velocity;

		return velocity;
	}
}