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
		mRestitution(1.0f),
		mFriction(0.0f),
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
	/*
	glm::vec3 Contact::GetRelativeContactPositionForBody(int body) const
	{
		assert(body >= 0 && body < 2);
		return mRelativeContactPosition[body];
	}
	*/
	void Contact::Calculate(float elapsedTime)
	{
		mContactLocalMatrix = NMath::CreateOrthonormalBasis(mNormal);
		mWorldToContactMatrix = glm::transpose(mContactLocalMatrix);

		CalculateLocalVelocity();

		//desired velocity
		mDesiredDeltaVelocity = CalculateDesiredDeltaVelocity();
	}

	real Contact::CalculateDesiredDeltaVelocity()
	{
		return -mLocalVelocity.x * (1.0f + mRestitution);
	}

	void Contact::CalculateLocalVelocity()
	{
		for (int i = 0; i < 2; ++i)
		{
			std::shared_ptr<RigidBody> body = std::static_pointer_cast<RigidBody>(mBodies[i]);
			if (body)
			{
				mRelativeContactPosition[i] = mPoint - body->GetPosition();

				//vel body1 - vel body2
				//local velocity
				real sign = (1.0f - 2.0f * i);
				mLocalVelocity += sign * CalculateLocalVelocity(
					mRelativeContactPosition[i],
					body->GetVelocity(),
					body->GetRotation(),
					mWorldToContactMatrix);
			}
		}
	}

	void Contact::PerformActionForEachBodyEqualToTheContact(
		const std::shared_ptr<Contact>& contactResolved,
		std::function<void(int bodyIndex, int bodyResolvedIndex)> action)
	{
		for (int i = 0; i < 2; ++i)
		{
			std::shared_ptr<RigidBody> body = std::static_pointer_cast<RigidBody>(mBodies[i]);
			if (body)
			{
				for (int j = 0; j < 2; ++j)
				{
					if (body == contactResolved->GetBody(j))
					{
						action(i, j);
					}
				}
			}
		}
	}

	void Contact::UpdatePenetration(const std::shared_ptr<Contact>& contactResolved)
	{
		PerformActionForEachBodyEqualToTheContact(
			contactResolved,
			[this, &contactResolved](int bodyIndex, int bodyResolvedIndex) {
				glm::vec3 deltaPosition = contactResolved->GetLinearChange(bodyResolvedIndex) +
					glm::cross(contactResolved->GetAngularChange(bodyResolvedIndex), mRelativeContactPosition[bodyIndex]);

				real sign = 1.0f - 2.0f * bodyIndex; 
				mPenetration -= sign * glm::dot(deltaPosition, mNormal);
			});
	}

	void Contact::UpdateLocalVelocity(const std::shared_ptr<Contact>& contactResolved)
	{
		PerformActionForEachBodyEqualToTheContact(
			contactResolved,
			[this, &contactResolved](int bodyIndex, int bodyResolvedIndex) {
				glm::vec3 deltaVelocity = contactResolved->GetVelocityChange(bodyResolvedIndex) +
					glm::cross(contactResolved->GetRotationChange(bodyResolvedIndex), mRelativeContactPosition[bodyIndex]);

				real sign = 1.0f - 2.0f * bodyIndex;
				mLocalVelocity += sign * mWorldToContactMatrix * deltaVelocity;
				mDesiredDeltaVelocity = CalculateDesiredDeltaVelocity();
			});
	}

	glm::vec3 Contact::GetLinearChange(int bodyIndex)
	{
		assert(bodyIndex >= 0 && bodyIndex < 2);
		return mLinearChange[bodyIndex];
	}

	glm::vec3 Contact::GetAngularChange(int bodyIndex)
	{
		assert(bodyIndex >= 0 && bodyIndex < 2);
		return mAngularChange[bodyIndex];
	}

	glm::vec3 Contact::GetRotationChange(int bodyIndex)
	{
		assert(bodyIndex >= 0 && bodyIndex < 2);
		return mRotationChange[bodyIndex];
	}

	glm::vec3 Contact::GetVelocityChange(int bodyIndex)
	{
		assert(bodyIndex >= 0 && bodyIndex < 2);
		return mVelocityChange[bodyIndex];
	}

	void Contact::NotifyCollisionEnter()
	{
		for (int i = 0; i < 2; ++i)
		{
			std::shared_ptr<RigidBody> body = std::static_pointer_cast<RigidBody>(mBodies[i]);
			if (body)
			{
				body->OnCollisionEnter(*this);
			}
		}
	}

	void Contact::NotifyCollisionExit()
	{
		for (int i = 0; i < 2; ++i)
		{
			std::shared_ptr<RigidBody> body = std::static_pointer_cast<RigidBody>(mBodies[i]);
			if (body)
			{
				body->OnCollisionExit(*this);
			}
		}
	}

	void Contact::ApplyPositionChanges()
	{
		std::vector<real> inertiaAngularVelocity = { 0.0f, 0.0f };
		std::vector<real> inertiaLinearVelocity = { 0.0f, 0.0f };
		real totalInertia = 0.0f;
		
		//Calculate inertia velocities
		totalInertia = CalculateInertiaVelocity(inertiaLinearVelocity, inertiaAngularVelocity);
		mDeltaVelocity = totalInertia;

		real angular[2] = { 0.0f, 0.0f };
		real linear[2] = { 0.0f, 0.0f };		

		//Apply changes
		for (int i = 0; i < 2; ++i)
		{
			std::shared_ptr<RigidBody> body = std::static_pointer_cast<RigidBody>(mBodies[i]);
			if (body)
			{
				real sign = 1.0f - 2.0f * i;
				angular[i] = sign * mPenetration * (inertiaAngularVelocity[i] / totalInertia);
				linear[i] = sign * mPenetration * (inertiaLinearVelocity[i] / totalInertia);

				real totalMove = angular[i] + linear[i];
				angular[i] = CalculateLimitedAngular(mRelativeContactPosition[i], mNormal, angular[i]);
				linear[i] = totalMove - angular[i];

				if (angular[i] == 0.0f)
				{
					mAngularChange[i] = glm::vec3(0.0f);
				}
				else
				{
					glm::vec3 targetAngularDirection = glm::cross(mRelativeContactPosition[i], mNormal);
					mAngularChange[i] = body->GetInverseInertiaTensorWorldMatrix() * targetAngularDirection;
					mAngularChange[i] *= angular[i] / inertiaAngularVelocity[i];
				}

				mLinearChange[i] = mNormal * linear[i];

				//Apply change of values
				body->SetPosition(body->GetPosition() + mLinearChange[i]);
				//TODO no estoy muy seguro de este cálculo
				glm::quat angularChangeQuat = NMath::FromEulerAnglesToQuaternion(mAngularChange[i]);
				body->SetOrientation(body->GetOrientation() * angularChangeQuat);
			}
		}
	}

	void Contact::ApplyVelocityChange()
	{
		for (int i = 0; i < 2; ++i)
		{
			std::shared_ptr<RigidBody> body = std::static_pointer_cast<RigidBody>(mBodies[i]);
			if (body)
			{
				real sign = 1.0f - 2.0f * i;
				glm::mat3 inverseInertiaTensorMatrix = body->GetInverseInertiaTensorWorldMatrix();
				glm::vec3 impulseLocalContact;
				if (mFriction == 0.0f)
				{
					impulseLocalContact = CalculateFrictionlessImpulse();
				}
				else
				{
					//TODO to implement.
				}

				glm::vec3 impulseWorld = mWorldToContactMatrix * impulseLocalContact;

				//split impulse into linear and rotational components
				glm::vec3 impulseTorque = glm::cross(mRelativeContactPosition[i], impulseWorld);
				mRotationChange[i] = inverseInertiaTensorMatrix * impulseTorque;
				mVelocityChange[i] = impulseWorld * body->GetInverseMass() * sign;

				body->SetRotation(mRotationChange[i]);
				body->SetInitialVelocity(mVelocityChange[i]);
			}
		}
	}

	glm::vec3 Contact::CalculateFrictionlessImpulse()
	{
		return glm::vec3(mDesiredDeltaVelocity / mDeltaVelocity, 0.0f, 0.0f);
	}

	real Contact::CalculateInertiaVelocity(std::vector<real>& inertiaLinearVelocity, std::vector<real>& inertiaAngularVelocity)
	{
		real totalInertia = 0.0f;

		for (int i = 0; i < 2; ++i)
		{
			std::shared_ptr<RigidBody> body = std::static_pointer_cast<RigidBody>(mBodies[i]);
			if (body)
			{
				//Calculate inertia
				inertiaAngularVelocity[i] = CalculateInertiaAngularVelocity( 
					mRelativeContactPosition[i],
					mNormal,
					body->GetInverseInertiaTensorWorldMatrix());
				inertiaLinearVelocity[i] = CalculateInertiaLinearVelocity(body->GetInverseMass());
				totalInertia += inertiaAngularVelocity[i] + inertiaLinearVelocity[i];
			}
		}

		return totalInertia;
	}

	real Contact::CalculateLimitedAngular(const glm::vec3& relativePosition, const glm::vec3& normal, real angular)
	{
		const real angularLimit = 0.2f;
		real scalarProduct = glm::dot(-relativePosition, normal);
		glm::vec3 projection = relativePosition + normal * scalarProduct;
		real maxMagnitude = angularLimit * glm::length(projection);

		return glm::max(-maxMagnitude, glm::min(maxMagnitude, angular));
	}

	real Contact::CalculateInertiaAngularVelocity(
		const glm::vec3& relativeContactPosition,
		const glm::vec3& normal,
		const glm::mat3& inverseTensorMatrix)
	{
		glm::vec3 angularVelocityWorld = glm::cross(relativeContactPosition, normal);
		angularVelocityWorld = inverseTensorMatrix * angularVelocityWorld;
		angularVelocityWorld = glm::cross(angularVelocityWorld, relativeContactPosition);

		//TODO we should use the transform matrix for contact basis
		//work out the change in velocity in contact coordinates
		real angularVelocity = glm::dot(angularVelocityWorld, normal);

		return angularVelocity;
	}
	
	real Contact::CalculateInertiaLinearVelocity(real inversMass)
	{
		return inversMass;
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