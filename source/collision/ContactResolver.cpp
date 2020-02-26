#include "pch.h"
#include "ContactResolver.h"
#include "PotentialContact.h"
#include "Contact.h"
#include "../PhysicsObject.h"
#include "../rigidbody/RigidBody.h"

#include <iostream>

namespace NPhysics
{
	ContactResolver::ContactResolver(std::vector<std::shared_ptr<PotentialContact>>& potentialContacts) :
		mPotentialContacts(potentialContacts)
	{
	}

	void ContactResolver::Resolve()
	{
		for (const auto& potentialContact : mPotentialContacts)
		{
			potentialContact->Resolve(mContacts);
		}

		for (const auto& contact : mContacts)
		{
			real velocity = CalculateFrictionLessImpulse(contact);
			std::cout << "Velocity " << velocity << "\n";
		}
	}

	real ContactResolver::CalculateFrictionLessImpulse(const std::shared_ptr<Contact>& contact)
	{
		real velocity = 0.0f;
		glm::vec3 localVelocity(0.0f);

		for (int i = 0; i < 2; ++i)
		{
			std::shared_ptr<RigidBody> body = std::static_pointer_cast<RigidBody>(contact->GetBody(i));
			if (body)
			{
				glm::vec3 relativeContactPosition = contact->GetPoint() - body->GetPosition();
				velocity += CalculateFrictionLessImpulsePerBody(
					relativeContactPosition,
					contact->GetNormal(),
					body->GetInverseInertiaTensorWorldMatrix(),
					body->GetInverseMass());

				//vel body1 - vel body2
				localVelocity += (1.0f - 2.0f * i) * CalculateLocalVelocity(
					relativeContactPosition,
					body->GetVelocity(),
					body->GetRotation(),
					contact->GetWorldToContactMatrix());
			}
		}

		return velocity;
	}

	real ContactResolver::CalculateFrictionLessImpulsePerBody(
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

	glm::vec3 ContactResolver::CalculateLocalVelocity(
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