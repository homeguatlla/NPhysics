#pragma once
#include<glm/glm.hpp>
#include <vector>
#include <memory>

namespace NPhysics
{
	class PotentialContact;
	class Contact;

	class ContactResolver
	{
	public:
		ContactResolver(std::vector<std::shared_ptr<PotentialContact>>& potentialContacts);
		virtual ~ContactResolver() = default;

		void Resolve();
	private:
		real CalculateFrictionLessImpulse(const std::shared_ptr<Contact>& contact);
		real CalculateFrictionLessImpulsePerBody(
			const glm::vec3& relativeContactPosition,
			const glm::vec3& normal,
			const glm::mat3& inverseTensorMatrix,
			real inverseMass);
		glm::vec3 CalculateLocalVelocity(
			const glm::vec3& relativeContactPosition,
			const glm::vec3& bodyVelocity,
			const glm::vec3& bodyRotation,
			const glm::mat3& fromWorldToContactMatrix);

	private:
		std::vector<std::shared_ptr<PotentialContact>> mPotentialContacts;
		std::vector<std::shared_ptr<Contact>> mContacts;
	};
};

