#pragma once
#include <glm/glm.hpp>
#include <memory>

namespace NPhysics
{
	class PhysicsObject;

	class Contact
	{
	public:
		Contact(const glm::vec3& point, const glm::vec3& normal, real penetration);
		glm::vec3 GetPoint() const { return mPoint; }
		glm::vec3 GetNormal() const { return mNormal; }
		real GetPenetration() const { return mPenetration; }

		void SetBodies(std::shared_ptr<PhysicsObject> body1, std::shared_ptr<PhysicsObject> body2);
		std::shared_ptr<PhysicsObject> GetBody(unsigned int index);

	private:
		//Holds the position of the contact in world coordinates
		glm::vec3 mPoint;

		//Holds the direction of the contact in world coordinates
		glm::vec3 mNormal;

		//Holds the depth of penetration at the contact point. If both bodies
		//are specified then the contact point should be midway between the 
		//inter-penetrating points
		real mPenetration;

		std::shared_ptr<PhysicsObject> mBodies[2];
	};
};
