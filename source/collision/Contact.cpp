#include "pch.h"
#include "Contact.h"

namespace NPhysics
{
	Contact::Contact(const glm::vec3& point, const glm::vec3& normal, real penetration) :
		mPoint(point),
		mNormal(normal),
		mPenetration(penetration)
	{
	}
	void Contact::SetBodies(std::shared_ptr<PhysicsObject> body1, std::shared_ptr<PhysicsObject> body2)
	{
		mBodies[0] = body1;
		mBodies[1] = body2;
	}
}