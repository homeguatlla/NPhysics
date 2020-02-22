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
}