#include "pch.h"
#include "Contact.h"
#include "../utils/Math.h"

namespace NPhysics
{
	Contact::Contact(const glm::vec3& point, const glm::vec3& normal, real penetration) :
		mPoint(point),
		mNormal(normal),
		mPenetration(penetration),
		mBodies { nullptr, nullptr }
	{
		mContactLocalMatrix = NMath::CreateOrthonormalBasis(normal);
		mWorldToContactMatrix = glm::transpose(mContactLocalMatrix);
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
}