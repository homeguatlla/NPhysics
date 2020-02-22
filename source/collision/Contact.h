#pragma once
#include "glm/glm.hpp"

namespace NPhysics
{
	class Contact
	{
	public:
		Contact(const glm::vec3& point, const glm::vec3& normal, real penetration);

	private:
		//Holds the position of the contact in world coordinates
		glm::vec3 mPoint;

		//Holds the direction of the contact in world coordinates
		glm::vec3 mNormal;

		//Holds the depth of penetration at the contact point. If both bodies
		//are specified then the contact point should be midway between the 
		//inter-penetrating points
		real mPenetration;
	};
};
