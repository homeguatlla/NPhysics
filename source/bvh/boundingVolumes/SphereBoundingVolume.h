#pragma once
#include "IBoundingVolume.h"
#include "../../framework.h"
#include <glm/glm.hpp>

namespace NPhysics
{
	class SphereBoundingVolume : public IBoundingVolume
	{
		glm::vec3 mCenter;
		real mRadius;

		// Heredado vía IBoundingVolume
		bool Overlaps(IBoundingVolume& volume) const override;
		real GetSize() const override;
	};
};

