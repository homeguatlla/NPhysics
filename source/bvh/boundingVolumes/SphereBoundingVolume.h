#pragma once
#include "IBoundingVolume.h"
#include "../../framework.h"
#include <glm/glm.hpp>

namespace NPhysics
{
	class SphereBoundingVolume : public IBoundingVolume
	{
	public:
		SphereBoundingVolume() = default;
		SphereBoundingVolume(const glm::vec3& center, real radius);
		SphereBoundingVolume(const SphereBoundingVolume& sphere1, const SphereBoundingVolume& sphere2);

		~SphereBoundingVolume() = default;

		// Heredado vía IBoundingVolume
		bool IsOverlapping(const SphereBoundingVolume& volume) const override;
		real GetVolume() const override;
		real GetGrowth(const SphereBoundingVolume& volume) const override;

		glm::vec3 GetCenter() const { return mCenter; }
		real GetRadius() const { return mRadius; }

	private:
		glm::vec3 mCenter;
		real mRadius;
	};
};

