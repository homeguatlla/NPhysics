#pragma once
#include "IBoundingVolume.h"
#include <glm/glm.hpp>
#include <memory>

namespace NPhysics
{
	class SphereBoundingVolume : public IBoundingVolume
	{
	public:
		SphereBoundingVolume();
		SphereBoundingVolume(const glm::vec3& center, real radius);
		SphereBoundingVolume(const SphereBoundingVolume& sphere1, const SphereBoundingVolume& sphere2);

		~SphereBoundingVolume() = default;

		// Heredado vía IBoundingVolume
		bool IsOverlapping(std::shared_ptr<IBoundingVolume> volume) const override;
		real GetVolume() const override;
		real GetGrowth(std::shared_ptr<IBoundingVolume> volume) const override;
		std::shared_ptr<IBoundingVolume> MergeBoundingVolumes(std::shared_ptr<IBoundingVolume> volume) const override;

		glm::vec3 GetCenter() const { return mCenter; }
		real GetRadius() const { return mRadius; }

	private:
		glm::vec3 mCenter;
		real mRadius;
	};
};

