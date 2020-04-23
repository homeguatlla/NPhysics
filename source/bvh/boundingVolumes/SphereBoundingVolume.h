#pragma once
#include "IBoundingVolume.h"
#include <glm/glm.hpp>
#include <memory>
#include <string>

namespace NPhysics
{
	class SphereBoundingVolume : public IBoundingVolume
	{
	public:
		SphereBoundingVolume();
		SphereBoundingVolume(const glm::vec3& center, float radius);
		SphereBoundingVolume(const SphereBoundingVolume& sphere1, const SphereBoundingVolume& sphere2);

		~SphereBoundingVolume() = default;

		// Heredado vía IBoundingVolume
		bool IsOverlapping(std::shared_ptr<IBoundingVolume> volume) const override;
		real GetVolume() const override;
		real GetGrowth(std::shared_ptr<IBoundingVolume> volume) const override;
		std::shared_ptr<IBoundingVolume> MergeBoundingVolumes(std::shared_ptr<IBoundingVolume> volume) const override;
		bool Contains(std::shared_ptr<IBoundingVolume> volume) const override;
		const glm::mat3 GetInertiaTensorMatrix(float mass, bool isShell) const override;

		void SetPosition(const glm::vec3& position) override;
		glm::vec3 GetPosition() const override { return mCenter; }

		real GetRadius() const { return mRadius; }
		glm::vec3 GetSize() const override { return glm::vec3(mRadius*2.0f); }
		void SetRadius(real radius) { mRadius = radius; }
		void SetCenter(const glm::vec3& center) { mCenter = center; }
		
		glm::mat4 GetTransformation() const override { return mTransformation; }

		static std::string GetClassName() { return std::string("SphereBoundingVolume"); }
		static std::shared_ptr<IBoundingVolume> Create();

	private:
		void UpdateData();

	private:
		glm::vec3 mCenter;
		real mRadius;
		glm::mat4 mTransformation;
	};
};

