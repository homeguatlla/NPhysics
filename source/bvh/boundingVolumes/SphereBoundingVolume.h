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
		SphereBoundingVolume(
			const glm::vec3& parentPosition,
			float radius,
			const glm::vec3& translation = { glm::vec3(0.0f) },
			const glm::vec3& scale = { glm::vec3(1.0f) },
			const glm::vec3& rotation = {glm::vec3(0.0f)});
		SphereBoundingVolume(SphereBoundingVolume& sphere1, SphereBoundingVolume& sphere2);

		~SphereBoundingVolume() = default;

		// Heredado vía IBoundingVolume
		bool IsOverlapping(std::shared_ptr<IBoundingVolume> volume) const override;
		real GetVolume() const override;
		real GetGrowth(std::shared_ptr<IBoundingVolume> volume) const override;
		std::shared_ptr<IBoundingVolume> MergeBoundingVolumes(std::shared_ptr<IBoundingVolume> volume) const override;
		bool Contains(std::shared_ptr<IBoundingVolume> volume) const override;
		const glm::mat3 GetInertiaTensorMatrix(float mass, bool isShell) const override;

		void SetParentTranslation(const glm::vec3& translation) override;
		void SetPosition(const glm::vec3& position) override;
		glm::vec3 GetPosition() override;

		real GetRadius() const { return mRadius; }
		glm::vec3 GetMinPoint() override;
		glm::vec3 GetMaxPoint() override;
		glm::vec3 GetSize() const override { return glm::vec3(mRadius*2.0f); }
		glm::vec3 GetParentTranslation() const override { return mParentTranslation; }

		void SetRadius(real radius);
		void SetCenter(const glm::vec3& center) { mCenter = center; }

		void SetLocalTransformation(const glm::vec3& translation, const glm::vec3& scale, const glm::vec3& rotation) override;

		glm::mat4 GetTransformationWithoutScale() override;

		glm::vec3 GetLocalTranslation() const override { return mLocalTranslation; }
		glm::vec3 GetLocalRotation() const override { return mLocalRotation; }

		void SetParentTransformation(const glm::vec3& position, const glm::vec3& scale, const glm::vec3& rotation) override;

		std::shared_ptr<IBoundingVolume> Clone() override;

		static std::string GetClassName() { return std::string("SphereBoundingVolume"); }
		static std::shared_ptr<IBoundingVolume> Create();

	private:
		void CalculateMinMaxPoints();
		void CalculateTransformation();
		void CalculateLocalTransformationWithoutScale();
		void UpdateDirty();

	private:
		glm::vec3 mCenter;
		real mRadius;
		glm::mat4 mLocalTransformationWithoutScale;
		glm::vec3 mMinPoint;
		glm::vec3 mMaxPoint;

		glm::vec3 mLocalTranslation;
		glm::vec3 mLocalScale;
		glm::vec3 mLocalRotation;

		glm::vec3 mParentTranslation;
		glm::vec3 mParentRotation;
		glm::vec3 mParentScale;
		glm::mat4 mTransformation;

		bool mIsDirty;
	};
};

