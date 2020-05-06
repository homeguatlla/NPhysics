#pragma once
#include "IBoundingVolume.h"
#include <string>

namespace NPhysics
{
	class BoxBoundingVolume : public IBoundingVolume
	{
	public:
		BoxBoundingVolume();
		BoxBoundingVolume(
			const glm::vec3& parentPosition, 
			const glm::vec3& size, 
			const glm::vec3& localTranslation = {glm::vec3(0.0f)}, 
			const glm::vec3& localScale = { glm::vec3(1.0f) }, 
			const glm::vec3& localRotation = { glm::vec3(0.0f) });
		BoxBoundingVolume(BoxBoundingVolume& box1, BoxBoundingVolume& box2);

		~BoxBoundingVolume() = default;

		// Heredado vía IBoundingVolume
		bool IsOverlapping(std::shared_ptr<IBoundingVolume> volume) const override;
		real GetGrowth(std::shared_ptr<IBoundingVolume> volume) const override;
		std::shared_ptr<IBoundingVolume> MergeBoundingVolumes(std::shared_ptr<IBoundingVolume> volume) const override;
		bool Contains(std::shared_ptr<IBoundingVolume> volume) const override;
		real GetVolume() const override;
		const glm::mat3 GetInertiaTensorMatrix(float mass, bool isShell) const override;
		glm::vec3 GetPosition() override;

		void SetParentTranslation(const glm::vec3& translation) override;
		void SetPosition(const glm::vec3& position) override;
		void SetSize(const glm::vec3& size);
		void SetLocalTransformation(const glm::vec3& translation, const glm::vec3& scale, const glm::vec3& rotation) override;

		glm::mat4 GetTransformationWithoutScale() override;

		glm::vec3 GetMinPoint() override;
		glm::vec3 GetMaxPoint() override;
		glm::vec3 GetSize() const override { return mSize; }
		glm::vec3 GetParentTranslation() const override { return mParentTranslation; }

		glm::vec3 GetLocalTranslation() const override { return mLocalTranslation; }
		glm::vec3 GetLocalRotation() const override { return mLocalRotation; }

		void SetParentTransformation(const glm::vec3& position, const glm::vec3& scale, const glm::vec3& rotation) override;

		std::shared_ptr<IBoundingVolume> Clone() override;

		static std::string GetClassName() { return std::string("BoxBoundingVolume"); }
		static std::shared_ptr<IBoundingVolume> Create();

	private:
		void CalculateMinMaxPoints();
		void CalculateLocalTransformationWithoutScale();
		void CalculateTransformation();
		void UpdateDirty();

	private:
		glm::vec3 mCenter;
		glm::vec3 mSize;

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

