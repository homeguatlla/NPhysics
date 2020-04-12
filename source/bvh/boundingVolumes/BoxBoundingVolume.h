#pragma once
#include "IBoundingVolume.h"

namespace NPhysics
{
	class BoxBoundingVolume : public IBoundingVolume
	{
	public:
		BoxBoundingVolume();
		BoxBoundingVolume(const glm::mat4& transformation, const glm::mat4& localTransformation = {glm::mat4(1.0f)});
		BoxBoundingVolume(const BoxBoundingVolume& box1, const BoxBoundingVolume& box2);

		~BoxBoundingVolume() = default;

		// Heredado vía IBoundingVolume
		bool IsOverlapping(std::shared_ptr<IBoundingVolume> volume) const override;
		real GetGrowth(std::shared_ptr<IBoundingVolume> volume) const override;
		std::shared_ptr<IBoundingVolume> MergeBoundingVolumes(std::shared_ptr<IBoundingVolume> volume) const override;
		bool Contains(std::shared_ptr<IBoundingVolume> volume) const override;
		real GetVolume() const override;
		const glm::mat3 GetInertiaTensorMatrix(float mass) const override;
		void SetPosition(const glm::vec3& position) override;
		glm::vec3 GetPosition() const override { return mCenter; }

		glm::mat4 GetTransformation() const { return mTransformation; }
		glm::vec3 GetMinPoint() const { return mCenter - mSize * 0.5f; }
		glm::vec3 GetMaxPoint() const { return mCenter + mSize * 0.5f; }
		glm::vec3 GetSize() const { return mSize; }

	private:
		glm::vec3 mCenter;
		glm::vec3 mSize;
		glm::mat4 mTransformation;
		glm::mat4 mLocalTransformation;
	};
};

