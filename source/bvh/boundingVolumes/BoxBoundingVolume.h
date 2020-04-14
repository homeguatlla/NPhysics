#pragma once
#include "IBoundingVolume.h"
#include <string>

namespace NPhysics
{
	class BoxBoundingVolume : public IBoundingVolume
	{
	public:
		BoxBoundingVolume();
		BoxBoundingVolume(const glm::vec3& center, const glm::vec3& size);
		BoxBoundingVolume(const glm::mat4& transformation);
		BoxBoundingVolume(const BoxBoundingVolume& box1, const BoxBoundingVolume& box2);

		~BoxBoundingVolume() = default;

		// Heredado v�a IBoundingVolume
		bool IsOverlapping(std::shared_ptr<IBoundingVolume> volume) const override;
		real GetGrowth(std::shared_ptr<IBoundingVolume> volume) const override;
		std::shared_ptr<IBoundingVolume> MergeBoundingVolumes(std::shared_ptr<IBoundingVolume> volume) const override;
		bool Contains(std::shared_ptr<IBoundingVolume> volume) const override;
		real GetVolume() const override;
		const glm::mat3 GetInertiaTensorMatrix(float mass) const override;
		glm::vec3 GetPosition() const override { return mCenter; }

		void SetPosition(const glm::vec3& position) override;
		void SetTransformation(const glm::mat4& transformation) override;
		void SetSize(const glm::vec3& size);

		glm::mat4 GetTransformation() const override;
		glm::vec3 GetMinPoint() const { return mCenter - mSize * 0.5f; }
		glm::vec3 GetMaxPoint() const { return mCenter + mSize * 0.5f; }
		glm::vec3 GetSize() const { return mSize; }
		

		static std::string GetClassName() { return std::string("BoxBoundingVolume"); }
		static std::shared_ptr<IBoundingVolume> Create();

	private:
		void UpdateData();

	private:
		glm::vec3 mCenter;
		glm::vec3 mSize;
		glm::mat4 mTransformation;
	};
};

