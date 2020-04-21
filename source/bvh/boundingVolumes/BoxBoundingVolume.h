#pragma once
#include "IBoundingVolume.h"
#include <string>

namespace NPhysics
{
	class BoxBoundingVolume : public IBoundingVolume
	{
	public:
		BoxBoundingVolume();
		BoxBoundingVolume(const glm::vec3& center, const glm::vec3& size, const glm::vec3& rotation = { glm::vec3(0) });
		BoxBoundingVolume(const BoxBoundingVolume& box1, const BoxBoundingVolume& box2);

		~BoxBoundingVolume() = default;

		// Heredado vía IBoundingVolume
		bool IsOverlapping(std::shared_ptr<IBoundingVolume> volume) const override;
		real GetGrowth(std::shared_ptr<IBoundingVolume> volume) const override;
		std::shared_ptr<IBoundingVolume> MergeBoundingVolumes(std::shared_ptr<IBoundingVolume> volume) const override;
		bool Contains(std::shared_ptr<IBoundingVolume> volume) const override;
		real GetVolume() const override;
		const glm::mat3 GetInertiaTensorMatrix(float mass, bool isShell) const override;
		glm::vec3 GetPosition() const override { return mCenter; }

		void SetPosition(const glm::vec3& position) override;
		void SetSize(const glm::vec3& size);
		void SetRotation(const glm::vec3& rotation);

		glm::mat4 GetTransformation() const override;
		glm::vec3 GetMinPoint() const { return mMinPoint; }
		glm::vec3 GetMaxPoint() const { return mMaxPoint;  }
		glm::vec3 GetSize() const { return mSize; }		

		static std::string GetClassName() { return std::string("BoxBoundingVolume"); }
		static std::shared_ptr<IBoundingVolume> Create();

	private:
		void CalculateMinMaxPoints();
		void CalculateTransformation();

	private:
		glm::vec3 mCenter;
		glm::vec3 mSize;
		glm::vec3 mRotation;

		glm::mat4 mTransformation;
		glm::vec3 mMinPoint;
		glm::vec3 mMaxPoint;
	};
};

