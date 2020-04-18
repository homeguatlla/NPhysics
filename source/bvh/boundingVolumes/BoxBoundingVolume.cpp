#include "pch.h"
#include "BoxBoundingVolume.h"
#include "../BoundingVolumeIntersectionResolverMap.h"
#include "glm/gtx/transform.hpp"
#include <glm/gtx/matrix_decompose.hpp>
#include <algorithm>
#include <vector>

namespace NPhysics
{
	BoxBoundingVolume::BoxBoundingVolume() : mCenter(0.0f), mSize(1.0f), mRotation(0.0f), mTransformation(1.0f)
	{
	}

	BoxBoundingVolume::BoxBoundingVolume(const glm::vec3& center, const glm::vec3& size, const glm::vec3& rotation) : 
		mCenter{center},
		mSize{size},
		mRotation(rotation)
	{
		CalculateTransformation();
		CalculateMinMaxPoints();
	}

	BoxBoundingVolume::BoxBoundingVolume(const BoxBoundingVolume& box1, const BoxBoundingVolume& box2)
	{
		glm::vec3 min1 = box1.GetMinPoint();
		glm::vec3 min2 = box2.GetMinPoint();
		glm::vec3 max1 = box1.GetMaxPoint();
		glm::vec3 max2 = box2.GetMaxPoint();

		glm::vec3 minPoint(
			std::min(min1.x, min2.x), 
			std::min(min1.y, min2.y),
			std::min(min1.z, min2.z));
		glm::vec3 maxPoint(
			std::max(max1.x, max2.x),
			std::max(max1.y, max2.y),
			std::max(max1.z, max2.z));

		mSize = (maxPoint - minPoint);
		mCenter = minPoint + mSize * 0.5f;

		//recalculate transformations.
		mTransformation = glm::translate(glm::mat4(1.0f), mCenter);
		CalculateMinMaxPoints();
	}

	bool BoxBoundingVolume::IsOverlapping(std::shared_ptr<IBoundingVolume> volume) const
	{
		auto intersectionFunction = BoundingVolumeIntersectionResolverMap::GetInstance().LookupOverlappingFunction(
			typeid(*this).name(),
			typeid(*volume).name());

		assert(intersectionFunction);

		std::shared_ptr<IBoundingVolume> thisVolume = std::make_shared<BoxBoundingVolume>(*this);
		return intersectionFunction(*thisVolume.get(), *volume.get());
	}

	real BoxBoundingVolume::GetGrowth(std::shared_ptr<IBoundingVolume> volume) const
	{
		auto newVolume = MergeBoundingVolumes(volume);
		return newVolume->GetVolume() - GetVolume();
	}

	std::shared_ptr<IBoundingVolume> BoxBoundingVolume::MergeBoundingVolumes(std::shared_ptr<IBoundingVolume> volume) const
	{
		auto mergeFunction = BoundingVolumeIntersectionResolverMap::GetInstance().LookupMergeFunction(
			typeid(*this).name(),
			typeid(*volume).name());

		assert(mergeFunction);
		return mergeFunction(*this, *volume.get());
	}

	bool BoxBoundingVolume::Contains(std::shared_ptr<IBoundingVolume> volume) const
	{
		auto containsFunction = BoundingVolumeIntersectionResolverMap::GetInstance().LookupContainsFunction(
			typeid(*this).name(),
			typeid(*volume).name());

		assert(containsFunction);
		return containsFunction(*this, *volume.get());
	}

	real BoxBoundingVolume::GetVolume() const
	{
		return mSize.x * mSize.y * mSize.z;
	}

	const glm::mat3 BoxBoundingVolume::GetInertiaTensorMatrix(float mass) const
	{
		real dx = mSize.x * mSize.x;
		real dy = mSize.y * mSize.y;
		real dz = mSize.z * mSize.z;
		real k = mass / 12.0f;

		glm::mat3 inertiaTensorMatrix = glm::scale(glm::vec3(k * (dy + dz), k * (dx + dz), k * (dx + dy)));

		return inertiaTensorMatrix;
	}

	void BoxBoundingVolume::SetPosition(const glm::vec3& position)
	{
		mTransformation = glm::translate(mTransformation, -mCenter);
		mCenter = position;
		mTransformation = glm::translate(mTransformation, mCenter);
		CalculateMinMaxPoints();
	}

	void BoxBoundingVolume::SetSize(const glm::vec3& size)
	{
		mSize = size;
		CalculateMinMaxPoints();
	}

	void BoxBoundingVolume::SetRotation(const glm::vec3& rotation)
	{
		mRotation = rotation;
		CalculateTransformation();
		CalculateMinMaxPoints();
	}

	glm::mat4 BoxBoundingVolume::GetTransformation() const
	{
		return mTransformation;
	}

	std::shared_ptr<IBoundingVolume> BoxBoundingVolume::Create()
	{
		return std::make_shared<BoxBoundingVolume>();
	}

	void BoxBoundingVolume::CalculateTransformation()
	{
		mTransformation = glm::translate(glm::mat4(1.0f), mCenter);
		mTransformation = glm::rotate(mTransformation, mRotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
		mTransformation = glm::rotate(mTransformation, mRotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
		mTransformation = glm::rotate(mTransformation, mRotation.z, glm::vec3(0.0f, 0.0f, 1.0f));
	}

	void BoxBoundingVolume::CalculateMinMaxPoints()
	{
		std::vector<glm::vec3> points;
		glm::vec3 halfSize = mSize * 0.5f;

		points.push_back(glm::vec3(-halfSize.x, -halfSize.y, -halfSize.z));
		points.push_back(glm::vec3(-halfSize.x, -halfSize.y, halfSize.z));
		points.push_back(glm::vec3(-halfSize.x, halfSize.y, -halfSize.z));
		points.push_back(glm::vec3(-halfSize.x, halfSize.y, halfSize.z));
		points.push_back(glm::vec3(halfSize.x, -halfSize.y, -halfSize.z));
		points.push_back(glm::vec3(halfSize.x, -halfSize.y, halfSize.z));
		points.push_back(glm::vec3(halfSize.x, halfSize.y, -halfSize.z));
		points.push_back(glm::vec3(halfSize.x, halfSize.y, halfSize.z));

		mMinPoint = glm::vec3(std::numeric_limits<float>::max());
		mMaxPoint = -glm::vec3(std::numeric_limits<float>::max());

		for (auto&& point : points)
		{
			point = glm::vec3(mTransformation * glm::vec4(point, 1));
			
			mMinPoint.x = glm::min(mMinPoint.x, point.x);
			mMinPoint.y = glm::min(mMinPoint.y, point.y);
			mMinPoint.z = glm::min(mMinPoint.z, point.z);

			mMaxPoint.x = glm::max(mMaxPoint.x, point.x);
			mMaxPoint.y = glm::max(mMaxPoint.y, point.y);
			mMaxPoint.z = glm::max(mMaxPoint.z, point.z);
		}
	}
}
