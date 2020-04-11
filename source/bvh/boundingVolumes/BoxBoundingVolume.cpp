#include "pch.h"
#include "BoxBoundingVolume.h"
#include "../BoundingVolumeIntersectionResolverMap.h"
#include "glm/gtx/transform.hpp"
#include <glm/gtx/matrix_decompose.hpp>
#include <algorithm>

namespace NPhysics
{
	BoxBoundingVolume::BoxBoundingVolume() : mCenter(0.0f), mSize(0.0f)
	{
	}

	BoxBoundingVolume::BoxBoundingVolume(const glm::vec3& center, const glm::vec3& size, const glm::mat4& transformationOffset) :
		mCenter(center), 
		mSize(size)
	{
		glm::vec3 scale;
		glm::quat rotation;
		glm::vec3 translation;
		glm::vec3 skew;
		glm::vec4 perspective;
		glm::decompose(transformationOffset, scale, rotation, translation, skew, perspective);

		mCenter = mCenter + translation;

		//TODO review this in order to have not only offset but also the parent transformation
		//mLocalTransformation = transformationOffset;
		mTransformation = glm::translate(glm::mat4(1.0f), mCenter);
		mTransformation = glm::rotate(mTransformation, rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
		mTransformation = glm::rotate(mTransformation, rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
		mTransformation = glm::rotate(mTransformation, rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));
		mTransformation = glm::scale(mTransformation, scale);
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

		mSize = maxPoint - minPoint;
		mCenter = (minPoint + mSize) * 0.5f;
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
		mCenter = position;
	}
}
