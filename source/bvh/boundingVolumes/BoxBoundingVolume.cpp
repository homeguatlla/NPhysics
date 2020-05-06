#include "pch.h"
#include "BoxBoundingVolume.h"
#include "../BoundingVolumeIntersectionResolverMap.h"
#include "glm/gtx/transform.hpp"
#include <algorithm>
#include <vector>

namespace NPhysics
{
	BoxBoundingVolume::BoxBoundingVolume() : 
		mCenter(0.0f), 
		mSize(1.0f), 
		mLocalTransformationWithoutScale(1.0f),
		mLocalTranslation(0.0f),
		mLocalScale(1.0f),
		mLocalRotation(0.0f),
		mParentTranslation(0.0f),
		mParentScale(1.0f),
		mParentRotation(0.0f),
		mTransformation(glm::mat4(1.0f)),
		mIsDirty(true)
	{
	}

	BoxBoundingVolume::BoxBoundingVolume(
		const glm::vec3& parentPosition, 
		const glm::vec3& size, 
		const glm::vec3& localTranslation, 
		const glm::vec3& localScale, 
		const glm::vec3& localRotation) :
		mLocalTransformationWithoutScale(1.0f),
		mLocalTranslation(localTranslation),
		mLocalScale(localScale),
		mLocalRotation(localRotation),
		mParentTranslation(parentPosition),
		mParentScale(1.0f),
		mParentRotation(0.0f),
		mTransformation(glm::mat4(1.0f)),
		mIsDirty(true)
	{
		SetLocalTransformation(localTranslation, localScale, localRotation);
		SetSize(size);
		mIsDirty = true;
	}

	BoxBoundingVolume::BoxBoundingVolume(BoxBoundingVolume& box1, BoxBoundingVolume& box2) :
		mLocalTransformationWithoutScale(1.0f),
		mLocalTranslation(0.0f),
		mLocalScale(1.0f),
		mLocalRotation(0.0f),
		mIsDirty(true)
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
		mParentTranslation = minPoint + mSize * 0.5f;

		//recalculate transformations.
		mLocalTransformationWithoutScale = glm::mat4(1.0f);
		mIsDirty = true;
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

	const glm::mat3 BoxBoundingVolume::GetInertiaTensorMatrix(float mass, bool isShell) const
	{
		//TODO replace formula if is a shell
		real dx = mSize.x * mSize.x;
		real dy = mSize.y * mSize.y;
		real dz = mSize.z * mSize.z;
		real k = mass / 12.0f;

		glm::mat3 inertiaTensorMatrix = glm::scale(glm::vec3(k * (dy + dz), k * (dx + dz), k * (dx + dy)));

		return inertiaTensorMatrix;
	}

	glm::vec3 BoxBoundingVolume::GetPosition()
	{
		if (mIsDirty)
		{
			UpdateDirty();
		}
		return mCenter;
	}

	void BoxBoundingVolume::SetParentTranslation(const glm::vec3& translation)
	{
		//position = parentPosition
		mParentTranslation = translation;
		mIsDirty = true;
	}

	void BoxBoundingVolume::SetPosition(const glm::vec3& position)
	{
		if (mIsDirty)
		{
			UpdateDirty();
		}
		auto offset = position - mCenter;
		SetParentTranslation(mParentTranslation + glm::vec3(offset));
		mIsDirty = true;
	}

	void BoxBoundingVolume::SetSize(const glm::vec3& size)
	{
		//size is size parent(with its scale)
		mSize = size * mLocalScale;
		mIsDirty = true;
	}

	void BoxBoundingVolume::SetLocalTransformation(const glm::vec3& translation, const glm::vec3& scale, const glm::vec3& rotation)
	{
		mLocalTranslation = translation;
		mLocalScale = scale;
		mLocalRotation = rotation;
		CalculateLocalTransformationWithoutScale();
		mIsDirty = true;
	}

	glm::mat4 BoxBoundingVolume::GetTransformationWithoutScale()
	{
		if (mIsDirty)
		{
			UpdateDirty();
		}
		return mTransformation;
	}

	glm::vec3 BoxBoundingVolume::GetMinPoint()
	{
		if (mIsDirty)
		{
			UpdateDirty();
		}
		return mMinPoint;
	}

	glm::vec3 BoxBoundingVolume::GetMaxPoint()
	{
		if (mIsDirty)
		{
			UpdateDirty();
		}
		return mMaxPoint;
	}


	void BoxBoundingVolume::SetParentTransformation(const glm::vec3& position, const glm::vec3& scale, const glm::vec3& rotation)
	{
		mParentTranslation = position;
		mParentScale = scale;
		mParentRotation = rotation;
		mIsDirty = true;
	}

	std::shared_ptr<IBoundingVolume> BoxBoundingVolume::Clone()
	{
		return std::make_shared<BoxBoundingVolume>(mCenter, mSize, glm::vec3(0.0f));
	}

	std::shared_ptr<IBoundingVolume> BoxBoundingVolume::Create()
	{
		return std::make_shared<BoxBoundingVolume>();
	}

	void BoxBoundingVolume::CalculateLocalTransformationWithoutScale()
	{
		mLocalTransformationWithoutScale = glm::translate(glm::mat4(1.0f), mLocalTranslation);
		mLocalTransformationWithoutScale = glm::rotate(mLocalTransformationWithoutScale, mLocalRotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
		mLocalTransformationWithoutScale = glm::rotate(mLocalTransformationWithoutScale, mLocalRotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
		mLocalTransformationWithoutScale = glm::rotate(mLocalTransformationWithoutScale, mLocalRotation.z, glm::vec3(0.0f, 0.0f, 1.0f));
	}

	void BoxBoundingVolume::UpdateDirty()
	{
		CalculateTransformation();
		CalculateMinMaxPoints();
		mCenter = mTransformation * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
		mIsDirty = false;
	}

	void BoxBoundingVolume::CalculateTransformation()
	{
		CalculateLocalTransformationWithoutScale();

		auto parentRotation = glm::rotate(glm::mat4(1.0f), mParentRotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
		parentRotation = glm::rotate(parentRotation, mParentRotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
		parentRotation = glm::rotate(parentRotation, mParentRotation.z, glm::vec3(0.0f, 0.0f, 1.0f));

		auto parentTranslation = glm::translate(glm::mat4(1.0f), mParentTranslation);

		mTransformation = parentTranslation * parentRotation * mLocalTransformationWithoutScale;
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
			point = mTransformation * glm::vec4(point, 1.0f);
			mMinPoint.x = glm::min(mMinPoint.x, point.x);
			mMinPoint.y = glm::min(mMinPoint.y, point.y);
			mMinPoint.z = glm::min(mMinPoint.z, point.z);

			mMaxPoint.x = glm::max(mMaxPoint.x, point.x);
			mMaxPoint.y = glm::max(mMaxPoint.y, point.y);
			mMaxPoint.z = glm::max(mMaxPoint.z, point.z);
		}
	}
}
