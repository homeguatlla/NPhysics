#include "pch.h"
#include "SphereBoundingVolume.h"
#include "../BoundingVolumeIntersectionResolverMap.h"
#include <glm\gtx\transform.hpp>

namespace NPhysics
{
	SphereBoundingVolume::SphereBoundingVolume() : 
		mCenter { glm::vec3(0.0f) },
		mRadius { 1.0f },
		mLocalTransformationWithoutScale { 1.0f },
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

	SphereBoundingVolume::SphereBoundingVolume(
		const glm::vec3& parentPosition, 
		float radius, 
		const glm::vec3& localTranslation, 
		const glm::vec3& localScale, 
		const glm::vec3& localRotation) :
		mLocalTransformationWithoutScale{ 1.0f },
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
		SetRadius(radius);
		mIsDirty = true;
	}

	SphereBoundingVolume::SphereBoundingVolume(SphereBoundingVolume& sphere1, SphereBoundingVolume& sphere2) :
		mLocalTransformationWithoutScale{ 1.0f },
		mLocalTranslation(0.0f),
		mLocalScale(1.0f),
		mLocalRotation(0.0f),
		mParentScale(1.0f),
		mParentRotation(0.0f),
		mTransformation(glm::mat4(1.0f)),
		mIsDirty(true)
	{
		//https://stackoverflow.com/questions/33532860/merge-two-spheres-to-get-a-new-one

		auto distanceBetweenSpheres = glm::distance(sphere1.GetPosition(), sphere2.GetPosition());
		
		//If the spheres enclose each other:
		bool sphere1EnclosesSphere2 = distanceBetweenSpheres + sphere2.GetRadius() < sphere1.GetRadius();
		bool sphere2EnclosesSphere1 = distanceBetweenSpheres + sphere1.GetRadius() < sphere2.GetRadius();

		if (sphere1EnclosesSphere2 || sphere2EnclosesSphere1)
		{
			if (sphere1.GetRadius() > sphere2.GetRadius())
			{
				mRadius = sphere1.GetRadius();
				mParentTranslation = sphere1.GetPosition();
			}
			else
			{
				mRadius = sphere2.GetRadius();
				mParentTranslation = sphere2.GetPosition();
			}
		}
		else
		{
			mRadius = (sphere1.GetRadius() + sphere2.GetRadius() + distanceBetweenSpheres) * 0.5f;
			//linear interpolation
			float lambda = (mRadius - sphere1.GetRadius()) / distanceBetweenSpheres;
			mParentTranslation = sphere1.GetPosition() + (sphere2.GetPosition() - sphere1.GetPosition()) * lambda;
		}

		//recalculate transformations.
		mLocalTransformationWithoutScale = glm::mat4(1.0f);
		mIsDirty = true;
	}

	bool SphereBoundingVolume::IsOverlapping(std::shared_ptr<IBoundingVolume> volume) const
	{
		auto intersectionFunction = BoundingVolumeIntersectionResolverMap::GetInstance().LookupOverlappingFunction(
			typeid(*this).name(), 
			typeid(*volume).name());
	
		assert(intersectionFunction);

		std::shared_ptr<IBoundingVolume> thisVolume = std::make_shared<SphereBoundingVolume>(*this);
		return intersectionFunction(*thisVolume.get(), *volume.get());
	}

	real SphereBoundingVolume::GetVolume() const
	{
		return 4.0f / 3.0f * glm::pi<float>() * mRadius * mRadius * mRadius;
	}

	real SphereBoundingVolume::GetGrowth(std::shared_ptr<IBoundingVolume> volume) const
	{
		auto newVolume = MergeBoundingVolumes(volume);
		return newVolume->GetVolume() - GetVolume();
	}

	std::shared_ptr<IBoundingVolume> SphereBoundingVolume::MergeBoundingVolumes(std::shared_ptr<IBoundingVolume> volume) const
	{
		auto mergeFunction = BoundingVolumeIntersectionResolverMap::GetInstance().LookupMergeFunction(
			typeid(*this).name(),
			typeid(*volume).name());

		assert(mergeFunction);
		return mergeFunction(*this, *volume.get());
	}

	const glm::mat3 SphereBoundingVolume::GetInertiaTensorMatrix(float mass, bool isShell) const
	{
		real k = isShell ? 2.0f / 3.0f : 2.0f / 5.0f;
		real value = k * (mass * mRadius * mRadius);
		
		glm::mat3 inertiaTensorMatrix = glm::scale(glm::vec3(value, value, value));

		return inertiaTensorMatrix;
	}

	void SphereBoundingVolume::SetParentTranslation(const glm::vec3& translation)
	{
		mParentTranslation = translation;
		mIsDirty = true;
	}

	void SphereBoundingVolume::SetPosition(const glm::vec3& position)
	{
		if (mIsDirty)
		{
			UpdateDirty();
		}
		auto offset = position - mCenter;
		SetParentTranslation(mParentTranslation + glm::vec3(offset));
		mIsDirty = true;
	}

	glm::vec3 SphereBoundingVolume::GetPosition()
	{
		if (mIsDirty)
		{
			UpdateDirty();
		}
		return mCenter;
	}

	glm::vec3 SphereBoundingVolume::GetMinPoint()
	{
		if (mIsDirty)
		{
			UpdateDirty();
		}
		return mMinPoint;
	}

	glm::vec3 SphereBoundingVolume::GetMaxPoint()
	{
		if (mIsDirty)
		{
			UpdateDirty();
		}
		return mMaxPoint;
	}

	void SphereBoundingVolume::SetRadius(real radius)
	{
		mRadius = radius * mLocalScale.x;
		mIsDirty = true;
	}

	void SphereBoundingVolume::SetLocalTransformation(const glm::vec3& translation, const glm::vec3& scale, const glm::vec3& rotation)
	{
		mLocalTranslation = translation;
		mLocalScale = scale;
		mLocalRotation = rotation;
		CalculateLocalTransformationWithoutScale();
		mIsDirty = true;
	}

	glm::mat4 SphereBoundingVolume::GetTransformationWithoutScale()
	{
		if (mIsDirty)
		{
			UpdateDirty();
		}
		return mTransformation;
	}

	void SphereBoundingVolume::SetParentTransformation(const glm::vec3& position, const glm::vec3& scale, const glm::vec3& rotation)
	{
		mParentTranslation = position;
		mParentScale = scale;
		mParentRotation = rotation;
		mIsDirty = true;
	}

	std::shared_ptr<IBoundingVolume> SphereBoundingVolume::Clone()
	{
		return std::make_shared<SphereBoundingVolume>(mCenter, mRadius);
	}

	std::shared_ptr<IBoundingVolume> SphereBoundingVolume::Create()
	{
		return std::make_shared<SphereBoundingVolume>();
	}

	bool SphereBoundingVolume::Contains(std::shared_ptr<IBoundingVolume> volume) const
	{
		auto containsFunction = BoundingVolumeIntersectionResolverMap::GetInstance().LookupContainsFunction(
			typeid(*this).name(),
			typeid(*volume).name());

		assert(containsFunction);
		return containsFunction(*this, *volume.get());
	}

	void SphereBoundingVolume::CalculateLocalTransformationWithoutScale()
	{
		mLocalTransformationWithoutScale = glm::translate(glm::mat4(1.0f), mLocalTranslation);
		mLocalTransformationWithoutScale = glm::rotate(mLocalTransformationWithoutScale, mLocalRotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
		mLocalTransformationWithoutScale = glm::rotate(mLocalTransformationWithoutScale, mLocalRotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
		mLocalTransformationWithoutScale = glm::rotate(mLocalTransformationWithoutScale, mLocalRotation.z, glm::vec3(0.0f, 0.0f, 1.0f));
	}

	void SphereBoundingVolume::UpdateDirty()
	{
		CalculateTransformation();
		CalculateMinMaxPoints();
		mCenter = mTransformation * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
		mIsDirty = false;
	}

	void SphereBoundingVolume::CalculateTransformation()
	{
		CalculateLocalTransformationWithoutScale();

		auto parentRotation = glm::rotate(glm::mat4(1.0f), mParentRotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
		parentRotation = glm::rotate(parentRotation, mParentRotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
		parentRotation = glm::rotate(parentRotation, mParentRotation.z, glm::vec3(0.0f, 0.0f, 1.0f));

		auto parentTranslation = glm::translate(glm::mat4(1.0f), mParentTranslation);

		mTransformation = parentTranslation * parentRotation * mLocalTransformationWithoutScale;
	}

	void SphereBoundingVolume::CalculateMinMaxPoints()
	{
		mMinPoint = mTransformation * glm::vec4(-mRadius, -mRadius, -mRadius, 1.0f);
		mMaxPoint = mTransformation * glm::vec4(mRadius, mRadius, mRadius, 1.0f);
	}
}