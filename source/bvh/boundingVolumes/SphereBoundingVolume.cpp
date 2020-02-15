#include "pch.h"
#include "SphereBoundingVolume.h"
#include "../BoundingVolumeIntersectionResolverMap.h"
#include <glm\gtx\transform.hpp>

namespace NPhysics
{
	SphereBoundingVolume::SphereBoundingVolume() : 
		mCenter { glm::vec3(0.0f) },
		mRadius { 0.0f }
	{
	}

	SphereBoundingVolume::SphereBoundingVolume(const glm::vec3& center, real radius) :
		mCenter{center},
		mRadius{ radius }
	{
	}

	SphereBoundingVolume::SphereBoundingVolume(const SphereBoundingVolume& sphere1, const SphereBoundingVolume& sphere2)
	{
		//https://stackoverflow.com/questions/33532860/merge-two-spheres-to-get-a-new-one

		auto distanceBetweenSpheres = glm::distance(sphere1.GetCenter(), sphere2.GetCenter());
		
		//If the spheres enclose each other:
		bool sphere1EnclosesSphere2 = distanceBetweenSpheres + sphere2.GetRadius() < sphere1.GetRadius();
		bool sphere2EnclosesSphere1 = distanceBetweenSpheres + sphere1.GetRadius() < sphere2.GetRadius();

		if (sphere1EnclosesSphere2 || sphere2EnclosesSphere1)
		{
			if (sphere1.GetRadius() > sphere2.GetRadius())
			{
				mRadius = sphere1.GetRadius();
				mCenter = sphere1.GetCenter();
			}
			else
			{
				mRadius = sphere2.GetRadius();
				mCenter = sphere2.GetCenter();
			}
		}
		else
		{
			mRadius = (sphere1.GetRadius() + sphere2.GetRadius() + distanceBetweenSpheres) * 0.5f;
			//linear interpolation
			float lambda = (mRadius - sphere1.GetRadius()) / distanceBetweenSpheres;
			mCenter = sphere1.GetCenter() + (sphere2.GetCenter() - sphere1.GetCenter()) * lambda;
		}
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
	const glm::mat3 SphereBoundingVolume::GetInertiaTensorMatrix(float mass) const
	{
		real value = (2.0f/5.0f) * (mass * mRadius * mRadius);
		
		glm::mat3 inertiaTensorMatrix = glm::scale(glm::vec3(value, value, value));

		return inertiaTensorMatrix;
	}
	std::shared_ptr<IBoundingVolume> SphereBoundingVolume::Create()
	{
		return std::make_shared<SphereBoundingVolume>();
	}
}