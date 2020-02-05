#include "pch.h"
#include "SphereBoundingVolume.h"
#include "../../utils/Math.h"

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

	bool SphereBoundingVolume::IsOverlapping(const SphereBoundingVolume& volume) const
	{
		return NMath::IsOverlapping(*this, volume);
	}

	real SphereBoundingVolume::GetVolume() const
	{
		return 4.0f / 3.0f * glm::pi<float>() * mRadius * mRadius * mRadius;
	}

	real SphereBoundingVolume::GetGrowth(const SphereBoundingVolume& volume) const
	{
		SphereBoundingVolume newVolume(*this, volume);
		return newVolume.GetVolume() - GetVolume();
	}
}