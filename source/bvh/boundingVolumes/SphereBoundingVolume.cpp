#include "pch.h"
#include "SphereBoundingVolume.h"

namespace NPhysics
{
	bool SphereBoundingVolume::Overlaps(IBoundingVolume& volume) const
	{
		return false;
	}
	real SphereBoundingVolume::GetSize() const
	{
		return 2.0f * glm::pi<float>() * mRadius;
	}
}