#pragma once
#include "../../../framework.h"

namespace NPhysics {
	class SphereBoundingVolume;
}

namespace NPhysics
{
	class IBoundingVolume
	{
		virtual bool IsOverlapping(const SphereBoundingVolume& volume) const = 0;
		//virtual bool Overlaps(const BoxBoundingVolume& volume) const = 0;
		virtual real GetGrowth(const SphereBoundingVolume& volume) const = 0;
		virtual real GetVolume() const = 0;
	};
};

