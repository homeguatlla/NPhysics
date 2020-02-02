#pragma once
#include "../../../framework.h"

namespace NPhysics
{
	class IBoundingVolume
	{
		virtual bool Overlaps(IBoundingVolume& volume) const = 0;
		virtual real GetSize() const = 0;
	};
};

