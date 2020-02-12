#pragma once
#include "../../../framework.h"
#include <memory>

namespace NPhysics
{
	class IBoundingVolume
	{
	public:
		virtual bool IsOverlapping(std::shared_ptr<IBoundingVolume> volume) const = 0;
		virtual real GetGrowth(std::shared_ptr<IBoundingVolume> volume) const = 0;
		virtual std::shared_ptr<IBoundingVolume> MergeBoundingVolumes(std::shared_ptr<IBoundingVolume> volume) const = 0;
		virtual real GetVolume() const = 0;
	};
};

