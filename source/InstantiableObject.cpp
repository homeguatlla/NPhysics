#include "pch.h"
#include "InstantiableObject.h"

namespace NPhysics
{
	std::map<std::string, InstantiableObject::BoundingVolumeFunction> InstantiableObject::mBoundingVolumeFactory;

	std::shared_ptr<NPhysics::IBoundingVolume> InstantiableObject::CreateBoundingVolume(const std::string& name)
	{
		if (mBoundingVolumeFactory.find(name) != mBoundingVolumeFactory.end())
		{
			return mBoundingVolumeFactory[name]();
		}
		else
		{
			return nullptr;
		}
	}
}