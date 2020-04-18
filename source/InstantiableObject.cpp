#include "pch.h"
#include "InstantiableObject.h"

namespace NPhysics
{
	std::map<std::string, InstantiableObject::BoundingVolumeFunction> InstantiableObject::mBoundingVolumeFactory;

	std::shared_ptr<NPhysics::IBoundingVolume> InstantiableObject::CreateBoundingVolume(const std::string& name, const glm::vec3& position, const glm::vec3& scale, const glm::vec3& rotation)
	{
		if (mBoundingVolumeFactory.find(name) != mBoundingVolumeFactory.end())
		{
			return mBoundingVolumeFactory[name](position, scale, rotation);
		}
		else
		{
			return nullptr;
		}
	}
}