#pragma once
#include <string>
#include <map>
#include <functional>
#include <memory>
#include <glm/glm.hpp>

namespace NPhysics
{
	class IBoundingVolume;

	class InstantiableObject
	{
	private:
		using BoundingVolumeFunction = std::function<std::shared_ptr<NPhysics::IBoundingVolume>(const glm::mat4& parentTransformation)>;
	public:

		template<class T>
		static void RegisterBoundingVolume()
		{
			mBoundingVolumeFactory[T::GetClassName()] = std::bind<std::shared_ptr<IBoundingVolume>>(&T::Create);
		}

		static std::shared_ptr<NPhysics::IBoundingVolume> CreateBoundingVolume(const std::string& name, const glm::mat4& parentTransformation);

	private:
		static std::map<std::string, BoundingVolumeFunction> mBoundingVolumeFactory;
	};
};

