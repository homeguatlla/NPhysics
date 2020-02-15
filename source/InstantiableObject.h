#pragma once
#include <string>
#include <map>
#include <functional>
#include <memory>

namespace NPhysics
{
	class IBoundingVolume;

	class InstantiableObject
	{
	private:
		using BoundingVolumeFunction = std::function<std::shared_ptr<NPhysics::IBoundingVolume>()>;
	public:

		template<class T>
		static void RegisterBoundingVolume()
		{
			mBoundingVolumeFactory[T::GetClassName()] = std::bind<std::shared_ptr<IBoundingVolume>>(&T::Create);
		}

		static std::shared_ptr<NPhysics::IBoundingVolume> CreateBoundingVolume(const std::string& name);

	private:
		static std::map<std::string, BoundingVolumeFunction> mBoundingVolumeFactory;
	};
};

