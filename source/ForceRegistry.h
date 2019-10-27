#pragma once
#include "IForceGenerator.h"
#include <vector>

namespace NPhysics {

	template<class Object>
	class ForceRegistry
	{
		struct ForceRegistration
		{
			ForceRegistration(std::shared_ptr<Object>& p, std::shared_ptr<IForceGenerator<Object>>& fg) :
				object(p),
				forceRegistrator(fg) {};

			std::shared_ptr<Object> object;
			std::shared_ptr<IForceGenerator<Object>> forceRegistrator;
		};

		using Registry = std::vector<ForceRegistration>;

	public:
		void Add(std::shared_ptr<Object>& particle, std::shared_ptr<IForceGenerator<Object>>& forceGenerator);
		void Remove(std::shared_ptr<Object>& particle, std::shared_ptr<IForceGenerator<Object>>& forceGenerator);
		void Clear();
		void UpdateForces(real duration);

	private:
		Registry mRegistrations;
	};

	template<class Object>
	void ForceRegistry<Object>::Add(std::shared_ptr<Object>& object, std::shared_ptr<IForceGenerator<Object>>& forceGenerator)
	{
		assert(object);
		assert(forceGenerator);

		ForceRegistration registry(object, forceGenerator);
		mRegistrations.push_back(registry);
	}

	template<class Object>
	void ForceRegistry<Object>::Remove(std::shared_ptr<Object>& object, std::shared_ptr<IForceGenerator<Object>>& forceGenerator)
	{
		typename Registry::iterator it = std::find_if(mRegistrations.begin(), mRegistrations.end(), [&](ForceRegistration& a) {
			return a.object == object && a.forceRegistrator == forceGenerator; });

		if (it != mRegistrations.end())
		{
			mRegistrations.erase(it);
		}
	}

	template<class Object>
	void ForceRegistry<Object>::Clear()
	{
		mRegistrations.clear();
	}

	template<class Object>
	void ForceRegistry<Object>::UpdateForces(real duration)
	{
		for (auto registry : mRegistrations)
		{
			registry.forceRegistrator->UpdateForce(registry.object, duration);
		}
	}
};

