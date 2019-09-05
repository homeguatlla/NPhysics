#include "pch.h"
#include "ParticleForceRegistry.h"

#include <algorithm>

namespace NPhysics {
	void ParticleForceRegistry::Add(std::shared_ptr<Particle>& particle, std::shared_ptr<IParticleForceGenerator>& forceGenerator)
	{
		ParticleForceRegistration registry(particle, forceGenerator);
		mRegistrations.push_back(registry);
	}

	void ParticleForceRegistry::Remove(std::shared_ptr<Particle>& particle, std::shared_ptr<IParticleForceGenerator>& forceGenerator)
	{
		Registry::iterator it = std::find_if(mRegistrations.begin(), mRegistrations.end(), [&](ParticleForceRegistration& a) {
			return a.particle == particle && a.forceRegistrator == forceGenerator; });

		if (it != mRegistrations.end())
		{
			mRegistrations.erase(it);
		}	
	}

	void ParticleForceRegistry::Clear()
	{
		mRegistrations.clear();
	}

	void ParticleForceRegistry::UpdateForces(real duration)
	{
		for (auto registry : mRegistrations)
		{
			registry.forceRegistrator->UpdateForce(registry.particle, duration);
		}
	}
}