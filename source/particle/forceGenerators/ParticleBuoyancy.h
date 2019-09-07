#pragma once
#include "../IParticleForceGenerator.h"

namespace NPhysics
{
	class ParticleBuoyancy : public IParticleForceGenerator
	{
	public:
		explicit ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity);
		virtual ~ParticleBuoyancy() = default;

		// Heredado vía IParticleForceGenerator
		void UpdateForce(std::shared_ptr<Particle> particle, real duration) override;

	private:
		real mMaxDepth;
		real mVolume;
		real mWaterHeight;
		real mLiquidDensity;
	};
};

