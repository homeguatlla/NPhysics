#pragma once
#include "../../IForceGenerator.h"

namespace NPhysics
{
	class Particle;

	class ParticleBuoyancy : public IForceGenerator<Particle>
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

