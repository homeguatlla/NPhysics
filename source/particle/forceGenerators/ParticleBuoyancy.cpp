#include "pch.h"
#include "ParticleBuoyancy.h"
#include "../Particle.h"
//#include <iostream>

namespace NPhysics
{
	ParticleBuoyancy::ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity) :
		mMaxDepth(maxDepth),
		mVolume(volume),
		mWaterHeight(waterHeight),
		mLiquidDensity(liquidDensity)
	{
	}

	void ParticleBuoyancy::UpdateForce(std::shared_ptr<Particle> particle, real duration)
	{
		//Calculate the submersion depth
		real depth = particle->GetPosition().y + mMaxDepth;

		//Check if we're out ot the water
		if (depth >= mWaterHeight + mMaxDepth)
		{
			//std::cout << "Over water depth: " << depth << "\n";
			return;
		}

		glm::vec3 force(0.0f);

		//Check if we're at maximum depth
		if (depth <= mWaterHeight - mMaxDepth)
		{
			force.y = mLiquidDensity * mVolume;
			particle->AddForce(force);
			//std::cout << "Below water depth: " << depth << " force: " << force.y << "\n";
			return;
		}

		//Otherwise we are partly submerged
		float depthOverWater = depth + mMaxDepth - mWaterHeight;
		float depthBelowWater = (2.0f * mMaxDepth) - depthOverWater;
		force.y = mLiquidDensity * mVolume * (depthBelowWater / (2.0f * mMaxDepth));
		particle->AddForce(force);

		//std::cout << "Middle water depth: " << depth << " force: " << force.y << " over water: " << depthOverWater << "\n";
	}
}
