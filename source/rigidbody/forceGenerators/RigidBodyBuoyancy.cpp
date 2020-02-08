#include "pch.h"
#include "RigidBodyBuoyancy.h"

namespace NPhysics
{
	RigidBodyBuoyancy::RigidBodyBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity, const glm::vec3& centreOfBuoyancy) : 
		mMaxDepth(maxDepth),
		mVolume(volume),
		mWaterHeight(waterHeight),
		mLiquidDensity(liquidDensity),
		mCentreOfBuoyancy(centreOfBuoyancy)
	{
	}

	void RigidBodyBuoyancy::UpdateForce(std::shared_ptr<RigidBody> body, real duration)
	{
		// Calculate the submersion depth
		glm::vec3 pointInWorld = body->GetPointInWorldSpace(mCentreOfBuoyancy);
		real depth = pointInWorld.y;

		// Check if we're out of the water
		if (depth >= mWaterHeight + mMaxDepth)
		{
			return;
		}

		glm::vec3 force(0, 0, 0);

		// Check if we're at maximum depth
		if (depth <= mWaterHeight - mMaxDepth)
		{
			force.y = mLiquidDensity * mVolume;
			body->AddForceAtBodyPoint(force, mCentreOfBuoyancy);
			return;
		}

		// Otherwise we are partly submerged
		float depthOverWater = depth + mMaxDepth - mWaterHeight;
		float depthBelowWater = (2.0f * mMaxDepth) - depthOverWater;
		force.y = mLiquidDensity * mVolume * (depthBelowWater / (2.0f * mMaxDepth));

		body->AddForceAtBodyPoint(force, mCentreOfBuoyancy);
	}
}
