#pragma once
#include "../../IForceGenerator.h"
#include "../RigidBody.h"

namespace NPhysics
{
	class RigidBodyBuoyancy : public IForceGenerator<RigidBody>
	{
	public:
		explicit RigidBodyBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity, const glm::vec3& centreOfBuoyancy);
		virtual ~RigidBodyBuoyancy() = default;

		// Heredado vía IRigidBodyForceGenerator
		void UpdateForce(std::shared_ptr<RigidBody> body, real duration) override;

	private:
		real mMaxDepth;
		real mVolume;
		real mWaterHeight;
		real mLiquidDensity;
		glm::vec3 mCentreOfBuoyancy;
	};
};

