#pragma once
#include "../../IForceGenerator.h"
#include "../RigidBody.h"

namespace NPhysics
{
	class RigidBodyDrag : public IForceGenerator<RigidBody>
	{
	public:
		explicit RigidBodyDrag(real k1, real k2);
		virtual ~RigidBodyDrag() = default;

		// Heredado vía IRigidBodyForceGenerator
		void UpdateForce(std::shared_ptr<RigidBody> body, real duration) override;

	private:
		//Drag coefficients (GPDE pag 78)
		real mK1;
		real mK2;
	};
};

