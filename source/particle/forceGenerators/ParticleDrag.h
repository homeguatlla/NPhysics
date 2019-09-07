#pragma once
#include "../IParticleForceGenerator.h"

namespace NPhysics
{
	class ParticleDrag : public IParticleForceGenerator
	{
	public:
		explicit ParticleDrag(real k1, real k2);
		virtual ~ParticleDrag() = default;

		// Heredado vía IParticleForceGenerator
		void UpdateForce(std::shared_ptr<Particle> particle, real duration) override;

	private:
		//Drag coefficients (GPDE pag 78)
		real mK1;
		real mK2;
	};
};
