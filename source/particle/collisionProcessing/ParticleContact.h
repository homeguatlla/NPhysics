#pragma once
#include <memory>
#include "../../../framework.h"
#include <glm/glm.hpp>

namespace NPhysics
{
	class Particle;
}

namespace NPhysics {
	class ParticleContact
	{
	public:
		//Resolves this contact for both velocity and interpenetration
		void Resolve(real duration);

		//Calculate the separiting velocity at this contact.
		real CalculateSeparitingVelocity() const;

	private:
		//Handles the impulse calculations for this collision
		void ResolveVelocity(real duration);
		//Handles the interpenetration resolution for this contact
		void ResolveInterpenetration(real duration);

		//Handles the separation velocity when an object is resing over another and 
		//gravity makes them move and penetrate. We remove this gravity effect.
		real ModifySepVelocityDueAccelerationOnly(real sepVelocity, real duration);

	private:
		//Holds the particles are involved in the contact. 
		//The second can be null if colliding with the scene not a movible object.
		std::shared_ptr<Particle> mParticles[2];

		//Holds the normal restitution coefficient at the contact.
		//1 means the two objects will separate with the same velocity
		//0 means they rest together.
		real mRestitution;

		//Holds the direction of the contact in world coordinates.
		glm::vec3 mContactNormal;

		//Holds the depth of penetration at the contact
		//this is, the depth of penetration of one object into the other
		real mPenetration;
	};
};

