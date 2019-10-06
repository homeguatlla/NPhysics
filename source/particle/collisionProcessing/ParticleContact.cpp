#include "pch.h"
#include "ParticleContact.h"
#include "../Particle.h"

namespace NPhysics
{
	void ParticleContact::Resolve(real duration)
	{
		ResolveVelocity(duration);
		ResolveInterpenetration(duration);
	}

	real ParticleContact::CalculateSeparitingVelocity() const
	{
		glm::vec3 relativeVelocity = mParticles[0]->GetVelocity();
		if (mParticles[1])
		{
			relativeVelocity -= mParticles[1]->GetVelocity();
		}

		return glm::dot(relativeVelocity, mContactNormal);
	}

	void ParticleContact::ResolveVelocity(real duration)
	{
		//Find the velocity in the direction of the contact
		real separatingVelocity = CalculateSeparitingVelocity();

		//Check whether it need to be resolved
		if (separatingVelocity > 0)
		{
			//The contact is either separting or stationary
			//there's no impulse required
			//the objects are going in the same direction then no collision
			//they are separating each other
			return;
		}

		//Calculate the new separating velocity
		real newSepVelocity = -separatingVelocity * mRestitution;

		//Check the velocity build-up due to acceleration only
		newSepVelocity = ModifySepVelocityDueAccelerationOnly(newSepVelocity, duration);

		real deltaVelocity = newSepVelocity - separatingVelocity;

		//We apply the change in velocity to each object in proportion to its inverse mass.
		//those with lower inverse mass (higher actual mass) get less change in velocity
		real totalInverseMass = mParticles[0]->GetInverseMass();
		if (mParticles[1])
		{
			totalInverseMass += mParticles[1]->GetInverseMass();
		}

		//If all particles have infitine mass, then impulses have no effect
		if (totalInverseMass <= 0.0f)
		{
			return;
		}

		//Calculate the impulse to apply
		real impulse = deltaVelocity / totalInverseMass;

		//Find the amount of impulse per unit of inverse mass
		glm::vec3 impulsePerIMass = mContactNormal * impulse;

		//Apply impulses: they are applied in the direction of the contact, and are proportional to the inverse mass
		mParticles[0]->SetInitialVelocity(mParticles[0]->GetVelocity() + impulsePerIMass * mParticles[0]->GetInverseMass());
		if (mParticles[1])
		{
			//Particle 1 goes in the opposite direction
			mParticles[1]->SetInitialVelocity(mParticles[1]->GetVelocity() - impulsePerIMass * mParticles[1]->GetInverseMass());
		}
	}

	//this code is to determine when a resting particle is moving due to the gravity
	//we don't want it starts moving due to gravity if is resting over other object.
	//explanation pag 116 GPED
	real ParticleContact::ModifySepVelocityDueAccelerationOnly(real sepVelocity, real duration)
	{
		glm::vec3 accCausedVelocity = mParticles[0]->GetAcceleration();
		if (mParticles[1])
		{
			accCausedVelocity -= mParticles[1]->GetAcceleration();
		}
		real accCausedSepVelocity = glm::dot(accCausedVelocity, mContactNormal) * duration;

		if (accCausedSepVelocity < 0.0f)
		{
			sepVelocity += mRestitution * accCausedSepVelocity;
			//Make sure we haven't removed more than was there to remove
			sepVelocity = glm::max(sepVelocity, 0.0f);
		}

		return sepVelocity;
	}

	void ParticleContact::ResolveInterpenetration(real duration)
	{
		//If we don't have any penetration, skip this step
		if (mPenetration <= 0.0f)
		{
			return;
		}

		//The movement of each object is based on its inverse mass, so total that.
		real totalInverseMass = mParticles[0]->GetInverseMass();
		if (mParticles[1])
		{
			totalInverseMass += mParticles[1]->GetInverseMass();
		}

		//If all particles have infinite mass, then do nothing
		if (totalInverseMass <= 0.0f)
		{
			return;
		}

		//Find the amount of penetration resolution per unit of inverse mass.
		//WARNING I have changed the sign of this part. Look at page 115 of GPED
		glm::vec3 movePerIMass = mContactNormal * (mPenetration / totalInverseMass);

		//Apply the penetration resolution
		mParticles[0]->SetPosition(mParticles[0]->GetPosition() + movePerIMass * mParticles[0]->GetInverseMass());
		if (mParticles[1])
		{
			mParticles[1]->SetPosition(mParticles[1]->GetPosition() - movePerIMass * mParticles[1]->GetInverseMass());
		}
	}
}