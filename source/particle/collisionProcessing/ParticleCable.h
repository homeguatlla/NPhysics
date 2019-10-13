#pragma once
#include "../../../framework.h"
#include <memory>

namespace NPhysics
{
	class Particle;
	class ParticleContact;
}

namespace NPhysics
{
	//Links connect two particles together, generating a contact if they violate the constraints of their link.
	//It is used as a base class for cables and rods, and could be used as a base class for springs with a limit
	//to their extension.
	class ParticleCable
	{
	public:
		//Fills the given contact structure with the contact needed to keep the link from violaing its constraint.
		//The contact pointer should point to the first available contact in a contact array, where limit is the
		//maximum number of contacts in the array that can be written to. The method returns the number of contacts
		//that have been written. This format is common to contact-generating functions, but this class 
		//can only generate a single contact, so the pointer can be a pointer to a single element. The limit
		//parameter is assumed to be at least one (zero isn't valid), and the return value is either 0, if the
		//cable wasn't averextended, or one if a contact was needed.
		virtual unsigned int FillContact(std::shared_ptr<ParticleContact> contact, unsigned int limit) = 0;
	protected:
		//Returns the current length of the cable
		real CurrentLength() const;

	private:
		//Holds the pair of particles that are connected by this link
		std::shared_ptr<Particle> mParticles[2];
	};
};

