#pragma once
#include "../../framework.h"
#include <glm/glm.hpp>

namespace NPhysics {

	class Particle
	{
	public:
		explicit Particle(const glm::vec3& initialPosition, const glm::vec3& initialVelocity);
		~Particle() = default;

		void SetAcceleration(const glm::vec3& acceleration);
		void SetDamping(real damping);
		void SetMass(real mass);
		void SetInfiniteMass();

		void Integrate(real duration);
		void AddForce(const glm::vec3& force);

		bool HasInfinteMass() const { return mInverseMass <= 0.0f; }
		glm::vec3 GetPosition() const { return mPosition; }
		glm::vec3 GetVelocity() const { return mVelocity; }
		glm::vec3 GetAcceleration() const { return mAcceleration; }
		real GetMass() const;

	private:
		void ResetForceAccumulated();

	private:

		glm::vec3 mPosition;
		glm::vec3 mVelocity;
		glm::vec3 mAcceleration;

		/*
			Holds the amount of damping applied to linear motion. 
			Damping is required to remove energy added through numerical inestability in the integrator.
		*/
		real mDamping;

		/*
			Holds the inverse of mass. One particle inmovable has a inverseMass of zero.
			It will be impossible to have a 1.0/mass infinite because of mass = 0.0
		*/
		real mInverseMass;

		glm::vec3 mForceAccumulated;
	};

};
