#pragma once
#include "../framework.h"
#include <glm/glm.hpp>

namespace NPhysics
{
	class PhysicsObject
	{
	public:
		PhysicsObject(const glm::vec3& initialPosition, const glm::vec3& initialVelocity, bool isStatic);
		void SetAcceleration(const glm::vec3& acceleration);
		void SetDamping(real damping);
		void SetMass(real mass);
		void SetInfiniteMass();
		void SetPosition(const glm::vec3& position);
		void SetRotation(const glm::vec3& rotation);

		void SetInitialVelocity(const glm::vec3& velocity);

		bool HasFiniteMass() const { return mInverseMass > 0.0f; }

		real GetMass() const;
		real GetInverseMass() const;

		glm::vec3 GetPosition() const { return mPosition; }
		glm::vec3 GetVelocity() const { return mVelocity; }
		glm::vec3 GetAcceleration() const { return mAcceleration; }
		glm::vec3 GetRotation() const { return DoGetRotation(); }

		bool IsStatic() const { return mIsStatic; }

	private:
		virtual void DoResetForceAccumulated() = 0;
		virtual void DoSetPosition(const glm::vec3& position) {}
		virtual glm::vec3 DoGetRotation() const { return glm::vec3(0.0f); }
		virtual void DoSetRotation(const glm::vec3& rotation) {}

	protected:

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

		//Holds if the body is static or dynamic. If it's static will never be afected by collisions
		bool mIsStatic;
	};
};

