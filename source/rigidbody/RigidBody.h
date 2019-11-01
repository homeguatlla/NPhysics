#pragma once
#include "../../framework.h"
#include <glm/gtx/quaternion.hpp>
#include <glm/glm.hpp>


namespace NPhysics
{
	//A RigidBody is the basic simulation object in the physics core.
	class RigidBody
	{
	public:
		//Adds the given force to the center of mass of the rigid body.
		//The force is expressed int world coordinates.
		void AddForce(const glm::vec3& force);

		//Adds the given force to the given point on the rigid body.
		//The direction of the force is given in world coordinate,
		//but the application point is given in body space.
		void AddForceAtBodyPoint(const glm::vec3& force, const glm::vec3& point);

		void ResetForceAndTorqueAccumulated();

		bool HasFiniteMass() const { return mInverseMass > 0.0f; }
		real GetMass() const;

		void Integrate(real duration);

	private:
		void CalculateDerivedData();
		void CalculateTransformationMatrix(glm::mat4& matrix, const glm::vec3& position, const glm::quat& orientation);
		void SetInertiaTensor(const glm::mat3& inertiaTensor);
		void CalculateTransformInertiaTensor(glm::mat3& iiWorld, const glm::quat& q, const glm::mat3& iiBody, glm::mat4& rotmat);
		
		void AddForceAtPoint(const glm::vec3& force, const glm::vec3& point);
		glm::vec3 GetPointInWorldSpace(const glm::vec3& point);

	private:
		//Holds the inverse of mass of the rigid boy.
		real mInverseMass;

		//Holds the linear positon of the rigid body in world space
		glm::vec3 mPosition;

		//Holds the angular orientation of the rigid body in world space
		glm::quat mOrientation;

		//Holds the linear velocity of the rigid body in world space
		glm::vec3 mVelocity;

		//Holds the angular velocity or rotation of the rigid body in world space
		glm::vec3 mRotation;

		//Holds a transform matrix for converting body space into world space
		//and vice versa. 
		glm::mat4 mTransformationMatrix;

		//Holds the inverse of the body's inertia tensor. Given in body space.
		glm::mat3 mInverseInertiaTensor;

		//Holds the inverse inertia tensor of the body in world space.
		glm::mat3 mInverseInertiaTensorWorld;

		glm::vec3 mForceAccumulated;
		glm::vec3 mTorqueAccumulated;

		//Holds the acceleration of the rigid body. This value can be used to set acceleration due to gravity
		//(its primary use) or any other constant acceleration.
		glm::vec3 mAcceleration;

		//Holds the linear acceleration of the rigid body for the previous frame.
		glm::vec3 mLastFrameAcceleration;

		//Holds the amount of damping applied to linear motion. Damping is required to remove energy added through numerical 
		//inestability in the integrator
		real mLinearDamping;

		//Holds the amount of damping applied to angular motion.
		real mAngularDamping;
	};
};

