#pragma once
#include "../../framework.h"
#include "../PhysicsObject.h"
#include <glm/gtx/quaternion.hpp>
#include "../collision/Contact.h"
#include <glm/glm.hpp>
#include <functional>

namespace NPhysics
{
	//A RigidBody is the basic simulation object in the physics core.
	class RigidBody : public PhysicsObject
	{
	public:
		explicit RigidBody(const glm::vec3& position, const glm::vec3& angularVelocity, const glm::vec3& initialVelocity, bool isStatic);

		//Adds the given force to the center of mass of the rigid body.
		//The force is expressed int world coordinates.
		void AddForce(const glm::vec3& force);

		//Adds the given force to the given point on the rigid body.
		//The direction of the force is given in world coordinate,
		//but the application point is given in body space.
		void AddForceAtBodyPoint(const glm::vec3& force, const glm::vec3& point);

		void SetInertiaTensorMatrix(const glm::mat3& matrix);
		glm::mat3 GetInverseInertiaTensorWorldMatrix() const { return mInverseInertiaTensorWorld; }

		void Integrate(real duration);

		glm::vec3 GetPointInWorldSpace(const glm::vec3& point);

		void DoResetForceAccumulated() override;

		void SetRotation(const glm::vec3& initialRotation);
		glm::vec3 GetRotation() const;
		void SetOrientation(const glm::quat& orientation);
		glm::quat GetOrientation() const { return mOrientation; }

		void SetAngularDamping(float damping);

		void RegisterCollisionEnterHandler(std::function<void(const Contact& contact)> callback) { mCollisionEnterHandler = callback; }
		void RegisterCollisionExitHandler(std::function<void(const Contact& contact)> callback) { mCollisionExitHandler = callback; }
		void OnCollisionEnter(const Contact& contact);
		void OnCollisionExit(const Contact& contact);

	private:
		void DoSetPosition(const glm::vec3& position) override;
		void DoSetRotation(const glm::vec3& rotation) override;
		virtual glm::vec3 DoGetRotation() const override;

		void CalculateDerivedData();
		glm::mat4 CalculateTransformationMatrix(const glm::vec3& position, const glm::quat& orientation);
		void SetInertiaTensor(const glm::mat3& inertiaTensor);
		glm::mat3 CalculateTransformInertiaTensor(const glm::quat& q, const glm::mat3& iiBody, glm::mat4& rotmat);
		
		void AddForceAtPoint(const glm::vec3& force, const glm::vec3& point);
		void ResetForceAndTorqueAccumulated();
		
	private:
		//Holds the angular orientation of the rigid body in world space
		glm::quat mOrientation;

		//Holds the angular velocity or rotation of the rigid body in world space
		glm::vec3 mAngularVelocity;

		//Holds a transform matrix for converting body space into world space
		//and vice versa. 
		glm::mat4 mTransformationMatrix;

		//Holds the inverse of the body's inertia tensor. Given in body space.
		glm::mat3 mInverseInertiaTensor;

		//Holds the inverse inertia tensor of the body in world space.
		glm::mat3 mInverseInertiaTensorWorld;

		glm::vec3 mForceAccumulated;
		glm::vec3 mTorqueAccumulated;

		//Holds the linear acceleration of the rigid body for the previous frame.
		glm::vec3 mLastFrameAcceleration;

		//Holds the amount of damping applied to angular motion.
		real mAngularDamping;

		std::function<void(const Contact& contact)> mCollisionEnterHandler;
		std::function<void(const Contact& contact)> mCollisionExitHandler;
	};
};

