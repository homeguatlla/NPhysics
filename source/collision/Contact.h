#pragma once
#include <glm/glm.hpp>
#include <memory>
#include <vector>
#include <functional>

namespace NPhysics
{
	class PhysicsObject;

	class Contact
	{
	public:
		Contact(const glm::vec3& point, const glm::vec3& normal, real penetration);
		glm::vec3 GetPoint() const { return mPoint; }
		glm::vec3 GetNormal() const { return mNormal; }
		real GetPenetration() const { return mPenetration; }
		real GetDesiredDeltaVelocity() const { return mDesiredDeltaVelocity; }

		void SetRestitution(real restitution) { mRestitution = restitution; }
		void SetFriction(real friction) { mFriction = friction; }

		void SetBodies(std::shared_ptr<PhysicsObject> body1, std::shared_ptr<PhysicsObject> body2);
		std::shared_ptr<PhysicsObject> GetBody(unsigned int index);

		//glm::mat3 GetWorldToContactMatrix() const { return mWorldToContactMatrix; }
		//glm::vec3 GetRelativeContactPositionForBody(int body) const;
		void ApplyPositionChanges();
		void ApplyVelocityChange();

		void Calculate(float elapsedTime);
		void UpdatePenetration(const std::shared_ptr<Contact>& contactResolved);
		void UpdateLocalVelocity(const std::shared_ptr<Contact>& contactResolved, real elapsedTime);
		glm::vec3 GetLinearChange(int bodyIndex);
		glm::vec3 GetAngularChange(int bodyIndex);
		glm::vec3 GetRotationChange(int bodyIndex);
		glm::vec3 GetVelocityChange(int bodyIndex);

		void NotifyCollisionEnter();
		void NotifyCollisionExit();

	private:
		
		real CalculateInertiaAngularVelocity(
			const glm::vec3& relativeContactPosition,
			const glm::vec3& normal,
			const glm::mat3& inverseTensorMatrix);
		real CalculateInertiaLinearVelocity(real inversMass);
		glm::vec3 CalculateLocalVelocity(
			const glm::vec3& relativeContactPosition,
			const glm::vec3& bodyVelocity,
			const glm::vec3& bodyRotation,
			const glm::mat3& fromWorldToContactMatrix);
		real CalculateDesiredDeltaVelocity(real elapsedTime);
		real CalculateLimitedAngular(const glm::vec3& relativePosition, const glm::vec3& normal, real angular);
		void CalculateLocalVelocity();
		glm::vec3 CalculateFrictionlessImpulse();
		real CalculateInertiaVelocity(
			std::vector<real>& inertiaLinearVelocity, 
			std::vector<real>& inertiaAngularVelocity);
		void PerformActionForEachBodyEqualToTheContact(
			const std::shared_ptr<Contact>& contactResolved,
			std::function<void(int bodyIndex, int bodyResolvedIndex)> action);

	private:
		//Holds the position of the contact in world coordinates
		glm::vec3 mPoint;

		//Holds the direction of the contact in world coordinates
		glm::vec3 mNormal;

		//Holds the depth of penetration at the contact point. If both bodies
		//are specified then the contact point should be midway between the 
		//inter-penetrating points
		real mPenetration;

		std::shared_ptr<PhysicsObject> mBodies[2];

		//Holds the normal restitution coefficient at the contact.
		//value of 0.0 the objects colliding will not separate
		//value of 1.0 the objects colliding will separate (elastic)
		//the value depends on the materials.
		//En realidad depende de la relación de la velocidad antes y después de la colisión de ambos bodies.
		real mRestitution;

		//Holds the lateral friction coefficient at the contact.
		real mFriction;

		//Info saved to be used in collisionResolver phases
		glm::mat3 mContactLocalMatrix;
		glm::mat3 mWorldToContactMatrix;

		//Holds the closing velocity at the point of contact.
		glm::vec3 mLocalVelocity;

		//Holds the required change in velocity for this contact to be resolved.
		real mDesiredDeltaVelocity;
		//Holds the total inertia velocity
		real mDeltaVelocity;
		bool mIsDeltaVelocityCalculated;

		//Holds the world space position of the contact point relative to the center of each body.
		glm::vec3 mRelativeContactPosition[2];

		std::vector<glm::vec3> mLinearChange{ glm::vec3(0.0f), glm::vec3(0.0f) };
		std::vector<glm::vec3> mAngularChange{ glm::vec3(0.0f), glm::vec3(0.0f) };

		std::vector<glm::vec3> mVelocityChange{ glm::vec3(0.0f), glm::vec3(0.0f) };
		std::vector<glm::vec3> mRotationChange{ glm::vec3(0.0f), glm::vec3(0.0f) };
	};
};
