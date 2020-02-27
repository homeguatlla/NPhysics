#pragma once
#include <glm/glm.hpp>
#include <memory>

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

		void SetBodies(std::shared_ptr<PhysicsObject> body1, std::shared_ptr<PhysicsObject> body2);
		std::shared_ptr<PhysicsObject> GetBody(unsigned int index);

		//glm::mat3 GetWorldToContactMatrix() const { return mWorldToContactMatrix; }
		glm::vec3 GetRelativeContactPositionForBody(int body) const;

		void Calculate(float elapsedTime);
	private:
		real CalculateFrictionLessImpulse();
		real CalculateFrictionLessImpulsePerBody(
			const glm::vec3& relativeContactPosition,
			const glm::vec3& normal,
			const glm::mat3& inverseTensorMatrix,
			real inverseMass);
		glm::vec3 CalculateLocalVelocity(
			const glm::vec3& relativeContactPosition,
			const glm::vec3& bodyVelocity,
			const glm::vec3& bodyRotation,
			const glm::mat3& fromWorldToContactMatrix);
		real CalculateDesiredDeltaVelocity();

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

		//Info saved to be used in collisionResolver phases
		glm::mat3 mContactLocalMatrix;
		glm::mat3 mWorldToContactMatrix;

		//Holds the closing velocity at the point of contact.
		glm::vec3 mLocalVelocity;

		//Holds the required change in velocity for this contact to be resolved.
		real mDesiredDeltaVelocity;

		//Holds the world space position of the contact point relative to the center of each body.
		glm::vec3 mRelativeContactPosition[2];
	};
};
