#pragma once
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include "../bvh/boundingVolumes/SphereBoundingVolume.h"
#include "../bvh/boundingVolumes/BoxBoundingVolume.h"
#include "../collision/Contact.h"

#include <glm/gtc/epsilon.hpp>
#include <glm/gtx/transform.hpp>
#include <iostream>

namespace NPhysics
{
	class NMath
	{
	public:
		static float FRAME_TIME;
		static float FLOAT_TOLERANCE;

		static glm::quat FromEulerAnglesToQuaternion(const glm::vec3& euler) {
			// Assuming the angles are in radians.
			double c1 = glm::cos(euler.y * 0.5f);
			double s1 = glm::sin(euler.y * 0.5f);
			double c2 = glm::cos(euler.x * 0.5f);
			double s2 = glm::sin(euler.x * 0.5f);
			double c3 = glm::cos(euler.z * 0.5f);
			double s3 = glm::sin(euler.z * 0.5f);
			/*double w = glm::sqrt(1.0f + c1 * c2 + c1 * c3 - s1 * s2 * s3 + c2 * c3) / 2.0f;
			double w4 = (4.0f * w);
			double x = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / w4;
			double y = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / w4;
			double z = (-s1 * s3 + c1 * s2 * c3 + s2) / w4;
			*/

			double c1c2 = c1 * c2;
			double s1s2 = s1 * s2;
			double w = c1c2 * c3 - s1s2 * s3;
			double x = c1c2 * s3 + s1s2 * c3;
			double y = s1 * c2 * c3 + c1 * s2 * s3;
			double z = c1 * s2 * c3 - s1 * c2 * s3;

			return glm::normalize(glm::quat((float)w, (float)x, (float)y, (float)z));
		}

		static glm::vec3 FromQuatToEulerAngles(const glm::quat& q) {
			double sqw = q.w * q.w;
			double sqx = q.x * q.x;
			double sqy = q.y * q.y;
			double sqz = q.z * q.z;
			double unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
			double test = q.x * q.y + q.z * q.w;

			double heading, attitude, bank;

			if (test > 0.499 * unit) { // singularity at north pole
				heading = 2 * atan2(q.x, q.w);
				attitude = glm::pi<float>() / 2.0f;
				bank = 0;
				return glm::vec3(attitude, heading, bank);
			}

			if (test < -0.499 * unit) { // singularity at south pole
				heading = -2 * atan2(q.x, q.w);
				attitude = -glm::pi<float>() / 2.0f;
				bank = 0;
				return glm::vec3(attitude, heading, bank);
			}

			heading = atan2(2 * q.y * q.w - 2 * q.x * q.z, sqx - sqy - sqz + sqw);
			attitude = asin(2 * test / unit);
			bank = atan2(2 * q.x * q.w - 2 * q.y * q.z, -sqx + sqy - sqz + sqw);

			return glm::vec3(attitude, heading, bank);
		}

		static bool IsNearlyEqual(const glm::vec3& v1, const glm::vec3& v2, const float epsilon = 0.0001f)
		{
			const glm::vec3 epsilon3(epsilon);
			//return glm::epsilonEqual<glm::vec3>(v1, v2, epsilon3);
			return glm::all(glm::lessThan(glm::abs(v1 - v2), epsilon3));
		}

		static bool IsNearlyEqual(float a, float b, const float epsilon = 0.0001f)
		{
			return glm::abs(a - b) < epsilon;
		}

		//v1 must be normalized and cannot be zero
		static glm::mat3 CreateOrthonormalBasis(const glm::vec3& v1)
		{
			assert(v1 != glm::vec3(0.0f));

			glm::vec3 v2 = glm::abs(v1.x) > glm::abs(v1.y) ? glm::vec3(v1.z, 0.0f, -v1.x) : glm::vec3(0.0f, -v1.z, v1.y);
			v2 = glm::normalize(v2);

			glm::vec3 v3 = glm::cross(v1, v2);

			return glm::mat3(v1, v2, v3);
		}

		static bool IsOverlapping(const SphereBoundingVolume& sphere1, const SphereBoundingVolume& sphere2)
		{
			real distance = glm::distance(sphere1.GetPosition(), sphere2.GetPosition());
			return distance < sphere1.GetRadius() + sphere2.GetRadius();
		}

		static bool IsOverlapping(const BoxBoundingVolume& box1, const BoxBoundingVolume& box2)
		{
			glm::vec3 axis11 = glm::normalize(box1.GetTransformation()[0]);
			glm::vec3 axis12 = glm::normalize(box1.GetTransformation()[1]);
			glm::vec3 axis13 = glm::normalize(box1.GetTransformation()[2]);

			glm::vec3 axis21 = glm::normalize(box2.GetTransformation()[0]);
			glm::vec3 axis22 = glm::normalize(box2.GetTransformation()[1]);
			glm::vec3 axis23 = glm::normalize(box2.GetTransformation()[2]);

			bool isOverlapping = 
				OverlapOnAxis(box1, box2, axis11) &&
				OverlapOnAxis(box1, box2, axis12) &&
				OverlapOnAxis(box1, box2, axis13) &&

				OverlapOnAxis(box1, box2, axis21) &&
				OverlapOnAxis(box1, box2, axis22) &&
				OverlapOnAxis(box1, box2, axis23) &&

				OverlapOnAxis(box1, box2, glm::cross(axis11, axis21)) &&
				OverlapOnAxis(box1, box2, glm::cross(axis11, axis22)) &&
				OverlapOnAxis(box1, box2, glm::cross(axis11, axis23)) &&
				OverlapOnAxis(box1, box2, glm::cross(axis12, axis21)) &&
				OverlapOnAxis(box1, box2, glm::cross(axis12, axis22)) &&
				OverlapOnAxis(box1, box2, glm::cross(axis12, axis23)) &&
				OverlapOnAxis(box1, box2, glm::cross(axis13, axis21)) &&
				OverlapOnAxis(box1, box2, glm::cross(axis13, axis22)) &&
				OverlapOnAxis(box1, box2, glm::cross(axis13, axis23));

			return isOverlapping;
		}

		static bool IsOverlapping(const BoxBoundingVolume& box, const SphereBoundingVolume& sphere)
		{
			auto size = box.GetSize();
			auto transformation = box.GetTransformation();
			auto transformationInverse = glm::inverse(transformation);
			auto centerInBoxSpace = transformationInverse * glm::vec4(sphere.GetPosition(), 1.0f);
			
			auto halfSize = size * 0.5f;
			bool outInX = glm::abs(centerInBoxSpace.x) - sphere.GetRadius() > halfSize.x;
			bool outInY = glm::abs(centerInBoxSpace.y) - sphere.GetRadius() > halfSize.y;
			bool outInZ = glm::abs(centerInBoxSpace.z) - sphere.GetRadius() > halfSize.z;

			return !(outInX || outInY || outInZ);
		}

		static std::shared_ptr<IBoundingVolume> MergeBoundingVolumes(const SphereBoundingVolume& sphere1, const SphereBoundingVolume& sphere2)
		{
			auto newVolume = SphereBoundingVolume(sphere1, sphere2);

			return std::make_shared<SphereBoundingVolume>(newVolume);
		}

		static std::shared_ptr<IBoundingVolume> MergeBoundingVolumes(const BoxBoundingVolume& box1, const BoxBoundingVolume& box2)
		{
			auto newVolume = BoxBoundingVolume(box1, box2);

			return std::make_shared<BoxBoundingVolume>(newVolume);
		}

		static std::shared_ptr<IBoundingVolume> MergeBoundingVolumes(const BoxBoundingVolume& box, const SphereBoundingVolume& sphere)
		{
			auto box2 = BoxBoundingVolume(sphere.GetPosition(), glm::vec3(sphere.GetRadius() * 2));
			auto newVolume = BoxBoundingVolume(box, box2);

			return std::make_shared<BoxBoundingVolume>(newVolume);
		}

		//volume1 contains volume2
		static bool Contains(const SphereBoundingVolume& sphere1, const SphereBoundingVolume& sphere2)
		{
			real distance = glm::distance(sphere1.GetPosition(), sphere2.GetPosition());
			return distance < sphere1.GetRadius() - sphere2.GetRadius();
		}

		static bool Contains(const BoxBoundingVolume& box1, const BoxBoundingVolume& box2)
		{
			return false;
		}

		static bool Contains(const BoxBoundingVolume& box, const SphereBoundingVolume& sphere)
		{
			auto size = box.GetSize();
			auto transformation = box.GetTransformation();
			auto transformationInverse = glm::inverse(transformation);
			auto centerInBoxSpace = transformationInverse * glm::vec4(sphere.GetPosition(), 1.0f);
			
			auto halfSize = size * 0.5f;
			bool contains = glm::abs(centerInBoxSpace.x) + sphere.GetRadius() < halfSize.x;
			contains |= glm::abs(centerInBoxSpace.y) + sphere.GetRadius() < halfSize.y;
			contains |= glm::abs(centerInBoxSpace.z) + sphere.GetRadius() < halfSize.z;

			return contains;
		}

		//sphere 1 collides with sphere2
		static std::shared_ptr<Contact> ResolveCollision(const SphereBoundingVolume& sphere1, const SphereBoundingVolume& sphere2)
		{
			glm::vec3 position1 = sphere1.GetPosition();
			glm::vec3 position2 = sphere2.GetPosition();

			//Find the vector between positions
			glm::vec3 midline = position1 - position2;
			real size = glm::length(midline);

			if (size > 0.0f && size < sphere1.GetRadius() + sphere2.GetRadius())
			{
				glm::vec3 normal = midline / size;
				glm::vec3 point = position2 + normal * sphere2.GetRadius();
				real penetration = sphere1.GetRadius() + sphere2.GetRadius() - size;
				//TODO there is a case, when two spheres are coincident. 
				//the normal is zero and cannot be calculated here because we need the current velocity
				//of one of the spheres to know where it was coming and then calculate the normal in the 
				//direction of the movement.
				
				return std::make_shared<Contact>(
					point, 
					normal, 
					penetration);
			}

			return nullptr;
		}

		static std::shared_ptr<Contact> ResolveCollision(const BoxBoundingVolume& box1, const BoxBoundingVolume& box2)
		{
			return std::make_shared<Contact>(glm::vec3(0.0f), glm::vec3(0.0f), 0.0f);
		}

		static std::shared_ptr<Contact> ResolveCollision(const BoxBoundingVolume& box, const SphereBoundingVolume& sphere, float sign)
		{
			auto size = box.GetSize();
			auto transformation = box.GetTransformation();
			auto transformationInverse = glm::inverse(transformation);
			auto centerInBoxSpace = transformationInverse * glm::vec4(sphere.GetPosition(), 1.0f);

			auto halfSize = size * 0.5f;
			glm::vec3 contactPoint;

			contactPoint.x = glm::max(glm::min(halfSize.x, centerInBoxSpace.x), -halfSize.x);
			contactPoint.y = glm::max(glm::min(halfSize.y, centerInBoxSpace.y), -halfSize.y);
			contactPoint.z = glm::max(glm::min(halfSize.z, centerInBoxSpace.z), -halfSize.z);

			real distance2 = glm::distance2(contactPoint, glm::vec3(centerInBoxSpace));

			if (distance2 <= sphere.GetRadius() * sphere.GetRadius())
			{
				auto contactPointWorld = glm::vec3(transformation * glm::vec4(contactPoint, 1.0f));
				glm::vec3 normal = contactPointWorld - sphere.GetPosition();
				if (IsNearlyEqual(normal, glm::vec3(0.0f)))
				{
					//normal zero: esto hace que se calcule mal los datos de la colisión
					//hay que calcular la normal lo más aproximado posible, quizá aquí no porque no tienes todos los datos.
					//Lo ideal seria tener la velocidad y -posicionar- el objeto un poco antes para que no coincidan en punto.
					//pero igual se tendría que hacer en el contactResolver cuando ve que la normal devuelta es cero.
					normal = box.GetPosition() - sphere.GetPosition();
				}
				normal = glm::normalize(normal);

				//std::cout << "normal = " << normal.x << ", " << normal.y << ", " << normal.z << std::endl;

				real penetration = sphere.GetRadius() - glm::sqrt(distance2);

				return std::make_shared<Contact>(
					contactPointWorld,
					normal * sign,
					penetration);
			}
			else
			{
				return nullptr;
			}			
		}

		static bool OverlapOnAxis(const BoxBoundingVolume& box1, const BoxBoundingVolume& box2, const glm::vec3& axis)
		{
			real distance1 = TransformToAxis(box1, axis);
			real distance2 = TransformToAxis(box2, axis);

			if (NMath::IsNearlyEqual(distance1, 0.0f) || NMath::IsNearlyEqual(distance2, 0.0f))
			{
				return true;
			}

			glm::vec3 center = box2.GetPosition() - box1.GetPosition();
			real distanceBoxes = glm::abs(glm::dot(center, axis));

			bool isOverlapping = distanceBoxes < distance1 + distance2;

			if (!isOverlapping)
			{
				int kk = 0;
				++kk;
			}
			return isOverlapping;
		}

		static inline real TransformToAxis(const BoxBoundingVolume& box, const glm::vec3& axis)
		{
			auto transformation = box.GetTransformation();
			glm::vec3 halfSize = box.GetSize() * 0.5f;

			glm::vec4 axis0 = transformation[0];
			glm::vec4 axis1 = transformation[1];
			glm::vec4 axis2 = transformation[2];

			return
				halfSize.x * glm::abs(glm::dot(glm::vec4(axis, 0.0f), transformation[0])) +
				halfSize.y * glm::abs(glm::dot(glm::vec4(axis, 0.0f), transformation[1])) +
				halfSize.z * glm::abs(glm::dot(glm::vec4(axis, 0.0f), transformation[2]));
		}
	};
};

