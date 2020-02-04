#pragma once
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include "../bvh/boundingVolumes/SphereBoundingVolume.h"

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

			return glm::normalize(glm::quat(w, x, y, z));
		}

		static glm::vec3 FromQuatToEulerAngles(const glm::quat& q) {
			double sqw = q.w * q.w;
			double sqx = q.x * q.x;
			double sqy = q.y * q.y;
			double sqz = q.z * q.z;
			double unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
			double test = q.x * q.y + q.z * q.w;

			float heading, attitude, bank;

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

		static bool IsOverlapping(const SphereBoundingVolume& sphere1, const SphereBoundingVolume& sphere2)
		{
			real distance = glm::distance(sphere1.GetCenter(), sphere2.GetCenter());
			return distance < sphere1.GetRadius() + sphere2.GetRadius();
		}

		static SphereBoundingVolume MergeBoundingVolumes(const SphereBoundingVolume& sphere1, const SphereBoundingVolume& sphere2)
		{
			return SphereBoundingVolume(sphere1, sphere2);
		}
	};
};

