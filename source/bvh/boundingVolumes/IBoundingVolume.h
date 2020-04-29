#pragma once
#include "../../../framework.h"
#include <glm/glm.hpp>
#include <memory>

namespace NPhysics
{
	class IBoundingVolume
	{
	public:
		virtual bool IsOverlapping(std::shared_ptr<IBoundingVolume> volume) const = 0;
		virtual real GetGrowth(std::shared_ptr<IBoundingVolume> volume) const = 0;
		virtual std::shared_ptr<IBoundingVolume> MergeBoundingVolumes(std::shared_ptr<IBoundingVolume> volume) const = 0;
		virtual bool Contains(std::shared_ptr<IBoundingVolume> volume) const = 0;
		virtual real GetVolume() const = 0;
		virtual const glm::mat3 GetInertiaTensorMatrix(float mass, bool isShell) const = 0;
		virtual void SetPosition(const glm::vec3& position) = 0;
		virtual glm::vec3 GetPosition() const = 0;
		virtual glm::mat4 GetTransformation() const = 0;
		virtual glm::vec3 GetSize() const = 0;
		virtual std::shared_ptr<IBoundingVolume> Clone() = 0;
	};
};

