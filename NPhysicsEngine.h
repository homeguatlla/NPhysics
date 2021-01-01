#pragma once
#include "source/particle/ParticlePhysicsEngine.h"
#include "source/rigidbody/RigidBodyPhysicsEngine.h"
#include "source/particle/Particle.h"
#include "source/rigidbody/RigidBody.h"
#include "glm/gtx/transform.hpp"

namespace NPhysics
{
	class IBoundingVolume;

	class NPhysicsEngine
	{
	public:
		NPhysicsEngine();
		~NPhysicsEngine() = default;

		void AddParticle(std::shared_ptr<Particle> particle);
		void RegisterParticleForceGenerator(std::shared_ptr<Particle> particle, std::shared_ptr<IForceGenerator<Particle>> forceGenerator);
		void Update(real duration);

		void AddRigidBody(std::shared_ptr<RigidBody> body, std::shared_ptr<IBoundingVolume> volume);
		void RegisterRigidBodyForceGenerator(std::shared_ptr<RigidBody> body, std::shared_ptr<IBoundingVolume> volume, std::shared_ptr<IForceGenerator<RigidBody>> forceGenerator);

		[[deprecated("Each IBoundingVolume should implement its own inertia tensor matrix.")]]
		static glm::mat3 GetInertiaTensorMatrix(real mass, const glm::vec3& size)
		{
			real dx = size.x * size.x;
			real dy = size.y * size.y;
			real dz = size.z * size.z;
			real k = mass / 12.0f;

			glm::mat3 inertiaTensorMatrix = glm::scale(glm::vec3(k * (dy + dz), k * (dx + dz), k * (dx + dy)));

			return inertiaTensorMatrix;
		}

	private:
		ParticlePhysicsEngine mParticlePhysicsEngine;
		RigidBodyPhysicsEngine mRigidBodyPhysicsEngine;
	};
};

