#pragma once
#include "source/particle/ParticlePhysicsEngine.h"
#include "source/rigidbody/RigidBodyPhysicsEngine.h"
#include "source/particle/Particle.h"
#include "source/rigidbody/RigidBody.h"
#include "glm/gtx/transform.hpp"

namespace NPhysics
{
	class NPhysicsEngine
	{
	public:
		void AddParticle(std::shared_ptr<Particle>& particle);
		void RegisterParticleForceGenerator(std::shared_ptr<Particle>& particle, std::shared_ptr<IForceGenerator<Particle>>& forceGenerator);
		void Update(real duration);

		void AddRigidBody(std::shared_ptr<RigidBody>& body);
		void RegisterRigidBodyForceGenerator(std::shared_ptr<RigidBody>& body, std::shared_ptr<IForceGenerator<RigidBody>>& forceGenerator);

		static glm::mat3 GetInertiaTensorMatrix(real mass, glm::vec3 size)
		{
			real dx = size.x * size.x;
			real dy = size.y * size.y;
			real dz = size.z * size.z;
			real k = 1.0f / 12.0f * mass;

			glm::mat3 inertiaTensorMatrix = glm::scale(glm::vec3(k * (dy + dz), k * (dx + dz), k * (dx + dy)));

			return inertiaTensorMatrix;
		}

	private:
		ParticlePhysicsEngine mParticlePhysicsEngine;
		RigidBodyPhysicsEngine mRigidBodyPhysicsEngine;
	};
};

