#pragma once
#include <memory>
#include <vector>

namespace NPhysics
{
	class IBoundingVolume;
	class PotentialContact;
	class PhysicsObject;

	class BoundingVolumeHierarchyNode : public std::enable_shared_from_this<BoundingVolumeHierarchyNode>
	{
	public:
		BoundingVolumeHierarchyNode() = default;
		BoundingVolumeHierarchyNode(std::weak_ptr<BoundingVolumeHierarchyNode> parent, std::shared_ptr<PhysicsObject> object, std::shared_ptr<IBoundingVolume> volume);
		~BoundingVolumeHierarchyNode() = default;
		
		std::shared_ptr<PhysicsObject> GetPhysicsObject() const { return mPhysicsObject; }

		//Checks if this node is at the bottom of the hierarchy.
		bool IsLeaf() const { return mPhysicsObject != nullptr; }
		std::shared_ptr<IBoundingVolume> GetBoundingVolume() const { return mVolume; }
		unsigned int GetPotentialContacts(
			std::vector<std::shared_ptr<PotentialContact>>& contacts, 
			unsigned int limit);
		void Insert(
			const std::shared_ptr<PhysicsObject> object,
			const std::shared_ptr<IBoundingVolume> volume);
		void Remove(std::shared_ptr<BoundingVolumeHierarchyNode> node);
		bool Find(
			const std::shared_ptr<PhysicsObject> object, 
			const std::shared_ptr<IBoundingVolume> volume, 
			std::shared_ptr<BoundingVolumeHierarchyNode>& nodeFound);

	private:
		bool IsOverlapping(const std::shared_ptr<BoundingVolumeHierarchyNode> node) const;
		unsigned int GetPotentialContactsWith(
			std::shared_ptr<BoundingVolumeHierarchyNode> other, 
			std::vector<std::shared_ptr<PotentialContact>>& contacts,
			unsigned int limit, bool shouldDescend);
		bool HasParent() const { return mParent.expired() == false; }
		std::weak_ptr<BoundingVolumeHierarchyNode> GetParent() const { return mParent; }

	protected:
		std::shared_ptr<BoundingVolumeHierarchyNode> GetChildren(unsigned int index)
		{
			assert(index >= 0 && index < 2);  return mChildren[index];
		}
		void RecalculateBoundingVolume();

		//Holds parent reference
		std::weak_ptr<BoundingVolumeHierarchyNode> mParent;

		//Holds the child nodes of this node. Binary tree.
		std::shared_ptr<BoundingVolumeHierarchyNode> mChildren[2]{ nullptr, nullptr };

		//Holds a single bounding volume encompassing all the descendets of this node
		std::shared_ptr<IBoundingVolume> mVolume{ nullptr };

		//Holds the physics object at this node of the hieratchy. 
		//Only leaf nodes can have a physics object defined.
		std::shared_ptr<PhysicsObject> mPhysicsObject{ nullptr };
	};
};

