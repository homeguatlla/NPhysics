#pragma once
#include "boundingVolumes/IBoundingVolume.h"
#include "../PhysicsObject.h"
#include "../collision/PotentialContact.h"
#include "../utils/Math.h"

#include <memory>
#include <vector>

namespace NPhysics
{
	template<class boundingVolumeT>
	class BoundingVolumeHierarchyNode
	{
	public:
		BoundingVolumeHierarchyNode() = default;
		BoundingVolumeHierarchyNode(std::shared_ptr<BoundingVolumeHierarchyNode<boundingVolumeT>> parent, std::shared_ptr<PhysicsObject> object, boundingVolumeT volume);
		~BoundingVolumeHierarchyNode() = default;
		
		std::shared_ptr<PhysicsObject> GetPhysicsObject() const { return mPhysicsObject; }

		//Checks if this node is at the bottom of the hierarchy.
		bool IsLeaf() const { return mPhysicsObject != nullptr; }
		boundingVolumeT GetBoundingVolume() const { return mVolume; }
		unsigned int GetPotentialContacts(
			std::vector<std::shared_ptr<PotentialContact>>& contacts, 
			unsigned int limit);
		std::shared_ptr<BoundingVolumeHierarchyNode<boundingVolumeT>> GetChildren(unsigned int index)
				{ assert(index >= 0 && index < 2);  return mChildren[index]; }
		void Insert(
			std::shared_ptr<PhysicsObject>& object,
			const boundingVolumeT& volume);
	private:
		bool IsOverlapping(const std::shared_ptr<BoundingVolumeHierarchyNode<boundingVolumeT>> node) const;
		unsigned int GetPotentialContactsWith(
			std::shared_ptr<BoundingVolumeHierarchyNode> other, 
			std::vector<std::shared_ptr<PotentialContact>>& contacts,
			unsigned int limit);
		void RecalculateBoungingVolume();
		bool HasParent() const { mParent != nullptr; }

	private:
		//Holds parent reference
		std::shared_ptr<BoundingVolumeHierarchyNode<boundingVolumeT>> mParent;

		//Holds the child nodes of this node. Binary tree.
		std::shared_ptr<BoundingVolumeHierarchyNode<boundingVolumeT>> mChildren[2]{ nullptr, nullptr };

		//Holds a single bounding volume encompassing all the descendets of this node
		boundingVolumeT mVolume;

		//Holds the physics object at this node of the hieratchy. 
		//Only leaf nodes can have a physics object defined.
		std::shared_ptr<PhysicsObject> mPhysicsObject{ nullptr };
	};
	template<class boundingVolumeT>
	inline BoundingVolumeHierarchyNode<boundingVolumeT>::BoundingVolumeHierarchyNode(std::shared_ptr<BoundingVolumeHierarchyNode<boundingVolumeT>> parent, std::shared_ptr<PhysicsObject> object, boundingVolumeT volume) :
		mParent { parent },
		mVolume{ volume },
		mPhysicsObject { object }
		
	{
	}

	template<class boundingVolumeT>
	inline unsigned int BoundingVolumeHierarchyNode<boundingVolumeT>::GetPotentialContacts(std::vector<std::shared_ptr<PotentialContact>>& contacts, unsigned int limit)
	{
		if (IsLeaf() || limit == 0)
		{
			return 0;
		}
		else
		{
			return mChildren[0]->GetPotentialContactsWith(mChildren[1], contacts, limit);
		}
	}

	template<class boundingVolumeT>
	inline void BoundingVolumeHierarchyNode<boundingVolumeT>::Insert(std::shared_ptr<PhysicsObject>& object, const boundingVolumeT& volume)
	{
		//If we are a leaf, then the only option is to spawn two new children and place the new body in one
		if (IsLeaf())
		{
			mChildren[0] = std::make_shared<BoundingVolumeHierarchyNode>(this, mVolume, mPhysicsObject);
			mChildren[1] = std::make_shared<BoundingVolumeHierarchyNode>(this, volume, object);
			mPhysicsObject = nullptr;
			RecalculateBoungingVolume();
		}
		else
		{
			if (mChildren[0] == nullptr && mChildren[1] == nullptr)
			{
				mVolume = volume;
				mPhysicsObject = object;
			}
			else
			{
				unsigned int childIndex = mChildren[0]->GetVolume().GetGrowth(volume) < mChildren[1]->GetVolume().GetGrowth(volume) ? 0 : 1;
				mChildren[childIndex]->Insert(object, volume);
			}
		}
	}

	template<class boundingVolumeT>
	inline bool BoundingVolumeHierarchyNode<boundingVolumeT>::IsOverlapping(const std::shared_ptr<BoundingVolumeHierarchyNode<boundingVolumeT>> node) const
	{
		return mVolume->Overlaps(node->GetBoundingVolume());
	}

	template<class boundingVolumeT>
	inline unsigned int BoundingVolumeHierarchyNode<boundingVolumeT>::GetPotentialContactsWith(std::shared_ptr<BoundingVolumeHierarchyNode> other, std::vector<std::shared_ptr<PotentialContact>>& contacts, unsigned int limit)
	{
		//Early out, if we don't overlap or if we have no room to report contacts
		if (!IsOverlapping(other) || limit == 0)
		{
			return 0;
		}

		//if we're both at leaf nodes, then we have a potential contact
		if (IsLeaf() && other->IsLeaf())
		{
			auto contact = std::make_shared<PotentialContact>(mPhysicsObject, other->GetPhysicsObject());
			contacts.push_back(contact);
			return 1;
		}

		//Determine which node to descend into. If either is a leaf, then we descend the other.
		//If both are branches, then we use the one with the largest size.
		if (other->IsLeaf() || (!IsLeaf() && mVolume->GetSize() >= other->GetVolume()->GetSize()))
		{
			//Recurse into ourself
			auto count = mChildren[0]->GetPotentialContactsWith(other, contacts, limit);
			//Checks whether we have enough slots to do the other side too
			if (limit > count)
			{
				return count + mChildren[1]->GetPotentialContactsWith(other, contacts, limit - count);
			}
			else
			{
				return count;
			}
		}
		else
		{
			//Recurse into the other node
			auto count = GetPotentialContactsWith(other->GetChildren(0), contacts, limit);
			if (limit > count)
			{
				return count + GetPotentialContactsWith(other->GetChildren(1), contacts, limit - count);
			}
			else
			{
				return count;
			}
		}
	}

	template<class boundingVolumeT>
	inline void BoundingVolumeHierarchyNode<boundingVolumeT>::RecalculateBoungingVolume()
	{
		if (!IsLeaf())
		{
			mVolume = NMath::MergeBoundingVolumes(mChildren[0]->GetVolume(), mChildren[1]->GetVolume());
			// Recurse up the tree
			if (HasParent())
			{
				mParent->RecalculateBoundingVolume();
			}
		}
	}
};

