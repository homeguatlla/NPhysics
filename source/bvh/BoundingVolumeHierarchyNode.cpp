#include "pch.h"
#include "BoundingVolumeHierarchyNode.h"
#include "../collision/PotentialContact.h"
#include "../utils/Math.h"
#include "../PhysicsObject.h"

namespace NPhysics
{
	BoundingVolumeHierarchyNode::BoundingVolumeHierarchyNode(
		std::shared_ptr<BoundingVolumeHierarchyNode> parent,
		std::shared_ptr<PhysicsObject> object,
		std::shared_ptr<IBoundingVolume> volume) :
		mParent{ parent },
		mVolume{ volume },
		mPhysicsObject{ object }
	{
	}

	unsigned int BoundingVolumeHierarchyNode::GetPotentialContacts(std::vector<std::shared_ptr<PotentialContact>>& contacts, unsigned int limit)
	{
		if (IsLeaf() || limit == 0)
		{
			return 0;
		}
		else
		{
			if (mChildren[0] == nullptr && mChildren[1] == nullptr)
			{
				return 0;
			}
			else
			{
				return mChildren[0]->GetPotentialContactsWith(mChildren[1], contacts, limit, true);
			}
		}
	}

	void BoundingVolumeHierarchyNode::Insert(const std::shared_ptr<PhysicsObject> object, const std::shared_ptr<IBoundingVolume> volume)
	{
		volume->SetPosition(object->GetPosition());

		//If we are a leaf, then the only option is to spawn two new children and place the new body in one
		if (IsLeaf())
		{
			auto mySelf = this->shared_from_this();
			mChildren[0] = std::make_shared<BoundingVolumeHierarchyNode>(mySelf, mPhysicsObject, mVolume);
			mChildren[1] = std::make_shared<BoundingVolumeHierarchyNode>(mySelf, object, volume);
			mPhysicsObject = nullptr;
			RecalculateBoundingVolume();
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
				real growth0 = mChildren[0]->GetBoundingVolume()->GetGrowth(volume);
				real growth1 = mChildren[1]->GetBoundingVolume()->GetGrowth(volume);
				unsigned int childIndex = growth0 < growth1 ? 0 : 1;
				mChildren[childIndex]->Insert(object, volume);
			}
		}
	}

	bool BoundingVolumeHierarchyNode::IsOverlapping(const std::shared_ptr<BoundingVolumeHierarchyNode> node) const
	{
		return mVolume->IsOverlapping(node->GetBoundingVolume());
	}

	unsigned int BoundingVolumeHierarchyNode::GetPotentialContactsWith(std::shared_ptr<BoundingVolumeHierarchyNode> other, std::vector<std::shared_ptr<PotentialContact>>& contacts, unsigned int limit, bool shouldDescend)
	{
		unsigned int count = 0;

		if (shouldDescend)
		{
			if (!IsLeaf())
			{
				count += mChildren[0]->GetPotentialContactsWith(mChildren[1], contacts, limit - count, shouldDescend);
			}

			if (!other->IsLeaf())
			{
				count += other->GetChildren(0)->GetPotentialContactsWith(other->GetChildren(1), contacts, limit - count, shouldDescend);
			}
		}

		//Early out, if we don't overlap or if we have no room to report contacts
		if (!IsOverlapping(other) || limit == 0)
		{
			return count;
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
		if (other->IsLeaf() || (!IsLeaf() && mVolume->GetVolume() >= other->GetBoundingVolume()->GetVolume()))
		{
			//Recurse into ourself
			count += mChildren[0]->GetPotentialContactsWith(other, contacts, limit, false);
			//Checks whether we have enough slots to do the other side too
			if (limit > count)
			{
				return count + mChildren[1]->GetPotentialContactsWith(other, contacts, limit - count, false);
			}
			else
			{
				return count;
			}
		}
		else
		{
			//Recurse into the other node
			count += GetPotentialContactsWith(other->GetChildren(0), contacts, limit, false);
			if (limit > count)
			{
				return count + GetPotentialContactsWith(other->GetChildren(1), contacts, limit - count, false);
			}
			else
			{
				return count;
			}
		}

		return count;
	}

	void BoundingVolumeHierarchyNode::RecalculateBoundingVolume()
	{
		if (!IsLeaf())
		{
			mVolume = mChildren[0]->GetBoundingVolume()->MergeBoundingVolumes(mChildren[1]->GetBoundingVolume());
			// Recurse up the tree
			if (HasParent())
			{
				mParent->RecalculateBoundingVolume();
			}
		}
	}
}