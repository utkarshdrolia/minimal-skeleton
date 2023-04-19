#include "Skeleton.h"
#include "Link.h"
#include <string.h>
#include <iostream>
#include "BVHReader.h"
#include <assert.h>

Skeleton::Skeleton()
{
	m_linkCnt = 0;
}
Skeleton::~Skeleton()
{

}


void Skeleton::CreateSkeletonFromBVH(char* filename, AnimRec* pAnimRec, bool inToM)//, CKinSkeleton * skel)
{
	BVHReader parser;
	//load the bvh
	std::ifstream file(filename);
	parser.BuildSkelFromHeader(file, this, pAnimRec, inToM);

}

//adds link addMe to the tree that defines the skeleton such that its
//parent in the tree is called parentName.  Note:  $ground is the parent
//of the root node.
//Returns true if node can be added, false otherwise
//The link is also added to the skeletons link array.  This is used to directly
//access links to set their state, for instance
bool Skeleton::AddToSkeleton(Link* addMe, char* parentName)
{
	if (strcmp(parentName, "$ground") == 0)
	{
		if (m_pSkelRoot)
		{
			std::cerr <<"Defining a root for the skeleton, but one has already been defined." << std::endl;
		}
		//this is the root
		m_pSkelRoot = addMe;
	}
	else
	{
		Link* par = FindNode(parentName, m_pSkelRoot);
		if (par)
		{
			par->AddChild(addMe);

		}
		else
		{
			std::cerr << "failed to find the parent, so could not add node to skeleton" << std::endl;
			return false;
		}

	}


	//add link to array
	m_linkArray[m_linkCnt] = addMe;
	m_linkCnt++;

	return true;
}


//Searches the tree for a node with the given name
//Returns a pointer to the node if it finds it, otherwise returns null
//The search is started at curNode
//depth first search
Link* Skeleton::FindNode(char* name, Link* curNode)
{
	if (strcmp(curNode->GetName(), name) == 0)
	{
		return curNode;
	}
	else
	{
		//recurse on each of the children until we find the desired node
		//or exhaust the tree

		int childCnt = 0;
		Link* link;
		while (childCnt < KL_MAX_CHILDREN)
		{
			link = curNode->GetChild(childCnt);
			if (link == NULL)
			{
				//there are no more children to search, so return NULL
				return NULL;
			}
			else
			{
				//recurse on the child
				link = FindNode(name, link);

				//if we found the node, we are done
				if (link)
				{
					return link;
				}

				//othewise, we will progress through the rest of the children
				childCnt++;
			}


		}
		return NULL; //no child contained node
	}

}

void Skeleton::SetSkelState(double* state)
{
	//go through the state and apply values to each dof
	Link* curLink;
	int stateCnt = 0;//where we are in the state
	for (int i = 0; i < m_linkCnt; i++)
	{
		curLink = m_linkArray[i];
		int jntType = curLink->GetJointType();

		if (jntType == J_FREE)
		{
			//this should be the root
			//It is the only free joint we support in our skeleton
			assert(i == 0);

			//we have two components, a translation followed by a quaternion rotation

			//the translation is that which should be applied to get to the root location
			//of the hip (I think!)
			curLink->SetParTranslation(state[0], state[1], state[2]);

			//now set the rotation
			curLink->SetDOFValues(&state[3]);

			stateCnt += 6;
		}
		else if (jntType == J_EULER_SIX)
		{
			//this should be the root (alternate representation using euler angles instead of quaternions)
			//It is the only free joint we support in our skeleton
			assert(i == 0);

			//we have two components, a translation followed by a quaternion rotation

			//the translation is that which should be applied to get to the root location
			//of the hip (I think!)
			curLink->SetParTranslation(state[0], state[1], state[2]);

			//now set the rotation
			curLink->SetDOFValues(&state[3]);

			stateCnt += 6;
		}
		else if (jntType == J_GIMBAL || jntType == J_PIN || jntType == J_BALL
			|| jntType == J_UNIVERSAL)
		{
			//set the rotation for the link
			curLink->SetDOFValues(&state[stateCnt]);

			//move x forward in state where x is the number of dofs in this link
			stateCnt += curLink->GetNumRotations();
		}
		else if (jntType == J_WELD)
		{
			//skip it!
		}

		else
		{
			std::cerr <<"Don't currently handle joint type " << jntType  << " in KinSkel" << std::endl;
			assert(false);
		}

	}

}

void Skeleton::CalcVertexLocations(int maxEntries, int* curLocation, VERTEX** outCoords)
{
	m_pSkelRoot->CalcVertexLocations(maxEntries, curLocation, outCoords);
}
void Skeleton::AddGeometry()
{
	m_pSkelRoot->CalcParentGeomAndRecurse();
}
void Skeleton::UpdateLinks()
{
	//Start at the root and have the skeleton update itself recursively
	m_pSkelRoot->UpdateAndRecurse(this);

}
