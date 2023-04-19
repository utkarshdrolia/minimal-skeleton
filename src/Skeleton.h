#pragma once

#include "defs.h"

#define MAX_NUM_LINKS 100

class Link;
class AnimRec;

class Skeleton
{
public:
	//adds link addMe to the tree that defines the skeleton such that its
	//parent in the tree is called parentName.  Note:  $ground is the parent
	//of the root node.
	//Returns true if node can be added, false otherwise
	bool AddToSkeleton(Link* addMe, char* parentName);

	//Searches the tree for a node with the given name
	//Returns a pointer to the node if it finds it, otherwise returns null
	//The search is started at curNode
	Link* FindNode(char* name, Link* curNode);

	//Create a skeleton based on the pre-amble of a bvh file
	void CreateSkeletonFromBVH(char* filename, AnimRec* pAnimRec, bool inToM);

	void SetSkelState(double* state);

	//maxEntries is the number of values that you can put in the outCoords array
	//curLocation is the next empty location where you can start adding
	void CalcVertexLocations(int maxEntries, int* curLocation, VERTEX** outCoords);

	//recalculate all the transformations with the current joint data
	void UpdateLinks();

	void AddGeometry();

	Skeleton();
	~Skeleton();

private:
	//root of the skeleton
	Link* m_pSkelRoot;

	//an array of all the links that make up the skeleton.
	//These are added as they are added to the skeleton.
	Link* m_linkArray[MAX_NUM_LINKS];
//DO I NEED THIS?

	//number of links in array
	int m_linkCnt;


};

