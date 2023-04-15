#pragma once
#include <fstream>

class Skeleton;
class AnimRec;

class BVHReader
{

public:
	void BuildSkelFromHeader(std::ifstream& file, Skeleton* newSkel, AnimRec* pAnimRec, bool inToM);

};

