#pragma once


#include <vector>

class AnimRec
{
public:
	AnimRec(void);
	~AnimRec(void);

	void SetFrameTime(float f);
	float GetFrameTime();
	void StoreLine(char * line, bool inToM);
	void SetNumDOFs(int n);
	int GetNumDOFs();

	double GetStartTime();
	double GetEndTime();

	bool Interpolate(double time, double* val);

	int GetNumFrames();
	void GetFrame(int index, double * val);

private:

	std::vector<float *> m_animData;
	float m_frameTime;
	int m_numDOFs;

};
