
#include "AnimRec.h"
#include "defs.h"

#include <assert.h>


double ToRadians(double deg)
{
	return (deg / 180.0) * PI;

}

AnimRec::AnimRec(void)
{
	m_numDOFs = 0;
	m_frameTime = 0;
}

AnimRec::~AnimRec(void)
{
}

void AnimRec::SetFrameTime(float f)
{
	m_frameTime = f;
}
float AnimRec::GetFrameTime()
{
	return m_frameTime;
}
void AnimRec::SetNumDOFs(int n)
{
	m_numDOFs = n;
}
int AnimRec::GetNumDOFs()
{
	return m_numDOFs;
}
//inToM: inch to metre
void AnimRec::StoreLine(char * line, bool inToM)
{
	float * data = new float[m_numDOFs];
//	double * doubDat = new double[m_numDOFs];//need for quaternion conversion code

	int index = 0;
	char* str = NULL;
	str = strtok(line, " \t");
	// clean up any line feeds or carriage returns
	while (str != NULL && str[0] != 13)
	{
		double val = atof(str);

		data[index] = val;
//		doubDat[index] = data[index];

		if(inToM&&index<3) data[index]*=.0254;

		//Some code relies on this being radians, some expects the root and shoulders to be quats.  I'll store it as radians.
		if(index>=3) data[index] = ToRadians(data[index]);

		index++;


		str = strtok(NULL, " \t");
	}

	m_animData.push_back(data);


}
int AnimRec::GetNumFrames()
{
	return m_animData.size();
}

void AnimRec::GetFrame(int index, double * val)
{

	int size = m_animData.size();
	assert(index<size);

	if(index>=size)
	{
		return;
	}

	float * data;
	data = m_animData.at(index);

	for(int i =0; i<m_numDOFs; i++)
	{
		val[i] = data[i];
	}
}

bool AnimRec::Interpolate(double time, double* val)
{
	int startFrame = time/m_frameTime;
	double weight = time/m_frameTime - startFrame;

	float * low, *high;
	int size = m_animData.size();
	if(startFrame<size)
	{
		low = m_animData.at(startFrame);
	}
	else
	{
		return false;
	}
	if(startFrame+1<size)
	{
		high = m_animData.at(startFrame+1);
	}
	else
	{
		high = low;
	}

	for(int i =0; i<m_numDOFs; i++)
	{
		val[i] = (1-weight) * low[i] + weight * high[i];
		//check for conditions where the set of euler angles flips between subsequent frames
		//e.g. an angle might go from near 180 to near -180.  Blade is bad for this.


		if(fabs(low[i] - high[i]) > 6 )//in radians
		{
			float change = 0;
//			if(fabs(low[i] - high[i]) > 6)
//			{
				change = 2*PI;
//			}
//			else
//			{
//				change = PI; // this case seems to come up, I presume when the other angles are different.
//			}

			float useLow, useHigh;
			useLow = low[i];
			useHigh = high[i];

			if(low[i] < 0 && high[i] > 0 || low[i] < -3)
			{
				useLow = low[i] + change;
				useHigh = high[i];
			}
			else if(low[i] > 0 && high[i] < 0 || high[i] < -3)
			{
				useLow = low[i] ;
				useHigh = high[i] + change;
			}
			val[i] = (1-weight) * useLow + weight * useHigh;

		}
	}
}

double AnimRec::GetStartTime()
{
	return 0;
}
double AnimRec::GetEndTime()
{
	return m_frameTime*m_animData.size();
}