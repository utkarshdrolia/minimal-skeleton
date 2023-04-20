#include "Link.h"
#include <string.h>
#include <iostream>
#include "linmath.h"
#include <assert.h>
#include "MyMath.h"

#ifndef EPS
#define EPS 0.0000001
#endif

Link::Link()
{
	int i;
	for (i = 0; i < KL_MAX_CHILDREN; i++) m_children[i] = NULL;


	m_numChildren = 0;
	m_jointType = 0;
	m_parNde = NULL;

	m_axisOrder[0] = X_AXIS;
	m_axisOrder[1] = Y_AXIS;
	m_axisOrder[2] = Z_AXIS;

	m_parTrans[0] = m_parTrans[1] = m_parTrans[2] = 0;


	m_jointTypeToNumRotations[0] = 0;
	m_jointTypeToNumRotations[1] = 1;
	m_jointTypeToNumRotations[2] = 3;
	m_jointTypeToNumRotations[3] = 1;
	m_jointTypeToNumRotations[4] = 0;
	m_jointTypeToNumRotations[5] = 3;
	m_jointTypeToNumRotations[6] = 3;
	m_jointTypeToNumRotations[7] = 0;
	m_jointTypeToNumRotations[8] = 1;
	m_jointTypeToNumRotations[9] = 3;
	m_jointTypeToNumRotations[10] = 2;
	m_jointTypeToNumRotations[11] = 3;
	m_jointTypeToNumRotations[12] = 3;

	m_dofValues[0] = m_dofValues[1] = m_dofValues[2] = 0.0;

	m_geomFromParent = NULL;

}
Link::~Link()
{
	if (m_geomFromParent) delete[] m_geomFromParent;
}

void Link::SetName(char* n)
{
	strncpy(m_name, n, MAX_NAME_LEN);
	m_name[MAX_NAME_LEN] = '\0';
}

char* Link::GetName()
{
	return m_name;
}
//Returns the number of rotations of the joint associated with this link
int Link::GetNumRotations()
{
	return m_jointTypeToNumRotations[m_jointType];
}

//returns child with the given index number
//null indicates that there is no such child
Link* Link::GetChild(int num)
{
	if (num < KL_MAX_CHILDREN)
	{
		return m_children[num];
	}
	else
	{
		return NULL;
	}

}

//adds child to the link 
//Children will be added at the first open location in the 
//child array.
//If the array is full, returns false and does not add node
//otherwise, returns true.
bool Link::AddChild(Link* l)
{
	if (m_numChildren >= KL_MAX_CHILDREN)
	{
		std::cerr << "Cannot add child to kinematic skeleton as joint valence has been met." << std::endl;
		//no room for child
		return false;
	}
	else
	{
		l->SetParent(this);
		m_children[m_numChildren] = l;
		m_numChildren++;
	}
	return true;
}
void Link::RemoveChild(char* name)
{
	for (int i = 0; i < m_numChildren; i++)
	{
		if (0 == strcmp(m_children[i]->GetName(), name))
		{
			m_numChildren--;
			//remove this one
			for (int j = i; j < m_numChildren; j++)
			{
				m_children[j] = m_children[j + 1];
			}
			break;
		}
	}

}
void Link::GetParTranslation(double v[])
{
	v[0] = m_parTrans[0];
	v[1] = m_parTrans[1];
	v[2] = m_parTrans[2];

}
void Link::SetAxisOrder(int rotOrder, int axisNum)
{
	if (rotOrder < 3 && rotOrder >= 0)
	{
		m_axisOrder[rotOrder] = axisNum;
	}
}
void Link::SetParent(Link* p)
{
	m_parNde = p;
}

int Link::GetJointType()
{
	return m_jointType;
}
//The translation is the location of the end of the link in local coordinates
//From the sd/fast file it is -bodyTojoint + child->inbToJoint
void Link::SetParTranslation(double x, double y, double z)
{
	m_parTrans[0] = x;
	m_parTrans[1] = y;
	m_parTrans[2] = z;

}
void Link::SetJointType(const char* type)
{
	const char* types[13] =  { "unknown","pin","ball","cylinder","slider","free",
		"gimbal","weld","planar","bearing","ujoint","eulersix","bushing" };

	int i;
	bool found = false;

	for (i = J_UNDEF; i <= J_BUSHING && (found == false); i++)
	{
		if (strcmp(type, types[i]) == 0)
		{
			m_jointType = i;
			found = true;
		}
	}

}
void Link::SetDOFValues(double* v)
{
	//copy all the dof values.
	//The number of values will depend on the number of axes the associated joint
	//has (1:1).
	for (int i = 0; i < m_jointTypeToNumRotations[m_jointType]; i++)
	{
		m_dofValues[i] = v[i];
	}

}
void Link::GetLToWTransMat(mat4x4 m)
{
	int i, j;
	for (i = 0; i < 4; ++i)
		for (j = 0; j < 4; ++j)
			m[i][j] = m_LToWTrans[i][j];
}

//Sets the local transformation matrix of the link and location of the joint
//based upon the state data
void Link::UpdateAndRecurse(Skeleton* pSkel)
{
	// Calculate the local translation matrix from joint state data
	mat4x4 parTM;
	if (m_parNde == NULL) {
		mat4x4_identity(parTM);
		mat4x4_translate(m_LToWTrans, m_parTrans[0], m_parTrans[1], m_parTrans[2]);
	} else {
		//Parent Transaltion in parTM
		mat4x4_dup(parTM, m_parNde->m_LToWTrans);

		// Calculate local translation matrix
		mat4x4 localTranslationMat;
		mat4x4_translate(localTranslationMat, m_parTrans[0], m_parTrans[1], m_parTrans[2]);

		// Calculate local rotation matrix
		mat4x4 localRotMat;
		mat4x4_identity(localRotMat);
		MakeLinkRotMatrixLocal(localRotMat);

		//Multiply local translation and rotation matrices
		mat4x4 transformationMatrix;
		mat4x4_mul(transformationMatrix, localTranslationMat, localRotMat);

		// Multiply local transformation matrix with parent Transaformation matrix
		mat4x4_mul(m_LToWTrans, parTM, transformationMatrix);
	}
	

	// Recurse over children
	for (int i = 0; i < m_numChildren; i++) {
		m_children[i]->UpdateAndRecurse(pSkel);
	}
}


//maxEntries is the number of values that you can put in the outCoords array
//curLocation is the next empty location where you can start adding
void Link::CalcVertexLocations(int maxEntries, int* curLocation, VERTEX** outCoords)
{
	if (*curLocation + 12 > maxEntries)
	{
		std::cerr << "Exceeded vertex array size" << std::endl;
		return;
	}
	assert(m_geomFromParent);
	if (!m_geomFromParent)
	{
		std::cerr << "Geom from parent is not set.  Be sure to create local geometry before calculating vertex locations" << std::endl;
		
	}



	mat4x4 tm;

	if(m_parNde == NULL) {
		mat4x4_translate(tm, m_parTrans[0], m_parTrans[1], m_parTrans[2]);
	} else {
		m_parNde->GetLToWTransMat(tm);
	}

	
	//TO DO: figure out the correct transformation
	

	//Apply it to all the vertices in the pyramid
	for (int i = 0; i < 12; i++)
	{
		TransformVERTEX(tm, &m_geomFromParent[i], &(*outCoords)[i + *curLocation]);

		(*outCoords)[i + *curLocation].r = m_geomFromParent[i].r;
		(*outCoords)[i + *curLocation].g = m_geomFromParent[i].g;
		(*outCoords)[i + *curLocation].b = m_geomFromParent[i].b;
	}
	*curLocation += 12;


	//TO DO:  Add some more code...
	for (int i = 0; i < m_numChildren; i++) {
		m_children[i]->CalcVertexLocations(maxEntries, curLocation, outCoords);
	}
	
}

void Link::CalcParentGeomAndRecurse()
{
	MakePyramid(m_parTrans, &m_geomFromParent);
	for (int i = 0; i < KL_MAX_CHILDREN; i++)
	{
		if (m_children[i])
		{
			m_children[i]->CalcParentGeomAndRecurse();
		}
	}

}
//This will make a pyramid that is y-up by default, and then rotate it to align with the given vector (axis)
void Link::MakePyramid(float axis[3], VERTEX** outCoords)
{
	//It would be a bit cleaner to set this up with an element array
	VERTEX* out = new VERTEX[12];
	float len = vec3_len(axis);
	float offset = len / 20.0;
	int startInd = 0;
	for (int i = 0; i < 4; i++)
	{
		out[startInd].y = 0;
		out[startInd + 2].y = 0;
		out[startInd + 1].x = 0;
		out[startInd + 1].y = len;
		out[startInd + 1].z = 0;
		out[startInd].r = out[startInd + 2].r = 1.0f;
		out[startInd].g = out[startInd + 2].g = 0.5f;
		out[startInd].b = out[startInd + 2].b = 0.2f;
		out[startInd + 1].r = 1.0f;
		out[startInd + 1].g = 0.f;
		out[startInd + 1].b = 0.f;
		if (i == 0)
		{
			out[startInd].x = offset;
			out[startInd].z = offset;
			out[startInd + 2].x = -offset;
			out[startInd + 2].z = offset;
		}
		else if (i == 1)
		{
			out[startInd].x = offset;
			out[startInd].z = -offset;
			out[startInd + 2].x = offset;
			out[startInd + 2].z = offset;
		}
		else if (i == 2)
		{
			out[startInd].x = -offset;
			out[startInd].z = -offset;
			out[startInd + 2].x = offset;
			out[startInd + 2].z = -offset;
		}
		else
		{
			out[startInd].x = -offset;
			out[startInd].z = offset;
			out[startInd + 2].x = -offset;
			out[startInd + 2].z = -offset;
		}

		startInd += 3;
	}

	//now we need to rotate the geometry to align with the desired axis
	float quat[4];
	float yup[3] = { 0.,1.,0. };
	float perp[3] = { 1,0,0 };
	this->CalcQuatToAlignWithVector(yup, axis, quat, perp);
	mat4x4 tm;
	mat4x4_from_quat(tm, quat);

	for (int i = 0; i < 12; i++)
	{
		TransformVERTEX_rowMajor(tm, &out[i], &out[i]);
	}
	(*outCoords) = out;
}
void Link::TransformVERTEX_rowMajor(mat4x4 tm, VERTEX* vertIn, VERTEX*vertOut)
{
	vec4 in, out;
	in[0] = vertIn->x;
	in[1] = vertIn->y;
	in[2] = vertIn->z;
	in[3] = 1.0;

	mat4x4_mul_vec4(out, tm, in);

	vertOut->x = out[0];
	vertOut->y = out[1];
	vertOut->z = out[2];
}
void Link::TransformVERTEX(mat4x4 tm, VERTEX* vertIn, VERTEX* vertOut)
{

	//TO ADD
	vec4 in, out;
	in[0] = vertIn->x;
	in[1] = vertIn->y;
	in[2] = vertIn->z;
	in[3] = 1.0;

	mat4x4_mul_vec4(out, tm, in);

	vertOut->x = out[0];
	vertOut->y = out[1];
	vertOut->z = out[2];

}
//maintains the rotation and normalizes the rest of the values to make the quat unit length
void Link::NormalizeQuaternion(float* e1, float* e2, float* e3, float* e4)
{
	if (*e4 >= 1.0)
	{
		*e1 = 0;
		*e2 = 0;
		*e3 = 0;
		*e4 = 1.0;
		return;
	}

	double mag = sqrt((*e1 * *e1 + *e2 * *e2 + *e3 * *e3) / (1 - *e4 * *e4));
	*e1 /= mag;
	*e2 /= mag;
	*e3 /= mag;
}

//calculates the angle between two vectors.
//v1 and v2 should both be of length three
//the angleBetween is returned by the function
double Link::AngleBetween(float* v1, float* v2)
{
	double ab = (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) / (vec3_len(v1) * vec3_len(v2));
	if (ab >= 1 - EPS)
	{
		ab = 0;
	}
	else if (ab <= -1 + EPS)
	{
		ab = PI;
	}
	else
	{
		ab = acos(ab);
	}
	return ab;
}

//This function will calculate a quaternion that will align a vector in a given coordinate
//frame with a target vector in that same coordinate frame.  For instance, the base vector
//may be the default orientation of an arm and the target vector a given orientation
//desired for the arm.  
//The quaternion is returned through quat
//perp is perpendiculat to the baseVec
void Link::CalcQuatToAlignWithVector(float baseVec[], float targetVec[], float quat[], float perp[])
{


	//************************************
	//Now calculate the main rotation
	float x, y, z;

	vec3 out;
	vec3_mul_cross(out, baseVec, targetVec);
	x = out[0]; y = out[1]; z = out[2];

	float q2[4];

	if (x * x + y * y + z * z < EPS)
	{	
		//vectors are colinear
		float bet = AngleBetween(baseVec, targetVec);

		if (bet > PI / 2)
		{//vectors are opposite, flip 180
			q2[3] = 0;
			q2[0] = perp[0];
			q2[1] = perp[1];
			q2[2] = perp[2];

		}
		else
		{
			//vectors are aligned, so do nothing.
			q2[0] = 0;
			q2[1] = 0;
			q2[2] = 0;
			q2[3] = 1;
		}
	}
	else
	{
		double theta = acos((baseVec[0] * targetVec[0] + baseVec[1] * targetVec[1] + baseVec[2] * targetVec[2])
			/ (sqrt(baseVec[0] * baseVec[0] + baseVec[1] * baseVec[1] + baseVec[2] * baseVec[2]) * sqrt(targetVec[0] * targetVec[0] + targetVec[1] * targetVec[1] + targetVec[2] * targetVec[2])));

		double halfTheta = theta / 2.0;

		double sinHT = sin(halfTheta);
		double cosHT = cos(halfTheta);

		//scale all the output parameters
		q2[0] = x * sinHT;
		q2[1] = y * sinHT;
		q2[2] = z * sinHT;
		q2[3] = cosHT;



		float qlen = sqrt(q2[0] * q2[0] + q2[1] * q2[1] + q2[2] * q2[2] + q2[3] * q2[3]);
		//normalize
		NormalizeQuaternion(&q2[0], &q2[1], &q2[2], &q2[3]);

	}

	for (int i = 0; i < 4; i++)
		quat[i] = q2[i];


}

//makes a rotation matrix for this joint and stores the result in rot
//The bool forward is used to determine whether to calculate the normal
//matrix or its inverse.  If forward is true, we calculate the normal
//matrix.  If it is false, we calculate the inverse (change signs of rotations
//and reverse order of transformations)  The forward matrix is used when
//traversing the skeleton tree from the root to the extremeties, the reverse
//form is used when moving from the extremeties to the root.
//
void Link::MakeLinkRotMatrixLocal(mat4x4 rot)
{
	//To add
	for(int i = 0; i < m_jointTypeToNumRotations[m_jointType]; i++) {
		switch (m_axisOrder[i])
		{
		case 0:
			mat4x4_rotate(rot, rot, 1.0f, 0.0f, 0.0f, m_dofValues[i]);
			break;
		case 1:
			mat4x4_rotate(rot, rot, 0.0f, 1.0f, 0.0f, m_dofValues[i]);
			break;
		case 2:
			mat4x4_rotate(rot, rot, 0.0f, 0.0f, 1.0f, m_dofValues[i]);
			break;
		
		default:
			break;
		}
	}
}
