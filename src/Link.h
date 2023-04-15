#pragma once

#include "defs.h"
#include "linmath.h"

#define KL_MAX_CHILDREN 5
#define MAX_NAME_LEN 80


// types of joints
#define	J_UNDEF	     0
#define	J_PIN	     1
#define	J_BALL	     2
#define	J_CYLINDER   3
#define	J_SLIDER     4
#define	J_FREE	     5
#define	J_GIMBAL     6
#define	J_WELD	     7
#define	J_PLANAR     8
#define	J_BEARING    9
#define	J_UNIVERSAL 10
//This joint has six dofs translation followed by six dofs of rotation as Euler angles
#define J_EULER_SIX 11
#define	J_BUSHING   12

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

class Skeleton;

class Link
{

public:
	Link* GetParent();
	void SetParent(Link* p);

	//adds child to the link 
	//Children will be added at the first open location in the 
	//child array.
	//If the array is full, returns false and does not add node
	//otherwise, returns true.
	bool AddChild(Link* l);
	void RemoveChild(char* name);

	//returns child with the given index number
	//null indicates that there is no such child
	Link* GetChild(int num);

	int GetNumRotations();

	//*******************
	//access functions for link name
	char* GetName();
	void SetName(char* n);

	void GetParTranslation(double v[3]);
	void SetParTranslation(double x, double y, double z);
	void SetAxisOrder(int rotOrder, int axisNum);
	void SetJointType(const char* type);
	int GetJointType();

	void SetDOFValues(double* v);

	//maxEntries is the number of values that you can put in the outCoords array
	//curLocation is the next empty location where you can start adding
	void CalcVertexLocations(int maxEntries, int* curLocation, VERTEX** outCoords);

	//update the transformations
	void UpdateAndRecurse(Skeleton* pSkel);

	//copy matrix to m
	void GetLToWTransMat(mat4x4 m);

	//calculate the geometry used for rendering 
	//currently a pyramid from the parent joint to this joint in the
	//parent frame
	void CalcParentGeomAndRecurse();

	Link();
	virtual ~Link();

private:
	void MakePyramid(float axis[3], VERTEX** outCoords);

	//This function will calculate a quaternion that will align a vector in a given coordinate
	//frame with a target vector in that same coordinate frame.  For instance, the base vector
	//may be the default orientation of an arm and the target vector a given orientation
	//desired for the arm.  
	//The quaternion is returned through quat
	void CalcQuatToAlignWithVector(float baseVec[], float targetVec[], float quat[], float perp[]);
	void TransformVERTEX(mat4x4 tm, VERTEX* vertIn, VERTEX* vertOut);
	void TransformVERTEX_rowMajor(mat4x4 tm, VERTEX* vertIn, VERTEX* vertOut);

	void NormalizeQuaternion(float* e1, float* e2, float* e3, float* e4);
	double AngleBetween(float* v1, float* v2);

	void MakeLinkRotMatrixLocal(mat4x4 rot);


	//link name
	char m_name[MAX_NAME_LEN + 1];

	//children of this node
	Link* m_children[KL_MAX_CHILDREN];

	//parent of this node, NULL if this node is the root
	Link* m_parNde;

	//The parent translation is the location of the joint of this link it its
	//parent coordinates
	float m_parTrans[3];

	//defines the order that the axial rotations should be applied in
	//m_axisOrder[0] is the first rotation to be applied etc.
	//Note, some joint types have less than three axes
	//This is the order of the pins in the SDFast file, so an
	//XZY joint in the system description file would be a 0,2,1 axisorder
	int m_axisOrder[3];

	//number of children attached to this link
	int m_numChildren;

	//what kind of joint is associated with the link
	int m_jointType;


	//The local to world transformation matrix for this link
	mat4x4 m_LToWTrans;

	//hash table used to convert a joint type to a a number of rotational DOFs
	int m_jointTypeToNumRotations[13];

	//these are the values associated with the rotation DOFs of this link
	//They are given in the same order as the axes (i.e. if the z rotation should
	//be performed first, the z dof is the first dof)
	double m_dofValues[3];

	//This contains the geometry used to show the bone going from
	//the parent of this joint's to this joint.
	//This is expressed in the local frame of the parent.
	VERTEX* m_geomFromParent;

};

