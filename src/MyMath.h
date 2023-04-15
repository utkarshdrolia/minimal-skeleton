#pragma once
void MultMatrices(float c[4][4], float m1[4][4], float m2[4][4]);

void MultMatricesSafe(float c[4][4], float m1[4][4], float m2[4][4]);

void MakeTransMatrix(float x, float y, float z, float m[4][4]);

//type determines whether it is an x (0), y (1) or z (2) rotation
void MakeRotMatrix(int type, float rad, float m[4][4]);

void MakeIdentity(float c[4][4]);

void CopyMatrix(float dest[4][4], float src[4][4]);

void MultPointByMatrix(float* pOut, float* p, float m[4][4]);


//This allows pOut and p to be the same vector
void MultPointByMatrixSafe(float* pOut, float* p, float m[4][4]);
