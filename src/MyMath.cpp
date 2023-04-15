#include <memory>
#include"MyMath.h"
#include <assert.h>

void MultMatrices(float c[4][4], float m1[4][4], float m2[4][4])
{
	int	i, j, k;

	for (i = 0; i < 4; i++)
	{

		for (j = 0; j < 4; j++)
		{
			c[i][j] = 0.0;
			for (k = 0; k < 4; k++)
				c[i][j] += m1[i][k] * m2[k][j];
		}
	}
}

void MultMatricesSafe(float c[4][4], float m1[4][4], float m2[4][4])
{
	float r[4][4];
	MultMatrices(r, m1, m2);
	memcpy(c, r, sizeof(float) * 16);
}

void MakeTransMatrix(float x, float y, float z, float m[4][4])
{
	MakeIdentity(m);

	m[0][3] = x;
	m[1][3] = y;
	m[2][3] = z;

}
//type determines whether it is an x (0), y (1) or z (2) rotation
void MakeRotMatrix(int type, float rad, float m[4][4])
{
	MakeIdentity(m);

	int ind1, ind2;

	if (type == 0)
	{
		ind1 = 1;
		ind2 = 2;


	}
	else if (type == 1)
	{
		ind1 = 0;
		ind2 = 2;
		rad = -rad;
	}
	else if (type == 2)
	{
		ind1 = 0;
		ind2 = 1;

	}
	else
	{
		assert(false);
	}

	float c = cos(rad);
	float s = sin(rad);

	m[ind1][ind1] = c;
	m[ind1][ind2] = -s;
	m[ind2][ind1] = s;
	m[ind2][ind2] = c;



}
void MakeIdentity(float c[4][4])
{
	int i, j;
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			if (i == j)
			{
				c[i][j] = 1;
			}
			else
			{
				c[i][j] = 0;
			}
		}
	}

}
void CopyMatrix(float dest[4][4], float src[4][4])
{
	memcpy(dest, src, 16 * sizeof(float));
}

void MultPointByMatrix(float* pOut, float* p, float m[4][4])
{
	int i, j;
	for (i = 0; i < 4; i++)
	{
		pOut[i] = 0;
		for (j = 0; j < 4; j++)
		{
			pOut[i] += m[i][j] * p[j];
		}
	}

}


//This allows pOut and p to be the same vector
void MultPointByMatrixSafe(float* pOut, float* p, float m[4][4])
{

	float pTemp[4];
	MultPointByMatrix(pTemp, p, m);
	memcpy(pOut, pTemp, sizeof(float) * 4);

}