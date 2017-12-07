#include <math.h>

float P[2][2] = {{0, 0}, {0, 0}};
float Q[2][2] = {{1, 0}, {0, 1}};
float R[3][3] = {{211*pow(10, -9), 0, 0}, {0, 211*pow(10, -9), 0}, {0, 0, 211*pow(10, -9)}};
float D[3][1] = {{0}, {0}, {0}};

void kalman() {
	float x, y, z;

	x = (D[0][0]*D[0][0]-D[1][0]*D[1][0]+X2*X2)/(2*X2);
	y = (D[0][0]*D[0][0]-D[2][0]*D[2][0]+X3*X3+Y3*Y3-2*X3*x)/(2*Y3);
	z =	pow(fabs(pow(D[0][0], 2) - pow(x, 2) - pow(y, 2)), 0.5);
	// TODO: check z against its real value to determine incoherent measures

	// Compute Pproj
	float Pproj[2][2];
	addMatrices(2, 2, P, Q, Pproj, 0);

	// Compute H
	float H[3][2];
	float dist[3][1];
	dist[0][0] = pow(pow(radioData.xVect[0][0], 2) + pow(radioData.xVect[1][0], 2), 0.5);
	H[0][0] = radioData.xVect[0][0]/dist[0][0];
	H[0][1] = radioData.xVect[1][0]/dist[0][0];
	dist[1][0] = pow(pow(radioData.xVect[0][0] - X2, 2) + pow(radioData.xVect[1][0], 2), 0.5);
	H[1][0] = (radioData.xVect[0][0] - X2)/dist[1][0];
	H[1][1] = radioData.xVect[1][0]/dist[1][0];
	dist[2][0] = pow(pow(radioData.xVect[0][0] - X3, 2) + pow(radioData.xVect[1][0] - Y3, 2), 0.5);
	H[2][0] = (radioData.xVect[0][0] - X3)/dist[2][0];
	H[2][1] = (radioData.xVect[1][0] - Y3)/dist[2][0];

	// Compute S
	float Hprime[2][3];
	transposeMatrix(3, 2, H, Hprime);
	float PH[2][3];
	multiplyMatrices(2, 2, 3, Pproj, Hprime, PH);
	float tmp[3][3];
	multiplyMatrices(3, 2, 3, H, PH, tmp);
	float S[3][3];
	addMatrices(3, 3, tmp, R, S, 0);

	// Compute K
	invert33Matrix(S, tmp);
	float K[2][3];
	multiplyMatrices(2, 3, 3, PH, tmp, K);

	// Compute X
	float Y[3][1];
	addMatrices(3, 1, D, dist, Y, 1);
	float tmp2[2][1];
	multiplyMatrices(2, 3, 1, K, Y, tmp2);
	addMatrices(2, 1, radioData.xVect, tmp2, radioData.xVect, 0);

	// Compute P
	float tmp3[2][2];
	multiplyMatrices(2, 3, 2, K, H, tmp3);
	float id[2][2] = {{1, 0}, {0, 1}};
	addMatrices(2, 2, id, tmp3, tmp3, 1);
	multiplyMatrices(2, 2, 2, tmp3, Pproj, P);
}

void invert33Matrix(float a[][3], float b[][3]) {
	float det = a[0][0] * (a[1][1] * a[2][2] - a[2][1] * a[1][2]) -
							 a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
							 a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

	b[0][0] = (a[1][1] * a[2][2] - a[2][1] * a[1][2]) / det;
	b[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / det;
	b[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / det;
	b[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) / det;
	b[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / det;
	b[1][2] = (a[1][0] * a[0][2] - a[0][0] * a[1][2]) / det;
	b[2][0] = (a[1][0] * a[2][1] - a[2][0] * a[1][1]) / det;
	b[2][1] = (a[2][0] * a[0][1] - a[0][0] * a[2][1]) / det;
	b[2][2] = (a[0][0] * a[1][1] - a[1][0] * a[0][1]) / det;
}

void multiplyMatrices(int aRows, int innerDim, int bColumns, float a[][innerDim], float b[][bColumns], float c[][bColumns]) {
	int i, j, k;

	//initialize c to 0
	for (i=0;i<aRows;i++) {
		for (j=0;j<bColumns;j++) {
			c[i][j] = 0;
		}
	}

	//compute c
	for (i=0;i<aRows;i++) {
		for (j=0;j<bColumns;j++) {
			for (k=0;k<innerDim;k++) {
				c[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}

void transposeMatrix(int rows, int columns, float a[][columns], float b[][rows]) {
	int i, j;

	for (i=0;i<rows;i++) {
		for (j=0;j<columns;j++) {
			b[i][j] = a[j][i];
		}
	}
}

void addMatrices(int rows, int columns, float a[][columns], float b[][columns], float c[][columns], int subtract) {
	int i, j;

	for (i=0;i<rows;i++) {
		for (j=0;j<columns;j++) {
			if (subtract)
				c[i][j] = a[i][j] - b[i][j];
			else
				c[i][j] = a[i][j] + b[i][j];
		}
	}
}
