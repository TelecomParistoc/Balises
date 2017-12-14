#include <math.h>
#include "kalman.h"

double P[2][2] = {{0, 0}, {0, 0}};
double Q[2][2] = {{22, 0}, {0, 22}};
double R[3][3] = {{211*pow(10, -3), 0, 0}, {0, 211*pow(10, -3), 0}, {0, 0, 211*pow(10, -3)}};

struct point_t kalmanIteration(double a, double b, double d1, double d2, double d3) {
  double xVect[2][1] = {{a}, {b}};
  double D[3][1] = {{d1}, {d2}, {d3}};
	double x, y, z;

	x = (D[0][0]*D[0][0]-D[1][0]*D[1][0]+X2*X2)/(2*X2);
	y = (D[0][0]*D[0][0]-D[2][0]*D[2][0]+X3*X3+Y3*Y3-2*X3*x)/(2*Y3);
	z =	pow(fabs(pow(D[0][0], 2) - pow(x, 2) - pow(y, 2)), 0.5);
	// TODO: check z against its real value to determine incoherent measures

	// Compute Pproj
	double Pproj[2][2];
	addMatrices(2, 2, P, Q, Pproj, 0);

	// Compute H
	double H[3][2];
	double dist[3][1];
	dist[0][0] = pow(pow(xVect[0][0], 2) + pow(xVect[1][0], 2), 0.5);
	H[0][0] = xVect[0][0]/dist[0][0];
	H[0][1] = xVect[1][0]/dist[0][0];
	dist[1][0] = pow(pow(xVect[0][0] - X2, 2) + pow(xVect[1][0], 2), 0.5);
	H[1][0] = (xVect[0][0] - X2)/dist[1][0];
	H[1][1] = xVect[1][0]/dist[1][0];
	dist[2][0] = pow(pow(xVect[0][0] - X3, 2) + pow(xVect[1][0] - Y3, 2), 0.5);
	H[2][0] = (xVect[0][0] - X3)/dist[2][0];
	H[2][1] = (xVect[1][0] - Y3)/dist[2][0];

	// Compute S
	double Hprime[2][3];
	transposeMatrix(3, 2, H, Hprime);
	double PH[2][3];
	multiplyMatrices(2, 2, 3, Pproj, Hprime, PH);
	double tmp[3][3];
	multiplyMatrices(3, 2, 3, H, PH, tmp);
	double S[3][3];
	addMatrices(3, 3, tmp, R, S, 0);

	// Compute K
	invert33Matrix(S, tmp);
	double K[2][3];
	multiplyMatrices(2, 3, 3, PH, tmp, K);
	// printMatrix(2,3,K);

	// Compute X
	double Y[3][1];
	addMatrices(3, 1, D, dist, Y, 1);
	double tmp2[2][1];
	multiplyMatrices(2, 3, 1, K, Y, tmp2);
	addMatrices(2, 1, xVect, tmp2, xVect, 0);
	// printMatrix(2,1,xVect);

	// Compute P
	double tmp3[2][2];
	multiplyMatrices(2, 3, 2, K, H, tmp3);
	double id[2][2] = {{1, 0}, {0, 1}};
	addMatrices(2, 2, id, tmp3, tmp3, 1);
	multiplyMatrices(2, 2, 2, tmp3, Pproj, P);
	// printMatrix(2,2,P);

  struct point_t ret;
  ret.a = xVect[0][0];
  ret.b = xVect[1][0];
  return ret;
}

void printMatrix(int rows, int columns, double a[][columns]) {
	int i, j;
	for (i=0;i<rows;i++) {
		for (j=0;j<columns;j++) {
			printf("%f,", a[i][j]);
		}
		printf(";");
	}
	printf("\r\n");
}

void invert33Matrix(double a[][3], double b[][3]) {
	double det = a[0][0] * (a[1][1] * a[2][2] - a[2][1] * a[1][2]) -
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

void multiplyMatrices(int aRows, int innerDim, int bColumns, double a[][innerDim], double b[][bColumns], double c[][bColumns]) {
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

void transposeMatrix(int rows, int columns, double a[][columns], double b[][rows]) {
	int i, j;

	for (i=0;i<rows;i++) {
		for (j=0;j<columns;j++) {
			b[j][i] = a[i][j];
		}
	}
}

void addMatrices(int rows, int columns, double a[][columns], double b[][columns], double c[][columns], int subtract) {
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
