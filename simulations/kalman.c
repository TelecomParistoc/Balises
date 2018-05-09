#include <math.h>
#include <stdlib.h>
#include "kalman.h"

#define  PI 3.14159265358979323846
/* Speed of light in air, in metres per second */
#define SPEED_OF_LIGHT 299702547

void printMatrix(int rows, int columns, float a[][columns]) {
	int i, j;
	for (i=0;i<rows;i++) {
		for (j=0;j<columns;j++) {
			printf("%f,", a[i][j]);
		}
		printf(";");
	}
	printf("\r\n");
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
			b[j][i] = a[i][j];
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

void awgn (const int n, const float mean[n], const float variance[n], const float arrayIn[n], float arrayOut[n]) {
  int    i;
  float x1, x2, tmp, x, xn;

  for (i=0; i<n; i++) {
    arrayOut[i] = arrayIn[i];
  }

  for (i=0; i<n; i++) {
    // x1, x2: uniform random variables in the range [0, 1]
    do
    x1 = (float)rand()/RAND_MAX;
    while (x1 == 0);    // x1 can't be zero
    x2 = (float)rand()/RAND_MAX;

    // x: unit normal random variables, ~N(0, 1)
    // xn: normal random variables, ~N(mean, variance)
    x = sqrt(-2*log(x1)) * cos(2*PI*x2);
    xn = mean[i] + sqrt(variance[i]) * x;

    tmp = arrayIn[i] + xn;   // Add noise to pixel
    if (tmp < 0)
      arrayOut[i] = 0;
    else
      arrayOut[i] = tmp;
  }
}

void initCst() {
  var = 1e010;
  dt = 0.026;
  q = 100000;

  int i;
  for (i=0;i<6;i++) {
    P[i][i] = 1;
    xVect[i][0] = 0;
  }

  Q[0][0] = q*pow(dt,3)/3;
  Q[1][1] = q*pow(dt,3)/3;
  Q[2][2] = q*dt;
  Q[3][3] = q*dt;
  Q[0][2] = Q[2][0] = q*pow(dt,2)/2;
  Q[1][3] = Q[3][1] = q*pow(dt,2)/2;

  // float R[2][2] = {{var/(2*X2*X2),var*(X2-2*X3)/(4*X2*X2*Y3)},{var*(X2-2*X3)/(4*X2*X2*Y3),var*(1+(X3/X2)*(X3/X2-1))/(2*Y3*Y3)}};
  R[0][0] = 747;
  R[1][1] = 1261;

  A[0][0] = 1;
  A[1][1] = 1;
  A[2][2] = 1;
  A[3][3] = 1;
  A[0][2] = dt;
  A[1][3] = dt;

  transposeMatrix(6, 6, A, At);

  H[0][0] = 1;
  H[1][1] = 1;
  transposeMatrix(2, 6, H, Ht);
}

void test() {
  printf("%f\n", q);
}

/* Cholesky-decomposition matrix-inversion code, adapated from
http://jean-pierre.moreau.pagesperso-orange.fr/Cplus/choles_cpp.txt */

static int choldc1(float * a, float * p, int n) {
  int i,j,k;
  float sum;

  for (i = 0; i < n; i++) {
    for (j = i; j < n; j++) {
      sum = a[i*n+j];
      for (k = i - 1; k >= 0; k--) {
        sum -= a[i*n+k] * a[j*n+k];
      }
      if (i == j) {
        if (sum <= 0) {
          return 1; /* error */
        }
        p[i] = sqrt(sum);
      }
      else {
        a[j*n+i] = sum / p[i];
      }
    }
  }

  return 0; /* success */
}

static int choldcsl(float * A, float * a, float * p, int n)
{
  int i,j,k; float sum;
  for (i = 0; i < n; i++)
  for (j = 0; j < n; j++)
  a[i*n+j] = A[i*n+j];
  if (choldc1(a, p, n)) return 1;
  for (i = 0; i < n; i++) {
    a[i*n+i] = 1 / p[i];
    for (j = i + 1; j < n; j++) {
      sum = 0;
      for (k = i; k < j; k++) {
        sum -= a[j*n+k] * a[k*n+i];
      }
      a[j*n+i] = sum / p[j];
    }
  }

  return 0; /* success */
}


static int cholsl(float * A, float * a, float * p, int n)
{
  int i,j,k;
  if (choldcsl(A,a,p,n)) return 1;
  for (i = 0; i < n; i++) {
    for (j = i + 1; j < n; j++) {
      a[i*n+j] = 0.0;
    }
  }
  for (i = 0; i < n; i++) {
    a[i*n+i] *= a[i*n+i];
    for (k = i + 1; k < n; k++) {
      a[i*n+i] += a[k*n+i] * a[k*n+i];
    }
    for (j = i + 1; j < n; j++) {
      for (k = j; k < n; k++) {
        a[i*n+j] += a[k*n+i] * a[k*n+j];
      }
    }
  }
  for (i = 0; i < n; i++) {
    for (j = 0; j < i; j++) {
      a[i*n+j] = a[j*n+i];
    }
  }

  return 0; /* success */
}

static struct point_t LSQ(float d1, float d2, float d3) {
  float A[2][2] = {{2*(X1-X3), 2*(Y1-Y3)}, {2*(X2-X3), 2*(Y2-Y3)}};
  float b[2][1] = {{pow(X1,2)-pow(X3,2)+pow(Y1,2)-pow(Y3,2)+pow(d3,2)-pow(d1,2)}, {pow(X2,2)-pow(X3,2)+pow(Y2,2)-pow(Y3,2)+pow(d3,2)-pow(d2,2)}};

  float At[2][2];
  transposeMatrix(2, 2, A, At);
  float AtA[2][2];
  multiplyMatrices(2, 2, 2, At, A, AtA);
  float Atb[2][1];
  multiplyMatrices(2, 2, 1, At, b, Atb);
  float array[2];
  float inv[2][2];
  cholsl(&AtA[0][0], &inv[0][0], &array[0], 2);
  float xVect[2][1];
  multiplyMatrices(2, 2, 1, inv, Atb, xVect);

  struct point_t ret;
  ret.a = (xVect[0][0] < 4000 && xVect[0][0] > -1000) ? xVect[0][0] : 0;
  ret.b = (xVect[1][0] < 3000 && xVect[1][0] > -1000) ? xVect[1][0] : 0;
  return ret;
}

static const int RSLToRangeBiais[17] = { // mm
  -198,
  -187,
  -179,
  -163,
  -143,
  -127,
  -109,
  -84,
  -59,
  -31,
  0,
  36,
  65,
  84,
  97,
  106,
  110
};

/* see APS011, pages 10-14 */
static void rangeBiais(float * d) {
  const float Pt = -14.3; // dBm
  const int G = 0; // dB, TODO: calibrate gain
  const int fc = 3994; // MHz

  float Pr = Pt + G + 20*log10(SPEED_OF_LIGHT)-20*log10(4*M_PI*fc*(*d)) - 60; // dBm

  int i = 0;
  while ((Pr < -2*i-61) && (-93 < -2*i-61)) {
    i++;
  }
  *d = *d - RSLToRangeBiais[i];
}

struct point_t kalmanIteration(float d1, float d2, float d3) {
  float distances[3] = {d1, d2, d3};
  float distancesWGN[3] = {0, 0, 0};
  float mean[3] = {0, 0, 0};
  float variance[3] = {729, 729, 729};
  awgn(3, mean, variance, distances, distancesWGN);

  // rangeBiais(&distancesWGN[0]);
  // rangeBiais(&distancesWGN[1]);
  // rangeBiais(&distancesWGN[2]);


  struct point_t lsq = LSQ(distancesWGN[0], distancesWGN[1], distancesWGN[2]);

  float input[2][1] = {{lsq.a}, {lsq.b}};

  // Compute current state estimate
  float Xproj[6][1];
  multiplyMatrices(6, 6, 1, A, xVect, Xproj);

  // Compute Pproj
  float Pproj[6][6];
  multiplyMatrices(6, 6, 6, A, P, Pproj);
  float tmp2[6][6];
  multiplyMatrices(6, 6, 6, Pproj, At, tmp2);
  addMatrices(6, 6, tmp2, Q, Pproj, 0);

  // Compute innovation
  float Hx[2][1];
  multiplyMatrices(2, 6, 1, H, Xproj, Hx);
  float yTilde[2][1];
  addMatrices(2, 1, input, Hx, yTilde, 1);

  // Compute S2
  float PH[6][2];
  multiplyMatrices(6, 6, 2, Pproj, Ht, PH);
  float tmp[2][2];
  multiplyMatrices(2, 6, 2, H, PH, tmp);
  float S[2][2];
  addMatrices(2, 2, tmp, R, S, 0);

  // Compute K
  float array[2];
  cholsl(&S[0][0], &tmp[0][0], &array[0], 2);

  float K[6][2];
  multiplyMatrices(6, 2, 2, PH, tmp, K);
  // printMatrix(2,3,K);

	// Compute X
	float tmp3[6][1];
	multiplyMatrices(6, 2, 1, K, yTilde, tmp3);
	addMatrices(6, 1, Xproj, tmp3, xVect, 0);
	// printMatrix(2,1,xVect);

	// Compute P
	multiplyMatrices(6, 2, 6, K, H, tmp2);
	float id[6][6] = {{1,0,0,0,0,0},{0,1,0,0,0,0},{0,0,1,0,0,0},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}};
	addMatrices(6, 6, id, tmp2, tmp2, 1);
	multiplyMatrices(6, 6, 6, tmp2, Pproj, P);
	// printMatrix(2,2,P);

  struct point_t ret;
  ret.a = xVect[0][0];
  ret.b = xVect[1][0];
  return ret;
}
