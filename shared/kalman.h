#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include <stdio.h>

extern float var;
extern float dt;
extern float q;
extern float P[6][6];
extern float Q[6][6];
extern float R[2][2];
extern float A[6][6];
extern float At[6][6];
extern float H[2][6];
extern float Ht[6][2];
extern float xVect[6][1];

void initKalmanCst(void);
void kalmanIteration(float d1, float d2, float d3);

float rangeBiais(float d);

void transposeMatrix(int rows, int columns, float a[][columns], float b[][rows]);
void multiplyMatrices(int aRows, int innerDim, int bColumns, float a[][innerDim], float b[][bColumns], float c[][bColumns]);

#endif
