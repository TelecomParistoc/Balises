#ifndef KALMAN_H
#define KALMAN_H

#include "ch.h"

extern float P[2][2];
extern float Q[2][2];
extern float R[3][3];
extern float D[3][1];

void kalmanIteration(void);

void invert33Matrix(float a[][3], float b[][3]);

void multiplyMatrices(int aRows, int innerDim, int bColumns, float a[][innerDim], float b[][bColumns], float c[][bColumns]);

void transposeMatrix(int rows, int columns, float a[][columns], float b[][rows]);

void addMatrices(int rows, int columns, float a[][columns], float b[][columns], float c[][columns], int subtract);

#endif
