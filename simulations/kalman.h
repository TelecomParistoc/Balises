#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include <stdio.h>

#define X2 3000
#define X3 1500
#define Y3 2000

extern double P[2][2];
extern double Q[2][2];
extern double R[3][3];

struct point_t {
  double a;
  double b;
};

void printMatrix(int rows, int columns, double a[][columns]);

struct point_t kalmanIteration(double x, double y, double d1, double d2, double d3);

void invert33Matrix(double a[][3], double b[][3]);

void multiplyMatrices(int aRows, int innerDim, int bColumns, double a[][innerDim], double b[][bColumns], double c[][bColumns]);

void transposeMatrix(int rows, int columns, double a[][columns], double b[][rows]);

void addMatrices(int rows, int columns, double a[][columns], double b[][columns], double c[][columns], int subtract);

#endif
