#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include <stdio.h>

#define X1 (-60)
#define Y1 1000
#define X2 3060
#define Y2 1950
#define X3 3060
#define Y3 50

float var;
float dt;
float q;
float P[6][6];
float Q[6][6];
float R[2][2];
float A[6][6];
float At[6][6];
float H[2][6];
float Ht[6][2];
float xVect[6][1];

struct point_t {
  float a;
  float b;
};

void initCst(void);
struct point_t kalmanIteration(float d1, float d2, float d3);

#endif
