#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include <stdio.h>

#define X2 3000
#define X3 1500
#define Y3 2000

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
