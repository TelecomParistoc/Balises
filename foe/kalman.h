#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include <stdio.h>

#define X2 3000
#define X3 1500
#define Y3 2000

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

#endif
