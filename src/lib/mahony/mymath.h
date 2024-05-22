#ifndef __MYMATH_H
#define __MYMATH_H


float function(float x[]);

float q_function(float x[],float ax,float ay,float az);

float* arr_add(float x[],float y[]);


void arr_add1(float des[],float s1[],float s2[]);


void arr_subtract(float des[],float s1[],float s2[]);


float* num_mul_arr(float x,float y[]);

void arr_norm(float x[]);


float arr_dot(float x[],float y[]);

#endif  /*__MPU6050*/
