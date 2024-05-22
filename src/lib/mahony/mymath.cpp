#include "mymath.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

float function(float x[])
{
    float y = 0.0;
    y = fabs(2.0*(double)(x[1]*x[3]-x[0]*x[2])-1.55) + fabs(2.0*(double)(x[0]*x[1]-x[2]*x[3])-1.85) + fabs((double)(x[0]*x[0]-x[1]*x[1]-x[2]*x[2]+x[3]*x[3])-1.33);
    //y =(x[0] - 1) *(x[0] - 1) + 2 * (x[1] + 2) *(x[1] + 2) + (x[2] - 3) *(x[2] - 3) + (x[3] + 4) *(x[3] + 4);
    return y;
}

float q_function(float x[],float ax,float ay,float az)
{
		float y = 0.0;
    y = fabs(2*(x[1]*x[3]-x[0]*x[2])-ax) + fabs(2*(x[0]*x[1]-x[2]*x[3])-ay) + fabs(x[0]*x[0]-x[1]*x[1]-x[2]*x[2]+x[3]*x[3]-az);
    //y =(x[0] - 1) *(x[0] - 1) + 2 * (x[1] + 2) *(x[1] + 2) + (x[2] - 3) *(x[2] - 3) + (x[3] + 4) *(x[3] + 4);
    return y;

}






float* arr_add(float x[],float y[])
{
    static float arradd[4] = {0,0,0,0};
		int i;
    for(i = 0;i<4;i++)
    {
        arradd[i] = x[i] + y[i];
    }

    return arradd;
}

void arr_add1(float des[],float s1[],float s2[])
{
    int i;
    for(i = 0;i<4;i++)
    {
        des[i] = s1[i] + s2[i];
    }

}

void arr_subtract(float des[],float s1[],float s2[])
{
    int i;
    for(i = 0;i<4;i++)
    {
        des[i] = s1[i] - s2[i];
    }

}


float* num_mul_arr(float x,float y[])
{
    static float arradd[4] = {0,0,0,0};
		int i;
    for(i = 0;i<4;i++)
    {
        arradd[i] =x * y[i];
    }

    return arradd;
}

void arr_norm(float x[])
{
    int i;
    for(i = 0;i<4;i++)
    {
        x[i] =x[i]/sqrtf(pow(x[0],2)+pow(x[1],2)+pow(x[2],2)+pow(x[3],2));
    }

}

float arr_dot(float x[],float y[])
{
    float dot = 0;
		int i;
    for(i = 0;i<4;i++)
    {
        dot += x[i]*y[i];
    }

    return dot;
}






