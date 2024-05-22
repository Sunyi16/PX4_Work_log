#include "math.h"
#include "mymath.h"

#define halfT 0.025f

//声明
float invSqrt(float x) ;	//快速计算 1/Sqrt(x)
float Imu_Update(float gx,float gy,float gz,float ax,float ay,float az,bool k);

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

float Imu_Update(float gx,float gy,float gz,float ax,float ay,float az,bool k)
{

	float r = 0.4;

	int sym;
	float q_nr0[4] = {1, 0, 0, 0};
	float Q[4] = {1, 0, 0, 0};
	float zero[4] = {0, 0, 0, 0};
	float q_nr[4] = {1, 0, 0, 0};
	float q_nr_norm[4] = {1, 0, 0, 0};
	float sd[4][4] = {{1.0, 0,0,0},{0, 1.0,0,0},{0, 0,1.0,0},{0, 0,0,1.0}};
	float delta_0[4] = {1.0, 1.0, 1.0, 1.0};
	float alpha = 2.0;
	float beta = -0.5;
	float epsilon = 0.00001;
	int dim = 4;
	float y11[4] = {0, 0, 0, 0};
	float yk[4] = {0, 0, 0, 0};
	float delta[4] = {1.0, 1.0, 1.0, 1.0};

	// memcpy(y1, xk, sizeof(xk))
	float t[4] = {0, 0, 0, 0};
	float t1 = 0;
	float t2 = 0;
	float p[4] = {0, 0, 0, 0};

	float Pitch,Roll;
	float q0 = 1, q1 = 0, q2 = 0, q3 = 0;		//四元数

	float norm;
	float nr_norm;
	float Q_norm;

	q0 = Q[0];
	q1 = Q[1];
	q2 = Q[2];
	q3 = Q[3];


	//初始化
	arr_add1(q_nr,zero,q_nr0);
	arr_add1(y11,zero,q_nr);
	arr_add1(delta,zero,delta_0);
	for(int j=0;j<dim;j++){
		for(int i=0;i<dim;i++){
			if(i == j){
				sd[j][i] = 1.0;
			}
			else
				sd[j][i] = 0;
		}
	}

	//加速度计测量的重力方向(机体坐标系)
	norm = invSqrt(ax*ax + ay*ay + az*az);			//之前这里写成invSqrt(ax*ax + ay+ay + az*az)是错误的，现在修改过来了
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	while(1)
    	{
	sym = 0;
        float Lambda[4] = {0, 0, 0, 0};
        while(1)
        {
            arr_add1(yk,zero,y11);
            for(int i = 0;i<dim;i++)
            {
                t1 = q_function(arr_add(yk,num_mul_arr(delta[i],sd[i])),ax,ay,az);
                t2 = q_function(yk,ax,ay,az);

                if (t1 < t2)
                {
                    Lambda[i] += delta[i];
                    arr_add1(yk,yk,num_mul_arr(delta[i],sd[i]));
                    delta[i] *= alpha;
                }
                else
                {
                    delta[i] *= beta;

                }
            }

            if(q_function(yk,ax,ay,az) < q_function(y11,ax,ay,az))
            {

                arr_add1(y11,zero,yk);
            }
            else if(q_function(yk,ax,ay,az) < q_function(q_nr,ax,ay,az))
            {

                break;
            }
            else if((fabs(delta[0])<= (double)epsilon)&&(fabs(delta[1])<= (double)epsilon)&&(fabs(delta[2])<= (double)epsilon)&&(fabs(delta[3])<= (double)epsilon))
            {
                sym = 1;
                break;
            }
            else
                arr_add1(y11,zero,yk);
        }

	if(sym==1)
	{
		break;
	}

        if (sqrtf(pow(yk[0]-q_nr[0],2)+pow(yk[1]-q_nr[1],2)+pow(yk[2]-q_nr[2],2)+pow(yk[3]-q_nr[3],2))<= epsilon)
        {

		arr_add1(q_nr,zero,yk);
		break;
        }
        else
        {
            arr_add1(q_nr,zero,yk);
            arr_add1(y11,zero,yk);
            for(int i = 0;i<dim;i++)
            {
                    arr_add1(p,zero,zero);
                    for(int j =i;j<dim;j++)
                    {
                        arr_add1(p,p,num_mul_arr(Lambda[j],sd[j]));
                    }
                    arr_add1(sd[i],zero,p);
            }

            for(int j=1;j<dim;j++)
            {
                arr_add1(t,zero,zero);
                for(int i=0;i<j;i++)
                {
                   arr_add1(t,t,num_mul_arr(arr_dot(sd[i],sd[j])/arr_dot(sd[i],sd[i]),sd[i]));
                }
                arr_subtract(sd[j],sd[j],t);
            }

            for(int i = 0;i<dim;i++)
            {
                arr_norm(sd[i]);
            }
            arr_add1(delta,zero,delta_0);
        }
   	}
	nr_norm = invSqrt(q_nr[0]*q_nr[0] + q_nr[1]*q_nr[1] + q_nr[2]*q_nr[2] + q_nr[3]*q_nr[3]);
	q_nr_norm[0] = q_nr[0] * nr_norm;
	q_nr_norm[1] = q_nr[1] * nr_norm;
	q_nr_norm[2] = q_nr[2] * nr_norm;
	q_nr_norm[3] = q_nr[3] * nr_norm;

/*******************************************************************************/
/*
	//四元数推出的实际重力方向(机体坐标系)
	vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
	//看成3x1的列向量
	ex = (vx-ax);
	ey = (vy-ay);
	ez = (vz-az);


	雅可比矩阵3x4                          转置4x3
	-2*q2  2*q3   -2*q0   2*q1						-2*q2    2*q1   2*q0
	2*q1   2*q0   2*q3    2*q2						2*q3		 2*q0   -2*q1
	2*q0   -2*q1  -2*q2   2*q3						-2*q0		 2*q3   -2*q2
																				2*q1		 2*q2   2*q3

	计算梯度▽f = JT(q).e(q)   4x1
	cg0 = (-2*q2*ex) +  (2*q1*ey) +  (2*q0*ez);
	cg1 = (2*q3*ex)	+	 (2*q0*ey) +  (-2*q1*ez);
	cg2 = (-2*q0*ex)	+	 (2*q3*ey) +  (-2*q2*ez);
	cg3 = (2*q1*ex)	+	 (2*q2*ey) +  (2*q3*ez);

	用公式算出来初始补偿为-1 这里省略
	g0 = ▽f0   d0 = -g0 = -▽f0

*/

/***********************************************************************************************/
	//更新四元数
  	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

	//单位化四元数
  	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  	q0 = q0 * norm;
  	q1 = q1 * norm;
  	q2 = q2 * norm;
  	q3 = q3 * norm;

	Q[0] = (1.0-(double)r)*(double)q0 + (double)r*(double)q_nr_norm[0];
	Q[1] = (1.0-(double)r)*(double)q1 + (double)r*(double)q_nr_norm[1];
	Q[2] = (1.0-(double)r)*(double)q2 + (double)r*(double)q_nr_norm[2];
	Q[3] = (1.0-(double)r)*(double)q3 + (double)r*(double)q_nr_norm[3];

	Q_norm  = invSqrt(Q[0]*Q[0] + Q[1]*Q[1] + Q[2]*Q[2] + Q[3]*Q[3]);
	Q[0] = Q[0]*Q_norm;
	Q[1] = Q[1]*Q_norm;
	Q[2] = Q[2]*Q_norm;
	Q[3] = Q[3]*Q_norm;

	//四元数反解欧拉角
//	Yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f;
	Pitch = -asin(2.f * (Q[1]*Q[3] - Q[0]*Q[2]))* (double)57.3f;	//单位：deg
	Roll = atan2(2.f * Q[2]*Q[3] + 2.f * Q[0]*Q[1], Q[0]*Q[0] - Q[1]*Q[1] - Q[2]*Q[2] + Q[3]*Q[3])* (double)57.3f;

	float att;

	if(k){
		att = Pitch;
	}
	else
		att = Roll;

	return att;
}
