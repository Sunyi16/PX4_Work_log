/**
 * @file adrc.c
 *
 * Implementation of generic ADRC controller.
 *
 * @author sunyi
 */

#include "adrc.h"
#include <math.h>
#include <px4_platform_common/defines.h>

const float ADRC_Unit[4][16]=
{
  /*TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO           扰动补偿     非线性组合*/
  /*  r     h      N                  beta_01   beta_02    beta_03     b0     beta_0  beta_1     beta_2     N1     C    alpha1  alpha2  zeta       b*/
  {1000000 ,0.005 , 5,               300,      10000,      100000,      4,    0.85,   0.09,      0.001,       2,    5,    0.9,   1.2,    0.03,    0.1},
  {1000000 ,0.005 , 5,               300,      10000,      100000,      4,    0.85,   0.09,      0.001,       2,    5,    0.9,   1.2,    0.03,    0.1},
  {300000  ,0.005 , 3,               300,      4000,      10000,     100,   0.2,    0.5,      0.0010,       5,      5,    0.8,   1.5,    0.03,    0.05},
};


float Constrain_Float(float amt, float low, float high){
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

int16_t Sign_ADRC(float Input)
{
  int16_t output=0;
  if((double)Input>1E-6) output=1;
  else if((double)Input<-1E-6) output=-1;
  else output=0;
  return output;
}

int16_t Fsg_ADRC(float x,float d)
{
  int16_t output=0;
  output=(Sign_ADRC(x+d)-Sign_ADRC(x-d))/2;
  return output;
}

//参数初始化
void ADRC_Init(Fhan_Data *fhan_Input1,Fhan_Data *fhan_Input2,Fhan_Data *fhan_Input3)
{
  fhan_Input1->r=ADRC_Unit[0][0];
  fhan_Input1->h=ADRC_Unit[0][1];
  fhan_Input1->N0=(uint16)(ADRC_Unit[0][2]);
  fhan_Input1->beta_01=ADRC_Unit[0][3];
  fhan_Input1->beta_02=ADRC_Unit[0][4];
  fhan_Input1->beta_03=ADRC_Unit[0][5];
  fhan_Input1->b0=ADRC_Unit[0][6];
  fhan_Input1->beta_0=ADRC_Unit[0][7];
  fhan_Input1->beta_1=ADRC_Unit[0][8];
  fhan_Input1->beta_2=ADRC_Unit[0][9];
  fhan_Input1->N1=(uint16)(ADRC_Unit[0][10]);
  fhan_Input1->c=ADRC_Unit[0][11];

  fhan_Input1->alpha1=ADRC_Unit[0][12];
  fhan_Input1->alpha2=ADRC_Unit[0][13];
  fhan_Input1->zeta=ADRC_Unit[0][14];
  fhan_Input1->b=ADRC_Unit[0][15];

  fhan_Input2->r=ADRC_Unit[1][0];
  fhan_Input2->h=ADRC_Unit[1][1];
  fhan_Input2->N0=(uint16)(ADRC_Unit[1][2]);
  fhan_Input2->beta_01=ADRC_Unit[1][3];
  fhan_Input2->beta_02=ADRC_Unit[1][4];
  fhan_Input2->beta_03=ADRC_Unit[1][5];
  fhan_Input2->b0=ADRC_Unit[1][6];
  fhan_Input2->beta_0=ADRC_Unit[1][7];
  fhan_Input2->beta_1=ADRC_Unit[1][8];
  fhan_Input2->beta_2=ADRC_Unit[1][9];
  fhan_Input2->N1=(uint16)(ADRC_Unit[1][10]);
  fhan_Input2->c=ADRC_Unit[1][11];

  fhan_Input2->alpha1=ADRC_Unit[1][12];
  fhan_Input2->alpha2=ADRC_Unit[1][13];
  fhan_Input2->zeta=ADRC_Unit[1][14];
  fhan_Input2->b=ADRC_Unit[1][15];

  fhan_Input3->r=ADRC_Unit[2][0];
  fhan_Input3->h=ADRC_Unit[2][1];
  fhan_Input3->N0=(uint16)(ADRC_Unit[2][2]);
  fhan_Input3->beta_01=ADRC_Unit[2][3];
  fhan_Input3->beta_02=ADRC_Unit[2][4];
  fhan_Input3->beta_03=ADRC_Unit[2][5];
  fhan_Input3->b0=ADRC_Unit[2][6];
  fhan_Input3->beta_0=ADRC_Unit[2][7];
  fhan_Input3->beta_1=ADRC_Unit[2][8];
  fhan_Input3->beta_2=ADRC_Unit[2][9];
  fhan_Input3->N1=(uint16)(ADRC_Unit[2][10]);
  fhan_Input3->c=ADRC_Unit[2][11];

  fhan_Input3->alpha1=ADRC_Unit[2][12];
  fhan_Input3->alpha2=ADRC_Unit[2][13];
  fhan_Input3->zeta=ADRC_Unit[2][14];
  fhan_Input3->b=ADRC_Unit[2][15];
}

//ADRC最速跟踪微分器TD
void Fhan_ADRC(Fhan_Data *fhan_Input,float expect_ADRC)
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float x1_delta=0;
  x1_delta=fhan_Input->x1-expect_ADRC;
  fhan_Input->h0=fhan_Input->N0*fhan_Input->h;
  d=fhan_Input->r*fhan_Input->h0*fhan_Input->h0;
  a0=fhan_Input->h0*fhan_Input->x2;
  y=x1_delta+a0;
  a1=sqrt(d*(d+8*ABS(y)));
  a2=a0+Sign_ADRC(y)*(a1-d)/2;
  a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));
  fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_ADRC(a,d)
    -fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));
  fhan_Input->x1+=fhan_Input->h*fhan_Input->x2;
  fhan_Input->x2+=fhan_Input->h*fhan_Input->fh;
}

//原点附近有连线性段的连续幂次函数
float Fal_ADRC(float e,float alpha,float zeta)
{
  int16 s=0;
  float fal_output=0;
  s=(Sign_ADRC(e+zeta)-Sign_ADRC(e-zeta))/2;
  fal_output=e*s/(powf(zeta,1-alpha))+powf(ABS(e),alpha)*Sign_ADRC(e)*(1-s);
  return fal_output;
}

/************扩张状态观测器********************/
//状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
void ESO_ADRC(Fhan_Data *fhan_Input)
{
  fhan_Input->e=fhan_Input->z1-fhan_Input->y;

  fhan_Input->fe=Fal_ADRC(fhan_Input->e,0.5,fhan_Input->N1*fhan_Input->h);
  fhan_Input->fe1=Fal_ADRC(fhan_Input->e,0.25,fhan_Input->N1*fhan_Input->h);

  fhan_Input->z1+=fhan_Input->h*(fhan_Input->z2-fhan_Input->beta_01*fhan_Input->e);
  fhan_Input->z2+=fhan_Input->h*(fhan_Input->z3
                                 -fhan_Input->beta_02*fhan_Input->fe
                                   +fhan_Input->b0*fhan_Input->u);

  //ESO估计状态加速度信号，进行扰动补偿，传统MEMS陀螺仪漂移较大，估计会产生漂移
  fhan_Input->z3+=fhan_Input->h*(-fhan_Input->beta_03*fhan_Input->fe1);
}

void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
{
  float temp_e2=0;
  temp_e2=Constrain_Float(fhan_Input->e2_lpf,-3000,3000);
  fhan_Input->u0=fhan_Input->beta_1*Fal_ADRC(fhan_Input->e1,fhan_Input->alpha1,fhan_Input->zeta)
    +fhan_Input->beta_2*Fal_ADRC(temp_e2,fhan_Input->alpha2,fhan_Input->zeta);

}

//控制器主函数
float ADRC_Control(Fhan_Data *fhan_Input,float expect_ADRC,float feedback_ADRC)
{
  fhan_Input->Last_TD_Input=fhan_Input->TD_Input;
  fhan_Input->TD_Input=expect_ADRC;
  fhan_Input->TD_Input_Div=(fhan_Input->TD_Input-fhan_Input->Last_TD_Input)/fhan_Input->h;

  fhan_Input->Last_ESO_Input=fhan_Input->ESO_Input;
  fhan_Input->ESO_Input=feedback_ADRC;
  fhan_Input->ESO_Input_Div=(fhan_Input->ESO_Input-fhan_Input->Last_ESO_Input)/fhan_Input->h;

  Fhan_ADRC(fhan_Input,expect_ADRC);

  fhan_Input->y=feedback_ADRC;

  ESO_ADRC(fhan_Input);//低成本MEMS会产生漂移，扩展出来的z3此项会漂移。

  fhan_Input->e0+=fhan_Input->e1*fhan_Input->h;
  fhan_Input->e1=fhan_Input->x1-fhan_Input->z1;
  fhan_Input->e2=fhan_Input->x2-fhan_Input->z2;

  Nolinear_Conbination_ADRC(fhan_Input);

  /**********扰动补偿*******/
  fhan_Input->u=fhan_Input->u0
    -fhan_Input->z3/fhan_Input->b0;


  //由于MEMS传感器漂移比较严重，当beta_03取值比较大时，长时间z3漂移比较大，目前不加入扰动补偿控制量
  fhan_Input->u=Constrain_Float(fhan_Input->u0,-450,450);//不加入扰动补偿
  //fhan_Input->u=Constrain_Float(fhan_Input->u,-450,450);//加入扰动补偿
  return fhan_Input->u;
}

void ADRC_Integrate_Reset(Fhan_Data *fhan_Input)  {fhan_Input->e0=0.0f;}
