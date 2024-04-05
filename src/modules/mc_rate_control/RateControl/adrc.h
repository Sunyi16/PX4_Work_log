/**
 * @file adrc.h
 *
 * Definition of generic ADRC controller.
 *
 * @author Sunyi
 */

#ifndef ADRC_H_
#define ADRC_H_
#define ABS(X)  (((X)>0)?(X):-(X))
#include <stdint.h>

__BEGIN_DECLS

typedef   signed short     int int16;
typedef unsigned short     int uint16;

typedef struct
{

float x1;
float x2;
float r;
float h;
uint16 N0;

float h0;
float fh;


float z1;
float z2;
float z3;
float e;
float y;
float fe;
float fe1;
float beta_01;
float beta_02;
float beta_03;
float b;



float e0;
float e1;
float e2;
float u0;
float u;
float b0;


float beta_0;
float beta_1;
float beta_2;

float alpha1;
float alpha2;
float zeta;

float h1;
uint16 N1;

float c;

float e2_lpf;

float TD_Input;
float Last_TD_Input;
float TD_Input_Div;

float ESO_Input;
float Last_ESO_Input;
float ESO_Input_Div;
}Fhan_Data;

__EXPORT void ADRC_Init(Fhan_Data *fhan_Input1,Fhan_Data *fhan_Input2,Fhan_Data *fhan_Input3);
__EXPORT void Fhan_ADRC(Fhan_Data *fhan_Input,float expect_ADRC);
__EXPORT float ADRC_Control(Fhan_Data *fhan_Input,float expect_ADRC,float feedback);
__EXPORT void ADRC_Integrate_Reset(Fhan_Data *fhan_Input) ;
extern Fhan_Data ADRC_Pitch_Controller,ADRC_Roll_Controller;

void Advanced_ESO(Fhan_Data *fhan_Input);

__END_DECLS
#endif
