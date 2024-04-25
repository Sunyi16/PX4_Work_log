#include "dcm.hpp"

//限幅函数
Vector3f Constrain_Vector3f(Vector3f amt, float low, float high){
	Vector3f a;
	for (int i=0; i<3; i++){
		a(i) = ((amt(i))<(low)?(low):((amt(i))>(high)?(high):(amt(i))));
	}
  	return a;
}

//求矩阵转置
Dcmf matrix_t(Dcmf b_matrix)

//	a_matrix:转置后的矩阵
//	b_matrix:转置前的矩阵
//	krow    :行数
//	kline   :列数

{
	int k, k2;
	Dcmf a_matrix;

	for (k = 0; k < 3; k++)
	{
		for(k2 = 0; k2 < 3; k2++)
		{
			a_matrix(k2,k) = b_matrix(k,k2);
		}
	}
	return a_matrix;
}

//矩阵重规范化
Dcmf dcm_1(Dcmf a){
	Dcmf b;
	float xy_error = a(0,0)*a(1,0)+a(0,1)*a(1,1)+a(0,2)*a(1,2);
	float x[3] = {a(0,0)-a(1,0)*xy_error/2,a(0,1)-a(1,1)*xy_error/2,a(0,2)-a(1,2)*xy_error/2};
	float x_all = sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
	float y[3] = {a(1,0)-a(0,0)*xy_error/2,a(1,1)-a(0,1)*xy_error/2,a(1,2)-a(0,2)*xy_error/2};
	float y_all = sqrt(y[0]*y[0]+y[1]*y[1]+y[2]*y[2]);
	float z[3] = {x[1]*y[2]-y[1]*x[2],y[0]*x[2]-x[0]*y[2],x[0]*y[1]-x[1]*y[0]};
	float z_all = sqrt(z[0]*z[0]+z[1]*z[1]+z[2]*z[2]);
	b(0,0)= x[0]/x_all;
	b(0,1)= x[1]/x_all;
	b(0,2)= x[2]/x_all;

	b(1,0)= y[0]/y_all;
	b(1,1)= y[1]/y_all;
	b(1,2)= y[2]/y_all;

	b(2,0)= z[0]/z_all;
	b(2,1)= z[1]/z_all;
	b(2,2)= z[2]/z_all;

	return b;
}

//矩阵求行列式
float dcm_abs(Dcmf a){
	float sum1=a(0,2)*a(1,0)*a(2,1)+a(0,1)*a(1,2)*a(2,0)+a(0,0)*a(1,1)*a(2,2);
	float sum2=a(2,2)*a(0,1)*a(1,0)+a(2,1)*a(1,2)*a(0,0)+a(0,2)*a(1,1)*a(2,0);

	float sum=sum1-sum2;
	return sum;

}

//向量的绝对值
/* float vec_abs(Vector3f a){
	float sum = sqrt(a(0)*a(0)+a(1)*a(1)+a(2)*a(2));
	return sum;
} */

Vector3f vec_abs(Vector3f a, float d){
	Vector3f sum;
	for (int i=0;i<3;i++){
		sum(i) = sqrt(d*(d+8*abs(a(i))));
	}
	return sum;
}

//获取向量的符号
Vector3f vec_sign(Vector3f a){
	Vector3f b;
	for (int i=0; i<3; i++){
	if(a(i)<0){
		b(i) = -1;
	}
	else if(a(i)>0){
		b(i) = 1;
	}
	else{
		b(i) = 0;
	}

	}
	return b;
}


//矩阵到向量
Vector3f vee(Dcmf a){
	Vector3f b;
	b(0) = (a(2,1) - a(1,2))/2;
	b(1) = (a(0,2) - a(2,0))/2;
	b(2) = (a(1,0) - a(0,1))/2;
	return b;
}

//向量到矩阵
Dcmf wedge(Vector3f a){
	Dcmf b;
	b(0,0) = 0;
	b(0,1) = -a(2);
	b(0,2) = a(1);
	b(1,0) = a(2);
	b(1,1) = 0;
	b(1,2) = -a(0);
	b(2,0) = -a(1);
	b(2,1) = a(0);
	b(2,2) = 0;
	return b;
}

//矩阵加减
Dcmf matrix_a(Dcmf b_matrix, Dcmf c_matrix, int ktrl)

//	a_matrix=b_matrix+c_matrix
//	 ktrl   :大于0: 加法  不大于0:减法

{
	int k, k2;
	Dcmf a_matrix;

	for (k = 0; k < 3; k++)
	{
		for(k2 = 0; k2 < 3; k2++)
		{
			a_matrix(k,k2) = b_matrix(k,k2)
				+ ((ktrl > 0) ? c_matrix(k,k2) : -c_matrix(k,k2));
		}
	}
	return a_matrix;
}

//矩阵乘矩阵
Dcmf dcm_dcm(Dcmf a, Dcmf b){
	Dcmf c;
	c(0,0) = a(0,0)*b(0,0) + a(0,1)*b(1,0) + a(0,2)*b(2,0);
	c(0,1) = a(0,0)*b(0,1) + a(0,1)*b(1,1) + a(0,2)*b(2,1);
	c(0,2) = a(0,0)*b(0,2) + a(0,1)*b(1,2) + a(0,2)*b(2,2);
	c(1,0) = a(1,0)*b(0,0) + a(1,1)*b(1,0) + a(1,2)*b(2,0);
	c(1,1) = a(1,0)*b(0,1) + a(1,1)*b(1,1) + a(1,2)*b(2,1);
	c(1,2) = a(1,0)*b(0,2) + a(1,1)*b(1,2) + a(1,2)*b(2,2);
	c(2,0) = a(2,0)*b(0,0) + a(2,1)*b(1,0) + a(2,2)*b(2,0);
	c(2,1) = a(2,0)*b(0,1) + a(2,1)*b(1,1) + a(2,2)*b(2,1);
	c(2,2) = a(2,0)*b(0,2) + a(2,1)*b(1,2) + a(2,2)*b(2,2);

	return c;

}

//矩阵乘矩阵(逐元素相乘)
Dcmf dcm_dcm_t(Dcmf a, Dcmf b){
	Dcmf c;
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			c(i,j) = a(i,j)*b(i,j);
		}
	}

	return c;

}

//矩阵乘向量
Vector3f dcm_vec(Dcmf a, Vector3f b){
	Vector3f c;
	for(int i=0; i<3; i++){
		c(i) = a(i,0)*b(0) + a(i,1)*b(1) + a(i,2)*b(2);

	}
	return c;
}

//数值乘矩阵
Dcmf num_dcm(float a, Dcmf b){
	Dcmf c;
	for (int i=0; i<3; i++){
		for (int j =0; j<3; j++){
			c(i,j) = b(i,j)* a;
		}
	}
	return c;
}

//数值乘向量
Vector3f num_vec(float a, Vector3f b){
	Vector3f c;
	for(int i=0; i<3; i++){
		c(i) = a*b(i);
	}
	return c;
}

//矩阵加向量
Dcmf dcm_add_vec(Dcmf a, Vector3f b){
	Dcmf c;
	for (int i=0; i<3; i++){
		c(0,i) =a(0,i) + b(0);
		c(1,i) =a(1,i) + b(1);
		c(2,i) =a(2,i) + b(2);
	}
	return c;
}

//向量运算

/* 说明：两个向量的加法
** param[0]：向量A
** param[1]：向量B
** param[2]：长度
** output：向量
*/
Vector3f Vector3fAdd(Vector3f vecA, Vector3f vecB) {
    Vector3f C;
    for (int i = 0; i<3; i++) {
        C(i) = vecA(i) + vecB(i);
    }
    return C;
}

/* 说明：两个向量的减法
** param[0]：向量A
** param[1]：向量B
** param[2]：长度
** output：向量
*/
Vector3f Vector3fjian(Vector3f vecA, Vector3f vecB) {
    Vector3f C;
    for (int i = 0; i<3; i++) {
        C(i) = vecA(i) - vecB(i);
    }
    return C;
}

//向量加数字
Vector3f Vector3fAdd_num(Vector3f vecA, float a) {
    Vector3f C;
    for (int i = 0; i<3; i++) {
        C(i) = vecA(i) + a;
    }
    return C;
}

//向量加数字
Vector3f Vector3fjian_num(Vector3f vecA, float a) {
    Vector3f C;
    for (int i = 0; i<3; i++) {
        C(i) = vecA(i) - a;
    }
    return C;
}

//fsg
Vector3f fsg(Vector3f a, float b){
	Vector3f c;
	c = num_vec(0.5f,Vector3fjian(vec_sign(Vector3fAdd_num(a,b)),vec_sign(Vector3fjian_num(a,b))));
	return c;
}

//数字减向量
Vector3f num_jian_vec(float a, Vector3f b){
	Vector3f c;
	for(int i=0; i<3; i++){
		c(i) = a-b(i);
	}
	return c;
}

//向量.*向量
Vector3f vec_vec(Vector3f a, Vector3f b){
	Vector3f c;
	for(int i=0; i<3; i++){
		c(i) = a(i)*b(i);
	}
	return c;
}

//向量取反
Vector3f vec_fan(Vector3f a){
	Vector3f c;
	for(int i=0; i<3; i++){
		c(i) = -a(i);
	}
	return c;
}

float fabhs(double x)
{
  return ((x < 0) ? -x : x);
}

//矩阵求逆
Dcmf  matrix_inv(Dcmf a_matrix, int ndimen)

//	a_matrix:矩阵
//	ndimen :维数

{
	double tmp, tmp2, b_tmp[20], c_tmp[20];
	int k, k1, k2, k3, j, i, j2, i2, kme[20], kmf[20];
	i2 = j2 = 0;

	for (k = 0; k < ndimen; k++)
	{
		tmp2 = 0.0;
		for (i = k; i < ndimen; i++)
		{
			for (j = k; j < ndimen; j++)
			{
				if (fabs(a_matrix(i,j) ) <= fabhs(tmp2))
					continue;
				tmp2 = a_matrix(i,j);
				i2 = i;
				j2 = j;
			}
		}
		if (i2 != k)
		{
			for (j = 0; j < ndimen; j++)
			{
				tmp = a_matrix(i2,j);
				a_matrix(i2,j) = a_matrix(k,j);
				a_matrix(k,j) = tmp;
			}
		}
		if (j2 != k)
		{
			for (i = 0; i < ndimen; i++)
			{
				tmp = a_matrix(i,j2);
				a_matrix(i,j2) = a_matrix(i,k);
				a_matrix(i,k) = tmp;
			}
		}
		kme[k] = i2;
		kmf[k] = j2;
		for (j = 0; j < ndimen; j++)
		{
			if (j == k)
			{
				b_tmp[j] = 1.0 / tmp2;
				c_tmp[j] = 1.0;
			}
			else
			{
				b_tmp[j] = -double(a_matrix(k,j)) / tmp2;
				c_tmp[j] = a_matrix(j,k);
			}
			a_matrix(k,j) = 0.0;
			a_matrix(j,k) = 0.0;
		}
		for (i = 0; i < ndimen; i++)
		{
			for (j = 0; j < ndimen; j++)
			{
				a_matrix(i,j) = double(a_matrix(i,j)) + c_tmp[i] * b_tmp[j];
			}
		}
	}
	for (k3 = 0; k3 < ndimen;  k3++)
	{
		k  = ndimen - k3 - 1;
		k1 = kme[k];
		k2 = kmf[k];
		if (k1 != k)
		{
			for (i = 0; i < ndimen; i++)
			{
				tmp = a_matrix(i,k1);
				a_matrix(i,k1) = a_matrix(i,k);
				a_matrix(i,k) = tmp;
			}
		}
		if (k2 != k)
		{
			for(j = 0; j < ndimen; j++)
			{
				tmp = a_matrix(k2,j);
				a_matrix(k2,j) = a_matrix(k,j);
				a_matrix(k,j) = tmp;
			}
		}
	}
	return a_matrix;
}



