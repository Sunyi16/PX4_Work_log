#include <matrix/matrix/math.hpp>

__BEGIN_DECLS
using namespace matrix;

Vector3f Constrain_Vector3f(Vector3f amt, float low, float high);
Dcmf matrix_t(Dcmf b_matrix);
float dcm_abs(Dcmf a);
Vector3f vec_abs(Vector3f a, float d);
Vector3f vec_sign(Vector3f a);
Vector3f vee(Dcmf a);
Dcmf wedge(Vector3f a);
Dcmf matrix_a(Dcmf b_matrix, Dcmf c_matrix, int ktrl);
Dcmf dcm_dcm(Dcmf a, Dcmf b);
Dcmf dcm_dcm_t(Dcmf a, Dcmf b);
Vector3f dcm_vec(Dcmf a, Vector3f b);
Dcmf num_dcm(float a, Dcmf b);
Vector3f num_vec(float a, Vector3f b);
Dcmf dcm_add_vec(Dcmf a, Vector3f b);
Vector3f Vector3fAdd(Vector3f vecA, Vector3f vecB);
Vector3f Vector3fjian(Vector3f vecA, Vector3f vecB);
Vector3f Vector3fAdd_num(Vector3f vecA, float a);
Vector3f Vector3fjian_num(Vector3f vecA, float a);
Vector3f fsg(Vector3f a, float b);
Vector3f num_jian_vec(float a, Vector3f b);
Vector3f vec_vec(Vector3f a, Vector3f b);
Vector3f vec_fan(Vector3f a);
float fabhs(double x);
Dcmf  matrix_inv(Dcmf a_matrix, int ndimen);
Dcmf dcm_1(Dcmf a);
float trace(Dcmf a);


typedef struct {
	 Dcmf x1_pre ;
	 Dcmf z1_pre ;
	 Dcmf z2_pre ;
	 Dcmf z3_pre ;
	 Vector3f x2_pre;
	 Vector3f u_pre ;


}modd;


__END_DECLS






