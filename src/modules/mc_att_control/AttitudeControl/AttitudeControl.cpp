/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;



void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// compensate for the effect of the yaw weight rescaling the output
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}
}


/* matrix::Vector3f AttitudeControl::fhan(Dcmf v1, Dcmf x_d, Vector3f x2)
{
	Vector3f out_v1 = num_vec(0.5, vee(matrix_a(dcm_dcm(matrix_t(x_d),v1),dcm_dcm(matrix_t(v1),x_d),-1)));
	Vector3f fh;
	float d = r0*h0*h0;
	Vector3f a0 = num_vec(h0,x2);
	Vector3f y = Vector3fAdd(out_v1,a0);

	float a1 = sqrt(d*(d+8.0f*vec_abs(y)));
	Vector3f a2 = Vector3fAdd(a0,num_vec(a1-d,vec_sign(y))/2.0f);
	Vector3f a = Vector3fAdd(vec_vec(Vector3fAdd(a0,y),fsg(y,d)), vec_vec(a2,num_jian_vec(1,fsg(y,d))));
	fh = Vector3fAdd(vec_vec(num_vec(r0/d, a), fsg(a,d)), vec_vec(num_vec(r0,vec_sign(a)), num_jian_vec(1, fsg(a,d))));
	Vector3f fha = vec_fan(fh);
	return fha;

} */

matrix::Vector3f AttitudeControl::fhan(Dcmf v1, Dcmf x_d, Vector3f x2)
{
	Vector3f out_v1 = num_vec(0.5, vee(matrix_a(dcm_dcm(matrix_t(x_d),v1),dcm_dcm(matrix_t(v1),x_d),-1)));
	Vector3f fh;
	float d = r0*h0*h0;
	Vector3f a0 = num_vec(h0,x2);
	Vector3f y = Vector3fAdd(out_v1,a0);

	Vector3f a1 = vec_abs(y,d);
	Vector3f a2 = Vector3fAdd(a0,num_vec(0.5,vec_vec(num_jian_vec(-d,vec_fan(a1)),vec_sign(y))));
	Vector3f a = Vector3fAdd(vec_vec(Vector3fAdd(a0,y),fsg(y,d)), vec_vec(a2,num_jian_vec(1,fsg(y,d))));
	fh = Vector3fAdd(vec_vec(num_vec(r0/d, a), fsg(a,d)), vec_vec(num_vec(r0,vec_sign(a)), num_jian_vec(1, fsg(a,d))));
	Vector3f fha = vec_fan(fh);
	return fha;

}//自定义

matrix::Vector3f AttitudeControl::update(const Quatf &q, modd *modd_param)
{


	 //Dcmf x1_pre = modd_param.x1_pre;
	 //Dcmf z1_pre = modd_param.z1_pre;
	 //Dcmf z2_pre = modd_param.z2_pre;
	 //Dcmf z3_pre = modd_param.z3_pre;
	 //Vector3f x2_pre = modd_param.x2_pre;
	 //Vector3f u_pre = modd_param.u_pre;

	Quatf qd = _attitude_setpoint_q;


	Dcmf x_d(qd);
	Dcmf x(q);

	//PX4_WARN("DATA:%f%f%f", x(0,0), x(1,1), x(2,2));

	/* x_d = dcm_1(x_d);
	x = dcm_1(x); */
	//Vector3f x_dd = num_vec(0.5, vee(matrix_a(dcm_dcm(matrix_t(x_d),x),dcm_dcm(matrix_t(x),x_d),-1)));
	//adrc.test[0]= x_dd(0);
	//adrc.test[1]= x_dd(1);
	//adrc.test[2]= x_dd(2);

/*****************************************ADRC第一步：TD*********************************************/
	Dcmf x1 = dcm_1(matrix_a(modd_param->x1_pre, dcm_dcm(num_dcm(h,modd_param->x1_pre),wedge(modd_param->x2_pre)),1));
	//Dcmf x1 = matrix_a(modd_param->x1_pre,num_dcm(h, dcm_dcm(modd_param->x1_pre,wedge(modd_param->x2_pre))),1);
 	//Dcmf x1 = Dcmf(dcm_dcm(modd_param->x1_pre,dcm_dcm(modd_param->x1_pre,wedge(modd_param->x2_pre))));
 	//Dcmf x1 = dcm_1(matrix_a(modd_param->x1_pre,wedge(modd_param->x2_pre),1));
	Vector3f x2 = Vector3fAdd(modd_param->x2_pre,num_vec(h, fhan(modd_param->x1_pre,x_d,modd_param->x2_pre)));

	//PX4_WARN("DATA:%f%f%f", x1(0,0), x1(1,1), x1(2,2));
	//PX4_WARN("DATA:%f%f%f", x2(0), x2(1), x2(2));



/******************************************第二步：ESO*****************************************************/
	Vector3f e =num_vec(1/(2*sqrt(1+trace(dcm_dcm(matrix_t(x),modd_param->z1_pre)))), vee(matrix_a( dcm_dcm(matrix_t(x),modd_param->z1_pre),dcm_dcm(matrix_t(modd_param->z1_pre),x),-1)));

	//PX4_WARN("DATA:%f%f%f", e(0), e(1), e(2));


/* 	Dcmf z1 = matrix_a(modd_param->z1_pre,num_dcm(h, dcm_dcm(modd_param->z1_pre,
	 wedge(Vector3fjian(vee(modd_param->z2_pre), num_vec(l1/num_min, e))))), 1); */

	Dcmf z1 = matrix_a(modd_param->z1_pre,num_dcm(h, dcm_dcm(modd_param->z1_pre,
	 wedge(Vector3fjian(vee(modd_param->z2_pre), num_vec(300.0f, e))))), 1);

	//Dcmf z1 = dcm_1(matrix_a(modd_param->z1_pre, num_dcm(h, matrix_a(modd_param->z2_pre ,num_dcm(l1/num_min, matrix_a(modd_param->z1_pre,x,-1)),-1)),1));

	PX4_WARN("DATA:%f%f%f", z1(0,0), z1(1,1), z1(2,2));


	//Dcmf z1 = dcm_dcm(modd_param->z1_pre,  matrix_a(modd_param->z2_pre ,num_dcm(l1/num_min, matrix_a(modd_param->z1_pre,x1,-1)),-1));
	J(0,0) = 0.1;
	J(1,1) = 0.1;
	J(2,2) = 0.3;
	J(0,1) = 0;
	J(0,2) = 0;
	J(1,0) = 0;
	J(1,2) = 0;
	J(2,1) = 0;
	J(2,0) = 0;
	//Dcmf z_add2 =matrix_a(matrix_a(matrix_a(dcm_dcm(dcm_dcm(modd_param->z2_pre,matrix_t(x)),modd_param->z2_pre),modd_param->z3_pre,-1),num_dcm(l2/(num_min*num_min),matrix_a(modd_param->z1_pre,x,-1)),-1),dcm_dcm(dcm_dcm(x,matrix_t(J)),wedge(modd_param->u_pre)),1);

/* 	Dcmf z_add2 =matrix_a(matrix_a(dcm_dcm(dcm_dcm(x,matrix_inv(J,3)),wedge(modd_param->u_pre)),
	num_dcm(l2/(num_min*num_min),matrix_a(modd_param->z1_pre,x,-1)),-1),modd_param->z3_pre,-1);

	Dcmf z2 =matrix_a(modd_param->z2_pre,num_dcm(h, z_add2), 1); */

	//Vector3f z_add2 =Vector3fAdd(Vector3fAdd(vee(modd_param->z3_pre), num_vec(l2/(num_min*num_min), e)), dcm_vec(matrix_inv(J,3), modd_param->u_pre));
	Vector3f z_add2 =Vector3fAdd(Vector3fAdd(vee(modd_param->z3_pre), num_vec(6000.f, e)), dcm_vec(matrix_inv(J,3), modd_param->u_pre));
	Vector3f z2_true = Vector3fAdd(vee(modd_param->z2_pre), num_vec(h,z_add2));

	//PX4_WARN("DATA:%f%f%f", z2_true(0), z2_true(1), z2_true(2));
	Dcmf z2 = wedge(z2_true);


	//Dcmf z2 =dcm_1(dcm_dcm(modd_param->z2_pre, z_add2));
	//Dcmf z3 = matrix_a(modd_param->z3_pre,num_dcm(h, num_dcm(l3/(num_min*num_min*num_min),matrix_a(modd_param->z1_pre,x,-1))),-1);
	//Dcmf z3 = dcm_1(dcm_dcm(modd_param->z3_pre,num_dcm(-l3/(num_min*num_min*num_min),matrix_a(modd_param->z1_pre,x,-1))));

	Vector3f z3_true = Vector3fjian(vee(modd_param->z3_pre), num_vec(100000.f, e));
	Dcmf z3 = wedge(z3_true);
	//PX4_WARN("DATA:%f%f%f", z3(0,0), z3(1,1), z3(2,2));
	//PX4_WARN("DATA:%f%f%f", z3_true(0), z3_true(1), z3_true(2));




	//Vector3f z2_true = vee(dcm_dcm(matrix_t(x),z2));
	//Vector3f z3_true = vee(dcm_dcm(dcm_dcm(J,matrix_t(x)),z3));

/**********************************************第三步：DLSEF，非线性组合******************************************************/


	Vector3f e1 =num_vec(-1/2, vee(matrix_a( dcm_dcm(matrix_t(x1),z1),dcm_dcm(matrix_t(z1),x1),-1)));
	Vector3f e2 = Vector3fjian(x2, z2_true);
	//Vector3f u0 = Vector3fAdd( num_vec(-k1,e1),num_vec(-k2,e2));
	Vector3f u0 = Vector3fAdd( num_vec(k1,e1),num_vec(k2,e2));

	//PX4_WARN("DATA:%f%f%f", e2(0), e2(1), e2(2));


/*************************************************第四步：扰动补偿***************************************************************************/
	Vector3f u = Vector3fjian(u0,dcm_vec(J,z3_true));//添加了扰动补偿，陀螺仪误差较大时，会产生漂移
	u =Constrain_Vector3f(u0,-450,450);

	//PX4_WARN("DATA:%f%f%f", u(0), u(1), u(2));


/***********************把控制量发到速度控制统一处理******************************************/

	adrc.test[0] = matrix_a(modd_param->z1_pre,x,-1)(0,1);
	adrc.test[1] = num_dcm(2000.0f, matrix_a(modd_param->z1_pre,x,-1))(0,1);
	adrc.test[2] = matrix_a(modd_param->z2_pre ,num_dcm(l1/num_min, matrix_a(modd_param->z1_pre,x,-1)),-1)(0,1);

	adrc.timestamp = hrt_absolute_time();
	adrc.adrc_u[0] = u(0);
	adrc.adrc_u[1] = u(1);
	adrc.adrc_u[2] = u(2);

	adrc.v2[0] = x2(0);
	adrc.v2[1] = x2(1);
	adrc.v2[2] = x2(2);

	adrc.z2[0] = z2_true(0);
	adrc.z2[1] = z2_true(1);
	adrc.z2[2] = z2_true(2);

	adrc.z3[0] = z3_true(0);
	adrc.z3[1] = z3_true(1);
	adrc.z3[2] = z3_true(2);

	adrc.v1[0] = x1(0,0);
	adrc.v1[1] = x1(0,1);
	adrc.v1[2] = x1(0,2);
	adrc.v1[3] = x1(1,0);
	adrc.v1[4] = x1(1,1);
	adrc.v1[5] = x1(1,2);
	adrc.v1[6] = x1(2,0);
	adrc.v1[7] = x1(2,1);
	adrc.v1[8] = x1(2,2);

	adrc.z1[0] = z1(0,0);
	adrc.z1[1] = z1(0,1);
	adrc.z1[2] = z1(0,2);
	adrc.z1[3] = z1(1,0);
	adrc.z1[4] = z1(1,1);
	adrc.z1[5] = z1(1,2);
	adrc.z1[6] = z1(2,0);
	adrc.z1[7] = z1(2,1);
	adrc.z1[8] = z1(2,2);

	adrc.z_2[0] = z2(0,0);
	adrc.z_2[1] = z2(0,1);
	adrc.z_2[2] = z2(0,2);
	adrc.z_2[3] = z2(1,0);
	adrc.z_2[4] = z2(1,1);
	adrc.z_2[5] = z2(1,2);
	adrc.z_2[6] = z2(2,0);
	adrc.z_2[7] = z2(2,1);
	adrc.z_2[8] = z2(2,2);

	adrc.z_3[0] = z3(0,0);
	adrc.z_3[1] = z3(0,1);
	adrc.z_3[2] = z3(0,2);
	adrc.z_3[3] = z3(1,0);
	adrc.z_3[4] = z3(1,1);
	adrc.z_3[5] = z3(1,2);
	adrc.z_3[6] = z3(2,0);
	adrc.z_3[7] = z3(2,1);
	adrc.z_3[8] = z3(2,2);

	adrc.q[0] = x(0,0);
	adrc.q[1] = x(0,1);
	adrc.q[2] = x(0,2);
	adrc.q[3] = x(1,0);
	adrc.q[4] = x(1,1);
	adrc.q[5] = x(1,2);
	adrc.q[6] = x(2,0);
	adrc.q[7] = x(2,1);
	adrc.q[8] = x(2,2);

	adrc.xd[0] = x_d(0,0);
	adrc.xd[1] = x_d(0,1);
	adrc.xd[2] = x_d(0,2);
	adrc.xd[3] = x_d(1,0);
	adrc.xd[4] = x_d(1,1);
	adrc.xd[5] = x_d(1,2);
	adrc.xd[6] = x_d(2,0);
	adrc.xd[7] = x_d(2,1);
	adrc.xd[8] = x_d(2,2);

	adrc.error_eso[0] = e(0);
	adrc.error_eso[1] = e(1);
	adrc.error_eso[2] = e(2);

	adrc_u_pub.publish(adrc);



/* ********************************************************ADRC***********************************************************/

/* 	Vector3f att_sp;
	Vector3f att;
	att_sp(0) =  Eulerf(qd).theta();
	att_sp(1) =  Eulerf(qd).phi();
	att_sp(2) =  Eulerf(qd).psi();

	att(0) =  Eulerf(q).theta();
	att(1) =  Eulerf(q).phi();
	att(2) =  Eulerf(q).psi();

	static Fhan_Data ADRC_Pitch={0},ADRC_Roll={0},ADRC_Yaw{0};
	ADRC_Init(&ADRC_Pitch,&ADRC_Roll,&ADRC_Yaw);
	Vector3f torque;
	torque(0)= ADRC_Control(&ADRC_Pitch , att_sp(0) , att(0));
	torque(1)= ADRC_Control(&ADRC_Roll , att_sp(1) , att(1));
	torque(2)= ADRC_Control(&ADRC_Yaw , att_sp(2) , att(2));
	adrc.adrc_u[0]= torque(0);
	adrc.adrc_u[1]= torque(1);
	adrc.adrc_u[2]= torque(2);

	adrc_u_pub.publish(adrc);  */

/*****************************************以下为源码控制，为了后续完整性，保留这一部分，但是最后发给控制分配的是ADRC控制输出******************************************************************/

	// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		// full attitude control anyways generates no yaw input and directly takes the combination of
		// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
		qd_red = qd;

	} else {
		// transform rotation from current to desired thrust vector into a world frame reduced desired attitude
		qd_red *= q;
	}

	// mix full and reduced desired attitude
	Quatf q_mix = qd_red.inversed() * qd;
	q_mix.canonicalize();
	// catch numerical problems with the domain of acosf and asinf
	q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
	q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(q_mix(0))), 0, 0, sinf(_yaw_w * asinf(q_mix(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = q.inversed() * qd;

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	const Vector3f eq = 2.f * qe.canonical().imag();

	// calculate angular rates setpoint
	matrix::Vector3f rate_setpoint = eq.emult(_proportional_gain);

	// Feed forward the yaw setpoint rate.
	// yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
	// but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	// Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	// and multiply it by the yaw setpoint rate (yawspeed_setpoint).
	// This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
	// such that it can be added to the rates setpoint.
	if (is_finite(_yawspeed_setpoint)) {
		rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	// limit rates
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
