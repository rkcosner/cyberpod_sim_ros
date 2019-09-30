#ifndef UAV_DYNAMICS_H
#define UAV_DYNAMICS_H

#include <Eigen/Dense>
#include <math.h>

using namespace Eigen;

const double uavModel[21] = {2.424,    //m
                             0.026644, //Ixx
                             0.029230, //Iyy
                             0.046516, //Izz
                             0.0,      //Iyz
                             0.0,      //Ixz
                             0.0,      //Ixy
                             0.0,      //cgz
                             0.6,      //FrameD
                             0.015,    //mProp
                             3.5e-05,  //Jprop (along prop axis)
                             3.5e-06,  //Iprop
                             0.02,     //cgzProp
                             9.80665,  //gravity constant
                             2.4e-5,   //kf
                             4.5e-7,   //kt
                             0.0136,   //K
                             0.1,      //R
                             14.8,     //Umax
                             0.7,      //Cfl
                             0.0       //Cla
                            };

static void blkdiag(const double varargin_1[9],
                    const double varargin_2[49],
                          double y[100])
{
	int i3;
	int i4;
	memset(&y[0], 0, 100U * sizeof(double));
	for (i3 = 0; i3 < 3; i3++) {
		y[10 * i3] = varargin_1[3 * i3];
		y[1 + 10 * i3] = varargin_1[1 + 3 * i3];
		y[2 + 10 * i3] = varargin_1[2 + 3 * i3];
	}

	for (i3 = 0; i3 < 7; i3++) {
		for (i4 = 0; i4 < 7; i4++) {
			y[(i4 + 10 * (3 + i3)) + 3] = varargin_2[i4 + 7 * i3];
		}
	}
}

static void quat2eulCustom(const double q[4],
                                 double eul[3])
{
	double eul_tmp;
	eul_tmp = 2.0 * (q[2] * q[2]);
	eul[0] = atan2(2.0 * q[0] * q[3] + 2.0 * q[1] * q[2], (1.0 - eul_tmp) - 2.0 * (q[3] * q[3]));
	eul[1] = asin(2.0 * q[0] * q[2] - 2.0 * q[3] * q[1]);
	eul[2] = atan2(2.0 * q[0] * q[1] + 2.0 * q[2] * q[3], (1.0 - 2.0 * (q[1] * q[1])) - eul_tmp);
}

static void quat2rotmCustom(const double q[4],
	                               double M[9])
{
	double M_tmp;
	double b_M_tmp;
	double c_M_tmp;
	double d_M_tmp;
	double e_M_tmp;
	double f_M_tmp;
	M_tmp = 2.0 * (q[3] * q[3]);
	b_M_tmp = 2.0 * (q[2] * q[2]);
	M[0] = (1.0 - b_M_tmp) - M_tmp;
	c_M_tmp = 2.0 * q[1] * q[2];
	d_M_tmp = 2.0 * q[3] * q[0];
	M[3] = c_M_tmp - d_M_tmp;
	e_M_tmp = 2.0 * q[1] * q[3];
	f_M_tmp = 2.0 * q[2] * q[0];
	M[6] = e_M_tmp + f_M_tmp;
	M[1] = c_M_tmp + d_M_tmp;
	c_M_tmp = 1.0 - 2.0 * (q[1] * q[1]);
	M[4] = c_M_tmp - M_tmp;
	M_tmp = 2.0 * q[2] * q[3];
	d_M_tmp = 2.0 * q[1] * q[0];
	M[7] = M_tmp - d_M_tmp;
	M[2] = e_M_tmp - f_M_tmp;
	M[5] = M_tmp + d_M_tmp;
	M[8] = c_M_tmp - b_M_tmp;
}

void uavDynamicsExpanded(const double X[17],
                               double A[100],
                               double bf[10],
                               double bg[40],
                               double fPos[3],
                               double fQuat[4])
{
	double dv0[16];
	double A_tmp;
	double b_A_tmp;
	double c_A_tmp;
	int i0;
	double b_X[4];
	double dv1[9];
	double rotWorld2Body[9];
	double vb[3];
	double eul[3];
	double d_A_tmp;
	double e_A_tmp;
	double f_A_tmp;
	double g_A_tmp;
	double h_A_tmp;
	double A_tmp_tmp;
	double b_A_tmp_tmp;
	double i_A_tmp;
	double j_A_tmp;
	double k_A_tmp;
	double l_A_tmp;
	double m_A_tmp;
	double bf_tmp;
	double b_bf_tmp;
	double c_bf_tmp;
	double dv2[10];
	double d_bf_tmp;
	double e_bf_tmp;
	double f_bf_tmp;
	double g_bf_tmp;
	double h_bf_tmp;
	double i_bf_tmp;
	double j_bf_tmp;
	double k_bf_tmp;
	double l_bf_tmp;
	double m_bf_tmp;
	double n_bf_tmp;
	double o_bf_tmp;
	double p_bf_tmp;
	static const double dv3[49] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
	   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	   1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	   0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

	double dv4[100];
	double b_A[100];
	double c_A[10];
	int i1;
	int n_A_tmp;
	int i2;


	fPos[0] = X[7];
	fPos[1] = X[8];
	fPos[2] = X[9];
	dv0[0] = -0.0;
	dv0[4] = -0.5 * X[10];
	dv0[8] = -0.5 * X[11];
	dv0[12] = -0.5 * X[12];
	A_tmp = -0.5 * -X[10];
	dv0[1] = A_tmp;
	dv0[5] = -0.0;
	b_A_tmp = -0.5 * -X[12];
	dv0[9] = b_A_tmp;
	dv0[13] = -0.5 * X[11];
	c_A_tmp = -0.5 * -X[11];
	dv0[2] = c_A_tmp;
	dv0[6] = -0.5 * X[12];
	dv0[10] = -0.0;
	dv0[14] = A_tmp;
	dv0[3] = b_A_tmp;
	dv0[7] = c_A_tmp;
	dv0[11] = -0.5 * X[10];
	dv0[15] = -0.0;
	for (i0 = 0; i0 < 4; i0++)
	{
		fQuat[i0] = ((dv0[i0] * X[3] + dv0[i0 + 4] * X[4]) + dv0[i0 + 8] * X[5]) +
		dv0[i0 + 12] * X[6];
	}


	b_X[0] = X[3];
	b_X[1] = X[4];
	b_X[2] = X[5];
	b_X[3] = X[6];
	quat2rotmCustom(b_X, dv1);
	for (i0 = 0; i0 < 3; i0++)
	{
		rotWorld2Body[3 * i0] = dv1[i0];
		rotWorld2Body[1 + 3 * i0] = dv1[i0 + 3];
		rotWorld2Body[2 + 3 * i0] = dv1[i0 + 6];
	}


	for (i0 = 0; i0 < 3; i0++)
	{
		vb[i0] = (rotWorld2Body[i0] * fPos[0] + rotWorld2Body[i0 + 3] * fPos[1]) +
		rotWorld2Body[i0 + 6] * fPos[2];
	}

	b_X[0] = X[3];
	b_X[1] = X[4];
	b_X[2] = X[5];
	b_X[3] = X[6];
	quat2eulCustom(b_X, eul);

	d_A_tmp = uavModel[0] + 4.0 * uavModel[9];
	A[0] = d_A_tmp;
	A[10] = 0.0;
	A[20] = 0.0;
	A[30] = 0.0;
	e_A_tmp = uavModel[7] * uavModel[0] + 4.0 * uavModel[12] * uavModel[9];
	A[40] = e_A_tmp;
	A[50] = 0.0;
	A[60] = 0.0;
	A[70] = 0.0;
	A[80] = 0.0;
	A[90] = 0.0;
	A[1] = 0.0;
	A[11] = d_A_tmp;
	A[21] = 0.0;
	A[31] = -uavModel[7] * uavModel[0] + -4.0 * uavModel[12] * uavModel[9];
	A[41] = 0.0;
	A[51] = 0.0;
	A[61] = 0.0;
	A[71] = 0.0;
	A[81] = 0.0;
	A[91] = 0.0;
	A[2] = 0.0;
	A[12] = 0.0;
	A[22] = -uavModel[0] + -4.0 * uavModel[9];
	A[32] = 0.0;
	A[42] = 0.0;
	A[52] = 0.0;
	A[62] = 0.0;
	A[72] = 0.0;
	A[82] = 0.0;
	A[92] = 0.0;
	f_A_tmp = 1.4142135623730951 * uavModel[8] * uavModel[0];
	g_A_tmp = 5.6568542494923806 * uavModel[8] * uavModel[9];
	A[3] = f_A_tmp + g_A_tmp;
	A[13] = -1.4142135623730951 * uavModel[8] * d_A_tmp;
	A[23] = 0.0;
	h_A_tmp = 1.4142135623730951 * uavModel[7] * uavModel[8] * uavModel[0];
	A_tmp = 5.6568542494923806 * uavModel[12] * uavModel[8] * uavModel[9];
	A[33] = (4.0 * uavModel[5] + h_A_tmp) + A_tmp;
	A[43] = (4.0 * uavModel[4] + h_A_tmp) + A_tmp;
	h_A_tmp = uavModel[8] * uavModel[8];
	A[53] = 4.0 * uavModel[3] + 4.0 * h_A_tmp * uavModel[9];
	A[63] = 0.0;
	A[73] = 0.0;
	A[83] = 0.0;
	A[93] = 0.0;
	A[4] = 0.0;
	A[14] = 0.0;
	A[24] = 0.0;
	A[34] = 0.0;
	A[44] = 0.0;
	A[54] = uavModel[10];
	A[64] = uavModel[10];
	A[74] = 0.0;
	A[84] = 0.0;
	A[94] = 0.0;
	A[5] = 0.0;
	A[15] = 0.0;
	A[25] = 0.0;
	A[35] = 0.0;
	A[45] = 0.0;
	A[55] = uavModel[10];
	A[65] = 0.0;
	A[75] = uavModel[10];
	A[85] = 0.0;
	A[95] = 0.0;
	A[6] = 0.0;
	A[16] = 0.0;
	A[26] = 0.0;
	A[36] = 0.0;
	A[46] = 0.0;
	A[56] = uavModel[10];
	A[66] = 0.0;
	A[76] = 0.0;
	A[86] = uavModel[10];
	A[96] = 0.0;
	A[7] = 0.0;
	A[17] = -4.0 * e_A_tmp;
	A_tmp_tmp = -1.4142135623730951 * uavModel[8] * uavModel[0];
	b_A_tmp_tmp = -5.6568542494923806 * uavModel[8] * uavModel[9];
	A_tmp = A_tmp_tmp + b_A_tmp_tmp;
	A[27] = A_tmp;
	b_A_tmp = uavModel[7] * uavModel[7];
	c_A_tmp = uavModel[12] * uavModel[12];
	i_A_tmp = 2.0 * b_A_tmp * uavModel[0];
	j_A_tmp = 8.0 * c_A_tmp * uavModel[9];
	k_A_tmp = 8.0 * uavModel[11] + 2.0 * uavModel[1];
	A[37] = 2.0 * (((k_A_tmp + i_A_tmp) + j_A_tmp) + h_A_tmp * uavModel[9]);
	A[47] = 4.0 * uavModel[6];
	A[57] = 4.0 * uavModel[5];
	A[67] = 0.0;
	A[77] = 0.0;
	A[87] = 0.0;
	A[97] = 0.0;
	l_A_tmp = -4.0 * uavModel[7] * uavModel[0];
	m_A_tmp = -16.0 * uavModel[12] * uavModel[9];
	A[8] = l_A_tmp + m_A_tmp;
	A[18] = 0.0;
	A[28] = A_tmp;
	A[38] = -4.0 * uavModel[6];
	A[48] = (((-16.0 * uavModel[11] + -4.0 * uavModel[2]) + -4.0 * b_A_tmp *
		uavModel[0]) + -16.0 * c_A_tmp * uavModel[9]) + -2.0 * h_A_tmp *
	uavModel[9];
	A[58] = -4.0 * uavModel[4];
	A[68] = 0.0;
	A[78] = 0.0;
	A[88] = 0.0;
	A[98] = 0.0;
	A[9] = 0.0;
	A[19] = 0.0;
	A[29] = 0.0;
	A[39] = 0.0;
	A[49] = 0.0;
	A[59] = uavModel[10];
	A[69] = 0.0;
	A[79] = 0.0;
	A[89] = 0.0;
	A[99] = uavModel[10];
	A_tmp = -4.0 * uavModel[16] * (1.0 / uavModel[17]) * uavModel[18];
	bg[3] = A_tmp;
	b_A_tmp = 4.0 * uavModel[16] * (1.0 / uavModel[17]) * uavModel[18];
	bg[13] = b_A_tmp;
	bg[23] = A_tmp;
	bg[33] = b_A_tmp;
	A_tmp = uavModel[16] * (1.0 / uavModel[17]) * uavModel[18];
	bg[4] = A_tmp;
	bg[14] = 0.0;
	bg[24] = 0.0;
	bg[34] = 0.0;
	bg[5] = 0.0;
	b_A_tmp = -uavModel[16] * (1.0 / uavModel[17]) * uavModel[18];
	bg[15] = b_A_tmp;
	bg[25] = 0.0;
	bg[35] = 0.0;
	bg[6] = 0.0;
	bg[16] = 0.0;
	bg[26] = A_tmp;
	bg[36] = 0.0;
	bg[0] = 0.0;
	bg[1] = 0.0;
	bg[2] = 0.0;
	bg[7] = 0.0;
	bg[8] = 0.0;
	bg[10] = 0.0;
	bg[11] = 0.0;
	bg[12] = 0.0;
	bg[17] = 0.0;
	bg[18] = 0.0;
	bg[20] = 0.0;
	bg[21] = 0.0;
	bg[22] = 0.0;
	bg[27] = 0.0;
	bg[28] = 0.0;
	bg[30] = 0.0;
	bg[31] = 0.0;
	bg[32] = 0.0;
	bg[37] = 0.0;
	bg[38] = 0.0;
	bg[9] = 0.0;
	bg[19] = 0.0;
	bg[29] = 0.0;
	bg[39] = b_A_tmp;


	A_tmp = -4.0 * X[6] * fQuat[3];
	b_A_tmp = -4.0 * X[5] * fQuat[2];
	dv1[0] = b_A_tmp + A_tmp;
	c_A_tmp = 2.0 * fQuat[1] * X[5] + 2.0 * X[4] * fQuat[2];
	dv1[3] = (c_A_tmp + 2.0 * fQuat[0] * X[6]) + 2.0 * X[3] * fQuat[3];
	bf_tmp = 2.0 * fQuat[1] * X[6];
	b_bf_tmp = 2.0 * X[4] * fQuat[3];
	dv1[6] = ((-2.0 * fQuat[0] * X[5] + -2.0 * X[3] * fQuat[2]) + bf_tmp) +
	b_bf_tmp;
	dv1[1] = (c_A_tmp + -2.0 * fQuat[0] * X[6]) + -2.0 * X[3] * fQuat[3];
	c_A_tmp = -4.0 * X[4] * fQuat[1];
	dv1[4] = c_A_tmp + A_tmp;
	A_tmp = 2.0 * fQuat[2] * X[6];
	c_bf_tmp = 2.0 * X[5] * fQuat[3];
	dv1[7] = ((2.0 * fQuat[0] * X[4] + 2.0 * X[3] * fQuat[1]) + A_tmp) + c_bf_tmp;
	dv1[2] = ((2.0 * fQuat[0] * X[5] + 2.0 * X[3] * fQuat[2]) + bf_tmp) + b_bf_tmp;
	dv1[5] = ((-2.0 * fQuat[0] * X[4] + -2.0 * X[3] * fQuat[1]) + A_tmp) +
	c_bf_tmp;
	dv1[8] = c_A_tmp + b_A_tmp;
	for (i0 = 0; i0 < 3; i0++) {
		dv2[i0] = (dv1[i0] * fPos[0] + dv1[i0 + 3] * fPos[1]) + dv1[i0 + 6] * fPos[2];
	}

	for (i0 = 0; i0 < 7; i0++) {
		dv2[i0 + 3] = 0.0;
	}

	bf[0] = (((-e_A_tmp * X[10] * X[12] + -uavModel[19] * vb[0]) + -uavModel[0] *
	X[11] * vb[2]) + -4.0 * uavModel[9] * X[11] * vb[2]) + d_A_tmp * (X
	[12] * vb[1] + uavModel[13] * sin(eul[1]));
	d_bf_tmp = d_A_tmp * X[10];
	bf[1] = ((((-(uavModel[7] * uavModel[0] + 4.0 * uavModel[12] * uavModel[9]) *
		X[11] * X[12] + -d_A_tmp * X[12] * vb[0]) + -uavModel[19] * vb[1])
	+ d_bf_tmp * vb[2]) + -uavModel[13] * uavModel[0] * cos(eul[1]) *
	sin(eul[2])) + -4.0 * uavModel[13] * uavModel[9] * cos(eul[1]) * sin
	(eul[2]);
	e_bf_tmp = X[10] * X[10];
	f_bf_tmp = X[11] * X[11];
	g_bf_tmp = fabs(X[13]);
	h_bf_tmp = fabs(X[14]);
	i_bf_tmp = fabs(X[15]);
	j_bf_tmp = fabs(X[16]);
	bf[2] = ((((((((-(uavModel[7] * uavModel[0] + 4.0 * uavModel[12] * uavModel[9])
		* e_bf_tmp + -(uavModel[7] * uavModel[0] + 4.0 * uavModel[12] *
			uavModel[9]) * f_bf_tmp) + -(uavModel[0] + 4.0 * uavModel[9]) * X[11] * vb[0])
	+ d_bf_tmp * vb[1]) + uavModel[19] * vb[2]) + -uavModel[14] * X
	[13] * g_bf_tmp) + uavModel[14] * X[14] * h_bf_tmp) + -uavModel[14]
	* X[15] * i_bf_tmp) + uavModel[14] * X[16] * j_bf_tmp) + uavModel[13]
	* d_A_tmp * cos(eul[1]) * cos(eul[2]);
	A_tmp = uavModel[16] * uavModel[16];
	d_bf_tmp = 4.0 * A_tmp;
	k_bf_tmp = 1.4142135623730951 * uavModel[8] * d_A_tmp;
	l_bf_tmp = 1.4142135623730951 * uavModel[8] * e_A_tmp;
	m_bf_tmp = k_bf_tmp * vb[2];
	b_A_tmp = 1.4142135623730951 * uavModel[8] * uavModel[13];
	n_bf_tmp = b_A_tmp * d_A_tmp;
	o_bf_tmp = 4.0 * uavModel[4] + l_bf_tmp;
	p_bf_tmp = 1.4142135623730951 * uavModel[19] * uavModel[8];
	l_bf_tmp = -(4.0 * uavModel[5] + l_bf_tmp);
	bf[3] = ((((((((((((-4.0 * uavModel[6] * e_bf_tmp + 4.0 * uavModel[6] *
		f_bf_tmp) + -4.0 * uavModel[20] * X[12]) + d_bf_tmp * X[13]
	* (1.0 / uavModel[17])) + d_bf_tmp * X[14] * (1.0 /
	uavModel[17])) + d_bf_tmp * X[15] * (1.0 / uavModel[17])) + d_bf_tmp * X[16]
	* (1.0 / uavModel[17])) + -1.4142135623730951 * uavModel[19] *
	uavModel[8] * vb[0]) + p_bf_tmp * vb[1]) + k_bf_tmp * X[12] *
	(vb[0] + vb[1])) + -X[11] * (l_bf_tmp * X[12] + m_bf_tmp)) + -X[10]
	* ((-4.0 * (uavModel[1] + -uavModel[2]) * X[11] + o_bf_tmp * X[12])
		+ m_bf_tmp)) + n_bf_tmp * sin(eul[1])) + n_bf_tmp * cos(eul[1]) *
	sin(eul[2]);
	bf[4] = -A_tmp * X[13] * (1.0 / uavModel[17]) + -uavModel[15] * X[13] *
	g_bf_tmp;
	bf[5] = -(uavModel[16] * uavModel[16]) * X[14] * (1.0 / uavModel[17]) +
	-uavModel[15] * X[14] * h_bf_tmp;
	bf[6] = -(uavModel[16] * uavModel[16]) * X[15] * (1.0 / uavModel[17]) +
	-uavModel[15] * X[15] * i_bf_tmp;
	d_bf_tmp = -1.4142135623730951 * uavModel[8] * e_A_tmp;
	m_bf_tmp = X[12] * X[12];
	n_bf_tmp = ((4.0 * uavModel[10] * X[13] + 4.0 * uavModel[10] * X[14]) + 4.0 *
		uavModel[10] * X[15]) + 4.0 * uavModel[10] * X[16];
	d_A_tmp = -h_A_tmp * uavModel[9];
	c_bf_tmp = A_tmp_tmp * vb[1];
	c_A_tmp = b_A_tmp_tmp * vb[1];
	p_bf_tmp *= vb[2];
	bf_tmp = -4.0 * uavModel[6] * X[12];
	h_bf_tmp *= 2.8284271247461903 * uavModel[8] * uavModel[14] * X[14];
	b_bf_tmp = -2.8284271247461903 * uavModel[8] * uavModel[14];
	A_tmp = b_A_tmp * uavModel[0];
	b_A_tmp = 5.6568542494923806 * uavModel[8] * uavModel[13] * uavModel[9];
	bf[7] = (((((((((((d_bf_tmp * e_bf_tmp + -o_bf_tmp * f_bf_tmp) + 4.0 *
		uavModel[4] * m_bf_tmp) + 4.0 * e_A_tmp * X[12] * vb[0]) +
	-X[11] * (((n_bf_tmp + -2.0 * ((((((8.0 * uavModel[11] + 2.0 *
		uavModel[2]) + -2.0 * uavModel[3]) + -8.0 * uavModel[10]) + i_A_tmp) +
	j_A_tmp) + d_A_tmp) * X[12]) + f_A_tmp * vb[0]) + g_A_tmp * vb[0])) +
	p_bf_tmp) + -X[10] * ((((((4.0 * uavModel[20] + 4.0 * uavModel
		[5] * X[11]) + bf_tmp) + c_bf_tmp) + c_A_tmp) + 4.0 * uavModel[7] *
	uavModel[0] * vb[2]) + 16.0 * uavModel[12] * uavModel[9] * vb[2])) +
	b_bf_tmp * X[13] * g_bf_tmp) + h_bf_tmp) + A_tmp * cos(eul[1]) *
	cos(eul[2])) + b_A_tmp * cos(eul[1]) * cos(eul[2])) + 4.0 *
	uavModel[7] * uavModel[13] * uavModel[0] * cos(eul[1]) * sin(eul[2]))
	+ 16.0 * uavModel[12] * uavModel[13] * uavModel[9] * cos(eul[1]) * sin(eul[2]);
	bf[8] = ((((((((((((l_bf_tmp * e_bf_tmp + d_bf_tmp * f_bf_tmp) + 4.0 *
		uavModel[5] * m_bf_tmp) + l_A_tmp * X[12] * vb[1]) +
	m_A_tmp * X[12] * vb[1]) + -X[10] * ((((n_bf_tmp + 4.0 *
		uavModel[4] * X[11]) + -2.0 * (((((k_A_tmp + -2.0 * uavModel[3]) + -8.0 *
			uavModel[10]) + i_A_tmp) + j_A_tmp) + d_A_tmp) * X[12]) + c_bf_tmp) +
	c_A_tmp)) + p_bf_tmp) + -X[11] * ((bf_tmp + k_bf_tmp * vb[0]) + -4.0 *
	(uavModel[20] + e_A_tmp * vb[2]))) + h_bf_tmp) + b_bf_tmp * X[15] * i_bf_tmp)
	+ A_tmp * cos(eul[1]) * cos(eul[2])) + b_A_tmp * cos(eul[1]) * cos
	(eul[2])) + -4.0 * uavModel[7] * uavModel[13] * uavModel[0] * sin
	(eul[1])) + -16.0 * uavModel[12] * uavModel[13] * uavModel[9] * sin
	(eul[1]);
	bf[9] = -(uavModel[16] * uavModel[16]) * X[16] * (1.0 / uavModel[17]) +
	-uavModel[15] * X[16] * j_bf_tmp;
	blkdiag(rotWorld2Body, dv3, dv4);
	for (i0 = 0; i0 < 10; i0++)
	{
		c_A[i0] = 0.0;
		for (i1 = 0; i1 < 10; i1++)
		{
			n_A_tmp = i0 + 10 * i1;
			c_A[i0] += A[n_A_tmp] * dv2[i1];
			A_tmp = 0.0;
			for (i2 = 0; i2 < 10; i2++)
			{
				A_tmp += A[i0 + 10 * i2] * dv4[i2 + 10 * i1];
			}
			b_A[n_A_tmp] = A_tmp;
		}
		bf[i0] -= c_A[i0];
	}

	memcpy(&A[0], &b_A[0], 100U * sizeof(double));
}

void uavDynamics(const double x[],
                 const double u[],
                       double xDot[],
                 const double t)
{
	Map<const Matrix<double,4,1>> uEig(u,4,1);
	Map<Matrix<double,17,1>> xDotEig(xDot,17,1);

   Matrix<double,17,1> f;
   Matrix<double,17,4> g;
   Matrix<double,10,10> A;
   Matrix<double,10,1> bf;
   Matrix<double,10,4> bg;
   Matrix<double,3,1> fPos;
   Matrix<double,4,1> fQuat;

   uavDynamicsExpanded(x,A.data(),bf.data(),bg.data(),fPos.data(),fQuat.data());

   bf += bg*uEig;
   bf = A.partialPivLu().solve(bf);

   xDotEig.block<3,1>(0,0) = fPos;
   xDotEig.block<4,1>(3,0) = fQuat;
   xDotEig.block<10,1>(7,0) = bf;
}

#endif
